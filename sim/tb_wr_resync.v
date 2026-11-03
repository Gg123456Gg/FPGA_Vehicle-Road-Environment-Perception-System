//=====================================================================
// tb_wr_resync.v  —— 验证 v25 的"标签位自愈"
//
// 场景: 上游字流错位(多了一个字/少了一个字)。修复前写侧会把像素当成
// 块号标签, 于是拿到垃圾块号, 而且错位永久保持 —— 屏幕上"定格"或"长条"。
// 修复后: 块首不是标签就丢掉那一个字, 下一个真标签一到立刻对齐。
//
// 用例:
//   A) 干净字流            -> 3 个块, 块号 0/1/2, 每块 512 像素
//   B) 第 1 块后面多插一个字 -> 仍然 3 个块, 块号 0/1/2 (多出来的字被丢掉)
//   C) 一个字都不给          -> 不能死锁, 状态机必须停在 IDLE 等
//=====================================================================
`timescale 1ns/1ps
module tb_wr_resync;

reg clk = 0;
always #10 clk = ~clk;              // 50MHz

reg  rst_n = 0;
reg  init_done = 1;
reg  full_scr  = 0;
reg  cam_full  = 1;
reg  cam_vs    = 0;

wire        cam_rd;
wire        pc_rd;
wire [27:0] ad_data;
wire        ad_wr;
wire [15:0] wd_data;
wire        wd_en;
wire [31:0] cam_px, pc_px;
wire [15:0] chunk_cnt;
wire        frame_live, busy;
reg  [2:0]  wr_page_v;
wire        wr_page, wr_done_page;

//---- 摄像头字流 FIFO 模型 ----
reg  [16:0] q [0:4095];
integer wrp = 0, rdp = 0;

wire [16:0] cam_word = q[rdp];
wire        cam_empty = (rdp == wrp);
wire [12:0] cam_level = wrp - rdp;

quad_wr_ctrl dut(
    .clk          (clk        ),
    .rst_n        (rst_n      ),
    .init_done    (init_done  ),
    .cam_word     (cam_word   ),
    .cam_empty    (cam_empty  ),
    .cam_rd       (cam_rd     ),
    .cam_level    (cam_level  ),
    .pc_word      (17'd0      ),
    .pc_empty     (1'b1       ),
    .pc_rd        (pc_rd      ),
    .pc_level     (14'd0      ),
    .cam_vs       (cam_vs     ),
    .wr_page      (wr_page    ),
    .wr_done_page (wr_done_page),
    .mode_sel     (3'd0       ),
    .full_scr     (full_scr   ),
    .cam_full     (cam_full   ),
    .lane_bus     (128'd0   ),   // v31: 本台不测车道线, 给常量
    .lane_vld     (1'b0    ),
    .lane_en      (1'b1    ),   // 测试里图层常开
    .lane_osd_en  (1'b1    ),   // v51: 显示开关, 测试里常开
        .force_go  (1'b0 ), .force_l  (1'b0 ), .force_r  (1'b0 ),
        .alarm      (     ),
    .ad_data      (ad_data    ),
    .ad_wr        (ad_wr      ),
    .ad_level     (5'd0       ),
    .wd_data      (wd_data    ),
    .wd_en        (wd_en      ),
    .cam_px       (cam_px     ),
    .pc_px        (pc_px      ),
    .chunk_cnt    (chunk_cnt  ),
    .frame_live   (frame_live ),
    .busy         (busy       )
);

always @(posedge clk) if (cam_rd) rdp <= rdp + 1;

//---- 记录写出去的块号 ----
integer nblk = 0;
integer blkno [0:15];
integer px_per_blk [0:15];
integer cur_px = 0;

always @(posedge clk) begin
    if (ad_wr) begin
        if (nblk > 0) px_per_blk[nblk-1] = cur_px;
        cur_px = 0;
        blkno[nblk]  = ad_data[16:6];
        px_per_blk[nblk] = 0;
        nblk = nblk + 1;
    end
    if (wd_en) cur_px = cur_px + 1;
end

//---- 往流里塞字 ----
task put_tag(input [10:0] b);
    begin
        q[wrp] = {1'b1, 5'd0, b};
        wrp = wrp + 1;
    end
endtask

task put_block(input [10:0] b);
    integer i;
    begin
        put_tag(b);
        for (i = 0; i < 512; i = i + 1) begin
            q[wrp] = {1'b0, 16'h1000 + i[15:0]};
            wrp = wrp + 1;
        end
    end
endtask

task put_px(input [15:0] v);
    begin
        q[wrp] = {1'b0, v};
        wrp = wrp + 1;
    end
endtask

task show(input [255:0] tag);
    integer i;
    begin
        $write("  %0s : %0d 块 ->", tag, nblk);
        for (i = 0; i < nblk; i = i + 1)
            $write(" [%0d/%0dpx]", blkno[i], px_per_blk[i]);
        $write("\n");
    end
endtask

task reset_all;
    begin
        rst_n = 0; wrp = 0; rdp = 0; nblk = 0; cur_px = 0;
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);
    end
endtask

integer errs = 0;

task expect_blocks(input integer n);
    integer i;
    begin
        if (nblk != n) begin
            errs = errs + 1;
            $display("  FAIL: 期望 %0d 块, 实际 %0d 块", n, nblk);
        end else begin
            for (i = 0; i < n; i = i + 1) begin
                if (blkno[i] !== i) begin
                    errs = errs + 1;
                    $display("  FAIL: 第 %0d 块块号 = %0d, 期望 %0d", i, blkno[i], i);
                end
                if (px_per_blk[i] != 512 && px_per_blk[i] != 0) begin
                    // 最后一块在 run 结束时还没结算, 允许 0
                    errs = errs + 1;
                    $display("  FAIL: 第 %0d 块像素数 = %0d, 期望 512", i, px_per_blk[i]);
                end
            end
        end
    end
endtask

initial begin
    $display("==== v25 标签位自愈 验证 ====");

    //---------- A) 干净字流 ----------
    reset_all;
    put_block(0); put_block(1); put_block(2);
    repeat (513*3 + 200) @(posedge clk);
    show("A 干净");
    expect_blocks(3);
    if (errs == 0) $display("PASS A: 干净字流块号正常");

    //---------- B) 故意多插一个字 ----------
    reset_all;
    put_block(0);
    put_px(16'hDEAD);          // ★错位来源: 多出来的一个像素★
    put_block(1); put_block(2);
    repeat (513*4 + 400) @(posedge clk);
    show("B 多一字");
    expect_blocks(3);
    if (errs == 0) $display("PASS B: 多一个字后自动对齐, 3 块块号都对");

    //---------- C) 空流不能死锁 ----------
    reset_all;
    repeat (2000) @(posedge clk);
    if (busy !== 1'b0) begin
        $display("  FAIL C: 空流时 busy 不该拉高 (状态机没回 IDLE)");
        errs = errs + 1;
    end
    show("C 空流");
    if (errs == 0) $display("PASS C: 空流不死锁");

    $display("==== 失败项 %0d ====", errs);
    $finish;
end

endmodule
