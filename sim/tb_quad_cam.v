//=====================================================================
// tb_quad_cam.v   (v23 新增)
//---------------------------------------------------------------------
// 验 quad_wr_ctrl 的三态仲裁:
//
//   full_scr=1, cam_full=0  -> 只搬 PC   (PC 全屏)
//   full_scr=0, cam_full=0  -> 两路都搬, 摄像头优先 (四宫格)
//   full_scr=0, cam_full=1  -> 只搬摄像头 (摄像头全屏)
//
// 关键点是第三条: 摄像头全屏时如果 PC 还能插进来, 两边会抢同一批块号,
// 屏幕上就是两种画面互相覆盖。tb 用"像素值的高 2 位当来源标记"
// (cam=11, pc=10) 来抓这种串台。
//=====================================================================
`timescale 1ns/1ps
module tb_quad_cam;

reg         clk = 0;
always #10 clk = ~clk;              // 50MHz

reg         rst_n    = 0;
reg         full_scr = 0;
reg         cam_full = 0;
reg         cam_vs   = 0;

//---- 摄像头 / PC 字流 (预填, rd 弹出) ----
reg  [16:0] cam_mem [0:8191];
reg  [16:0] pc_mem  [0:8191];
integer     cam_wp, cam_rp, pc_wp, pc_rp;
wire        cam_rd, pc_rd;
wire        cam_empty, pc_empty;
wire [16:0] cam_word = cam_mem[cam_rp];
wire [16:0] pc_word  = pc_mem [pc_rp];
wire [12:0] cam_lvl  = cam_wp - cam_rp;
wire [13:0] pc_lvl   = pc_wp  - pc_rp;
assign cam_empty = (cam_rp >= cam_wp);
assign pc_empty  = (pc_rp  >= pc_wp);

always @(posedge clk) begin
    if (cam_rd) cam_rp <= cam_rp + 1;
    if (pc_rd)  pc_rp  <= pc_rp  + 1;
end

wire [27:0] ad_data;
wire        ad_wr;
wire [15:0] wd_data;
wire        wd_en;
wire        wr_page, wr_done_page;
wire        frame_live;
wire [31:0] cam_px, pc_px;
wire [15:0] chunk_cnt;
wire        busy;

quad_wr_ctrl dut(
    .clk          (clk         ),
    .rst_n        (rst_n       ),
    .init_done    (1'b1        ),
    .cam_word     (cam_word    ),
    .cam_empty    (cam_empty   ),
    .cam_rd       (cam_rd      ),
    .cam_level    (cam_lvl     ),
    .pc_word      (pc_word     ),
    .pc_empty     (pc_empty    ),
    .pc_rd        (pc_rd       ),
    .pc_level     (pc_lvl      ),
    .cam_vs       (cam_vs      ),
    .wr_page      (wr_page     ),
    .wr_done_page (wr_done_page),
    .mode_sel     (3'd0        ),   // 彩色直通, 方便比对像素值
    .full_scr     (full_scr    ),
    .cam_full     (cam_full    ),
    .lane_bus     (128'd0   ),   // v31: 本台不测车道线, 给常量
    .lane_vld     (1'b0    ),
    .lane_en      (1'b1    ),   // 测试里图层常开
    .lane_osd_en  (1'b1    ),   // v51: 显示开关, 测试里常开
        .force_go  (1'b0 ), .force_l  (1'b0 ), .force_r  (1'b0 ),
        .alarm      (     ),
    .ad_data      (ad_data     ),
    .ad_wr        (ad_wr       ),
    .ad_level     (5'd0        ),
    .wd_data      (wd_data     ),
    .wd_en        (wd_en       ),
    .cam_px       (cam_px      ),
    .pc_px        (pc_px       ),
    .chunk_cnt    (chunk_cnt   ),
    .frame_live   (frame_live  ),
    .busy         (busy        )
);

//---- 统计输出里每个像素的来源 ----
integer n_cam = 0, n_pc = 0, n_bad = 0;
always @(negedge clk) begin
    if (wd_en && rst_n) begin
        if (wd_data[15:14] === 2'b11)      n_cam = n_cam + 1;
        else if (wd_data[15:14] === 2'b10) n_pc  = n_pc  + 1;
        else begin
            n_bad = n_bad + 1;
            if (n_bad < 6) $display("  BAD pixel %h", wd_data);
        end
    end
end

task mem_put(input integer which, input integer idx, input [16:0] v);
    begin
        if (which == 0) cam_mem[idx] = v;
        else            pc_mem[idx]  = v;
    end
endtask

// 填一块: 标签(tag) + 512 个像素(高 2 位标记来源)
task fill(input integer which, input [10:0] tag);
    integer j, base;
    begin
        if (which == 0) base = cam_wp; else base = pc_wp;
        for (j = 0; j < 513; j = j + 1) begin
            if (j == 0)
                mem_put(which, base + j, {1'b1, 5'd0, tag});
            else
                mem_put(which, base + j, {which ? 2'b10 : 2'b11, 6'd0, j[7:0]});
        end
        if (which == 0) cam_wp = base + 513; else pc_wp = base + 513;
    end
endtask

task run_case(input integer f, input integer c);
    begin
        full_scr = f[0];
        cam_full = c[0];
        n_cam = 0; n_pc = 0; n_bad = 0;
        repeat (4000) @(negedge clk);
        @(negedge clk) cam_vs = 1;      // 给一个翻页节拍
        @(negedge clk) cam_vs = 0;
        repeat (2000) @(negedge clk);
        $display("  full_scr=%0d cam_full=%0d -> cam=%0d pc=%0d bad=%0d",
                 f[0], c[0], n_cam, n_pc, n_bad);
    end
endtask

initial begin
    cam_wp = 0; cam_rp = 0; pc_wp = 0; pc_rp = 0;
    $display("===== tb_quad_cam (v23) =====");
    repeat (10) @(negedge clk);
    rst_n = 1;

    //---- 1) 摄像头全屏: 只该搬摄像头 ----
    fill(0, 11'd100); fill(0, 11'd101); fill(0, 11'd102);
    fill(1, 11'd200); fill(1, 11'd201); fill(1, 11'd202);
    run_case(0, 1);
    if (n_pc == 0 && n_cam > 1000 && n_bad == 0)
        $display("  PASS cam-full: no PC pixel leaked in");
    else
        $display("  FAIL cam-full: cam=%0d pc=%0d bad=%0d", n_cam, n_pc, n_bad);

    //---- 2) 四宫格: 两路都搬 ----
    cam_wp = 0; cam_rp = 0; pc_wp = 0; pc_rp = 0;
    fill(0, 11'd103); fill(1, 11'd203);
    run_case(0, 0);
    if (n_pc > 0 && n_cam > 0 && n_bad == 0)
        $display("  PASS quad: both sources present");
    else
        $display("  FAIL quad: cam=%0d pc=%0d bad=%0d", n_cam, n_pc, n_bad);

    //---- 3) PC 全屏: 只该搬 PC ----
    cam_wp = 0; cam_rp = 0; pc_wp = 0; pc_rp = 0;
    fill(0, 11'd104); fill(1, 11'd204);
    run_case(1, 0);
    if (n_cam == 0 && n_pc > 500 && n_bad == 0)
        $display("  PASS pc-full: camera blocked");
    else
        $display("  FAIL pc-full: cam=%0d pc=%0d bad=%0d", n_cam, n_pc, n_bad);

    if (frame_live !== 1'b1) $display("  FAIL frame_live not set");
    else                     $display("  PASS frame_live set");

    $display("===== END =====");
    $finish;
end

endmodule
