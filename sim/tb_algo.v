//=====================================================================
// tb_algo.v   算法链验证台
//---------------------------------------------------------------------
// 只测 quad_wr_ctrl 的写通路算法部分, 不测 DDR/读侧。
//
// 激励: 每块 513 字 (1 个标签 + 512 个像素), 像素值 = 全局序号低 16 位,
//       所以 NBLK 块正好把 RGB565 全值域 65536 个编码走一遍。
// 采集: wd_en 有效时的 wd_data, 顺序写进文件, 交给 check_algo.py 比对。
//
// 用法: vsim -c -do "run -all; quit -f" tb_algo +MODE=1 +NBLK=128 +OUT=o.txt
//=====================================================================
`timescale 1ns/1ps
module tb_algo;

reg clk = 1'b0;
always #10 clk = ~clk;          // 50MHz

reg rst_n = 1'b0;

//--------------------------------------------------------------------
// 摄像头 FIFO 模型 (深度和真实的一样是 8192, 这里只放一块, 够用)
//--------------------------------------------------------------------
reg  [16:0] fifo [0:1023];
integer     rd_ptr = 0;
integer     wr_ptr = 0;

// 注意用 integer 直接索引: 若写成 rd_ptr[9:0], 第 512 个像素时 512&0x3FF=0
// 会绕回标签字, 最后那个像素就变成了 0x1xxxx, 是测试台自己的坑。
wire [16:0] cam_word  = fifo[rd_ptr];
wire        cam_empty = (rd_ptr == wr_ptr);
wire [12:0] cam_level = wr_ptr[12:0] - rd_ptr[12:0];
wire        cam_rd;

// PC 侧全程关掉, 只测摄像头那一路
wire [16:0] pc_word  = 17'd0;
wire        pc_empty = 1'b1;
wire [13:0] pc_level = 14'd0;
wire        pc_rd;

//--------------------------------------------------------------------
reg  [2:0]  mode_in   = 3'd0;
integer     mode_int  = 0;
reg  [2:0]  mode_sel;

reg         cam_vs   = 1'b0;
reg  [4:0]  ad_level = 5'd0;

wire [27:0] ad_data;
wire        ad_wr;
wire [15:0] wd_data;
wire        wd_en;
wire        wr_page, wr_done_page;
wire [31:0] cam_px, pc_px;
wire [15:0] chunk_cnt;
wire        busy;

quad_wr_ctrl u_dut(
    .clk          (clk          ),
    .rst_n        (rst_n        ),
    .init_done    (1'b1         ),
    .cam_word     (cam_word     ),
    .cam_empty    (cam_empty    ),
    .cam_rd       (cam_rd       ),
    .cam_level    (cam_level    ),
    .pc_word      (pc_word      ),
    .pc_empty     (pc_empty     ),
    .pc_rd        (pc_rd        ),
    .pc_level     (pc_level     ),
    .cam_vs       (cam_vs       ),
    .wr_page      (wr_page      ),
    .wr_done_page (wr_done_page ),
    .mode_sel     (mode_sel     ),
    .full_scr     (1'b0         ),   // v15: 四宫格模式
    .cam_full     (1'b0         ),   // v23: 三态里的摄像头全屏, 这些台子只测四宫格/PC 全屏
    .lane_osd_en  (1'b1    ),   // v51: 显示开关, 测试里常开
    .force_go     (1'b0         ),
    .force_l      (1'b0         ),
    .force_r      (1'b0         ),
    .alarm        (             ),
    .ad_data      (ad_data      ),
    .ad_wr        (ad_wr        ),
    .ad_level     (ad_level     ),
    .wd_data      (wd_data      ),
    .wd_en        (wd_en        ),
    .cam_px       (cam_px       ),
    .pc_px        (pc_px        ),
    .chunk_cnt    (chunk_cnt    ),
    .busy         (busy         )
);

always @(posedge clk) if (cam_rd) rd_ptr <= rd_ptr + 1;

//--------------------------------------------------------------------
integer fo, fi;
integer blk, i;
integer pat   = 0;
integer seed  = 12345;
integer nblk  = 128;
reg [1023:0] outname;
reg [1023:0] inname;

always @(posedge clk) if (wd_en) $fdisplay(fo, "%04x", wd_data);

// 逐拍探针 (调试用): +DBG=1
integer dbg = 0;
integer sw  = 0;
always @(posedge clk) if (dbg && u_dut.px_valid)
    $display("P pcnt=%0d word=%04x y=%0d q_y=%0d rd=%0d",
             u_dut.pcnt, u_dut.px_in, u_dut.y_val, u_dut.q_y, rd_ptr);

initial begin
    if (!$value$plusargs("MODE=%d",  mode_int)) mode_int = 0;
    if (!$value$plusargs("NBLK=%d",  nblk    )) nblk = 128;
    if (!$value$plusargs("OUT=%s",   outname )) outname = "o.txt";
    if (!$value$plusargs("IN=%s",    inname  )) inname  = "i.txt";
    if (!$value$plusargs("DBG=%d",   dbg     )) dbg = 0;
    if (!$value$plusargs("PAT=%d",   pat     )) pat = 0;
    if (!$value$plusargs("SWITCH=%d", sw      )) sw  = 0;
    mode_in  = mode_int[2:0];
    mode_sel = mode_in;

    fo = $fopen(outname, "w");
    fi = $fopen(inname,  "w");
    if (fo == 0) begin
        $display("ERROR: 打不开输出文件");
        $finish;
    end

    rst_n = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (3) @(posedge clk);

    // 打一个场同步, 让 mode_lat 采信 mode_sel (上电默认是 0=彩色)
    //   do_flip = flip_pend & (st==IDLE) & (ad_level==0) —— 这时三者都成立
    @(posedge clk) cam_vs <= 1'b1;
    @(posedge clk) cam_vs <= 1'b0;
    repeat (3) @(posedge clk);

    // 一块一块地喂: FIFO 模型只有 1024 深, 一次塞满会超
    for (blk = 0; blk < nblk; blk = blk + 1) begin
        // 模拟用户按键: 每块换一个档位, 顺带验证 mode_lat 是在块边界
        // 才更新(不会出现半页新档半页旧档), 以及切换不丢像素。
        if (sw) begin
            mode_in  = blk % 5;
            mode_sel = mode_in[2:0];
            @(posedge clk) cam_vs <= 1'b1;
            @(posedge clk) cam_vs <= 1'b0;
            repeat (3) @(posedge clk);
        end
        rd_ptr = 0;
        wr_ptr = 0;
        fifo[0] = {1'b1, 5'd0, blk[10:0]};        // 标签字
        for (i = 0; i < 512; i = i + 1) begin
            case (pat)
                // 随机: 中值网络在"非单调"三元素下才算真被考到。
                // 递增序列里 a<b<c 恒成立, median 只要选中间那个就行,
                // 排序网络写反了也看不出来。
                1: fifo[i + 1] = $random(seed) & 16'hFFFF;
                // 交替黑白: 验证块首透传不会把上一块的值拉进来
                2: fifo[i + 1] = ((i % 2) == 0) ? 16'hFFFF : 16'h0000;
                // 默认: 像素 = 全局序号, 正好把 RGB565 全值域走一遍
                default: fifo[i + 1] = (blk * 512 + i) & 16'hFFFF;
            endcase
            $fdisplay(fi, "%04x", fifo[i + 1]);
        end
        wr_ptr = 513;

        wait (chunk_cnt == (blk[15:0] + 16'd1));
        repeat (3) @(posedge clk);
    end

    repeat (10) @(posedge clk);
    $fclose(fo); $fclose(fi);
    $display("DONE: %0d 块, 采集 %0d 像素", nblk, cam_px);
    $finish;
end

// 卡死保护
initial begin
    #20_000_000;                 // 20ms
    $display("TIMEOUT: chunk_cnt=%0d", chunk_cnt);
    $fclose(fo); $fclose(fi);
    $finish;
end

endmodule
