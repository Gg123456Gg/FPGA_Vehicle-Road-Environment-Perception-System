//=====================================================================
// tb_mode_switch.v   —— 复现"帧中途切换 cam_full 导致块结构错位"
//
// 写控制器的契约（quad_wr_ctrl 就是照这个啃的）：
//   每块 = 1 个标签字(bit16=1) + 512 个像素字(bit16=0)，共 513 字。
// 一旦这个节奏被破坏一个字，后面整条流就全错位：写侧会把像素当标签、
// 拿到一个垃圾块号，块落到画面外面 —— 上板看到的就是"画面分裂成长条"
// 甚至"定格不动"。
//
// 这个测试台做两件事：
//   1) 正常模式，确认 513 节奏成立（回归）
//   2) 在某一行的中间切换 cam_full，看节奏还成不成立   <- 上板故障场景
//=====================================================================
`timescale 1ns/1ps
module tb_mode_switch;

reg         pclk = 0;
always #37 pclk = ~pclk;

reg         rst_n = 0;
reg         cam_full = 1;
reg         vsync = 0;
reg         valid = 0;
reg  [15:0] data  = 0;

wire        wr_en;
wire [16:0] wr_data;
wire [3:0]  dbg;

cam_line_pick dut(
    .cam_pclk           (pclk     ),
    .rst_n              (rst_n    ),
    .cam_full           (cam_full ),
    .lane_bus     (128'd0   ),   // v31: 本台不测车道线, 给常量
    .lane_vld     (1'b0    ),
    .lane_en      (1'b1    ),   // 测试里图层常开
    .lane_osd_en  (1'b1    ),   // v51: 显示开关, 测试里常开
        .force_go  (1'b0 ), .force_l  (1'b0 ), .force_r  (1'b0 ),
        .alarm      (     ),
    .cmos_frame_vsync   (vsync    ),
    .cmos_frame_href    (1'b0     ),
    .cmos_frame_valid   (valid    ),
    .cmos_frame_data    (data     ),
    .fifo_level         (13'd0    ),
    .fifo_full          (1'b0     ),
    .fifo_wr_en         (wr_en    ),
    .fifo_wr_data       (wr_data  ),
    .dbg                (dbg      )
);

//---- 收字 + 节奏检查 ----
integer  nw      = 0;      // 已经吐出多少个字
integer  bad     = 0;      // 节奏违例次数
integer  first_bad = -1;
reg      armed   = 0;      // 复位后等 vsync 再开始查

always @(negedge pclk) begin
    if (wr_en) begin
        if (armed) begin
            if (nw % 513 == 0) begin
                // 每组第 0 个字必须是标签
                if (wr_data[16] !== 1'b1) begin
                    bad = bad + 1;
                    if (first_bad < 0) begin
                        first_bad = nw;
                        $display("  ** 第 %0d 个字该是标签, 实际 bit16=0 (值 0x%04h)", nw, wr_data[15:0]);
                    end
                end
            end else begin
                // 其余必须是像素
                if (wr_data[16] !== 1'b0) begin
                    bad = bad + 1;
                    if (first_bad < 0) begin
                        first_bad = nw;
                        $display("  ** 第 %0d 个字该是像素, 实际 bit16=1 (标签 %0d)", nw, wr_data[10:0]);
                    end
                end
            end
        end
        nw = nw + 1;
    end
end

//---- 摄像头驱动 ----
task vsync_pulse;
    begin
        @(negedge pclk) vsync = 1;
        repeat (4) @(negedge pclk);
        vsync = 0;
        repeat (40) @(negedge pclk);      // 场消隐
    end
endtask

// 走一行: 1024 个 valid
task one_line;
    begin
        repeat (1024) begin
            @(negedge pclk);
            valid = 1;
            data  = $random;
        end
        @(negedge pclk);
        valid = 0;
        repeat (60) @(negedge pclk);      // 行消隐
    end
endtask

task run_frame(input integer nline);
    integer k;
    begin
        for (k = 0; k < nline; k = k + 1) one_line;
    end
endtask

integer base_nw;

initial begin
    $display("==== 帧中途切换 cam_full 的块结构检查 ====");
    #200 rst_n = 1;
    repeat (10) @(negedge pclk);

    // ---------- 第 1 帧: 全屏模式, 跑 20 行 ----------
    vsync_pulse;
    armed = 1;
    nw    = 0;
    run_frame(20);
    base_nw = nw;

    // 全屏一帧 20 行 = 40 块 = 40*513 = 20520 字
    $display("第 1 帧(全屏 20 行): 吐出 %0d 字, 期望 %0d, 节奏违例 %0d (首个 %0d)",
             nw, 20*2*513, bad, first_bad);
    if (bad == 0 && nw == 20*2*513)
        $display("PASS: 正常全屏模式块结构正确");
    else
        $display("FAIL: 正常模式下就已经不对");

    // ---------- 第 2 帧: 跑到第 5 行的第 300 个像素时切到四宫格 ----------
    vsync_pulse;
    nw  = 0;
    bad = 0;
    first_bad = -1;

    run_frame(5);
    // 再走一行的前 300 个像素
    repeat (300) begin
        @(negedge pclk);
        valid = 1;
        data  = $random;
    end
    // ★就在这里切换模式★ (行中间)
    cam_full = 0;
    $display("  >> 在第 6 行的第 300 个像素处把 cam_full 1 -> 0");
    // 这一行剩下的 724 个像素
    repeat (724) begin
        @(negedge pclk);
        valid = 1;
        data  = $random;
    end
    @(negedge pclk);
    valid = 0;
    repeat (60) @(negedge pclk);
    // 再跑 4 行(四宫格模式)
    run_frame(4);

    $display("切换后: 吐出 %0d 字, 节奏违例 %0d (首个违例在 %0d)", nw, bad, first_bad);
    if (bad == 0)
        $display("PASS: 帧中途切换后块结构仍然连续");
    else
        $display("FAIL: 帧中途切换把块结构打散了 —— 这就是上板看到的\"长条\"和\"定格\"");

    $display("==== 结束 ====");
    $finish;
end

endmodule
