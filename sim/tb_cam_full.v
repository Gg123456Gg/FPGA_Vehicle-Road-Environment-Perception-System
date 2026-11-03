//=====================================================================
// tb_cam_full.v   (v23 新增)
//---------------------------------------------------------------------
// 专门验 cam_line_pick 的"摄像头全屏"通道:
//
//   全屏  : 1024x600 原样送出, 每行 2 个标签 + 1024 像素 = 1026 字,
//           块号 = 源行*2 / 源行*2+1, 像素 = 行内全部 1024 个
//   四宫格: 原来的抽取路径必须一个字都不变 (每 2 行 1 行、偶数列)
//
// 检查方式是"逐字比对": 第 k 块必须是 标签(k) + 512 个连续像素,
// 任何一个像素错位/丢失都会被直接指出来。这里最容易出的错就是
// "标签和像素撞拍时丢一个像素", 一旦发生, 后面整块全错。
//=====================================================================
`timescale 1ns/1ps
module tb_cam_full;

reg         pclk = 0;
always #37 pclk = ~pclk;            // ~13.5MHz, 和 1024x600 22fps 相符

reg         rst_n    = 0;
reg         cam_full = 1;
reg         vsync    = 0;
reg         valid    = 0;
reg  [15:0] data     = 0;

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

//---- 收字 ----
reg  [16:0] mem [0:20000];
integer     wi;
always @(negedge pclk) begin
    if (wr_en) begin
        mem[wi] = wr_data;
        wi      = wi + 1;
    end
end

integer errs = 0;

task vsync_pulse;
    begin
        @(negedge pclk) vsync = 1;
        repeat (4) @(negedge pclk);
        vsync = 0;
        repeat (8) @(negedge pclk);
    end
endtask

// 送一行 1024 个像素, 值 = r*1024 + c 的低 16 位
task send_row(input integer r);
    integer c;
    begin
        for (c = 0; c < 1024; c = c + 1) begin
            @(negedge pclk);
            valid = 1;
            data  = (r * 1024 + c) & 16'hFFFF;
        end
        @(negedge pclk);
        valid = 0;
        repeat (20) @(negedge pclk);     // 行消隐
    end
endtask

//---- 全屏: nrow 行 -> nrow*2 块, 每块 标签 + 512 像素 ----
task check_full(input integer nrow);
    integer k, j, base, e, blk, px;
    begin
        e = errs;
        if (wi !== nrow * 1026) begin
            $display("  FAIL 全屏字数 %0d, 应为 %0d (每行 1026)", wi, nrow*1026);
            e = e + 1;
        end
        for (k = 0; k < nrow * 2; k = k + 1) begin
            base = k * 513;
            blk  = k;
            if (mem[base] !== {1'b1, 5'd0, blk[10:0]}) begin
                $display("  FAIL 块 %0d 标签 = %h, 应为块号 %0d", k, mem[base], blk);
                e = e + 1;
            end
            for (j = 0; j < 512; j = j + 1) begin
                px = (k / 2) * 1024 + (k % 2) * 512 + j;
                if (mem[base + 1 + j] !== {1'b0, px[15:0]}) begin
                    if (e < errs + 8)
                        $display("  FAIL 块 %0d 第 %0d 像素 = %h, 应为 %h",
                                 k, j, mem[base+1+j], px[15:0]);
                    e = e + 1;
                end
            end
        end
        if (e == errs)
            $display("  PASS 全屏 %0d 行 -> %0d 块, 块号/像素全部对齐", nrow, nrow*2);
        errs = e;
    end
endtask

//---- 四宫格: nsrc 源行 -> nsrc/2 块, 每块 标签 + 512 个偶数列像素 ----
task check_quad(input integer nsrc);
    integer k, j, base, e, blk, px;
    begin
        e = errs;
        if (wi !== (nsrc / 2) * 513) begin
            $display("  FAIL 四宫格字数 %0d, 应为 %0d", wi, (nsrc/2)*513);
            e = e + 1;
        end
        for (k = 0; k < nsrc / 2; k = k + 1) begin
            base = k * 513;
            blk  = k * 2;                       // 块号 = 目标行*2, 左半
            if (mem[base] !== {1'b1, 5'd0, blk[10:0]}) begin
                $display("  FAIL 块 %0d 标签 = %h, 应为块号 %0d", k, mem[base], blk);
                e = e + 1;
            end
            for (j = 0; j < 512; j = j + 1) begin
                px = (k * 2) * 1024 + j * 2;    // 偶数行 + 偶数列
                if (mem[base + 1 + j] !== {1'b0, px[15:0]}) begin
                    if (e < errs + 8)
                        $display("  FAIL 块 %0d 第 %0d 像素 = %h, 应为 %h",
                                 k, j, mem[base+1+j], px[15:0]);
                    e = e + 1;
                end
            end
        end
        if (e == errs)
            $display("  PASS 四宫格 %0d 源行 -> %0d 块, 抽取路径未变", nsrc, nsrc/2);
        errs = e;
    end
endtask

initial begin
    wi = 0;
    $display("===== tb_cam_full (v23) =====");
    repeat (10) @(negedge pclk);
    rst_n = 1;
    repeat (5)  @(negedge pclk);

    //---------- 全屏: 跑 3 行 ----------
    cam_full = 1;
    repeat (4) @(negedge pclk);          // 等两级同步
    wi = 0;
    vsync_pulse;
    send_row(0);
    send_row(1);
    send_row(2);
    repeat (60) @(negedge pclk);         // 等队列排空
    check_full(3);

    //---------- 切回四宫格: 跑 4 源行 ----------
    cam_full = 0;
    repeat (4) @(negedge pclk);
    wi = 0;
    vsync_pulse;
    send_row(0);
    send_row(1);
    send_row(2);
    send_row(3);
    repeat (60) @(negedge pclk);
    check_quad(4);

    //---------- 再切回全屏 (验证模式来回切不残留) ----------
    cam_full = 1;
    repeat (4) @(negedge pclk);
    wi = 0;
    vsync_pulse;
    send_row(0);
    send_row(1);
    repeat (60) @(negedge pclk);
    check_full(2);

    $display("===== %s =====", (errs == 0) ? "ALL PASS" : "HAS FAIL");
    $finish;
end

endmodule
