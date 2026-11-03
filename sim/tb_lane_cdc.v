//=====================================================================
// tb_lane_cdc.v  ——  lane_cdc 的跨时钟域验证
//---------------------------------------------------------------------
// 判据(三条, 缺一条都不算过):
//   ① 帧数不能少也不能多  —— vld_b 的个数 == vld_a 的个数
//   ② 每一帧的数据必须完整 —— 高 16 位放帧号, 低 112 位放帧号的 4 种
//      变换(取反/移位/异或), 收到后重新算一遍对比, 只要有一位被采到
//      "半新半旧"就会不匹配
//   ③ 顺序必须是递增的     —— 帧号乱序说明握手丢事件
//
// 两个时钟故意不同频且不成整数比(21ns / 20ns), 让采样点扫遍整个相位,
// 比"同频"更容易暴露问题。
//=====================================================================
`timescale 1ns/1ps
module tb_lane_cdc;

    reg clk_a = 0;          // cam_pclk  ~48MHz
    reg clk_b = 0;          // clk_50m   50MHz
    reg rst_n = 0;

    always #10.5 clk_a = ~clk_a;
    always #10.0 clk_b = ~clk_b;

    reg  [127:0] dat_a = 0;
    reg          vld_a = 0;
    wire [127:0] dat_b;
    wire         vld_b;

    lane_cdc u_dut (
        .clk_a (clk_a), .rst_n (rst_n),
        .dat_a (dat_a), .vld_a (vld_a),
        .clk_b (clk_b), .dat_b (dat_b), .vld_b (vld_b)
    );

    integer errs  = 0;
    integer nsend = 0;
    integer nrecv = 0;

    // 造数据: 高 16 位帧号, 其余按帧号做几种变换(保证每一位都动过)
    function [127:0] mk;
        input integer f;
        reg [111:0] p;
        begin
            p = {7{f[15:0]}};                          // 112 位, 每 16 位都是帧号
            p = p ^ ({28{4'hF}} >> f[3:0]);             // 每位都动过
            p = p + {104'd0, f[7:0]};
            mk = {f[15:0], p};
        end
    endfunction

    // 接收侧: 每来一个 vld_b 就检查
    integer  expect_f = 0;
    always @(posedge clk_b) begin
        if (rst_n && vld_b) begin
            nrecv = nrecv + 1;
            if (dat_b !== mk(expect_f)) begin
                $display("  *** FAIL: recv#%0d 期望帧 %0d, 数据不匹配", nrecv, expect_f);
                $display("      收: %032h", dat_b);
                $display("      期: %032h", mk(expect_f));
                errs = errs + 1;
            end else if (nrecv <= 3 || nrecv > 17)
                $display("  recv#%0d frame=%0d 数据一致", nrecv, dat_b[127:112]);
            expect_f = expect_f + 1;
        end
    end

    integer i;
    initial begin
        $display("==== lane_cdc: cam_pclk(21ns) -> clk_50m(20ns) ====");
        repeat (5) @(posedge clk_b);
        rst_n = 1;
        repeat (3) @(posedge clk_b);

        // 20 帧, 每帧间隔随机(模拟 22fps 附近抖动)
        for (i = 0; i < 20; i = i + 1) begin
            @(posedge clk_a);
            dat_a <= mk(i);
            vld_a <= 1'b1;
            nsend  = nsend + 1;
            @(posedge clk_a);
            vld_a <= 1'b0;
            // 帧间隔: 45ms 太慢, 缩到 200~300 个 clk_a 即可(相位照样扫遍)
            repeat (200 + (i * 37) % 100) @(posedge clk_a);
        end

        repeat (50) @(posedge clk_b);
        $display("---- 发 %0d 帧, 收 %0d 帧 ----", nsend, nrecv);
        if (nrecv != nsend) begin
            $display("  *** FAIL: 帧数不一致(丢/多事件)"); errs = errs + 1;
        end
        if (errs == 0) $display("LANE_CDC_PASS");
        else           $display("LANE_CDC_FAIL errs=%0d", errs);
        $finish;
    end

endmodule
