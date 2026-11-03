//=====================================================================
// tb_cmd_lane.v  ——  v50: UDP 命令解码器 (两条命令)
//---------------------------------------------------------------------
// 板子上现在能收两类 4 字节命令, 靠 magic 区分:
//   A5 5B <档位 0..4> <~档位>    算法档位 (v24 就有)
//   A5 5C <1/0>      <~值>       车道线识别开关 (v50 新增)
//
// 要证的四件事:
//   1. 两条命令互不串台 —— 发 A55C 不能改档位, 发 A55B 不能改车道开关
//   2. 校验字节错 -> 整条丢弃
//   3. 值越界 -> 丢弃 (A55B 只认 0..4, A55C 只认 0/1)
//   4. 视频包头 0xA55A -> 两个都不动 (否则每帧视频都会把开关翻一次)
//
// 时序按真实接口来: rec_en 在包内是"每拍一个字节"连续有效, 包与包之间
// 至少一拍低电平 —— 解码器就是靠这个低电平重新对齐的。
//=====================================================================
`timescale 1ns/1ps
module tb_cmd_lane;

    reg clk = 0;
    always #4 clk = ~clk;          // 125MHz
    reg rst_n = 0;

    reg       rec_en   = 0;
    reg [7:0] rec_data = 0;
    wire      cmd_tog;
    wire [2:0] cmd_mode;
    wire      lane_tog;
    wire      lane_on;

    udp_cmd_dec u_dut(
        .clk      (clk      ),
        .rst_n    (rst_n    ),
        .rec_en   (rec_en   ),
        .rec_data (rec_data ),
        .cmd_tog  (cmd_tog  ),
        .cmd_mode (cmd_mode ),
        .lane_tog (lane_tog ),
        .lane_on  (lane_on  )
    );

    integer errs = 0;

    // 发一个 4 字节包
    task send4;
        input [7:0] a, b, c, d;
        begin
            @(negedge clk); rec_en = 1'b0;
            @(negedge clk);
            @(negedge clk); rec_en = 1'b1; rec_data = a;
            @(negedge clk);              rec_data = b;
            @(negedge clk);              rec_data = c;
            @(negedge clk);              rec_data = d;
            @(negedge clk); rec_en = 1'b0;
            repeat (3) @(negedge clk);
        end
    endtask

    task chk;
        input [127:0] name;
        input         cond;
        begin
            if (!cond) begin
                $display("  *** FAIL: %0s  (mode=%0d lane=%0b)", name, cmd_mode, lane_on);
                errs = errs + 1;
            end else
                $display("    ok  %0s", name);
        end
    endtask

    reg t0;

    initial begin
        repeat (10) @(posedge clk);
        rst_n = 1;
        repeat (10) @(posedge clk);

        // ---- 上电默认: 识别开着, 档位 0 ----
        chk("power-on lane_on=1", lane_on === 1'b1);
        chk("power-on mode=0",    cmd_mode === 3'd0);

        // ---- 1) 档位命令仍然好使 ----
        send4(8'hA5, 8'h5B, 8'd3, 8'hFC);
        chk("A55B -> mode=3",  cmd_mode === 3'd3);
        chk("A55B keeps lane switch", lane_on === 1'b1);

        // ---- 2) 关车道线 ----
        t0 = lane_tog;
        send4(8'hA5, 8'h5C, 8'd0, 8'hFF);
        chk("A55C 00 -> lane_on=0", lane_on === 1'b0);
        chk("A55C toggles lane_tog",   lane_tog !== t0);
        chk("A55C keeps algo",        cmd_mode === 3'd3);

        // ---- 3) 再打开 ----
        send4(8'hA5, 8'h5C, 8'd1, 8'hFE);
        chk("A55C 01 -> lane_on=1", lane_on === 1'b1);

        // ---- 4) 校验字节错: 整条丢弃 ----
        t0 = lane_tog;
        send4(8'hA5, 8'h5C, 8'd0, 8'h00);     // 应该是 FF
        chk("bad checksum: switch unchanged", lane_on === 1'b1);
        chk("bad checksum: no toggle",   lane_tog === t0);

        // ---- 5) 值越界 ----
        send4(8'hA5, 8'h5C, 8'd2, 8'hFD);
        chk("A55C val=2 dropped", lane_on === 1'b1);

        t0 = cmd_tog;
        send4(8'hA5, 8'h5B, 8'd7, 8'hF8);     // 档位只有 0..4
        chk("A55B val=7 dropped", cmd_mode === 3'd3);
        chk("A55B val=7 no toggle", cmd_tog === t0);

        // ---- 6) 视频包头不能误触发 ----
        t0 = cmd_tog;
        send4(8'hA5, 8'h5A, 8'd0, 8'hFF);     // 视频包 magic, 后面跟什么都不认
        chk("A55A: algo unchanged", cmd_mode === 3'd3);
        chk("A55A: switch unchanged", lane_on === 1'b1);

        // ---- 7) 关->开 回环, 确认能反复切 ----
        send4(8'hA5, 8'h5C, 8'd0, 8'hFF);
        chk("2nd off", lane_on === 1'b0);
        send4(8'hA5, 8'h5C, 8'd1, 8'hFE);
        chk("2nd on", lane_on === 1'b1);

        if (errs == 0) $display("CMD_LANE_PASS");
        else           $display("CMD_LANE_FAIL errs=%0d", errs);
        $finish;
    end

    initial begin
        #200000;
        $display("  *** FAIL: timeout");
        $display("CMD_LANE_FAIL");
        $finish;
    end

endmodule
