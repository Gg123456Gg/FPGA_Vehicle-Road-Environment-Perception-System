//=====================================================================
// tb_ego_state.v  ——  v45: 左线锚点 8 帧差分 -> 直行/左转/右转
//---------------------------------------------------------------------
// 判据: dx = x_L(t) - x_L(t-8)
//   dx < -DX_TH : 左线左移 = 车右转
//   dx > +DX_TH : 左线右移 = 车左转
//   累积 ACC_TH 帧才算, 带滞回
//
// 用例重点是"方向"和"不误报":
//   直道小抖动不能亮; 持续单向移动必须亮; 反向要能改判; 无效锚点要冻结状态。
//=====================================================================
`timescale 1ns / 1ps

module tb_ego_state;

    reg         clk = 1'b0, rst_n = 1'b0, tick = 1'b0;
    reg  [10:0] lane_x = 11'd0;
    wire [1:0]  ego;

    ego_state #(
        .DX_TH (10'd24), .ACC_TH (5'd12), .ACC_HY (5'd4)
    ) u_dut (
        .clk(clk), .rst_n(rst_n), .tick(tick), .lane_x(lane_x), .proj_dx(5'sd0), .proj_vld(1'b0), .ego(ego)
    );

    always #10 clk = ~clk;

    integer errs = 0;

    // 一帧: 给一个锚点值, 打一拍 tick
    task frame;
        input [9:0] x;
        input       valid;
        begin
            lane_x = {valid, x};
            @(posedge clk);
            tick = 1'b1;
            @(posedge clk);
            tick = 1'b0;
            @(posedge clk);
        end
    endtask

    task hold;
        input integer n;
        input [9:0] x;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) frame(x, 1'b1);
        end
    endtask

    // 每帧移动 step 像素, 跑 n 帧
    task ramp;
        input integer n;
        input integer step;
        integer k; reg [9:0] v;
        begin
            v = lane_x[9:0];
            for (k = 0; k < n; k = k + 1) begin
                v = v + step[9:0];
                frame(v, 1'b1);
            end
        end
    endtask

    task chk;
        input [1:0] want;
        input [127:0] name;
        begin
            if (ego !== want) begin
                $display("  *** FAIL %0s  ego=%0d want=%0d", name, ego, want);
                errs = errs + 1;
            end else
                $display("  OK   %0s  ego=%0d", name, ego);
        end
    endtask

    initial begin
        $display("=== ego_state (v45: left-anchor 8-frame diff) ===");

        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (3) @(posedge clk);
        chk(2'd0, "after reset");

        // ---- 直道: 锚点稳在 265, 不该亮 ----
        hold(20, 10'd265);
        chk(2'd0, "steady straight");

        // ---- 抖 +-2: dx 最大 16, 小于 24, 不该亮 ----
        begin : jit
            integer k;
            for (k = 0; k < 24; k = k + 1)
                frame((k[0]) ? 10'd267 : 10'd263, 1'b1);
        end
        chk(2'd0, "jitter +-2 -> straight");

        // ---- 每帧 -4, 8 帧后 dx=-32, 累积 8 帧 -> 右转 ----
        hold(10, 10'd265);
        ramp(24, -4);
        chk(2'd2, "x decreasing -> RIGHT");

        // ---- 停下来, 滞回保持若干帧 ----
        hold(3, 10'd201);
        chk(2'd2, "hold -> still RIGHT");

        // ---- 继续停稳, acc 回零 -> 直行 ----
        hold(16, 10'd201);
        chk(2'd0, "settled -> straight");

        // ---- 反向: 每帧 +4 -> 左转 ----
        ramp(24, 4);
        chk(2'd1, "x increasing -> LEFT");

        hold(3, 10'd265);
        chk(2'd1, "hold -> still LEFT");

        hold(16, 10'd265);
        chk(2'd0, "settled -> straight");

        // ---- 慢速: 每帧 -3 (dx=-24 刚好等于阈值, 用严格小于, 不该亮) ----
        hold(10, 10'd265);
        ramp(10, -3);
        chk(2'd0, "dx == -24 exactly -> straight");

        // ---- 无效锚点: 保持上一次状态, 不乱动 ----
        hold(24, 10'd265);          // 够长, 把上一段的跳变挤出历史
        ramp(24, -4);
        chk(2'd2, "turn right again");
        begin : novalid
            integer k;
            for (k = 0; k < 20; k = k + 1) frame(10'd0, 1'b0);
        end
        chk(2'd2, "anchor invalid -> keep RIGHT");

        // 恢复后继续同一方向, 应仍是右转
        hold(4, 10'd200);
        chk(2'd2, "recover -> still RIGHT");

        $display("errs=%0d", errs);
        if (errs == 0) $display("EGO_STATE_PASS");
        $finish;
    end

endmodule
