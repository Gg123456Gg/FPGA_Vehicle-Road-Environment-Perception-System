//=====================================================================
// tb_lane_dec.v  ——  决策状态机验证
//---------------------------------------------------------------------
// 8 个场景, 每个场景连续喂 12 帧相同输入(超过去抖门限 5 帧),
// 然后检查稳定后的 dec / lvl。判据写在每个 case 的注释里。
//=====================================================================
`timescale 1ns/1ps
module tb_lane_dec;

    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;

    reg        tick = 0;
    reg [1:0]  ltyp = 0, rtyp = 0;
    reg [1:0]  offl = 0;
    reg [10:0] offv = 0;
    reg [2:0]  lstate = 7;
    reg [1:0]  cdir = 0;
    reg [2:0]  tl_kind = 0;
    reg        force_go = 0, force_l = 0, force_r = 0;   // v39/v40 模拟强制命令
    wire [3:0] dec;
    wire [1:0] lvl;
    wire [1:0] cross;
    wire       alarm;                                    // v40 唯一的报警灯

    lane_decision u_dut (
        .clk (clk), .rst_n (rst_n), .tick (tick),
        .ltyp (ltyp), .rtyp (rtyp), .offl (offl), .offv (offv),
        .lstate (lstate), .cdir (cdir), .tl_kind (tl_kind),
        .force_go (force_go), .force_l (force_l), .force_r (force_r),
        .dec (dec), .lvl (lvl), .lc_cross (cross), .alarm (alarm)
    );

    localparam [3:0] DEC_HOLD = 0, DEC_STOP = 2, DEC_LOST = 7, DEC_SLOW = 8,
                     DEC_LC_BOTH = 9, DEC_LC_L = 10, DEC_LC_R = 11, DEC_LC_NO = 12;

    integer errs = 0;
    integer f;

    // 一帧输入
    task frame;
        input [1:0] lt, rt;
        input [1:0] ol;
        input [10:0] ov;
        input [2:0] ls;
        input [1:0] cd;
        input [2:0] tk;
        begin
            ltyp = lt; rtyp = rt; offl = ol; offv = ov;
            lstate = ls; cdir = cd; tl_kind = tk;
            tick = 1'b1;
            @(posedge clk);
            tick = 1'b0;
            @(posedge clk);
        end
    endtask

    task run_n;
        input integer n;
        begin
            for (f = 0; f < n; f = f + 1) @(posedge clk);
        end
    endtask

    task chk;
        input [3:0] want_dec;
        input [1:0] want_lvl;
        input [127:0] name;
        begin
            if (dec !== want_dec || lvl !== want_lvl) begin
                $display("  *** FAIL %0s: dec=%0d lvl=%0d  期望 dec=%0d lvl=%0d",
                         name, dec, lvl, want_dec, want_lvl);
                errs = errs + 1;
            end else
                $display("  OK   %0s: dec=%0d lvl=%0d cross=%0d", name, dec, lvl, cross);
        end
    endtask

    task chk_alarm;
        input        want;
        input [127:0] name;
        begin
            if (alarm !== want) begin
                $display("  *** FAIL alarm %0s: alarm=%b 期望 %b", name, alarm, want);
                errs = errs + 1;
            end else
                $display("  OK   alarm %0s: alarm=%b", name, alarm);
        end
    endtask

    initial begin
        $display("==== lane_decision ====");
        rst_n = 0;
        run_n(5);
        rst_n = 1;
        run_n(3);

        // ===== v36: 判据换成"变道许可四态" =====
        //   核心变化: 不再要求"正在变道(lstate==3)"才报, 每帧直接按左右线型给结论。
        //   frame(lt, rt, ol, ov, ls, cd, tk)

        // 1) 两边都虚线 -> 左右都能变
        for (f = 0; f < 12; f = f + 1) frame(2'd2, 2'd2, 2'b11, 11'd0, 3'd0, 2'd0, 3'd0);
        chk(DEC_LC_BOTH, 2'd0, "both dashed");

        // 2) 两边都实线 -> 都不能变(注意级, 不是危险)
        for (f = 0; f < 12; f = f + 1) frame(2'd1, 2'd1, 2'b11, 11'd0, 3'd0, 2'd0, 3'd0);
        chk(DEC_LC_NO, 2'd1, "both solid");

        // 3) 左虚右实 -> 只能向左
        for (f = 0; f < 12; f = f + 1) frame(2'd2, 2'd1, 2'b11, 11'd0, 3'd0, 2'd0, 3'd0);
        chk(DEC_LC_L, 2'd0, "only left");

        // 4) 左实右虚 -> 只能向右
        for (f = 0; f < 12; f = f + 1) frame(2'd1, 2'd2, 2'b11, 11'd0, 3'd0, 2'd0, 3'd0);
        chk(DEC_LC_R, 2'd0, "only right");

        // 5) v37: 红灯不再参与决策 —— 两线都虚线, 照样报 BOTH, 等级 0
        //    红绿灯是独立识别通路, 只画框不上决策。这一条锁住这个语义:
        //    tl_kind=1 传进来必须被忽略。
        for (f = 0; f < 12; f = f + 1) frame(2'd2, 2'd2, 2'b11, 11'd0, 3'd0, 2'd0, 3'd1);
        chk(DEC_LC_BOTH, 2'd0, "red light ignored");

        // 6) 两边都没线: 丢失
        for (f = 0; f < 12; f = f + 1) frame(2'd0, 2'd0, 2'b00, 11'd0, 3'd7, 2'd0, 3'd0);
        chk(DEC_LOST, 2'd2, "lost");

        // 7) 两条线型都判不出 -> 不表态
        for (f = 0; f < 12; f = f + 1) frame(2'd0, 2'd0, 2'b11, 11'd0, 3'd0, 2'd0, 3'd0);
        chk(DEC_SLOW, 2'd1, "both unknown");

        // 8) 只有右边判不出 -> 也不表态(只报一边算瞎猜)
        for (f = 0; f < 12; f = f + 1) frame(2'd2, 2'd0, 2'b11, 11'd0, 3'd0, 2'd0, 3'd0);
        chk(DEC_SLOW, 2'd1, "right unknown");

        // 9) 变道中 + 都虚线: 结论和没变道时一样(证明不看 lstate)
        for (f = 0; f < 12; f = f + 1) frame(2'd2, 2'd2, 2'b11, 11'd40, 3'd3, 2'd1, 3'd0);
        chk(DEC_LC_BOTH, 2'd0, "changing+both");

        // 10) 变道中 + 都实线: 同样只看线型
        for (f = 0; f < 12; f = f + 1) frame(2'd1, 2'd1, 2'b11, 11'd40, 3'd3, 2'd1, 3'd0);
        chk(DEC_LC_NO, 2'd1, "changing+solid");

        // 11) 绿灯不干扰许可判定
        for (f = 0; f < 12; f = f + 1) frame(2'd2, 2'd1, 2'b11, 11'd0, 3'd0, 2'd0, 3'd2);
        chk(DEC_LC_L, 2'd0, "green+only left");

        // 12) 单帧毛刺不应改变结论(去抖有效性)
        for (f = 0; f < 12; f = f + 1) frame(2'd1, 2'd2, 2'b11, 11'd0, 3'd0, 2'd0, 3'd0);
        frame(2'd0, 2'd0, 2'b00, 11'd0, 3'd7, 2'd0, 3'd0);   // 突然丢一帧
        frame(2'd1, 2'd2, 2'b11, 11'd0, 3'd0, 2'd0, 3'd0);   // 立刻恢复
        chk(DEC_LC_R, 2'd0, "glitch filtered");

        //===== v40: 报警灯 (k2/k3/k4 三颗键共用一个灯) =====
        //  判据顺序: 先判环境(线型/灯色), 再判按键, 两者冲突才亮。
        //  这一组专门锁住"只按键不亮"—— 现场反馈过"一按键就亮"的毛病。
        force_go = 0; force_l = 0; force_r = 0;

        // A1 不按键 + 两边实线 + 红灯 -> 灭 (没有指令就没有报警)
        for (f = 0; f < 3; f = f + 1) frame(2'd1, 2'd1, 2'b11, 11'd0, 3'd0, 2'd0, 3'd1);
        chk_alarm(1'b0, "no key, solid+red");

        // A2 强制直行 + 红灯 -> 亮 (闯红灯)
        force_go = 1;
        for (f = 0; f < 3; f = f + 1) frame(2'd1, 2'd1, 2'b11, 11'd0, 3'd0, 2'd0, 3'd1);
        chk_alarm(1'b1, "go + red");

        // A3 强制直行 + 绿灯 -> 灭 ★只按键不亮★
        for (f = 0; f < 3; f = f + 1) frame(2'd1, 2'd1, 2'b11, 11'd0, 3'd0, 2'd0, 3'd2);
        chk_alarm(1'b0, "go + green (key only)");

        // A4 强制左变道 + 左线实线 -> 亮 (压实线)
        force_go = 0; force_l = 1;
        for (f = 0; f < 3; f = f + 1) frame(2'd1, 2'd1, 2'b11, 11'd0, 3'd0, 2'd0, 3'd0);
        chk_alarm(1'b1, "left cmd + left solid");

        // A5 强制左变道 + 左线虚线 -> 灭 ★虚线能压, 按了也不亮★
        for (f = 0; f < 3; f = f + 1) frame(2'd2, 2'd1, 2'b11, 11'd0, 3'd0, 2'd0, 3'd0);
        chk_alarm(1'b0, "left cmd + left dashed");

        // A6 强制右变道 + 右线实线 -> 亮
        force_l = 0; force_r = 1;
        for (f = 0; f < 3; f = f + 1) frame(2'd1, 2'd1, 2'b11, 11'd0, 3'd0, 2'd0, 3'd0);
        chk_alarm(1'b1, "right cmd + right solid");

        // A7 强制右变道 + 右线虚线 -> 灭
        for (f = 0; f < 3; f = f + 1) frame(2'd1, 2'd2, 2'b11, 11'd0, 3'd0, 2'd0, 3'd0);
        chk_alarm(1'b0, "right cmd + right dashed");

        // A8 三颗键同时按 + 全虚线 + 绿灯 -> 灭 (互斥优先级, 不会叠出报警)
        force_go = 1; force_l = 1; force_r = 1;
        for (f = 0; f < 3; f = f + 1) frame(2'd2, 2'd2, 2'b11, 11'd0, 3'd0, 2'd0, 3'd2);
        chk_alarm(1'b0, "all keys + dashed + green");

        // A9 松手 + 还是红灯/实线 -> 灭 (报警只跟"指令冲突"有关)
        force_go = 0; force_l = 0; force_r = 0;
        for (f = 0; f < 3; f = f + 1) frame(2'd1, 2'd1, 2'b11, 11'd0, 3'd0, 2'd0, 3'd1);
        chk_alarm(1'b0, "release all");

        if (errs == 0) $display("LANE_DEC_PASS");
        else           $display("LANE_DEC_FAIL errs=%0d", errs);
        $finish;
    end

endmodule
