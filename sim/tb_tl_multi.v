//=====================================================================
// tb_tl_multi.v   traffic_light v17 "一帧多个框" 的专项测试台
//---------------------------------------------------------------------
// v16 及以前每一帧只结算一个赢家 (best_*), 所以画面里只要有一个大红灯,
// 绿灯/黄灯永远出不来 —— 现场表现就是"有多个灯却只框红灯"。
//
// v17 改成: 每帧保留 4 个定稿团块, 按面积从大到小排好, 全部交给画框逻辑。
// 这个测试台验三件事:
//   1) 一帧里有 3 个不同颜色的灯 -> 3 个框全部出来, 颜色各对各
//   2) 排序: 0 号是面积最大的, 依次递减 (tb_tl 靠"0 号=最大"和模型对齐)
//   3) 上限: 给 5 个灯时只保留最大的 4 个, 最小的那个丢掉而不是挤掉大的
//
// 为什么用合成图案而不是视频: 视频里没有"三个灯同时在不同位置、尺寸已知"
// 的画面, 而 RTL 的输出坐标可以逐位对照, 合成图案能给出精确的期望值。
//=====================================================================
`timescale 1ns/1ps

module tb_tl_multi;

    reg clk = 1'b0;
    always #10 clk = ~clk;

    reg        rst_n       = 1'b0;
    reg        px_en       = 1'b0;
    reg [9:0]  px_x        = 10'd0;
    reg [9:0]  px_y        = 10'd0;
    reg        line_flag   = 1'b1;
    reg [15:0] px565       = 16'h0000;
    reg        frame_start = 1'b0;

    wire [39:0] bxy0, bxy1, bxy2, bxy3;
    wire [2:0]  bk0, bk1, bk2, bk3;
    wire [3:0]  bvv;
    wire        busy;

    traffic_light dut (
        .clk         (clk        ),
        .rst_n       (rst_n      ),
        .full        (1'b0       ),   // 四宫格参数: 面积门限 40, 小色块也能成团
        .px_en       (px_en      ),
        .px_x        (px_x       ),
        .px_y        (px_y       ),
        .line_flag   (line_flag  ),
        .px565       (px565      ),
        .frame_start (frame_start),
        .bxy0        (bxy0       ),
        .bxy1        (bxy1       ),
        .bxy2        (bxy2       ),
        .bxy3        (bxy3       ),
        .bk0         (bk0        ),
        .bk1         (bk1        ),
        .bk2         (bk2        ),
        .bk3         (bk3        ),
        .bv          (bvv        ),
        .busy        (busy       )
    );

    localparam [15:0] C_BLK = 16'h0000;
    localparam [15:0] C_RED = 16'hF800;   // (255,  0,  0)
    localparam [15:0] C_GRN = 16'h07E0;   // (  0,255,  0)
    localparam [15:0] C_YEL = 16'hFDE2;   // (255,190, 16)

    integer nfail;
    integer yy, xx;

    //------------------------------------------------------------------
    // 画面 1: 3 个灯 (尺寸各不相同, 面积 4560 / 2240 / 1080)
    //------------------------------------------------------------------
    function [15:0] scene3;
        input integer x, y;
        begin
            if ((x >= 200) && (x <= 275) && (y >=  40) && (y <=  99))
                scene3 = C_RED;    // A 76x60 = 4560
            else if ((x >=  60) && (x <= 115) && (y >= 150) && (y <= 189))
                scene3 = C_YEL;    // B 56x40 = 2240
            else if ((x >= 400) && (x <= 445) && (y >= 160) && (y <= 193))
                scene3 = C_GRN;    // C 46x34 = 1564
            else
                scene3 = C_BLK;
        end
    endfunction

    //------------------------------------------------------------------
    // 画面 2: 5 个灯, 按面积 A>B>C>D>E, 期望只留 A/B/C/D (E 被丢掉)
    //------------------------------------------------------------------
    function [15:0] scene5;
        input integer x, y;
        begin
            if ((x >= 200) && (x <= 275) && (y >=  40) && (y <=  99))
                scene5 = C_RED;    // A 76x60 = 4560
            else if ((x >=  60) && (x <= 115) && (y >= 150) && (y <= 189))
                scene5 = C_YEL;    // B 56x40 = 2240
            else if ((x >= 400) && (x <= 445) && (y >= 150) && (y <= 183))
                scene5 = C_GRN;    // C 46x34 = 1564
            else if ((x >= 300) && (x <= 335) && (y >= 230) && (y <= 255))
                scene5 = C_RED;    // D 36x26 =  936
            else if ((x >= 130) && (x <= 155) && (y >= 250) && (y <= 267))
                scene5 = C_GRN;    // E 26x18 =  468  <- 应该被挤掉
            else
                scene5 = C_BLK;
        end
    endfunction

    task do_frame_start;
        begin
            @(posedge clk);
            frame_start <= 1'b1;
            @(posedge clk);
            frame_start <= 1'b0;
            repeat (40) @(posedge clk);
        end
    endtask

    // sc: 3 = scene3, 5 = scene5
    task feed_scene;
        input integer sc;
        input integer y_lo, y_hi;
        begin
            line_flag <= 1'b1;
            for (yy = y_lo; yy <= y_hi; yy = yy + 1) begin
                px_en <= 1'b0;
                repeat (3) @(posedge clk);
                for (xx = 0; xx < 512; xx = xx + 1) begin
                    px_en <= 1'b1;
                    px_x  <= xx;
                    px_y  <= yy;
                    px565 <= (sc == 3) ? scene3(xx, yy) : scene5(xx, yy);
                    @(posedge clk);
                end
            end
            px_en <= 1'b0;
            repeat (20) @(posedge clk);
        end
    endtask

    task chk_box;
        input integer idx;
        input integer ex0, ey0, ex1, ey1;
        input integer ek;
        reg [39:0] xy;
        reg [1:0]  kk;
        begin
            case (idx)
                0:       begin xy = bxy0; kk = bk0; end
                1:       begin xy = bxy1; kk = bk1; end
                2:       begin xy = bxy2; kk = bk2; end
                default: begin xy = bxy3; kk = bk3; end
            endcase
            if ((xy[ 9: 0] === ex0[9:0]) && (xy[19:10] === ey0[9:0]) &&
                (xy[29:20] === ex1[9:0]) && (xy[39:30] === ey1[9:0]) &&
                (kk === ek[1:0])) begin
                $display("  框%0d  %0d,%0d-%0d,%0d  kind=%0d  PASS",
                         idx, xy[9:0], xy[19:10], xy[29:20], xy[39:30], kk);
            end else begin
                $display("  框%0d  实际 %0d,%0d-%0d,%0d kind=%0d",
                         idx, xy[9:0], xy[19:10], xy[29:20], xy[39:30], kk);
                $display("        期望 %0d,%0d-%0d,%0d kind=%0d   ** FAIL **",
                         ex0, ey0, ex1, ey1, ek);
                nfail = nfail + 1;
            end
        end
    endtask

    task chk_v;
        input [3:0] want;
        begin
            if (bvv === want)
                $display("  有效位 %b  PASS", bvv);
            else begin
                $display("  有效位 实际 %b 期望 %b  ** FAIL **", bvv, want);
                nfail = nfail + 1;
            end
        end
    endtask

    initial begin
        nfail = 0;
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (40) @(posedge clk);

        // ================= S0: 3 个灯, 三个都要出来 =================
        $display("");
        $display("S0  3 灯 (红 76x60 / 黄 56x40 / 绿 46x34)");
        do_frame_start;
        feed_scene(3, 20, 250);
        do_frame_start;                       // 结算 -> 结果出现在 bxy0..bv
        chk_v(4'b0111);
        chk_box(0, 200,  40, 275,  99, 1);    // 面积最大的是红
        chk_box(1,  60, 150, 115, 189, 3);    // 黄
        chk_box(2, 400, 160, 445, 193, 2);    // 绿

        // ================= S1: 5 个灯, 只留最大的 4 个 =================
        $display("");
        $display("S1  5 灯, 期望丢掉最小的 E(26x18)");
        do_frame_start;
        feed_scene(5, 20, 290);
        do_frame_start;
        chk_v(4'b1111);
        chk_box(0, 200,  40, 275,  99, 1);    // A 4560 红
        chk_box(1,  60, 150, 115, 189, 3);    // B 2240 黄
        chk_box(2, 400, 150, 445, 183, 2);    // C 1564 绿
        chk_box(3, 300, 230, 335, 255, 1);    // D  936 红 (E 468 被挤掉)

        $display("");
        if (nfail == 0) $display("RESULT: PASS  (多框输出/排序/4 个上限 全部正确)");
        else            $display("RESULT: FAIL  (%0d 项不符)", nfail);
        $display("DONE");
        $finish;
    end

    initial begin
        #500000000;
        $display("RESULT: FAIL  (超时)");
        $display("DONE");
        $finish;
    end

endmodule
