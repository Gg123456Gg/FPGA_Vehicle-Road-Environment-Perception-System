//=====================================================================
// tb_fps_osd.v  ——  摄像头帧率 OSD 验证 (v41)
//---------------------------------------------------------------------
// 测两件事:
//   1) 计数: N 个 cam_vs 过后, 显示值必须是 N 的 BCD (十位/个位分开),
//      超过 99 要封顶在 99 而不是回绕成 00。
//   2) 画点: 按【规格】重算每个像素该是什么 (字符号 = u/12, 列 = (u%12)/2,
//      行 = (y-5)/2), 和 DUT 逐像素比对。
//      这里刻意用除法/取模写参考模型, 不照抄 RTL 的移位写法 ——
//      两边写法不同才能查出 fci/fcu 那套下标算错位的问题。
//
// 两个档位各扫一遍:
//   四宫格     右边界 504,  画面只有 512 宽
//   摄像头全屏 右边界 1016, 画面 1024 宽
//=====================================================================
`timescale 1ns/1ps
module tb_fps_osd;

    reg clk = 0, rst_n = 0;
    always #10 clk = ~clk;              // 50MHz

    reg        cam_vs = 0;
    reg        cam_full = 0;
    reg        src = 0;
    reg [9:0]  px_x = 0, px_y = 0;
    wire       osd_on;
    wire [15:0] osd_col;

    // 窗口取 400 拍, 便于仿真: 23 帧只要 46 拍, 不会跨窗口
    fps_osd #(.WIN_TICKS(26'd399)) u_dut(
        .clk      (clk      ),
        .rst_n    (rst_n    ),
        .cam_vs   (cam_vs   ),
        .cam_full (cam_full ),
        .src      (src      ),
        .px_x     (px_x     ),
        .px_y     (px_y     ),
        .osd_on   (osd_on   ),
        .osd_col  (osd_col  )
    );

    integer errs = 0;
    integer xx, yy;
    integer xr, xl;
    integer ci, cu, col, row, sel;
    reg [4:0] trow;
    reg       e_on;
    reg [15:0] e_col;
    integer bad;

    //------------------------------------------------------------------
    // 参考字模 (和 RTL 同一张 5x7 表, 但这里是"规格"的独立副本)
    //   ch: 0=F 1=P 2=S 3=空格 4..13='0'..'9'
    //------------------------------------------------------------------
    function [4:0] refg;
        input integer ch;
        input integer r;
        begin
            case (ch)
                0: case (r)  0: refg=5'h1F; 1: refg=5'h10; 2: refg=5'h10;
                             3: refg=5'h1E; default: refg=5'h10; endcase
                1: case (r)  0: refg=5'h1E; 1: refg=5'h11; 2: refg=5'h11;
                             3: refg=5'h1E; default: refg=5'h10; endcase
                2: case (r)  0: refg=5'h0E; 1: refg=5'h10; 2: refg=5'h10;
                             3: refg=5'h0E; 4: refg=5'h01; 5: refg=5'h01;
                             default: refg=5'h0E; endcase
                3: refg = 5'h00;
                4: case (r)  0: refg=5'h0E; 1: refg=5'h11; 2: refg=5'h11;
                             3: refg=5'h11; 4: refg=5'h11; 5: refg=5'h11;
                             default: refg=5'h0E; endcase
                5: case (r)  0: refg=5'h04; 1: refg=5'h0C; 2: refg=5'h04;
                             3: refg=5'h04; 4: refg=5'h04; 5: refg=5'h04;
                             default: refg=5'h0E; endcase
                6: case (r)  0: refg=5'h0E; 1: refg=5'h11; 2: refg=5'h01;
                             3: refg=5'h02; 4: refg=5'h04; 5: refg=5'h08;
                             default: refg=5'h1F; endcase
                7: case (r)  0: refg=5'h1F; 1: refg=5'h02; 2: refg=5'h04;
                             3: refg=5'h02; 4: refg=5'h01; 5: refg=5'h11;
                             default: refg=5'h0E; endcase
                8: case (r)  0: refg=5'h02; 1: refg=5'h06; 2: refg=5'h0A;
                             3: refg=5'h12; 4: refg=5'h1F; 5: refg=5'h02;
                             default: refg=5'h02; endcase
                9: case (r)  0: refg=5'h1F; 1: refg=5'h10; 2: refg=5'h1E;
                             3: refg=5'h01; 4: refg=5'h01; 5: refg=5'h11;
                             default: refg=5'h0E; endcase
                10: case (r) 0: refg=5'h06; 1: refg=5'h08; 2: refg=5'h10;
                             3: refg=5'h1E; 4: refg=5'h11; 5: refg=5'h11;
                             default: refg=5'h0E; endcase
                11: case (r) 0: refg=5'h1F; 1: refg=5'h01; 2: refg=5'h02;
                             3: refg=5'h04; default: refg=5'h08; endcase
                12: case (r) 0: refg=5'h0E; 1: refg=5'h11; 2: refg=5'h11;
                             3: refg=5'h0E; 4: refg=5'h11; 5: refg=5'h11;
                             default: refg=5'h0E; endcase
                default: case (r) 0: refg=5'h0E; 1: refg=5'h11; 2: refg=5'h11;
                             3: refg=5'h0F; 4: refg=5'h01; 5: refg=5'h02;
                             default: refg=5'h0C; endcase
            endcase
        end
    endfunction

    //------------------------------------------------------------------
    // 任务
    //------------------------------------------------------------------
    task vs;                            // 打 n 个 cam_vs 脉冲 (各 1 拍)
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) begin
                @(posedge clk); #1 cam_vs = 1'b1;
                @(posedge clk); #1 cam_vs = 1'b0;
            end
        end
    endtask

    task wait_cycles;
        input integer n;
        integer k;
        begin
            for (k = 0; k < n; k = k + 1) @(posedge clk);
        end
    endtask

    task chk_cnt;                       // 检查 BCD 显示值
        input integer want_t, want_o;
        input [8*24:1] name;
        begin
            if ((u_dut.d_t !== want_t) || (u_dut.d_o !== want_o)) begin
                $display("  *** FAIL cnt %0s: shown %0d%0d  want %0d%0d",
                         name, u_dut.d_t, u_dut.d_o, want_t, want_o);
                errs = errs + 1;
            end else
                $display("  OK   cnt %0s: FPS=%0d%0d", name, u_dut.d_t, u_dut.d_o);
        end
    endtask

    // 扫一遍整个框, 逐像素比对 (按规格重算, 不照抄 RTL)
    task scan;
        input integer xright;
        input [8*24:1] name;
        begin
            bad = 0;
            xl  = xright - 71;
            for (yy = 0; yy < 24; yy = yy + 1) begin
                for (xx = xright - 80; xx <= xright + 8; xx = xx + 1) begin
                    px_x = xx[9:0];
                    px_y = yy[9:0];
                    #1;
                    // ---- 规格: 黑底 x in [xl-3, xr+3], y in [2,19]
                    e_on  = (xx >= xl - 3) && (xx <= xright + 3) &&
                            (yy >= 2) && (yy <= 19);
                    e_col = 16'h0000;
                    // ---- 字身: x in [xl, xr], y in [5,18]
                    if ((xx >= xl) && (xx <= xright) && (yy >= 5) && (yy <= 18)) begin
                        ci  = (xx - xl) / 12;             // 第几个字符 0..5
                        cu  = (xx - xl) % 12;             // 字符内列 0..11
                        row = (yy - 5) / 2;               // 字模行 0..6
                        if      (ci == 0) sel = 0;                     // F
                        else if (ci == 1) sel = 1;                     // P
                        else if (ci == 2) sel = 2;                     // S
                        else if (ci == 3) sel = 3;                     // 空格
                        else if (ci == 4) sel = 4 + u_dut.d_t;         // 十位
                        else              sel = 4 + u_dut.d_o;         // 个位
                        trow = refg(sel, row);
                        if (cu < 10) begin
                            col = cu / 2;                              // 0..4
                            if (trow[4 - col]) e_col = 16'hFFE0;       // 字 = 黄
                        end
                    end
                    if ((osd_on !== e_on) || (e_on && (osd_col !== e_col))) begin
                        if (bad < 8)
                            $display("  *** FAIL px(%0d,%0d) got on=%b col=%h  want on=%b col=%h",
                                     xx, yy, osd_on, osd_col, e_on, e_col);
                        bad = bad + 1;
                    end
                end
            end
            if (bad == 0)
                $display("  OK   scan %0s: pixel-exact", name);
            else begin
                $display("  *** FAIL scan %0s: %0d pixels differ", name, bad);
                errs = errs + 1;
            end
        end
    endtask

    //------------------------------------------------------------------
    // 主流程
    //------------------------------------------------------------------
    initial begin
        $display("==== fps_osd ====");
        rst_n = 0;
        wait_cycles(5);
        rst_n = 1;
        wait_cycles(2);

        // ---- 1) 23 帧 -> 显示 23 ----
        vs(23);
        wait_cycles(420);            // 等窗口翻一次
        chk_cnt(2, 3, "23 frames");

        // ---- 2) 逐像素扫描, 两个档位 ----
        cam_full = 1'b0;             // 四宫格: 右边界 504
        scan(504, "quad 512x300");
        cam_full = 1'b1;             // 摄像头全屏: 右边界 1016
        scan(1016, "camfull 1024x600");

        // ---- 3) src=1 (PC 像素) 一个点都不许画 ----
        src = 1'b1;
        px_x = 10'd500; px_y = 10'd10;
        #1;
        if (osd_on !== 1'b0) begin
            $display("  *** FAIL src=1 still drawn (on=%b)", osd_on);
            errs = errs + 1;
        end else
            $display("  OK   src=1 not drawn");
        src = 1'b0;

        // ---- 4) 超过 99 帧封顶在 99, 不回绕 ----
        rst_n = 0; wait_cycles(5); rst_n = 1;
        vs(105);
        wait_cycles(420);
        chk_cnt(9, 9, "105 frames -> 99 cap");

        // ---- 5) 复位后立刻显示 00 ----
        rst_n = 0; wait_cycles(5); rst_n = 1;
        wait_cycles(2);
        chk_cnt(0, 0, "reset -> 00");

        if (errs == 0) $display("FPS_OSD_PASS");
        else           $display("FPS_OSD_FAIL errs=%0d", errs);
        $finish;
    end

endmodule
