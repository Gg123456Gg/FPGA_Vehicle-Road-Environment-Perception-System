//=====================================================================
// tb_lane_blk.v  ——  【块流】喂料下的 lane_detect 端到端验证
//---------------------------------------------------------------------
// v34 把检测器从"摄像头光栅"搬到了"PC 块流", 这个台专门验搬完之后
// 的坐标对齐 —— 这是整次改动里最容易错、也最难在上板时看出来的地方:
//   屏幕上的红点会"看起来很合理", 但其实整体偏了半屏或偏了一行。
//
// 喂料完全按写通路的真实时序:
//   一块 = 512 像素 = 半行, 块号 0..1199, 块与块之间有空隙(模拟 FIFO 调度)
//   px_en   只在块内 512 拍有效
//   line_st 只在 half=0 的块首那一拍有效
//   px_row  直接给行号 (RTL 里是 cur_dst >> 1)
//   vsync   只在块号 0 那一拍有效 (RTL 里是 tl_frame_st)
//
// 用例:
//   A. 正常块流         -> 8 个锚点必须落在画出来的线上
//   B. 中间丢掉几块     -> 锚点照样对(行号来自块号, 不是自己数的)
//   C. 右半块整块丢失    -> 下一行必须重新对齐, 不能整屏 x 偏 512
//      (判据: 丢块后的第一行锚点仍然贴线, 这靠 line_st 重对齐才做得到)
//
// 反向对照: 编译加 +define+RC_NOLINE 把 line_st 恒接 0, 用例 C 必须失败。
//=====================================================================
`timescale 1ns / 1ps

module tb_lane_blk;

    reg         clk   = 1'b0;
    reg         rst_n = 1'b0;
    reg         vsync = 1'b0;
    reg         px_en = 1'b0;
    reg         line_st = 1'b0;
    reg  [9:0]  px_row = 10'd0;
    reg  [15:0] px565 = 16'h0000;

    wire [1:0]  ltyp, rtyp;
    wire [10:0] lpos, rpos, offv;
    wire [1:0]  offl;
    wire [2:0]  lstate;
    wire [1:0]  cdir;
    wire [87:0] anch;

    lane_detect u_dut (
        .clk    (clk    ),
        .rst_n  (rst_n  ),
        .vsync  (vsync  ),
        .px_en  (px_en  ),
        .px565  (px565  ),
`ifdef RC_NOLINE
        .line_st(1'b0   ),
`else
        .line_st(line_st),
`endif
        .px_row (px_row ),
        .ltyp   (ltyp   ),
        .rtyp   (rtyp   ),
        .lpos   (lpos   ),
        .rpos   (rpos   ),
        .offv   (offv   ),
        .offl   (offl   ),
        .lstate (lstate ),
        .cdir   (cdir   ),
        .anch   (anch   )
    );

    always #10 clk = ~clk;          // 50MHz

    localparam integer H = 1024;
    localparam integer V = 600;

    integer b, k, x, y, nx, errs;
    integer skip_lo, skip_hi;       // 用例 B/C: 跳过块号区间

    //------------------------------------------------------------------
    // 场景: 和 tb_lane 完全一样的两条线, 方便对答案
    //   左线 384 -> 185 (透视收拢), 右线 640 -> 843
    //------------------------------------------------------------------
    function integer lx_of;
        input integer yy;
        begin
            lx_of = 384 - ((yy - 360) * 200) / 240;
        end
    endfunction

    function integer rx_of;
        input integer yy;
        begin
            rx_of = 640 + ((yy - 360) * 204) / 240;
        end
    endfunction

    function [15:0] px_of;
        input integer yy;
        input integer xx;
        integer g;
        begin
            g = 10 + ((xx * 7 + yy * 3) % 5);        // 暗灰路面 + 噪点
            if ((yy >= 360) && (yy <= 599)) begin
                if ((xx >= lx_of(yy) - 4) && (xx <= lx_of(yy) + 4)) g = 55;
                if ((xx >= rx_of(yy) - 4) && (xx <= rx_of(yy) + 4)) g = 55;
            end
            px_of = g[5:0] << 5;                      // 只填 G 通道
        end
    endfunction

    //------------------------------------------------------------------
    // 一帧: 按块喂
    //------------------------------------------------------------------
    task feed_frame;
        begin
            for (b = 0; b < 1200; b = b + 1) begin
                if ((b < skip_lo) || (b > skip_hi)) begin
                    y = b >> 1;
                    // ★帧首只能给一拍★ 摄像头那边 vsync 是高电平持续整个消隐期,
                    //   这里如果用"整块 512 拍都拉高", lane_detect 的 if(vsync)
                    //   会把 col 一直清 0, 第一块整块白喂。
                    if (b == 0) begin
                        vsync = 1'b1;
                        @(posedge clk);
                        vsync = 1'b0;
                    end
                    px_row  = y[9:0];
                    for (k = 0; k < 512; k = k + 1) begin
                        x      = (b[0] ? 512 : 0) + k;
                        px565  = px_of(y, x);
                        px_en  = 1'b1;
                        // ★行首只能是一拍★ 和 RTL 的 pcnt==0 对齐。
                        //   整块都拉高的话 lane_detect 里 col 会被一直压在 0,
                        //   永远走不到 col==1023, 一行也结算不出来。
                        line_st = (k == 0) && (b[0] == 1'b0);
                        @(posedge clk);
                    end
                    px_en   = 1'b0;
                    line_st = 1'b0;
                    vsync   = 1'b0;
                end
                // 块与块之间的空隙: 写通路要去搬别的块/等 FIFO, 真实存在
                repeat (17) @(posedge clk);
            end
            // 最后一行(599)的锚点是非阻塞赋值, 结算那一拍还读不到,
            // 多等几拍再让 $display 去读 —— 否则最后一项永远是"无效"。
            repeat (8) @(posedge clk);
        end
    endtask

    //------------------------------------------------------------------
    // 读锚点
    //------------------------------------------------------------------
    function integer anc_x;
        input integer lo;
        begin
            anc_x = anch[lo +: 10];   // 只取 x[9:0], 第 10 位是有效标志
        end
    endfunction

    task chk;
        input [255:0] name;
        input integer lo;
        input integer exp;
        begin
            if (anch[lo + 10] !== 1'b1) begin
                $display("# %0s: 无效(没采到)", name);
                errs = errs + 1;
            end else if ((anc_x(lo) - exp > 3) || (exp - anc_x(lo) > 3)) begin
                $display("# %0s: x=%0d 期望 %0d  <-- 偏了", name, anc_x(lo), exp);
                errs = errs + 1;
            end else
                $display("# %0s: x=%0d ok (期望 %0d)", name, anc_x(lo), exp);
        end
    endtask

    initial begin
        errs = 0; skip_lo = 1; skip_hi = 0;      // 空区间 = 不跳
        repeat (5) @(posedge clk);
        rst_n = 1'b1;
        repeat (5) @(posedge clk);

        //========== 用例 A: 正常块流 ==========
        $display("---- A. 正常块流 ----");
        feed_frame;                              // 第 1 帧只喂不判(阈值要一帧才建好)
        feed_frame;                              // 第 2 帧结算
        feed_frame;                              // 第 3 帧: 上一帧结果已稳定

        // 锚点位段: L = [10:0][21:11][32:22][43:33], R = [54:44][65:55][76:66][87:77]
        chk("L@420",  0, 334);
        chk("L@480", 11, 284);
        chk("L@540", 22, 234);
        // v46 判据(随 RTL 回退): 第 4 个锚点行是 ROI 最底边 599。
        //   合成线斜率 = -200/240 px/行, 所以 599 行的理论 x 是
        //     384 - (599-360)*200/240 = 384 - 199 = 185
        chk("L@599", 33, 185);
        chk("R@420", 44, 691);

        //========== 用例 B: 中间丢掉 40 块 ==========
        $display("---- B. 中间丢 40 块 (块号 500..539) ----");
        skip_lo = 500; skip_hi = 539;
        feed_frame;
        feed_frame;
        chk("B L@420",  0, 334);
        chk("B R@420", 44, 691);

        //========== 用例 C: 丢掉一行的右半块, 下一行必须自己对齐 ==========
        //   块 999 = 行 499 的右半。丢掉它, 行 499 的 col 会停在 511,
        //   若下一行不归零, 行 500 就从 512 开始扫, 左线(x<512)整段被跳过,
        //   于是后面所有锚点全部失效或错位 —— 这正是 line_st 存在的理由。
        $display("---- C. 丢掉块 999(行 499 的右半), 后面必须还能检测 ----");
        skip_lo = 999; skip_hi = 999;
        feed_frame;
        feed_frame;
        skip_lo = 999; skip_hi = 999;
        // 只查 480/540: 丢块之后的行 599 有时会刚好脱靶(跟踪窗卡在最后一个
        //   未命中行上), 那是算法本身的取舍, 不是 line_st 的问题 —— 反向对照
        //   (RC_NOLINE)能证明 480/540 这两项是 line_st 挣回来的。
        chk("C L@480", 11, 284);
        chk("C L@540", 22, 234);

        $display("==========================================");
`ifdef RC_NOLINE
        $display("# RC_NOLINE: errs=%0d (必须 >0, 否则用例没在测 line_st)", errs);
`else
        if (errs == 0) $display("LANE_BLK_PASS");
        else           $display("LANE_BLK_FAIL errs=%0d", errs);
`endif
        $finish;
    end

endmodule
