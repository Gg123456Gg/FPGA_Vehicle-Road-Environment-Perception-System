//=====================================================================
// tb_lane_reacq.v  ——  "急弯跟丢 -> 自动放宽窗口重捕" 专项用例
//---------------------------------------------------------------------
// 这条路径 (WIN_LOST / missf>=2 / SEED 复位) 之前只有静态代码,
// 没有任何用例覆盖, 是文档 36.10 里写明的已知空白。补上。
//
// 场景(左线整体左移 100 像素 = 急弯把线甩出 ±64 跟踪窗):
//
//   f=0..2  双线, 左线在 384->185   -> 锁定, ltyp=1(实线), lpos≈185
//   f=3..4  左线瞬间左移 100        -> 跟丢: lpos 无效, ltyp=0,
//                                      且**不许**因此报"变道"
//   f=5..6  线不动                  -> missf 攒到 2, 窗口放宽到 192
//                                      并用 SEED_L 重捕, 重新锁上 lpos≈85
//
// 关键在 f=5: 窗口是从 SEED_L(384) 起算的 192 宽 = 192..576。
// 线在 284, 落在窗口内 -> 第一行就命中, 之后逐行跟下去。
//
// 反向对照: 编译时 +define+RC_NARROW 把 WIN_LOST 改回 64(等于没有放宽
// 机制), 必须报 FAIL —— 否则这个用例什么也没证明。
//=====================================================================
`timescale 1ns / 1ps
module tb_lane_reacq;

    reg         clk   = 1'b0;
    reg         rst_n = 1'b0;
    reg         vsync = 1'b0;
    reg         px_en = 1'b0;
    reg  [15:0] px565 = 16'h0000;

    wire [1:0]  ltyp, rtyp;
    wire [10:0] lpos, rpos, offv;
    wire [1:0]  offl;
    wire [2:0]  lstate;
    wire [1:0]  cdir;
    wire [87:0] anch;

`ifdef RC_NARROW
    localparam [9:0] TB_WIN_LOST = 10'd64;
`else
    localparam [9:0] TB_WIN_LOST = 10'd192;
`endif

    // v34: 行号改成外部输入了。这里是连续光栅场景, 用本地行计数喂进去,
    //   行为和原来模块内部自数完全一致(line_st 恒 0, 不触发重对齐)。
    reg [10:0] colc = 11'd0;
    reg [9:0]  rowc = 10'd0;
    always @(posedge clk) begin
        if (vsync) begin colc <= 11'd0; rowc <= 10'd0; end
        else if (px_en) begin
            if (colc == 11'd1023) begin colc <= 11'd0; rowc <= rowc + 10'd1; end
            else colc <= colc + 11'd1;
        end
    end

        lane_detect #(.WIN_LOST(TB_WIN_LOST)) u_dut (
        .clk (clk), .rst_n (rst_n), .vsync (vsync), .px_en (px_en), .px565  (px565 ),
        .line_st (1'b0  ),
        .px_row  (rowc  ),
        .ltyp (ltyp), .rtyp (rtyp), .lpos (lpos), .rpos (rpos), .offv (offv),
        .offl (offl), .lstate (lstate), .cdir (cdir), .anch (anch)
    );

    always #10 clk = ~clk;

    localparam integer NF = 7;
    localparam integer H  = 1024;
    localparam integer V  = 600;
    localparam integer SHIFT = -100;   // f>=3 时左线的左移量

    integer nx, ny, nf, gv, lx, rx, sh;
    integer errs = 0;

    // f>=3 左线左移
    function integer lshift;
        input integer f;
        begin
            lshift = (f >= 3) ? SHIFT : 0;
        end
    endfunction

    task feed_frame;
        integer s;
        begin
            s = lshift(nf);
            for (ny = 0; ny < V; ny = ny + 1) begin
                for (nx = 0; nx < H; nx = nx + 1) begin
                    // 背景: 暗灰 + 伪随机噪点
                    gv = 6'd10 + ((nx * 7 + ny * 3) % 5);
                    if ((ny >= 360) && (ny <= 599)) begin
                        lx = 384 - ((ny - 360) * 200) / 240 + s;
                        rx = 640 + ((ny - 360) * 204) / 240;
                        if ((nx >= lx - 4) && (nx <= lx + 4)) gv = 6'd55;
                        if ((nx >= rx - 4) && (nx <= rx + 4)) gv = 6'd55;
                    end
                    px565 = gv[5:0] << 5;
                    px_en = 1'b1;
                    @(posedge clk);
                end
            end
            px_en = 1'b0;
            repeat (4) @(posedge clk);
        end
    endtask

    task chk;
        input integer f;
        begin
            $display("# [f=%0d] ltyp=%0d rtyp=%0d offl=%b lpos=%0d(v=%b) state=%0d off=%0d",
                     f, ltyp, rtyp, offl, lpos[9:0], lpos[10], lstate, $signed(offv));
`ifdef DBG
            $display("    dbg: missf=%0d win=%0d pred=%0d seed=%0d hitsL=%0d cmaxL=%0d",
                     u_dut.missf_l, u_dut.win_l, u_dut.pred_l,
                     u_dut.seed_l, u_dut.hits_l, u_dut.cmax_l);
`endif

            // 任何一帧都不许报"变道"(这条线只是瞬移, 不是真的在变道)
            if (lstate === 3'd3) begin
                $display("  *** FAIL: 不该报变道 state=3");
                errs = errs + 1;
            end

            if (f <= 2) begin
                // 跟丢前: 左线锁在 185 附近
                if (lpos[10] !== 1'b1) begin
                    $display("  *** FAIL: 跟丢前左线应有效");
                    errs = errs + 1;
                end else if ((lpos[9:0] > 11'd193) || (lpos[9:0] < 11'd177)) begin
                    $display("  *** FAIL: 跟丢前左线应在 185±8, 实际 %0d", lpos[9:0]);
                    errs = errs + 1;
                end
                if (ltyp !== 2'd1) begin
                    $display("  *** FAIL: 跟丢前应判实线, 实际 %0d", ltyp);
                    errs = errs + 1;
                end
                if (offl !== 2'b11) begin
                    $display("  *** FAIL: 跟丢前两线都该有效 offl=%b", offl);
                    errs = errs + 1;
                end
            end else if (f <= 4) begin
                // 跟丢期间: 必须老实说"没看见", 不能拿旧值凑
                if (lpos[10] !== 1'b0) begin
                    $display("  *** FAIL: 跟丢期间左线必须无效, 实际 lpos=%0d", lpos[9:0]);
                    errs = errs + 1;
                end
                if (ltyp !== 2'd0) begin
                    $display("  *** FAIL: 跟丢期间应判不出(0), 实际 %0d", ltyp);
                    errs = errs + 1;
                end
            end else begin
                // 重捕后: 左线在 85 附近, 且重新判得出实线
                if (lpos[10] !== 1'b1) begin
                    $display("  *** FAIL: 应已重捕(左线有效), 实际无效");
                    errs = errs + 1;
                end else if ((lpos[9:0] > 11'd93) || (lpos[9:0] < 11'd77)) begin
                    $display("  *** FAIL: 重捕位置应为 85±8, 实际 %0d", lpos[9:0]);
                    errs = errs + 1;
                end
                if (ltyp !== 2'd1) begin
                    $display("  *** FAIL: 重捕后应恢复实线判定, 实际 %0d", ltyp);
                    errs = errs + 1;
                end
                if (offl !== 2'b11) begin
                    $display("  *** FAIL: 重捕后两线都该有效 offl=%b", offl);
                    errs = errs + 1;
                end
            end
        end
    endtask

    initial begin
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        for (nf = 0; nf < NF; nf = nf + 1) begin
            vsync = 1'b1;
            repeat (3) @(posedge clk);
            vsync = 1'b0;
            repeat (4) @(posedge clk);

            if (nf > 0) chk(nf - 1);
            feed_frame;
        end

        vsync = 1'b1;
        repeat (3) @(posedge clk);
        vsync = 1'b0;
        repeat (4) @(posedge clk);
        chk(NF - 1);

`ifdef RC_NARROW
        $display("# REACQ_RC(窗口不放宽) errs=%0d  <- 大于 0 才是对的", errs);
`else
        if (errs == 0) $display("# LANE_REACQ_PASS");
        else           $display("# LANE_REACQ_FAIL: %0d 项不符", errs);
`endif
        $finish;
    end

endmodule
