//=====================================================================
// tb_lane_robust.v  ——  lane_detect 的"不理想输入"验证
//---------------------------------------------------------------------
// tb_lane 用的是理想路面(黑底白线)。真摄像头上最先撞到的三类烂输入:
//
//   ① 只看得见一条线  —— 变道过程中正被跨越的那条会跑出画面。这时候
//      必须用上次学到的车道宽度把另一条补出来, 否则偏移直接失效,
//      而"变道判断"恰恰最依赖它。
//   ② 低对比度        —— 进隧道/逆光/夜里, 整幅画面发灰, 没有"白线"。
//      正确行为是判不出(ltyp=0 / state=7), 屏幕上是白色色块 + HOLD,
//      而不是随便指一条线乱报"实线/变道"。
//   ③ 路面有亮斑      —— 井盖/水渍/斑马线残影, 会被当成候选。判据是
//      "不许因此报变道"(state 不能变 3)。
//
// 场景表:
//   f=0..2  双线实线居中      -> typ=1/1, offl=11, state=0   (建立 lane_w)
//   f=3..5  只剩右线          -> rtyp=1,  offl=01, state=0   (靠宽度补)
//   f=6..8  低对比度(无线)    -> typ=0/0, state=7            (判不出)
//   f=9..11 恢复双线 + 亮斑   -> typ=1/1, state 不能是 3
//=====================================================================
`timescale 1ns / 1ps
module tb_lane_robust;

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

        lane_detect u_dut (
        .clk (clk), .rst_n (rst_n), .vsync (vsync), .px_en (px_en), .px565  (px565 ),
        .line_st (1'b0  ),
        .px_row  (rowc  ),
        .ltyp (ltyp), .rtyp (rtyp), .lpos (lpos), .rpos (rpos), .offv (offv),
        .offl (offl), .lstate (lstate), .cdir (cdir), .anch (anch)
    );

    always #10 clk = ~clk;

    localparam integer NF = 12;
    localparam integer H  = 1024;
    localparam integer V  = 600;

    integer nx, ny, nf, gv, lx, rx;
    integer errs = 0;

    // 场景编号
    function integer scn;
        input integer f;
        begin
            if (f < 3)       scn = 0;   // 双线
            else if (f < 6)  scn = 1;   // 只有右线
            else if (f < 9)  scn = 2;   // 低对比度
            else             scn = 3;   // 双线 + 亮斑
        end
    endfunction

    task feed_frame;
        integer s;
        begin
            s = scn(nf);
            for (ny = 0; ny < V; ny = ny + 1) begin
                for (nx = 0; nx < H; nx = nx + 1) begin
                    // 背景: 暗灰 + 伪随机噪点
                    gv = 6'd10 + ((nx * 7 + ny * 3) % 5);
                    if ((ny >= 360) && (ny <= 599)) begin
                        lx = 384 - ((ny - 360) * 200) / 240;
                        rx = 640 + ((ny - 360) * 204) / 240;

                        if (s == 2) begin
                            // 低对比度: 全画面只有 10/11/12 三档, 没有白线
                            gv = 6'd10 + ((nx + ny) % 3);
                        end else begin
                            // 左线: 单线场景(s==1)里画到画面外
                            if ((s != 1) && (nx >= lx - 4) && (nx <= lx + 4))
                                gv = 6'd55;
                            if ((nx >= rx - 4) && (nx <= rx + 4))
                                gv = 6'd55;
                            // 亮斑: 路面上几块孤立亮区(宽 6, 高 4)
                            if ((s == 3) && (nx >= 200) && (nx <= 205) &&
                                (ny >= 460) && (ny <= 463))
                                gv = 6'd55;
                            if ((s == 3) && (nx >= 880) && (nx <= 885) &&
                                (ny >= 520) && (ny <= 523))
                                gv = 6'd55;
                        end
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
        integer s;
        begin
            s = scn(f);
            $display("# [f=%0d scn=%0d] ltyp=%0d rtyp=%0d offl=%b off=%0d state=%0d",
                     f, s, ltyp, rtyp, offl, $signed(offv), lstate);

            if (s == 0 || s == 3) begin
                if (ltyp !== 2'd1 || rtyp !== 2'd1) begin
                    $display("  *** FAIL: 双线实线应判 1/1, 实际 %0d/%0d", ltyp, rtyp);
                    errs = errs + 1;
                end
                if (offl !== 2'b11) begin
                    $display("  *** FAIL: 两条线都该有效, offl=%b", offl);
                    errs = errs + 1;
                end
                if (lstate !== 3'd0) begin
                    $display("  *** FAIL: 居中应为 state=0, 实际 %0d", lstate);
                    errs = errs + 1;
                end
            end

            if (s == 1) begin
                // 只有右线: 左无效右有效, 但偏移必须还能算(用上次宽度补)
                if (rtyp !== 2'd1) begin
                    $display("  *** FAIL: 右线应仍判实线, 实际 %0d", rtyp);
                    errs = errs + 1;
                end
                if (offl !== 2'b01) begin
                    $display("  *** FAIL: 只该右线有效 offl=01, 实际 %b", offl);
                    errs = errs + 1;
                end
                if (lstate !== 3'd0) begin
                    $display("  *** FAIL: 单线时靠宽度补, 仍应居中 state=0, 实际 %0d", lstate);
                    errs = errs + 1;
                end
                if (($signed(offv) > 40) || ($signed(offv) < -40)) begin
                    $display("  *** FAIL: 单线补出来的偏移应接近 0, 实际 %0d", $signed(offv));
                    errs = errs + 1;
                end
            end

            if (s == 2) begin
                // 低对比度: 必须"判不出", 不能乱指
                if (ltyp !== 2'd0 || rtyp !== 2'd0) begin
                    $display("  *** FAIL: 低对比度应判不出(0/0), 实际 %0d/%0d", ltyp, rtyp);
                    errs = errs + 1;
                end
                if (lstate !== 3'd7) begin
                    $display("  *** FAIL: 低对比度应 state=7, 实际 %0d", lstate);
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

        if (errs == 0) $display("# LANE_ROBUST_PASS");
        else           $display("# LANE_ROBUST_FAIL: %0d 项不符", errs);
        $finish;
    end

endmodule
