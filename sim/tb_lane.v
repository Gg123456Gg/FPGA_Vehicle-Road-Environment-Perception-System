//=====================================================================
// tb_lane.v  ——  lane_detect 的合成场景验证
//---------------------------------------------------------------------
// 造 11 帧 1024x600 的假路面, 每帧按场景画两条线:
//   f=0..2   实线, 车道居中
//   f=3..5   虚线 (行方向 40 行一段 / 40 行一空), 居中
//   f=6..8   实线, 车道整体左移 40/80/120 像素 (= 本车向右变道)
// 判据:
//   实线帧 ltyp/rtyp 必须是 1, 虚线帧必须是 2
//   居中帧 lstate 必须是 0, f>=7 必须是 3 (变道中)
//   偏移量必须跟着 SHIFT 走
//
// ★vsync 每帧只能给一次★ 给两次会让累加器在空统计上再结算一次, 结果被清成 0。
//   所以结构是: 帧首给 vsync -> 这时上一帧的结果才出来 -> 先判上一帧 -> 再喂数据
//=====================================================================
`timescale 1ns / 1ps

module tb_lane;

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
        .clk    (clk   ),
        .rst_n  (rst_n ),
        .vsync  (vsync ),
        .px_en  (px_en ),
        .px565  (px565 ),
        .line_st (1'b0  ),
        .px_row  (rowc  ),
        .ltyp   (ltyp  ),
        .rtyp   (rtyp  ),
        .lpos   (lpos  ),
        .rpos   (rpos  ),
        .offv   (offv  ),
        .offl   (offl  ),
        .lstate (lstate),
        .cdir   (cdir  ),
        .anch   (anch  )
    );

    always #10 clk = ~clk;          // 20ns = 50MHz, 和 cam_pclk 同量级

    localparam integer NF = 10;
    localparam integer H  = 1024;
    localparam integer V  = 600;

    integer nx, ny, nf;
    integer SHIFT;
    integer MODE;                   // 0=实线 1=虚线
    integer lx, rx, gv;
    integer errs = 0;

    //------------------------------------------------------------------
    // 场景表
    //------------------------------------------------------------------
    function integer scen_shift;
        input integer f;
        begin
            if (f < 6) scen_shift = 0;
            else       scen_shift = -(f - 5) * 40;
        end
    endfunction

    function integer scen_mode;
        input integer f;
        begin
            scen_mode = ((f >= 3) && (f < 6)) ? 1 : 0;
        end
    endfunction

    //------------------------------------------------------------------
    // 一帧像素
    //------------------------------------------------------------------
    task feed_frame;
        begin
            for (ny = 0; ny < V; ny = ny + 1) begin
                for (nx = 0; nx < H; nx = nx + 1) begin
                    // 背景: 暗灰 + 一点伪随机噪点 (gv 10..14 -> g8 40..56)
                    gv = 6'd10 + ((nx * 7 + ny * 3) % 5);
                    if ((ny >= 360) && (ny <= 599)) begin
                        lx = 384 - ((ny - 360) * 200) / 240 + SHIFT;
                        rx = 640 + ((ny - 360) * 204) / 240 + SHIFT;
                        if ((nx >= lx - 4) && (nx <= lx + 4)) gv = 6'd55;   // 左线
                        if ((nx >= rx - 4) && (nx <= rx + 4)) gv = 6'd55;   // 右线
                        if (MODE == 1 && (((ny - 360) % 80) >= 40)) begin
                            // 虚线: 这段是"空"
                            if ((nx >= lx - 4) && (nx <= lx + 4)) gv = 6'd12;
                            if ((nx >= rx - 4) && (nx <= rx + 4)) gv = 6'd12;
                        end
                    end
                    px565 = gv[5:0] << 5;        // 只填 G 通道, 和真摄像头一致
                    px_en = 1'b1;
                    @(posedge clk);
                end
            end
            px_en = 1'b0;
            repeat (4) @(posedge clk);
        end
    endtask

    //------------------------------------------------------------------
    // 检查一帧结果
    //------------------------------------------------------------------
    task chk_frame;
        input integer f;
        integer exp_typ, exp_st;
        begin
            exp_typ = (scen_mode(f) == 1) ? 2 : 1;
            // v46 判据(随 RTL 回退): 单帧变化量超 CHG_TH 即判变道。
            //   合成激励从 f=6 起每帧左移 40px, 8 帧窗口内累计 > 56,
            //   所以 f>=7 必须报 state=3。
            if (f < 7) exp_st = 0;
            else       exp_st = 3;

            $display("# [f=%0d] ltyp=%0d rtyp=%0d offl=%b lx=%0d rx=%0d off=%0d state=%0d cdir=%0d  (want typ=%0d state=%0d)",
                     f, ltyp, rtyp, offl, lpos[9:0], rpos[9:0],
                     $signed(offv), lstate, cdir, exp_typ, exp_st);

            if (ltyp !== exp_typ[1:0]) begin
                $display("  *** FAIL: 左线线型 %0d != %0d", ltyp, exp_typ); errs = errs + 1;
            end
            if (rtyp !== exp_typ[1:0]) begin
                $display("  *** FAIL: 右线线型 %0d != %0d", rtyp, exp_typ); errs = errs + 1;
            end
            if (lstate !== exp_st[2:0]) begin
                $display("  *** FAIL: 状态 %0d != %0d", lstate, exp_st); errs = errs + 1;
            end
            // 偏移量方向: 只有变道帧才有意义
            if (f >= 7) begin
                if ($signed(offv) > -50) begin
                    $display("  *** FAIL: 变道时偏移应该明显为负, 实际 %0d", $signed(offv)); errs = errs + 1;
                end
            end
            $display("        anchor L420=%0d L480=%0d L540=%0d L599=%0d  R420=%0d R480=%0d R540=%0d R599=%0d",
                     anch[9:0],  anch[20:11], anch[31:22], anch[42:33],
                     anch[53:44], anch[64:55], anch[75:66], anch[86:77]);
            // 两条线都必须有效, 否则"偏移"是靠补出来的, 不是真看见的
            if (offl !== 2'b11) begin
                $display("  *** FAIL: 两条线没同时有效 offl=%b", offl); errs = errs + 1;
            end
        end
    endtask

    //------------------------------------------------------------------
    initial begin
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        for (nf = 0; nf < NF; nf = nf + 1) begin
            SHIFT = scen_shift(nf);
            MODE  = scen_mode(nf);

            // 帧首 vsync: 上一帧的结果在这里结算出来
            vsync = 1'b1;
            repeat (3) @(posedge clk);
            vsync = 1'b0;
            repeat (4) @(posedge clk);

            if (nf > 0) chk_frame(nf - 1);

            feed_frame;
        end

        // 最后一帧: 再给一次 vsync 把它的结果结算出来
        vsync = 1'b1;
        repeat (3) @(posedge clk);
        vsync = 1'b0;
        repeat (4) @(posedge clk);
        chk_frame(NF - 1);

        if (errs == 0) $display("# LANE_ALL_PASS");
        else           $display("# LANE_FAIL: %0d 项不符", errs);
        $finish;
    end

endmodule
