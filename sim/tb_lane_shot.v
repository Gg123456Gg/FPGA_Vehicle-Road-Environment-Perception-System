//=====================================================================
// tb_lane_shot.v  ——  把 lane_detect 的"识别结果"渲染成一幅图
//---------------------------------------------------------------------
// 目的: 前面所有测试台都只打印数字(ltyp=1 off=-3 state=0), 人看不出
//       "红点到底贴没贴在线上"。这个台把同一帧的像素 dump 到
//       lane_shot.hex, 同时把 anch 的 8 个锚点打出来, 交给
//       render_lane_shot.py 画成 PNG —— 肉眼直接判"贴没贴线"。
//
// 场景: 双线实线居中 (和 tb_lane_robust 的 scn=0 完全一致)
//       左线 y=360..599 从 x=384 斜到 185
//       右线 y=360..599 从 x=640 斜到 843
//       线宽 9, 亮度 gv=55 (-> g8=223), 路面 gv=10..14 (-> g8=40..44)
//
// 用法:
//   vlog -work work_shot ../rtl/eth_ddr3_lcd.v tb_lane_shot.v
//   vsim -c -work work_shot tb_lane_shot -do "run -all; quit -f"
//   python render_lane_shot.py
//=====================================================================
`timescale 1ns / 1ps
module tb_lane_shot;

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

    localparam integer H = 1024;
    localparam integer V = 600;

    integer nx, ny, nf, gv, lx, rx;
    integer fh;

    task feed_frame;
        input integer dump;
        begin
            if (dump) fh = $fopen("lane_shot.hex", "w");
            for (ny = 0; ny < V; ny = ny + 1) begin
                for (nx = 0; nx < H; nx = nx + 1) begin
                    gv = 6'd10 + ((nx * 7 + ny * 3) % 5);
                    if (ny >= 360) begin
                        lx = 384 - ((ny - 360) * 200) / 240;
                        rx = 640 + ((ny - 360) * 204) / 240;
                        if ((nx >= lx - 4) && (nx <= lx + 4)) gv = 6'd55;
                        if ((nx >= rx - 4) && (nx <= rx + 4)) gv = 6'd55;
                    end
                    px565 = gv[5:0] << 5;
                    px_en = 1'b1;
                    if (dump) $fwrite(fh, "%04x\n", px565);
                    @(posedge clk);
                end
            end
            px_en = 1'b0;
            if (dump) $fclose(fh);
            repeat (4) @(posedge clk);
        end
    endtask

    task dump_result;
        begin
            $display("SHOT_L0 %0d", anch[10:0]);
            $display("SHOT_L1 %0d", anch[21:11]);
            $display("SHOT_L2 %0d", anch[32:22]);
            $display("SHOT_L3 %0d", anch[43:33]);
            $display("SHOT_R0 %0d", anch[54:44]);
            $display("SHOT_R1 %0d", anch[65:55]);
            $display("SHOT_R2 %0d", anch[76:66]);
            $display("SHOT_R3 %0d", anch[87:77]);
            $display("SHOT_TYP %0d %0d", ltyp, rtyp);
            $display("SHOT_OFF %0d", $signed(offv));
            $display("SHOT_STATE %0d", lstate);
            $display("SHOT_POS %0d %0d", lpos[9:0], rpos[9:0]);
            $display("SHOT_VALID %b %b", lpos[10], rpos[10]);
        end
    endtask

    initial begin
        rst_n = 1'b0;
        repeat (10) @(posedge clk);
        rst_n = 1'b1;
        repeat (10) @(posedge clk);

        // 前两帧让它建立 lane_w 和 8 帧历史
        for (nf = 0; nf < 3; nf = nf + 1) begin
            vsync = 1'b1;
            repeat (3) @(posedge clk);
            vsync = 1'b0;
            repeat (4) @(posedge clk);
            feed_frame((nf == 2) ? 1 : 0);
        end

        dump_result;

        if (ltyp === 2'd1 && rtyp === 2'd1 && $signed(offv) < 40 && $signed(offv) > -40)
            $display("SHOT_PASS");
        else
            $display("SHOT_FAIL typ=%0d/%0d off=%0d", ltyp, rtyp, $signed(offv));

        $finish;
    end

endmodule
