//=====================================================================
// tb_osd_font.v  ——  决策文字的"字模"验证（v32 新增）
//---------------------------------------------------------------------
// 为什么单独测字模:
//   v31 的 15 个字母分三层函数 (glyph / glyph2 / glyph3) 存放,
//   每层只保留一个 default 分支 —— 这是"加字母不能顶掉老字母"的代价。
//   一旦某个字母落进别的层的 default, 屏幕上的表现是:
//      HOLD 显示成 HOLS / STOP 显示成 STOS ...
//   这种错**不会让任何算法用例失败**, 只会让人上板时看一眼才发现,
//   然后浪费一轮综合(20~40 分钟)。所以必须在这里挡住。
//
// 做法:
//   用 force 把 lane_dec 逐个设成 0..8 (HOLD/GO/STOP/LANE/SOLID/LEFT/
//   RIGHT/LOST/WAIT), 每设一次喂 20 行摄像头数据, 从写通路把
//   文字区的 5x7 点阵抓下来, 然后:
//     ① 每个字符格必须非空 (>=4 个点亮)
//     ② 超出词长的位置必须是空的 (dlen 写错会露出来)
//     ③ 15 个字母的点阵必须互不相同 (← 这条专门抓"层搞错")
//     ④ 文字像素的颜色必须是该决策码的颜色
//   最后把 9 个词打成 ASCII 图印到日志里, 人眼再确认一遍。
//=====================================================================
`timescale 1ns/1ps
module tb_osd_font;

    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;

    wire        wr_page, wr_done_page;
    reg  [16:0] cam_word;
    reg         cam_empty = 1;
    wire        cam_rd;
    reg  [12:0] cam_level;
    reg  [16:0] pc_word;
    reg         pc_empty = 1;
    wire        pc_rd;
    reg  [13:0] pc_level;
    reg         cam_vs;
    reg         full_scr = 1;   // v34: 叠加只在 PC 全屏做
    reg         cam_full = 0;
    reg  [2:0]  mode_sel = 0;
    reg  [4:0]  ad_level;
    wire [27:0] ad_data;
    wire        ad_wr;
    wire [15:0] wd_data;
    wire        wd_en;
    wire [31:0] cam_px, pc_px;
    wire [15:0] chunk_cnt;
    wire        frame_live, busy;

    // lane_bus 用固定值即可, 真正被测的是文字
    wire [127:0] lane_bus = {18'd0, 2'd1, 2'd1, 2'd0, 3'd0, 2'b11, 11'd0, 88'd0};
    reg          lane_vld = 0;

    quad_wr_ctrl u_dut(
        .clk (clk), .rst_n (rst_n), .init_done (1'b1),
        .cam_word (cam_word), .cam_empty (cam_empty), .cam_rd (cam_rd), .cam_level (cam_level),
        .pc_word  (pc_word ), .pc_empty  (pc_empty ), .pc_rd  (pc_rd ), .pc_level  (pc_level ),
        .cam_vs (cam_vs), .wr_page (wr_page), .wr_done_page (wr_done_page),
        .mode_sel (mode_sel), .full_scr (full_scr), .cam_full (cam_full),
        .lane_bus (lane_bus), .lane_vld (lane_vld), .lane_en (1'b1),
        .lane_osd_en  (1'b1    ),   // v51: 显示开关, 测试里常开
        .force_go (1'b0), .force_l (1'b0), .force_r (1'b0), .alarm (),
        .ad_data (ad_data), .ad_wr (ad_wr), .ad_level (ad_level),
        .wd_data (wd_data), .wd_en (wd_en),
        .cam_px (cam_px), .pc_px (pc_px), .chunk_cnt (chunk_cnt),
        .frame_live (frame_live), .busy (busy)
    );

    localparam CDEPTH = 65536;
    reg [16:0] cfifo [0:CDEPTH-1];
    integer cw = 0, cr = 0;
    reg [16:0] pfifo [0:CDEPTH-1];
    integer pw = 0, pr = 0;

    always @(*) begin
        cam_empty = (cw == cr);
        cam_level = cw - cr;
        pc_empty  = (pw == pr);
        pc_level  = pw - pr;
        cam_word  = cfifo[cr % CDEPTH];
        pc_word   = pfifo[pr % CDEPTH];
    end
    always @(posedge clk) begin
        if (cam_rd) cr <= cr + 1;
        if (pc_rd)  pr <= pr + 1;
    end

    integer   adcnt = 0;
    reg [9:0] pxleft = 0;
    always @(posedge clk) begin
        if (!rst_n) begin adcnt <= 0; pxleft <= 0; end
        else begin
            if (ad_wr) adcnt <= adcnt + 1;
            if (wd_en) begin
                if (pxleft == 10'd511) begin pxleft <= 10'd0; adcnt <= adcnt - 1; end
                else pxleft <= pxleft + 10'd1;
            end
        end
    end
    always @(*) ad_level = adcnt[4:0];

    //-------------------------------------------------------------
    // 抓点阵: 探针比写通路晚一拍, 所以先把 lt_on 和它的格坐标打一拍
    //-------------------------------------------------------------
    reg        q_on  = 0;
    reg [3:0]  q_ci  = 0;
    reg [3:0]  q_cu  = 0;
    reg [2:0]  q_row = 0;
    reg [15:0] q_col = 0;
    reg [9:0]  q_y   = 0;

    always @(posedge clk) begin
        q_on  <= u_dut.lt_on & (u_dut.osd_x >= 10'd56) & (u_dut.osd_x <= 10'd115);
        q_ci  <= u_dut.lci;
        q_cu  <= u_dut.ltcu;
        q_row <= u_dut.ltv2;
        q_col <= wd_data;
        q_y   <= u_dut.osd_y;
    end

    integer    dcur = 0;                    // 当前被 force 的决策码
    reg [34:0] fcell [0:44];                 // [d*5+ci] -> 7行x5列
    integer    cerr = 0, errs = 0;
    integer    i, j, k, r, c, nlit;

    // 每个决策码该显示的词长
    function integer wlen;
        input integer d;
        begin
            wlen = (d == 1) ? 2 : ((d == 4) || (d == 6)) ? 5 : 4;
        end
    endfunction

    function [15:0] dcol;
        input integer d;
        begin
            dcol = (d == 0) ? 16'h8410 :
                   (d == 1) ? 16'h07E0 :
                   (d == 3) ? 16'h07FF :
                   ((d == 2) || (d == 4) || (d == 7)) ? 16'hF800 : 16'hFFE0;
        end
    endfunction

    // 清掉一组的点阵
    task clear_cells;
        begin
            for (i = 0; i < 5; i = i + 1) fcell[dcur*5 + i] = 35'd0;
        end
    endtask

    task put_cam_blk;
        input [10:0] blk;
        begin
            while ((cw - cr) > 30000) @(posedge clk);
            cfifo[cw % CDEPTH] = {1'b1, 5'd0, blk}; cw = cw + 1;
            for (i = 0; i < 512; i = i + 1) begin
                cfifo[cw % CDEPTH] = {1'b0, 16'h1234};
                cw = cw + 1;
            end
        end
    endtask

    integer d, f;
    reg [3:0] dforce = 0;
    reg [34:0] uniq [0:44];
    integer    nu, dup, hit;
    reg        found;

    initial begin
        cam_vs = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (100) @(posedge clk);

        for (d = 0; d <= 8; d = d + 1) begin
            dcur = d;
            clear_cells;
`ifdef RC_SAME
            // 反向对照: 假装 dchr/dec_col 坏了, 所有决策都渲染成同一个词
            dforce = 4'd0;
`else
            dforce = d;
`endif
            force u_dut.lane_dec = dforce;

            // 只喂前 20 行(OSD 文字在 y=4..17), 每块 512 像素
            for (r = 0; r < 20; r = r + 1) begin
                put_cam_blk(r * 2);
                // 抓这一行的点阵 (每个像素一拍)
                for (f = 0; f < 600; f = f + 1) begin
                    if (q_on && (q_y <= 10'd17) && (q_y >= 10'd4)) begin
                        if (q_cu < 3'd5) begin
                            k = dcur*5 + q_ci;
                            fcell[k][q_row*5 + (3'd4 - q_cu)] = 1'b1;
                        end
                        if (wd_data != dcol(d)) begin
                            $display("  *** FAIL: d=%0d 文字颜色 %h, 期望 %h", d, wd_data, dcol(d));
                            cerr = cerr + 1;
                        end
                    end
                    @(posedge clk);
                end
                @(posedge clk);   // 让 q_on 反映最后一个像素
                if (q_on && (q_y <= 10'd17) && (q_y >= 10'd4) && (q_cu < 3'd5)) begin
                    k = dcur*5 + q_ci;
                    fcell[k][q_row*5 + (3'd4 - q_cu)] = 1'b1;
                end
            end
            release u_dut.lane_dec;

            // 打完这一帧, 回到行 0
            cam_vs = 1;
            @(posedge clk);
            cam_vs = 0;
            repeat (2000) @(posedge clk);
        end

        //---------------- 判据 ① ② ----------------
        for (d = 0; d <= 8; d = d + 1) begin
            // 打 ASCII 图: 7 行 x (5 字符 x 5 列)
            $display("---- d=%0d  (词长 %0d) ----", d, wlen(d));
            for (r = 0; r < 7; r = r + 1) begin
                $write("     ");
                for (c = 0; c < 5; c = c + 1) begin
                    for (k = 0; k < 5; k = k + 1)
                        $write("%s", fcell[d*5 + c][r*5 + k] ? "#" : ".");
                    $write("  ");
                end
                $write("\n");
            end

            for (c = 0; c < 5; c = c + 1) begin
                nlit = 0;
                for (k = 0; k < 35; k = k + 1) if (fcell[d*5 + c][k]) nlit = nlit + 1;
                if (c < wlen(d)) begin
                    if (nlit < 4) begin
                        $display("  *** FAIL: d=%0d 第 %0d 个字符是空的(点亮 %0d)", d, c, nlit);
                        errs = errs + 1;
                    end
                end else if (nlit != 0) begin
                    $display("  *** FAIL: d=%0d 第 %0d 格超出词长却画了东西(点亮 %0d)", d, c, nlit);
                    errs = errs + 1;
                end
            end
        end

        //---------------- 判据 ③ 15 个字母必须互不相同 ----------------
        nu = 0; dup = 0;
        for (d = 0; d <= 8; d = d + 1)
            for (c = 0; c < wlen(d); c = c + 1) begin
                found = 1'b0;
                for (j = 0; j < nu; j = j + 1)
                    if (uniq[j] == fcell[d*5 + c]) found = 1'b1;
                if (!found) begin
                    uniq[nu] = fcell[d*5 + c];
                    nu = nu + 1;
                end
            end
        $display("---- 不同字模 %0d 个 (期望 15) ----", nu);
        if (nu != 15) begin
            $display("  *** FAIL: 有字母共用了同一个字模(层搞错了?)，只数出 %0d 个", nu);
            errs = errs + 1;
        end

        if (cerr != 0) errs = errs + 1;

`ifdef RC_SAME
        $display("---- RC_SAME: errs=%0d (必须 >0, 否则用例没在测 dchr) ----", errs);
`else
        if (errs == 0) $display("OSD_FONT_PASS");
        else           $display("OSD_FONT_FAIL errs=%0d", errs);
`endif
        $finish;
    end

endmodule
