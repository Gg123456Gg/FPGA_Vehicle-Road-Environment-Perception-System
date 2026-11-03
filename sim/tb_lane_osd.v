//=====================================================================
// tb_lane_osd.v  ——  验证车道线 OSD 叠加的坐标映射 (v31)
//---------------------------------------------------------------------
// 这一段逻辑最容易错的地方是"坐标换算", 而且错了以后现场表现是
// "红点乱飞", 很容易被误判成"检测算法不对"。所以单独测:
//
// 四宫格下摄像头被抽成 512x300 (见 cam_line_pick: 只留偶行偶列),
// 所以原图坐标 (ax, ay) 在屏幕上的落点必须是 (ax/2, ay/2)。
//
// 用例: 只给两个锚点
//   L@420 = 400  -> 屏幕上应该是 (200, 210) 的 3x3 点
//   R@599 = 800  -> 屏幕上应该是 (400, 299) 的 3x3 点
// 期望红点总数 6 (每行 3 个像素 x 2 行), 且不允许出现在别处。
// 另外验证状态块 b0 的位置: 左线实线 -> 绿块落在 (4..15, 4..15) 共 144 像素。
//=====================================================================
`timescale 1ns/1ps
module tb_lane_osd;

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
    reg         full_scr = 0;        // 四宫格
    reg         cam_full = 0;
    reg  [2:0]  mode_sel = 0;        // 彩色(不做灰度变换, 颜色好判)
    reg  [4:0]  ad_level;
    wire [27:0] ad_data;
    wire        ad_wr;
    wire [15:0] wd_data;
    wire        wd_en;
    wire [31:0] cam_px, pc_px;
    wire [15:0] chunk_cnt;
    wire        frame_live, busy;

    // 固定的车道结果: 左线实线, 右线虚线, 居中
    //   [87:0] 锚点  [109:88] {ltyp,rtyp,cdir,lstate,offl,offv}
    //   锚点打包顺序(见 lane_detect): [10:0]=L@420 [21:11]=L@480
    //   [32:22]=L@540 [43:33]=L@599 [54:44]=R@420 [65:55]=R@480
    //   [76:66]=R@540 [87:77]=R@599   —— 低位是 L@420, 不是 L@599
    wire [127:0] lane_bus = {18'd0,
                             2'd1, 2'd2, 2'd0, 3'd0, 2'b11, 11'd0,
                             {1'b1, 10'd800},          // [87:77] R@599
                             11'd0, 11'd0, 11'd0,      // R@540/480/420
                             11'd0, 11'd0, 11'd0,      // L@599/540/480
                             {1'b1, 10'd400}};         // [10:0]  L@420
    reg          lane_vld = 0;

    quad_wr_ctrl u_dut(
        .clk          (clk        ),
        .rst_n        (rst_n      ),
        .init_done    (1'b1       ),
        .cam_word     (cam_word   ),
        .cam_empty    (cam_empty  ),
        .cam_rd       (cam_rd     ),
        .cam_level    (cam_level  ),
        .pc_word      (pc_word    ),
        .pc_empty     (pc_empty   ),
        .pc_rd        (pc_rd      ),
        .pc_level     (pc_level   ),
        .cam_vs       (cam_vs     ),
        .wr_page      (wr_page    ),
        .wr_done_page (wr_done_page),
        .mode_sel     (mode_sel   ),
        .full_scr     (full_scr   ),
        .cam_full     (cam_full   ),
        .lane_bus     (lane_bus   ),
        .lane_vld     (lane_vld   ),
        .lane_en      (1'b1    ),   // 测试里图层常开
        .lane_osd_en  (1'b1    ),   // v51: 显示开关, 测试里常开
        .force_go  (1'b0 ), .force_l  (1'b0 ), .force_r  (1'b0 ),
        .alarm      (     ),
        .ad_data      (ad_data    ),
        .ad_wr        (ad_wr      ),
        .ad_level     (ad_level   ),
        .wd_data      (wd_data    ),
        .wd_en        (wd_en      ),
        .cam_px       (cam_px     ),
        .pc_px        (pc_px      ),
        .chunk_cnt    (chunk_cnt  ),
        .frame_live   (frame_live ),
        .busy         (busy       )
    );

    //-------------------------------------------------------------
    // 字流 FIFO + 假 DDR
    //-------------------------------------------------------------
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

    integer  adcnt = 0;
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
    // 探针: 抓每一笔写进 DDR 的叠加像素
    //-------------------------------------------------------------
    integer ndot = 0, ngrn = 0, errs = 0, i, bad, na, nb;
    integer dotx [0:63], doty [0:63];

    integer n598 = 0, n_last = 0, ntxt = 0, nbad = 0, ndbg = 0;
    always @(posedge clk) begin
        if (wd_en && (u_dut.src == 1'b0) && (u_dut.osd_y == 10'd10) &&
            (u_dut.osd_x >= 10'd56) && (u_dut.osd_x <= 10'd62) && (ndbg < 20)) begin
            $display("   dbg x=%0d lth=%b lci=%0d ltcu=%0d ltv2=%0d row=%b dec=%0d on=%b col=%h",
                     u_dut.osd_x, u_dut.lth, u_dut.lci, u_dut.ltcu, u_dut.ltv2,
                     u_dut.lt_row, u_dut.lane_dec, u_dut.lt_on, wd_data);
            ndbg = ndbg + 1;
        end
    end
    always @(posedge clk) begin
        if (wd_en && (u_dut.src == 1'b0)) begin
            if (u_dut.cur_dst == 11'd598) n598 = n598 + 1;
            if (u_dut.osd_y >= 10'd297) n_last = n_last + 1;
            if (wd_data == 16'hF800) begin           // 锚点红
                if (ndot < 64) begin
                    dotx[ndot] = u_dut.osd_x;
                    doty[ndot] = u_dut.osd_y;
                end
                ndot = ndot + 1;
            end else if (wd_data == 16'h07E0)
                ngrn = ngrn + 1;                     // 状态块 0 (实线绿)
            else if (wd_data == 16'h8410) begin
                // 灰色既是"决策文字(HOLD)"也是"决策色块(保持)", 靠 x 区分:
                //   色块在 36..47, 文字在 56..115
                if ((u_dut.osd_x >= 10'd56) && (u_dut.osd_x <= 10'd115))
                    ntxt = ntxt + 1;
                else
                    nbad = nbad + 1;                // 决策色块, 合法
            end
        end
    end

    //-------------------------------------------------------------
    // 激励: 四宫格, 只发摄像头那 300 块(dst 0,2,...,598)
    //-------------------------------------------------------------
    integer tp, row, fr;
    reg [10:0] tblk;

    task put_cam_blk;
        input [10:0] blk;
        begin
            while ((cw - cr) > 30000) @(posedge clk);
            cfifo[cw % CDEPTH] = {1'b1, 5'd0, blk}; cw = cw + 1;
            for (tp = 0; tp < 512; tp = tp + 1) begin
                cfifo[cw % CDEPTH] = {1'b0, 16'h1234};
                cw = cw + 1;
            end
        end
    endtask

    initial begin
        cam_vs = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (100) @(posedge clk);

        for (fr = 0; fr < 2; fr = fr + 1) begin
            for (row = 0; row < 300; row = row + 1) begin
                tblk = row * 2;
                put_cam_blk(tblk);
                repeat (400) @(posedge clk);
            end
            repeat (5000) @(posedge clk);
            $display("    frame %0d done: cam_px=%0d  dst598 像素=%0d  y>=297 像素=%0d",
                     fr, cam_px, n598, n_last);
            cam_vs = 1;                     // 翻页(四宫格靠 cam_vs)
            @(posedge clk);
            cam_vs = 0;
            repeat (5000) @(posedge clk);
        end
        repeat (2000) @(posedge clk);

        //---------------- 判据 ----------------
        $display("---- 红点 %0d 个, 绿块像素 %0d 个 ----", ndot, ngrn);
        for (i = 0; i < ndot && i < 64; i = i + 1)
            $display("   dot(%0d,%0d)", dotx[i], doty[i]);

        // 探针比写通路晚一拍, 观察到的是"下一个像素", 所以允许 ±2 的偏差;
        // 坐标映射若写错(忘记减半/用错块号)偏差会是几十到几百像素, 抓得住。
        bad = 0; na = 0; nb = 0;
        for (i = 0; i < ndot && i < 64; i = i + 1) begin
            if ((doty[i] == 210) && (dotx[i] >= 198) && (dotx[i] <= 203)) na = na + 1;
            else if ((doty[i] == 299) && (dotx[i] >= 398) && (dotx[i] <= 403)) nb = nb + 1;
            else begin
                $display("  *** FAIL: 红点跑到 (%0d,%0d)", dotx[i], doty[i]);
                bad = bad + 1;
            end
        end
        if (na < 3) begin
            $display("  *** FAIL: L@420 应落在 (200,210), 只抓到 %0d 个点", na);
            errs = errs + 1;
        end
        if (nb < 3) begin
            $display("  *** FAIL: R@599 应落在 (400,299), 只抓到 %0d 个点", nb);
            errs = errs + 1;
        end
        errs = errs + bad;
        $display("    L 簇 %0d 点, R 簇 %0d 点", na, nb);

        $display("---- 决策文字像素 %0d 个, 决策色块像素 %0d 个 ----", ntxt, nbad);
        if (ntxt < 100) begin
            $display("  *** FAIL: 决策文字没画出来(只有 %0d 个像素)", ntxt);
            errs = errs + 1;
        end
        if ((nbad == 0) || (nbad % 144 != 0)) begin
            $display("  *** FAIL: 决策色块像素 %0d, 期望 144 (12x12) 的整数倍", nbad);
            errs = errs + 1;
        end
        if ((ngrn == 0) || (ngrn % 144 != 0)) begin
            $display("  *** FAIL: 绿块像素 %0d, 期望 144 (12x12) 的整数倍", ngrn);
            errs = errs + 1;
        end

        if (errs == 0) $display("LANE_OSD_PASS");
        else           $display("LANE_OSD_FAIL errs=%0d", errs);
        $finish;
    end

endmodule
