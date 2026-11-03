//=====================================================================
// tb_lane_osd_full.v  ——  摄像头全屏(cam_full)下 OSD 叠加是否真的出现
//---------------------------------------------------------------------
// 现场反馈: "四宫格能看见识别，切到全屏就看不见了"。
// 代码上 cam_osd = (src==0) & (~full_scr) & lane_en —— 摄像头全屏时
// full_scr=0、src 全是 0，按理应该为 1。所以必须实测，不能靠读代码下结论。
//
// 这个台和 tb_lane_osd 的区别只有输入模式:
//   四宫格 : 只发 300 块(左上格, 偶数块号)        -> 坐标要减半
//   全屏   : 发满 1200 块(全部摄像头)              -> 坐标就是原图坐标
//
// 判据(全屏不该有任何缩放):
//   L@420 = 400  -> 红点必须落在 (400, 420)
//   R@599 = 800  -> 红点必须落在 (800, 599)
//   状态块三兄弟仍然在 (4..15, 4..15) 这一片
//=====================================================================
`timescale 1ns/1ps
module tb_lane_osd_full;

    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;

    wire        wr_page, wr_done_page;
    reg  [16:0] cam_word;
    reg         cam_empty = 1;
    wire        cam_rd;
    reg  [12:0] cam_level;
    reg  [16:0] pc_word   = 17'd0;
    reg         pc_empty  = 1;
    wire        pc_rd;
    reg  [13:0] pc_level  = 14'd0;
    reg         cam_vs;
    reg         full_scr = 1'b0;      // ★不是 PC 全屏
    reg         cam_full = 1'b1;      // ★是摄像头全屏
    reg  [2:0]  mode_sel = 0;         // 彩色
    reg  [4:0]  ad_level;
    wire [27:0] ad_data;
    wire        ad_wr;
    wire [15:0] wd_data;
    wire        wd_en;
    wire [31:0] cam_px, pc_px;
    wire [15:0] chunk_cnt;
    wire        frame_live, busy;

    wire [127:0] lane_bus = {18'd0,
                             2'd1, 2'd2, 2'd0, 3'd0, 2'b11, 11'd0,
                             {1'b1, 10'd800},          // [87:77] R@599
                             11'd0, 11'd0, 11'd0,
                             11'd0, 11'd0, 11'd0,
                             {1'b1, 10'd400}};         // [10:0]  L@420
    reg          lane_vld = 0;

    quad_wr_ctrl u_dut(
        .clk (clk), .rst_n (rst_n), .init_done (1'b1),
        .cam_word (cam_word), .cam_empty (cam_empty), .cam_rd (cam_rd),
        .cam_level (cam_level),
        .pc_word (pc_word), .pc_empty (pc_empty), .pc_rd (pc_rd), .pc_level (pc_level),
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

    always @(*) begin
        cam_empty = (cw == cr);
        cam_level = (cw - cr > 8191) ? 13'd8191 : cw - cr;
        cam_word  = cfifo[cr % CDEPTH];
    end
    always @(posedge clk) if (cam_rd) cr <= cr + 1;

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
    // 探针
    //-------------------------------------------------------------
    integer ndot = 0, ngrn = 0, errs = 0, i, bad, na, nb;
    integer dotx [0:255], doty [0:255];
    integer nosd = 0, nwrite = 0;

    always @(posedge clk) begin
        if (u_dut.cam_osd) nosd = nosd + 1;
        if (wd_en && (u_dut.src == 1'b0)) begin
            nwrite = nwrite + 1;
            if (wd_data == 16'hF800) begin
                if (ndot < 256) begin
                    dotx[ndot] = u_dut.osd_x;
                    doty[ndot] = u_dut.osd_y;
                end
                ndot = ndot + 1;
            end else if (wd_data == 16'h07E0)
                ngrn = ngrn + 1;
        end
    end

    //-------------------------------------------------------------
    // 激励: 发满 1200 块, 全归摄像头
    //-------------------------------------------------------------
    integer tp, blk, fr;
    task put_cam_blk;
        input integer b;
        begin
            while ((cw - cr) > 30000) @(posedge clk);
            cfifo[cw % CDEPTH] = {1'b1, 5'd0, b[10:0]}; cw = cw + 1;
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
            for (blk = 0; blk < 1200; blk = blk + 1) begin
                put_cam_blk(blk);
                repeat (120) @(posedge clk);
            end
            repeat (3000) @(posedge clk);
            $display("    frame %0d done: cam_px=%0d", fr, cam_px);
            cam_vs = 1;
            @(posedge clk);
            cam_vs = 0;
            repeat (3000) @(posedge clk);
        end
        repeat (1000) @(posedge clk);

        $display("---- cam_osd 为 1 的拍数 %0d / 写入像素 %0d ----", nosd, nwrite);
        $display("---- 红点 %0d 个, 绿块像素 %0d 个 ----", ndot, ngrn);
        for (i = 0; i < ndot && i < 40; i = i + 1)
            $display("   dot(%0d,%0d)", dotx[i], doty[i]);

        if (nosd == 0) begin
            $display("  *** FAIL: 全屏下 cam_osd 一直是 0, 叠加整层被关掉");
            errs = errs + 1;
        end

        bad = 0; na = 0; nb = 0;
        for (i = 0; i < ndot && i < 256; i = i + 1) begin
            if ((doty[i] >= 418) && (doty[i] <= 422) &&
                (dotx[i] >= 398) && (dotx[i] <= 402)) na = na + 1;
            else if ((doty[i] >= 597) && (doty[i] <= 599) &&
                     (dotx[i] >= 798) && (dotx[i] <= 802)) nb = nb + 1;
            else begin
                $display("  *** FAIL: 红点跑到 (%0d,%0d)", dotx[i], doty[i]);
                bad = bad + 1;
            end
        end
        if (na < 3) begin
            $display("  *** FAIL: L@420=400 应落在 (400,420), 只抓到 %0d 个", na);
            errs = errs + 1;
        end
        if (nb < 3) begin
            $display("  *** FAIL: R@599=800 应落在 (800,599), 只抓到 %0d 个", nb);
            errs = errs + 1;
        end
        errs = errs + bad;
        $display("    L 簇 %0d 点, R 簇 %0d 点", na, nb);

        if (errs == 0) $display("LANE_OSD_FULL_PASS");
        else           $display("LANE_OSD_FULL_FAIL errs=%0d", errs);
        $finish;
    end

endmodule
