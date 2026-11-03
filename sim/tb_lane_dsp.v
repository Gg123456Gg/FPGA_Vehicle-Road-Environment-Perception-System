//=====================================================================
// tb_lane_dsp.v  ——  v49: "丢线"在左上角是不是真的显示成 BOTH
//---------------------------------------------------------------------
// 需求: 删掉 LOST 这一类, 把它并入"左右都能变"(BOTH)。
//
// 这个台的做法是最硬的比对, 不做"某个像素是不是青色"这种软检查:
//   同一份像素激励跑两轮, 唯一差别是强制 u_dut.lane_dec
//     第 1 轮 force 4'd7 (丢线)    -> 抓左上角 128x20 全部像素
//     第 2 轮 force 4'd9 (BOTH)    -> 再抓一份
//   然后逐像素比。完全一致 = RTL 里 7 和 9 走的是同一条显示路径,
//   也就是"LOST 在屏幕上已经不存在了"。
//
// ★喂料必须走 PC 通道★
//   cam_osd = (src==1) & full_scr & lane_en —— v34 起叠加只画在 PC 像素上,
//   所以 full_scr=1 且往 pc 通道送 1200 块。老台 tb_lane_osd_full 用的是
//   摄像头通道(v33 口径), 早就退役了, 不能照抄。
//
// 另外两条断言(都在第 1 轮里查):
//   A. 左上角必须出现青色 16'h07FF —— 块 2 和文字跟 BOTH 走同一个颜色
//   B. 左上角不允许出现红色 16'hF800 —— 丢线红块/红字彻底消失
//      (块 0/1 是线型色, 无线时是白 FFFF, 不会误伤)
//=====================================================================
`timescale 1ns/1ps
module tb_lane_dsp;

    reg clk = 0;
    always #10 clk = ~clk;
    reg rst_n = 0;

    wire        wr_page, wr_done_page;
    reg  [16:0] cam_word;
    reg         cam_empty = 1;
    wire        cam_rd;
    reg  [12:0] cam_level;
    reg  [16:0] pc_word;
    reg         pc_empty  = 1;
    wire        pc_rd;
    reg  [13:0] pc_level;
    reg         cam_vs;
    reg         full_scr = 1'b1;      // ★PC 全屏, 叠加层才开
    reg         cam_full = 1'b0;
    reg  [2:0]  mode_sel = 0;
    reg  [4:0]  ad_level;
    wire [27:0] ad_data;
    wire        ad_wr;
    wire [15:0] wd_data;
    wire        wd_en;
    wire [31:0] cam_px, pc_px;
    wire [15:0] chunk_cnt;
    wire        frame_live, busy;
    wire [1:0]  drv_state;

    wire [127:0] lane_bus = {18'd0,
                             2'd0, 2'd0, 2'd0, 3'd0, 2'b00, 11'd0,
                             11'd0, 11'd0, 11'd0, 11'd0,
                             11'd0, 11'd0, 11'd0, 11'd0};
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
        .frame_live (frame_live), .busy (busy), .drv_state (drv_state)
    );

    localparam CDEPTH = 65536;
    reg [16:0] pfifo [0:CDEPTH-1];
    integer pw = 0, pr = 0;

    always @(*) begin
        cam_empty = 1'b1;
        cam_level = 13'd0;
        cam_word  = 17'd0;
        pc_empty  = (pw == pr);
        pc_level  = (pw - pr > 16383) ? 14'd16383 : pw - pr;
        pc_word   = pfifo[pr % CDEPTH];
    end
    always @(posedge clk) if (pc_rd) pr <= pr + 1;

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
    // 抓图: 左上角 128x20
    //-------------------------------------------------------------
    localparam integer AW = 128, AH = 20;
    reg [15:0] cap_a [0:AW*AH-1];     // 第 1 轮 (lane_dec = 7)
    reg [15:0] cap_b [0:AW*AH-1];     // 第 2 轮 (lane_dec = 9)
    integer phase = 0;
    integer seen_a = 0, seen_b = 0;
    integer i, diff, errs = 0;
    integer ncyan = 0, nred = 0, npix = 0;

    always @(posedge clk) begin
        if (wd_en && (u_dut.src == 1'b1) &&
            (u_dut.osd_x < AW) && (u_dut.osd_y < AH)) begin
            if (phase == 0) begin
                cap_a[u_dut.osd_y * AW + u_dut.osd_x] <= wd_data;
                seen_a <= seen_a + 1;
            end else begin
                cap_b[u_dut.osd_y * AW + u_dut.osd_x] <= wd_data;
                seen_b <= seen_b + 1;
            end
            if (wd_data == 16'h07FF) ncyan = ncyan + 1;
            if (wd_data == 16'hF800) nred  = nred  + 1;
            npix = npix + 1;
        end
    end

    //-------------------------------------------------------------
    // 激励: PC 全屏 1200 块 x 512 像素
    //-------------------------------------------------------------
    integer tp, blk, fr;
    task put_pc_blk;
        input integer b;
        begin
            while ((pw - pr) > 6000) @(posedge clk);
            pfifo[pw % CDEPTH] = {1'b1, 5'd0, b[10:0]}; pw = pw + 1;
            for (tp = 0; tp < 512; tp = tp + 1) begin
                pfifo[pw % CDEPTH] = {1'b0, 16'hBEEF};
                pw = pw + 1;
            end
        end
    endtask

    task one_frame;
        begin
            for (blk = 0; blk < 1200; blk = blk + 1) begin
                put_pc_blk(blk);
                repeat (120) @(posedge clk);
            end
            repeat (6000) @(posedge clk);
        end
    endtask

    initial begin
        cam_vs = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (100) @(posedge clk);

        // ---------------- 第 1 轮: 强制丢线 ----------------
        phase = 0;
        force u_dut.lane_dec = 4'd7;
        repeat (200) @(posedge clk);
        for (fr = 0; fr < 2; fr = fr + 1) one_frame;
        repeat (2000) @(posedge clk);

        // ---------------- 第 2 轮: 强制 BOTH ----------------
        phase = 1;
        force u_dut.lane_dec = 4'd9;
        repeat (200) @(posedge clk);
        for (fr = 0; fr < 2; fr = fr + 1) one_frame;
        repeat (2000) @(posedge clk);
        release u_dut.lane_dec;

        $display("---- 左上角采样: 第1轮 %0d 拍, 第2轮 %0d 拍 ----", seen_a, seen_b);
        $display("---- 第1轮内 青(07FF) %0d 拍, 红(F800) %0d 拍 ----", ncyan, nred);

        if (seen_a < AW * AH || seen_b < AW * AH) begin
            $display("  *** FAIL: 左上角没采满 (%0d / %0d, 需要 %0d)", seen_a, seen_b, AW * AH);
            errs = errs + 1;
        end
        if (nred != 0) begin
            $display("  *** FAIL: 丢线状态下左上角还有红色像素 %0d 拍 (LOST 没删干净)", nred);
            errs = errs + 1;
        end
        if (ncyan < 40) begin
            $display("  *** FAIL: 青色只有 %0d 拍, 块2/文字没跟上 BOTH 的颜色", ncyan);
            errs = errs + 1;
        end

        diff = 0;
        for (i = 0; i < AW * AH; i = i + 1) begin
            if (cap_a[i] !== cap_b[i]) begin
                if (diff < 10)
                    $display("    (%0d,%0d) dec=7 -> %h, dec=9 -> %h",
                             i % AW, i / AW, cap_a[i], cap_b[i]);
                diff = diff + 1;
            end
        end
        if (diff != 0) begin
            $display("  *** FAIL: 7 和 9 的显示不一致, 有 %0d 个像素不同", diff);
            errs = errs + 1;
        end else
            $display("    dec=7 与 dec=9 的左上角 %0d 个像素完全一致", AW * AH);

        if (errs == 0) $display("LANE_DSP_PASS");
        else           $display("LANE_DSP_FAIL errs=%0d", errs);
        $finish;
    end

endmodule
