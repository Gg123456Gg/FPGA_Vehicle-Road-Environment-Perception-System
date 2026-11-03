//=====================================================================
// tb_lane_off.v  ——  v51: 关掉【显示】以后, 屏幕上干净但后台照跑
//---------------------------------------------------------------------
// v50 的做法是"真停检测", 所以这里当初断言的是"锚点清零"。
// v51 改需求: 面板开关只管显示, 检测流水常开。于是本台要证的是:
//
//   A. 关了以后, 左上角 128x20 每个像素都必须是原始底板色 (16'h0000),
//      不许残留红点 / 色块 / 文字
//   B. 关了以后锚点总线仍然非 0, 并且和开着那一轮的数值一样
//      —— 这一条是关键: 说明 lane_detect 还在逐帧算, 不是把结果藏起来
//   C. 开了那一轮左上角必须有 OSD 像素, 否则"关掉没变化"毫无意义
//
// 激励: PC 全屏 1200 块 x 512 像素, 底 16'h0000, 在 ROI 行 360..599
//       左右各画一条 11px 竖直白线 (随行缓慢收窄, 模拟透视)。
//       灰度只用绿通道, 底 g8=0, 线 g8=255, (0+255)/2=127 的阈值切得开。
//=====================================================================
`timescale 1ns/1ps
module tb_lane_off;

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
    reg         full_scr = 1'b1;
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

    reg          lane_osd_en = 1'b1;      // ★被测开关★

    wire [127:0] lane_bus = 128'd0;
    reg          lane_vld = 0;

    quad_wr_ctrl u_dut(
        .clk (clk), .rst_n (rst_n), .init_done (1'b1),
        .cam_word (cam_word), .cam_empty (cam_empty), .cam_rd (cam_rd),
        .cam_level (cam_level),
        .pc_word (pc_word), .pc_empty (pc_empty), .pc_rd (pc_rd), .pc_level (pc_level),
        .cam_vs (cam_vs), .wr_page (wr_page), .wr_done_page (wr_done_page),
        .mode_sel (mode_sel), .full_scr (full_scr), .cam_full (cam_full),
        .lane_bus (lane_bus), .lane_vld (lane_vld), .lane_en (1'b1),
        .lane_osd_en (lane_osd_en),
        .force_go (1'b0), .force_l (1'b0), .force_r (1'b0), .alarm (),
        .ad_data (ad_data), .ad_wr (ad_wr), .ad_level (ad_level),
        .wd_data (wd_data), .wd_en (wd_en),
        .cam_px (cam_px), .pc_px (pc_px), .chunk_cnt (chunk_cnt),
        .frame_live (frame_live), .busy (busy), .drv_state (drv_state)
    );

    localparam [15:0] FLOOR = 16'h0000;   // 底板 (暗)
    localparam [15:0] WHITE = 16'hFFFF;   // 车道线

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
    // 合成帧: row 360..599 上左右各一条 11px 竖白线
    //-------------------------------------------------------------
    function [15:0] px_at;
        input integer row;
        input integer x;
        integer xl, xr;
        begin
            xl = 300 + ((row - 360) / 4);
            xr = 720 - ((row - 360) / 4);
            if ((row >= 360) && (row <= 599) &&
                (((x >= xl - 5) && (x <= xl + 5)) ||
                 ((x >= xr - 5) && (x <= xr + 5))))
                px_at = WHITE;
            else
                px_at = FLOOR;
        end
    endfunction

    //-------------------------------------------------------------
    // 抓左上角 128x20
    //-------------------------------------------------------------
    localparam integer AW = 128, AH = 20;
    integer phase = 0;                 // 0 = 打开轮, 1 = 关闭轮
    integer n_on_px = 0, n_off_px = 0, n_off_bad = 0;
    integer n_on_anch_nz = 0, n_off_anch_nz = 0;
    reg [87:0] anch_on_last  = 88'd0;
    reg [87:0] anch_off_last = 88'd0;
    integer errs = 0;
    reg [15:0] first_bad;
    integer    bad_x, bad_y;

    always @(posedge clk) begin
        if (wd_en && (u_dut.src == 1'b1) &&
            (u_dut.osd_x < AW) && (u_dut.osd_y < AH)) begin
            if (phase == 0) begin
                if (wd_data !== FLOOR) n_on_px = n_on_px + 1;
            end else begin
                n_off_px = n_off_px + 1;
                if (wd_data !== FLOOR) begin
                    if (n_off_bad == 0) begin
                        first_bad = wd_data;
                        bad_x = u_dut.osd_x;
                        bad_y = u_dut.osd_y;
                    end
                    n_off_bad = n_off_bad + 1;
                end
            end
        end
    end

    //-------------------------------------------------------------
    integer tp, blk, fr;
    localparam integer NF = 4;         // 每轮 4 帧

    task put_pc_blk;
        input integer b;
        integer rw, hf, xg;
        begin
            rw = b / 2;
            hf = b % 2;
            while ((pw - pr) > 6000) @(posedge clk);
            pfifo[pw % CDEPTH] = {1'b1, 5'd0, b[10:0]}; pw = pw + 1;
            for (tp = 0; tp < 512; tp = tp + 1) begin
                xg = hf * 512 + tp;
                pfifo[pw % CDEPTH] = {1'b0, px_at(rw, xg)};
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

    task do_reset;
        begin
            rst_n = 1'b0;
            repeat (20) @(posedge clk);
            rst_n = 1'b1;
            repeat (50) @(posedge clk);
        end
    endtask

    initial begin
        cam_vs = 0;
        do_reset;

        // ---------------- 轮 1: 显示开着 ----------------
        phase = 0;
        for (fr = 0; fr < NF; fr = fr + 1) begin
            one_frame;
            if (u_dut.ld_anch !== 88'd0) begin
                n_on_anch_nz = n_on_anch_nz + 1;
                anch_on_last = u_dut.ld_anch;
            end
        end

        // ---------------- 轮 2: 显示关掉 ----------------
        do_reset;
        phase = 1;
        lane_osd_en = 1'b0;
        for (fr = 0; fr < NF; fr = fr + 1) begin
            one_frame;
            if (u_dut.ld_anch !== 88'd0) begin
                n_off_anch_nz = n_off_anch_nz + 1;
                anch_off_last = u_dut.ld_anch;
            end
        end
        repeat (2000) @(posedge clk);

        $display("---- ON : OSD-overwritten pixels in TL corner = %0d ----", n_on_px);
        $display("---- ON : frames with nonzero anchors = %0d / %0d ----", n_on_anch_nz, NF);
        $display("---- OFF: sampled %0d, non-original pixels %0d ----", n_off_px, n_off_bad);
        $display("---- OFF: frames with nonzero anchors = %0d / %0d ----", n_off_anch_nz, NF);
        $display("---- anch  ON = %h", anch_on_last);
        $display("---- anch OFF = %h", anch_off_last);

        // C. 开着那一轮确实画了东西
        if (n_on_px < 40) begin
            $display("  *** FAIL: too few OSD pixels while ON, comparison meaningless");
            errs = errs + 1;
        end
        // A. 关掉后左上角一个像素都没被改写
        if (n_off_px < AW * AH) begin
            $display("  *** FAIL: OFF phase not fully sampled (%0d < %0d)", n_off_px, AW * AH);
            errs = errs + 1;
        end
        if (n_off_bad != 0) begin
            $display("  *** FAIL: %0d pixels still overwritten after OFF, first at (%0d,%0d) = %h",
                     n_off_bad, bad_x, bad_y, first_bad);
            errs = errs + 1;
        end
        // 激励本身得有效: 开着的时候必须检出线
        if (n_on_anch_nz < 2) begin
            $display("  *** FAIL: synthesised lines not detected even while ON (%0d frames)", n_on_anch_nz);
            errs = errs + 1;
        end
        // B. 关键: 关掉显示以后检测还在跑, 锚点和开着时一致
        if (n_off_anch_nz < 2) begin
            $display("  *** FAIL: detector stopped after OFF (%0d nonzero frames)", n_off_anch_nz);
            errs = errs + 1;
        end
        if (anch_off_last !== anch_on_last) begin
            $display("  *** FAIL: anchors differ, ON=%h OFF=%h", anch_on_last, anch_off_last);
            errs = errs + 1;
        end else if (anch_off_last !== 88'd0)
            $display("    anchors identical while OFF -> detector still running");

        if (errs == 0) $display("LANE_OFF_PASS");
        else           $display("LANE_OFF_FAIL errs=%0d", errs);
        $finish;
    end

    initial begin
        #200000000;     // 2 轮 x 4 帧, 余量给足
        $display("  *** FAIL: timeout");
        $display("LANE_OFF_FAIL");
        $finish;
    end

endmodule
