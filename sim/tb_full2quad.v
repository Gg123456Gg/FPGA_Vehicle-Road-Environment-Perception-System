//=====================================================================
// tb_full2quad.v   —— 先跑 PC 全屏, 再切到四宫格, 看写侧还翻不翻页
//---------------------------------------------------------------------
// 现场(2026-11-02 23:16 的视频): 右上格(PC 上半区)正常, 但**切到四宫格之后
// 下面的画面卡在那里**, 一直不更新。
//
// 四宫格 PC 的块号(见 pc/video_sender.py quad_dst):
//   seq   0..299 : 上半区右半  dst = 1,3,5,...,599   (左上角是摄像头)
//   seq 300..899 : 下半区整行  dst = 600,601,...,1199
// 所以"下面卡住" == dst>=600 这一半没有被本帧覆盖。
//
// 判据: 每翻一页, 统计这一页里 dst<600 / dst>=600 各被写了多少块、帧号范围。
//       只要出现"某页下半区一块没写"或"很久不翻页", 就是现场那个现象。
//=====================================================================
`timescale 1ns/1ps

module tb_full2quad;

    reg clk = 0;
    always #10 clk = ~clk;             // 50MHz
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
    reg         full_scr = 1;
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
        .lane_bus     (128'd0   ),   // v31: 本台不测车道线, 给常量
        .lane_vld     (1'b0    ),
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
    // 字流 FIFO (比真实深度大很多, 专门用来观察"写侧停手"而不是"FIFO 满丢包")
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

    //-------------------------------------------------------------
    // 假 DDR
    //-------------------------------------------------------------
    integer  adcnt = 0;
    reg [9:0] pxleft = 0;
    always @(posedge clk) begin
        if (!rst_n) begin
            adcnt  <= 0;
            pxleft <= 0;
        end else begin
            if (ad_wr)  adcnt <= adcnt + 1;
            if (wd_en) begin
                if (pxleft == 10'd511) begin
                    pxleft <= 10'd0;
                    adcnt  <= adcnt - 1;
                end else
                    pxleft <= pxleft + 10'd1;
            end
        end
    end
    always @(*) ad_level = adcnt[4:0];

    //-------------------------------------------------------------
    // 每页每块最后被写的帧号
    //-------------------------------------------------------------
    integer tb_pcf = 0;
    integer blkf [0:1][0:1210];
    wire [10:0] awr_blk  = ad_data[16:6];
    wire        awr_page = ad_data[27];
    always @(posedge clk) begin
        if (ad_wr) blkf[awr_page][awr_blk] <= tb_pcf;
    end

    //-------------------------------------------------------------
    // 翻页监视
    //-------------------------------------------------------------
    reg     wd_prev = 0;
    integer nflip = 0, errs = 0;
    integer b, pg, newest, nold;
    integer n_hi = 0, n_lo = 0;        // 上半区(dst<600) / 下半区(dst>=600) 写了多少块
    integer first_lo = -1, last_lo = -1;
    integer nblk = 0, nprev = 0;
    integer last_flip_t = 0, gap_t = 0, worst_gap = 0;

    task scan_page;
        begin
            newest = -1;
            for (b = 0; b < 1200; b = b + 1)
                if (blkf[pg][b] > newest) newest = blkf[pg][b];

            nold = 0; n_hi = 0; n_lo = 0; first_lo = -1; last_lo = -1;
            for (b = 0; b < 1200; b = b + 1) begin
                if (blkf[pg][b] != newest) nold = nold + 1;
                // 只看 PC 该负责的块
                if (b >= 600) begin
                    if (blkf[pg][b] == newest) begin
                        n_lo = n_lo + 1;
                        if (first_lo < 0) first_lo = b;
                        last_lo = b;
                    end
                end else if (b[0]) begin      // 上半区右半(奇数块)才是 PC 的
                    if (blkf[pg][b] == newest) n_hi = n_hi + 1;
                end
            end

            $display("# [flip %0d @%0dms] page %0d : newest_f=%0d  pc_hi=%0d/300  pc_lo=%0d/600 (dst %0d..%0d)  old=%0d  written=%0d  %s",
                     nflip, $time/1000000, pg, newest,
                     n_hi, n_lo, first_lo, last_lo, nold, nblk - nprev,
                     ((pc_hi_ok(n_hi) && pc_lo_ok(n_lo)) ? "OK" : "*** PARTIAL ***"));
            nprev = nblk;
            if (!(pc_hi_ok(n_hi) && pc_lo_ok(n_lo))) errs = errs + 1;
        end
    endtask

    function pc_hi_ok; input integer v; pc_hi_ok = (v > 280); endfunction
    function pc_lo_ok; input integer v; pc_lo_ok = (v > 560); endfunction

    always @(posedge clk) begin
        if (ad_wr) nblk = nblk + 1;
        if (rst_n && (wr_done_page !== wd_prev)) begin
            wd_prev = wr_done_page;
            nflip   = nflip + 1;
            pg      = wr_done_page;
            gap_t   = ($time/1000000) - last_flip_t;
            if (nflip > 1 && gap_t > worst_gap) worst_gap = gap_t;
            last_flip_t = $time/1000000;
            if (nflip >= 2) scan_page;
        end
    end

    // 精确探针: 翻页请求到翻页成立等了多久 + 何时停手
    reg     fp_prev = 0;
    integer ndrop = 0;
    reg     gt_prev = 1;
    integer t_pend  = 0;
    integer t_lastw = 0;
    always @(posedge clk) begin
        if (u_dut.flip_pend && !fp_prev) t_pend = $time/1000;
        if (u_dut.do_flip)
            $display("# FLIP @%0dus pend@%0dus wait=%0dus st=%0d ad=%0d pgs=%0d pgl=%0d gate=%0d align=%0d dst=%0d",
                     $time/1000, t_pend, $time/1000 - t_pend, u_dut.st, ad_level,
                     u_dut.pg_started, u_dut.pg_last, u_dut.pc_gate, u_dut.pc_align,
                     u_dut.pc_word[10:0]);
        if (u_dut.st == 3'd3 && u_dut.pc_align) begin
            if (ndrop < 12)
                $display("#   DROP @%0dus dst=%0d align=%0d gate=%0d seen=%0d pend=%0d",
                         $time/1000, u_dut.pc_word[10:0], u_dut.pc_align,
                         u_dut.pc_gate, u_dut.pc_seen, u_dut.flip_pend);
            ndrop = ndrop + 1;
        end else if (u_dut.st != 3'd3) ndrop = 0;
        if (u_dut.pc_gate !== gt_prev) begin
            $display("# GATE %0d @%0dus dst=%0d st=%0d", u_dut.pc_gate, $time/1000,
                     u_dut.pc_word[10:0], u_dut.st);
            gt_prev = u_dut.pc_gate;
        end
        if (ad_wr) t_lastw = $time/1000;
        fp_prev = u_dut.flip_pend;
    end

    // 状态探针: 每 20ms 打一次, 看是不是卡在 flip_pend=1 出不来
    integer dbg = 0;
    always @(posedge clk) begin
        dbg = dbg + 1;
        if (dbg % 1000000 == 0)
            $display("#   t=%0dms st=%0d pend=%0d pgs=%0d pgl=%0d pgto=%0d ad=%0d gate=%0d align=%0d dst=%0d full=%0d",
                     $time/1000000, u_dut.st, u_dut.flip_pend, u_dut.pg_started,
                     u_dut.pg_last, u_dut.pg_to, ad_level, u_dut.pc_gate,
                     u_dut.pc_align, u_dut.pc_word[10:0], full_scr);
    end

    //-------------------------------------------------------------
    // 激励
    //-------------------------------------------------------------
    integer k, seq, dst, tp, row, half;
    reg [10:0] tblk;

    task put_pc_blk;
        input [10:0] blk;
        begin
            while ((pw - pr) > 30000) @(posedge clk);
            pfifo[pw % CDEPTH] = {1'b1, 5'd0, blk}; pw = pw + 1;
            for (tp = 0; tp < 512; tp = tp + 1) begin
                pfifo[pw % CDEPTH] = {1'b0, 16'hBEEF};
                pw = pw + 1;
            end
        end
    endtask

    // 摄像头: 只发左上那一格(上半区左半, dst 0,2,...,598), 每块 513 字
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

    integer fp, nf, i;
    initial begin
        for (b = 0; b < 1210; b = b + 1) begin blkf[0][b] = -1; blkf[1][b] = -1; end
        cam_vs = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (100) @(posedge clk);

        //---------------- 全屏 6 帧 ----------------
        fp = 0;
        fork
            // 摄像头场同步: 22fps
            begin
                for (i = 0; i < 40; i = i + 1) begin
                    repeat (2270000) @(posedge clk);
                    cam_vs = 1;                 // ★真实 cam_vs 只有 1 拍★
                    @(posedge clk);
                    cam_vs = 0;
                end
            end
            begin
                for (fp = 0; fp < 6; fp = fp + 1) begin
                    tb_pcf = fp;
                    for (seq = 0; seq < 1200; seq = seq + 1) begin
                        tblk = seq[10:0];
                        put_pc_blk(tblk);
                        repeat (1738) @(posedge clk);       // 24fps
                    end
                end

                //---------------- 切到四宫格 ----------------
                $display("# ---- switch to QUAD at %0dms ----", $time/1000000);
                full_scr = 0;

                for (fp = 6; fp < 14; fp = fp + 1) begin
                    tb_pcf = fp;
                    // 摄像头那格(左上): 每行一块, 300 块
                    for (row = 0; row < 300; row = row + 1) begin
                        tblk = row * 2;
                        put_cam_blk(tblk);
                        repeat (400) @(posedge clk);
                    end
                    // PC: 900 包, 41.7ms/帧
                    for (seq = 0; seq < 900; seq = seq + 1) begin
                        if (seq < 300) begin
                            row  = seq;
                            half = 1;
                        end else begin
                            row  = 300 + (seq - 300) / 2;
                            half = (seq - 300) % 2;
                        end
                        dst = row * 2 + half;
                        tblk = dst[10:0];
                        put_pc_blk(tblk);
                        repeat (2315) @(posedge clk);
                    end
                end

                //---------------- 再切回全屏 ----------------
                $display("# ---- back to FULL at %0dms ----", $time/1000000);
                full_scr = 1;
                for (fp = 14; fp < 22; fp = fp + 1) begin
                    tb_pcf = fp;
                    for (seq = 0; seq < 1200; seq = seq + 1) begin
                        tblk = seq[10:0];
                        put_pc_blk(tblk);
                        repeat (1738) @(posedge clk);
                    end
                end
            end
        join

        repeat (40000) @(posedge clk);
        $display("");
        $display("# ============================================================");
        $display("# flips=%0d  bad_pages=%0d  worst_flip_gap=%0dms", nflip, errs, worst_gap);
        if (errs == 0 && worst_gap < 200) $display("# FULL2QUAD PASS");
        else $display("# FULL2QUAD FAIL: 切到四宫格后有页缺下半区 / 翻页间隔过大(卡住)");
        $display("# ============================================================");
        $finish;
    end

endmodule
