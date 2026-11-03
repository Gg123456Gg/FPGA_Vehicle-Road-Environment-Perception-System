//=====================================================================
// tb_full_align.v   —— PC 全屏(1024x600) 每一页有没有"没被本帧覆盖的块"
//---------------------------------------------------------------------
// 现场现象(全屏 PC): 屏幕**顶部**一条横带是旧内容, 下面才是新画面,
//                   带子高度固定, 时有时无。
//
// 判据和 tb_page_align 一样, 但只跑全屏, 而且多统计一项:
//   "这一页里, 块号 0..k-1 有多少块是**更早的帧**写的" —— 那就是顶部那条带子。
//   stale_prefix = 从块 0 开始连续"不是最新帧"的块数, 除以 2 就是屏幕行数。
//
// 全屏 PC 是 24fps(1200 块/41.7ms), 页面节拍由"块 0 到达"驱动。
// 只要翻页那一刻块 0 还没被消费, 新页就从块 0 开始, 一页干净;
// 只要块 0..k-1 已经被写进上一页, 这一页顶部就永远缺 k 块 -> 旧画面。
//=====================================================================
`timescale 1ns/1ps

module tb_full_align;

    reg clk = 0;
    always #10 clk = ~clk;             // 50MHz, 20ns
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
    reg         full_scr = 1;          // ★全屏★
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

    //-----------------------------------------------------------------
    // 字流 FIFO (只要够大)
    //-----------------------------------------------------------------
    localparam CDEPTH = 32768;
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

    //-----------------------------------------------------------------
    // 假 DDR: 1 个地址 = 512 像素, 按数据流消费
    //-----------------------------------------------------------------
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

    //-----------------------------------------------------------------
    // 记录每一页每个块最后一次被写的帧号
    //-----------------------------------------------------------------
    integer tb_pcf = 0;
    integer blkf [0:1][0:1210];
    wire [10:0] awr_blk  = ad_data[16:6];
    wire        awr_page = ad_data[27];

    always @(posedge clk) begin
        if (ad_wr) blkf[awr_page][awr_blk] <= tb_pcf;
    end

    //-----------------------------------------------------------------
    // 翻页监视
    //-----------------------------------------------------------------
    reg     wd_prev = 0;
    integer nflip = 0, errs = 0;
    integer b, pg, newest, nmix, nstale_blk, stale_rows, nold, bfirst, blast;
    integer nblk = 0, nprev = 0, i;
    integer worst_rows = 0;

    task scan_page;
        begin
            // 这一页里最新帧号 = 该页出现过的最小帧号(刚写满的页里最老的是上一帧
            // 的尾巴, 最新的是本帧)。用"出现次数最多的帧号"更稳:
            newest  = -1;
            for (b = 0; b < 1200; b = b + 1)
                if (blkf[pg][b] > newest) newest = blkf[pg][b];

            // 顶部连续"不是最新帧"的块数
            b = 0;
            while ((b < 1200) && (blkf[pg][b] != newest)) b = b + 1;
            nstale_blk = b;
            stale_rows = b / 2;

            // 整页有几个块不是最新帧
            nold = 0;
            nmix = 0;
            bfirst = -1;
            blast  = -1;
            for (b = 0; b < 1200; b = b + 1) begin
                if (blkf[pg][b] != newest) begin
                    nold = nold + 1;
                    if (bfirst < 0) bfirst = b;
                    blast = b;
                end
            end

            if (stale_rows > worst_rows) worst_rows = stale_rows;

            $display("# [flip %0d @%0dms] page %0d : newest_f=%0d  top_stale_blk=%0d (%0d rows)  stale_blk=%0d..%0d n=%0d  new_written=%0d  %s",
                     nflip, $time/1000000, pg, newest,
                     nstale_blk, stale_rows,
                     bfirst, blast, nold, nblk - nprev,
                     (nold == 0) ? "OK" : "*** MIXED ***");
            nprev = nblk;
            if (nold != 0) errs = errs + 1;
        end
    endtask

    always @(posedge clk) begin
        if (ad_wr) nblk = nblk + 1;
        if (rst_n && (wr_done_page !== wd_prev)) begin
            wd_prev = wr_done_page;
            nflip   = nflip + 1;
            pg      = wr_done_page;
            if (nflip >= 3) scan_page;
        end
    end

    // 探针: 丢块/丢字/翻页原因
    reg [2:0] stp = 0;
    reg       fpp = 0;
    always @(posedge clk) begin
        if (u_dut.st == 3'd3 && stp != 3'd3)
            $display("#   DROP  @%0dus dst=%0d pc_align=%0d pglast=%0d",
                     $time/1000, u_dut.pc_word[10:0], u_dut.pc_align, u_dut.pg_last);
        if (u_dut.st == 3'd2 && stp != 3'd2)
            $display("#   SKIP  @%0dus skip_pc=%0d", $time/1000, u_dut.skip_pc);
        if (u_dut.do_flip && !fpp)
            $display("#   FLIP  @%0dus pg_started=%0d pg_last=%0d pg_to=%0d flip_pend=%0d",
                     $time/1000, u_dut.pg_started, u_dut.pg_last, u_dut.pg_to, u_dut.flip_pend);
        stp = u_dut.st;
        fpp = u_dut.do_flip;
    end

    // 探针
    integer gap_max = 0, ad_max = 0;
    always @(posedge clk) begin
        if ((pw - pr) > gap_max) gap_max = pw - pr;
        if (adcnt > ad_max) ad_max = adcnt;
    end

    //-----------------------------------------------------------------
    // 激励: 全屏 1200 块/帧, 24fps
    //-----------------------------------------------------------------
    localparam integer PC_BLK_GAP = 1738;    // 41.7ms/1200 = 34.75us -> 1738 拍

    integer k, seq, dst, tp;

    task put_pc_blk;
        input [10:0] blk;
        begin
            while ((pw - pr) > 3500) @(posedge clk);   // 信用窗口
            pfifo[pw % CDEPTH] = {1'b1, 5'd0, blk}; pw = pw + 1;
            for (tp = 0; tp < 512; tp = tp + 1) begin
                pfifo[pw % CDEPTH] = {1'b0, 16'hBEEF};
                pw = pw + 1;
            end
        end
    endtask

    integer fp, nf;
    reg STALL = 1;
    integer STALL_CYC = 3000000;   // 60ms
    initial begin
        for (b = 0; b < 1210; b = b + 1) begin blkf[0][b] = -1; blkf[1][b] = -1; end
        cam_vs = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (100) @(posedge clk);

        nf = 14;
        for (fp = 0; fp < nf; fp = fp + 1) begin
            tb_pcf = fp;
            for (seq = 0; seq < 1200; seq = seq + 1) begin
                dst = seq;                       // 全屏: 块号就是 0..1199
                put_pc_blk(dst[10:0]);
                repeat (PC_BLK_GAP) @(posedge clk);
                // ★造一次"发送端卡顿"★ 真实的 Windows 发包/信用窗口会抖,
                //   卡 60ms 就会踩到写侧的 50ms 饥饿节拍和 20ms 翻页超时。
                if (STALL && (fp == 5 || fp == 9) && seq == 600)
                    repeat (STALL_CYC) @(posedge clk);
            end
        end

        repeat (40000) @(posedge clk);
        $display("");
        $display("# ============================================================");
        $display("# flips=%0d  bad_pages=%0d  worst_top_rows=%0d  pc_lvl_max=%0d  ad_max=%0d",
                 nflip, errs, worst_rows, gap_max, ad_max);
        if (errs == 0) $display("# FULL_ALIGN PASS");
        else           $display("# FULL_ALIGN FAIL: some page holds blocks of two frames -> horizontal seam");
        $display("# ============================================================");
        $finish;
    end

endmodule
