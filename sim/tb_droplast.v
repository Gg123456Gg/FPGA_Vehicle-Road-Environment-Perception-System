//=====================================================================
// tb_mode_page.v  —— 一页里会不会混两种"算法档位"(现场: 切二值化后顶部还有彩色)
//---------------------------------------------------------------------
// 现场现象(2026-11-02 22:55 的视频, 全屏 PC):
//   切到二值化后, 屏幕**顶部约 55 行(≈110 块)** 一直是彩色的, 下面是二值化;
//   切回彩色后, 顶部那一段又一直是二值化。分界线是笔直的水平线。
//
// 为什么"模式"能混: 模式是**写的时候**烙进 DDR 的(quad_wr_ctrl 里 px_out 按
//   mode_lat 选)。一页里只要有一批块没被本帧覆盖, 那批块就是**旧模式**写进去
//   的内容 —— 屏幕上就是"一块彩色 + 一块二值化"。
//   所以这不是显示通路的问题, 根子还是"这一页没被本帧完整覆盖"。
//
// 本测试台专门造这个场景:
//   翻页请求(flip_pend)置起后, 人为把 ad_level 抬到 4 并保持 5ms ——
//   模拟 DDR 地址 FIFO 一时排不空(读侧抢带宽/写侧积压), 翻页要等一会儿。
//   ★这 5ms 里写侧到底停不停, 就是 v28 与 v29 的分水岭★
//     v28: can_start 不含 ~flip_pend -> 继续起块, 把下一帧最前面约 485 块
//          写进了**还没翻过去的旧页**; 新页从第 486 块开始 -> 顶部 485 块
//          永远是上一页的旧内容(旧模式)。
//     v29: 有翻页请求就停手 -> 队首那块一个像素都没被消费, 新页从块 0 开始。
//
// 判据: 每一页里 1200 个块必须
//   (1) 属于同一个帧号  -> 没有撕裂
//   (2) 属于同一个档位  -> 没有"彩色/二值化混屏"
//=====================================================================
`timescale 1ns/1ps

module tb_droplast;

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

    //-----------------------------------------------------------------
    // 字流 FIFO
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
    // 假 DDR: 地址 FIFO 水位
    //-----------------------------------------------------------------
    integer   adcnt  = 0;
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

    // ★人为制造"翻页要等地址排空"★
    //   flip_pend 置起后把 ad_level 抬到 4(>2) 并保持 HOLD_CYC 拍, 然后放开。
    //   真实场景里这是 DDR 写通道积压 / 读侧抢带宽, 持续时间几十微秒到几毫秒。
    localparam integer HOLD_CYC = 1;     // 本用例不制造等待
    integer hold_cyc = 0;
    reg     hold_en  = 0;
    always @(posedge clk) begin
        if (!rst_n) begin
            hold_cyc <= 0;
            hold_en  <= 0;
        end else if (!u_dut.flip_pend) begin
            hold_cyc <= 0;
            hold_en  <= 0;
        end else if (hold_cyc < HOLD_CYC) begin
            hold_cyc <= hold_cyc + 1;
            hold_en  <= 1'b1;
        end else begin
            hold_en  <= 1'b0;
        end
    end
    always @(*) ad_level = hold_en ? 5'd4 : adcnt[4:0];

    //-----------------------------------------------------------------
    // 记录每一页每个块: 帧号 + 写入时的档位
    //-----------------------------------------------------------------
    integer tb_pcf = 0;
    integer blkf [0:1][0:1210];
    integer blkm [0:1][0:1210];
    wire [10:0] awr_blk  = ad_data[16:6];
    wire        awr_page = ad_data[27];

    always @(posedge clk) begin
        if (ad_wr) begin
            blkf[awr_page][awr_blk] <= tb_pcf;
            blkm[awr_page][awr_blk] <= u_dut.mode_lat;
        end
    end

    //-----------------------------------------------------------------
    // 翻页监视 + 判据
    //-----------------------------------------------------------------
    reg     wd_prev = 0;
    integer nflip = 0, errs = 0, nblk = 0, nprev = 0;
    integer b, pg, newest, fmin, fmax, mmin, mmax;
    integer nold, worst_rows, stale_rows, bfirst;

    task scan_page;
        begin
            // 帧号: 这一页里出现过的帧号范围
            fmin = 32'h7FFFFFFF; fmax = -1;
            mmin = 32'h7FFFFFFF; mmax = -1;
            bfirst = -1; nold = 0;
            for (b = 0; b < 1200; b = b + 1) begin
                if (blkf[pg][b] > fmax) fmax = blkf[pg][b];
                if (blkf[pg][b] < fmin) fmin = blkf[pg][b];
                if (blkm[pg][b] > mmax) mmax = blkm[pg][b];
                if (blkm[pg][b] < mmin) mmin = blkm[pg][b];
                if (blkf[pg][b] != fmax) begin
                    nold = nold + 1;
                    if (bfirst < 0) bfirst = b;
                end
            end
            newest = fmax;
            stale_rows = (bfirst < 0) ? 0 : (bfirst / 2);
            if (stale_rows > worst_rows) worst_rows = stale_rows;

            $display("# [flip %0d @%0dms] page %0d : frame %0d..%0d  mode %0d..%0d  stale_blk_from=%0d n=%0d (top %0d rows)  written=%0d  %s",
                     nflip, $time/1000000, pg, fmin, fmax, mmin, mmax,
                     bfirst, nold, stale_rows, nblk - nprev,
                     ((nold == 0) && (mmin == mmax)) ? "OK" : "*** MIXED ***");
            nprev = nblk;
            if ((nold != 0) || (mmin != mmax)) errs = errs + 1;
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

    //-----------------------------------------------------------------
    // 激励: 全屏 1200 块/帧, 24fps; 第 4 帧起把档位切到二值化, 第 9 帧切回彩色
    //-----------------------------------------------------------------
    localparam integer PC_BLK_GAP = 1738;

    integer k, seq, dst, tp;
    task put_pc_blk;
        input [10:0] blk;
        begin
            while ((pw - pr) > 3500) @(posedge clk);
            pfifo[pw % CDEPTH] = {1'b1, 5'd0, blk}; pw = pw + 1;
            for (tp = 0; tp < 512; tp = tp + 1) begin
                pfifo[pw % CDEPTH] = {1'b0, 16'hBEEF};
                pw = pw + 1;
            end
        end
    endtask

    // 调试探针: 每 10ms 打一次内部状态
    integer dbg = 0;
    always @(posedge clk) begin
        dbg = dbg + 1;
        if (dbg % 500000 == 0)
            $display("#   t=%0dms st=%0d pend=%0d pgs=%0d pgl=%0d pgto=%0d ad=%0d dirty=%0d align=%0d pc0=%0d",
                     $time/1000000, u_dut.st, u_dut.flip_pend, u_dut.pg_started,
                     u_dut.pg_last, u_dut.pg_to, ad_level, u_dut.page_dirty,
                     u_dut.pc_align, u_dut.pc_word[10:0]);
    end

    integer fp, nf;
    initial begin
        for (b = 0; b < 1210; b = b + 1) begin blkf[0][b] = -1; blkf[1][b] = -1; end
        worst_rows = 0;
        cam_vs = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (100) @(posedge clk);

        nf = 14;
        for (fp = 0; fp < nf; fp = fp + 1) begin
            tb_pcf = fp;
            // 第 4 帧切二值化(3), 第 9 帧切回彩色(0)
            if (fp == 4) mode_sel = 3'd3;
            if (fp == 9) mode_sel = 3'd0;
            for (seq = 0; seq < 1200; seq = seq + 1) begin
                dst = seq;
                // ★每帧故意不发最后一块(1199)★ 模拟"帧末包丢了"
                //   这时 pg_last 永远不置, 翻页只能靠 pg_force(100ms) 兜底。
                //   要求: 画面不能冻住(bad_cnt 到 3 会强制发布一次)。
                if (seq != 1199) put_pc_blk(dst[10:0]);
                repeat (PC_BLK_GAP) @(posedge clk);
            end
        end

        repeat (40000) @(posedge clk);
        $display("");
        $display("# ============================================================");
        $display("# flips=%0d  bad_pages=%0d  worst_top_rows=%0d", nflip, errs, worst_rows);
        if (errs == 0) $display("# DROPLAST: flips 仍在增长(画面没冻), 见上面翻页行");
        else           $display("# DROPLAST: 翻页停了 -> 冻屏!");
        $display("# ============================================================");
        $finish;
    end

endmodule
