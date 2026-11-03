//=====================================================================
// tb_quad_ovf.v  —— 四宫格"右上格顶部横带"的复现与验收台
//---------------------------------------------------------------------
// 现场事实(2026-11-03 12:20 板子遥测):
//   四宫格稳态下板子每秒只"收下" 16625 / 18000 包 = 92.4%。
//   丢的 7.6% 不是随机丢, 而是集中在每页末尾 —— 因为一页 PC 写完之后
//   pc_gate 关闸等 cam_vs(约 3.7ms), 这 3.7ms 里写侧一个块都不消费,
//   PC FIFO(8192 字 = 16 块)瞬间灌满, 之后到的包被 udp_video_rx 整包丢掉
//   (room=0, pkt_cnt 都不加)。丢掉的正好是**下一帧的开头**(dst=1,3,5...),
//   也就是右上格最上面那几十行。
//   然后 pc_align 等不到 dst==1, 100ms 超时硬对齐, 这一页从帧中间起头,
//   顶上那一段永远写不到 -> 切二值化之后它们还是彩色 = 那条横带。
//
// 这个台按真实条件建模, 专门量三件事:
//   ① 每页丢了多少包 (RX 侧 room 判据, 和 udp_video_rx 完全一致)
//   ② 每页的 900 个 PC 块里, 有多少块不是本页最新帧写的 (= 横带高度)
//   ③ 每页的第一块是不是 dst==1 (起头对不对)
// 判据: 不能出现"顶部有横带"的页(top_rows > 0), 且丢包要少。
//
//   +define+RC_NO_DRAIN : 把泄压阀关掉, 复现老行为(必须失败)
//=====================================================================
`timescale 1ns/1ps

module tb_quad_ovf;

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
    reg         full_scr = 0;          // 四宫格
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
        .clk          (clk         ),
        .rst_n        (rst_n       ),
        .init_done    (1'b1        ),
        .cam_word     (cam_word    ),
        .cam_empty    (cam_empty   ),
        .cam_rd       (cam_rd      ),
        .cam_level    (cam_level   ),
        .pc_word      (pc_word     ),
        .pc_empty     (pc_empty    ),
        .pc_rd        (pc_rd       ),
        .pc_level     (pc_level    ),
        .cam_vs       (cam_vs      ),
        .wr_page      (wr_page     ),
        .wr_done_page (wr_done_page),
        .mode_sel     (mode_sel    ),
        .full_scr     (full_scr    ),
        .cam_full     (cam_full    ),
        .lane_bus     (128'd0      ),
        .lane_vld     (1'b0        ),
        .lane_en      (1'b1        ),
        .lane_osd_en  (1'b1    ),   // v51: 显示开关, 测试里常开
        .force_go  (1'b0 ), .force_l  (1'b0 ), .force_r  (1'b0 ),
        .alarm      (     ),
        .ad_data      (ad_data     ),
        .ad_wr        (ad_wr       ),
        .ad_level     (ad_level    ),
        .wd_data      (wd_data     ),
        .wd_en        (wd_en       ),
        .cam_px       (cam_px      ),
        .pc_px        (pc_px       ),
        .chunk_cnt    (chunk_cnt   ),
        .frame_live   (frame_live  ),
        .busy         (busy        )
    );

    //=================================================================
    // PC FIFO: ★按真实深度建模★
    //   深度 8192 字, RX 侧 room = level < (8192-512-8) = 7672
    //   room=0 时整包丢掉, 一个字都不收 —— 和 udp_video_rx.v 一模一样
    //=================================================================
    localparam integer PCDEP  = 8192;
    localparam integer ROOM   = PCDEP - 512 - 8;     // 7672

    reg  [16:0] pmem [0:PCDEP-1];
    integer     pw = 0, pr = 0, pfill = 0;
    integer     ndrop_pkt = 0;         // 被 room 挡掉的包数
    integer     nok_pkt   = 0;

    always @(*) begin
        pc_empty = (pfill == 0);
        pc_level = pfill[13:0];
        pc_word  = pmem[pr];
    end
    always @(posedge clk) begin
        if (pc_rd && pfill > 0) begin
            pr    <= (pr + 1) % PCDEP;
            pfill <= pfill - 1;
        end
    end

    // 摄像头 FIFO (够大, 不是本台的观察对象)
    localparam integer CDEP = 65536;
    reg  [16:0] cmem [0:CDEP-1];
    integer     cw = 0, cr = 0, cfill = 0;
    always @(*) begin
        cam_empty = (cfill == 0);
        cam_level = cfill[12:0];
        cam_word  = cmem[cr];
    end
    always @(posedge clk) begin
        if (cam_rd && cfill > 0) begin
            cr    <= (cr + 1) % CDEP;
            cfill <= cfill - 1;
        end
    end

    //=================================================================
    // 假 DDR: 一个块 512 像素, 写完一个块地址 FIFO 才 -1
    //=================================================================
    integer  adcnt = 0;
    reg [9:0] pxleft = 0;
    always @(posedge clk) begin
        if (!rst_n) begin
            adcnt  <= 0;
            pxleft <= 0;
        end else begin
            if (ad_wr) adcnt <= adcnt + 1;
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

    //=================================================================
    // 每页每块最后被写的帧号
    //=================================================================
    integer tb_pcf = 0;
    integer blkf [0:1][0:1210];
    wire [10:0] awr_blk  = ad_data[16:6];
    wire        awr_page = ad_data[27];
    always @(posedge clk) begin
        if (ad_wr) blkf[awr_page][awr_blk] <= tb_pcf;
    end

    //=================================================================
    // 翻页监视 + 判据
    //=================================================================
    reg     wd_prev = 0;
    integer nflip = 0, errs = 0, band_pages = 0;
    integer b, pg, newest, top_rows, n_pc, n_head_ok, first_old;
    integer pc_done = 0;      // PC 发完以后不再统计(那时候"旧"是必然的)
    integer drop_at_flip = 0, drop_total = 0;
    integer last_flip_t = 0, worst_gap = 0;
    integer worst_band = 0;

    task scan_page;
        begin
            newest = -1;
            for (b = 0; b < 1200; b = b + 1)
                if (blkf[pg][b] > newest) newest = blkf[pg][b];

            top_rows = 0; n_pc = 0; n_head_ok = 0; first_old = -1;
            // 右上格 = dst 1,3,...,599 (奇数块, 行 0..299)
            for (b = 1; b < 600; b = b + 2) begin
                n_pc = n_pc + 1;
                if (blkf[pg][b] != newest) begin
                    top_rows = top_rows + 1;
                    if (first_old < 0) first_old = b;
                end
            end
            // 顶部横带高度 = 右上格里"不是本页最新帧"的行数(从 dst=1 顺数)
            if (blkf[pg][1] == newest) n_head_ok = 1;

            drop_at_flip = drop_total;
            drop_total   = 0;

            if (top_rows > 0) begin
                band_pages = band_pages + 1;
                if (top_rows > worst_band) worst_band = top_rows;
            end
            $display("# [flip %0d @%0dms] page %0d newest_f=%0d  TR_old_rows=%0d/300  head=%0d  drop=%0d  first_old_dst=%0d  %s",
                     nflip, $time/1000000, pg, newest, top_rows, n_head_ok,
                     drop_at_flip, first_old,
                     (top_rows == 0) ? "OK" : "*** BAND ***");
        end
    endtask

    always @(posedge clk) begin
        if (rst_n && (wr_done_page !== wd_prev)) begin
            wd_prev = wr_done_page;
            nflip   = nflip + 1;
            pg      = wr_done_page;
            if (nflip > 1 && ($time/1000000 - last_flip_t) > worst_gap)
                worst_gap = $time/1000000 - last_flip_t;
            last_flip_t = $time/1000000;
            if (nflip >= 2 && !pc_done) scan_page;
        end
    end

    //=================================================================
    // 激励
    //=================================================================
    integer k, seq, dst, row, half, tp;
    reg [10:0] tblk;
    integer credit_seen = 4096;      // 发送端"看到的"水位 (2ms 前采的样)

    // 发一个 PC 包: 513 字一次性落进 FIFO; room 不够就整包丢
    task put_pc_pkt;
        input [10:0] blk;
        begin
            if (pfill + 513 > ROOM) begin
                ndrop_pkt = ndrop_pkt + 1;
                drop_total = drop_total + 1;
            end else begin
                pmem[pw] = {1'b1, 5'd0, blk};  pw = (pw + 1) % PCDEP;
                for (tp = 0; tp < 512; tp = tp + 1) begin
                    pmem[pw] = {1'b0, 16'hBEEF};  pw = (pw + 1) % PCDEP;
                end
                pfill = pfill + 513;
                nok_pkt = nok_pkt + 1;
            end
        end
    endtask

    task put_cam_blk;
        input [10:0] blk;
        begin
            cmem[cw] = {1'b1, 5'd0, blk};  cw = (cw + 1) % CDEP;
            for (tp = 0; tp < 512; tp = tp + 1) begin
                cmem[cw] = {1'b0, 16'h1234};  cw = (cw + 1) % CDEP;
            end
            cfill = cfill + 513;
        end
    endtask

    integer fc, fp, i;
    initial begin
        for (b = 0; b < 1210; b = b + 1) begin blkf[0][b] = -1; blkf[1][b] = -1; end
        cam_vs = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (100) @(posedge clk);

        fork
            // ---- 摄像头: 22fps, 300 块铺在前 14ms, 帧尾一个 cam_vs ----
            begin
                for (fc = 0; fc < 30; fc = fc + 1) begin
                    for (k = 0; k < 300; k = k + 1) begin
                        put_cam_blk(k * 2);
                        repeat (900) @(posedge clk);
                    end
                    // 剩余到 45.45ms
                    repeat (2272727 - 300 * 900) @(posedge clk);
                    cam_vs = 1; @(posedge clk); cam_vs = 0;
                end
            end
            // ---- PC: 900 包 / 帧, 每包 2315 拍 = 46.3us (24fps) ----
            //   发送端按 credit 发: 每 2ms 采一次板子水位, >4096 就等
            begin
                for (fp = 0; fp < 26; fp = fp + 1) begin
                    tb_pcf = fp;
                    for (seq = 0; seq < 900; seq = seq + 1) begin
                        // 每 100000 拍(2ms) 采一次水位, 模拟遥测延迟
                        if ((seq % 43) == 0) credit_seen = pfill;
                        while (credit_seen + 512 > 4096) begin
                            repeat (100000) @(posedge clk);
                            credit_seen = pfill;
                        end
                        if (seq < 300) begin
                            row  = seq;
                            half = 1;
                        end else begin
                            row  = 300 + (seq - 300) / 2;
                            half = (seq - 300) % 2;
                        end
                        dst  = row * 2 + half;
                        put_pc_pkt(dst[10:0]);
                        repeat (2315) @(posedge clk);
                    end
                end
            end
            // PC 一停就统计, 别让后面的空页污染结果
            pc_done = 1;
        join

        repeat (20000) @(posedge clk);
        $display("");
        $display("# ============================================================");
        $display("# flips=%0d  带横带的页=%0d (最高 %0d 行)  丢包=%0d  收下=%0d  worst_gap=%0dms",
                 nflip, band_pages, worst_band, ndrop_pkt, nok_pkt, worst_gap);
`ifdef RC_NO_DRAIN
        $display("# RC_NO_DRAIN: 泄压阀关掉的老行为");
`endif
        if (band_pages == 0 && ndrop_pkt == 0 && nflip > 8)
            $display("# QUAD_OVF PASS");
        else
            $display("# QUAD_OVF FAIL");
        $display("# ============================================================");
        $finish;
    end

endmodule
