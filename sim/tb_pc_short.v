//=====================================================================
// tb_page_align.v   —— 验证"一页里有没有混两帧"
//---------------------------------------------------------------------
// 现场现象: 四宫格里 PC 那几格是横向条带, 像几帧画面拼在一起, 带子慢慢跑;
//           PC 全屏也一样; 摄像头全屏(PC 关掉)正常。
//
// 这个测试台不猜, 直接把"每一页里到底装了几帧"数出来:
//   假 DDR 只做一件事: 记住每个块被写进去的那一刻, 源头是第几帧。
//   每次翻页的瞬间, 把刚写满的那一页 1200 个块扫一遍,
//   分别统计 摄像头块 和 PC 块 的帧号种类数。
//
//   1 种 = 干净;  >1 种 = 这一页里混了帧, 屏幕上就是一条接缝。
//
// 摄像头 22fps(45.4ms), PC 24fps(41.7ms), 两个节拍本来就不同步 ——
// 这正是要复现的条件。
//=====================================================================
`timescale 1ns/1ps

module tb_pc_short;

    // ---------------- 时钟 / 复位 ----------------
    reg clk = 0;
    always #10 clk = ~clk;             // 50MHz, 20ns
    reg rst_n = 0;

    // ---------------- DUT 连线 ----------------
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

    //=================================================================
    // 两个字流 FIFO (够大, 只测读写行为, 不测容量)
    //=================================================================
    localparam CDEPTH = 16384;
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

    //=================================================================
    // 假 DDR: 地址 FIFO 的消费节奏 (1 个地址 = 512 个像素)
    //=================================================================
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

    //=================================================================
    // 帧号记录: 每个块被写进去时, 源头是第几帧
    //   块号 = ad_data[16:6], 页 = ad_data[27]
    //   四宫格: 块号 < 600 且为偶数 = 左上格 = 摄像头, 其余 = PC
    //=================================================================
    integer tb_camf = 0;      // 当前摄像头帧号
    integer tb_pcf  = 0;      // 当前 PC 帧号
    integer blkf [0:1][0:1203];

    wire [10:0] awr_blk    = ad_data[16:6];
    wire        awr_page   = ad_data[27];
    wire        awr_is_cam = (~full_scr) && (awr_blk < 11'd600) && (awr_blk[0] == 1'b0);

    always @(posedge clk) begin
        if (ad_wr) blkf[awr_page][awr_blk] <= awr_is_cam ? tb_camf : tb_pcf;
    end

    //=================================================================
    // 翻页监视: 每次 wr_done_page 变化, 扫一遍刚写满的那一页
    //   (用阻塞赋值, 否则 pg / nflip 会晚一拍, 扫到的是上一页)
    //=================================================================
    reg     wd_prev = 0;
    integer nflip = 0, errs = 0;
    integer b, cmin, cmax, pmin, pmax, nbad;
    integer pg;
    integer nblk = 0, ncamb = 0, npcb = 0;
    integer nprev = 0;

    task scan_page;
        begin
            cmin = 100000; cmax = -100000;
            pmin = 100000; pmax = -100000;
            nbad = 0;
            for (b = 0; b < 1200; b = b + 1) begin
                if ((b < 600) && ((b % 2) == 0)) begin        // 摄像头格
                    if (blkf[pg][b] < cmin) cmin = blkf[pg][b];
                    if (blkf[pg][b] > cmax) cmax = blkf[pg][b];
                end else begin                                // PC 格
                    if (blkf[pg][b] < pmin) pmin = blkf[pg][b];
                    if (blkf[pg][b] > pmax) pmax = blkf[pg][b];
                end
            end
            if ((cmax - cmin) > 0) nbad = nbad + 1;
            if ((pmax - pmin) > 0) nbad = nbad + 1;
            // ---- 找出 PC 块里"不是本页主帧"的块号范围 ----
            begin : offcalc
                integer omain, o1, o2, oo, olo, ohi;
                o1 = 0; o2 = 0;
                for (b = 0; b < 1200; b = b + 1)
                    if (!((b < 600) && ((b % 2) == 0))) begin
                        if (blkf[pg][b] == pmin) o1 = o1 + 1;
                        if (blkf[pg][b] == pmax) o2 = o2 + 1;
                    end
                omain = (o1 >= o2) ? pmin : pmax;
                oo = 0; olo = 99999; ohi = -1;
                for (b = 0; b < 1200; b = b + 1)
                    if (!((b < 600) && ((b % 2) == 0)))
                        if (blkf[pg][b] != omain) begin
                            oo = oo + 1;
                            if (b < olo) olo = b;
                            if (b > ohi) ohi = b;
                        end
                if (oo != 0)
                    $display("#    -> PC 缺/混 %0d 块, 块号 %0d..%0d  (主帧 %0d)",
                             oo, olo, ohi, omain);
            end
            $display("# [flip %0d @%0dms] page %0d : cam %0d..%0d (%0d块)  pc %0d..%0d (%0d块)  本页新写%0d块 %s",
                     nflip, $time/1000000, pg, cmin, cmax, ncamb,
                     pmin, pmax, npcb, nblk - nprev,
                     (nbad == 0) ? "OK" : "*** MIXED ***");
            nprev = nblk;
            if (nbad != 0) errs = errs + 1;
        end
    endtask

    always @(posedge clk) begin
        if (ad_wr) begin
            nblk = nblk + 1;
            if (awr_is_cam) ncamb = ncamb + 1;
            else            npcb  = npcb  + 1;
        end
        if (rst_n && (wr_done_page !== wd_prev)) begin
            wd_prev = wr_done_page;
            nflip   = nflip + 1;
            pg      = wr_done_page;
            if (nflip >= 2) scan_page;         // 前两页还在暖机
            ncamb = 0;
            npcb  = 0;
        end
    end

    //---- 内部探针: 闸门什么时候关的, 丢字多少次 ----
    reg     gprev = 1;
    integer nskip = 0, nwrap = 0, gap_max = 0, ad_max = 0;
    always @(posedge clk) begin
        if (u_dut.pc_gate !== gprev) begin
            $display("#   GATE %0d->%0d @%0dms  pc_lvl=%0d dst=%0d st=%0d can=%0d",
                     gprev, u_dut.pc_gate, $time/1000000, pw - pr,
                     u_dut.cur_dst, u_dut.st, u_dut.can_start);
            gprev = u_dut.pc_gate;
        end
        if (u_dut.st == 3'd2) nskip = nskip + 1;
        if (u_dut.pc_wrap)    nwrap = nwrap + 1;
        if ((pw - pr) > gap_max) gap_max = pw - pr;
        if (adcnt > ad_max) ad_max = adcnt;
    end

    //=================================================================
    // 激励
    //=================================================================
    localparam integer CAM_BLK_GAP = 7566;    // 45.4ms / 300  -> 22fps
    localparam integer PC_BLK_GAP  = 2600;    // ★故意比翻页慢: 900*2600*20ns=46.8ms > 45.4ms★

    integer k, seq, row, half, dst, t, f;

    // ★两个 task 会被两个并发进程同时调用, 循环变量必须是各自独立的★
    //   Verilog 的 task 内部变量是静态的, 共用 t 会互相踩, 写出来的块长
    //   一会儿 400 一会儿 600, 字流直接乱掉 —— 第一版就是这么假失败的。
    integer tc, tp;

    task put_cam_blk;
        input [10:0] blk;
        begin
            while ((cw - cr) > 3000) @(posedge clk);   // 摄像头那一路的节流
            cfifo[cw % CDEPTH] = {1'b1, 5'd0, blk}; cw = cw + 1;
            for (tc = 0; tc < 512; tc = tc + 1) begin
                cfifo[cw % CDEPTH] = {1'b0, 16'hC0DE};
                cw = cw + 1;
            end
        end
    endtask

    task put_pc_blk;
        input [10:0] blk;
        begin
            while ((pw - pr) > 3500) @(posedge clk);   // 模拟信用窗口
            pfifo[pw % CDEPTH] = {1'b1, 5'd0, blk}; pw = pw + 1;
            for (tp = 0; tp < 512; tp = tp + 1) begin
                pfifo[pw % CDEPTH] = {1'b0, 16'hBEEF};
                pw = pw + 1;
            end
        end
    endtask

    initial begin
        for (b = 0; b < 1200; b = b + 1) begin blkf[0][b] = -1; blkf[1][b] = -1; end
        cam_vs = 0;
        repeat (20) @(posedge clk);
        rst_n = 1;
        repeat (100) @(posedge clk);

        fork
            // --- 摄像头: 每帧 300 块, 块尾来一个 cam_vs ---
            begin : cam_gen
                integer fc;
                for (fc = 0; fc < 6; fc = fc + 1) begin
                    tb_camf = fc;
                    for (k = 0; k < 300; k = k + 1) begin
                        put_cam_blk(k * 2);
                        repeat (CAM_BLK_GAP) @(posedge clk);
                    end
                    cam_vs = 1; @(posedge clk); cam_vs = 0;
                end
            end
            // --- PC: 24fps ---
            begin : pc_gen
                integer fp;
                for (fp = 0; fp < 7; fp = fp + 1) begin
                    tb_pcf = fp;
                    for (seq = 0; seq < 900; seq = seq + 1) begin
                        if (seq < 300) begin
                            row = seq; half = 1;
                        end else begin
                            row  = 300 + (seq - 300) / 2;
                            half = (seq - 300) % 2;
                        end
                        dst = row * 2 + half;
                        put_pc_blk(dst[10:0]);
                        repeat (PC_BLK_GAP) @(posedge clk);
                    end
                end
            end
        join

        repeat (20000) @(posedge clk);
        $display("");
        $display("# ============================================================");
        $display("# flips=%0d mixed=%0d  skip=%0d wrap=%0d pc_lvl_max=%0d ad_max=%0d", nflip, errs, nskip, nwrap, gap_max, ad_max);
        if (errs == 0) $display("# PAGE_ALIGN PASS");
        else           $display("# PAGE_ALIGN FAIL");
        $display("# ============================================================");
        $finish;
    end

endmodule
