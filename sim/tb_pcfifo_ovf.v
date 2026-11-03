//=====================================================================
// tb_pcfifo_ovf.v   复现 "PC FIFO 被灌满后切回来, 字流错位"
//---------------------------------------------------------------------
// 目的: 只验一件事 —— 摄像头全屏(cam_full=1)期间 quad_wr_ctrl 不读 PC,
//       PC FIFO(深度 8192)会被灌满; 切回来重新开始读的那一瞬间,
//       udp_video_rx 的"整包准入"(room)判决是否还可靠。
//
// 判据: 读出来的字流必须是严格的 [标签 + 512 像素] 重复结构。
//       一旦出现 "标签后不足 512 像素" 或 "像素位置吃到标签", 就是错位,
//       错位在板上就是"画面斜着拉成条纹"且永久不恢复。
//
// 用的是真实的 udp_video_rx 和 async_fifo, 没有任何替身。
//=====================================================================
`timescale 1ns/1ps

module tb_pcfifo_ovf;

    // ---- 时钟: 写侧 125MHz(GMII), 读侧 50MHz(搬运) ----
    reg wr_clk = 0;
    reg rd_clk = 0;
    always #4  wr_clk = ~wr_clk;
    always #10 rd_clk = ~rd_clk;

    reg rst_n = 0;

    // ---- udp_video_rx ----
    reg         rec_en   = 0;
    reg  [7:0]  rec_data = 0;
    wire        f_wen;
    wire [16:0] f_wdata;
    wire        f_full;
    wire [13:0] f_wlvl;
    wire [15:0] pkt_cnt;
    wire        f_start, f_done;
    wire [15:0] f_id;

    // ---- async_fifo ----
    wire        f_empty;
    wire [16:0] f_rdata;
    wire        f_ren;
    wire [13:0] f_rlvl;

    udp_video_rx #(
        .IMG_W    (1024),
        .IMG_H    (600 ),
        .TOTAL_PX (614400),
        .NPIX_PKT (512),
        .FIFO_AW  (13)
    ) u_rx (
        .clk        (wr_clk),
        .rst_n      (rst_n),
        .rec_en     (rec_en),
        .rec_data   (rec_data),
        .fifo_level ({2'b0, f_wlvl}),
        .fifo_full  (f_full),
        .wr_en      (f_wen),
        .wr_data    (f_wdata),
        .pkt_cnt    (pkt_cnt),
        .frame_start(f_start),
        .frame_done (f_done),
        .frame_id   (f_id)
    );

    async_fifo #(.DW(17), .AW(13)) u_fifo (
        .rst_n    (rst_n),
        .wr_clk   (wr_clk),
        .wr_en    (f_wen),
        .wr_data  (f_wdata),
        .wr_full  (f_full),
        .wr_level (f_wlvl),
        .rd_clk   (rd_clk),
        .rd_en    (f_ren),
        .rd_data  (f_rdata),
        .rd_empty (f_empty),
        .rd_level (f_rlvl)
    );

    //------------------------------------------------------------------
    // 读侧模型: 搬运器。blocked=1 模拟"摄像头全屏, PC 整路关掉"
    //------------------------------------------------------------------
    reg blocked = 0;
    assign f_ren = rst_n & ~blocked & ~f_empty;

    integer nread  = 0;
    integer nviol  = 0;
    integer ntag   = 0;
    integer npix   = 0;
    integer first_bad_at = -1;
    reg [16:0] first_bad_word;

    reg        expect_tag = 1;   // 1 = 下一个字应该是标签
    reg [8:0]  pix_cnt    = 0;

    // 真实水位(直接看 FIFO 内部指针, 只用于打印)
    wire [13:0] true_lvl = u_fifo.wptr - u_fifo.ld_ptr;

    reg  [13:0] wlvl_min_high = 14'h3FFF;   // 真实水位很高时, 报告水位的最小值
    reg         seen_clamp    = 0;          // 见过"真实满 / 报告 0"

    always @(posedge rd_clk) begin
        if (!rst_n) begin
            expect_tag <= 1'b1;
            pix_cnt    <= 9'd0;
        end else if (!blocked && !f_empty) begin
            nread = nread + 1;
            if (expect_tag) begin
                if (f_rdata[16] !== 1'b1) begin
                    nviol = nviol + 1;
                    if (first_bad_at < 0) begin
                        first_bad_at   = nread;
                        first_bad_word = f_rdata;
                    end
                    // 错位了: 重新找标签
                    expect_tag <= 1'b1;
                    pix_cnt    <= 9'd0;
                end else begin
                    ntag       = ntag + 1;
                    expect_tag <= 1'b0;
                    pix_cnt    <= 9'd0;
                end
            end else begin
                if (f_rdata[16] === 1'b1) begin
                    // 还没吃满 512 个像素就又来标签 -> 前一块被截断
                    nviol = nviol + 1;
                    if (first_bad_at < 0) begin
                        first_bad_at   = nread;
                        first_bad_word = f_rdata;
                    end
                    ntag       = ntag + 1;
                    expect_tag <= 1'b0;
                    pix_cnt    <= 9'd0;
                end else begin
                    npix = npix + 1;
                    pix_cnt <= pix_cnt + 9'd1;
                    if (pix_cnt == 9'd511) expect_tag <= 1'b1;
                end
            end
        end
    end

    // 监控 "真实水位高 但 报告水位极低" 这个病态
    always @(posedge wr_clk) begin
        if (rst_n) begin
            if ((true_lvl > 14'd7000) && (f_wlvl < 14'd64)) begin
                seen_clamp = 1'b1;
                if (f_wlvl < wlvl_min_high) wlvl_min_high = f_wlvl;
            end
        end
    end

    //------------------------------------------------------------------
    // 发送侧模型: 模拟 PC 每包 = 8 字节头 + 1024 字节负载
    //------------------------------------------------------------------
    reg [7:0] payload [0:1023];
    integer   gap_cyc = 24;

    task send_pkt;
        input [10:0] dst;
        integer i;
        begin
            @(posedge wr_clk);
            // 头: magic A55A | frame_id(2) | seq/dst(2) | npix(2)
            rec_en   <= 1'b1;
            rec_data <= 8'hA5; @(posedge wr_clk);
            rec_data <= 8'h5A; @(posedge wr_clk);
            rec_data <= 8'h00; @(posedge wr_clk);
            rec_data <= 8'h01; @(posedge wr_clk);
            rec_data <= {5'd0, dst[10:8]}; @(posedge wr_clk);
            rec_data <= dst[7:0];          @(posedge wr_clk);
            rec_data <= 8'h02; @(posedge wr_clk);
            rec_data <= 8'h00; @(posedge wr_clk);
            for (i = 0; i < 1024; i = i + 1) begin
                rec_data <= payload[i];
                @(posedge wr_clk);
            end
            rec_en   <= 1'b0;
            rec_data <= 8'h00;
            repeat (gap_cyc) @(posedge wr_clk);
        end
    endtask

    integer i, p;
    integer pkt_sent;

    initial begin
        for (i = 0; i < 1024; i = i + 1)
            payload[i] = i[7:0] ^ 8'h3C;

        repeat (10) @(posedge wr_clk);
        rst_n = 1'b1;
        repeat (20) @(posedge wr_clk);

        // ---- 阶段 1: 正常搬运, 每包都被消化 ----
        blocked  = 1'b0;
        gap_cyc  = 200;
        pkt_sent = 0;
        $display("");
        $display("=== 阶段 1: 正常 (边收边搬) ===");
        for (i = 0; i < 8; i = i + 1) begin
            send_pkt(i[10:0]);
            pkt_sent = pkt_sent + 1;
        end
        repeat (4000) @(posedge rd_clk);
        $display("  读完 %0d 字, 标签 %0d 个, 违例 %0d, 报告水位 %0d 真实水位 %0d",
                 nread, ntag, nviol, f_wlvl, true_lvl);

        // ---- 阶段 2: 摄像头全屏 -> PC 整路关掉, FIFO 灌满 ----
        $display("");
        $display("=== 阶段 2: 模拟摄像头全屏, PC 通路被堵 ===");
        blocked = 1'b1;
        gap_cyc = 6;                      // 发送端还在一帧一帧猛发
        for (i = 0; i < 40; i = i + 1) begin
            send_pkt(11'd600 + i[10:0]);
            pkt_sent = pkt_sent + 1;
        end
        $display("  灌完: 报告水位 %0d, 真实水位 %0d, 满标志 %0d, 收到包数 %0d",
                 f_wlvl, true_lvl, f_full, pkt_cnt);

        // ---- 阶段 3: 切回 PC 全屏, 重新开始搬 ----
        $display("");
        $display("=== 阶段 3: 切回来, 重新开始搬 (关键) ===");
        blocked = 1'b0;
        gap_cyc = 6;
        for (i = 0; i < 24; i = i + 1) begin
            send_pkt(11'd1 + i[10:0]);
            pkt_sent = pkt_sent + 1;
        end
        repeat (8000) @(posedge rd_clk);

        $display("");
        $display("---- 结果 ----");
        $display("  发包 %0d, 读字 %0d, 标签 %0d, 像素 %0d", pkt_sent, nread, ntag, npix);
        $display("  结构违例 %0d", nviol);
        if (first_bad_at >= 0)
            $display("  首次违例: 第 %0d 个字 = 0x%05x (bit16=%0d)",
                     first_bad_at, first_bad_word, first_bad_word[16]);
        $display("  出现'真实满/报告水位极低': %0d  (最低报告水位 %0d)",
                 seen_clamp, wlvl_min_high);
        if (nviol == 0) $display("  RESULT: PASS  (字流结构完好)");
        else            $display("  RESULT: FAIL  (字流错位, 板上就是条纹)");
        $display("");
        $finish;
    end

endmodule
