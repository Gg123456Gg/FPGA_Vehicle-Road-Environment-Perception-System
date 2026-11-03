//=====================================================================
// udp_video_rx.v   (Verilog-2001)   工作在 gmii_rx_clk(125MHz)
//---------------------------------------------------------------------
// 从 UDP 负载里解析 8 字节头, 输出"带目标地址标签"的 16bit 像素流。
//
// 头格式 (大端, 共 8 字节):
//   magic    [15:0]  固定 0xA55A
//   dst      [15:0]  本包像素要写到 DDR 的哪一块
//                    块号 = 目标行*2 + 半(0=左半,1=右半), 见文档
//   seq      [15:0]  本包在本帧内的序号, ==0 表示"一帧的第一包"
//   npix     [15:0]  本包携带的像素个数 (必须是 NPIX_PKT)
//
// ★v10 关键改动 1: 标签写进同一个 FIFO★
//   和摄像头那边一样, 标签不再是另一个 FIFO, 而是像素流里的一个字:
//        bit16 = 1 -> 标签字, bit[10:0] = 目标块号
//        bit16 = 0 -> 像素
//   这样两块内容永远不会错位。顺序: 标签(T+1) -> 第一个像素(T+3)。
//
// ★v10 关键改动 2: 整包准入 (room)★
//   包头一收齐就先看 FIFO 还剩多少空间。一个包要占 513 个字, 装不下就
//   整包丢掉(一个像素都不发), 绝不"发一半" —— 半包会破坏"块 = 512 像素"
//   的算术, 之后整幅画面被斜着拉成条纹。
//
// ★v10 关键改动 3: pkt_cnt★
//   被接收的包计数(低 16 位), 和 PC 端"已发包数"对比就知道有没有丢包。
//=====================================================================
module udp_video_rx #(
    parameter integer IMG_W    = 1024,
    parameter integer IMG_H    = 600 ,
    parameter integer TOTAL_PX = IMG_W * IMG_H,  // 整帧像素数(全屏模式用)
    parameter integer NPIX_PKT = 512 ,           // 每个包必须携带的像素数
    parameter integer FIFO_AW  = 13              // 像素 FIFO 深度 = 2^FIFO_AW
)(
    input  wire        clk          ,
    input  wire        rst_n        ,
    input  wire        rec_en       ,
    input  wire [7:0]  rec_data     ,
    // FIFO 写域水位(本域), 用来判断"装不装得下一整包"
    input  wire [15:0] fifo_level   ,
    input  wire        fifo_full    ,
    // 写侧: 17bit 字流 (bit16=1 -> 标签字)
    output wire        wr_en        ,
    output wire [16:0] wr_data      ,
    // 诊断: 已接收包数(低 16 位)
    output reg  [15:0] pkt_cnt      ,
    output reg         frame_start  ,
    output reg         frame_done   ,
    output reg  [15:0] frame_id
);
    localparam [15:0] MAGIC  = 16'hA55A;
    // 注意: 不能写 NPIX_PKT[15:0] (对 integer 参数做位选, 有的工具不认)
    localparam [15:0] NPIX_V = NPIX_PKT;
    // 一整包 = 1 标签 + NPIX_PKT 个像素; 再留 8 个字余量
    localparam integer DEPTH    = (1 << FIFO_AW);
    localparam integer ROOM_MAX = DEPTH - NPIX_PKT - 8;

    localparam S_HDR  = 2'd0;
    localparam S_DATA = 2'd1;
    localparam S_SKIP = 2'd2;

    reg  [1:0]  st        ;
    reg  [3:0]  bidx      ;   // 头字节序号 1..7
    reg  [63:0] hdr       ;   // 8 字节头移位寄存器
    reg         phase     ;   // 0=取高字节 1=取低字节
    reg  [7:0]  hi_byte   ;
    reg  [15:0] npix_left ;   // 本包剩余像素
    reg  [31:0] pix_total ;
    reg         rec_en_d  ;
    reg  [10:0] dst_lat   ;   // 本包目标块号
    reg         tag_pend  ;   // 下一拍写一个标签字
    reg         pix_wr    ;   // 下一拍写一个像素
    reg  [15:0] pix_data  ;

    wire [63:0] hdr_next = {hdr[55:0], rec_data};
    wire        room     = ({1'b0, fifo_level} < ROOM_MAX) & ~fifo_full;

    // 写口: 标签优先(标签在 T+1, 本包第一个像素在 T+3, 不会撞车)
    assign wr_en   = tag_pend | pix_wr;
    assign wr_data = tag_pend ? {1'b1, 5'd0, dst_lat} : {1'b0, pix_data};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st          <= S_HDR;
            bidx        <= 4'd0;
            hdr         <= 64'd0;
            phase       <= 1'b0;
            hi_byte     <= 8'd0;
            npix_left   <= 16'd0;
            pix_total   <= 32'd0;
            rec_en_d    <= 1'b0;
            tag_pend    <= 1'b0;
            pix_wr      <= 1'b0;
            pix_data    <= 16'd0;
            pkt_cnt     <= 16'd0;
            frame_start <= 1'b0;
            frame_done  <= 1'b0;
            frame_id    <= 16'd0;
            dst_lat     <= 11'd0;
        end else begin
            tag_pend    <= 1'b0;
            pix_wr      <= 1'b0;
            frame_start <= 1'b0;
            frame_done  <= 1'b0;
            rec_en_d    <= rec_en;

            if (rec_en && !rec_en_d) begin
                // 包起点: 第一个字节就是 magic 的高字节
                st    <= S_HDR;
                bidx  <= 4'd1;
                hdr   <= {56'd0, rec_data};
                phase <= 1'b0;
            end
            else if (rec_en) begin
                if (st == S_HDR) begin
                    if (bidx == 4'd7) begin
                        // 第 8 个字节到齐 -> 本拍就能判定
                        if ((hdr_next[63:48] == MAGIC) &&
                            (hdr_next[15:0]  == NPIX_V) && room) begin
                            frame_id    <= hdr_next[47:32];
                            dst_lat     <= hdr_next[42:32];   // dst[10:0]
                            npix_left   <= hdr_next[15:0];
                            pkt_cnt     <= pkt_cnt + 16'd1;
                            if (hdr_next[31:16] == 16'd0) begin
                                frame_start <= 1'b1;          // 一帧的第一包
                                pix_total   <= 32'd0;
                            end
                            tag_pend  <= 1'b1;                // T+1 写标签
                            st        <= S_DATA;
                            phase     <= 1'b0;
                        end else begin
                            // magic 错 / 包长不对 / FIFO 装不下 -> 整包丢掉
                            //   宁可这一块保留上一帧内容, 也绝不发半包
                            st   <= S_SKIP;
                            bidx <= 4'd0;
                        end
                        hdr <= hdr_next;
                    end else begin
                        hdr  <= hdr_next;
                        bidx <= bidx + 4'd1;
                    end
                end
                else if (st == S_DATA) begin
                    // 负载: 两字节拼一个像素
                    if (!phase) begin
                        hi_byte <= rec_data;
                        phase   <= 1'b1;
                    end else begin
                        phase <= 1'b0;
                        if (npix_left != 16'd0) begin
                            pix_data  <= {hi_byte, rec_data};
                            pix_wr    <= 1'b1;
                            npix_left <= npix_left - 16'd1;
                            pix_total <= pix_total + 32'd1;
                            if (pix_total + 32'd1 == TOTAL_PX)
                                frame_done <= 1'b1;
                        end
                    end
                end
                // S_SKIP: 什么都不做, 等 rec_en 落下去后重新找包起点
            end
        end
    end
endmodule
