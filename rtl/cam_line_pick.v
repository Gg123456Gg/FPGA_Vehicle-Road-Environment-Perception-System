//=====================================================================
// cam_line_pick.v   (Verilog-2001)   工作在 cam_pclk 域
//---------------------------------------------------------------------
// 摄像头抽取 + "带地址标签"的像素流输出。
//
// 1) 两种输出比例 (v23 新增 cam_full)
//    四宫格  (cam_full=0): 只保留"偶数行 + 偶数列", 1024x600 -> 512x300,
//                          正好是四宫格里左上那一格。每行压 1 个标签。
//    摄像头全屏(cam_full=1): 一点都不抽, 1024x600 原样送出去, 每行压 2 个
//                          标签(左半块 2r / 右半块 2r+1), 正好铺满 1200 块。
//                          ★摄像头本来就是 1024x600 原生输出★ 所以全屏
//                          不需要任何缩放, 而且比四宫格里那格清晰一倍。
//
// 2) ★v10 关键改动: 标签写进同一个 FIFO★
//    v9 是"像素一个 FIFO + 标签另一个 FIFO", 两者靠"行首压标签"保持 1:1。
//    问题是: 只要像素 FIFO 因为任何原因丢了几个像素(比如满), 两边的
//    对应关系就永久错开, 之后每一块都少几个像素、误差逐块累积 ——
//    上板看到的就是"画面被斜着拉成条纹"。
//    现在标签和像素走同一条 17bit 通道:
//        bit16 = 1  -> 这一拍是标签, bit[10:0] = 目标块号
//        bit16 = 0  -> 这一拍是像素
//    一条通道里不可能错位; 丢也是整行丢(见 room 判决)。
//
// 3) ★v23 全屏的"同拍冲突"怎么解决★
//    全屏一行 1024 像素, 却要写 1026 个字(2 标签 + 1024 像素)。标签必须
//    严格插在 p511 和 p512 之间, 而这两个像素是背靠背产生的 —— 同一拍
//    要写两个东西, 而 FIFO 只有一组写口。
//    做法: 全屏模式下出口串一个 4 深小队列, 每拍固定吐出 1 个字。撞车的
//    那一拍多出来的那个先进队列, 之后每拍"出 1 入 1"保持 1 个积压, 到行
//    末消隐期自然排空。★一个像素都不会丢, 也不会错位★
//    四宫格模式走原来的零延迟组合直出, 行为与 v10 完全一致(回归零风险)。
//
// 4) 顺序保证(为什么像素要延迟两级)
//    行 k 的最后一个像素(T-2 拍取)在第 T 拍写进 FIFO;
//    标签在第 T+1 拍写; 行 k+1 的第一个像素(T 拍取)在第 T+2 拍写。
//    天然有序: [上一行像素][标签][本行 512 像素]。
//
// 5) 只用 valid + vsync, 不依赖 href
//    上板遥测显示: 场同步 22 次/秒, 但 href 一直是 0 (41 例程里也是空接的),
//    一旦把 href 掺进判决条件, 摄像头一个像素都写不进来。
//=====================================================================
module cam_line_pick(
    input  wire        cam_pclk        ,
    input  wire        rst_n           ,
    // v23: 1 = 摄像头全屏(1024x600 原样送出, 1200 块)
    //      0 = 四宫格  (抽成 512x300, 300 块)
    input  wire        cam_full        ,
    // 来自 ov5640_dri (已是 RGB565, 每像素一个 valid 脉冲)
    input  wire        cmos_frame_vsync,
    input  wire        cmos_frame_href ,
    input  wire        cmos_frame_valid,
    input  wire [15:0] cmos_frame_data ,
    // FIFO 写域水位 (本域), 用来看"还剩几行空间"
    input  wire [12:0] fifo_level      ,
    input  wire        fifo_full       ,
    // FIFO 写口 (17bit: bit16=1 表示标签字)
    output wire        fifo_wr_en      ,
    output wire [16:0] fifo_wr_data    ,
    // 诊断: {href_seen, valid_seen, emit, tag_seen}
    output wire [3:0]  dbg
);

    localparam [10:0] H_PIXEL   = 11'd1024;   // 摄像头一行的有效像素
    // 只有 FIFO 里还不到 4 行(2048)时才接收新的一行:
    //   四宫格: 一行 = 1 标签 + 512 像素 = 513 字, 收完最多 2561
    //   全屏  : 一行 = 2 标签 + 1024 像素 = 1026 字, 收完最多 3074
    // 两者都远小于深度 4096, 所以这个门限不用按模式分。
    localparam [12:0] LEVEL_MAX = 13'd2048;
    // 全屏一行压 2 个标签, 右半标签插在第 512 个像素之前
    localparam [10:0] HALF_PIX  = 11'd512;

    //------------------------------------------------------------------
    // 全屏标志从 clk_50m 域同步过来 (两级, 慢变信号, 不做边沿检测)
    //------------------------------------------------------------------
    reg  fm_s0, fm_s1;
    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin fm_s0 <= 1'b0; fm_s1 <= 1'b0; end
        else        begin fm_s0 <= cam_full; fm_s1 <= fm_s0; end
    end

    // ★v25 关键修复: 模式只在帧边界(vsync 上升沿)才采信★
    //   原来直接用电平 fm_s1, 按键在"一行搬到一半"时换模式, 这一行的前半截
    //   按旧模式出字、后半截按新模式出字:
    //     四宫格 -> 全屏: 行首那个标签是按四宫格压的(块号 = (row/2)*2),
    //       可像素突然变成 1024 个, 一块装不下 -> 写侧的"1 标签 + 512 像素"
    //       节奏从这一行起全乱, 之后每个块号都取到像素值(垃圾块号),
    //       屏幕上就是"画面分裂成长条"; 全屏 -> 四宫格同理。
    //   更糟的是出口那个 4 深小队列在 fm_chg 时被清空, 里面压着的那 1 个字
    //   直接丢掉 —— 又额外少一个字, 错位再 +1, 而且永远没有机会纠正。
    //   改成帧边界锁存以后, 任何一帧从头到尾只属于一个模式, 节奏恒定:
    //     四宫格 每行 1+512 = 513 字, 全屏 每行 2*(1+512) = 1026 字,
    //   两者都是 513 的整数倍, 所以帧与帧交界处也不会错位。
    //   代价: 换模式最多晚一帧(约 45ms)生效, 肉眼看不出来。
    reg  fmode_r;
    wire fm_sync = fm_s1;
    wire fmode   = fmode_r;

    reg        vsync_d;
    reg [10:0] col_cnt;         // 行内像素计数(按 valid 计), 0..1023
    reg [9:0]  row_cnt;         // 行计数 0..599
    reg        emit;            // 这一行是否在收
    reg        tag_pend;        // 下一拍要压一个标签字
    reg [10:0] tag_val;         // 标签字里的块号

    // 像素延迟两级(见文件头第 4 条)
    reg        take_d1, take_d2;
    reg [15:0] data_d1, data_d2;

    reg        valid_seen;      // 诊断: 本帧出现过 valid
    reg        href_seen;       // 诊断: 本帧出现过 href (仅参考)
    reg        tag_seen;        // 诊断: 本帧发过标签

    wire vsync_rise = cmos_frame_vsync & ~vsync_d;
    wire at_row_0   = (col_cnt == 11'd0);
    // 全屏一行 1024 列全要; 四宫格只要偶数列、偶数行
    wire row_ok     = fmode ? 1'b1 : ~row_cnt[0];
    wire col_ok     = fmode ? 1'b1 : ~col_cnt[0];
    // 一次要占 513 个字(全屏是 1026, 门限已留够)
    wire room       = (fifo_level < LEVEL_MAX) & ~fifo_full;

    // 一行的第一个像素处做"收/不收"的决定
    wire start_row  = cmos_frame_valid & at_row_0 & row_ok & room;
    // 收的行里, 该收的列才要
    wire take       = cmos_frame_valid & row_ok & col_ok & (emit | start_row);

    //------------------------------------------------------------------
    // 标签产生点
    //   行首          : 四宫格 / 全屏的左半块
    //   第 512 个像素 : 只有全屏才有, 作为右半块的分界
    //------------------------------------------------------------------
    wire tag_at_head = at_row_0 & start_row;
    wire tag_at_half = fmode & cmos_frame_valid & emit & (col_cnt == HALF_PIX);

    //------------------------------------------------------------------
    // 全屏写出口: 4 深小队列 (只有 fmode=1 时启用)
    //   入队顺序: 像素在前, 标签在后 —— 必须这样, 否则右半标签会跑到
    //   左半块最后一个像素前面(见文件头第 3 条的场景)。
    //------------------------------------------------------------------
    reg  [16:0] sq [0:3];
    reg  [2:0]  sq_w, sq_r, sq_n;
    reg         q_we;
    reg  [16:0] q_wd;

    wire [16:0] px_dat  = {1'b0, data_d2};
    wire [16:0] tag_dat = {1'b1, 5'd0, tag_val};
    wire        q_px_in = fmode & take_d2;
    wire        q_tg_in = fmode & tag_pend;
    wire [2:0]  q_in_n  = {2'b0, q_tg_in} + {2'b0, q_px_in};
    wire [2:0]  q_out_n = (sq_n != 3'd0) ? 3'd1 : 3'd0;
    // 标签的入队位置: 像素先占一个
    wire [2:0]  q_w2    = sq_w + (q_px_in ? 3'd1 : 3'd0);

    // 模式切换时把队列清干净, 免得旧模式的残留字混进新的一帧
    reg  fm_d;
    wire fm_chg = fmode ^ fm_d;

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            sq_w <= 3'd0; sq_r <= 3'd0; sq_n <= 3'd0;
            q_we <= 1'b0; q_wd <= 17'd0;
        end else begin
            q_we <= 1'b0;
            if (fm_chg) begin
                sq_w <= 3'd0; sq_r <= 3'd0; sq_n <= 3'd0;
            end else begin
                // ---- 出队: 每拍固定吐 1 个字 ----
                if (sq_n != 3'd0) begin
                    q_we <= 1'b1;
                    q_wd <= sq[sq_r[1:0]];      // ★只取低 2 位当地址★
                    sq_r <= sq_r + 3'd1;
                end
                // ---- 入队: 最多 2 个, 像素在前 ----
                if (q_px_in) sq[sq_w[1:0]]  <= px_dat;
                if (q_tg_in) sq[q_w2[1:0]]  <= tag_dat;
                sq_w <= sq_w + q_in_n;
                sq_n <= sq_n - q_out_n + q_in_n;
            end
        end
    end

    //---- 出口选择: 四宫格走原来的零延迟组合直出, 全屏走队列 ----
    assign fifo_wr_en   = fmode ? q_we : (tag_pend | take_d2);
    assign fifo_wr_data = fmode ? q_wd
                                : (tag_pend ? tag_dat : px_dat);

    always @(posedge cam_pclk or negedge rst_n) begin
        if (!rst_n) begin
            vsync_d    <= 1'b0;
            col_cnt    <= 11'd0;
            row_cnt    <= 10'd0;
            emit       <= 1'b0;
            tag_pend   <= 1'b0;
            tag_val    <= 11'd0;
            take_d1    <= 1'b0;
            take_d2    <= 1'b0;
            data_d1    <= 16'd0;
            data_d2    <= 16'd0;
            valid_seen <= 1'b0;
            href_seen  <= 1'b0;
            tag_seen   <= 1'b0;
            fm_d       <= 1'b0;
            fmode_r    <= 1'b0;
        end else begin
            vsync_d  <= cmos_frame_vsync;
            fm_d     <= fmode;
            take_d1  <= take;
            take_d2  <= take_d1;
            if (take)    data_d1 <= cmos_frame_data;
            if (take_d1) data_d2 <= data_d1;

            tag_pend <= 1'b0;               // 默认只压一拍

            if (vsync_rise) begin
                col_cnt    <= 11'd0;
                row_cnt    <= 10'd0;
                emit       <= 1'b0;
                valid_seen <= 1'b0;
                href_seen  <= 1'b0;
                tag_seen   <= 1'b0;
                fmode_r    <= fm_sync;      // ★v25: 新模式从下一行(这一帧)开始生效
            end else if (cmos_frame_valid) begin
                // ---- 行列计数: 以 valid 为像素节拍 ----
                if (col_cnt == H_PIXEL - 11'd1) begin
                    col_cnt <= 11'd0;
                    row_cnt <= row_cnt + 10'd1;
                end else begin
                    col_cnt <= col_cnt + 11'd1;
                end

                // ---- 行首: 决定这一行收不收 ----
                if (at_row_0)
                    emit <= start_row;

                // ---- 压标签 ----
                //   四宫格: 块号 = (源行/2)*2 = 目标行*2, 左半
                //   全屏  : 左半 = 源行*2, 右半 = 源行*2 + 1
                if (tag_at_head | tag_at_half) begin
                    tag_pend <= 1'b1;
                    tag_val  <= fmode ? {row_cnt, tag_at_half}
                                      : {1'b0, row_cnt[9:1], 1'b0};
                    tag_seen <= 1'b1;
                end
            end

            // ---- 诊断 sticky ----
            if (cmos_frame_valid) valid_seen <= 1'b1;
            if (cmos_frame_href)  href_seen  <= 1'b1;
        end
    end

    assign dbg = {href_seen, valid_seen, emit, tag_seen};

endmodule
