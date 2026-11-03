//**************************************************************************//
// File name   : eth_ddr3_lcd.v
// Descriptions: 四宫格显示顶层
//               - 左上 512x300 : OV5640 摄像头 (抽取缩放 1024x600 -> 512x300)
//               - 其余三格     : PC 通过千兆以太网送来的同一路视频
//               - 只有一片 DDR3, 只有一个顺序写口, 所以两路必须先"合并"成
//                 一路按扫描顺序的像素流, 再写进 DDR3。
//
//  改动说明 (相对原 55 例程):
//   1) 新增摄像头链路: ov5640_dri -> cam_line_pick -> async_fifo(cam)
//   2) 新增 PC 链路缓冲: udp_video_rx -> async_fifo(pc)
//   3) 新增 quad_wr_ctrl: 两路合成一路, 喂 ddr3_ctrl_top 写口
//   4) wd_clk 从 gmii_rx_clk(125M) 改为 clk_50m:
//      让合并逻辑全部落在 50MHz 单一域, 不再有第三个时钟域
//   5) wr_load 置 0: 帧边界交给 ddr3_rw_ctrl 的自然翻页(写满 614400 自动
//      回零并翻页), 这样乒乓一定成对, 不会只写不翻
//   6) mem_a 仍是 [14:0], 和 55 例程的 DDR3 IP 保持一致
//**************************************************************************//
module eth_ddr3_lcd(
    input                  sys_clk     ,  // FPGA 外部时钟 50MHz
    input                  rst_n       ,  // 复位, 低有效
    input       [3:0]      key         ,  // 4 个用户按键(低有效): k1切屏 k2直行 k3左变 k4右变
    output      [3:0]      led         ,  // 4 个 LED(高有效): led[0]=报警灯, 其余保留
    // 以太网 RGMII
    input                  eth_rxc     ,
    input                  eth_rx_ctl  ,
    input       [3:0]      eth_rxd     ,
    output                 eth_txc     ,
    output                 eth_tx_ctl  ,
    output      [3:0]      eth_txd     ,
    // OV5640 摄像头 (DVP)
    input                  cam_pclk    ,
    input                  cam_vsync   ,
    input                  cam_href    ,
    input       [7:0]      cam_data    ,
    output                 cam_rst_n   ,
    output                 cam_pwdn    ,
    output                 cam_scl     ,
    inout                  cam_sda     ,
    // DDR3
    output                 mem_rst_n   ,
    output                 mem_ck      ,
    output                 mem_ck_n    ,
    output                 mem_cke     ,
    output                 mem_ras_n   ,
    output                 mem_cas_n   ,
    output                 mem_we_n    ,
    output                 mem_odt     ,
    output                 mem_cs_n    ,
    output      [14:0]     mem_a       ,
    output      [2:0]      mem_ba      ,
    inout       [1:0]      mem_dqs     ,
    inout       [1:0]      mem_dqs_n   ,
    inout       [15:0]     mem_dq      ,
    output      [1:0]      mem_dm      ,
    // LCD
    output                 lcd_hs      ,
    output                 lcd_vs      ,
    output                 lcd_de      ,
    inout       [23:0]     lcd_rgb     ,
    output                 lcd_bl      ,
    output                 lcd_rst     ,
    output                 lcd_pclk
);

//------------------------------------------------------------------
// 模式开关
//   1 = 四宫格 (摄像头 + PC)   <- 本次目标
//   0 = 老行为 (PC 全屏), 出问题时可一键退回
//------------------------------------------------------------------
localparam QUAD_MODE = 1'b1;

//------------------------------------------------------------------
// 摄像头参数 (面板固定 1024x600, 这里直接写死, 不依赖 LCD ID 读取结果)
//   total_h/v 是 OV5640 的 HTS/VTS, 决定帧率
//   y_addr_st/end 是 ISP 开窗
//------------------------------------------------------------------
localparam [12:0] CAM_H_PIXEL   = 13'd1024;
localparam [12:0] CAM_V_PIXEL   = 13'd600;
localparam [12:0] CAM_TOTAL_H   = 13'd2200;
localparam [12:0] CAM_TOTAL_V   = 13'd1000;
localparam [12:0] CAM_Y_ADDR_ST = 13'd201;
localparam [12:0] CAM_Y_ADDR_END= 13'd1749;

// PC 每帧实际发来的像素数 = 300*512 (上半区右半) + 300*1024 (下半区)
localparam integer PC_PIX_FRAME  = 460800;

//parameter define
parameter  BOARD_MAC = 48'h00_11_22_33_44_55;
parameter  BOARD_IP  = {8'd192,8'd168,8'd1,8'd10};
parameter  DES_MAC   = 48'hff_ff_ff_ff_ff_ff;
parameter  DES_IP    = {8'd192,8'd168,8'd1,8'd102};

//------------------------------------------------------------------
// 时钟 / 复位
//------------------------------------------------------------------
wire         gmii_rx_clk   ;   // 以太网 125M
wire         clk_50m       ;   // PLL 输出 50M
wire         locked        ;
wire         lcd_clk       ;   // LCD 驱动时钟 (0x7016 时等于 50M)
wire         sys_rst_n     ;

assign sys_rst_n = rst_n & locked;

clk_wiz_0 u_clk_wiz_0 (
    .pll_rst  (1'b0    ),
    .clkin1   (sys_clk ),
    .pll_lock (locked  ),
    .clkout0  (clk_50m )
);

//------------------------------------------------------------------
// DDR3 相关
//------------------------------------------------------------------
wire         ddr3_init_done ;
wire  [15:0] rd_data        ;
wire         rdata_req      ;
wire         rd_vsync       ;
wire  [15:0] lcd_id         ;
wire  [12:0] h_disp         ;
wire  [27:0] ddr3_addr_max  ;
wire  [11:0] pixel_xpos     ;
wire  [11:0] pixel_ypos     ;

wire  [15:0] ddr_wd_data    ;
wire         ddr_wd_en      ;

// 四宫格布局是写死 1024x600 的, 所以 DDR 侧的分辨率必须和它一致,
// 不能跟着 LCD ID 读取结果走(读错一次整个画面就散了)。
assign ddr3_addr_max = 28'd614400;   // 1024*600
assign h_disp        = 13'd1024;

//------------------------------------------------------------------
// 摄像头采集
//------------------------------------------------------------------
wire         cmos_frame_vsync ;
wire         cmos_frame_href  ;
wire         cmos_frame_valid ;
wire  [15:0] cmos_frame_data  ;
wire         cam_init_done    ;

ov5640_dri u_ov5640_dri(
    .clk               (clk_50m        ),
    .rst_n             (sys_rst_n      ),
    .cam_pclk          (cam_pclk       ),
    .cam_vsync         (cam_vsync      ),
    .cam_href          (cam_href       ),
    .cam_data          (cam_data       ),
    .cam_rst_n         (cam_rst_n      ),
    .cam_pwdn          (cam_pwdn       ),
    .cam_scl           (cam_scl        ),
    .cam_sda           (cam_sda        ),
    .cmos_h_pixel      (CAM_H_PIXEL    ),
    .cmos_v_pixel      (CAM_V_PIXEL    ),
    .total_h_pixel     (CAM_TOTAL_H    ),
    .total_v_pixel     (CAM_TOTAL_V    ),
    .y_addr_st         (CAM_Y_ADDR_ST  ),
    .y_addr_end        (CAM_Y_ADDR_END ),
    .capture_start     (ddr3_init_done ),
    .cam_init_done     (cam_init_done  ),
    .cmos_frame_vsync  (cmos_frame_vsync),
    .cmos_frame_href   (cmos_frame_href ),
    .cmos_frame_valid  (cmos_frame_valid),
    .cmos_frame_data   (cmos_frame_data )
);

//------------------------------------------------------------------
// 摄像头抽取 -> 字流 FIFO (17bit: bit16=1 表示"这是标签字")
//   v10: 标签和像素进同一个 FIFO, 从结构上排除错位
//------------------------------------------------------------------
// v23: 这一位决定摄像头走"抽取成 512x300"还是"1024x600 原样铺满"。它由下面
//      按键那一段产生, 但例化在这里就要用到, 所以先声明、后面再 assign。
wire         cam_full      ;
wire         cam_fifo_wr   ;
wire  [16:0] cam_fifo_wdata;
wire         cam_fifo_full ;
wire  [12:0] cam_wr_level;   // 写域水位(给 cam_line_pick 判空间)
wire  [12:0] cam_rd_level;   // 读域水位(给搬运器判"够不够一整块")
wire  [16:0] cam_rdata     ;
wire         cam_empty     ;
wire         cam_rd        ;
wire  [3:0]  cam_dbg       ;

cam_line_pick u_cam_line_pick(
    .cam_pclk          (cam_pclk        ),
    .rst_n             (sys_rst_n       ),
    // v23: 1 = 1024x600 原样送出(摄像头全屏), 0 = 抽成 512x300(四宫格)
    .cam_full          (cam_full        ),
    .cmos_frame_vsync  (cmos_frame_vsync),
    .cmos_frame_href   (cmos_frame_href ),
    .cmos_frame_valid  (cmos_frame_valid),
    .cmos_frame_data   (cmos_frame_data ),
    .fifo_level        (cam_wr_level    ),
    .fifo_full         (cam_fifo_full   ),
    .fifo_wr_en        (cam_fifo_wr     ),
    .fifo_wr_data      (cam_fifo_wdata  ),
    .dbg               (cam_dbg         )
);

// 深度 4096 = 8 行(每行 513 字: 1 标签 + 512 像素)
async_fifo #(.DW(17), .AW(12)) u_cam_fifo(
    .rst_n    (sys_rst_n      ),
    .wr_clk   (cam_pclk       ),
    .wr_en    (cam_fifo_wr    ),
    .wr_data  (cam_fifo_wdata ),
    .wr_full  (cam_fifo_full  ),
    .wr_level (cam_wr_level   ),
    .rd_clk   (clk_50m        ),
    .rd_en    (cam_rd         ),
    .rd_data  (cam_rdata      ),
    .rd_empty (cam_empty      ),
    .rd_level (cam_rd_level   )
);

//------------------------------------------------------------------
// v34: 车道线识别已经挪进 quad_wr_ctrl.v (clk_50m 域, 直接吃 PC 像素)
//   所以这里不再有 cam_pclk 域的 lane_detect, 也不再需要 lane_cdc。
//   摄像头画面照旧显示, 只是不再对它做任何识别(按 11-03 的决定)。
//------------------------------------------------------------------

//------------------------------------------------------------------
// 以太网收包 -> 字流 (同样是 17bit 的标签+像素)
//------------------------------------------------------------------
wire  [7:0]  rec_data ;
// v24: rec_en 以前一直是隐式 net (第一次出现在 udp_video_rx 的端口连接里)。
// 现在命令解码器要提前用它, 显式声明一下, 免得两个模块各自把它当隐式网线。
wire         rec_en   ;

wire         vid_wr_en     ;
wire  [16:0] vid_wr_data   ;
wire         vid_frame_start;
wire         vid_frame_done;
wire  [15:0] vid_frame_id  ;
wire  [15:0] vid_pkt_cnt   ;

wire         pc_fifo_wr    ;
wire         pc_fifo_full  ;
wire  [13:0] pc_wr_level   ;
wire  [16:0] pc_rdata      ;
wire         pc_empty      ;
wire         pc_rd         ;
wire  [13:0] pc_level      ;

udp_video_rx #(
    .IMG_W   (1024           ),
    .IMG_H   (600            ),
    .TOTAL_PX(PC_PIX_FRAME   ),
    .NPIX_PKT(512            ),
    .FIFO_AW (13             )
) u_udp_video_rx (
    .clk         (gmii_rx_clk   ),
    .rst_n       (sys_rst_n     ),
    .rec_en      (rec_en        ),
    .rec_data    (rec_data      ),
    .fifo_level  ({2'b0, pc_wr_level}),
    .fifo_full   (pc_fifo_full  ),
    .wr_en       (vid_wr_en     ),
    .wr_data     (vid_wr_data   ),
    .pkt_cnt     (vid_pkt_cnt   ),
    .frame_start (vid_frame_start),
    .frame_done  (vid_frame_done),
    .frame_id    (vid_frame_id  )
);

assign pc_fifo_wr  = QUAD_MODE ? vid_wr_en : 1'b0;

// 深度 8192 = 16 包(每包 513 字)
async_fifo #(.DW(17), .AW(13)) u_pc_fifo(
    .rst_n    (sys_rst_n   ),
    .wr_clk   (gmii_rx_clk ),
    .wr_en    (pc_fifo_wr  ),
    .wr_data  (vid_wr_data ),
    .wr_full  (pc_fifo_full),
    .wr_level (pc_wr_level ),
    .rd_clk   (clk_50m     ),
    .rd_en    (pc_rd       ),
    .rd_data  (pc_rdata    ),
    .rd_empty (pc_empty    ),
    .rd_level (pc_level    )
);


//------------------------------------------------------------------
// 两路带地址搬运 (v9)
//------------------------------------------------------------------
wire [15:0] q_wd_data;
wire        q_wd_en  ;
wire [27:0] ad_data  ;
wire        ad_wr    ;
wire [4:0]  ad_level ;
wire [31:0] q_cam_px ;
wire [31:0] q_pc_px  ;
wire [15:0] q_chunk  ;
wire        q_busy   ;
wire        wr_page  ;
wire        wr_done_page;

// DDR3 初始化完成信号跨到 50M 域
reg init_d0, init_d1;
always @(posedge clk_50m or negedge sys_rst_n)
    if (!sys_rst_n) begin init_d0 <= 1'b0; init_d1 <= 1'b0; end
    else begin init_d0 <= ddr3_init_done; init_d1 <= init_d0; end
wire init_sync = init_d1;

// 摄像头场同步 -> 50MHz 域 1 拍脉冲 (用于换页)
reg vs_d, vs_tog;
always @(posedge cam_pclk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin vs_d <= 1'b0; vs_tog <= 1'b0; end
    else begin
        vs_d <= cmos_frame_vsync;
        if (cmos_frame_vsync & ~vs_d) vs_tog <= ~vs_tog;
    end
end

reg vs_s0, vs_s1, vs_s2;
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n) begin vs_s0 <= 1'b0; vs_s1 <= 1'b0; vs_s2 <= 1'b0; end
    else begin vs_s0 <= vs_tog; vs_s1 <= vs_s0; vs_s2 <= vs_s1; end
end
wire cam_vs = vs_s1 ^ vs_s2;

reg [31:0] cam_vs_cnt;
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n) cam_vs_cnt <= 32'd0;
    else if (cam_vs) cam_vs_cnt <= cam_vs_cnt + 32'd1;
end

//------------------------------------------------------------------
// 4 个按键消抖 + 功能分配 [v39]
//------------------------------------------------------------------
//   key[0] (V12) = k1 : 三态显示循环 (切屏)
//   key[1] (T12) = k2 : 模拟强制直行
//   key[2] (N9)  = k3 : 模拟强制左变道
//   key[3] (M10) = k4 : 模拟强制右变道
//
//   ★道路识别功能常开★ 原来的 key[1] 是"车道线图层开关", 现在去掉这个开关,
//   车道识别一直运行、一直叠加显示, key[1] 改成强制直行。
//
// key 是机械按键, 按下去会抖 5~10ms, 直接拿来计数会把一次按压当成几十次。
// 所以每个按键:
//   1) 两级触发器把异步信号同步到 50MHz 域 (不然会亚稳态);
//   2) 电平连续保持 20ms 不变才承认它 (50MHz x 0.02 = 1,000,000 拍);
//   3) 只认"按下"的那一个下降沿 (低有效: 1->0), 松手不动作。
//------------------------------------------------------------------
reg [3:0] key_s0, key_s1;
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n) begin key_s0 <= 4'hF; key_s1 <= 4'hF; end
    else begin key_s0 <= key; key_s1 <= key_s0; end
end

reg [19:0] key_cnt;
reg [3:0]  key_st;              // 消抖之后的稳定电平
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        key_cnt <= 20'd0;
        key_st  <= 4'hF;        // 上拉, 没按 = 1
    end else if (key_s1 != key_st) begin
        if (key_cnt == 20'd999999) begin
            key_cnt <= 20'd0;
            key_st  <= key_s1;
        end else
            key_cnt <= key_cnt + 20'd1;
    end else
        key_cnt <= 20'd0;       // 中途弹回来就重新计时
end

reg [3:0] key_d;
wire [3:0] key_press = key_d & ~key_st;     // "按下"那一下, 只高一拍
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n) key_d <= 4'hF;
    else            key_d <= key_st;
end

// 各按键单独取边沿 (顶层其余代码沿用 key_press[0] 作为切屏键)
wire k1_press = key_press[0];   // 切屏
wire k2_press = key_press[1];   // 强制直行
wire k3_press = key_press[2];   // 强制左变道
wire k4_press = key_press[3];   // 强制右变道

//------------------------------------------------------------------
// v39: 模拟强制命令 (k2/k3/k4), 三个互斥
//   每颗键是"按住才有效"还是"按一下锁存"? 这里做成【按住持续有效】——
//   松手即恢复自动决策。演示时按住 k2 就能看到车"被要求直行", 松手回到
//   正常识别, 最直观。三键同时按住时优先级: 直行 > 左变道 > 右变道。
//------------------------------------------------------------------
wire force_go = k2_press | (~key_st[1]);   // k2 按住 = 强制直行
wire force_l  = (k3_press | (~key_st[2])) & ~force_go;
wire force_r  = (k4_press | (~key_st[3])) & ~force_go & ~force_l;

// 道路识别常开 (原 key2 开关功能取消)
//   v51: PC 面板上的开关只关【显示】, 不关识别 —— 见下面的 lane_osd_en。
//        检测流水在 v51 起恒开, 面板开关只决定 OSD 那层画不画。
//        这个 lane_en 保留为"图层总闸", 恒 1。
wire lane_en = 1'b1;

//------------------------------------------------------------------
// 算法档位: 由 PC 的 UDP 命令写入        [v24]
//------------------------------------------------------------------
// 命令包负载 4 字节 (端口随便 —— 板子这一层根本不解析端口):
//   [0..1] 0xA5 0x5B   视频包是 0xA5 0x5A, 靠头两字节区分
//   [2]    档位 0..4   0 彩色 1 灰度 2 去噪 3 二值化 4 YCbCr
//   [3]    ~档位       单比特校验
//
// cmd_tog 在 125MHz 域翻转, 这里两级同步后取边沿, 边沿那一拍锁存档位。
// 用翻转不用单拍脉冲: 125MHz 一个脉冲只有 8ns, 而 50MHz 采样周期 20ns,
// 单拍脉冲会被整拍错过。
//
// mode_r 只在翻页时被 mode_lat 采样, 所以切档位有约一帧的延迟 —— 这是故意的,
// 一帧里换算法会让画面上半截和下半截不是一个样子。
//------------------------------------------------------------------
wire       cmd_tog;
wire [2:0] cmd_mode;
wire       lane_cmd_tog;      // v50: 车道识别【显示】开关命令 (0xA55C)
wire       lane_cmd_on;
udp_cmd_dec u_udp_cmd_dec(
    .clk      (gmii_rx_clk ),
    .rst_n    (sys_rst_n   ),
    .rec_en   (rec_en      ),
    .rec_data (rec_data    ),
    .cmd_tog  (cmd_tog     ),
    .cmd_mode (cmd_mode    ),
    .lane_tog (lane_cmd_tog),
    .lane_on  (lane_cmd_on )
);

reg       cmd_s0, cmd_s1, cmd_s2;
reg [2:0] mode_r;               // 算法档位 0..4
// v50: 车道识别【显示】开关。上电默认开 —— 和 v49 完全一致, 不接 PC 也能正常演示。
//      125MHz 域翻转标志 -> 50MHz 域两级同步后取边沿, 与 mode_r 同一套做法。
//      v51 起它只门控 OSD 那一层, 检测流水常开(见 quad_wr_ctrl 的 lane_px_en)。
reg       lane_s0, lane_s1, lane_s2;
reg       lane_osd_en;
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        cmd_s0 <= 1'b0;
        cmd_s1 <= 1'b0;
        cmd_s2 <= 1'b0;
        mode_r <= 3'd0;         // 上电: 彩色
        lane_s0 <= 1'b0;
        lane_s1 <= 1'b0;
        lane_s2 <= 1'b0;
        lane_osd_en <= 1'b1;    // 上电: 显示
    end else begin
        cmd_s0 <= cmd_tog;
        cmd_s1 <= cmd_s0;
        cmd_s2 <= cmd_s1;
        if (cmd_s1 ^ cmd_s2)    // cmd_tog 翻过一次 = 收到一条有效命令
            mode_r <= cmd_mode;

        lane_s0 <= lane_cmd_tog;
        lane_s1 <= lane_s0;
        lane_s2 <= lane_s1;
        if (lane_s1 ^ lane_s2)
            lane_osd_en <= lane_cmd_on;
    end
end
wire [2:0] mode_sel = mode_r;

//------------------------------------------------------------------
// ★v23 三态显示★ 按键 1 循环切换, 上电默认 PC 全屏:
//     0 = PC 全屏      整屏 1024x600 全是 PC 视频 (上电默认)
//     1 = 四宫格       左上 OV5640 + 其余三格 PC 视频
//     2 = 摄像头全屏   整屏 1024x600 全是摄像头实时画面
//   循环顺序 0 -> 1 -> 2 -> 0。
//   摄像头本来就是 1024x600 原生输出, 所以全屏这一档不需要任何缩放。
//------------------------------------------------------------------
localparam [1:0] D_PCFULL = 2'd0;
localparam [1:0] D_QUAD   = 2'd1;
localparam [1:0] D_CAMFUL = 2'd2;

reg [1:0] disp_mode;

//------------------------------------------------------------------
// ★v34 全屏断流 / 恢复的自动切换★
//   赛题是"给一段视频做自主决策", 所以 PC 全屏是唯一的主画面。
//   ① PC 不发了  -> 直接切【摄像头全屏】, 不再退回四宫格
//      (v16~v33 是退四宫格, 现场反馈"想看摄像头就该满屏, 四宫格没法用")
//   ② PC 又发了  -> 自动回 PC 全屏, 不用按键
//      (只在"自动切过去"的情况下才自动切回; 用户手动按键进的摄像头全屏不会被抢)
//   ③ 上电就没视频 -> 1s 后同样落到摄像头全屏, 不至于黑屏等着
//
//   判据用的是【收包侧】的像素活动, 不是 pc_rd:
//     摄像头全屏时写通路根本不调度 PC, pc_rd 恒为 0, 拿它判断"PC 还活着"
//     会永远判死。所以在 gmii_rx_clk 域数 vid_wr_en, 每 2048 像素翻一位,
//     到 clk_50m 域检测翻转(两级同步 + 边沿), 只要还在收包就有翻转。
//   不想要这个行为, 把 AUTO_BACK 改成 1'b0 即可。
//------------------------------------------------------------------
localparam AUTO_BACK = 1'b1;

// ---- 收包侧活动心跳: gmii_rx_clk 域每 2048 像素翻一位 ----
//   为什么不用 pc_rd: 摄像头全屏时写通路只搬摄像头, PC 一次都轮不到,
//   pc_rd 恒 0 —— 拿它当"PC 还活着"的判据, 切过去就永远回不来了。
reg [11:0] pc_beat_cnt;
reg        pc_beat_tog;
always @(posedge gmii_rx_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        pc_beat_cnt <= 12'd0;
        pc_beat_tog <= 1'b0;
    end else if (vid_wr_en) begin
        pc_beat_cnt <= pc_beat_cnt + 12'd1;
        if (pc_beat_cnt == 12'd2047) pc_beat_tog <= ~pc_beat_tog;
    end
end

// ---- 到 clk_50m 域: 两级同步 + 边沿 -> 清零超时计数 ----
reg  b0, b1, b2;
reg [26:0] pc_idle;             // 27 位 @50MHz -> 1.34s 饱和
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        b0 <= 1'b0; b1 <= 1'b0; b2 <= 1'b0;
        pc_idle <= 27'd0;
    end else begin
        b0 <= pc_beat_tog;
        b1 <= b0;
        b2 <= b1;
        if (b1 ^ b2)                    pc_idle <= 27'd0;   // 还在收包
        else if (pc_idle != 27'd75_000_000) pc_idle <= pc_idle + 27'd1;
    end
end

// "PC 在发" = 200ms 内有过翻转。24fps 一帧 41.7ms, 余量足够。
wire pc_alive = (pc_idle < 27'd10_000_000);
// "PC 断了" = 1.5s 一点动静都没有
wire pc_lost  = AUTO_BACK & (disp_mode == D_PCFULL) & (pc_idle >= 27'd75_000_000);

// ---- 是不是"自动切到摄像头全屏的" ----
//   手动按键切过去的不能自动抢回来, 否则用户按到 D_CAMFUL 一眼就被弹走。
reg auto_cam;
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n)                     auto_cam <= 1'b0;
    else if (k1_press)                  auto_cam <= 1'b0;   // 手动 -> 不再自动
    else if (pc_lost)                   auto_cam <= 1'b1;
    else if (disp_mode != D_CAMFUL)     auto_cam <= 1'b0;
end

// PC 恢复后要连续 84ms 有话才切回, 免得断流尾巴抖一下屏幕就跳
reg [21:0] pc_back_cnt;
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n)                     pc_back_cnt <= 22'd0;
    else if (!auto_cam)                 pc_back_cnt <= 22'd0;
    else if (pc_alive) begin
        if (pc_back_cnt != 22'h3FFFFF)  pc_back_cnt <= pc_back_cnt + 22'd1;
    end else                            pc_back_cnt <= 22'd0;
end
wire pc_back = auto_cam & (pc_back_cnt == 22'h3FFFFF);

always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n)
        disp_mode <= D_PCFULL;   // 上电: PC 全屏
    else if (k1_press) begin    // 按下那一下: 循环 +1
        if (disp_mode == D_CAMFUL) disp_mode <= D_PCFULL;
        else                       disp_mode <= disp_mode + 2'd1;
    end
    else if (pc_lost)            // v34: PC 断流 -> 摄像头全屏(不再退四宫格)
        disp_mode <= D_CAMFUL;
    else if (pc_back)            // v34: PC 恢复 -> 自动回 PC 全屏
        disp_mode <= D_PCFULL;
end
wire full_scr = (disp_mode == D_PCFULL);   // 整屏只用 PC 数据
assign cam_full = (disp_mode == D_CAMFUL); // 整屏只用摄像头数据

wire alarm;         // v39: 报警灯(来自 quad_wr_ctrl 的 lane_decision)
wire [1:0] drv_state;  // v44: 行驶状态 0=直行 1=左转 2=右转

wire frame_live;      // v23: DDR 里已经有真实画面 (上电黑屏用)

quad_wr_ctrl u_quad_wr_ctrl(
    .clk           (clk_50m       ),
    .rst_n         (sys_rst_n     ),
    .init_done     (init_sync     ),
    // v10: 每一路都是一条 17bit 字流, 标签字就混在里面
    .cam_word      (cam_rdata     ),
    .cam_empty     (cam_empty     ),
    .cam_rd        (cam_rd        ),
    .cam_level     (cam_rd_level  ),
    .pc_word       (pc_rdata      ),
    .pc_empty      (pc_empty      ),
    .pc_rd         (pc_rd         ),
    .pc_level      (pc_level      ),
    .cam_vs        (cam_vs        ),
    .wr_page       (wr_page       ),
    .wr_done_page  (wr_done_page  ),
    // v12: 算法档位, 按钮循环切换 彩色->灰度->去噪->二值->YCbCr
    .mode_sel      (mode_sel      ),
    // v15: 显示模式, v24 起由按键 1 循环三态 (原来是按键2 翻转)
    .full_scr      (full_scr      ),
    // v23: 摄像头全屏 (整屏 1200 块全归摄像头) + DDR 里有没有真实画面
    .cam_full      (cam_full      ),
    // v34: 车道线检测已内置, 不再从外部送结果(这两个端口保留但不再使用,
    //   删端口要改十几个测试台的例化, 得不偿失)
    .lane_en       (lane_en       ),
    // v50: PC 面板的"车道边线识别"开关 (UDP 0xA55C), 上电默认 1
    .lane_osd_en   (lane_osd_en   ),
    // v39: 模拟强制命令 + 报警灯
    .force_go      (force_go      ),
    .force_l       (force_l       ),
    .force_r       (force_r       ),
    .alarm         (alarm         ),
    .drv_state     (drv_state     ),
    .frame_live    (frame_live    ),
    .ad_data       (ad_data       ),
    .ad_wr         (ad_wr         ),
    .ad_level      (ad_level      ),
    .wd_data       (q_wd_data     ),
    .wd_en         (q_wd_en       ),
    .cam_px        (q_cam_px      ),
    .pc_px         (q_pc_px       ),
    .chunk_cnt     (q_chunk       ),
    .busy          (q_busy        )
);

// 非四宫格老模式: 只把像素写进去, 跳过标签字
assign ddr_wd_en   = QUAD_MODE ? q_wd_en     : (vid_wr_en & ~vid_wr_data[16]);
assign ddr_wd_data = QUAD_MODE ? q_wd_data   : vid_wr_data[15:0];

//------------------------------------------------------------------
// DDR3 控制器
//------------------------------------------------------------------
ddr3_ctrl_top u_ddr3_ctrl_top(
    .clk             (clk_50m       ),
    .rst_n           (sys_rst_n     ),
    .ddr3_init_done  (ddr3_init_done),
    // 用户接口
    .wd_clk          (clk_50m       ),   // 写时钟: 移到这里, 合并逻辑全在 50M 域
    .rd_clk          (lcd_clk       ),
    .wd_en           (ddr_wd_en     ),
    .wd_data         (ddr_wd_data   ),
    .rd_en           (rdata_req     ),
    .rd_data         (rd_data       ),
    .addr_rd_min     (28'b0         ),
    .addr_rd_max     (ddr3_addr_max ),
    .rd_burst_len    (h_disp[12:3]  ),
    .addr_wd_min     (28'b0         ),
    .addr_wd_max     (ddr3_addr_max ),
    .wd_burst_len    (10'd64       ),   // v9: 一个突发 = 512 像素 = 半个屏幕行
    .wr_load         (1'b0          ),
    // ★ 原来接 rd_vsync: 每个 vsync 会 (a) 把 rd_fifo 整个复位清空,
    //   (b) 把 rd_addr_n 强行拉回帧首。而连续写模式下 vsync 到来时
    //   rd_fifo 里还压着上一帧提前读进来的数据, 一清空就等于"丢掉 N 个
    //   像素但 DDR 地址已经走过去了" -> 此后整帧像素与地址永久错开
    //   (实测错开 96 字 = 768 像素 = 屏幕宽度的 3/4)。
    //   改为 0: 读地址完全由 rd_end 自然回绕, 像素与地址严格一一对应。
    .rd_load         (1'b0          ),
    .ddr3_pingpang_en(1'b1          ),
    .ddr3_read_valid (1'b1          ),
    // v9: 写地址流(每 512 像素一个地址) + 写页
    .wd_a_wr         (ad_wr         ),
    .wd_a_data       (ad_data       ),
    .wd_a_level      (ad_level      ),
    .wr_page         (wr_done_page  ),
    // DDR3 IO
    .mem_rst_n       (mem_rst_n     ),
    .mem_ck          (mem_ck        ),
    .mem_ck_n        (mem_ck_n      ),
    .mem_cke         (mem_cke       ),
    .mem_ras_n       (mem_ras_n     ),
    .mem_cas_n       (mem_cas_n     ),
    .mem_we_n        (mem_we_n      ),
    .mem_odt         (mem_odt       ),
    .mem_cs_n        (mem_cs_n      ),
    .mem_a           (mem_a         ),
    .mem_ba          (mem_ba        ),
    .mem_dqs         (mem_dqs       ),
    .mem_dqs_n       (mem_dqs_n     ),
    .mem_dq          (mem_dq        ),
    .mem_dm          (mem_dm        )
);

//------------------------------------------------------------------
// LCD
//------------------------------------------------------------------
// ★v23 上电黑屏★ DDR 上电后内容是随机的, 读侧一上电就在读, 屏幕上就是满屏
//   雪花 —— 评委看到会以为板子坏了。frame_live 在"第一次真正翻页"之后才置起,
//   在那之前把送进 LCD 的数据强制清 0, 于是上电是标准的黑屏, 第一帧画面
//   直接就是完整的。单 bit 跨到 lcd_clk 域, 打两拍就够。
reg fl_s0, fl_s1;
always @(posedge lcd_clk or negedge sys_rst_n) begin
    if (!sys_rst_n) begin fl_s0 <= 1'b0; fl_s1 <= 1'b0; end
    else            begin fl_s0 <= frame_live; fl_s1 <= fl_s0; end
end
wire [15:0] rd_data_v = fl_s1 ? rd_data : 16'h0000;

lcd_rgb_top u_lcd_rgb_top(
    .sys_clk        (clk_50m       ),
    .sys_rst_n      (sys_rst_n     ),
    .sys_init_done  (ddr3_init_done),
    .lcd_id         (lcd_id        ),
    .lcd_hs         (lcd_hs        ),
    .lcd_vs         (lcd_vs        ),
    .lcd_de         (lcd_de        ),
    .lcd_rgb        (lcd_rgb       ),
    .lcd_bl         (lcd_bl        ),
    .lcd_rst        (lcd_rst       ),
    .lcd_pclk       (lcd_pclk      ),
    .lcd_clk        (lcd_clk       ),
    .out_vsync      (rd_vsync      ),
    .h_disp         (              ),
    .v_disp         (              ),
    .pixel_xpos     (pixel_xpos    ),
    .pixel_ypos     (pixel_ypos    ),
    .data_in        (rd_data_v     ),
    .data_req       (rdata_req     )
);

//------------------------------------------------------------------
// 遥测总线 (诊断用, 由 20ms 心跳包捎带回 PC, 不影响显示)
//   [127:96] 摄像头已消费像素数   [95:64] 摄像头场同步个数
//   [63:48]  合成器 ox            [47:32] 合成器 oy
//   [31:24]  标志字节             [23:16] 摄像头等待计时高位
//------------------------------------------------------------------
wire [127:0] tl_bus;

//------------------------------------------------------------------
// 流控信用包的寄存器 (必须在 eth_top 例化【之前】声明)
//------------------------------------------------------------------
// 踩过的坑: 这两个原来只在后面 always 块旁边写 `reg [31:0] credit_val;`,
// 而 eth_top 的端口连接 .credit_val(credit_val) 出现在更前面 —— Verilog 会
// 先给它隐式建一个 1 位 net, 于是后面那句 reg 就变成"重复声明"。PDS 恰好
// 按 32 位放过, ModelSim 直接报 vlog-2388。声明提到前面, 两边都干净。
reg  [31:0] credit_val;     // 信用包携带的累计值
reg         credit_tog;     // 每发一个信用包翻转一次(跨时钟域用电平翻转)

//------------------------------------------------------------------
// 以太网
//------------------------------------------------------------------
eth_top #(
    .BOARD_MAC (BOARD_MAC),
    .BOARD_IP  (BOARD_IP ),
    .DES_MAC   (DES_MAC  ),
    .DES_IP    (DES_IP   )
) u_eth_top(
    .sys_clk      (sys_clk     ),
    .sys_rst_n    (sys_rst_n   ),
    .eth_rxc      (eth_rxc     ),
    .eth_rx_ctl   (eth_rx_ctl  ),
    .eth_rxd      (eth_rxd     ),
    .eth_txc      (eth_txc     ),
    .eth_tx_ctl   (eth_tx_ctl  ),
    .eth_txd      (eth_txd     ),
    .udp_rec_en   (rec_en      ),
    .udp_rec_data (rec_data    ),
    .gmii_rx_clk  (gmii_rx_clk ),
    // 流控信用包: 每消费 1024 个 PC 像素回一个广播包给 PC
    .credit_tog   (credit_tog  ),
    .credit_val   (credit_val  ),
    .tl_bus       (tl_bus      )
);

//------------------------------------------------------------------
// PC 像素消费计数 -> 流控信用包
//------------------------------------------------------------------
// 为什么必须有:
//   写合成器在等摄像头的时候会整段停住(每帧几十毫秒), 这段时间 PC 若继续
//   灌数据, 板上 pc_fifo(8 行) 立刻装满; 而 ctrl_fifo 里 wr_fifo 的 wr_full
//   压根没接出来, 满了就静默丢像素, 写地址却照常前进 —— DDR 里"像素序号"
//   与"地址"从此永久错开。屏幕上看到的就是:
//     - 同一幅画面横向重复出现
//     - 绿色填充块跑到右半屏(它本该只在左上摄像头格)
//     - 每帧还在慢慢平移
//   加了信用包之后, PC 端只在"欠账 <= 窗口"时才发, FIFO 永远不会满。
// ★v9★ 信用直接用"已落盘"的 PC 像素数 q_pc_px
//   为什么不再用 pc_rd 计数: v1~v8 的 pc_rd 里有"挂起时抽干 FIFO"的动作,
//   会把"丢弃"也算成"消费", 于是板子回报"我全收到了", PC 拼命发,
//   数据全被扔掉 —— 上一版整屏变蓝就是这么来的。
//   现在只有"真正写进 DDR"才计数, 语义是干净的。
reg  [11:0] credit_gap;     // 距上一次"按像素"信用包的像素数
reg  [17:0] hb_cnt;         // 心跳: 2ms @50MHz

// 应答节奏: 每消费 512 像素立刻回一个包, 空闲时每 2ms 回一个心跳。
//   v8 把 1024/20ms 收紧到 512/2ms: 反馈延迟直接决定闭环吞吐
//   (吞吐 ≈ 窗口 / 反馈延迟), 原来 20ms 的粒度让窗口 6144 只剩
//   0.3M px/s, 根本喂不饱 1 像素/时钟的写侧。
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n)
        hb_cnt <= 18'd0;
    else if (hb_cnt == 18'd99_999)
        hb_cnt <= 18'd0;
    else
        hb_cnt <= hb_cnt + 18'd1;
end
wire hb_tick = (hb_cnt == 18'd99_999);

wire credit_evt = pc_rd ? (credit_gap == 12'd511) : hb_tick;

always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n) begin
        credit_gap <= 12'd0;
        credit_val <= 32'd0;
        credit_tog <= 1'b0;
    end
    else begin
        if (pc_rd)
            credit_gap <= (credit_gap == 12'd511) ? 12'd0
                                                 : (credit_gap + 12'd1);
        if (credit_evt) begin
            credit_val <= q_pc_px;          // 真正写进 DDR 的 PC 像素累计
            credit_tog <= ~credit_tog;
        end
    end
end

//------------------------------------------------------------------
// 遥测采集 (纯诊断, 不参与显示逻辑)
//   信用包里的 16 字节, 和 tl_bus 严格 1:1
//   [127:96] PC 已落盘像素 (也是信用值的来源)
//   [ 95:64] 摄像头已落盘像素        <- 真值, 不再截断/256
//   [ 63:48] PC FIFO 水位 (字, 满 8192)
//   [ 47:32] 摄像头 FIFO 水位 (字, 满 4096)
//   [ 31:16] 已接收的 UDP 视频包数(低 16 位) -> 和 PC 端发包数对比就知道丢没丢
//   [ 15: 8] 标志字节
//   [  7: 0] 摄像头诊断 {4'b0, href_seen, valid_seen, emit, tag_seen}
//
// ★v10 修正★ 这一段以前是 124 位(cam_dbg 只有 4 bit), 赋给 128 位 wire 时
//   Verilog 在最高位补 4 个 0, 于是所有字段整体下移 4 位: PC 端看到的
//   "PC 水位 61440 / 摄像头水位 32799"(FIFO 深度只有 8192/4096)就是证据,
//   而 "PC 落盘" 其实是真值 >> 4。现在每个字段宽度都精确填满 128 位。
//------------------------------------------------------------------
wire ad_ne = (ad_level != 5'd0);

wire [7:0] tl_flags = { full_scr,       // [7] 显示模式: 1=全屏 0=四宫格 (v15)
                        cam_init_done, // [6] 摄像头配置完成
                        q_busy,        // [5] 搬运器有活
                        q_wd_en,       // [4] 本拍在写 DDR
                        pc_empty,      // [3] PC 队列空
                        cam_empty,     // [2] 摄像头队列空
                        init_sync,     // [1] DDR3 初始化完成(50M 域)
                        ad_ne };       // [0] 还有地址没落盘

wire [127:0] tl_now = { q_pc_px,                    // 127:96
                        q_cam_px,                   //  95:64
                        {2'b0, pc_level},           //  63:48
                        {3'b0, cam_rd_level},       //  47:32
                        vid_pkt_cnt,                //  31:16
                        tl_flags,                   //  15: 8
                        // ★v24 最低字节重新分配★
                        //   [7:5] 算法档位 0..4 (PC 命令写进去的那个值, 回传给 PC 确认)
                        //   [4:3] 显示模式 0=PC全屏 1=四宫格 2=摄像头全屏
                        //   [2:0] 3'b101 固件版本标记
                        //   为什么要标记: v23 及以前显示模式放在 [5:4], 位置变了;
                        //   万一手上的 exe 和板子版本不一致, 靠这 3 位能立刻看出来,
                        //   不会显示成"模式乱跳"让人以为坏了。
                        //   摄像头那 4 位诊断(cam_dbg)不再传了 —— 看 cam_px 在不在涨
                        //   就够了, 而那 4 位正好被上面三个字段占满。
                        {mode_r, disp_mode, 3'b101} };   //   7: 0

// ★ 必须在"发信用的那一拍"整体锁存 ★
//   128 位直接跨到 125MHz 域, 每位各自两级同步会采到"半新半旧"
//   (高 16 位新、低 16 位旧) -> PC 端算出几十万的假增量。
reg [127:0] tl_hold;
always @(posedge clk_50m or negedge sys_rst_n) begin
    if (!sys_rst_n)
        tl_hold <= 128'd0;
    else if (credit_evt)
        tl_hold <= tl_now;
end
assign tl_bus = tl_hold;

//------------------------------------------------------------------
// v44: LED 输出
//   led[0] (C8, 3.3V, 高有效) = 报警灯, 三个强制键共用这一个灯:
//     - 识别到红灯 + 强制直行
//     - 左线实线 + 强制左变道
//     - 右线实线 + 强制右变道
//   报警只跟"决策冲突"有关: 虚线可以压、非红灯可以过, 这两种按了键也不亮。
//
//   led[1..3] = 车辆行驶状态(三态互斥, 每时每刻只有一颗亮):
//     led[1] (L14, 1.5V) = 直行   drv_state == 0
//     led[2] (P16, 1.5V) = 左转   drv_state == 1
//     led[3] (P15, 1.5V) = 右转   drv_state == 2
//   ★和 v39 的区别★ v39 这三个灯接的是"按了哪个键", 所以一按键就亮;
//     现在接的是 ego_state 判出来的实际行驶状态, 跟按键完全无关。
//
//   注意 led[1..3] 在 BANK2 的 1.5V 域, 板上限流电阻按 3.3V 算的, 这三位
//   比 led[0] 暗一些, 属于硬件特性(见 fdc 里的电平约束)。
//------------------------------------------------------------------
assign led[0] = alarm;
assign led[1] = (drv_state == 2'd0);   // 直行
assign led[2] = (drv_state == 2'd1);   // 左转
assign led[3] = (drv_state == 2'd2);   // 右转

endmodule


//=====================================================================
//  以下三个模块为 v31 新增 (车道线检测 / 过域 / 决策)
//---------------------------------------------------------------------
//  ★为什么不单独放 .v 文件★
//    PDS 工程只编译 eth_ddr_lcd.pds 里列出的文件, 新增文件必须在 PDS 里
//    Add File 才会参与综合。为了让"明天直接 Compile 就能用", 这里把它们
//    并进已经在工程里的顶层文件 —— 一个文件里放多个 module 是本工程既有
//    的做法(traffic_light 就并 quad_wr_ctrl.v)。
//    代价: 单文件变长(约 1400 行), 但省掉了所有工程配置操作。
//=====================================================================


//=====================================================================
// v34: lane_detect / lane_cdc / lane_decision 全部搬到 quad_wr_ctrl.v 末尾
//---------------------------------------------------------------------
// 为什么搬: 识别对象从摄像头改成了 PC 视频。PC 像素只存在于 quad_wr_ctrl
//   内部的块流里(块号 + 块内 pcnt), 顶层拿不到, 所以检测器必须搬进去。
//   搬过去之后两边同处 clk_50m 域, lane_cdc 整个不需要了。
//   PDS 不 Add File, 新东西一律并进工程里已有的文件。
//=====================================================================

