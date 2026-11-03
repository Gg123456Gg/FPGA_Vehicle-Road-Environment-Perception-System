//=====================================================================
// ddr3_rw_ctrl.v   (Verilog-2001)
//---------------------------------------------------------------------
// v9 关键改动: 写地址不再由内部计数器顺序递增, 而是【每个突发由外部给】。
//
// 为什么必须改:
//   v1~v8 的写法是"两路像素必须按屏幕扫描顺序合成一路, 才能顺序写 DDR"。
//   摄像头 22fps、PC 几十 fps, 两路节奏天然不同步, 于是写侧必须不停
//   等待/超时/填色/对齐 —— 每加一个补丁就多一个死锁点, 上板反复出现
//   "画面定格 / 整格变蓝 / 整格变绿 / 内容错位"。
//
//   现在改成: 每个 512 像素块自带目标地址(由源端给出), 谁有数据谁写,
//   两路彻底解耦。写侧再也没有"等谁"的概念, 也就不可能再死锁。
//
// 地址单位(和原设计完全一致, 不要动):
//   1 个 wd_addr 单位 = 8 个像素 (16 byte)
//   一帧 1024*600 = 76800 单位
//   一行 1024 像素 = 128 单位
//   半个屏幕行 512 像素 = 64 单位  <- 本设计的突发长度
//=====================================================================
module ddr3_rw_ctrl(
    input                   rst_n               ,
    input                   clk                 ,   // ui_clk
    input       [27:0]      addr_rd_min         ,   //读 ddr3 的起始地址
    input       [27:0]      addr_rd_max         ,   //读 ddr3 的结束地址
    input       [9:0]       rd_burst_len        ,   //从 ddr3 中读数据时的突发长度
    input       [27:0]      addr_wd_min         ,   //(保留, 已不再使用)
    input       [27:0]      addr_wd_max         ,   //(保留, 已不再使用)
    input       [9:0]       wd_burst_len        ,   //写突发长度(单位同上)
    //与fifo交互信号
    input       [10:0]      rfifo_wcount        ,   //读fifo的写入数据量clk
    input       [10:0]      wfifo_rcount        ,   //写fifo的写入数据量
    // ddr3交互信号
    input                   ddr3_init_done      ,
    input                   wd_finish           ,
    output  reg             wd_req              ,   //写请求信号
    output  reg [27:0]      wd_addr             ,   //写地址信号
    output  reg [9:0]       wd_len              ,   //写突发次数
    // ★v9 外部写地址流: 一个突发消耗一个地址(FWFT, 非空即可用) ★
    input       [27:0]      wd_addr_i           ,
    input                   wd_addr_empty       ,
    output  reg             wd_addr_pop         ,
    input                   rd_finish           ,
    output  reg             rd_req              ,
    output  reg [27:0]      rd_addr             ,
    output  reg [9:0]       rd_len              ,
    // 乒乓操作
    input                   rd_load             ,   //(保留)
    input                   wr_load             ,   //(保留)
    input                   ddr3_pingpang_en    ,
    input       [0:0]       wr_page             ,   //写侧当前页: 读侧在 rd_end 跟过去
    input                   ddr3_read_valid
);
//localparam define
localparam IDLE      = 4'd0    ;
localparam DDR3_DONE = 4'd1    ;
localparam WRITE     = 4'd2    ;
localparam READ      = 4'd3    ;
//reg define
reg [3:0]   state_cnt           ;
reg [27:0]  addr_rd_min_d0      ;
reg [27:0]  addr_rd_max_d0      ;
reg [27:0]  addr_wd_min_d0      ;
reg [27:0]  addr_wd_max_d0      ;
reg [9:0]   rd_burst_len_d0     ;
reg [9:0]   wd_burst_len_d0     ;
reg         rd_load_d0          ;
reg         rd_load_d1          ;
reg         wr_load_d0          ;
reg         wr_load_d1          ;
reg         rd_end              ;
reg  [27:0] rd_addr_n           ;
reg  [27:0] wd_addr_n           ;
reg         raddr_page          ;
reg         raddr_rst_h         ;
reg [10:0]  raddr_rst_h_cnt     ;
reg         wr_page_s0          ;
reg         wr_page_s1          ;

//*****************************************************
//**                    main code
//*****************************************************

// 读地址: 高位由 raddr_page 决定(读页), 低 27 位由读指针给
// 写地址: 完整 28 位全部来自外部地址流(第 27 位就是写页, 由四宫格写侧
//         在"推地址"的那一刻决定), 这样翻页点天然落在块边界上。
always@(*)begin
    if(!rst_n)begin
        rd_addr = 28'b0;
        wd_addr = 28'b0;
    end
    else begin
        if(ddr3_pingpang_en)
            rd_addr = {raddr_page, rd_addr_n[26:0]};
        else
            rd_addr = rd_addr_n;
        wd_addr = wd_addr_n;
    end
end

//对异步信号进行打拍处理
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        addr_rd_min_d0 <= 28'b0;
        addr_rd_max_d0 <= 28'b0;
        rd_burst_len_d0 <= 10'b0;
        addr_wd_min_d0 <= 28'b0;
        addr_wd_max_d0 <= 28'b0;
        wd_burst_len_d0 <= 10'b0;
        rd_load_d0 <= 1'b0;
        rd_load_d1 <= 1'b0;
        wr_load_d0 <= 1'b0;
        wr_load_d1 <= 1'b0;
    end
    else begin
        addr_rd_min_d0 <= addr_rd_min;
        addr_rd_max_d0 <= addr_rd_max;
        rd_burst_len_d0 <= rd_burst_len;
        addr_wd_min_d0 <= addr_wd_min;
        addr_wd_max_d0 <= addr_wd_max;
        wd_burst_len_d0 <= wd_burst_len;
        rd_load_d0 <= rd_load;
        rd_load_d1 <= rd_load_d0;
        wr_load_d0 <= wr_load;
        wr_load_d1 <= wr_load_d0;
    end
end

// 写页跨时钟域(50MHz -> ui_clk)
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        wr_page_s0 <= 1'b0;
        wr_page_s1 <= 1'b0;
    end
    else begin
        wr_page_s0 <= wr_page[0];
        wr_page_s1 <= wr_page_s0;
    end
end

//对输出源的帧复位脉冲
always@(posedge  clk or negedge rst_n)begin
    if(!rst_n)
        raddr_rst_h <= 1'b0;
    else if(rd_load_d0 && !rd_load_d1)
        raddr_rst_h <= 1'b1;
    else if(rd_addr_n == addr_rd_min_d0)
        raddr_rst_h <= 1'b0;
    else
        raddr_rst_h <= raddr_rst_h;
end

//对输出源的帧复位脉冲进行计数
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        raddr_rst_h_cnt <= 11'b0;
    else if(raddr_rst_h)
        raddr_rst_h_cnt <= raddr_rst_h_cnt + 1'b1;
    else
        raddr_rst_h_cnt <= 11'b0;
end

//对输出源帧的读地址高位切换
//   在读帧边界(rd_end, 正好读满一整帧)切, 保证页与地址同时归位。
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)
        raddr_page <= 1'b0;
    else if(rd_end)
        raddr_page <= wr_page_s1;
    else
        raddr_page <= raddr_page;
end

//DDR3读写
always@(posedge clk or negedge rst_n)begin
    if(!rst_n)begin
        state_cnt <= IDLE;
        rd_addr_n <= addr_rd_min;
        wd_addr_n <= 28'd0;
        wd_req <= 1'b0;
        wd_len <= 10'd0;
        rd_req <= 1'b0;
        rd_len <= 10'b0;
        rd_end <= 1'b0;
        wd_addr_pop <= 1'b0;
    end
    else begin
        // 帧结束标志默认清零: 保证 rd_end 只维持 1 拍
        rd_end <= 1'b0;
        wd_addr_pop <= 1'b0;
        // wd_req 默认清零: 只在 WRITE 状态维持高电平。
        //   不清零的话, 突发结束回到 DDR3_DONE 时 wd_req 仍然是 1, 而
        //   aq_axi_master 一回到 S_WR_IDLE 就会立刻再启动一次写, 用的是
        //   还没更新的旧地址 -> 突发地址错位一格(整幅画面横向串位)。
        wd_req <= 1'b0;
        case(state_cnt)
            IDLE : begin
                if(ddr3_init_done)
                    state_cnt <= DDR3_DONE;
                else
                    state_cnt <= IDLE;
            end
            DDR3_DONE : begin
                // 读指针走到帧尾 -> 回帧首, 产生读帧边界
                if(rd_addr_n >= addr_rd_max_d0[27:3])begin
                    state_cnt <= DDR3_DONE;
                    rd_addr_n <= addr_rd_min_d0;
                    rd_end <= 1'b1;
                end
                // 写: fifo 里够一个完整突发, 且外部有地址 -> 开一次突发
                else if((wfifo_rcount >= (wd_burst_len_d0 - 2'd1)) && !wd_addr_empty)begin
                    state_cnt  <= WRITE;
                    wd_addr_n  <= wd_addr_i;      // 本突发的起始地址
                    wd_addr_pop <= 1'b1;          // 消耗掉这个地址
                end
                // 读: fifo 里的数据不够一次突发 -> 发起读
                else if(rfifo_wcount <= (rd_burst_len_d0 - 2'd1))begin
                    state_cnt <= READ;
                    rd_addr_n <= rd_addr_n;
                end
                else begin
                    state_cnt <= state_cnt;
                    rd_end <= 1'b0;
                end
            end
            WRITE : begin
                // 一次突发写完成
                if(wd_finish)begin
                    state_cnt <= DDR3_DONE;
                end
                else if(wfifo_rcount < (wd_burst_len_d0 - 2'd1))
                    wd_req <= 1'b0;
                else begin
                    wd_len <= wd_burst_len_d0;
                    wd_req <= 1'b1;
                end
            end
            READ : begin
                if(rd_finish)begin
                    state_cnt <= DDR3_DONE;
                    rd_addr_n <= rd_addr_n + rd_burst_len_d0;
                end
                else if(rfifo_wcount > (rd_burst_len_d0 - 2'd1))
                    rd_req <= 1'b0;
                else begin
                    rd_len <= rd_burst_len_d0;
                    rd_req <= 1'b1;
                end
            end
            default:begin
                state_cnt <= IDLE;
            end
        endcase
    end
end
endmodule
