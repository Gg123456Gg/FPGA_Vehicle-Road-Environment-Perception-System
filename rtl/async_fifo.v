//=====================================================================
// async_fifo.v   (Verilog-2001, 纯逻辑, 不依赖任何 IP 核)
//---------------------------------------------------------------------
// 通用双时钟异步 FIFO, 格雷码指针跨域, 秀前读出 (FWFT):
//   rd_empty = 0 时 rd_data 当拍就是有效数据, rd_en 拉高一拍消费一个。
//   吞吐 1 像素/周期。
//
// v2 修正:
//   1) wr_full 的格雷码比较位宽写错了(原来拼出来 AW+2 位, 和 AW+1 位的
//      写指针比较, 高一位被 Verilog 补 0 => 满标志几乎永远不成立 =>
//      写指针会一路越过读指针, 把未读数据覆盖掉 => 满屏错位/花屏)。
//      正确写法: {~rptr[AW:AW-1], rptr[AW-2:0]}, 正好 AW+1 位。
//   2) 新增 rd_level 输出: 读域看到的"还能读多少像素",
//      给写合成器做水位判断用(防止 PC 侧缓冲被冲爆)。
//
// 参数:
//   DW : 数据位宽
//   AW : 地址位宽, 深度 = 2^AW
//=====================================================================
module async_fifo #(
    parameter DW = 16,
    parameter AW = 12
)(
    input  wire           rst_n      ,
    // ---- 写侧 ----
    input  wire           wr_clk     ,
    input  wire           wr_en      ,
    input  wire [DW-1:0]  wr_data    ,
    output wire           wr_full    ,
    output reg  [AW:0]    wr_level   ,   // 已存数据量(写域, 偏大/保守)
    // ---- 读侧 ----
    input  wire           rd_clk     ,
    input  wire           rd_en      ,
    output wire [DW-1:0]  rd_data    ,
    output wire           rd_empty   ,   // 1 = 无数据
    output wire [AW:0]    rd_level       // 已存数据量(读域, 偏小/保守)
);

    localparam DEPTH = (1 << AW);

    reg [DW-1:0] mem [0:DEPTH-1];

    // 写域指针 (二进制 + 格雷码)
    reg [AW:0] wptr;
    reg [AW:0] wptr_g;
    // 读域指针: ld_ptr = 已经搬进 rdata 的下一个地址
    reg [AW:0] ld_ptr;
    reg [AW:0] ld_ptr_g;

    reg [DW-1:0] rdata;
    reg          rvalid;

    // 跨域同步寄存器
    reg [AW:0] rg_s1, rg_s2;   // 读指针格雷码 -> 写域
    reg [AW:0] wg_s1, wg_s2;   // 写指针格雷码 -> 读域

    integer i;

    //------------------------------------------------------------------
    // 写侧
    //------------------------------------------------------------------
    // 满: 写指针比读指针多绕一整圈
    //     注意位宽必须是 AW+1: ~rg_s2[AW:AW-1] 是 2 位, rg_s2[AW-2:0] 是
    //     AW-1 位, 合起来正好 AW+1 位, 和 wptr_g 同宽。
    assign wr_full = (wptr_g == {~rg_s2[AW:AW-1], rg_s2[AW-2:0]});

    always @(posedge wr_clk or negedge rst_n) begin
        if (!rst_n) begin
            rg_s1 <= {(AW+1){1'b0}};
            rg_s2 <= {(AW+1){1'b0}};
        end else begin
            rg_s1 <= ld_ptr_g;
            rg_s2 <= rg_s1;
        end
    end

    always @(posedge wr_clk or negedge rst_n) begin
        if (!rst_n) begin
            wptr   <= {(AW+1){1'b0}};
            wptr_g <= {(AW+1){1'b0}};
        end else if (wr_en && !wr_full) begin
            mem[wptr[AW-1:0]] <= wr_data;
            wptr   <= wptr + 1'b1;
            wptr_g <= (wptr + 1'b1) ^ ((wptr + 1'b1) >> 1);
        end
    end

    // 写域估算已存数据量: 格雷码 -> 二进制 -> 相减
    reg [AW:0] rg_bin_c;

    always @(*) begin
        rg_bin_c[AW] = rg_s2[AW];
        for (i = AW-1; i >= 0; i = i - 1)
            rg_bin_c[i] = rg_bin_c[i+1] ^ rg_s2[i];
    end

    // v10 修正: 相减是模 2^(AW+1) 的, 当"读指针跑到写指针前面"(FIFO 快空,
    //   格雷码同步值偏旧)时结果会绕成一个巨大的数(比如 16383), 水位看着像
    //   "快满了", 上层所有 `level >= N` 的判断全部失效 —— 之前上板看到的
    //   "PC 水位 61440 / 摄像头水位 32799"(深度只有 8192 / 4096)就是它。
    //   合法值最大就是 DEPTH(=2^AW), 比它大一定是绕回, 一律当 0。
    wire [AW:0] wl_raw = wptr - rg_bin_c;

    always @(posedge wr_clk or negedge rst_n) begin
        if (!rst_n)
            wr_level <= {(AW+1){1'b0}};
        else if (wl_raw > DEPTH)
            wr_level <= {(AW+1){1'b0}};
        else
            wr_level <= wl_raw;
    end

    //------------------------------------------------------------------
    // 读侧
    //------------------------------------------------------------------
    always @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n) begin
            wg_s1 <= {(AW+1){1'b0}};
            wg_s2 <= {(AW+1){1'b0}};
        end else begin
            wg_s1 <= wptr_g;
            wg_s2 <= wg_s1;
        end
    end

    wire mem_empty = (ld_ptr_g == wg_s2);          // 内存里没有未搬出的数据
    wire rpop      = rd_en & rvalid;               // 消费掉当前 rdata
    wire can_load  = (~rvalid | rpop) & ~mem_empty;

    always @(posedge rd_clk or negedge rst_n) begin
        if (!rst_n) begin
            ld_ptr   <= {(AW+1){1'b0}};
            ld_ptr_g <= {(AW+1){1'b0}};
            rvalid   <= 1'b0;
            rdata    <= {DW{1'b0}};
        end else if (can_load) begin
            rdata    <= mem[ld_ptr[AW-1:0]];
            ld_ptr   <= ld_ptr + 1'b1;
            ld_ptr_g <= (ld_ptr + 1'b1) ^ ((ld_ptr + 1'b1) >> 1);
            rvalid   <= 1'b1;
        end else if (rpop) begin
            rvalid   <= 1'b0;
        end
    end

    // 读域估算水位: 写指针(同步值, 偏旧) - 读指针(含已预取的一个)
    //   v10: 同样要夹掉绕回(写指针同步值偏旧时会被读指针超过)
    reg [AW:0] wg_bin_c;

    always @(*) begin
        wg_bin_c[AW] = wg_s2[AW];
        for (i = AW-1; i >= 0; i = i - 1)
            wg_bin_c[i] = wg_bin_c[i+1] ^ wg_s2[i];
    end

    wire [AW:0] rl_raw = wg_bin_c - ld_ptr;

    assign rd_data  = rdata;
    assign rd_empty = ~rvalid;
    assign rd_level = (rl_raw > DEPTH) ? {(AW+1){1'b0}} : rl_raw;

endmodule
