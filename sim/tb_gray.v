`timescale 1ns/1ps
//---------------------------------------------------------------------
// tb_gray.v  验证 quad_wr_ctrl 的 RGB565->灰度
//
// 只测"数据正确性", 不测流控:
//   推一个标签 + 512 个像素进 cam_word, 把 wd_en 时吐出的 wd_data 全收集,
//   再和 Python 独立算出来的期望值逐字节比对。
//---------------------------------------------------------------------
module tb_gray;

reg         clk = 0;
reg         rst_n = 0;
always #10 clk = ~clk;          // 50MHz

// ---- 摄像头字流 ----
reg  [16:0] mem [0:512];
reg  [9:0]  idx = 0;
wire        cam_rd;
wire [16:0] cam_word;
assign cam_word = mem[idx];

// FIFO 模型: 复位期间必须是空的, 否则 st 虽然被复位成 IDLE, 但组合的
// cam_rd 不受 rst_n 影响, 会在复位那几拍把数据先读空 —— 正好错开 5~6 个字
reg  [12:0] cam_level = 13'd0;
wire        cam_empty;
assign cam_empty = (cam_level == 13'd0);

always @(posedge clk) begin
    if (cam_rd) begin
        idx <= idx + 1'd1;
        if (cam_level != 13'd0) cam_level <= cam_level - 13'd1;
    end
end

// ---- PC 字流 (全关) ----
wire [16:0] pc_word;
reg         pc_empty = 1;
reg  [13:0] pc_level = 14'd0;
wire        pc_rd;
assign pc_word = 17'd0;

reg         cam_vs   = 0;
reg         gray_en  = 1;
reg  [4:0]  ad_level = 5'd0;

wire        wr_page, wr_done_page;
wire [27:0] ad_data;
wire        ad_wr;
wire [15:0] wd_data;
wire        wd_en;
wire [31:0] cam_px, pc_px;
wire [15:0] chunk_cnt;
wire        busy;

quad_wr_ctrl dut (
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
    .gray_en      (gray_en     ),
    .ad_data      (ad_data     ),
    .ad_wr        (ad_wr       ),
    .ad_level     (ad_level    ),
    .wd_data      (wd_data     ),
    .wd_en        (wd_en       ),
    .cam_px       (cam_px      ),
    .pc_px        (pc_px       ),
    .chunk_cnt    (chunk_cnt   ),
    .busy         (busy        )
);

// ---- 收集输出 ----
integer fout;
integer ncap = 0;

always @(posedge clk) begin
    if (rst_n && wd_en) begin
        $fwrite(fout, "%04x\n", wd_data);
        ncap = ncap + 1;
    end
end

// ---- 测试向量 ----
integer i;
initial begin
    fout = $fopen("gray_out.txt", "w");

    mem[0] = {1'b1, 5'd0, 11'd1};     // 标签字: 块号 1
    mem[1] = 16'h0000;                // 黑
    mem[2] = 16'hFFFF;                // 白
    mem[3] = 16'hF800;                // 纯红
    mem[4] = 16'h07E0;                // 纯绿
    mem[5] = 16'h001F;                // 纯蓝
    mem[6] = 16'hFFFF;                // 白
    mem[7] = 16'h8410;                // 中灰
    mem[8] = 16'h7BEF;                // 灰
    for (i = 9; i <= 512; i = i + 1)
        mem[i] = 16'h1234;

    // 先给一个 vsync 脉冲, 让 do_flip 把 gray_en 锁进 gray_lat
    rst_n = 0;
    repeat (5) @(posedge clk);
    rst_n = 1;
    repeat (2) @(posedge clk);
    // 先给 vsync 把 gray_en 锁进 gray_lat —— 必须赶在 FIFO 有数据之前,
    // 否则一旦起块, flip_ok 要等整块搬完(512 拍)才成立, 这一块已经用旧值搬走了
    @(posedge clk) cam_vs <= 1'b1;
    @(posedge clk) cam_vs <= 1'b0;
    repeat (10) @(posedge clk);
    cam_level = 13'd513;          // 现在才喂数据
    repeat (3) @(posedge clk);

    // 等这一块搬完
    wait (ncap >= 512);
    repeat (10) @(posedge clk);

    $fclose(fout);
    $display("captured = %0d", ncap);
    $finish;
end

initial begin
    #200000;
    $display("TIMEOUT captured=%0d", ncap);
    $fclose(fout);
    $finish;
end

endmodule
