//=====================================================================
// tb_flip_to.v   翻页死锁的专项测试台 (v16)
//---------------------------------------------------------------------
// 现场现象: 视频停止传输时按按键, 屏幕定格、按键全都没反应, 只能复位。
//
// 机制:
//   do_flip 依赖 flip_ok, 而 flip_ok 原来要求 "ad_level == 0"。
//   ad_level 是地址 FIFO 的水位, 只有 DDR 那边每消费一个地址才会降 1;
//   而 DDR 消费的条件是"写数据 FIFO 里够一个完整突发"。PC 一断流, 写侧
//   没有新数据, 数据 FIFO 被抽干, DDR 就停在 DDR3_DONE 不再消费地址。
//   于是 ad_level 卡在原处, flip_ok 永远不成立, wr_page 再也不翻转 ——
//   写侧永远往同一页写, 屏幕定格, 按键换档/换模式都看不到变化。
//
// 本测试台把 ad_level 强行钉死在 6 (can_start 也因此为 0, 模拟彻底停摆),
// 然后发一个 cam_vs, 看 wr_done_page 能不能在超时窗口内翻过去:
//   - 旧逻辑 (要求 ad_level == 0): 永远不翻 -> 本测试会 FAIL
//   - 新逻辑 (ad_level <= 2 或 20ms 超时): 约 20ms 后翻 -> PASS
//
// 用例:
//   T0  ad_level=0 发 cam_vs     -> 应立刻翻 (正常路径不能被超时拖慢)
//   T1  ad_level=6 发 cam_vs     -> 应在 20~25ms 内翻 (超时救回来)
//   T2  ad_level=6 再发一次      -> 同样能翻 (说明不是一次性的偶然)
//=====================================================================
`timescale 1ns/1ps
module tb_flip_to;

reg clk = 1'b0;
always #10 clk = ~clk;          // 50MHz, 1 拍 = 20ns

reg rst_n = 1'b0;

//--------------------------------------------------------------------
// PC FIFO 模型 (FWFT: word 当拍有效, rd 拉高才前进)
//--------------------------------------------------------------------
reg  [16:0] fifo [0:1023];
integer     rd_ptr = 0;
integer     wr_ptr = 0;

wire [16:0] pc_word  = fifo[rd_ptr];
wire        pc_empty = (rd_ptr == wr_ptr);
wire [13:0] pc_level = wr_ptr[13:0] - rd_ptr[13:0];
wire        pc_rd;

wire [16:0] cam_word  = 17'd0;
wire        cam_empty = 1'b1;
wire [12:0] cam_level = 13'd0;
wire        cam_rd;

reg  [2:0]  mode_sel = 3'd0;
reg         cam_vs   = 1'b0;
reg  [4:0]  ad_level = 5'd0;     // ★本测试的主控信号★

wire [27:0] ad_data;
wire        ad_wr;
wire [15:0] wd_data;
wire        wd_en;
wire        wr_page, wr_done_page;
wire [31:0] cam_px, pc_px;
wire [15:0] chunk_cnt;
wire        busy;

quad_wr_ctrl u_dut(
    .clk          (clk          ),
    .rst_n        (rst_n        ),
    .init_done    (1'b1         ),
    .cam_word     (cam_word     ),
    .cam_empty    (cam_empty    ),
    .cam_rd       (cam_rd       ),
    .cam_level    (cam_level    ),
    .pc_word      (pc_word      ),
    .pc_empty     (pc_empty     ),
    .pc_rd        (pc_rd        ),
    .pc_level     (pc_level     ),
    .cam_vs       (cam_vs       ),
    .wr_page      (wr_page      ),
    .wr_done_page (wr_done_page ),
    .mode_sel     (mode_sel     ),
    .full_scr     (1'b0         ),
    .cam_full     (1'b0         ),   // v23: 三态里的摄像头全屏, 这些台子只测四宫格/PC 全屏
    .lane_osd_en  (1'b1    ),   // v51: 显示开关, 测试里常开
    .force_go     (1'b0         ),
    .force_l      (1'b0         ),
    .force_r      (1'b0         ),
    .alarm        (             ),
    .ad_data      (ad_data      ),
    .ad_wr         (ad_wr       ),
    .ad_level      (ad_level    ),
    .wd_data       (wd_data     ),
    .wd_en         (wd_en       ),
    .cam_px        (cam_px      ),
    .pc_px         (pc_px       ),
    .chunk_cnt     (chunk_cnt   ),
    .busy          (busy        )
);

// ★必须夹住 rd_ptr★ 不加限位的话, 数据读完后 rd_ptr 会一路超过 wr_ptr,
//   pc_level 变成巨大的数(word 是 14 位, 减法绕回), pc_empty 永远是 0,
//   状态机会拿越界的 x 一直"读"下去 —— 现象就是喂完一块之后整体行为全乱。
always @(posedge clk) if (pc_rd && (rd_ptr < wr_ptr)) rd_ptr <= rd_ptr + 1;

integer nfail = 0;
integer i, c0;
reg     old_page;

// 喂一块 (标签 + 512 黑像素), 让 page_dirty 置 1
task feed_one;
    input integer dst;
    begin
        rd_ptr = 0;
        wr_ptr = 0;
        fifo[0] = {1'b1, 5'd0, dst[10:0]};
        for (i = 0; i < 512; i = i + 1)
            fifo[i + 1] = 17'h0000;
        wr_ptr = 513;
        c0 = chunk_cnt;
        wait (chunk_cnt == ((c0 + 1) & 16'hFFFF));
        repeat (5) @(posedge clk);
    end
endtask

task pulse_vs;
    begin
        @(posedge clk);
        cam_vs <= 1'b1;
        @(posedge clk);
        cam_vs <= 1'b0;
        repeat (5) @(posedge clk);
    end
endtask

// 发一个 cam_vs, 然后在 limit 拍内等 wr_done_page 变化
task try_flip;
    input [8*30:1] tag;
    input integer  limit;
    integer n;
    begin
        // ★盯 wr_page 而不是 wr_done_page★ wr_done_page 是"可以显示的那一页",
        //   它滞后一页: 第一次翻页时 wr_page 0->1, 而 wr_done_page 是 0->0,
        //   盯它就会漏掉第一次翻页, 误判成"没翻"。
        old_page = wr_page;
        pulse_vs;
        n = 0;
        while ((wr_page === old_page) && (n < limit)) begin
            @(posedge clk);
            n = n + 1;
        end
        if (wr_page !== old_page)
            $display("%0s  -> 翻页成功, 用时 %0d 拍 (%.2f ms)",
                     tag, n, n * 20.0 / 1000000.0);
        else begin
            $display("%0s  -> ** FAIL: %0d 拍内没翻页 **", tag, limit);
            nfail = nfail + 1;
        end
    end
endtask

initial begin
    repeat (5) @(posedge clk);
    rst_n <= 1'b1;
    repeat (60) @(posedge clk);

    // ---------------- T0: 正常情况, ad_level=0, 应立刻翻 ----------------
    ad_level = 5'd0;
    feed_one(1);
    try_flip("T0 正常 ad_level=0", 200);

    // ---------------- T1: DDR 停摆, ad_level 钉死 6 ----------------
    // 先再写一块把 page_dirty 置 1, 否则 do_flip 会因为"本页没写过"跳过
    ad_level = 5'd0;
    feed_one(3);
    ad_level = 5'd6;                 // ← 从这里开始 DDR 再也不消费地址
    try_flip("T1 停摆 ad_level=6", 1400000);   // 1400000 * 20ns = 28ms

    // ---------------- T2: 再停一次, 确认不是一次性 ----------------
    ad_level = 5'd0;
    feed_one(5);
    ad_level = 5'd6;
    try_flip("T2 停摆 ad_level=6 再来一次", 1400000);

    $display("");
    if (nfail == 0) $display("RESULT: PASS  (ad_level 卡死时翻页仍能被超时救回)");
    else            $display("RESULT: FAIL  (%0d 项)", nfail);
    $display("DONE");
    $finish;
end

initial begin
    #200000000;
    $display("RESULT: FAIL  (超时)");
    $display("DONE");
    $finish;
end

endmodule
