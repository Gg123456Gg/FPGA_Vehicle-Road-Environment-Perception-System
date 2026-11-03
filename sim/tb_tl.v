//=====================================================================
// tb_tl.v   traffic_light (v15 全屏版) 的比对测试台
//---------------------------------------------------------------------
// 把真实视频帧 (512x300, RGB565) 按 1 像素/拍喂进 traffic_light, 读出框,
// 和 Python 黄金模型 (pc/tl_blob.py) 的结果逐像素对比。
//
// 为什么要这么较真:
//   阈值在 Python 里是一组十进制常数, 抄进 Verilog 时很容易抄错一个数;
//   位宽、>= 还是 >、先减后比还是先比后减, 每个地方都能差 1。
//   这些错在板上只表现为"框偏了几像素", 根本看不出来, 但说明算法没移植对。
//
// 向量: pc/tl_mkvec.py 生成, 帧 60/180/270 分别是绿/红/红三个时刻。
// 期望值: 用 pc/tl_blob.py 直接读同一份 .hex 算出来的 (不是手抄的),
//         所以这个测试台实际在验"RTL 和模型是否逐位等价"。
//=====================================================================
`timescale 1ns/1ps

module tb_tl;

    reg clk = 1'b0;
    always #10 clk = ~clk;          // 50MHz

    reg        rst_n       = 1'b0;
    reg        px_en       = 1'b0;
    reg [9:0]  px_x        = 10'd0;
    reg [9:0]  px_y        = 10'd0;
    reg [15:0] px565       = 16'd0;
    reg        line_flag   = 1'b1;      // v15: 测试台是整行连续喂, 恒为 1
    reg        full        = 1'b0;      // v15: 0 = 四宫格 512x300
    reg        frame_start = 1'b0;

    // ---- v17: traffic_light 改成一帧交 4 个框 (打包成 {y1,x1,y0,x0}) ----
    // 这里把 0 号框 (面积最大的那个) 拆成老的单框信号, 期望值都是从黄金模型
    // 拿的, 单团块场景下 0 号框就是那一个团块, 所以下面的检查逻辑不用改。
    wire [39:0] bxy0, bxy1, bxy2, bxy3;
    wire [2:0]  bk0, bk1, bk2, bk3;
    wire [3:0]  bvv;
    wire        busy;
    wire [9:0] bx0 = bxy0[ 9: 0];
    wire [9:0] by0 = bxy0[19:10];
    wire [9:0] bx1 = bxy0[29:20];
    wire [9:0] by1 = bxy0[39:30];
    wire [2:0] bkind  = bk0;
    wire       bvalid = bvv[0];

    // ★v18: 这里以前把每个参数都显式传了一遍, 结果模块里 MX_G 从 150 改成
    //   115 时, 测试台还按 150 在跑 —— 表现为"RTL 和黄金模型对不上", 但真正
    //   的原因是测试台锁死了旧判据 (查了半小时)。现在一律用模块默认值,
    //   改参数只改一处。
    traffic_light dut (
        .clk         (clk        ),
        .rst_n       (rst_n      ),
        .full        (full       ),
        .px_en       (px_en      ),
        .px_x        (px_x       ),
        .px_y        (px_y       ),
        .line_flag   (line_flag  ),
        .px565       (px565      ),
        .frame_start (frame_start),
        .bxy0        (bxy0       ),
        .bxy1        (bxy1       ),
        .bxy2        (bxy2       ),
        .bxy3        (bxy3       ),
        .bk0         (bk0        ),
        .bk1         (bk1        ),
        .bk2         (bk2        ),
        .bk3         (bk3        ),
        .bv          (bvv        ),
        .busy        (busy       )
    );

    reg [15:0] mem [0:153599];

    integer k, y, x;
    reg [1023:0] fname;
    integer      nfail;
    integer      ex0, ey0, ex1, ey1, ekind;

    // 一次 frame_start: 交出上一帧的结果并清空统计
    task do_frame_start;
        begin
            @(posedge clk);
            frame_start <= 1'b1;
            @(posedge clk);
            frame_start <= 1'b0;
            repeat (50) @(posedge clk);
        end
    endtask

    task expect_box;
        input [10:0] x0, y0, x1, y1;
        input [1:0]  kd;
        begin
            ex0 = x0; ey0 = y0; ex1 = x1; ey1 = y1; ekind = kd;
        end
    endtask

    initial begin
        nfail = 0;
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (50) @(posedge clk);

        for (k = 0; k < 3; k = k + 1) begin
            case (k)
                0: fname = "C:/VERILOG/55_eth_ddr_lcd/pc/tl_vec/tlf060.hex";
                1: fname = "C:/VERILOG/55_eth_ddr_lcd/pc/tl_vec/tlf180.hex";
                default: fname = "C:/VERILOG/55_eth_ddr_lcd/pc/tl_vec/tlf270.hex";
            endcase
            $readmemh(fname, mem);

            do_frame_start;             // 上一帧的结果交出去, 统计清零

            // 300 行 x 512 像素, 1 像素/拍
            for (y = 0; y < 300; y = y + 1) begin
                for (x = 0; x < 512; x = x + 1) begin
                    px_en   <= 1'b1;
                    px_x    <= x[9:0];
                    px_y    <= y[9:0];
                    px565   <= mem[y * 512 + x];
                    @(posedge clk);
                end
            end
            px_en   <= 1'b0;

            do_frame_start;             // 结算本帧 -> 结果出现在 bx0..bkind

            // 期望值由 pc/tl_blob.py 读同一份 hex 算出
            // ★v18 更新了 060 的期望★ MX_G 150->115 之后, 绿灯外圈更暗的一圈
            //   像素也被算进团块, 框从 44x31 变成 47x33 (右下各多 2~3 像素)。
            //   这是"能检到更多绿"的必然结果, 不是回归。
            // ★v37 又更新了一次 060★ 打开青绿容差 (G_BT=12) 之后, 旁边那块
            //   绿色倒计时点阵的笔画满足了绿判据, 和绿灯在 x 上重叠, 被合并成
            //   同一个团块 —— 框从 47x33 变成 62x53。这正是 v20 当初不敢开
            //   G_BT 的原因(它当年放到 24 时更严重, 直接连成 62x52 把真灯挤出
            //   前 4 名)。现在只放到 12, 真灯还在 0 号位, 只是框变大了。
            //   这个代价换来的是 红绿灯2/3/4/5 四个素材的绿灯从"0 个像素"
            //   变成能检出 —— 记录在 QUAD_README 第 42 节。
            case (k)
                0: expect_box(11'd204, 11'd121, 11'd265, 11'd173, 2'd2);
                1: expect_box(11'd203, 11'd119, 11'd267, 11'd177, 2'd1);
                default: expect_box(11'd339, 11'd59, 11'd393, 11'd94, 2'd1);
            endcase

            $display("FRAME %0d  box = %0d,%0d - %0d,%0d (%0dx%0d) kind=%0d valid=%0d",
                     k, bx0, by0, bx1, by1, bx1 - bx0 + 1, by1 - by0 + 1, bkind, bvalid);
            // v18 调试: 把 4 个框和列表面积都打出来, 才能看出是"排序变了"
            //   还是"RTL 把某个团块丢了"。
            $display("   RTL bv=%b  0:%0d,%0d-%0d,%0d(%0d) 1:%0d,%0d-%0d,%0d(%0d)",
                     bvv, bxy0[9:0], bxy0[19:10], bxy0[29:20], bxy0[39:30], bk0,
                     bxy1[9:0], bxy1[19:10], bxy1[29:20], bxy1[39:30], bk1);
            $display("                2:%0d,%0d-%0d,%0d(%0d) 3:%0d,%0d-%0d,%0d(%0d)",
                     bxy2[9:0], bxy2[19:10], bxy2[29:20], bxy2[39:30], bk2,
                     bxy3[9:0], bxy3[19:10], bxy3[29:20], bxy3[39:30], bk3);
            $display("   list area = %0d %0d %0d %0d",
                     dut.l_area[0], dut.l_area[1], dut.l_area[2], dut.l_area[3]);
            $display("          期望 %0d,%0d - %0d,%0d (%0dx%0d) kind=%0d",
                     ex0, ey0, ex1, ey1, ex1 - ex0 + 1, ey1 - ey0 + 1, ekind);

            if (bvalid !== 1'b1) begin
                $display("  ** FAIL: 应该检出, 实际 valid=0");
                nfail = nfail + 1;
            end else if ((bx0 !== ex0) || (by0 !== ey0) ||
                         (bx1 !== ex1) || (by1 !== ey1) ||
                         (bkind !== ekind)) begin
                $display("  ** FAIL: 和黄金模型对不上");
                nfail = nfail + 1;
            end else begin
                $display("  PASS");
            end
        end

        $display("");
        if (nfail == 0) $display("RESULT: PASS  (3/3 帧与模型逐位一致)");
        else            $display("RESULT: FAIL  (%0d 帧不一致)", nfail);
        $display("DONE");
        $finish;
    end

endmodule
