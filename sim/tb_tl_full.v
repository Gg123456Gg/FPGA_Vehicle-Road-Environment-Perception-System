//=====================================================================
// tb_tl_full.v   traffic_light v15 全屏模式的专项测试台
//---------------------------------------------------------------------
// tb_tl.v 只验了四宫格 (full=0) 的算法等价性, v15 新增的两件事没被覆盖:
//   (1) 参数切换: full=1 时面积/亮度/尺寸门限要换成 *_F 那一套
//   (2) 全屏坐标: 一行是"两块 512"拼的, 跨屏幕中线的目标必须还是一个 blob
// 这个测试台就专门打这两个点, 用合成图案, 不依赖视频向量。
//
// 三帧:
//   F0  full=1  12x12 红点              -> 期望 检不到 (面积 144 < AREA_MIN_F 160)
//   F1  full=0  同样 12x12 红点          -> 期望 检到 300,200-311,211 (面积 144 >= 40)
//                                          F0/F1 同一图案, 结果相反 = 证明参数真的切换了
//   F2  full=1  60x40 绿块 跨 x=512      -> 期望 检到 500,380-559,419, 绿
//                                          坐标 y=380 超过 300, 证明 10 位 y 没被截断;
//                                          x 跨 512, 证明 line_flag 把它并成了一块
//=====================================================================
`timescale 1ns/1ps

module tb_tl_full;

    reg clk = 1'b0;
    always #10 clk = ~clk;

    reg        rst_n       = 1'b0;
    reg        px_en       = 1'b0;
    reg [9:0]  px_x        = 10'd0;
    reg [9:0]  px_y        = 10'd0;
    reg        line_flag   = 1'b1;
    reg        full        = 1'b0;
    reg [15:0] px565       = 16'h0000;
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

    // 全部用默认参数 —— 要验的就是默认值本身
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

    localparam [15:0] C_BLK = 16'h0000;
    localparam [15:0] C_RED = 16'hF800;   // 纯红
    localparam [15:0] C_GRN = 16'h07E0;   // 纯绿

    integer nfail;
    integer ex0, ey0, ex1, ey1, ekind;
    integer yy, xx, hf;

    // 图案: 全黑, 只在指定矩形里填色
    function [15:0] px_of;
        input integer x, y;
        input integer rx0, ry0, rx1, ry1;
        input [15:0]  c;
        begin
            if ((x >= rx0) && (x <= rx1) && (y >= ry0) && (y <= ry1))
                px_of = c;
            else
                px_of = C_BLK;
        end
    endfunction

    task do_frame_start;
        begin
            @(posedge clk);
            frame_start <= 1'b1;
            @(posedge clk);
            frame_start <= 1'b0;
            repeat (40) @(posedge clk);
        end
    endtask

    // 全屏喂帧: 一行两块 512, 中间留 3 拍间隙 (和板上 pc_en 断开的形态一致)
    // y_lo..y_hi 之外的行不喂 (省仿真时间, RTL 不需要整帧)
    task feed_full;
        input integer y_lo, y_hi;
        input integer rx0, ry0, rx1, ry1;
        input [15:0]  c;
        begin
            for (yy = y_lo; yy <= y_hi; yy = yy + 1) begin
                for (hf = 0; hf < 2; hf = hf + 1) begin
                    px_en <= 1'b0;
                    repeat (3) @(posedge clk);
                    line_flag <= (hf == 0);
                    for (xx = 0; xx < 512; xx = xx + 1) begin
                        px_en <= 1'b1;
                        px_x  <= hf * 512 + xx;
                        px_y  <= yy;
                        px565 <= px_of(hf * 512 + xx, yy, rx0, ry0, rx1, ry1, c);
                        @(posedge clk);
                    end
                end
            end
            px_en <= 1'b0;
            repeat (20) @(posedge clk);
        end
    endtask

    // 四宫格喂帧: 一行一块 512
    task feed_quad;
        input integer y_lo, y_hi;
        input integer rx0, ry0, rx1, ry1;
        input [15:0]  c;
        begin
            line_flag <= 1'b1;
            for (yy = y_lo; yy <= y_hi; yy = yy + 1) begin
                px_en <= 1'b0;
                repeat (3) @(posedge clk);
                for (xx = 0; xx < 512; xx = xx + 1) begin
                    px_en <= 1'b1;
                    px_x  <= xx;
                    px_y  <= yy;
                    px565 <= px_of(xx, yy, rx0, ry0, rx1, ry1, c);
                    @(posedge clk);
                end
            end
            px_en <= 1'b0;
            repeat (20) @(posedge clk);
        end
    endtask

    task check;
        input [8*24:1] tag;
        input          want_valid;
        input integer  x0, y0, x1, y1, kd;
        begin
            $display("%0s", tag);
            $display("          实际 %0d,%0d - %0d,%0d (%0dx%0d) kind=%0d valid=%0d",
                     bx0, by0, bx1, by1, bx1 - bx0 + 1, by1 - by0 + 1, bkind, bvalid);
            if (!want_valid) begin
                if (bvalid !== 1'b0) begin
                    $display("          期望 检不到,  ** FAIL **");
                    nfail = nfail + 1;
                end else begin
                    $display("          期望 检不到  PASS");
                end
            end else begin
                $display("          期望 %0d,%0d - %0d,%0d (%0dx%0d) kind=%0d",
                         x0, y0, x1, y1, x1 - x0 + 1, y1 - y0 + 1, kd);
                if (bvalid !== 1'b1) begin
                    $display("          ** FAIL: 应该检出, 实际 valid=0 **");
                    nfail = nfail + 1;
                end else if ((bx0 !== x0) || (by0 !== y0) ||
                             (bx1 !== x1) || (by1 !== y1) || (bkind !== kd)) begin
                    $display("          ** FAIL: 框/颜色对不上 **");
                    nfail = nfail + 1;
                end else begin
                    $display("          PASS");
                end
            end
            $display("");
        end
    endtask

    initial begin
        nfail = 0;
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (40) @(posedge clk);

        // ---------------- F0: 全屏, 12x12 小红点, 应该被面积门限挡掉 ----------------
        full = 1'b1;
        do_frame_start;
        feed_full(0, 240, 300, 200, 311, 211, C_RED);
        do_frame_start;
        check("F0 全屏 12x12 红 (面积 144 < 160)", 1'b0, 0, 0, 0, 0, 0);

        // ---------------- F1: 四宫格, 同一图案, 应该检到 ----------------
        full = 1'b0;
        do_frame_start;
        feed_quad(0, 240, 300, 200, 311, 211, C_RED);
        do_frame_start;
        check("F1 四宫格 同样 12x12 红 (面积 144 >= 40)", 1'b1,
              300, 200, 311, 211, 2'd1);

        // ---------------- F2: 全屏, 60x40 绿块跨屏幕中线 ----------------
        full = 1'b1;
        do_frame_start;
        feed_full(0, 460, 500, 380, 559, 419, C_GRN);
        do_frame_start;
        check("F2 全屏 60x40 绿 跨 x=512, y=380..419", 1'b1,
              500, 380, 559, 419, 2'd2);

        $display("");
        if (nfail == 0) $display("RESULT: PASS  (全屏参数切换 / 跨中线 / 10位坐标 全部正确)");
        else            $display("RESULT: FAIL  (%0d 项不符)", nfail);
        $display("DONE");
        $finish;
    end

    // 超时保护
    initial begin
        #2000000000;
        $display("RESULT: FAIL  (超时)");
        $display("DONE");
        $finish;
    end

endmodule
