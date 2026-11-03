//=====================================================================
// tb_tl_color.v   traffic_light v16 三色判据的专项测试台
//---------------------------------------------------------------------
// v16 新增黄灯。这一版最容易出问题的地方是"黄和红的重叠":
//   实测黄灯 (255,190,20) 的 R-G 只有 65, 而红判据只要 R-G >= 50,
//   所以黄灯天然满足红判据。如果只是简单地把三条判据并起来, 黄灯会
//   永远被投票成红灯 —— 这种错不会报任何错, 只会"颜色一直不对"。
//
// 四帧, 每帧一个 40x40 的色块 (面积 1600, 四宫格门限 40 够):
//   F0  黄 (255,190,20)  -> 期望 kind=3 (黄)   ★核心用例: 不能被判成红
//   F1  红 (255,  0,  0) -> 期望 kind=1 (红)
//   F2  绿 (  0,255,  0) -> 期望 kind=2 (绿)
//   F3  白 (255,255,255) -> 期望 检不到         (过曝灯芯不该算任何一个颜色)
//
// 记下这四帧的 RGB565 值 (8bit 是位复制展开后的实际值):
//   黄 0xFDE2 -> c=(255,190, 16)   红 0xF800 -> c=(255,  0,  0)
//   绿 0x07E0 -> c=(  0,255,  0)   白 0xFFFF -> c=(255,255,255)
//=====================================================================
`timescale 1ns/1ps

module tb_tl_color;

    reg clk = 1'b0;
    always #10 clk = ~clk;

    reg        rst_n       = 1'b0;
    reg        px_en       = 1'b0;
    reg [9:0]  px_x        = 10'd0;
    reg [9:0]  px_y        = 10'd0;
    reg        line_flag   = 1'b1;
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

    traffic_light dut (
        .clk         (clk        ),
        .rst_n       (rst_n      ),
        .full        (1'b0       ),   // 四宫格参数, 面积门限低, 小色块也能检到
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

    localparam [15:0] C_BLK  = 16'h0000;
    localparam [15:0] C_YEL  = 16'hFDE2;   // 255,190, 16
    localparam [15:0] C_RED  = 16'hF800;   // 255,  0,  0
    localparam [15:0] C_GRN  = 16'h07E0;   //   0,255,  0
    localparam [15:0] C_WHT  = 16'hFFFF;   // 255,255,255

    integer nfail;
    integer yy, xx;

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
        input [8*28:1] tag;
        input          want_valid;
        input integer  kd;
        begin
            $display("%0s", tag);
            $display("          实际 kind=%0d valid=%0d  box=%0d,%0d-%0d,%0d",
                     bkind, bvalid, bx0, by0, bx1, by1);
            if (!want_valid) begin
                if (bvalid !== 1'b0) begin
                    $display("          期望 检不到   ** FAIL **");
                    nfail = nfail + 1;
                end else begin
                    $display("          期望 检不到   PASS");
                end
            end else if (bvalid !== 1'b1) begin
                $display("          期望 kind=%0d  ** FAIL: 应该检出 **", kd);
                nfail = nfail + 1;
            end else if (bkind !== kd[2:0]) begin
                $display("          期望 kind=%0d  ** FAIL: 颜色判错 **", kd);
                nfail = nfail + 1;
            end else begin
                $display("          期望 kind=%0d  PASS", kd);
            end
            $display("");
        end
    endtask

    initial begin
        nfail = 0;
        repeat (5) @(posedge clk);
        rst_n <= 1'b1;
        repeat (40) @(posedge clk);

        // ---------------- F0: 黄  (核心: 必须 kind=3, 不能被判成红) ----------------
        do_frame_start;
        feed_quad(195, 245, 300, 200, 339, 239, C_YEL);
        do_frame_start;
        check("F0 黄 0xFDE2 (255,190,16)", 1'b1, 3);

        // ---------------- F1: 红 ----------------
        do_frame_start;
        feed_quad(195, 245, 300, 200, 339, 239, C_RED);
        do_frame_start;
        check("F1 红 0xF800 (255,0,0)", 1'b1, 1);

        // ---------------- F2: 绿 ----------------
        do_frame_start;
        feed_quad(195, 245, 300, 200, 339, 239, C_GRN);
        do_frame_start;
        check("F2 绿 0x07E0 (0,255,0)", 1'b1, 2);

        // ---------------- F3: 白, 应该一个颜色都不算 ----------------
        // ★要跑满 HOLD_N+2 帧★ 检测器有"连续 5 帧没检到才熄灭"的沿用机制
        //   (HOLD_N=5), 只跑一帧的话 bvalid 还是上一帧(绿)沿用的值, 会误判
        //   成"白色被检出了"。
        do_frame_start;
        feed_quad(195, 245, 300, 200, 339, 239, C_WHT);
        repeat (7) do_frame_start;
        check("F3 白 0xFFFF (过曝灯芯)", 1'b0, 0);

        $display("");
        if (nfail == 0) $display("RESULT: PASS  (黄/红/绿/白 四色全部判对)");
        else            $display("RESULT: FAIL  (%0d 项不符)", nfail);
        $display("DONE");
        $finish;
    end

    initial begin
        #500000000;
        $display("RESULT: FAIL  (超时)");
        $display("DONE");
        $finish;
    end

endmodule
