//=====================================================================
// tb_tl_left.v   v19 左转箭头判定的集成验证
//---------------------------------------------------------------------
// 验证两件事, 缺一不可:
//   1) 绿色左箭头 -> bk=4 (K_LEFT), 文字 LEFT (4 个字符 = 48 像素宽)
//   2) 同一帧里的绿色圆灯仍然 bk=2 (K_GRN), 文字 GO (2 个字符 = 24 像素宽)
//   ★第 2 条比第 1 条更重要★ 判据一旦太松, 现场就是"每个绿灯都显示 LEFT",
//     那种 bug 比检不出箭头更难看。
//
// 合成图 (512x300, 和 pc/tl_synth.py 里那张同形状):
//   左箭头  x 100..179, y 120..155  (80x36, 三角头 + 直尾杆, fill 约 0.51)
//   圆灯    圆心 (340,140) r=20     (41x41, fill 约 0.78)
//   图案色 0x07E1 (绿, 故意和画框色 0x07E0 差一个 LSB, 否则分不出图案和框)
//
// 期望: 箭头面积 1470 > 圆灯 1320 -> 0 号框是箭头 (bk0=4), 1 号框是圆灯 (bk1=2)
//
// 用法: vsim -c -do "run -all; quit -f" tb_tl_left
//=====================================================================
`timescale 1ns/1ps
module tb_tl_left;

reg clk = 1'b0;
always #10 clk = ~clk;          // 50MHz

reg rst_n = 1'b0;

//--------------------------------------------------------------------
// PC FIFO 模型
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

reg  [2:0]  mode_sel  = 3'd0;
reg         cam_vs    = 1'b0;
reg  [4:0]  ad_level  = 5'd0;

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
    .ad_wr        (ad_wr        ),
    .ad_level     (ad_level     ),
    .wd_data      (wd_data      ),
    .wd_en        (wd_en        ),
    .cam_px       (cam_px       ),
    .pc_px        (pc_px        ),
    .chunk_cnt    (chunk_cnt    ),
    .busy         (busy         )
);

always @(posedge clk) if (pc_rd) rd_ptr <= rd_ptr + 1;

function integer row_of(input integer dst);
    begin
        row_of = dst / 2;
        if (row_of >= 300) row_of = row_of - 300;
    end
endfunction

//--------------------------------------------------------------------
// 画框像素采集: 只认画框色 0x07E0 (图案是 0x07E1)
//--------------------------------------------------------------------
integer gx, gy;
integer ntxt_arrow = 0;         // 箭头框上方的文字像素数 (期望 LEFT)
integer ntxt_round = 0;         // 圆灯框上方的文字像素数 (期望 GO)
integer tmin_arrow = 9999, tmax_arrow = -1;
integer tmin_round = 9999, tmax_round = -1;
integer txtrow_minx = 9999, txtrow_maxx = -1;   // 文字第一行: 整行 (E/F/T 的首行是满的)
integer lgap = 0;                               // L 首行里 x 102..111 的像素数, 期望 0

always @(posedge clk) begin
    if (wd_en && (wd_data == 16'h07E0) && (chunk_cnt > 16'd900)) begin
        gx = u_dut.pcnt - 2;
        gy = row_of(u_dut.cur_dst);
        // 文字区 y 跟着框走 (t0y = q0y0-19), 框 y0 是 121/122, 所以别写死 101
        if ((gy >= 100) && (gy <= 120)) begin
            if (gx < 260) begin
                ntxt_arrow = ntxt_arrow + 1;
                if (gx < tmin_arrow) tmin_arrow = gx;
                if (gx > tmax_arrow) tmax_arrow = gx;
                if (gy == u_dut.t1y) begin
                    if (gx < txtrow_minx) txtrow_minx = gx;
                    if (gx > txtrow_maxx) txtrow_maxx = gx;
                    // L 的 row0 是 0x10, 只有最左一列亮 -> 它的右边 10 像素必须是空的
                    if ((gx >= 102) && (gx <= 111)) lgap = lgap + 1;
                end
            end else begin
                ntxt_round = ntxt_round + 1;
                if (gx < tmin_round) tmin_round = gx;
                if (gx > tmax_round) tmax_round = gx;
            end
        end
    end
end

//--------------------------------------------------------------------
// 合成图
//--------------------------------------------------------------------
function [15:0] pix_of(input integer x, input integer y);
    integer dy, half;
    begin
        pix_of = 16'h0000;
        // ---- 左箭头 ----
        if ((x >= 100) && (x <= 179) && (y >= 120) && (y <= 155)) begin
            if (x <= 136) begin
                // 三角: 尖在 (100,137), 到 x=136 时张开到 120..155
                half = ((x - 100) * 17) / 36;
                dy   = (y >= 137) ? (y - 137) : (137 - y);
                if (dy <= half)
                    pix_of = 16'h07E1;
            end else if ((y >= 131) && (y <= 144)) begin
                // 尾杆: 高 14 行, 一直顶到右边界
                pix_of = 16'h07E1;
            end
        end
        // ---- 圆灯 ----
        else if (((x - 340) * (x - 340) + (y - 140) * (y - 140)) <= 400) begin
            pix_of = 16'h07E1;
        end
    end
endfunction

//--------------------------------------------------------------------
integer seq, i, frame, blk;
integer dst, src_row;
integer errs = 0;

initial begin
    rst_n = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (300) @(posedge clk);

    for (frame = 0; frame < 2; frame = frame + 1) begin
        for (seq = 0; seq < 900; seq = seq + 1) begin
            if (seq < 300) begin
                dst     = seq * 2 + 1;
                src_row = seq;
            end else begin
                dst     = (300 + (seq - 300) / 2) * 2 + ((seq - 300) % 2);
                src_row = (seq - 300) / 2;
            end
            rd_ptr = 0;
            wr_ptr = 0;
            fifo[0] = {1'b1, 5'd0, dst[10:0]};
            for (i = 0; i < 512; i = i + 1)
                fifo[i + 1] = pix_of(i, src_row);
            wr_ptr = 513;

            blk = frame * 900 + seq;
            wait (chunk_cnt == (blk[15:0] + 16'd1));
            repeat (2) @(posedge clk);
        end
    end

    repeat (20) @(posedge clk);

    $display("");
    $display("BOX0 ok=%0d  %0d,%0d-%0d,%0d  bk=%0d",
             u_dut.tl_bv[0], u_dut.q0x0, u_dut.q0y0, u_dut.q0x1, u_dut.q0y1,
             u_dut.tl_bk0);
    $display("BOX1 ok=%0d  %0d,%0d-%0d,%0d  bk=%0d",
             u_dut.tl_bv[1], u_dut.q1x0, u_dut.q1y0, u_dut.q1x1, u_dut.q1y1,
             u_dut.tl_bk1);
    $display("BOX2 ok=%0d  %0d,%0d-%0d,%0d  bk=%0d",
             u_dut.tl_bv[2], u_dut.q2x0, u_dut.q2y0, u_dut.q2x1, u_dut.q2y1,
             u_dut.tl_bk2);
    $display("txt LEFT: n=%0d x %0d..%0d  (expect 46 wide)", ntxt_arrow, tmin_arrow, tmax_arrow);
    $display("txt GO  : n=%0d x %0d..%0d  (expect 22 wide)", ntxt_round, tmin_round, tmax_round);
    $display("L row0 (0x10 -> only leftmost column): x %0d..%0d", txtrow_minx, txtrow_maxx);

    // 排序按面积: 圆灯 1255 > 箭头 1225, 所以 0 号是圆灯、1 号是箭头
    if (u_dut.tl_bk0 !== 3'd2) begin
        $display("  FAIL: 0 hao kuang (yuan deng) bk=%0d, expect 2 (K_GRN)",
                 u_dut.tl_bk0);
        errs = errs + 1;
    end else $display("  0 hao kuang (yuan deng) bk=2  PASS  <- yuan deng bu xu bei pan cheng jian tou");

    if (u_dut.tl_bk1 !== 3'd4) begin
        $display("  FAIL: 1 hao kuang (zuo jian tou) bk=%0d, expect 4 (K_LEFT)",
                 u_dut.tl_bk1);
        errs = errs + 1;
    end else $display("  1 hao kuang (zuo jian tou) bk=4  PASS");

    // LEFT 4 个字符: 墨迹宽 46 (最后一列只有 T 的横杠顶到 x0+45)
    if ((tmax_arrow - tmin_arrow + 1) != 46) begin
        $display("  FAIL: LEFT wen zi kuan %0d, expect 46",
                 tmax_arrow - tmin_arrow + 1);
        errs = errs + 1;
    end else $display("  LEFT wen zi kuan 46  PASS");

    // GO 2 个字符: 墨迹宽 20 (O 的两个边列都亮)
    if ((tmax_round - tmin_round + 1) != 22) begin
        $display("  FAIL: GO wen zi kuan %0d, expect 22",
                 tmax_round - tmin_round + 1);
        errs = errs + 1;
    end else $display("  GO wen zi kuan 22  PASS");

    // L 的首行只有最左一列 (x 100..101); E/F/T 的首行是满的, 所以整行 100..145
    if ((txtrow_minx != 100) || (txtrow_maxx != 145) || (lgap != 0)) begin
        $display("  FAIL: row0 should be 100..145 with gap 102..111 empty; got %0d..%0d lgap=%0d",
                 txtrow_minx, txtrow_maxx, lgap);
        errs = errs + 1;
    end else $display("  L row0 + E/F/T row0  PASS");

    $display("");
    if (errs == 0) $display("RESULT: PASS  (green arrow -> LEFT, green round -> GO)");
    else           $display("RESULT: FAIL  (%0d items)", errs);
    $display("DONE");
    $finish;
end

initial begin
    #50_000_000;
    $display("TIMEOUT: chunk_cnt=%0d", chunk_cnt);
    $finish;
end

endmodule
