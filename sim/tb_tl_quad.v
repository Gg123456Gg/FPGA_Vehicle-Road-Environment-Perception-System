//=====================================================================
// tb_tl_quad.v   红绿灯框在 quad_wr_ctrl 里的集成验证
//---------------------------------------------------------------------
// traffic_light 本身已经和 Python 黄金模型逐像素对上了 (tb_tl.v),
// 但"块号 -> 格内行列"这一步是 quad_wr_ctrl 里现算的, 错一格都看不出来。
// 所以这里不喂视频, 直接喂一张位置已知的合成图:
//
//     512x300 全黑, 只有 x=200..260, y=100..150 一块红 0xF820
//     (不写纯红 0xF800: 画框也用它, 框和图案就分不出来了)
//     (61x51 = 3111 个命中, 要在 MAX_HIT=5000 以内, 否则会被当成整屏误检)
//
// 期望的框:  x 200..263 (列桶 50..65, 桶边界在 4 像素上)
//            y 100..150
// 然后检查第二帧输出里所有"框色" (红灯 -> 0xF800) 的像素:
//   1) 必须全部落在框的四条边带内 (宽 3 像素)
//   2) 边带的范围必须正好是上面那个矩形
// 只要有一条不满足, 就是坐标换算或画框逻辑错了。
//
// 用法: vsim -c -do "run -all; quit -f" tb_tl_quad
//=====================================================================
`timescale 1ns/1ps
module tb_tl_quad;

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

// 摄像头全程关掉
wire [16:0] cam_word  = 17'd0;
wire        cam_empty = 1'b1;
wire [12:0] cam_level = 13'd0;
wire        cam_rd;

reg  [2:0]  mode_sel  = 3'd0;   // 彩色直通, 这样只有画框才会出现 0xFFFF
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
    .full_scr     (1'b0         ),   // v15: 四宫格模式
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

//--------------------------------------------------------------------
// 块号 -> 期望的格内行 (和顶层画框用的是同一套换算, 这里独立算一遍)
//--------------------------------------------------------------------
function integer row_of(input integer dst);
    begin
        row_of = dst / 2;
        if (row_of >= 300) row_of = row_of - 300;
    end
endfunction

//--------------------------------------------------------------------
// 画框像素采集
//--------------------------------------------------------------------
integer nwhite   = 0;
integer ntxt     = 0;
integer bad      = 0;
integer wx_min   = 9999, wx_max = -1;
integer wy_min   = 9999, wy_max = -1;
integer gx, gy;

always @(posedge clk) begin
    if (wd_en && (wd_data == 16'hF800) && (chunk_cnt > 16'd900)) begin
        // ★减 2 而不是减 1★
        //   wd_en / wd_data 是 DUT 在拍 N 用 pcnt=P 非阻塞赋的, 到拍 N+1
        //   我们才观察到, 这时 pcnt 已经走到 P+1 了。像素索引 = P-1,
        //   所以用观察到的 pcnt 反推要减 2。减 1 会让整个框向右/下挪一格。
        gx = u_dut.pcnt - 2;
        gy = row_of(u_dut.cur_dst);
        // ★v16: 框上方多了 STOP/GO/WAIT 文字, 颜色和框同色, 一样走
        //   wd_data==0xF800 这条路。必须分开统计 —— 否则 14 行文字会被
        //   当成"画到框外面的框线", 全部判 BAD。
        //   文字几何: txt_y0 = by0 - 19 = 81, 高 14 -> y 81..95
        //             宽 = 4 字符 x 12 = 48 -> x 200..247
        if (gy >= 100) begin
            nwhite = nwhite + 1;
            if (gx < wx_min) wx_min = gx;
            if (gx > wx_max) wx_max = gx;
            if (gy < wy_min) wy_min = gy;
            if (gy > wy_max) wy_max = gy;
            // 框的四条边: 上下两条横边横跨 x0..x1, 左右两条竖边横跨 y0..y1
            if (!((gx >= 200) && (gx <= 260) && (gy >= 100) && (gy <= 150) &&
                  ((gx <= 202) || (gx >= 258) || (gy <= 102) || (gy >= 148)))) begin
                if (bad < 8)
                    $display("  BAD point (%0d,%0d) chunk=%0d", gx, gy, chunk_cnt);
                bad = bad + 1;
            end
        end else begin
            ntxt = ntxt + 1;
            if (!((gy >= 81) && (gy <= 95) && (gx >= 200) && (gx <= 247))) begin
                if (bad < 8)
                    $display("  BAD txt (%0d,%0d) chunk=%0d", gx, gy, chunk_cnt);
                bad = bad + 1;
            end
        end
    end
end

//--------------------------------------------------------------------
integer seq, i, frame, blk;
integer dst, src_row, px;

// 合成图: 黑底上一个纯红矩形
function [15:0] pix_of(input integer x, input integer y);
    begin
        if ((x >= 200) && (x <= 260) && (y >= 100) && (y <= 150))
            pix_of = 16'hF820;      // 红(带一点点绿), 和框色 0xF800 区分开
        else
            pix_of = 16'h0000;
    end
endfunction

initial begin
    rst_n = 1'b0;
    repeat (5) @(posedge clk);
    rst_n = 1'b1;
    repeat (300) @(posedge clk);        // 等 traffic_light 的 S_INIT 清完

    // 两帧: 第一帧建立检测, 第二帧才会画框 (延迟一帧, 和板上一样)
    for (frame = 0; frame < 2; frame = frame + 1) begin
        for (seq = 0; seq < 900; seq = seq + 1) begin
            // ---- 包序号 -> 块号 (与 video_sender.quad_dst 完全一致) ----
            if (seq < 300) begin
                dst     = seq * 2 + 1;
                src_row = seq;
            end else begin
                dst     = (300 + (seq - 300) / 2) * 2 + ((seq - 300) % 2);
                src_row = (seq - 300) / 2;
            end

            // ---- 灌这一块的 513 个字 ----
            rd_ptr = 0;
            wr_ptr = 0;
            fifo[0] = {1'b1, 5'd0, dst[10:0]};       // 标签字
            for (i = 0; i < 512; i = i + 1)
                fifo[i + 1] = pix_of(i, src_row);
            wr_ptr = 513;

            blk = frame * 900 + seq;
            wait (chunk_cnt == (blk[15:0] + 16'd1));
            repeat (2) @(posedge clk);
        end
        $display("frame %0d 走完, chunk_cnt=%0d", frame, chunk_cnt);
    end

    repeat (20) @(posedge clk);

    $display("");
    $display("BOX %0d %0d %0d %0d ok=%0d",
             u_dut.q0x0, u_dut.q0y0, u_dut.q0x1, u_dut.q0y1, u_dut.tl_bv[0]);
    // v17: 一帧最多 4 个框, 这里把 4 个都打出来 —— 顺带验证"只有一个团块时
    //   1~3 号框必须无效", 否则就是列表里塞进了重复项。
    $display("BOX1 ok=%0d  %0d,%0d-%0d,%0d", u_dut.tl_bv[1],
             u_dut.q1x0, u_dut.q1y0, u_dut.q1x1, u_dut.q1y1);
    $display("BOX2 ok=%0d  %0d,%0d-%0d,%0d", u_dut.tl_bv[2],
             u_dut.q2x0, u_dut.q2y0, u_dut.q2x1, u_dut.q2y1);
    $display("BOX3 ok=%0d  %0d,%0d-%0d,%0d", u_dut.tl_bv[3],
             u_dut.q3x0, u_dut.q3y0, u_dut.q3x1, u_dut.q3y1);
    $display("LIST area = %0d %0d %0d %0d",
             u_dut.u_traffic_light.l_area[0], u_dut.u_traffic_light.l_area[1],
             u_dut.u_traffic_light.l_area[2], u_dut.u_traffic_light.l_area[3]);
    $display("框色像素: 共 %0d 个", nwhite);
    $display("  实测 x %0d..%0d   y %0d..%0d", wx_min, wx_max, wy_min, wy_max);
    $display("  expect x 200..260  y 100..150");
    $display("文字像素: 共 %0d 个 (期望 y 81..95 / x 200..247)", ntxt);
    $display("  不在边带内的白点: %0d", bad);
    if ((nwhite > 0) && (ntxt > 0) && (bad == 0) &&
        (wx_min == 200) && (wx_max == 260) &&
        (wy_min == 100) && (wy_max == 150))
        $display("RESULT: PASS");
    else
        $display("RESULT: FAIL");
    $display("DONE");
    $finish;
end

initial begin
    #50_000_000;
    $display("TIMEOUT: chunk_cnt=%0d", chunk_cnt);
    $finish;
end

endmodule
