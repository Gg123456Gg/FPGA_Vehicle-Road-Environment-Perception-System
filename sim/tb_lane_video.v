//=====================================================================
// tb_lane_video.v  ——  真实视频帧喂 lane_detect, 对比 offv 与锚点中心
//---------------------------------------------------------------------
// 诊断结论用:
//   offv   = (cur_l + cur_r)/2 - 512   cur 是"本帧最后一次命中", 单行跟错就毁整帧
//   c_bot  = (L@599 + R@599)/2         锚点是固定行采样, 跨帧保持, 稳得多
//   bend   = c_top - c_bot             车道朝向(顶部相对底部的横移)
//
// 文件: 每帧 240 行 x 1024 字节, 原始绿色值(0..255), 对应 row 360..599
// 用法: vsim ... -gFN=361 -gFILE=...
//=====================================================================
`timescale 1ns / 1ps

module tb_lane_video;

    parameter FN   = 60;
    parameter FILE = "C:/Users/Asus/AppData/Local/Temp/s2_roi.bin";
    parameter ATH  = 8;        // ACC_TH 扫描用
    parameter DTH  = 24;       // DX_TH 扫描用
    // ★v46 修正: 原来这里硬编码 ACC_HY=3, 而 RTL 例化是 ACC_HY=4, 两者不一致。
    //   退出阈值 = ACC_TH-ACC_HY, 差 1 格就会让"退出早晚"不同, 实测能差出
    //   6 帧的结论。改参数化, 默认值跟 RTL 对齐。
    parameter HY   = 4;        // ACC_HY (必须与 quad_wr_ctrl 里 ego_state 例化一致)
    // v47: 扫 ROI 上下边界用。板子默认 360..599, 这里可覆盖做对比实验。
    parameter TY0  = 360;
    parameter TY1  = 599;
    // v47: 同步扫线型门限 (换 ROI 高度必须按比例改, 否则实线永远判不出)
    parameter HMIN = 70;
    parameter SON  = 168;

    reg         clk     = 1'b0;
    reg         rst_n   = 1'b0;
    reg         vsync   = 1'b0;
    reg         px_en   = 1'b0;
    reg         line_st = 1'b0;
    reg  [9:0]  px_row  = 10'd0;
    reg  [15:0] px565   = 16'h0000;

    wire [1:0]  ltyp, rtyp, offl, cdir;
    wire [10:0] lpos, rpos, offv;
    wire [2:0]  lstate;
    wire [87:0] anch;

    lane_detect u_dut (
        .clk(clk), .rst_n(rst_n), .vsync(vsync),
        .px_en(px_en), .px565(px565), .line_st(line_st), .px_row(px_row),
        .ltyp(ltyp), .rtyp(rtyp), .lpos(lpos), .rpos(rpos), .offv(offv),
        .offl(offl), .lstate(lstate), .cdir(cdir), .anch(anch)
    );
    defparam u_dut.Y0 = TY0;
    defparam u_dut.Y1 = TY1;
    defparam u_dut.HIT_MIN = HMIN;
    defparam u_dut.SOLID_ON = SON;

    // v45: 串上 ego_state, 端到端验证"真实视频 -> 灯"
    wire [1:0] ego;
    reg        eg_tick = 1'b0;

    ego_state #(
        .DX_TH(DTH[9:0]), .ACC_TH(ATH[4:0]), .ACC_HY(HY[4:0])
    ) u_ego (
        .clk(clk), .rst_n(rst_n), .tick(eg_tick),
        .lane_x(anch[43:33]), .proj_dx(5'sd0), .proj_vld(1'b0), .ego(ego)
    );

    always #10 clk = ~clk;

    // 锚点字段: 每段 {valid, x[9:0]}
    wire [10:0] a_l0 = anch[10:0];      // L@420
    wire [10:0] a_l3 = anch[43:33];     // L@599
    wire [10:0] a_r0 = anch[54:44];     // R@420
    wire [10:0] a_r3 = anch[87:77];     // R@599

    wire bot_ok = a_l3[10] & a_r3[10];
    wire top_ok = a_l0[10] & a_r0[10];
    wire [10:0] c_bot = bot_ok ? ((({1'b0,a_l3[9:0]} + {1'b0,a_r3[9:0]}) >> 1)) : 11'd512;
    wire [10:0] c_top = top_ok ? ((({1'b0,a_l0[9:0]} + {1'b0,a_r0[9:0]}) >> 1)) : 11'd512;
    wire signed [11:0] off_bot = {c_bot[10], c_bot} - 12'sd512;
    wire signed [11:0] off_top = {c_top[10], c_top} - 12'sd512;
    wire signed [11:0] bend    = off_top - off_bot;

    reg [7:0] line [0:1023];
    integer   fd, n, x, row, fi;

    task tick; begin @(posedge clk); end endtask

    task show; begin
        $display("  %3d %5d  %5d %5d %5d   %b   %d   %d %d  %4d %4d %4d %4d   %s",
                 fi-1, $signed(offv), off_bot, off_top, bend,
                 {bot_ok, top_ok}, lstate, ltyp, rtyp,
                 a_l3[9:0], a_l0[9:0], a_r3[9:0], a_r0[9:0],
                 (ego==0)?"STR":((ego==1)?"LEFT":"RGHT"));
    end endtask

    initial begin
        fd = $fopen(FILE, "rb");
        if (fd == 0) begin $display("ERR: open %s", FILE); $finish; end

        rst_n = 1'b0;
        repeat (20) tick;
        rst_n = 1'b1;
        repeat (5) tick;

        $display("=== %s  frames=%0d ===", FILE, FN);
        $display("   fi  offv  offB  offT  bend  vt   ls  lt rt   L599 L420 R599 R420  ego");

        for (fi = 0; fi < FN; fi = fi + 1) begin
            px_en = 1'b1; px_row = 10'd359; px565 = 16'h0; tick;
            px_en = 1'b0; vsync = 1'b1; tick; vsync = 1'b0; tick;
            eg_tick = 1'b1; tick; eg_tick = 1'b0;   // lane_vld 那一拍
            if (fi > 0) show;

            for (row = 360; row < 600; row = row + 1) begin
                n = $fread(line, fd);
                px_en = 1'b1; px_row = row[9:0];
                for (x = 0; x < 1024; x = x + 1) begin
                    px565 = {5'd0, line[x][7:2], 5'd0};
                    tick;
                end
                px_en = 1'b0;
                tick;
            end
        end
        px_en = 1'b1; px_row = 10'd359; px565 = 16'h0; tick;
        px_en = 1'b0; vsync = 1'b1; tick; vsync = 1'b0; tick;
        eg_tick = 1'b1; tick; eg_tick = 1'b0;
        show;
        $display("DONE");
        $finish;
    end

endmodule
