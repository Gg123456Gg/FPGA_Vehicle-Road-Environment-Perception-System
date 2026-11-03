//=====================================================================
// tb_proj.v  ——  真实视频喂 lane_detect + proj_shift, 看投影位移
//---------------------------------------------------------------------
// 每一帧打印: L599 / 投影位移 dx(桶, +16px) / conf / ego
// 用法: vsim -c -work work_pv tb_proj -gFN=361 -gFILE=... -do "run -all; quit -f"
//=====================================================================
`timescale 1ns / 1ps

module tb_proj;

    parameter FN   = 60;
    parameter FILE = "C:/Users/Asus/AppData/Local/Temp/s2_roi.bin";
    parameter ATH  = 12;
    parameter DTH  = 24;

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
    wire        pj_vld;
    wire [5:0]  pj_bkt;
    wire [7:0]  pj_g;

    lane_detect u_dut (
        .clk(clk), .rst_n(rst_n), .vsync(vsync),
        .px_en(px_en), .px565(px565), .line_st(line_st), .px_row(px_row),
        .ltyp(ltyp), .rtyp(rtyp), .lpos(lpos), .rpos(rpos), .offv(offv),
        .offl(offl), .lstate(lstate), .cdir(cdir), .anch(anch),
        .proj_vld(pj_vld), .proj_bkt(pj_bkt), .proj_g(pj_g)
    );

    wire signed [4:0] pj_dx;
    wire              pj_ok, pj_conf;
    reg               frm_end = 1'b0;

    proj_shift u_proj (
        .clk(clk), .rst_n(rst_n),
        .px_vld(pj_vld), .px_bkt(pj_bkt), .px_g(pj_g),
        .frm_end(frm_end),
        .dx(pj_dx), .dx_vld(pj_ok), .dx_conf(pj_conf)
    );

    wire [1:0] ego;
    reg        eg_tick = 1'b0;

    ego_state #(
        .DX_TH(DTH[9:0]), .ACC_TH(ATH[4:0]), .ACC_HY(5'd4)
    ) u_ego (
        .clk(clk), .rst_n(rst_n), .tick(eg_tick),
        .lane_x(anch[43:33]), .proj_dx(pj_dx), .proj_vld(pj_ok), .ego(ego)
    );

    always #10 clk = ~clk;

    wire [10:0] a_l3 = anch[43:33];
    wire [10:0] a_r3 = anch[87:77];

    reg [7:0] line [0:1023];
    integer   fd, n, x, row, fi;

    task tick; begin @(posedge clk); end endtask

    initial begin
        fd = $fopen(FILE, "rb");
        if (fd == 0) begin $display("ERR: open %s", FILE); $finish; end

        rst_n = 1'b0;
        repeat (20) tick;
        rst_n = 1'b1;
        repeat (5) tick;

        $display("=== %s  frames=%0d ===", FILE, FN);
        $display("   fi  L599  R599  lt rt  dx  cf ego  up jp lk");

        for (fi = 0; fi < FN; fi = fi + 1) begin
            px_en = 1'b1; px_row = 10'd359; px565 = 16'h0; tick;
            px_en = 1'b0; vsync = 1'b1; tick; vsync = 1'b0; tick;
            eg_tick = 1'b1; frm_end = 1'b1; tick;
            eg_tick = 1'b0; frm_end = 1'b0;

            if (fi > 0)
                $display("  %3d  %4d  %4d   %d  %d  %4d  %d  %s   %d  ac=%3d cf=%d id=%2d",
                         fi-1, a_l3[9:0], a_r3[9:0], ltyp, rtyp,
                         pj_dx, pj_conf,
                         (ego==0)?"STR":((ego==1)?"LEFT":"RGHT"),
                         u_ego.use_proj, u_ego.acc_p, u_ego.cft_cnt, u_ego.idle_cnt);

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
        $display("DONE");
        $finish;
    end

endmodule
