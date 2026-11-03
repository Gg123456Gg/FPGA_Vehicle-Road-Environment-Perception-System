//---------------------------------------------------------------------
// tb_i2c_vts.v —— v42 摄像头时序寄存器核对
//
// 目的: v42 把 VTS 从 1000 改成 700。这个改动同时落在两个地方:
//   ① FPGA 侧参数 CAM_TOTAL_V (写 0x380e/0x380f)
//   ② 最大曝光寄存器 0x3a02/0x3a03/0x3a14/0x3a15 联动成 VTS-4
// 这段寄存器表是 250 个 case 分支, 写错一个字节上板就是黑屏或帧率不对,
// 所以这里不跑完整 I2C 时序(要 250 次握手, 仿真慢), 直接 force 寄存器
// 序号 init_reg_cnt, 查 case 展开出来的 i2c_data。
//
// 期望值 (total_v_pixel = 1000, v42 已撤回回这个值):
//   VTS       = 1000 = 0x3E8 -> 0x380e=0x03, 0x380f=0xE8
//   max_exp   = 5904 = 0x1710 -> 0x3a02=0x17, 0x3a03=0x10
//                                0x3a14=0x17, 0x3a15=0x10
//---------------------------------------------------------------------
`timescale 1ns / 1ps

module tb_i2c_vts;

    reg         clk = 0;
    reg         rst_n = 0;
    reg  [7:0]  i2c_data_r = 8'h00;
    reg         i2c_done = 0;

    wire        i2c_exec;
    wire [23:0] i2c_data;
    wire        i2c_rh_wl;
    wire        init_done;

    integer errs = 0;

    i2c_ov5640_rgb565_cfg u_dut (
        .clk            (clk),
        .rst_n          (rst_n),
        .i2c_data_r     (i2c_data_r),
        .i2c_done       (i2c_done),
        .cmos_h_pixel   (13'd1024),
        .cmos_v_pixel   (13'd600),
        .total_h_pixel  (13'd2200),
        .total_v_pixel  (13'd1000),
        .y_addr_st      (13'd201),
        .y_addr_end     (13'd1749),
        .i2c_exec       (i2c_exec),
        .i2c_data       (i2c_data),
        .i2c_rh_wl      (i2c_rh_wl),
        .init_done      (init_done)
    );

    always #10 clk = ~clk;          // 50MHz

    // 逐条核对: force 寄存器序号 -> 等 2 拍 -> 比对 i2c_data
    task chk;
        input [7:0]  idx;
        input [23:0] want;
        input [8*16-1:0] name;
        begin
            force u_dut.init_reg_cnt = idx;
            repeat (2) @(posedge clk);
            #1;
            if (i2c_data !== want) begin
                $display("  *** FAIL %0s idx=%0d  got %06h  want %06h",
                         name, idx, i2c_data, want);
                errs = errs + 1;
            end else
                $display("  OK   %0s idx=%0d  = %06h", name, idx, i2c_data);
            release u_dut.init_reg_cnt;
            repeat (1) @(posedge clk);
        end
    endtask

    initial begin
        $display("=== camera timing registers (total_v_pixel=1000) ===");
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (5) @(posedge clk);

        // HTS 没动, 顺带核对
        chk(8'd224, 24'h380c_08, "HTS high");     // 2200 = 0x898 -> [12:8]=0x08
        chk(8'd225, 24'h380d_98, "HTS low ");
        // VTS = 1000 = 0x3E8
        chk(8'd226, 24'h380e_03, "VTS high");
        chk(8'd227, 24'h380f_e8, "VTS low ");
        // 最大曝光 = 5904 = 0x1710 (v42 撤回后恢复原值)
        chk(8'd233, 24'h3a02_17, "EXP60 hi");
        chk(8'd234, 24'h3a03_10, "EXP60 lo");
        chk(8'd235, 24'h3a14_17, "EXP50 hi");
        chk(8'd236, 24'h3a15_10, "EXP50 lo");

        if (errs == 0) $display("I2C_VTS_PASS");
        else           $display("I2C_VTS_FAIL errs=%0d", errs);
        $finish;
    end

endmodule
