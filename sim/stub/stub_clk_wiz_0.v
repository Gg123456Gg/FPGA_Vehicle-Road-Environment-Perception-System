// 仿真桩: PLL。clkout0 直接跟随 clkin1 (板子上就是 50MHz 进 50MHz 出),
// pll_lock 复位后 1us 拉高, 让 sys_rst_n 能释放。
`timescale 1ns/1ps
module clk_wiz_0 (
    input  wire pll_rst,
    input  wire clkin1,
    output wire pll_lock,
    output wire clkout0
);
    reg lock_r;
    initial begin
        lock_r = 1'b0;
        #1000;
        lock_r = 1'b1;
    end
    assign pll_lock = lock_r;
    assign clkout0  = clkin1;
endmodule
