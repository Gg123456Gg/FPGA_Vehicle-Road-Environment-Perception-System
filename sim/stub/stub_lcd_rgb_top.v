// 仿真桩, 自动生成, 不要手改, 不进综合
`timescale 1ns/1ps
module lcd_rgb_top (
  sys_clk,
  sys_rst_n,
  sys_init_done,
  lcd_clk,
  lcd_hs,
  lcd_vs,
  lcd_de,
  lcd_rgb,
  lcd_bl,
  lcd_rst,
  lcd_pclk,
  lcd_id,
  out_vsync,
  pixel_xpos,
  pixel_ypos,
  h_disp,
  v_disp,
  data_in,
  data_req
);
  // 参数照抄原 IP
  localparam LCD_SELFTEST = 1'b0;
  input               sys_clk;
  input               sys_rst_n;
  input               sys_init_done;
  output              lcd_clk;
  output              lcd_hs;
  output              lcd_vs;
  output              lcd_de;
  inout  [23:0]       lcd_rgb;
  output              lcd_bl;
  output              lcd_rst;
  output              lcd_pclk;
  output [15:0]       lcd_id;
  output              out_vsync;
  output [11:0]       pixel_xpos;
  output [11:0]       pixel_ypos;
  output [10:0]       h_disp;
  output [10:0]       v_disp;
  input  [15:0]       data_in;
  output              data_req;
  assign lcd_clk = 0;
  assign lcd_hs = 0;
  assign lcd_vs = 0;
  assign lcd_de = 0;
  assign lcd_bl = 0;
  assign lcd_rst = 0;
  assign lcd_pclk = 0;
  assign lcd_id = 0;
  assign out_vsync = 0;
  assign pixel_xpos = 0;
  assign pixel_ypos = 0;
  assign h_disp = 0;
  assign v_disp = 0;
  assign data_req = 0;
endmodule
