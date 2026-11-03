// 仿真桩, 自动生成, 不要手改, 不进综合
`timescale 1ns/1ps
module ov5640_dri (
  clk,
  rst_n,
  cam_pclk,
  cam_vsync,
  cam_href,
  cam_data,
  cam_rst_n,
  cam_pwdn,
  cam_scl,
  cam_sda,
  cmos_h_pixel,
  cmos_v_pixel,
  total_h_pixel,
  total_v_pixel,
  y_addr_st,
  y_addr_end,
  capture_start,
  cam_init_done,
  cmos_frame_vsync,
  cmos_frame_href,
  cmos_frame_valid,
  cmos_frame_data
);
  input               clk;
  input               rst_n;
  input               cam_pclk;
  input               cam_vsync;
  input               cam_href;
  input  [7:0]        cam_data;
  output              cam_rst_n;
  output              cam_pwdn;
  output              cam_scl;
  inout               cam_sda;
  input  [12:0]       cmos_h_pixel;
  input  [12:0]       cmos_v_pixel;
  input  [12:0]       total_h_pixel;
  input  [12:0]       total_v_pixel;
  input  [12:0]       y_addr_st;
  input  [12:0]       y_addr_end;
  input               capture_start;
  output              cam_init_done;
  output              cmos_frame_vsync;
  output              cmos_frame_href;
  output              cmos_frame_valid;
  output [15:0]       cmos_frame_data;
  assign cam_rst_n = 0;
  assign cam_pwdn = 0;
  assign cam_scl = 0;
  assign cam_init_done = 0;
  assign cmos_frame_vsync = 0;
  assign cmos_frame_href = 0;
  assign cmos_frame_valid = 0;
  assign cmos_frame_data = 0;
endmodule
