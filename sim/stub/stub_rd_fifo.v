// 仿真桩, 自动生成, 不要手改, 不进综合
`timescale 1ns/1ps
module rd_fifo (
  write,
  read,
  wr_data,
  wr_en,
  wr_clk,
  wr_rst,
  wr_full,
  almost_full,
  wr_water_level,
  rd_data,
  rd_en,
  rd_clk,
  rd_rst,
  rd_empty,
  almost_empty,
  rd_water_level,
  ister
);
  // 参数照抄原 IP
  localparam WR_DEPTH_WIDTH = 10 ;
  localparam WR_DATA_WIDTH = 128 ;
  localparam RD_DEPTH_WIDTH = 13 ;
  localparam RD_DATA_WIDTH = 16 ;
  localparam OUTPUT_REG = 0 ;
  localparam RD_OCE_EN = 0 ;
  localparam RD_CLK_OR_POL_INV = 0 ;
  localparam RESET_TYPE = "ASYNC" ;
  localparam POWER_OPT = 0 ;
  localparam WR_BYTE_EN = 0 ;
  localparam BE_WIDTH = 1 ;
  localparam FIFO_TYPE = "ASYN_FIFO" ;
  localparam ASYN_FIFO_EN = "1" ;
  localparam ALMOST_FULL_NUM = 252 ;
  localparam ALMOST_EMPTY_NUM = 4 ;
  localparam FULL_WL_EN = 1 ;
  localparam EMPTY_WL_EN = 1 ;
  localparam BYTE_SIZE = 8 ;
  localparam  FIFO_TYPE_SEL      = (FIFO_TYPE=="SYN_FIFO") ? "SYN" : "ASYN" ;
  localparam  DEVICE_NAME        = "PGL25G";
  localparam  WR_DATA_WIDTH_WRAP = ((DEVICE_NAME == "PGT30G") && (WR_DATA_WIDTH <= 9)) ? 10 : WR_DATA_WIDTH;
  localparam  RD_DATA_WIDTH_WRAP = ((DEVICE_NAME == "PGT30G") && (RD_DATA_WIDTH <= 9)) ? 10 : RD_DATA_WIDTH;
  localparam  SIM_DEVICE         = ((DEVICE_NAME == "PGL22G") || (DEVICE_NAME == "PGL22GS")) ? "PGL22G" : "LOGOS";
  input               write;
  input               read;
  input  [WR_DATA_WIDTH-1 : 0] wr_data;
  input               wr_en;
  input               wr_clk;
  input               wr_rst;
  output              wr_full;
  output              almost_full;
  output [WR_DEPTH_WIDTH : 0] wr_water_level;
  output [RD_DATA_WIDTH-1 : 0] rd_data;
  input               rd_en;
  input               rd_clk;
  input               rd_rst;
  output              rd_empty;
  output              almost_empty;
  output [RD_DEPTH_WIDTH : 0] rd_water_level;
  output              ister;
  assign wr_full = 0;
  assign almost_full = 0;
  assign wr_water_level = 0;
  assign rd_data = 0;
  assign rd_empty = 0;
  assign almost_empty = 0;
  assign rd_water_level = 0;
  assign ister = 0;
endmodule
