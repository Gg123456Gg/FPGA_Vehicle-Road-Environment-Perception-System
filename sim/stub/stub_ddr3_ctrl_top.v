// 仿真桩, 自动生成, 不要手改, 不进综合
`timescale 1ns/1ps
module ddr3_ctrl_top (
  clk,
  wd_clk,
  rd_clk,
  rst_n,
  wd_en,
  wd_data,
  rd_en,
  rd_data,
  wr_load,
  rd_load,
  ddr3_pingpang_en,
  ddr3_read_valid,
  wd_a_wr,
  wd_a_data,
  wd_a_level,
  wr_page,
  addr_rd_min,
  addr_rd_max,
  rd_burst_len,
  addr_wd_min,
  addr_wd_max,
  wd_burst_len,
  ddr3_init_done,
  mem_rst_n,
  mem_ck,
  mem_ck_n,
  mem_cke,
  mem_ras_n,
  mem_cas_n,
  mem_we_n,
  mem_odt,
  mem_cs_n,
  mem_a,
  mem_ba,
  mem_dqs,
  mem_dqs_n,
  mem_dq,
  mem_dm
);
  input               clk;
  input               wd_clk;
  input               rd_clk;
  input               rst_n;
  input               wd_en;
  input  [15:0]       wd_data;
  input               rd_en;
  output [15:0]       rd_data;
  input               wr_load;
  input               rd_load;
  input               ddr3_pingpang_en;
  input               ddr3_read_valid;
  input               wd_a_wr;
  input  [27:0]       wd_a_data;
  output [4:0]        wd_a_level;
  input               wr_page;
  input  [27:0]       addr_rd_min;
  input  [27:0]       addr_rd_max;
  input  [9:0]        rd_burst_len;
  input  [27:0]       addr_wd_min;
  input  [27:0]       addr_wd_max;
  input  [9:0]        wd_burst_len;
  output              ddr3_init_done;
  output              mem_rst_n;
  output              mem_ck;
  output              mem_ck_n;
  output              mem_cke;
  output              mem_ras_n;
  output              mem_cas_n;
  output              mem_we_n;
  output              mem_odt;
  output              mem_cs_n;
  output [14:0]       mem_a;
  output [2:0]        mem_ba;
  inout  [1:0]        mem_dqs;
  inout  [1:0]        mem_dqs_n;
  inout  [15:0]       mem_dq;
  output [1:0]        mem_dm;
  assign rd_data = 0;
  assign wd_a_level = 0;
  assign ddr3_init_done = 0;
  assign mem_rst_n = 0;
  assign mem_ck = 0;
  assign mem_ck_n = 0;
  assign mem_cke = 0;
  assign mem_ras_n = 0;
  assign mem_cas_n = 0;
  assign mem_we_n = 0;
  assign mem_odt = 0;
  assign mem_cs_n = 0;
  assign mem_a = 0;
  assign mem_ba = 0;
  assign mem_dm = 0;
endmodule
