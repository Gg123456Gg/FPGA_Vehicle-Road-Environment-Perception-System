// 仿真桩, 自动生成, 不要手改, 不进综合
`timescale 1ns/1ps
module eth_top #(
  parameter BOARD_MAC = 48'h00_11_22_33_44_55,
  parameter BOARD_IP  = {8'd192,8'd168,8'd1,8'd10},
  parameter DES_MAC   = 48'hff_ff_ff_ff_ff_ff,
  parameter DES_IP    = {8'd192,8'd168,8'd1,8'd102}
) (

  sys_clk,
  sys_rst_n,
  eth_rxc,
  eth_rx_ctl,
  eth_rxd,
  eth_txc,
  eth_tx_ctl,
  eth_txd,
  udp_rec_en,
  udp_rec_data,
  gmii_rx_clk,
  credit_tog,
  credit_val,
  tl_bus
);
  input               sys_clk;
  input               sys_rst_n;
  input               eth_rxc;
  input               eth_rx_ctl;
  input  [3:0]        eth_rxd;
  output              eth_txc;
  output              eth_tx_ctl;
  output [3:0]        eth_txd;
  output              udp_rec_en;
  output [7:0]        udp_rec_data;
  output              gmii_rx_clk;
  input               credit_tog;
  input  [31:0]       credit_val;
  input  [127:0]      tl_bus;
  assign eth_txc = 0;
  assign eth_tx_ctl = 0;
  assign eth_txd = 0;
  assign udp_rec_en = 0;
  assign udp_rec_data = 0;
  assign gmii_rx_clk = 0;
endmodule
