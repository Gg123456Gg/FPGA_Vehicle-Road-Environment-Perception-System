// 仿真桩, 自动生成, 不要手改, 不进综合
`timescale 1ns/1ps
module rgmii_rx (
  rgmii_rxc,
  rgmii_rx_ctl,
  rgmii_rxd,
  gmii_rx_clk,
  gmii_rx_dv,
  gmii_rxd,
  gmii_tx_clk_deg,
  pll_lock
);
  input               rgmii_rxc;
  input               rgmii_rx_ctl;
  input  [3:0]        rgmii_rxd;
  output              gmii_rx_clk;
  output              gmii_rx_dv;
  output [7:0]        gmii_rxd;
  output              gmii_tx_clk_deg;
  output              pll_lock;
  assign gmii_rx_clk = 0;
  assign gmii_rx_dv = 0;
  assign gmii_rxd = 0;
  assign gmii_tx_clk_deg = 0;
  assign pll_lock = 0;
endmodule
