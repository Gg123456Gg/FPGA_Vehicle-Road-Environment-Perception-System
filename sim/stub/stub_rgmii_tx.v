// 仿真桩, 自动生成, 不要手改, 不进综合
`timescale 1ns/1ps
module rgmii_tx (
  reset,
  gmii_tx_er,
  gmii_tx_clk,
  gmii_tx_en,
  gmii_txd,
  gmii_tx_clk_deg,
  rgmii_txc,
  rgmii_tx_ctl,
  rgmii_txd
);
  input               reset;
  input               gmii_tx_er;
  input               gmii_tx_clk;
  input               gmii_tx_en;
  input  [7:0]        gmii_txd;
  input               gmii_tx_clk_deg;
  output              rgmii_txc;
  output              rgmii_tx_ctl;
  output [3:0]        rgmii_txd;
  assign rgmii_txc = 0;
  assign rgmii_tx_ctl = 0;
  assign rgmii_txd = 0;
endmodule
