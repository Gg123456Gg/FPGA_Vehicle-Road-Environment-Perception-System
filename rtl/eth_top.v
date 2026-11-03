//****************************************Copyright (c)***********************************//
//原子哥在线教学平台：www.yuanzige.com
//技术支持：www.openedv.com
//淘宝店铺：http://openedv.taobao.com 
//关注微信公众平台微信号："正点原子"，免费获取ZYNQ & FPGA & STM32 & LINUX资料。
//版权所有，盗版必究。
//Copyright(C) 正点原子 2018-2028
//All rights reserved                                  
//----------------------------------------------------------------------------------------
// File name:           eth_udp_loop
// Last modified Date:  2020/2/18 9:20:14
// Last Version:        V1.0
// Descriptions:        以太网通信UDP通信环回顶层模块
//----------------------------------------------------------------------------------------
// Created by:          正点原子
// Created date:        2020/2/18 9:20:14
// Version:             V1.0
// Descriptions:        The original version
//
//----------------------------------------------------------------------------------------
//****************************************************************************************//

module eth_top(
    input              sys_clk   , //系统时钟
    input              sys_rst_n , //系统复位信号，低电平有效 
    //以太网RGMII接口   
    input              eth_rxc   , //RGMII接收数据时钟
    input              eth_rx_ctl, //RGMII输入数据有效信号
    input       [3:0]  eth_rxd   , //RGMII输入数据
    output             eth_txc   , //RGMII发送数据时钟    
    output             eth_tx_ctl, //RGMII输出数据有效信号
    output      [3:0]  eth_txd    , //RGMII输出数据   
    output             udp_rec_en ,
    output      [7:0]  udp_rec_data,
    output             gmii_rx_clk  ,
    input              credit_tog  ,   // flow control: one toggle per credit
    input       [31:0] credit_val  ,   // flow control: consumed pixel counter
    input      [127:0] tl_bus          // telemetry bus, quasi-static (50MHz domain)
    );

//parameter define
//开发板MAC地址 00-11-22-33-44-55
parameter  BOARD_MAC = 48'h00_11_22_33_44_55;     
//开发板IP地址 192.168.1.10
parameter  BOARD_IP  = {8'd192,8'd168,8'd1,8'd10};  
//目的MAC地址 ff_ff_ff_ff_ff_ff
parameter  DES_MAC   = 48'hff_ff_ff_ff_ff_ff;    
//目的IP地址 192.168.1.102     
parameter  DES_IP    = {8'd192,8'd168,8'd1,8'd102};  

//wire define					  
wire          gmii_rx_clk		  ; //GMII接收时钟
wire          gmii_rx_dv 		  ; //GMII接收数据有效信号
wire  [7:0]   gmii_rxd   		  ; //GMII接收数据
wire          gmii_tx_clk		  ; //GMII发送时钟
wire          gmii_tx_en 		  ; //GMII发送数据使能信号
wire  [7:0]   gmii_txd   		  ; //GMII发送数据     

wire          arp_gmii_tx_en	  ; //ARP GMII输出数据有效信号 
wire  [7:0]   arp_gmii_txd  	  ; //ARP GMII输出数据
wire          arp_rx_done   	  ; //ARP接收完成信号
wire          arp_rx_type   	  ; //ARP接收类型 0:请求  1:应答
wire  [47:0]  src_mac       	  ; //接收到目的MAC地址
wire  [31:0]  src_ip        	  ; //接收到目的IP地址    
wire          arp_tx_en     	  ; //ARP发送使能信号
wire          arp_tx_type   	  ; //ARP发送类型 0:请求  1:应答
wire  [47:0]  des_mac       	  ; //发送的目标MAC地址
wire  [31:0]  des_ip        	  ; //发送的目标IP地址   
wire          arp_tx_done   	  ; //ARP发送完成信号

wire          icmp_gmii_tx_en	  ; //ICMP GMII输出数据有效信号 
wire  [7:0]   icmp_gmii_txd  	  ; //ICMP GMII输出数据
wire          icmp_rec_pkt_done   ; //ICMP单包数据接收完成信号
wire          icmp_rec_en         ; //ICMP接收的数据使能信号
wire  [ 7:0]  icmp_rec_data       ; //ICMP接收的数据
wire  [15:0]  icmp_rec_byte_num   ; //ICMP接收的有效字节数 单位:byte 
wire  [15:0]  icmp_tx_byte_num    ; //ICMP发送的有效字节数 单位:byte 
wire          icmp_tx_done   	  ; //ICMP发送完成信号
wire          icmp_tx_req         ; //ICMP读数据请求信号
wire  [ 7:0]  icmp_tx_data        ; //ICMP待发送数据
wire          icmp_tx_start_en    ; //ICMP发送开始使能信号

wire          udp_gmii_tx_en	  ; //UDP GMII输出数据有效信号 
wire  [7:0]   udp_gmii_txd  	  ; //UDP GMII输出数据
wire          rec_pkt_done  	  ; //UDP单包数据接收完成信号
wire          udp_rec_en    	  ; //UDP接收的数据使能信号
wire  [ 7:0]  udp_rec_data  	  ; //UDP接收的数据
wire  [15:0]  rec_byte_num  	  ; //UDP接收的有效字节数 单位:byte 
reg   [15:0]  tx_byte_num   ; //UDP发送的有效字节数 单位:byte 
wire          udp_tx_done   	  ; //UDP发送完成信号
wire          udp_tx_req    	  ; //UDP读数据请求信号
wire  [ 7:0]  udp_tx_data   	  ; //UDP待发送数据
reg           tx_start_en   ; //UDP发送开始使能信号
								  
wire  [7:0]	  rec_data			  ;
wire		  rec_en			  ;
wire		  tx_req			  ;
wire  [7:0]	  tx_data	    	  ;
//*****************************************************
//**                    main code
//*****************************************************

assign icmp_tx_start_en = icmp_rec_pkt_done;
assign icmp_tx_byte_num = icmp_rec_byte_num;

//=====================================================================
//  flow control : credit packet sender   (gmii_rx_clk domain)
//---------------------------------------------------------------------
//  board sends a 20-byte UDP packet (broadcast) every 1024 consumed PC
//  pixels.  payload : 'C','R',0x01,0x00, counter[31:0] big-endian.
//  PC only transmits while (sent - counter) <= window, so the on-board
//  pc_fifo can never overflow.  Without this the composer stalls while
//  waiting for the camera, PC data piles up, wr_fifo silently drops
//  pixels and the DDR address/data relationship is destroyed.
//=====================================================================
reg         ct_tog_d0, ct_tog_d1;
reg  [31:0] ct_val_d0, ct_val_d1, ct_val;
reg [127:0] tl_d0, tl_d1, tl_val;
reg         ct_busy  ;
reg  [5:0]  ct_idx   ;
reg  [7:0]  ct_data  ;
reg         txq_d     ;
reg  [16:0] ct_wdog   ;
wire        ct_edge  = ct_tog_d0 ^ ct_tog_d1;

always @(posedge gmii_rx_clk or negedge sys_rst_n) begin
    if(!sys_rst_n) begin
        ct_tog_d0 <= 1'b0; ct_tog_d1 <= 1'b0;
        ct_val_d0 <= 32'd0; ct_val_d1 <= 32'd0;
        tl_d0 <= 128'd0; tl_d1 <= 128'd0; tl_val <= 128'd0;
        ct_busy <= 1'b0; ct_idx <= 6'd0; ct_data <= 8'd0; txq_d <= 1'b0;
        ct_wdog <= 17'd0;
        tx_start_en <= 1'b0; tx_byte_num <= 16'd0;
    end
    else begin
        ct_tog_d0 <= credit_tog;
        ct_tog_d1 <= ct_tog_d0;
        ct_val_d0 <= credit_val;
        ct_val_d1 <= ct_val_d0;
        tl_d0 <= tl_bus;
        tl_d1 <= tl_d0;
        tx_start_en <= 1'b0;

        if(!ct_busy) begin
            if(ct_edge) begin
                ct_val      <= ct_val_d1;
                tl_val      <= tl_d1;
                ct_idx      <= 6'd0;
                ct_data     <= 8'h43;      // 'C'
                ct_busy     <= 1'b1;
                ct_wdog     <= 17'd0;
                tx_start_en <= 1'b1;
                tx_byte_num <= 16'd39;     // 37 bytes: byte0 is eaten by the
                                           // udp_tx pipeline, so 1..36 arrive
            end
        end
        else begin
            if(tx_req) begin
                ct_idx <= ct_idx + 6'd1;
                case(ct_idx + 6'd1)
                    //  0='C'  1='R'  2..5 = counter (big endian)
                    //  the 'C','R'+counter block is repeated 3 times so the
                    //  PC can find a byte-aligned copy of it.
                    6'd1 : ct_data <= 8'h52;
                    6'd2 : ct_data <= ct_val[31:24];
                    6'd3 : ct_data <= ct_val[23:16];
                    6'd4 : ct_data <= ct_val[15:8];
                    6'd5 : ct_data <= ct_val[7:0];
                    6'd6 : ct_data <= 8'h43;
                    6'd7 : ct_data <= 8'h52;
                    6'd8 : ct_data <= ct_val[31:24];
                    6'd9 : ct_data <= ct_val[23:16];
                    6'd10: ct_data <= ct_val[15:8];
                    6'd11: ct_data <= ct_val[7:0];
                    6'd12: ct_data <= 8'h43;
                    6'd13: ct_data <= 8'h52;
                    6'd14: ct_data <= ct_val[31:24];
                    6'd15: ct_data <= ct_val[23:16];
                    6'd16: ct_data <= ct_val[15:8];
                    6'd17: ct_data <= ct_val[7:0];
                    //  18,19 = 0 pad
                    6'd18: ct_data <= 8'h00;
                    6'd19: ct_data <= 8'h00;
                    //  ---- telemetry block: 'T' 'L' + 14 bytes ----
                    //  20='T' 21='L'
                    //  22..25 cam pixels consumed   26..29 camera vsync count
                    //  30..31 composite ox           32..33 composite oy
                    //  34     flags                  35     cam fifo level/128
                    6'd20: ct_data <= 8'h54;
                    6'd21: ct_data <= 8'h4C;
                    6'd22: ct_data <= tl_val[127:120];
                    6'd23: ct_data <= tl_val[119:112];
                    6'd24: ct_data <= tl_val[111:104];
                    6'd25: ct_data <= tl_val[103:96];
                    6'd26: ct_data <= tl_val[95:88];
                    6'd27: ct_data <= tl_val[87:80];
                    6'd28: ct_data <= tl_val[79:72];
                    6'd29: ct_data <= tl_val[71:64];
                    6'd30: ct_data <= tl_val[63:56];
                    6'd31: ct_data <= tl_val[55:48];
                    6'd32: ct_data <= tl_val[47:40];
                    6'd33: ct_data <= tl_val[39:32];
                    6'd34: ct_data <= tl_val[31:24];
                    6'd35: ct_data <= tl_val[23:16];
                    6'd36: ct_data <= tl_val[15:8];
                    6'd37: ct_data <= tl_val[7:0];
                    default : ct_data <= 8'h00;
                endcase
            end
            // watchdog: 1ms. a 37-byte UDP frame takes far less than that,
            // so tx_done should always come; this only un-sticks the FSM.
            ct_wdog <= (ct_wdog == 17'h1FFFF) ? 17'h1FFFF : (ct_wdog + 17'd1);
            if(udp_tx_done || (ct_wdog == 17'h1FFFF))
                ct_busy <= 1'b0;
        end
    end
end

assign tx_data = ct_data;

assign des_mac = src_mac;
assign des_ip = src_ip;


//GMII接口转RGMII接口
gmii_to_rgmii u_gmii_to_rgmii(
    .gmii_rx_clk   (gmii_rx_clk ),
    .gmii_rx_dv    (gmii_rx_dv  ),
    .gmii_rxd      (gmii_rxd    ),
    .gmii_tx_clk   (gmii_tx_clk ),
    .gmii_tx_en    (gmii_tx_en  ),
    .gmii_txd      (gmii_txd    ),
    
    .rgmii_rxc     (eth_rxc     ),
    .rgmii_rx_ctl  (eth_rx_ctl  ),
    .rgmii_rxd     (eth_rxd     ),
    .rgmii_txc     (eth_txc     ),
    .rgmii_tx_ctl  (eth_tx_ctl  ),
    .rgmii_txd     (eth_txd     )
    );

//ARP通信
arp                                             
   #(
    .BOARD_MAC     (BOARD_MAC),      //参数例化
    .BOARD_IP      (BOARD_IP ),
    .DES_MAC       (DES_MAC  ),
    .DES_IP        (DES_IP   )
    )
   u_arp(
    .rst_n         (sys_rst_n  	  ),
                    
    .gmii_rx_clk   (gmii_rx_clk	  ),
    .gmii_rx_dv    (gmii_rx_dv 	  ),
    .gmii_rxd      (gmii_rxd   	  ),
    .gmii_tx_clk   (gmii_tx_clk	  ),
    .gmii_tx_en    (arp_gmii_tx_en),
    .gmii_txd      (arp_gmii_txd  ),
                    
    .arp_rx_done   (arp_rx_done	  ),
    .arp_rx_type   (arp_rx_type	  ),
    .src_mac       (src_mac    	  ),
    .src_ip        (src_ip     	  ),
    .arp_tx_en     (arp_tx_en  	  ),
    .arp_tx_type   (arp_tx_type	  ),
    .des_mac       (des_mac    	  ),
    .des_ip        (des_ip     	  ),
    .tx_done       (arp_tx_done	  )
    );

//ICMP通信
icmp                                             
   #(
    .BOARD_MAC     (BOARD_MAC),      //参数例化
    .BOARD_IP      (BOARD_IP ),
    .DES_MAC       (DES_MAC  ),
    .DES_IP        (DES_IP   )
    )
   u_icmp(
    .rst_n         (sys_rst_n   	 ),  
		 
    .gmii_rx_clk   (gmii_rx_clk 	 ),           
    .gmii_rx_dv    (gmii_rx_dv  	 ),         
    .gmii_rxd      (gmii_rxd    	 ),                   
    .gmii_tx_clk   (gmii_tx_clk 	 ), 
    .gmii_tx_en    (icmp_gmii_tx_en	 ),         
    .gmii_txd      (icmp_gmii_txd	 ),  

    .rec_pkt_done  (icmp_rec_pkt_done),    
    .rec_en        (icmp_rec_en      ), 		  
    .rec_data      (icmp_rec_data    ),   	    
    .rec_byte_num  (icmp_rec_byte_num),      
    .tx_start_en   (icmp_tx_start_en ),        
    .tx_data       (icmp_tx_data     ),       
    .tx_byte_num   (icmp_tx_byte_num ),  
    .des_mac       (des_mac     	 ),
    .des_ip        (des_ip      	 ),    
    .tx_done       (icmp_tx_done	 ),        
    .tx_req        (icmp_tx_req      )        
    ); 

//UDP通信
udp                                             
   #(
    .BOARD_MAC     (BOARD_MAC),      //参数例化
    .BOARD_IP      (BOARD_IP ),
    .DES_MAC       (DES_MAC  ),
    .DES_IP         ({8'd192,8'd168,8'd1,8'd255})
    )
   u_udp(
    .rst_n         (sys_rst_n   ),  
    
    .gmii_rx_clk   (gmii_rx_clk ),           
    .gmii_rx_dv    (gmii_rx_dv  ),         
    .gmii_rxd      (gmii_rxd    ),                   
    .gmii_tx_clk   (gmii_tx_clk ), 
    .gmii_tx_en    (udp_gmii_tx_en),         
    .gmii_txd      (udp_gmii_txd),  

    .rec_pkt_done  (rec_pkt_done),    
    .rec_en        (udp_rec_en  ),     
    .rec_data      (udp_rec_data),         
    .rec_byte_num  (rec_byte_num),      
    .tx_start_en   (tx_start_en ),        
    .tx_data       (udp_tx_data ),         
    .tx_byte_num   (tx_byte_num ),  
    .des_mac        (48'hff_ff_ff_ff_ff_ff),
    .des_ip        ({8'd192,8'd168,8'd1,8'd255}), // 广播: 不依赖 ARP 学到的 src_ip
    .tx_done       (udp_tx_done ),        
    .tx_req        (udp_tx_req  )           
    ); 

//异步FIFO
sync_fifo_2048x8b sync_fifo_2048x8b_inst (
  .clk(gmii_rx_clk),                      // input
  .rst(~sys_rst_n),                       // input
  .wr_en(1'b0),                     // 原 rec_en，防止溢出
  .wr_data(rec_data),                     // input [31:0]
  .wr_full(),                             // output
  .almost_full(),                         // output
  .rd_en(tx_req),                         // input
  .rd_data(),                      // output [31:0]
  .rd_empty(),                            // output
  .almost_empty()                         // output
);

//以太网控制模块
eth_ctrl u_eth_ctrl(
    .clk            	(gmii_rx_clk	 ),
    .rst_n          	(sys_rst_n		 ),
										 
    .arp_rx_done    	(arp_rx_done   	 ),
    .arp_rx_type    	(arp_rx_type   	 ),
    .arp_tx_en      	(arp_tx_en     	 ),
    .arp_tx_type    	(arp_tx_type   	 ),
    .arp_tx_done    	(arp_tx_done   	 ),
    .arp_gmii_tx_en 	(arp_gmii_tx_en	 ),
    .arp_gmii_txd   	(arp_gmii_txd  	 ),
										 
    .icmp_tx_start_en	(icmp_tx_start_en),
    .icmp_tx_done		(icmp_tx_done	 ),
    .icmp_gmii_tx_en	(icmp_gmii_tx_en ),
    .icmp_gmii_txd		(icmp_gmii_txd	 ),
										 
	.icmp_rec_en       	(icmp_rec_en     ),
	.icmp_rec_data     	(icmp_rec_data   ),
	.icmp_tx_req       	(icmp_tx_req     ),
	.icmp_tx_data      	(icmp_tx_data    ),
										 
    .udp_tx_start_en	(tx_start_en   	 ),
    .udp_tx_done    	(udp_tx_done   	 ),    
    .udp_gmii_tx_en 	(udp_gmii_tx_en	 ),
    .udp_gmii_txd   	(udp_gmii_txd  	 ),
										 
	.udp_rec_data		(udp_rec_data	 ),		
	.udp_rec_en			(udp_rec_en		 ),    
	.udp_tx_req			(udp_tx_req		 ),    
	.udp_tx_data		(udp_tx_data	 ),		
										 
	.rec_data			(rec_data	     ),	
	.rec_en	        	(rec_en	         ),
    .tx_req	        	(tx_req	         ),
	.tx_data	    	(tx_data	     ),
										 
    .gmii_tx_en     	(gmii_tx_en    	 ),
    .gmii_txd       	(gmii_txd      	 )
    );
	
endmodule