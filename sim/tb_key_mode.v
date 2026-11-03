//**************************************************************************//
// tb_key_mode.v  -- v24 key -> 3-state display cycle, real RTL (IP stubbed)
//
//  1) power-on default = D_PCFULL(0)
//  2) each press of key[0](V12) advances 0->1->2->0, exactly one step
//  3) holding 25ms then releasing = one step only
//  4) 8ms of bounce before settling = one step only (debounce works)
//  5) key[1](T12) = force-go: pressing it must NOT change disp_mode
//
//  NOTE debounce is 20ms on BOTH edges, so a "release" also has to last
//  >=20ms before the next press can be recognised (real fingers are fine).
//**************************************************************************//
`timescale 1ns/1ps

module tb_key_mode;

    reg         sys_clk  = 0;
    reg         rst_n    = 0;
    reg  [3:0]  key      = 4'hF;
    reg         eth_rxc  = 0;
    reg         cam_pclk = 0;

    wire        eth_txc, eth_tx_ctl;
    wire [3:0]  eth_txd;
    wire        cam_rst_n, cam_pwdn, cam_scl;
    wire        mem_rst_n, mem_ck, mem_ck_n, mem_cke, mem_ras_n, mem_cas_n, mem_we_n, mem_odt, mem_cs_n;
    wire [14:0] mem_a;
    wire [2:0]  mem_ba;
    wire [1:0]  mem_dm;
    wire        lcd_hs, lcd_vs, lcd_de, lcd_bl, lcd_rst, lcd_pclk;
    wire [3:0]  led;

    tri  [1:0]  mem_dqs, mem_dqs_n;
    tri  [15:0] mem_dq;
    tri  [23:0] lcd_rgb;
    tri         cam_sda;

    assign mem_dqs   = 2'bz;
    assign mem_dqs_n = 2'bz;
    assign mem_dq    = 16'bz;
    assign lcd_rgb   = 24'bz;
    assign cam_sda   = 1'bz;

    eth_ddr3_lcd u_dut (
        .sys_clk    (sys_clk   ),
        .rst_n      (rst_n     ),
        .key        (key       ),
        .led        (led       ),
        .eth_rxc    (eth_rxc   ),
        .eth_rx_ctl (1'b0      ),
        .eth_rxd    (4'd0      ),
        .eth_txc    (eth_txc   ),
        .eth_tx_ctl (eth_tx_ctl),
        .eth_txd    (eth_txd   ),
        .cam_pclk   (cam_pclk  ),
        .cam_vsync  (1'b0      ),
        .cam_href   (1'b0      ),
        .cam_data   (8'd0      ),
        .cam_rst_n  (cam_rst_n ),
        .cam_pwdn   (cam_pwdn  ),
        .cam_scl    (cam_scl   ),
        .cam_sda    (cam_sda   ),
        .mem_rst_n  (mem_rst_n ),
        .mem_ck     (mem_ck    ),
        .mem_ck_n   (mem_ck_n  ),
        .mem_cke    (mem_cke   ),
        .mem_ras_n  (mem_ras_n ),
        .mem_cas_n  (mem_cas_n ),
        .mem_we_n   (mem_we_n  ),
        .mem_odt    (mem_odt   ),
        .mem_cs_n   (mem_cs_n  ),
        .mem_a      (mem_a     ),
        .mem_ba     (mem_ba    ),
        .mem_dqs    (mem_dqs   ),
        .mem_dqs_n  (mem_dqs_n ),
        .mem_dq     (mem_dq    ),
        .mem_dm     (mem_dm    ),
        .lcd_hs     (lcd_hs    ),
        .lcd_vs     (lcd_vs    ),
        .lcd_de     (lcd_de    ),
        .lcd_rgb    (lcd_rgb   ),
        .lcd_bl     (lcd_bl    ),
        .lcd_rst    (lcd_rst   ),
        .lcd_pclk   (lcd_pclk  )
    );

    always #10    sys_clk  = ~sys_clk;      // 50MHz
    always #4     eth_rxc  = ~eth_rxc;      // 125MHz
    always #20.8  cam_pclk = ~cam_pclk;

    integer err  = 0;
    integer step = 0;

    reg [1:0] dm_d = 2'd0;
    always @(posedge sys_clk) begin
        dm_d <= u_dut.disp_mode;
        if (u_dut.disp_mode !== dm_d)
            $display("    @%0d us  disp_mode %0d -> %0d", $time / 1000, dm_d, u_dut.disp_mode);
    end

    task check_mode(input [1:0] exp, input [255:0] tag);
        begin
            step = step + 1;
            if (u_dut.disp_mode === exp)
                $display("PASS %0d: %0s  => mode=%0d", step, tag, u_dut.disp_mode);
            else begin
                err = err + 1;
                $display("FAIL %0d: %0s  => mode=%0d (expected %0d)", step, tag, u_dut.disp_mode, exp);
            end
        end
    endtask

    task press_key1;
        begin
            key[0] = 1'b0; #25_000_000;        // hold 25ms  (debounce 20ms)
            key[0] = 1'b1; #25_000_000;        // release 25ms
        end
    endtask

    initial begin
        $display("==== v24 key -> 3-state cycle ====");
        #200;
        rst_n = 1'b1;
        #2_000_000;
        check_mode(2'd0, "power-on default: PC full screen");

        press_key1;  check_mode(2'd1, "press #1 -> quad");
        press_key1;  check_mode(2'd2, "press #2 -> camera full");
        press_key1;  check_mode(2'd0, "press #3 -> back to PC full");

        // bounce: 8ms of chatter, then a clean hold -> only one step
        key[0] = 1'b0; #1_000_000; key[0] = 1'b1; #700_000;
        key[0] = 1'b0; #1_200_000; key[0] = 1'b1; #500_000;
        key[0] = 1'b0; #900_000;   key[0] = 1'b1; #400_000;
        key[0] = 1'b0; #25_000_000;
        key[0] = 1'b1; #25_000_000;
        check_mode(2'd1, "8ms bounce then hold -> one step only");

        // long hold must not repeat
        key[0] = 1'b0; #25_000_000;
        check_mode(2'd2, "hold 25ms, not released -> one step");
        #30_000_000;
        check_mode(2'd2, "keep holding 30ms more -> no change");
        key[0] = 1'b1; #25_000_000;

        // key[1](T12) is force-go now: must NOT change disp_mode
        key[1] = 1'b0; #25_000_000;
        key[1] = 1'b1; #25_000_000;
        check_mode(2'd2, "press key[1](T12, force-go) -> disp_mode unchanged");

        press_key1;  check_mode(2'd0, "press -> wrap to 0");

        $display("==== result: %0d checks, %0d failed ====", step, err);
        if (err == 0) $display("ALL PASS");
        $finish;
    end

    initial begin
        #800_000_000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
