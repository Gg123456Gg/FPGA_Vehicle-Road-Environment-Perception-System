//=====================================================================
// tb_udp_cmd.v   (Verilog-2001)
//---------------------------------------------------------------------
// 测 udp_cmd_dec: 只认"包头是 0xA55B + 档位 + 取反校验"的 4 字节包,
// 其它一律不许动 cmd_tog。重点覆盖三类误触发场景:
//   1) 视频包 (0xA55A) 被当成命令
//   2) 负载中间随机出现 0xA5 0x5B
//   3) 校验错 / 档位越界 还照收
//=====================================================================
`timescale 1ns / 1ps

module tb_udp_cmd;

    reg         clk = 0;
    reg         rst_n = 0;
    reg         rec_en = 0;
    reg  [7:0]  rec_data = 0;
    wire        cmd_tog;
    wire [2:0]  cmd_mode;

    integer n_pass = 0;
    integer n_fail = 0;
    integer toggles = 0;

    always #4 clk = ~clk;              // 125MHz

    udp_cmd_dec dut(
        .clk      (clk      ),
        .rst_n    (rst_n    ),
        .rec_en   (rec_en   ),
        .rec_data (rec_data ),
        .cmd_tog  (cmd_tog  ),
        .cmd_mode (cmd_mode )
    );

    // 数 cmd_tog 翻了几次
    reg cmd_tog_d = 0;
    always @(posedge clk) begin
        cmd_tog_d <= cmd_tog;
        if (cmd_tog ^ cmd_tog_d) toggles = toggles + 1;
    end

    task chk(input [255:0] name, input integer cond);
        begin
            if (cond) begin
                n_pass = n_pass + 1;
                $display("  PASS  %0s", name);
            end else begin
                n_fail = n_fail + 1;
                $display("  FAIL  %0s", name);
            end
        end
    endtask

    // 打一个"包": 前面留 2 拍空闲, 逐字节送, 后面留 2 拍空闲
    task send_byte(input [7:0] b);
        begin
            @(negedge clk);
            rec_en   = 1'b1;
            rec_data = b;
        end
    endtask

    task end_pkt;
        begin
            @(negedge clk);
            rec_en = 1'b0;
            @(negedge clk);
            @(negedge clk);
        end
    endtask

    // 命令包: A5 5B mode ~mode
    task send_cmd(input [7:0] m);
        begin
            send_byte(8'hA5);
            send_byte(8'h5B);
            send_byte(m);
            send_byte(~m);
            end_pkt;
            repeat (3) @(negedge clk);
        end
    endtask

    integer i;
    reg [7:0] m;

    initial begin
        $display("==== tb_udp_cmd ====");
        repeat (5) @(negedge clk);
        rst_n = 1;
        repeat (5) @(negedge clk);

        // ---- 1. 正常命令 ----
        for (i = 0; i <= 4; i = i + 1) begin
            m = i[7:0];
            toggles = 0;
            send_cmd(m);
            chk("cmd mode", (cmd_mode == i[2:0]) && (toggles == 1));
        end

        // ---- 2. 校验错: 不许动 ----
        toggles = 0;
        send_byte(8'hA5); send_byte(8'h5B); send_byte(8'd3); send_byte(8'h00);
        end_pkt; repeat (3) @(negedge clk);
        chk("bad checksum ignored", toggles == 0 && cmd_mode == 3'd4);

        // ---- 3. 档位越界 (7): 不许动 ----
        toggles = 0;
        send_cmd(8'd7);
        chk("mode 7 ignored", toggles == 0 && cmd_mode == 3'd4);

        // ---- 4. 视频包 (0xA55A + 512 字节): 不许动 ----
        toggles = 0;
        send_byte(8'hA5); send_byte(8'h5A); send_byte(8'd0); send_byte(8'd0);
        for (i = 0; i < 512; i = i + 1) send_byte(i[7:0]);
        end_pkt; repeat (3) @(negedge clk);
        chk("video packet ignored", toggles == 0 && cmd_mode == 3'd4);

        // ---- 5. 视频包里"中间"埋一个完整的命令序列: 不许动 ----
        //    这是最容易骗过自由滑窗解码器的场景, 包头对齐后就该免疫
        toggles = 0;
        send_byte(8'hA5); send_byte(8'h5A); send_byte(8'd1); send_byte(8'd2);
        for (i = 0; i < 20; i = i + 1) send_byte(8'h00);
        send_byte(8'hA5); send_byte(8'h5B); send_byte(8'd2); send_byte(8'hFD);
        for (i = 0; i < 20; i = i + 1) send_byte(8'h00);
        end_pkt; repeat (3) @(negedge clk);
        chk("embedded cmd in payload ignored", toggles == 0 && cmd_mode == 3'd4);

        // ---- 6. 连发两条中间不留空 (PC 会连发 3 份) ----
        toggles = 0;
        send_byte(8'hA5); send_byte(8'h5B); send_byte(8'd1); send_byte(8'hFE);
        @(negedge clk); rec_en = 1'b0; @(negedge clk);
        send_byte(8'hA5); send_byte(8'h5B); send_byte(8'd1); send_byte(8'hFE);
        end_pkt; repeat (3) @(negedge clk);
        chk("back-to-back cmds both seen", toggles == 2 && cmd_mode == 3'd1);

        // ---- 7. 只有 2 字节的短包: 不许崩, 不许动 ----
        toggles = 0;
        send_byte(8'hA5); send_byte(8'h5B);
        end_pkt; repeat (3) @(negedge clk);
        chk("short packet safe", toggles == 0);

        $display("");
        $display("==== RESULT: %0d PASS, %0d FAIL ====", n_pass, n_fail);
        if (n_fail == 0) $display("ALL PASS");
        $finish;
    end

    initial begin
        #2000000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
