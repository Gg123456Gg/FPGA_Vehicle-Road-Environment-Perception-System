//**************************************************************************//
// File name   : udp_cmd_dec.v
// Descriptions: 从 UDP 负载里挑出"算法档位"命令包 (v24)
//               工作在 gmii_rx_clk (125MHz)
//---------------------------------------------------------------------//
//  命令包负载 (4 字节), v50 起有两条:
//    A) 算法档位
//       [0..1] magic 0xA55B  -> 视频包是 0xA55A, 靠头 2 字节区分
//       [2]    档位 0..4
//       [3]    ~档位          -> 单比特校验, 防误码
//    B) 车道识别【显示】开关 (v50 起)
//       [0..1] magic 0xA55C
//       [2]    1=显示 0=不显示     (v51: 只关 OSD 那层, 检测照跑)
//       [3]    ~值
//    两条命令用不同的 magic, 所以同一个解码器里各走各的分支,
//    互不影响 —— 老 PC 端只发 A55B, 行为与 v49 完全一致。
//
//  ★为什么可以旁路解码, 一点都不动 udp_video_rx★
//    udp_rx.v 只按 IP 地址过滤, 端口那两个字节压根没取; 所有发给板子 IP 的
//    UDP 负载都从同一组 rec_en/rec_data 出来。所以:
//      - 命令包可以走任意端口 (PC 直接发 1234 就行), 以太网那一层不用改
//      - 视频解析器见到 0xA55B 会因为 magic 不符自动走 S_SKIP,
//        像素流不会被污染, 也就不用动那条已经验证过的路径
//
//  ★为什么只看"包起点后 4 个字节"★
//    如果做成自由滑动的移位寄存器, 视频负载里随机字节凑出 0xA5 0x5B 就会
//    误触发 (一帧 60 万像素, 概率不低)。只在包起点对齐之后才判, 结构上排除。
//
//  ★为什么输出翻转标志而不是单拍脉冲★
//    这里 125MHz 一个脉冲只有 8ns, 而 clk_50m 的采样周期是 20ns ——
//    单拍脉冲会被整拍错过。翻转成电平让对面两级同步后取边沿才可靠。
//**************************************************************************//
module udp_cmd_dec(
    input  wire       clk,        // gmii_rx_clk, 125MHz
    input  wire       rst_n,
    input  wire       rec_en,     // 负载字节有效 (来自 udp_rx)
    input  wire [7:0] rec_data,
    output reg        cmd_tog,    // 每收到一条有效"档位"命令翻转一次
    output reg  [2:0] cmd_mode,   // 最近一次有效命令的档位
    output reg        lane_tog,   // 每收到一条有效"车道显示开关"命令翻转一次
    output reg        lane_on     // 1=显示 0=不显示 (上电默认 1)
);

    localparam [15:0] CMAGIC = 16'hA55B;
    localparam [15:0] LMAGIC = 16'hA55C;

    reg       rec_d;
    reg [1:0] bi;          // 本包第几个字节: 0..3
    reg [7:0] b0;          // 第 0 字节
    reg [7:0] bd;          // 第 3 字节 (第 2 字节只参与 magic 比较, 不用存)
    reg       hit;         // 头 2 字节 == A55B (档位命令)
    reg       lhs;         // 头 2 字节 == A55C (车道开关命令)

    wire sop = rec_en & ~rec_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rec_d    <= 1'b0;
            bi       <= 2'd0;
            b0       <= 8'd0;
            bd       <= 8'd0;
            hit      <= 1'b0;
            lhs      <= 1'b0;
            cmd_tog  <= 1'b0;
            cmd_mode <= 3'd0;
            lane_tog <= 1'b0;
            lane_on  <= 1'b1;      // ★上电默认开★ 没有命令时和 v49 行为一致
        end else begin
            rec_d <= rec_en;

            if (!rec_en) begin
                // 包结束: 下一个包重新对齐
                bi  <= 2'd0;
                hit <= 1'b0;
                lhs <= 1'b0;
            end
            else if (sop) begin
                // 包起点那一拍, rec_data 就是第 0 个字节
                b0  <= rec_data;
                bi  <= 2'd1;
                hit <= 1'b0;
                lhs <= 1'b0;
            end
            else begin
                case (bi)
                    2'd1: begin
                        hit <= ({b0, rec_data} == CMAGIC);
                        lhs <= ({b0, rec_data} == LMAGIC);
                        bi  <= 2'd2;
                    end
                    2'd2: begin
                        bd <= rec_data;
                        bi <= 2'd3;
                    end
                    2'd3: begin
                        // 校验: 值取反要等于第 4 字节
                        if (rec_data == ~bd) begin
                            if (hit && (bd <= 8'd4)) begin
                                cmd_mode <= bd[2:0];
                                cmd_tog  <= ~cmd_tog;
                            end else if (lhs && (bd <= 8'd1)) begin
                                lane_on  <= bd[0];
                                lane_tog <= ~lane_tog;
                            end
                        end
                        bi  <= 2'd0;
                        hit <= 1'b0;
                        lhs <= 1'b0;
                    end
                    default: bi <= 2'd0;   // 多出来的字节一律不理
                endcase
            end
        end
    end

endmodule
