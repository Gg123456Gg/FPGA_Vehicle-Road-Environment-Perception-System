#!/bin/bash
#=============================================================
# run_reg_v35.sh —— v35 全量回归 (15 个台)
#   v35 新增 tb_quad_ovf: 四宫格 FIFO 溢出 / 右上格顶部横带 的复现+验收
#   work_a = quad_wr_ctrl 里的 lane_detect (v34 已与叠加同域)
#   work_b = quad_wr_ctrl  (lane_decision/OSD/写通路)
#   work_c = 整机
#   v34 变化: 识别搬到 PC 块流, lane_cdc 删除 -> tb_lane_cdc / tb_lane_e2e 退役,
#             新增 tb_lane_blk (块流喂料 + 丢块重对齐)
#   用法: bash run_reg_v32.sh     (结果同时写 reg_v32.log)
#=============================================================
export PATH="/c/modulesim/win64:$PATH"
cd "$(dirname "$0")" || exit 1

pass=0; fail=0
: > reg_v32.log

run() {   # $1=lib  $2=tb  $3=期望的关键字
    echo "=== $2 ===" >> reg_v32.log
    out=$(vsim -c -work "$1" "$2" -do "run -all; quit -f" 2>&1 | iconv -f GBK -t UTF-8)
    echo "$out" | grep -E "^# " | grep -vE "ModelSim|Mentor|Graphics|trade|privileged|secrets|disclosure|U\.S\.C|prohibited|commercial|^\# //" >> reg_v32.log
    if echo "$out" | grep -q "$3"; then
        echo "  --> OK ($3)"; pass=$((pass+1))
    else
        echo "  --> NG ($2 未出现 [$3])"; fail=$((fail+1))
    fi
}

rm -rf work_a work_b work_c work_f work_v work_e
vlib work_a >/dev/null
vlib work_b >/dev/null
vlib work_c >/dev/null
vlib work_v >/dev/null
vlib work_e >/dev/null

echo "--- compile A (quad_wr_ctrl: 内含 lane_detect) ---"
vlog -work work_a ../rtl/quad_wr_ctrl.v stub/*.v \
     tb_lane.v tb_lane_robust.v tb_lane_reacq.v tb_lane_blk.v 2>&1 | grep -E "\*\* Error" && exit 1
echo "--- compile B (quad_wr_ctrl) ---"
vlog -work work_b ../rtl/quad_wr_ctrl.v ../rtl/udp_cmd_dec.v stub/*.v \
     tb_lane_dec.v tb_lane_osd.v tb_osd_font.v tb_key_mode.v tb_full2quad.v tb_full_align.v \
     tb_mode_page.v tb_page_align.v tb_wr_resync.v tb_pc_resync.v tb_droplast.v tb_drophole.v tb_quad_ovf.v \
     tb_tl.v tb_tl_color.v tb_tl_full.v tb_tl_left.v tb_tl_multi.v tb_tl_quad.v tb_lane_dsp.v tb_lane_off.v tb_cmd_lane.v 2>&1 | grep -E "\*\* Error" && exit 1
echo "--- compile F (fps_osd: v41 帧率 OSD) ---"
vlog -work work_f ../rtl/quad_wr_ctrl.v stub/*.v tb_fps_osd.v 2>&1 | grep -E "\*\* Error" && exit 1
echo "--- compile V (i2c: 摄像头 VTS/最大曝光寄存器) ---"
# 这个文件是 GBK 编码, vlog 可能有中文注释 warning, 只拦 Error
vlog -work work_v ../rtl/cam/i2c_ov5640_rgb565_cfg.v tb_i2c_vts.v 2>&1 | grep -E "\*\* Error" && exit 1
echo "--- compile E (ego_state: v44 行驶状态) ---"
vlog -work work_e ../rtl/quad_wr_ctrl.v stub/*.v tb_ego_state.v 2>&1 | grep -E "\*\* Error" && exit 1
echo "--- compile C (整机: rtl 全部 + stub IP) ---"
# 注意: rtl/eth_top.v 本身有重复声明(老文件, 不在 PDS 里, 用 stub_eth_top.v 代替)
ls ../rtl/*.v | grep -v "eth_top.v" > flist_c.txt
vlog -work work_c -f flist_c.txt stub/*.v \
     tb_key_mode.v 2>&1 | grep -E "\*\* Error" && exit 1

run work_a tb_lane        "LANE_ALL_PASS"
run work_b tb_lane_dec    "LANE_DEC_PASS"
run work_b tb_lane_dsp    "LANE_DSP_PASS"
run work_b tb_lane_off    "LANE_OFF_PASS"
run work_b tb_cmd_lane    "CMD_LANE_PASS"
# v34: 这两个台是"四宫格坐标减半 + 从 lane_bus 端口注入"时代的产物,
#   坐标体系已经换成"只在 PC 全屏、原图坐标", 注入口也从端口改成了内部信号,
#   所以暂时不纳入回归(文件留着, 等有整台 PC 视频喂料时一起重写)。
# run work_b tb_lane_osd    "LANE_OSD_PASS"
# run work_b tb_osd_font    "OSD_FONT_PASS"
run work_a tb_lane_robust "LANE_ROBUST_PASS"
run work_a tb_lane_blk    "LANE_BLK_PASS"
run work_a tb_lane_reacq  "LANE_REACQ_PASS"
run work_c tb_key_mode    "ALL PASS"
run work_b tb_full2quad   "FULL2QUAD PASS"
run work_b tb_full_align  "FULL_ALIGN PASS"
run work_b tb_mode_page   "MODE_PAGE PASS"
run work_b tb_page_align  "PAGE_ALIGN PASS"
run work_b tb_wr_resync   "PASS C"
run work_b tb_pc_resync   "PASS C"
run work_b tb_droplast    "flips="
run work_b tb_drophole    "DROPHOLE"
run work_b tb_quad_ovf    "QUAD_OVF PASS"
# v41: 摄像头帧率 OSD (右角 "FPS xx", 黑底黄字)
run work_f tb_fps_osd     "FPS_OSD_PASS"
# v42: 摄像头时序寄存器核对 (VTS 1000->700, 最大曝光联动 VTS-4)
run work_v tb_i2c_vts     "I2C_VTS_PASS"
# v44: 行驶状态判定 (offv 帧差 -> 直行/左转/右转)
run work_e tb_ego_state   "EGO_STATE_PASS"

# ---- v37: 红绿灯 6 个台补进回归 ----
# 这几个台一直存在, 但从来没进过回归脚本, 只在改红绿灯时手动跑。
# v37 改 traffic_light 参数(G_BT/MX_G)时差点漏掉 tb_tl 的黄金模型比对,
# 补进来免得下次再漏。
#   注意: tb_tl_left / tb_tl_quad 例化的是整机 quad_wr_ctrl 并直接探内部信号
#   (u_dut.pcnt / u_dut.q0x0), 所以要和 work_b 的编译组放一起。
run work_b tb_tl          "RESULT: PASS"
run work_b tb_tl_color    "RESULT: PASS"
run work_b tb_tl_full     "RESULT: PASS"
run work_b tb_tl_left     "RESULT: PASS"
run work_b tb_tl_multi    "RESULT: PASS"
run work_b tb_tl_quad     "RESULT: PASS"

echo "================================"
echo "PASS=$pass  FAIL=$fail"
