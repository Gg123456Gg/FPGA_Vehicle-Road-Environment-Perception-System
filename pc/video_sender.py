#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
video_sender.py  --  FPGA 连续视频发送端 (UDP, 无握手, 零第三方依赖)

协议 (每个 UDP 负载 = 8 字节头 + N*2 字节像素):
    头(大端): magic(0xA55A) | frame_id | seq | npix
    - magic    : 固定 0xA55A, FPGA 用它对齐/丢弃错包
    - frame_id : 帧号, 每帧 +1
    - seq      : 本包在帧内序号, ==0 表示一帧的第一包
    - npix     : 本包像素个数
    负载: 高字节在前, 每 2 字节一个 RGB565 像素

三种模式:
  1) 全屏模式 (默认)        : 一帧 1024x600 = 614400 像素
  2) 全屏四宫格 (--tile)    : 把 512x300 源视频平铺成 1024x600 再发
  3) 硬件四宫格 (--quad)    : 只发 FPGA 真正需要的 460800 像素,
                              左上那格留给 OV5640 摄像头。
                              >>> 这是配合摄像头分屏用的正确模式 <<<

--quad 的数据排布 (每帧 900 包, 每包 512 像素):
    包 0   .. 299  : 源图第 0..299 行  -> 正好落在屏幕右上半区
    包 300 .. 899  : 源图第 0..299 行, 每行发两遍 -> 落在屏幕下半区整行
  为什么分两段节奏发: 上半区由摄像头的行节拍拖着走(慢), 下半区在摄像头
  消隐期一次性灌完(快)。不分段的话 PC 会先灌满缓冲, 导致下半区提前溢出。

用法:
    python video_sender.py --quad --video v512.rgb565 --fps 0
    python video_sender.py --quad                       # 内置测试图案
    python video_sender.py --tile --video v512.rgb565   # 纯 PC 四宫格(无摄像头)
    python video_sender.py --fps 30                     # 全屏限速

把 mp4 转成裸 RGB565 (高字节在前, 必须 be):
    ffmpeg -i in.mp4 -vf "scale=512:300,fps=30" -f rawvideo -pix_fmt rgb565be -y v512.rgb565
"""
import socket
import struct
import time
import math
import argparse
import os
import mmap
import subprocess

# ---- 别在 GUI 里弹出黑窗口 (v22) ----
#   ffmpeg 是控制台程序。从「命令行 python video_sender.py」里启动它时, 它继承
#   当前控制台, 什么都不会多出来; 但从 GUI exe (windowed, 没有控制台) 里启动,
#   Windows 会给它新开一个黑色 cmd 窗口 —— 传输过程中一直挂在那儿, 还抢焦点。
#   CREATE_NO_WINDOW 就是不建控制台; startupinfo 再兜一层 SW_HIDE。
#   命令行方式下这两个参数不会改变任何可见行为, 所以直接全局用。
_SPAWN = {}
if os.name == "nt":
    _SPAWN["creationflags"] = 0x08000000        # CREATE_NO_WINDOW
    try:
        _si = subprocess.STARTUPINFO()
        _si.dwFlags |= subprocess.STARTF_USESHOWWINDOW
        _si.wShowWindow = subprocess.SW_HIDE
        _SPAWN["startupinfo"] = _si
    except Exception:
        pass

MAGIC       = 0xA55A
PIX_PER_PKT = 732          # 全屏模式: 732*2 + 8 = 1472 字节, 卡在 MTU 内

# ---- 停止请求 (v21, 给 GUI 用) ----
#   run_quad / run_full 都是 `while True` + `except KeyboardInterrupt` 收尾, 命令行
#   下按 Ctrl-C 就停了。但 GUI 是在子线程里跑这两个函数, 主线程没法给它抛
#   KeyboardInterrupt (硬抛要用 ctypes 去异步中断, 会把 socket 状态搞坏)。
#   所以加一个模块级标志, 每帧检查一次 —— 命令行永远是 False, 行为和原来逐位一致。
#   注意: 检查点放在"帧开头", 所以停止延迟最多一帧 (默认 24fps => 42ms)。
STOP        = False


def request_stop():
    """请求停止发送, 下一个帧边界生效 (可被 GUI 线程调用)"""
    global STOP
    STOP = True


def clear_stop():
    """开始新一轮发送前必须调用, 否则上一次的停止请求会立刻把新的一轮顶掉"""
    global STOP
    STOP = False


# ---- 当前显示模式 (v22 加, v23 扩成三态) ----
#   板子按键1 循环切换 三档, 板子把模式塞进遥测最低字节回传 (v24 起在 [4:3]):
#       0 = PC 全屏      整屏 1024x600 全是 PC 视频 (板子上电默认)
#       1 = 四宫格       左上 OV5640 + 其余三格 PC 视频
#       2 = 摄像头全屏   整屏 1024x600 全是摄像头画面, ★PC 不用发包★
#   GUI 要能把它显示出来, 所以这里留一个模块级字典: run_quad / run_full
#   每次确定模式就写一次, GUI 的定时器 200ms 读一次, 只读不写, 不加锁。
MODE_NAME = {0: "PC 全屏", 1: "四宫格", 2: "摄像头全屏"}
MODE = {"mode": 1, "full": False, "name": "四宫格", "known": False}


def set_mode(m):
    m = 2 if int(m) == 2 else (0 if int(m) == 0 else 1)
    MODE["mode"]  = m
    MODE["full"]  = (m == 0)
    MODE["name"]  = MODE_NAME[m]


def tl_mode(tl):
    """从信用包遥测里解出显示模式 0/1/2

    v24 起最低字节重新分配了, 所以先看"固件版本标记"再决定按哪套读:
      v24 固件: dbg[2:0] = 3'b101, dbg[4:3] = 模式, dbg[7:5] = 算法档位
      v23 及以前: dbg[5:4] = 模式, 同时 flags[7] = (mode==0)
    这样手上万一拿错了版本, 也能读对, 不会显示成一堆乱跳的模式。
    """
    d = tl["dbg"]
    if (d & 0x07) == 0x05:                 # v24
        m = (d >> 3) & 0x3
        return m if m <= 2 else 1
    f = tl["flags"]                        # 旧固件
    if f & 0x80:
        return 0
    m = (d >> 4) & 0x3
    if m == 0:
        return 1
    return m if m <= 2 else 1


def tl_algo(tl):
    """板子当前算法档位 0..4; 旧固件没有回传, 返回 -1"""
    d = tl["dbg"]
    if (d & 0x07) != 0x05:
        return -1
    a = (d >> 5) & 0x7
    return a if a <= 4 else -1

# ---- 画面算法档位 (v24) ----
#   按键 1 改成切显示模式了, 算法改由 PC 用一条 4 字节 UDP 命令控制:
#       A5 5B <档位> <~档位>      端口随便填 —— 板子这层根本不解析端口
#   命令可能丢 (UDP 无确认), 所以两头兜:
#     1) 每次下发连发 3~5 份;
#     2) 板子把"实际档位"塞进遥测回传, 这里对着看, 对不上就重发。
#   now = -1 表示板子还是旧固件(没有档位回传), 这时只能盲发, 按不上就说一声。
ALGO_NAME = ["彩色", "灰度", "去噪", "二值", "YCbCr"]
ALGO = {"want": 0, "now": -1}
_CMD_SOCK = None


# ---- 车道识别【显示】开关 (v50 加, v51 改成只管显示) ----
#   负载还是一样的 4 字节, 只是 magic 换成 0xA55C, 所以老固件见到它会当
#   未知包丢掉(不会误改档位), 新固件见到 0xA55B 也照旧处理 A55B。
#       A5 5C <1=显示 0=不显示> <~值>
#   ★v51: 这条命令只关 OSD 那一层, 不关识别★
#     板子内部的车道检测和变道判据照常逐帧跑, LED 上的报警灯/车态灯照常亮,
#     只是屏幕上不再画红点、色块和决策文字。所以它等于"隐藏结果", 不是"停机"。
#   板子上电默认"显示", 不接 PC 也和以前完全一样。
#   这条命令没有回读(遥测那个字节已经被档位/模式/版本号占满了), 所以:
#     - 点开关时连发 5 份
#     - 发送循环里每 1s 再无条件补 3 份 (幂等, 多收几次没有副作用)
#   这样最坏情况丢两轮包, 2 秒内也一定生效。
LANE = {"on": 1}


def set_lane(v):
    """只改"想要什么", 真正发由 push_lane 做"""
    LANE["on"] = 1 if v else 0


def push_lane(ip, port, times=3):
    global _CMD_SOCK
    try:
        if _CMD_SOCK is None:
            _CMD_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        v = LANE["on"] & 0xFF
        pkt = bytes((0xA5, 0x5C, v, (~v) & 0xFF))
        for _ in range(times):
            _CMD_SOCK.sendto(pkt, (ip, int(port)))
        return True
    except OSError:
        return False


def set_algo(m):
    """只改"想要什么", 真正发由 push_algo 做"""
    ALGO["want"] = int(m) & 0x7


def push_algo(ip, port, times=3):
    """把期望档位发给板子。丢一两个不要紧, 发送循环里还有每 0.5s 一次的重发。"""
    global _CMD_SOCK
    try:
        if _CMD_SOCK is None:
            _CMD_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        m = ALGO["want"]
        pkt = bytes((0xA5, 0x5B, m, (~m) & 0xFF))
        for _ in range(times):
            _CMD_SOCK.sendto(pkt, (ip, int(port)))
        return True
    except OSError:
        return False


# ---- --quad 模式的固定几何 ----
Q_W      = 512             # 每格宽
Q_H      = 300             # 每格高
Q_PIXPKT = 512             # 每包 512 像素 (1024 字节负载 + 8 头)
Q_NPKT   = 900             # 300 (上半区右半) + 600 (下半区整行)
Q_TOTAL  = Q_NPKT * Q_PIXPKT   # 460800 像素, 与 FPGA 端 PC_PIX_FRAME 一致


def build_base(W, H):
    """一次性生成静态渐变底色 (RGB565, 高字节在前)"""
    buf = bytearray(W * H * 2)
    i = 0
    for y in range(H):
        g = (y * 255) // H
        gp = ((g >> 2) & 0x3F) << 5
        for x in range(W):
            r = (x * 255) // W
            v = ((r >> 3) << 11) | gp | (128 >> 3)
            buf[i]     = (v >> 8) & 0xFF
            buf[i + 1] = v & 0xFF
            i += 2
    return bytes(buf)


def build_probe(W, H):
    """诊断图案: 一眼看出"哪一格、有没有平移/翻转/错位"

    - 整幅是渐变底 (能看出撕裂/条纹)
    - 四边留 3px 白框 (能看出左右/上下被裁)
    - 左上角一个实心白方块 + 一条 64px 白横线 (分辨左右)
    - 正中一个白十字 (分辨中心偏移)
    - 右下角一条 32px 白色短线 (分辨上下翻转)
    """
    buf = bytearray(build_base(W, H))
    W2 = W * 2

    def px(x, y, v=0xFFFF):
        if 0 <= x < W and 0 <= y < H:
            o = y * W2 + x * 2
            buf[o] = (v >> 8) & 0xFF
            buf[o + 1] = v & 0xFF

    # 白框
    for x in range(W):
        for t in range(3):
            px(x, t); px(x, H - 1 - t)
    for y in range(H):
        for t in range(3):
            px(t, y); px(W - 1 - t, y)
    # 左上角方块 + 横线
    for y in range(20, 52):
        for x in range(20, 52):
            px(x, y)
    for x in range(60, 60 + 64):
        for y in range(32, 40):
            px(x, y)
    # 正中十字
    for x in range(W // 2 - 24, W // 2 + 24):
        for y in range(H // 2 - 2, H // 2 + 2):
            px(x, y)
    for y in range(H // 2 - 24, H // 2 + 24):
        for x in range(W // 2 - 2, W // 2 + 2):
            px(x, y)
    # 右下角短线
    for x in range(W - 60, W - 20):
        for y in range(H - 40, H - 34):
            px(x, y)
    return bytes(buf)


def quad_dst(seq):
    """包序号 -> 目标块号 (板子用块号*64 直接算 DDR 地址)

    块 = 屏幕的半个行 = 512 像素。四宫格里 PC 负责:
      seq   0..299  : 第 seq 行, 右半   (dst = row*2 + 1)
      seq 300..899  : 下半屏整行, 每行两包 (左半/右半)
    板子不需要知道这个排版, 它只看包里带的 dst。
    """
    if seq < Q_H:
        row  = seq
        half = 1
    else:
        j    = seq - Q_H
        row  = Q_H + j // 2
        half = j % 2
    return (row * 2 + half) & 0xFFFF


def build_quad_layout(src, sw, sh):
    """把 sw x sh (必须 512x300) 的源帧展开成 --quad 需要的 900 包数据

    包 0..299  : 源图对应行
    包 300..899: 源图对应行再发一遍 (每行连续两包 = 水平方向翻倍)
    """
    row_b = sw * 2
    out   = bytearray(Q_NPKT * row_b)
    for i in range(sh):                                   # 上半区右半
        out[i * row_b:(i + 1) * row_b] = src[i * row_b:(i + 1) * row_b]
    base = sh * row_b
    for j in range(sh * 2):                               # 下半区整行
        r = j // 2
        out[base + j * row_b: base + (j + 1) * row_b] = src[r * row_b:(r + 1) * row_b]
    return bytes(out)


def wait_until(t):
    """精确等到 perf_counter() >= t

    Windows 的 time.sleep 粒度约 15.6ms, 直接 sleep 会让帧率严重偏低,
    所以最后 2ms 改成忙等。
    """
    while True:
        d = t - time.perf_counter()
        if d <= 0:
            return
        if d > 0.003:
            time.sleep(d - 0.002)


def udp_send(sock, buf):
    """connect 之后的 UDP 发送, 顺手吞掉 Windows 的 ICMP 假错误

    connected UDP socket 有个坑: 如果板子回过 ICMP port-unreachable
    (比如 bit 还没烧、IP 填错、板子刚复位), Windows 会把这个错误挂到
    *下一次* send 上抛 10054/10061。不处理的话发几包就整个脚本崩掉,
    看起来像是"发到一半断了"。这里直接吞掉, 真断了流控超时会另外报。
    """
    try:
        sock.send(buf)
    except (ConnectionResetError, ConnectionRefusedError):
        pass


def send_frame(sock, data, fid, npkt, total, gap=0.0):
    """全屏模式: 按 732 像素/包 切开发

    socket 已在 main() 里 connect 过, 这里用 send() 而不是 sendto():
    sendto 每包都要重查一次路由表, 实测 57.9us/包; connect 之后 send
    只要 8.8us/包 —— 差 6.6 倍, 而 22fps 全屏一帧 840 包, 光发就要 48ms。

    gap: 包间隔(秒)。0 = 连发。和 --quad 一样, 连发会被板子 MAC 的突发
    缓冲整包丢掉(实测约 19%), 所以正常都要传一个 >= GAP_MIN_US 的值。
    """
    off = 0
    t_send0 = time.perf_counter()
    for seq in range(npkt):
        if gap > 0.0:
            tgt = t_send0 + seq * gap
            while time.perf_counter() < tgt:
                pass
        n   = min(PIX_PER_PKT, total - off // 2)
        hdr = struct.pack(">HHHH", MAGIC, fid & 0xFFFF, seq & 0xFFFF, n)
        udp_send(sock, hdr + data[off:off + n * 2])
        off += n * 2


# ==================================================================
# --quad : 闭环流控 (credit) 发送
# ==================================================================
# 板上每消费 1024 个 PC 像素, 就往 PC 回一个 20 字节的广播 UDP 包:
#     'C' 'R' 0x01 0x00 | counter[31:0] (大端, 累计已消费像素)
# PC 端只在  (已发送 - 已消费) + 512 <= WINDOW  时才发下一个包。
# 这样板上 pc_fifo 永远不会被灌满 —— 摄像头卡顿、写合成器等数据、
# 甚至摄像头掉线, 都不会再出现"丢像素 -> 地址错位 -> 画面重复/绿块乱跑"。
CR_PORT   = 1234
# v10: 流控改成"看板子自己的积压水位", 不再用"已发 - 已落盘"的累计差
#   为什么: 累计差一旦被一包坏数据带偏(或者网络真丢了一包), 那个差额会
#   永远留在账上, PC 就永久卡死在上板那一秒。水位是"当前还剩多少", 板子
#   一排水位就掉, 自带纠错。
CR_HIGH   = 4096          # 板子 pc_fifo 积压超过这个字数就等 (深度 8192)
CR_MAX_STEP = 1 << 19     # 一个回包之间板子最多能多消费这么多像素, 超过就是坏包

# 板子实测能吃的帧率上限
#   2026-11-01 用 bench_speed.py loss 精确测(把"发多少包"和板子"收多少包 /
#   落多少像素"三段对齐), 结论和以前猜的完全不一样:
#
#     目标帧率  包间隔    板子收包    落盘/应得   PC FIFO 水位
#      24fps    连发(0)     81.2%      81.2%        7500  (快满)
#      13.7fps  连发(0)     81.4%      81.4%        7500  (快满)
#       8fps    连发(0)     80.7%      80.7%        7300  (快满)
#      24fps     40us      100.0%     100.0%         520  (空)
#      30fps     30us      100.0%     100.0%         530  (空)
#
#   ★关键★ 丢包率和帧率无关, 8fps 也丢 19% —— 所以根本不是带宽/算力不够,
#   而是【一帧 900 包连发】把板子的接收通路打爆了: MAC 侧突发缓冲一满就
#   整包丢, 丢掉的包 udp_video_rx 根本不计数, FIFO 反而被灌到满。
#   把 900 包按整帧周期均匀铺开(每包 >= 40us)之后, 24fps/30fps 都是 0 丢包。
#
# BOARD_FPS_DEFAULT 是帧级安全阀: 万一收不到遥测(另开 credit_monitor 抢走
# 了 1234 端口 / 防火墙拦了), 流控会整体失效。以前这时 period=0 -> 全速
# 68fps 狂发, 板子吃不下全丢, 屏上"隔几帧取一帧"跳着播 —— 表现就是
# 【1 分钟的视频 20 秒就播完】。现在无论流控在不在, 帧级都限死。
BOARD_FPS_DEFAULT = 24.0

# 包与包之间的最小间隔。实测 < 40us 板子开始整包丢(见上面表格)。
# 这个值同时决定了帧率天花板: 1e6 / (40 * 900) = 27.8 fps。
GAP_MIN_US = 40.0


def frame_period(args):
    """每帧最短间隔(秒)。--fps 优先, 否则用 --max-fps 兜底。

    永远不要返回 0: 那就是"不限速", 一旦流控失效就会把板子灌爆。
    """
    f = args.fps if args.fps > 0 else getattr(args, "max_fps", BOARD_FPS_DEFAULT)
    return (1.0 / f) if (f and f > 0) else (1.0 / BOARD_FPS_DEFAULT)



def open_credit_sock():
    """开一个非阻塞 UDP 口收板子的回包 (1234)"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        s.bind(("", CR_PORT))
    except OSError as e:
        print("[警告] 无法监听 UDP %d 接收流控包: %s" % (CR_PORT, e))
        print("       可能被别的进程占用 -> 本次只能按固定节奏发(更容易花屏)")
        s.close()
        return None
    s.setblocking(False)
    return s


class Credit(object):
    """解析板子回传的 39 字节包

    里面有两样东西:
      'C','R' + counter 连发 3 遍   -> 板子已落盘像素的绝对计数
      'T','L' + 16 字节             -> 遥测(PC/摄像头落盘、两个 FIFO 水位、收包数)
    """

    def __init__(self):
        self.raw  = None       # 上一次采信的绝对计数
        self.ack  = 0          # 单调累计(用模 2^32 增量处理绕回)
        self.got  = 0          # 收到多少个回包
        self.bad  = 0          # 判定"不可能"而丢弃的坏包
        self.realign = 0       # 重新对齐次数(板子复位/严重丢包)
        self.back = 0
        self.last_rx = 0.0
        self.last_change = 0.0     # ack 最后一次真正增长的时刻
        self.pkt_hex = ""
        self.tl      = None
        self.reset_evt = False

    # ---------------- 遥测 ----------------
    def _parse_tl(self, pkt):
        """在包里找 'T''L', 后面 16 字节就是遥测 (v10 布局):
           pc_px(4) cam_px(4) pc_lvl(2) cam_lvl(2) pkts(2) flags(1) dbg(1)

        ★为什么不按固定偏移★ 板子的 UDP 发送流水线会把包头吃掉 0~N 个字节,
        整包内容是"右对齐"的, 偏移每次都可能不同。所以只能靠标记定位。
        而且计数器的随机字节里也可能碰巧出现 'T''L', 所以要把所有候选都
        试一遍, 只接受"水位不超过 FIFO 深度"的那个。
        """
        self.tl = None
        start = 0
        while True:
            k = pkt.find(b"TL", start)
            if k < 0:
                return
            b = pkt[k + 2:k + 18]
            if len(b) == 16:
                pc_lvl = struct.unpack(">H", b[8:10])[0]
                cam_lvl = struct.unpack(">H", b[10:12])[0]
                if pc_lvl <= 8192 and cam_lvl <= 4096:
                    self.tl = {
                        "pc_px":   struct.unpack(">I", b[0:4])[0],
                        "cam_px":  struct.unpack(">I", b[4:8])[0],
                        "pc_lvl":  pc_lvl,
                        "cam_lvl": cam_lvl,
                        "pkts":    struct.unpack(">H", b[12:14])[0],
                        "flags":   b[14],
                        "dbg":     b[15],
                    }
                    return
            start = k + 1

    # ---------------- 计数 ----------------
    def _counter(self, pkt):
        """板子把 'C','R' + counter 连发 3 遍, 取出现次数最多且一致的那个值

        同样不按固定偏移: 包头可能被吃掉几个字节。这里把所有 'CR' 位置都
        取出来, 值出现 >=2 次的才认。
        """
        cand = {}
        start = 0
        while True:
            k = pkt.find(b"CR", start)
            if k < 0:
                break
            if k + 6 <= len(pkt):
                v = struct.unpack(">I", pkt[k + 2:k + 6])[0]
                cand[v] = cand.get(v, 0) + 1
            start = k + 1
        if not cand:
            return None
        v, n = max(cand.items(), key=lambda kv: kv[1])
        return v if n >= 2 else None

    def poll(self, cs):
        if cs is None:
            return
        now = time.perf_counter()
        while True:
            try:
                pkt, _ = cs.recvfrom(256)
            except (BlockingIOError, InterruptedError):
                return
            except OSError:
                return

            self._parse_tl(pkt)
            v = self._counter(pkt)
            if v is None:
                continue

            self.got += 1
            self.pkt_hex = pkt.hex()
            self.last_rx = now

            if self.raw is None:                 # 第一包: 直接对齐(板子可能已经跑了很久)
                self.raw = v
                self.ack = v
                self.last_change = now
                continue
            if v == self.raw:
                continue

            d = (v - self.raw) & 0xFFFFFFFF      # 模 2^32 增量(绕回也是小正数)
            if d < 0x80000000:
                if d > CR_MAX_STEP:              # 一步涨这么多不可能 -> 坏包, 丢弃
                    self.bad += 1
                    continue
                self.raw = v
                self.ack += d
                self.back = 0
                self.last_change = now
            else:                                # 倒退: 坏包 or 板子复位
                self.back += 1
                if self.back >= 5:
                    # 连续 5 次倒退 = 板子复位了。这里只重新对齐"显示用"的
                    # 计数, 绝对不去改发送节奏(流控只看板子报的水位)
                    self.raw = v
                    self.ack = v
                    self.back = 0
                    self.realign += 1

    # ---------------- 流控 ----------------
    def has_room(self, high, npx, force=False):
        """板子 FIFO 还装得下吗? 没有遥测时一律放行"""
        if self.tl is None:
            return True
        if force:
            return True
        return self.tl["pc_lvl"] + npx <= high

def tl_text(cr):
    """把遥测排成一行"""
    t = cr.tl
    if not t:
        return ""
    f = t["flags"]
    g = t["dbg"]
    return ("PC落盘 %-9d 水位 %-5d | 摄像头落盘 %-9d 水位 %-5d | 板子收包 %-5d"
            " | DDR%s 忙%s 写%s | pc%s cam%s | %s %s %s %s"
            % (t["pc_px"], t["pc_lvl"],
               t["cam_px"], t["cam_lvl"],
               t["pkts"],
               "OK" if f & 0x02 else "--",
               "1" if f & 0x20 else "0",
               "1" if f & 0x10 else "0",
               "空" if f & 0x08 else "有",
               "空" if f & 0x04 else "有",
               "href" if g & 0x08 else "-",
               "valid" if g & 0x04 else "-",
               "emit" if g & 0x02 else "-",
               "tag" if g & 0x01 else "-"))


def tl_diag(cr, dt_pc=0, dt_cam=0):
    """根据遥测给出根因判断, dt_* = 最近 1 秒的增量"""
    t = cr.tl
    if not t:
        return "收不到遥测(板子还是旧 bit?)"
    f = t["flags"]
    if not (f & 0x02):
        return "DDR3 初始化未完成 -> 写通路根本没启动"
    if t["pc_lvl"] > 6000:
        return "板子 PC 队列快满了但没在排空 -> 写侧卡住(看 忙/写 标志)"
    if dt_pc == 0:
        return "PC 一个像素都没落盘 -> 板子没收到整包(对比'板子收包'与'已发包数')"
    if dt_cam == 0:
        return "摄像头一个像素都没落盘 -> 查排线/cam_pclk/I2C 初始化"
    return "PC 每秒 %d px, 摄像头每秒 %d px" % (dt_pc, dt_cam)

# ==================================================================
# v15: 显示模式 (四宫格 / 全屏) 与统一的帧源
# ==================================================================
# 板子上按键1 循环三态, 通过信用包遥测回传。PC 在这里跟进:
#   四宫格: 源 512x300,  900 包/帧 (块号 1,3,5..599 + 600..1199)
#   全屏  : 源 1024x600, 1200 包/帧 (块号 0..1199 顺序)
# 两种模式的包头格式完全一样 (MAGIC, dst, seq, 512), 板子端不用区分,
# 差别只在"发几个包"和"发什么内容"。所以切换只是换源 + 换 dst 表。
def mode_geom(full):
    """按模式返回 (w, h, npkt, dsts, src_off, total_px)"""
    rb_pkt = Q_PIXPKT * 2          # 每包 512 像素 = 1024 字节
    if full:
        w, h = 1024, 600
        npkt = (w * h) // Q_PIXPKT                  # 1200
        dsts = list(range(npkt))                    # 块号 = 半行号, 顺序
        src_off = [s * rb_pkt for s in range(npkt)]
        return w, h, npkt, dsts, src_off, w * h
    w, h = Q_W, Q_H
    npkt = Q_NPKT                                   # 900
    dsts = [quad_dst(i) for i in range(npkt)]
    src_off = [(s if s < Q_H else (s - Q_H) // 2) * rb_pkt for s in range(npkt)]
    return w, h, npkt, dsts, src_off, w * h


# 裸 RGB565 文件的原始分辨率。
# make_video.py 产出的都是 512x300, 所以默认就是它; 真放了 1024x600 的裸
# 文件就用 --raw-size 1024x600 明说。
RAW_DEFAULT = (512, 300)


def parse_raw_size(s):
    for sep in ("x", "X", "*", ","):
        if sep in s:
            a, b = s.split(sep, 1)
            try:
                return int(a), int(b)
            except ValueError:
                break
    raise SystemExit("--raw-size 要写成 512x300 这种形式, 收到的是 %r" % s)


def detect_raw_geom(fsize, hint):
    """猜裸文件的原始帧尺寸, 返回 (w, h, nframes), 猜不出返回 None

    ★这里必须猜对★ 裸文件没有文件头, 一旦按错的尺寸去读, 每一次 read() 拿到的
      其实是好几张图摞在一起 —— 屏幕上就是"同一幅画面被贴了 4 遍"(全屏模式拿
      512x300 的素材去当 1024x600 用, 正是这个现象), 比直接报错难查得多。
      所以宁可多算几步、多问一句。
    """
    cands = [hint, (512, 300), (1024, 600), (480, 272), (800, 480), (640, 480)]
    seen = set()
    for (cw, ch) in cands:
        if (cw, ch) in seen:
            continue
        seen.add((cw, ch))
        fb = cw * ch * 2
        if fb > 0 and fsize % fb == 0:
            return cw, ch, fsize // fb
    return None


class FrameSource(object):
    """三种帧源统一成 read(fid) -> bytes(w*h*2)

    1) --mp4    : ffmpeg 实时解码 + 缩放到目标尺寸, 播完自动重头
                  (不落盘成 .rgb565: 1024x600 一帧 1.2MB, 一分钟就 1.4GB)
    2) --video  : 裸 RGB565 大端文件。原始尺寸==目标尺寸时走 mmap 零拷贝;
                  原始尺寸不同(比如四宫格 512x300 的素材要上全屏 1024x600)
                  则自动改走 ffmpeg 缩放, ★不能直接按目标尺寸 read★
    3) 都没有   : 内置测试图案 (底色/定位图), 带 --bar 时还会跑竖条
    """

    def __init__(self, args, w, h):
        self.w, self.h = w, h
        self.fb = w * h * 2
        self.mm = None
        self.fh = None
        self.proc = None
        self.pipe_kind = None       # "mp4" / "raw", 决定 EOF 后怎么重开
        self.fix = None
        self.nframes = 0
        self.nat_w = self.nat_h = 0  # 裸文件的原始尺寸
        self.args = args
        self.bar_b = b'\xff\xff' * args.bar if args.bar else b''
        if args.mp4:
            if not os.path.isfile(args.mp4):
                raise SystemExit("找不到视频文件: %s" % args.mp4)
            self.pipe_kind = "mp4"
            self._open_mp4_pipe()
        elif args.video:
            if not os.path.isfile(args.video):
                raise SystemExit("找不到视频文件: %s" % args.video)
            fsize = os.path.getsize(args.video)
            g = detect_raw_geom(fsize, parse_raw_size(
                getattr(args, "raw_size", "512x300") or "512x300"))
            if g is None:
                raise SystemExit(
                    "裸文件 %s 大小 %d 字节, 除不尽任何已知帧尺寸。\n"
                    "  512x300  每帧 %d 字节\n"
                    "  1024x600 每帧 %d 字节\n"
                    "  用 --raw-size WxH 明确告诉脚本它的真实分辨率。"
                    % (args.video, fsize, 512 * 300 * 2, 1024 * 600 * 2))
            self.nat_w, self.nat_h, nat_n = g
            self.nframes = min(nat_n, args.frames) if args.frames else nat_n
            if (self.nat_w, self.nat_h) == (w, h):
                # 尺寸正好对上: 走 mmap 零拷贝, 最快
                self.fh = open(args.video, "rb")
                self.mm = mmap.mmap(self.fh.fileno(), 0, access=mmap.ACCESS_READ)
            else:
                # ★尺寸对不上必须交给 ffmpeg 缩放★
                #   硬按目标尺寸 read(fb) 会把好几张图当成一帧, 屏幕变"照片墙"。
                self.pipe_kind = "raw"
                self._open_raw_pipe()
        elif args.probe:
            self.fix = build_probe(w, h)
        else:
            self.fix = build_base(w, h)


    def _kill_pipe(self):
        if self.proc is not None:
            try:
                self.proc.kill()
                self.proc.stdout.close()
            except Exception:
                pass
            self.proc = None

    def _open_mp4_pipe(self):
        self._kill_pipe()
        self.proc = subprocess.Popen(
            ["ffmpeg", "-v", "error", "-i", self.args.mp4,
             "-vf", "scale=%d:%d" % (self.w, self.h),
             "-pix_fmt", "rgb565be", "-f", "rawvideo", "-"],
            stdout=subprocess.PIPE, bufsize=self.fb * 4, **_SPAWN)

    def _open_raw_pipe(self):
        """裸 RGB565 文件 -> ffmpeg 缩放 -> 目标尺寸

        素材是 512x300 而目标要 1024x600 时必须走这条, 不能直接 mmap 切片。
        """
        self._kill_pipe()
        self.proc = subprocess.Popen(
            ["ffmpeg", "-v", "error",
             "-f", "rawvideo", "-pixel_format", "rgb565be",
             "-video_size", "%dx%d" % (self.nat_w, self.nat_h),
             "-i", self.args.video,
             "-vf", "scale=%d:%d" % (self.w, self.h),
             "-pix_fmt", "rgb565be", "-f", "rawvideo", "-"],
            stdout=subprocess.PIPE, bufsize=self.fb * 4, **_SPAWN)

    def _reopen(self):
        if self.pipe_kind == "raw":
            self._open_raw_pipe()
        else:
            self._open_mp4_pipe()

    def read(self, fid):
        if self.mm is not None:
            idx = fid % self.nframes
            return self.mm[idx * self.fb:(idx + 1) * self.fb]
        if self.proc is not None:
            b = self.proc.stdout.read(self.fb)
            if len(b) < self.fb:            # 播放到结尾了, 重头再来
                self._reopen()
                b = self.proc.stdout.read(self.fb)
            return b if len(b) == self.fb else b'\x00' * self.fb
        buf = bytearray(self.fix)
        if self.args.bar:
            bx = (fid * 8) % self.w
            if bx + self.args.bar > self.w:
                bx = self.w - self.args.bar
            for y in range(self.h):
                o = (y * self.w + bx) * 2
                buf[o:o + self.args.bar * 2] = self.bar_b
        return bytes(buf)

    def desc(self):
        """一行说明"源是什么 -> 送到多大", 打日志用"""
        if self.mm is not None:
            return "裸文件 %dx%d, %d 帧, mmap 零拷贝" % (
                self.nat_w, self.nat_h, self.nframes)
        if self.pipe_kind == "raw":
            return "裸文件 %dx%d -> ffmpeg 缩放 %dx%d" % (
                self.nat_w, self.nat_h, self.w, self.h)
        if self.pipe_kind == "mp4":
            return "mp4 实时解码 -> %dx%d" % (self.w, self.h)
        return "内置图案 %dx%d" % (self.w, self.h)

    def close(self):
        if self.mm is not None:
            self.mm.close()
            self.mm = None
        if self.fh is not None:
            try:
                self.fh.close()
            except Exception:
                pass
            self.fh = None
        self._kill_pipe()


def run_quad(sock, dst, args):
    full = bool(args.full)                 # 命令行给的初始模式, 之后跟板子走
    cur_mode = 0 if full else 1            # v23: 0=PC全屏 1=四宫格 2=摄像头全屏
    src_ready = True                       # 摄像头全屏时把帧源关掉
    t_pause  = time.perf_counter()          # 进"摄像头全屏"的时刻, 只为打印
    MODE["known"] = False
    set_mode(cur_mode)
    sw, sh, npkt, dsts, src_off, total_px = mode_geom(full)
    src_obj = FrameSource(args, sw, sh)

    if args.mp4:
        print("视频: %s   %s  (%s)"
              % (args.mp4, src_obj.desc(), MODE_NAME[cur_mode]))
    elif args.video:
        print("视频: %s   %d 帧   %s  (%s)"
              % (args.video, src_obj.nframes, src_obj.desc(),
                 MODE_NAME[cur_mode]))
        if src_obj.pipe_kind == "raw":
            print("   ★素材 %dx%d 和显示模式 %dx%d 不一致, 已自动缩放★"
                  % (src_obj.nat_w, src_obj.nat_h, sw, sh))
    elif args.probe:
        print("生成定位图案 %dx%d (白框 + 左上角块 + 正中十字) ..." % (sw, sh))
    else:
        print("生成测试底色 %dx%d ..." % (sw, sh))

    print("-> %s:%d   %d 包/帧 x %d 像素 = %d 像素/帧   [%s]"
          % (args.ip, args.port, npkt, Q_PIXPKT, total_px,
             MODE_NAME[cur_mode]))
    if not full:
        print("   左上 512x300 由 OV5640 填, PC 只负责其余 3 格")
    print("   模式跟随板子按键1 (三态: PC 全屏 / 四宫格 / 摄像头全屏), 运行中自动切换")

    cs = None if args.no_flow else open_credit_sock()
    cr = Credit()
    if cs is not None:
        print("   流控: 监听 UDP %d 收板子的信用包, 板子积压超过 %d 字就等"
              % (CR_PORT, CR_HIGH))
    else:
        print("   流控: 关闭 (--no-flow), 按 %.0f us/行 的固定节奏发" % args.row_us)
    print("   帧率上限: %.1f fps   (板子实测量, 超了会丢包跳帧; --max-fps 可调)"
          % (1.0 / frame_period(args)))

    rb    = Q_PIXPKT * 2
    fid   = 0
    nf    = 0
    sent  = 0
    t0    = time.perf_counter()
    tnext = time.perf_counter()
    period = frame_period(args)          # ★永不为 0, 见 frame_period 注释
    # 每包间隔: 整帧铺开, 但不低于实测安全值 40us。
    # 留 2ms 给"取源帧 + 逐包组包"的固定开销, 否则 900 包正好排满整帧,
    # 开销一挤每帧就多出 0.7ms, 24fps 会掉成 23.6。
    def calc_gap(per, n):
        return max((per - 0.002) / n, GAP_MIN_US * 1e-6)

    pkt_gap = calc_gap(period, npkt)
    # dsts / src_off 由 mode_geom() 按模式生成:
    #   四宫格 seq 0..299  -> 右上格第 seq 行;  300..899 -> 下半区, 行号取半
    #   全屏   seq 0..1199 -> 块号顺序, 每包半行, 源帧偏移连续
    if period / npkt < GAP_MIN_US * 1e-6:
        print("   注意: %.0f fps 要求 %.0f us/包, 低于板子安全值 %.0f us, "
              "已夹到 %.0f us -> 实际最高 %.1f fps"
              % (1.0 / period, period / npkt * 1e6, GAP_MIN_US,
                 GAP_MIN_US, 1e6 / (GAP_MIN_US * npkt)))
    else:
        print("   发包节奏: 每包间隔 %.0f us (整帧铺开, 避免突发丢包)"
              % (pkt_gap * 1e6))
    fallback_warned = False
    stall_warned    = False
    flow_off        = False     # 收不到信用包时临时关掉窗口等待
    got_at_off      = 0
    last_stat       = time.perf_counter()
    last_force      = 0.0
    last_algo       = 0.0      # v24: 上次下发算法档位的时刻
    last_lane       = 0.0      # v50: 上次重发车道线开关的时刻
    prev_pc = prev_cam = prev_ack = 0
    prev_sp = prev_pk = None
    stat_dpc = stat_dc = 0

    try:
        while True:
            if STOP:                 # GUI 点了"停止" -> 在帧边界干净退出
                break
            # ---------- 显示模式跟随板子按键1 (v23: 三态) ----------
            #   0 = PC 全屏 -> 发 1200 包
            #   1 = 四宫格  -> 发 900 包
            #   2 = 摄像头全屏 -> ★一包都不发★ 只保持收信用包, 等板子切回来
            #   切换时要重开帧源(尺寸不同), 会有一次几百毫秒的停顿。
            if cr.tl is not None:
                MODE["known"] = True
                ALGO["now"] = tl_algo(cr.tl)     # v24: 板子回报的实际档位
                m = tl_mode(cr.tl)
                if m != cur_mode:
                    cur_mode = m
                    set_mode(m)
                    fid = 0
                    print("")
                    print("=" * 62)
                    if m == 2:
                        # 摄像头独占整屏, PC 这时候发什么都是在抢块号。
                        # 把 ffmpeg 关掉省 CPU, 只留轮询等切回来。
                        src_obj.close()
                        src_ready = False
                        t_pause = time.perf_counter()
                        print("[模式] 板子切到 摄像头全屏 -> PC 暂停发送")
                        print("       摄像头 1024x600 原生画面铺满整屏, 不缩放")
                        print("       再按一次按键1 回到 PC 全屏, 这里会自动接着发")
                    else:
                        full = (m == 0)
                        sw, sh, npkt, dsts, src_off, total_px = mode_geom(full)
                        src_obj = FrameSource(args, sw, sh)
                        src_ready = True
                        pkt_gap = calc_gap(period, npkt)
                        print("[模式] 板子切到 %s -> 源 %dx%d, %d 包/帧, %.1f fps 上限"
                              % (MODE_NAME[m], sw, sh, npkt,
                                 1e6 / (GAP_MIN_US * npkt)))
                        print("       帧源: %s" % src_obj.desc())
                        if src_obj.pipe_kind == "raw":
                            print("       ★裸素材 %dx%d 与目标 %dx%d 不一致, 已自动缩放★"
                                  % (src_obj.nat_w, src_obj.nat_h, sw, sh))
                    print("=" * 62)

            # ---------- 算法档位: 对不上就重发 (v24) ----------
            #   板子会把实际档位回传, 所以"丢了"能自己纠回来, 不用人去点第二遍。
            #   0.5s 一次是有意的: 太密会和视频包抢发送队列, 太稀现场会嫌"点了没反应"。
            now = time.perf_counter()
            if now - last_algo >= 0.5:
                last_algo = now
                if ALGO["now"] != ALGO["want"]:
                    push_algo(args.ip, args.port, times=3)

            # ---------- 车道线识别开关: 无条件定期补发 (v50) ----------
            #   没有回读, 所以不能"对不上才发"。好在命令幂等: 板子收到 1
            #   就是开, 收到 0 就是关, 重复收到同一个值等于什么都没发生。
            #   1s 一次 x 3 份, 带宽上完全是噪声级(12 字节/秒)。
            if now - last_lane >= 1.0:
                last_lane = now
                push_lane(args.ip, args.port, times=3)

            # ---------- 摄像头全屏: 这一档 PC 不出图 ----------
            if cur_mode == 2:
                if cs is not None:
                    cr.poll(cs)
                now = time.perf_counter()
                if now - last_stat >= 1.0:
                    last_stat = now
                    print("   [摄像头全屏] 已等 %0.0fs, 等板子按键1 切回"
                          % (now - t_pause))
                time.sleep(0.02)
                continue

            # ---------- 取源帧 ----------
            src = src_obj.read(fid)

            # 不再先把整帧展开成 900KB 中间缓冲(每帧一次分配+900 次切片),
            # 直接用 src_off 表按包取行: 省掉 0.9MB 分配和一次全量拷贝,
            # 帧率能多出 1 fps 左右, 也少惹 GC。
            # 不用按 seq 顺序猜行列了: 每个包自己带目标块号(写进包头 dst)
            # 这一帧的发送基准时刻对齐到节拍 tnext(而不是"现在"), 否则每帧
            # 取源图花掉的时间会一帧一帧攒成漂移, 24fps 会慢慢掉成 22.7。
            t_frame0 = tnext if tnext > 0 else time.perf_counter()
            if t_frame0 < time.perf_counter():        # 已经落后了就重锚
                t_frame0 = time.perf_counter()

            # ---------- 发一帧 ----------
            for seq in range(npkt):
                if cs is not None and not flow_off:
                    # ★v10 流控★ 看板子自己还剩多少空间, 不看累计差
                    while True:
                        cr.poll(cs)
                        now = time.perf_counter()
                        # 板子 2 秒一点没动 -> 每 200ms 硬塞一包探路(板子装不下
                        # 会整包拒收, 不会有半包, 所以这一塞是安全的)
                        force = ((now - cr.last_change) > 2.0 and
                                 (now - last_force) > 0.2)
                        if force:
                            last_force = now
                        if cr.has_room(CR_HIGH, Q_PIXPKT, force=force):
                            break

                        # 每秒一行进度: 一眼看出是"在等"还是"死等"
                        if now - last_stat >= 1.0:
                            last_stat = now
                            print("[等待] 已发 %-8d 板子已落盘 %-8d 积压 %-5d "
                                  "回包 %d 个(坏包 %d)"
                                  % (sent, cr.ack,
                                     cr.tl["pc_lvl"] if cr.tl else -1,
                                     cr.got, cr.bad))
                            if cr.tl:
                                t = cr.tl
                                stat_dpc = (t["pc_px"] - prev_pc) & 0xFFFFFFFF
                                stat_dc  = (t["cam_px"] - prev_cam) & 0xFFFFFFFF
                                if stat_dpc > 0x80000000: stat_dpc -= 0x100000000
                                if stat_dc  > 0x80000000: stat_dc  -= 0x100000000
                                print("       " + tl_text(cr))
                                print("       本秒增量: PC +%d px | 摄像头 +%d px"
                                      % (stat_dpc, stat_dc))
                                prev_pc, prev_cam = t["pc_px"], t["cam_px"]
                                prev_ack = cr.ack
                            elif cr.pkt_hex:
                                print("       最近回包: %s" % cr.pkt_hex)

                        # (a) 一个回包都没有 -> 链路/防火墙
                        if cr.got == 0 and (now - t0) > 5.0:
                            if not fallback_warned:
                                fallback_warned = True
                                print("")
                                print("=" * 62)
                                print("[错误] 5 秒内一个流控回包都没收到, 停止发送。")
                                print("  1) 用管理员权限运行  .\\fw_allow_python.bat")
                                print("  2) 网络位置改成'专用'")
                                print("  3) 板子烧的是含流控的新 bit")
                                print("=" * 62)
                                print("")
                            raise SystemExit(2)

                        # (b) 有回包但一个 PC 像素都没消费 -> 写合成器卡住了
                        if (cr.got > 0 and (now - cr.last_change) > 3.0
                                and not stall_warned):
                            stall_warned = True
                            print("")
                            print("-" * 62)
                            print("[注意] 板子在回心跳, 但 3 秒内一个 PC 像素都没消费。")
                            print("       根因判断: " + tl_diag(cr, stat_dpc, stat_dc))
                            print("       已发包 %d / 板子收包 %d (低16位), 差值 = 路上丢的"
                                  % ((sent // Q_PIXPKT) & 0xFFFF,
                                     cr.tl["pkts"] if cr.tl else 0))
                            print("       脚本会继续等, 板子一恢复就自动往下走。")
                            print("-" * 62)
                            print("")
                        if (now - cr.last_change) < 1.0:
                            stall_warned = False        # 又动了, 允许下次再报
                elif args.row_us > 0:
                    # 无流控时的保守节奏: 每行一批
                    if full:
                        wait_until(tnext + (seq + 1) * args.row_us / 1e6)
                    elif seq < Q_H:
                        wait_until(tnext + (seq + 1) * args.row_us / 1e6)
                    else:
                        k = seq - Q_H
                        wait_until(tnext + (Q_H + k // 2 + 1) * args.row_us / 1e6)

                # ★节奏★ 900 个包必须均匀铺满整帧, 不能一口气打完
                #   实测: 连发时约 19% 的包被板子 MAC 的突发缓冲整包丢掉,
                #   而且丢包率与帧率完全无关(8fps 也丢 19%), 所以这不是
                #   带宽/算力问题, 纯是突发打爆了接收通路。铺开后 0 丢包。
                t_pkt = t_frame0 + seq * pkt_gap
                now2  = time.perf_counter()
                if t_pkt > now2:
                    wait_until(t_pkt)
                elif now2 - t_pkt > pkt_gap:
                    # 被流控或编码拖后一个包以上, 重新锚定, 免得追着补发变成突发
                    t_frame0 = now2 - seq * pkt_gap

                udp_send(sock, struct.pack(">HH", MAGIC, dsts[seq]) +
                               struct.pack(">HH", seq, Q_PIXPKT) +
                               src[src_off[seq]:src_off[seq] + rb])
                sent += Q_PIXPKT

            # 即使退回保守限速也继续收包, 一旦开始收到就自动恢复流控
            if cs is not None:
                cr.poll(cs)
                if flow_off and (cr.got - got_at_off) >= 10:
                    flow_off = False
                    period = frame_period(args)          # ★永不为 0, 见 frame_period 注释
                    tnext = time.perf_counter()
                    print("[恢复] 已收到板子的流控包, 切回闭环流控")

            fid = (fid + 1) & 0xFFFF
            nf += 1

            if period > 0.0:
                tnext += period
                if time.perf_counter() > tnext:      # 已经超时, 重锚
                    tnext = time.perf_counter()
                else:
                    wait_until(tnext)
            else:
                tnext = time.perf_counter()

            if nf % 20 == 0:
                dt = time.perf_counter() - t0
                if cs is not None and not flow_off:
                    extra = "在途 %-7d 信用 %d 包 已落盘 %d px" % (
                        max(0, sent - cr.ack), cr.got, cr.ack)
                elif cs is not None:
                    extra = "无流控(保守限速) 信用 %d 包" % cr.got
                else:
                    extra = "无流控(保守限速)"
                print("frame %5d   %5.1f fps   %6.1f MB/s   %s"
                      % (fid, nf / dt, (nf * total_px * 2 / 1e6) / dt, extra))
                # 每 20 帧把板子的遥测打一行: 一眼看到"发了多少 / 收了多少 / 积压多少"
                # 注意: 这里必须比【本周期增量】。两个计数都只有 16 位, 各自
                # 独立绕回, 直接拿绝对值相减会得出莫名其妙的"丢包率"。
                if cr.tl:
                    cur_sp = sent // Q_PIXPKT
                    cur_pk = cr.tl["pkts"]
                    if prev_pk is None:      # 第一个窗口: 板子计数是从上电算的, 先对齐
                        prev_sp, prev_pk = cur_sp, cur_pk
                        print("      " + tl_text(cr) + "  本周期 基线对齐中")
                    else:
                        dsn = cur_sp - prev_sp
                        dpk = (cur_pk - prev_pk) & 0xFFFF
                        prev_sp, prev_pk = cur_sp, cur_pk
                        print("      " + tl_text(cr)
                              + "  本周期 发%d包/收%d包 收包率%.1f%%"
                              % (dsn, dpk, (100.0 * dpk / dsn) if dsn else 0.0))
    except KeyboardInterrupt:
        print("\n[stop] 共发送 %d 帧" % nf)
    finally:
        src_obj.close()
        if cs is not None:
            cs.close()


# ==================================================================
# 全屏模式
# ==================================================================
def run_full(sock, dst, args):
    W, H  = args.width, args.height
    MODE["known"] = True
    set_mode(0)                      # v23: --full 就是 PC 全屏这一档
    total = W * H
    npkt  = math.ceil(total / PIX_PER_PKT)

    mm      = None
    nframes = 0
    base    = None
    bar_bytes = b'\xff\xff' * args.bar

    sw, sh     = W, H
    fbytes     = total * 2
    fbytes_src = fbytes
    if args.tile:
        sw, sh = W // 2, H // 2
        fbytes_src = sw * sh * 2

    if args.video:
        if not os.path.isfile(args.video):
            raise SystemExit("找不到视频文件: %s" % args.video)
        fsize = os.path.getsize(args.video)
        fbytes_load = fbytes_src
        nframes = fsize // fbytes_load
        if args.frames:
            nframes = min(nframes, args.frames)
        if nframes <= 0:
            raise SystemExit("文件太小, 不足一帧 (%d 字节, 一帧需 %d 字节)"
                             % (fsize, fbytes_load))
        fh = open(args.video, "rb")
        mm = mmap.mmap(fh.fileno(), 0, access=mmap.ACCESS_READ)
        if args.tile:
            print("视频: %s   %d 帧   源 %dx%d -> 四宫格 %dx%d"
                  % (args.video, nframes, sw, sh, W, H))
        else:
            print("视频: %s   %d 帧" % (args.video, nframes))
        extra = fsize % fbytes_load
        if extra:
            print("   提示: 文件尾部有 %d 字节不足一帧, 已忽略" % extra)
    else:
        print("生成底色 %dx%d ..." % (W, H))
        base = build_base(W, H)

    print("-> %s:%d   %dx%d   %d 包/帧   负载 %d 字节/包"
          % (args.ip, args.port, W, H, npkt, PIX_PER_PKT * 2 + 8))
    print("   FPGA MAC 00:11:22:33:44:55, PC 需同网段 192.168.1.x")

    fid   = 0
    nf    = 0
    t0    = time.perf_counter()
    tnext = time.perf_counter()
    # 和 --quad 同一套节奏: 整帧铺开, 但不低于 40us/包
    per     = frame_period(args)
    pkt_gap = max((per - 0.002) / npkt, GAP_MIN_US * 1e-6)
    print("   帧率上限: %.1f fps   包间隔 %.0f us" % (1.0 / per, pkt_gap * 1e6))

    try:
        while True:
            if STOP:                 # GUI 点了"停止" -> 在帧边界干净退出
                break
            if mm is not None:
                idx = fid % nframes
                if args.tile:
                    src  = mm[idx * fbytes_src:(idx + 1) * fbytes_src]
                    data = tile2x2(src, sw, sh, W, H)
                else:
                    data = mm[idx * fbytes:(idx + 1) * fbytes]
            else:
                buf = bytearray(base)
                if args.bar:
                    bx = (fid * 8) % W
                    if bx + args.bar > W:
                        bx = W - args.bar
                    for y in range(H):
                        o = (y * W + bx) * 2
                        buf[o:o + args.bar * 2] = bar_bytes
                data = bytes(buf)

            send_frame(sock, data, fid, npkt, total, pkt_gap)

            fid = (fid + 1) & 0xFFFF
            nf += 1

            if nf % 20 == 0:
                dt = time.perf_counter() - t0
                print("frame %5d   %5.1f fps   %6.1f MB/s"
                      % (fid, nf / dt, (nf * W * H * 2 / 1e6) / dt))

            # 同样加板子帧率安全阀: 不限速会把板子灌爆, 画面变成跳着播
            # 用 wait_until 而不是 time.sleep: Windows sleep 粒度 15.6ms,
            # 24fps(41.7ms/帧) 会被它拖成 46ms/帧
            tnext += per
            if time.perf_counter() > tnext:      # 已经超时, 重锚
                tnext = time.perf_counter()
            else:
                wait_until(tnext)
    except KeyboardInterrupt:
        print("\n[stop] 共发送 %d 帧" % nf)
    finally:
        if mm is not None:
            mm.close()


def tile2x2(frame, sw, sh, W, H):
    """把 sw x sh 的 RGB565(大端) 帧平铺成 W x H 的四宫格"""
    out = bytearray(W * H * 2)
    src_stride = sw * 2
    dst_stride = W * 2
    for y in range(H):
        sy = (y % sh) * src_stride
        row = frame[sy:sy + src_stride]
        full = (row * ((W + sw - 1) // sw))[:dst_stride]
        out[y * dst_stride:(y + 1) * dst_stride] = full
    return bytes(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip",     default="192.168.1.10", help="FPGA 板卡 IP")
    ap.add_argument("--port",   type=int, default=8080, help="目标 UDP 端口")
    ap.add_argument("--width",  type=int, default=1024)
    ap.add_argument("--height", type=int, default=600)
    ap.add_argument("--bar",    type=int, default=16,   help="移动竖条宽度(像素), 0=静止")
    ap.add_argument("--fps",    type=float, default=0,
                    help="限速(每帧最少间隔), 0 = 用 --max-fps 的值")
    ap.add_argument("--max-fps", type=float, default=BOARD_FPS_DEFAULT,
                    help="板子能吃的帧率上限, 默认 %.1f。★别再设成 0/不限速★ "
                         "超了板子会丢包, 画面变成跳着播(看着像快进)"
                         % BOARD_FPS_DEFAULT)
    ap.add_argument("--video",  default="",             help="裸 RGB565(大端) 流文件")
    ap.add_argument("--raw-size", default="512x300",
                    help="--video 裸文件的原始分辨率, 默认 512x300 (make_video.py 的产出)。"
                         "素材尺寸和显示模式不一致时会自动用 ffmpeg 缩放, 但原始尺寸必须说对")
    ap.add_argument("--frames", type=int, default=0,    help="视频最多使用多少帧, 0=全部")
    ap.add_argument("--probe",  action="store_true",
                    help="发定位图案(白框+左上角块+正中十字), 用来判断格子映射对不对")
    ap.add_argument("--tile",   action="store_true",    help="全屏四宫格(纯 PC, 无摄像头)")
    ap.add_argument("--quad",   action="store_true",    help="硬件四宫格(左上留给摄像头)")
    ap.add_argument("--full",   action="store_true",
                    help="启动时就是全屏(板子按键1 切走后会自动跟随)")
    ap.add_argument("--mp4",    default="",
                    help="直接用 mp4 实时解码(按模式自动缩放), 不用先转 .rgb565")
    ap.add_argument("--row-us", type=float, default=88.0,
                    help="--quad/--no-flow: 每行发送间隔(us), 约等于摄像头两行时间, 默认 88")
    ap.add_argument("--no-flow", action="store_true",
                    help="--quad: 关闭信用流控, 改用 --row-us 固定节奏(更容易花屏)")
    ap.add_argument("--seg-ms", type=float, default=0.0,
                    help="(旧参数) 等价于 --row-us = seg-ms*1000/600")
    args = ap.parse_args()

    # 旧参数兼容: --seg-ms 30  <=>  --row-us 50
    if args.seg_ms > 0:
        args.row_us = args.seg_ms * 1000.0 / 600.0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    # 4MB 发送缓冲: 一帧 quad 是 900 包 x 1032B ≈ 0.93MB, 全屏 840 包 ≈ 0.6MB。
    # 缓冲够大才能在两次阻塞之间把一帧全部塞进内核, 不被 send 的返回时机切成一段段。
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 << 20)
    dst = (args.ip, args.port)
    # ★关键★ connect 一次, 后续全部用 send()
    #   UDP 的 connect 不发任何报文, 只是在 socket 上记下对端, 之后 send()
    #   跳过 sendto 的"每包查路由+校验地址"那一段, 实测 57.9us -> 8.8us 每包。
    sock.connect(dst)

    if args.quad:
        return run_quad(sock, dst, args)
    return run_full(sock, dst, args)


if __name__ == "__main__":
    main()
