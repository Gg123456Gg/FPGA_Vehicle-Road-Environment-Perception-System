#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
video_gui.py  --  视频传输小工具 (tkinter 界面)

为什么要有这个东西:
    命令行发视频要记一长串参数 (--quad --mp4 xxx.mp4 --ip ...), 比赛现场演示
    的时候手忙脚乱容易敲错。这里做成"列表里点一个 -> 点开始", 剩下的参数
    拼装全部交给 video_sender.py 里已经上板验证过的那套逻辑, 一行传输代码
    都没有重写。

界面:
    +------------------------------------------+
    | 视频列表 (来自程序旁边的 videos 文件夹)   |
    |   [浏览...]  [刷新]                       |
    |  目标 IP [..] 端口 [..]                   |
    |  发送方式 (o) 硬件四宫格  ( ) 全屏         |
    |  [ 开始传输 ]  [ 停止 ]                   |
    |  状态: ...                                |
    +------------------------------------------+

★ 停止是怎么实现的 ★
    video_sender.run_quad / run_full 都是 `while True` + `except KeyboardInterrupt`
    退出。GUI 在子线程里跑它们, 主线程没法给子线程抛异常, 所以在
    video_sender 里加了一个模块级 STOP 标志 (request_stop / clear_stop),
    每帧检查一次, 最多延迟一帧 (24fps => 42ms) 退出。

★ 为什么不能直接 subprocess 调 video_sender.py ★
    打包成 exe 之后, video_sender.py 会被塞进 exe 内部 (不再是一个独立文件),
    subprocess 就找不到它了。所以这里必须 import 进来在同一个进程里跑。

★ 运行环境 ★
    必须用带 tkinter 的 Python。本机默认的 python (3.13, WorkBuddy 自带那份)
    没有 tkinter, 要用 C:\\Python\\python.exe (3.10)。打包成 exe 后就无所谓了。

★ v22 改动 (现场提的四个问题) ★
   1) 列表里的条目没法删 -> 加了 "移除" 按钮 (只清列表, 不动磁盘);
      勾上 "同时删掉文件" 才真的删, 而且是扔进回收站, 能撤回来。
   2) 传输时弹黑窗口 -> ffmpeg 是控制台程序, 从无控制台的 GUI 里启动会被
      Windows 新开一个 cmd。video_sender 里给 Popen 加了 CREATE_NO_WINDOW。
   3) 换台机器/换个文件夹能不能用 -> exe 本身可以随便挪; videos 文件夹得跟着
      exe 走; ffmpeg 优先用 exe 旁边的那份, 找不到再吃系统 PATH。
   4) "全屏" 单选没用 -> 这不是 bug: 显示模式由【板子上的按键 1】决定, 板子
      通过信用包 flags[7] 告诉 PC, PC 只能跟着切。所以选项换成了实时状态显示。
"""
import os
import sys
import threading
import time
import socket
import argparse
import traceback

# ---- 无控制台模式 (--noconsole) 下 sys.stdout 是 None, 而 video_sender 里到处
#     都是 print, 会直接抛 AttributeError: 'NoneType' object has no attribute 'write'。
#     所以要在 import 之前先把 stdout/stderr 替换掉。用 Sink 而不是 StringIO:
#     StringIO 会把所有输出都存着, 跑一晚能吃掉几百 MB。
class Sink(object):
    """丢弃输出, 只保留最近几十行 (给状态栏和出错排查用)"""

    def __init__(self, keep=60):
        self.lines = []
        self.keep = keep
        self._buf = ""

    def write(self, s):
        if not s:
            return
        self._buf += s
        while "\n" in self._buf:
            line, self._buf = self._buf.split("\n", 1)
            line = line.rstrip()
            if line:
                self.lines.append(line)
        if len(self.lines) > self.keep:
            del self.lines[:len(self.lines) - self.keep]

    def flush(self):
        pass

    def last(self):
        return self.lines[-1] if self.lines else ""

    def tail(self, n=3):
        return self.lines[-n:]


class Tee(object):
    """既进 SINK 也照常打到原来的 stdout —— 源码方式运行时还能在控制台看完整日志,
    界面上也照样有最后一行。exe (无控制台) 时根本走不到这里, 直接把 stdout 换成 SINK。"""

    def __init__(self, sink, orig):
        self.sink = sink
        self.orig = orig

    def write(self, s):
        self.sink.write(s)
        try:
            self.orig.write(s)
        except Exception:
            pass

    def flush(self):
        try:
            self.orig.flush()
        except Exception:
            pass


SINK = Sink()
if sys.stdout is None or sys.stderr is None:
    # --noconsole 的 exe: 没有 stdout, 不换掉的话 video_sender 里每个 print
    # 都会抛 AttributeError: 'NoneType' object has no attribute 'write'
    sys.stdout = SINK
    sys.stderr = SINK
else:
    sys.stdout = Tee(SINK, sys.stdout)
    sys.stderr = Tee(SINK, sys.stderr)

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)


# ==================================================================
# 路径: 打包成 exe 后 __file__ 指向临时解压目录, videos 必须放在 exe 旁边,
#       否则用户把视频放进 exe 旁边是找不到的 (双击也不会有反应)。
# ==================================================================
def app_dir():
    if getattr(sys, "frozen", False):           # PyInstaller 打出来的 exe
        return os.path.dirname(sys.executable)
    return HERE


def prep_tools(d):
    """把 exe 旁边的 ffmpeg.exe 挂到 PATH 最前面

    为什么要这样: 这个工具真正依赖外部程序的只有 ffmpeg 一个。如果只靠系统
    PATH, 把 exe 拷到比赛用的那台机器上 (没装 ffmpeg) 就发不了 mp4, 而且报的
    是"找不到 ffmpeg"这种看不懂的错。把 ffmpeg.exe 跟 exe 放同一个文件夹,
    这里自动认它, 整个文件夹拷到哪台机器都能用。
    """
    if os.name != "nt":
        return
    if os.path.isfile(os.path.join(d, "ffmpeg.exe")):
        os.environ["PATH"] = d + os.pathsep + os.environ.get("PATH", "")


prep_tools(app_dir())

import video_sender as vs

import tkinter as tk
from tkinter import ttk, filedialog, messagebox


VIDEO_DIR = os.path.join(app_dir(), "videos")

BG      = "#f4f5f7"
CARD    = "#ffffff"
FG      = "#1f2329"
FG_DIM  = "#8a9099"
ACCENT  = "#2f6fed"
OK      = "#1a9c53"
WARN    = "#d98014"   # v23: 摄像头全屏用橙色(那会儿 PC 是停在一边的)
ERR     = "#d93026"
FONT    = ("Microsoft YaHei UI", 10)
FONT_B  = ("Microsoft YaHei UI", 10, "bold")
FONT_S  = ("Microsoft YaHei UI", 9)


def recycle(path):
    """把文件扔进 Windows 回收站 (不是永久删除, 后悔了能右键还原)

    用 SHFileOperationW + FOF_ALLOWUNDO: 这是资源管理器"删除"走的同一个接口,
    所以行为完全一样 —— 进回收站、能还原。不装 send2trash 之类的东西。
    返回 True 表示删掉了 (或文件本来就不在)。
    """
    if os.name != "nt":
        try:
            os.remove(path)
            return True
        except OSError:
            return False

    import ctypes
    from ctypes import wintypes

    class SHFILEOPSTRUCTW(ctypes.Structure):
        _fields_ = [("hwnd", wintypes.HWND),
                    ("wFunc", wintypes.UINT),
                    ("pFrom", wintypes.LPCWSTR),
                    ("pTo", wintypes.LPCWSTR),
                    ("fFlags", ctypes.c_uint16),
                    ("fAnyOperationsAborted", wintypes.BOOL),
                    ("hNameMappings", ctypes.c_void_p),
                    ("lpszProgressTitle", wintypes.LPCWSTR)]

    FO_DELETE = 3
    FOF_ALLOWUNDO      = 0x0040
    FOF_NOCONFIRMATION = 0x0010
    FOF_SILENT         = 0x0004
    FOF_NOERRORUI      = 0x0400

    if not os.path.exists(path):
        return True
    # pFrom 必须以 \0\0 结尾 (它是"以双空结尾的路径列表", 不是普通字符串)
    src = os.path.abspath(path) + "\0\0"
    op = SHFILEOPSTRUCTW(None, FO_DELETE, src, None,
                         FOF_ALLOWUNDO | FOF_NOCONFIRMATION | FOF_SILENT | FOF_NOERRORUI,
                         False, None, None)
    rc = ctypes.windll.shell32.SHFileOperationW(ctypes.byref(op))
    return rc == 0 and not op.fAnyOperationsAborted


class App(object):
    def __init__(self, root):
        self.root = root
        self.th = None                  # 发送线程
        self.running = False
        self.t_start = 0.0
        self.result = None              # 线程结束原因: None/ok/err
        self.err_msg = ""

        root.title("视频传输  ·  FPGA 显示")
        root.configure(bg=BG)
        # ★高度按自然尺寸量出来的★ 列表卡片是 expand 的, 但 reqheight 由内容
        #   决定: 标题+列表最小高+参数条+按钮+两行状态 = 534。设 470 会把
        #   底部的状态栏和日志行裁掉 (看着像"点了没反应")。给到 560 留余量。
        # ★窗口尺寸不写死★
        #   这台机器是 150% 缩放, tk 的 10pt 字在物理像素下比 96dpi 时大一半,
        #   写死 540 像素高会把底部按钮和状态栏挤出窗口 (截图里就是"按钮只剩一半")。
        #   改成"按内容自然高度撑开", 换台机器/换缩放比例都不会裁。

        self.files = []                 # 列表里每项对应的完整路径
        self.build_ui()
        self.refresh_list()
        self.fit_window()
        self.tick()

        root.protocol("WM_DELETE_WINDOW", self.on_close)

    # ---------------- 布局 ----------------
    def fit_window(self):
        """按内容的自然尺寸定窗口大小, 然后居中 —— 不依赖 DPI 缩放比例"""
        self.root.update_idletasks()
        w = max(680, self.root.winfo_reqwidth())
        h = self.root.winfo_reqheight()
        # minsize 也设成自然尺寸: 用户手动拉小的时候不允许把内容裁掉
        self.root.minsize(w, h)
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        self.root.geometry("%dx%d+%d+%d" % (w, h, (sw - w) // 2, (sh - h) // 4))

    def build_ui(self):
        pad = 14
        # 标题
        head = tk.Frame(self.root, bg=BG)
        head.pack(fill="x", padx=pad, pady=(pad, 6))
        tk.Label(head, text="视频传输", font=("Microsoft YaHei UI", 15, "bold"),
                 bg=BG, fg=FG).pack(side="left")
        tk.Label(head, text="选视频 → 点开始", font=FONT_S,
                 bg=BG, fg=FG_DIM).pack(side="left", padx=(10, 0), pady=(4, 0))

        # 视频列表卡片
        card = tk.Frame(self.root, bg=CARD, highlightthickness=1,
                        highlightbackground="#e3e5e8")
        # ★不要 expand=True★ 列表卡是唯一能长高的东西, 一旦让它吃满剩余空间,
        #   pack 就会把后面的参数条/按钮/状态栏全部推到窗口外 —— 截图里就是
        #   "按钮不见了、窗口像被切掉一半"。这里用固定的 8 行高度, 多了靠滚动条。
        card.pack(fill="x", padx=pad, pady=(0, 8))

        bar = tk.Frame(card, bg=CARD)
        bar.pack(fill="x", padx=10, pady=(8, 4))
        tk.Label(bar, text="视频文件", font=FONT_B, bg=CARD, fg=FG).pack(side="left")
        tk.Label(bar, text="videos 文件夹", font=FONT_S, bg=CARD,
                 fg=FG_DIM).pack(side="left", padx=(8, 0))

        body = tk.Frame(card, bg=CARD)
        body.pack(fill="both", expand=True, padx=10, pady=(0, 6))

        self.lb = tk.Listbox(body, font=FONT, bg="#fbfbfc", fg=FG, height=8,
                             activestyle="none", borderwidth=1, relief="solid",
                             highlightthickness=0, selectbackground=ACCENT,
                             selectforeground="#ffffff")
        self.lb.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(body, orient="vertical", command=self.lb.yview)
        sb.pack(side="left", fill="y")
        self.lb.config(yscrollcommand=sb.set)
        self.lb.bind("<Double-Button-1>", lambda e: self.start())
        self.lb.bind("<Delete>", lambda e: self.remove_selected())
        self.lb.bind("<BackSpace>", lambda e: self.remove_selected())

        btns = tk.Frame(card, bg=CARD)
        btns.pack(fill="x", padx=10, pady=(0, 10))
        self.b_browse = self.mk_btn(btns, "浏览…", self.browse, primary=False)
        self.b_browse.pack(side="left")
        self.b_remove = self.mk_btn(btns, "移除", self.remove_selected, primary=False)
        self.b_remove.pack(side="left", padx=(8, 0))
        self.b_refresh = self.mk_btn(btns, "刷新", self.refresh_list, primary=False)
        self.b_refresh.pack(side="left", padx=(8, 0))

        self.v_del = tk.BooleanVar(value=False)
        self.c_del = tk.Checkbutton(btns, text="同时删掉文件", variable=self.v_del,
                                    font=FONT_S, bg=CARD, fg=FG_DIM,
                                    activebackground=CARD, activeforeground=FG,
                                    selectcolor=CARD, cursor="hand2",
                                    highlightthickness=0, bd=0)
        self.c_del.pack(side="left", padx=(14, 0))

        # ★定位图案★
        #   勾上以后不发视频, 发"白框 + 六个色块 + 正中十字"的静态图。
        #   屏幕上一旦缺块/错位, 图案会立刻变形(框断口、十字断开、色块错位),
        #   比看视频更容易判断到底坏在哪一块。不用选视频文件也能开始。
        self.v_probe = tk.BooleanVar(value=False)
        self.c_probe = tk.Checkbutton(btns, text="定位图案", variable=self.v_probe,
                                      font=FONT_S, bg=CARD, fg=FG_DIM,
                                      activebackground=CARD, activeforeground=FG,
                                      selectcolor=CARD, cursor="hand2",
                                      highlightthickness=0, bd=0)
        self.c_probe.pack(side="left", padx=(14, 0))

        self.lb_hint = tk.Label(btns, text="", font=FONT_S, bg=CARD, fg=FG_DIM)
        self.lb_hint.pack(side="right")

        # 参数卡片
        opt = tk.Frame(self.root, bg=CARD, highlightthickness=1,
                       highlightbackground="#e3e5e8")
        opt.pack(fill="x", padx=pad, pady=(0, 8))
        row = tk.Frame(opt, bg=CARD)
        row.pack(fill="x", padx=10, pady=9)

        tk.Label(row, text="目标 IP", font=FONT, bg=CARD, fg=FG).pack(side="left")
        self.v_ip = tk.StringVar(value="192.168.1.10")
        tk.Entry(row, textvariable=self.v_ip, font=FONT, width=14,
                 relief="solid", borderwidth=1).pack(side="left", padx=(6, 14))

        tk.Label(row, text="端口", font=FONT, bg=CARD, fg=FG).pack(side="left")
        self.v_port = tk.StringVar(value="8080")
        tk.Entry(row, textvariable=self.v_port, font=FONT, width=6,
                 relief="solid", borderwidth=1).pack(side="left", padx=(6, 16))

        self.v_mode = tk.StringVar(value="—")     # v24: 没收到遥测前不要假装知道
        tk.Label(row, text="显示模式", font=FONT, bg=CARD, fg=FG).pack(side="left")
        self.l_mode = tk.Label(row, textvariable=self.v_mode, font=FONT_B,
                               bg=CARD, fg=FG_DIM)
        self.l_mode.pack(side="left", padx=(8, 6))
        tk.Label(row, text="板子按键1 切换", font=FONT_S, bg=CARD,
                 fg=FG_DIM).pack(side="left")

        # ---- 画面算法档位卡片 (v24) ----
        #   以前这是板子上按键1 干的活, v24 起按键1 改去切显示模式了,
        #   算法改成在这里点 —— 板子那边收到 UDP 命令就换 (一帧后生效)。
        #   右边那个"板子当前"是板子回传的真实值, 不是"我点过什么":
        #   命令丢包的时候这里会一直停在旧档位, 发送循环里每 0.5s 自动重发,
        #   所以正常情况下 1 秒内会自己追上。
        alg = tk.Frame(self.root, bg=CARD, highlightthickness=1,
                       highlightbackground="#e3e5e8")
        alg.pack(fill="x", padx=pad, pady=(0, 8))
        arow = tk.Frame(alg, bg=CARD)
        arow.pack(fill="x", padx=10, pady=9)

        tk.Label(arow, text="画面算法", font=FONT, bg=CARD, fg=FG).pack(side="left")
        self.b_algo = []
        for i, nm in enumerate(vs.ALGO_NAME):
            b = tk.Button(arow, text=nm, font=FONT_S, bd=0, relief="flat",
                          padx=10, pady=3, cursor="hand2",
                          command=lambda k=i: self.pick_algo(k))
            b.pack(side="left", padx=(9 if i == 0 else 4, 0))
            self.b_algo.append(b)

        self.v_algo = tk.StringVar(value="—")
        self.l_algo = tk.Label(arow, textvariable=self.v_algo, font=FONT_B,
                               bg=CARD, fg=FG_DIM)
        self.l_algo.pack(side="right")
        tk.Label(arow, text="板子当前", font=FONT_S, bg=CARD,
                 fg=FG_DIM).pack(side="right", padx=(0, 6))
        self.paint_algo(-1)

        # ---- 车道识别【显示】开关 (v50 加, v51 改成只管显示) ----
        #   和算法档位同一张卡片: 两个都是"改板子行为"的按钮, 摆在一起顺手,
        #   而且都不占视频列表的地方。
        #
        #   ★这是显示开关, 不是识别开关★
        #     关掉以后只是 OSD 那一层不画 —— 左上角的点、色块、决策文字消失,
        #     画面变成干净的原始视频。板子内部的检测流水(每像素游程 / 逐行统计 /
        #     帧末判型 / 变道决策)照常逐帧运行, 报警灯和三个车态灯也照常亮。
        #     所以这是"不看结果", 不是"不干活"; 重新打开当帧即正确, 不用等收敛。
        #     指令走 UDP 0xA55C, 上电默认显示。
        lrow = tk.Frame(alg, bg=CARD)
        lrow.pack(fill="x", padx=10, pady=(0, 9))
        tk.Label(lrow, text="车道识别显示", font=FONT, bg=CARD,
                 fg=FG).pack(side="left")
        self.v_lane = tk.BooleanVar(value=True)
        self.b_lane = tk.Button(lrow, text="显示", font=FONT_B, bd=0, relief="flat",
                                padx=16, pady=3, cursor="hand2",
                                command=self.toggle_lane)
        self.b_lane.pack(side="left", padx=(9, 0))
        tk.Label(lrow, text="只关画面这一层，识别与车态灯照常在跑",
                 font=FONT_S, bg=CARD, fg=FG_DIM).pack(side="left", padx=(12, 0))
        self.paint_lane()

        # 底部操作区
        foot = tk.Frame(self.root, bg=BG)
        foot.pack(fill="x", padx=pad, pady=(0, pad))
        self.b_start = self.mk_btn(foot, "开始传输", self.start, primary=True)
        self.b_start.pack(side="left")
        self.b_stop = self.mk_btn(foot, "停止", self.stop, primary=False)
        self.b_stop.pack(side="left", padx=(8, 0))
        self.set_disabled(self.b_stop)

        self.v_state = tk.StringVar(value="就绪 · 选好视频后点 \"开始传输\"")
        self.l_state = tk.Label(self.root, textvariable=self.v_state, font=FONT_B,
                                bg=BG, fg=FG_DIM, anchor="w")
        self.l_state.pack(fill="x", padx=pad + 2, pady=(0, 2))

        self.v_log = tk.StringVar(value="")
        self.l_log = tk.Label(self.root, textvariable=self.v_log, font=FONT_S,
                              bg=BG, fg=FG_DIM, anchor="w")
        self.l_log.pack(fill="x", padx=pad + 2, pady=(0, 8))

    def set_disabled(self, btn, primary=False):
        # tk.Button 的 disabled 只改文字颜色, 背景仍然是蓝的, 看起来像还能点。
        # 手动把底色也压灰 —— 现场演示最怕"点了没反应", 得一眼看出哪个能点。
        btn.config(state="disabled",
                   bg=("#aeb9cf" if primary else "#f4f5f7"),
                   activebackground=("#aeb9cf" if primary else "#f4f5f7"))

    def set_enabled(self, btn, primary=False):
        btn.config(state="normal",
                   bg=(ACCENT if primary else "#eceef1"),
                   activebackground=("#2a63d4" if primary else "#e2e5e9"))

    def mk_btn(self, parent, text, cmd, primary):
        if primary:
            b = tk.Button(parent, text=text, command=cmd, font=FONT_B,
                          bg=ACCENT, fg="#ffffff", activebackground="#2a63d4",
                          activeforeground="#ffffff", relief="flat",
                          padx=22, pady=6, cursor="hand2",
                          disabledforeground="#c9cdd4")
        else:
            b = tk.Button(parent, text=text, command=cmd, font=FONT,
                          bg="#eceef1", fg=FG, activebackground="#e2e5e9",
                          relief="flat", padx=16, pady=6, cursor="hand2",
                          disabledforeground="#b9bec6")
        return b

    # ---------------- 画面算法档位 (v24) ----------------
    def paint_algo(self, now, want=None):
        """画 5 个算法按钮。

        now  = 板子回传的实际档位 (-1 = 还没收到 / 旧固件没有这个字段)
        want = 刚点下去、还没被板子确认的那个 -> 画成浅蓝, 让人知道"点了, 在路上"
        """
        for i, b in enumerate(self.b_algo):
            if i == now:
                b.config(bg=ACCENT, fg="#ffffff",
                         activebackground="#2a63d4", activeforeground="#ffffff")
            elif want is not None and i == want:
                b.config(bg="#d7e2f7", fg=ACCENT,
                         activebackground="#c7d6f2", activeforeground=ACCENT)
            else:
                b.config(bg="#eceef1", fg=FG,
                         activebackground="#e2e5e9", activeforeground=FG)

    def pick_algo(self, k):
        vs.set_algo(k)
        self.v_algo.set("发送中…")
        self.l_algo.config(fg=ACCENT)
        self.paint_algo(vs.ALGO["now"], k)

        try:
            port = int(self.v_port.get().strip())
        except Exception:
            port = 8080
        ip = self.v_ip.get().strip() or "192.168.1.10"
        # 连发 5 份: UDP 会丢, 多打几枪比做重传机制省事。
        # 真正跑起来之后, 发送循环里每 0.5s 还会对着回传值重发, 不会漏。
        vs.push_algo(ip, port, times=5)

    # ---------------- 车道识别显示开关 (v50 加, v51 只管显示) ----------------
    def paint_lane(self):
        """显示 = 绿底白字, 隐藏 = 灰底黑字 —— 现场一眼能看出现在是哪种"""
        on = self.v_lane.get()
        self.b_lane.config(text="显示" if on else "隐藏",
                           bg=(OK if on else "#eceef1"),
                           fg=("#ffffff" if on else FG),
                           activebackground=(OK if on else "#e2e5e9"),
                           activeforeground=("#ffffff" if on else FG))

    def toggle_lane(self):
        on = 0 if self.v_lane.get() else 1
        self.v_lane.set(bool(on))
        vs.set_lane(on)
        self.paint_lane()

        try:
            port = int(self.v_port.get().strip())
        except Exception:
            port = 8080
        ip = self.v_ip.get().strip() or "192.168.1.10"
        # 没有回读可对, 所以这里多发几份; 传输过程中发送循环每 1s 还会补 3 份。
        vs.push_lane(ip, port, times=5)
        self.v_state.set("车道识别显示: %s   ·   已下发" % ("显示" if on else "隐藏"))
        SINK.write("[lane] 车道识别显示 -> %s (0xA55C x5)\n" % ("on" if on else "off"))

    # ---------------- 列表 ----------------
    def refresh_list(self):
        try:
            os.makedirs(VIDEO_DIR, exist_ok=True)
        except Exception:
            pass
        self.files = []
        self.lb.delete(0, "end")
        if os.path.isdir(VIDEO_DIR):
            for fn in sorted(os.listdir(VIDEO_DIR)):
                if fn.lower().endswith((".mp4", ".mov", ".avi", ".mkv")):
                    full = os.path.join(VIDEO_DIR, fn)
                    self.files.append(full)
                    self.lb.insert("end", "  " + fn + "   " + self.size_str(full))
        if self.files:
            self.lb.selection_set(0)
            self.lb_hint.config(text="共 %d 个" % len(self.files))
        else:
            self.lb_hint.config(text="videos 文件夹是空的, 点\"浏览…\"选一个")

    @staticmethod
    def size_str(p):
        try:
            n = os.path.getsize(p)
            if n < 1048576:
                return "%d KB" % max(1, n // 1024)
            return "%.1f MB" % (n / 1048576.0)
        except Exception:
            return ""

    def browse(self):
        p = filedialog.askopenfilename(
            title="选择视频",
            filetypes=[("视频", "*.mp4 *.mov *.avi *.mkv"), ("全部文件", "*.*")])
        if not p:
            return
        full = os.path.abspath(p)
        self.files.append(full)
        self.lb.insert("end", "  " + os.path.basename(full) + "   "
                       + self.size_str(full))
        self.lb.selection_clear(0, "end")
        self.lb.selection_set("end")
        self.lb.see("end")
        self.lb_hint.config(text="共 %d 个" % len(self.files))

    def pick(self):
        sel = self.lb.curselection()
        if not sel or sel[0] >= len(self.files):
            return None
        return self.files[sel[0]]

    def remove_selected(self):
        """从列表里拿掉选中的条目

        默认只动列表, 不碰磁盘 —— 现场临时拖进来的 mp4 往往还想留着。勾了
        "同时删掉文件" 才会真删, 走回收站, 删错了能还原。
        """
        if self.running:
            return
        sel = self.lb.curselection()
        if not sel:
            self.v_log.set("先在列表里点一条, 再按\"移除\"")
            return
        i = sel[0]
        path = self.files[i]
        name = os.path.basename(path)

        if self.v_del.get():
            if not messagebox.askokcancel(
                    "删掉文件?",
                    "把下面这个文件扔进回收站?\n\n%s\n\n(回收站里能还原)" % path):
                return
            if not recycle(path):
                messagebox.showerror("删除失败", "删不掉:\n%s\n\n可能正被别的程序占用。"
                                                 % path)
                return
            note = "已删除(进回收站): " + name
        else:
            note = "已从列表移除(文件还在): " + name

        del self.files[i]
        self.lb.delete(i)
        # 删完顺手选中相邻的一条, 连按 Delete 时不用每次重新点
        if self.files:
            n = min(i, len(self.files) - 1)
            self.lb.selection_clear(0, "end")
            self.lb.selection_set(n)
            self.lb.activate(n)
            self.lb.see(n)
        self.lb_hint.config(text="共 %d 个" % len(self.files))
        self.v_log.set(note[:100])

    # ---------------- 开始 / 停止 ----------------
    def start(self):
        if self.running:
            return
        probe = bool(self.v_probe.get())
        path = "" if probe else self.pick()
        if not probe:
            if not path:
                messagebox.showinfo("还没选视频", "先在列表里选一个视频, 或者点\"浏览…\"。")
                return
            if not os.path.isfile(path):
                messagebox.showerror("文件不存在", "找不到:\n%s" % path)
                self.refresh_list()
                return

        try:
            port = int(self.v_port.get().strip())
        except Exception:
            messagebox.showerror("端口不对", "端口要填数字。")
            return
        ip = self.v_ip.get().strip()
        if not ip:
            messagebox.showerror("IP 为空", "填一下板子的 IP。")
            return

        # ★显示模式不是这里选的★
        #   三态由板子上按键 1 决定, 板子在遥测里回传, PC 只能
        #   跟着切。以前这里有个"全屏"单选, 点了没反应就是因为 PC 把全屏的图塞进
        #   板子的四宫格布局里 —— 板子该怎么显示还是怎么显示。所以固定走
        #   run_quad (它内部会跟随板子), 界面上只做状态显示。
        args = argparse.Namespace(
            ip=ip, port=port, width=1024, height=600, bar=16,
            fps=0.0, max_fps=vs.BOARD_FPS_DEFAULT,
            video="", raw_size="512x300", frames=0,
            probe=probe, tile=False,
            quad=True, full=False,
            mp4=("" if probe else path), row_us=88.0, no_flow=False, seg_ms=0.0,
        )

        vs.clear_stop()
        SINK.lines = []
        self.result = None
        self.err_msg = ""
        self.running = True
        self.t_start = time.perf_counter()
        self.set_disabled(self.b_start, primary=True)
        self.set_enabled(self.b_stop)
        self.set_disabled(self.b_browse)
        self.set_disabled(self.b_remove)
        self.set_disabled(self.b_refresh)
        self.c_del.config(state="disabled")
        self.c_probe.config(state="disabled")
        self.l_state.config(fg=ACCENT)
        self.v_state.set("正在连接…")

        self.th = threading.Thread(target=self.worker, args=(args,), daemon=True)
        self.th.start()

    def worker(self, args):
        """在子线程里跑真正上板验证过的那套发送逻辑, 一行没改"""
        sock = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 4 << 20)
            dst = (args.ip, args.port)
            sock.connect(dst)            # UDP connect 不发包, 只是省掉每包查路由
            if args.quad:
                vs.run_quad(sock, dst, args)
            else:
                vs.run_full(sock, dst, args)
            self.result = "ok"
        except Exception:
            self.result = "err"
            self.err_msg = traceback.format_exc()
        finally:
            try:
                if sock is not None:
                    sock.close()
            except Exception:
                pass

    def stop(self):
        if not self.running:
            return
        self.set_disabled(self.b_stop)
        self.v_state.set("正在停止…")
        vs.request_stop()

    # ---------------- 定时刷新状态 ----------------
    def tick(self):
        # ★异常必须在这里吃掉★
        #   定时器是靠 self.root.after(200, self.tick) 自己续命的, 一旦中间抛异常,
        #   这一行就走不到, 状态栏从此不再刷新 —— 表现为"线程已经停了, 界面还写着
        #   发送中", 而且停止按钮怎么也点不出结果。所以用 try/finally 保证续命。
        try:
            self._tick_once()
        except Exception:
            SINK.write("[tick] " + traceback.format_exc().replace("\n", " ") + "\n")
        finally:
            self.root.after(200, self.tick)

    def _tick_once(self):
        if self.running:
            if self.th is not None and not self.th.is_alive():
                self.finish()
            else:
                dt = time.perf_counter() - self.t_start
                mode = vs.MODE
                m = mode.get("mode", 1)
                if mode["known"]:
                    # v23 三态: 0=PC 全屏(绿) 1=四宫格(蓝) 2=摄像头全屏(橙)
                    self.v_mode.set(mode["name"])
                    self.l_mode.config(fg=(OK if m == 0 else
                                           (WARN if m == 2 else ACCENT)))
                    tail = "  ·  PC 暂停发送" if m == 2 else ""
                    self.v_state.set("发送中   %s   ·   %s%s"
                                     % (self.dur(dt), mode["name"], tail))
                else:
                    self.v_mode.set("等待板子…")
                    self.l_mode.config(fg=FG_DIM)
                    self.v_state.set("发送中   %s   ·   等板子回传遥测" % self.dur(dt))
                self.l_state.config(fg=ACCENT)
                # v24: 算法档位也照着板子的回传刷新 —— 显示的是"板子现在是什么",
                # 不是"我点过什么"。两者不一致时发送循环会自己重发。
                a = vs.ALGO["now"]
                if a >= 0:
                    self.v_algo.set(vs.ALGO_NAME[a])
                    self.l_algo.config(fg=FG)
                    self.paint_algo(a)
                else:
                    self.v_algo.set("—")
                    self.l_algo.config(fg=FG_DIM)
                    self.paint_algo(-1, vs.ALGO["want"])
                last = SINK.last()
                self.l_log.config(fg=FG_DIM)
                hint = {0: "PC 全屏 1024x600, 整屏都是电脑画面",
                        1: "四宫格: 左上 512x300 留给 OV5640 摄像头",
                        2: "摄像头全屏 1024x600, 板子摄像头原生画面"}
                self.v_log.set(last[:100] if last else hint.get(m, hint[1]))

    def finish(self):
        dt = time.perf_counter() - self.t_start
        self.running = False
        self.set_enabled(self.b_start, primary=True)
        self.set_disabled(self.b_stop)
        self.set_enabled(self.b_browse)
        self.set_enabled(self.b_remove)
        self.set_enabled(self.b_refresh)
        self.c_del.config(state="normal")
        self.c_probe.config(state="normal")
        if self.result == "err":
            self.l_state.config(fg=ERR)
            self.v_state.set("出错了 · 已停止 %s" % self.dur(dt))
            tail = "\n".join(SINK.tail(12)) or self.err_msg
            self.v_log.set(tail.split("\n")[-1][:100])
            messagebox.showerror("发送失败", self.err_msg[-1200:])
        else:
            self.l_state.config(fg=OK)
            self.v_state.set("已停止   %s" % self.dur(dt))
            last = SINK.last()
            self.v_log.set(last[:100] if last else "")

    @staticmethod
    def dur(sec):
        m, s = divmod(int(sec), 60)
        return ("%d 分 %02d 秒" % (m, s)) if m else ("%d.%d 秒" % (s, int(sec * 10) % 10))

    def on_close(self):
        if self.running:
            if not messagebox.askokcancel("还在发送", "正在发送视频, 确定要退出吗?"):
                return
            vs.request_stop()
            if self.th is not None:
                self.th.join(timeout=1.5)
        self.root.destroy()


def main():
    # 打包后没有控制台, 这里顺手把 ffmpeg 能不能用查一下 —— 缺了的话
    # mp4 解码会失败, 但报错信息在 GUI 里看不清, 所以提前弹窗。
    # 查的是"prep_tools 之后"的 PATH: 也就是说 exe 旁边放了 ffmpeg.exe 就不算缺。
    import shutil
    if shutil.which("ffmpeg") is None:
        r = tk.Tk()
        r.withdraw()
        messagebox.showwarning(
            "找不到 ffmpeg",
            "找不到 ffmpeg, 发 mp4 会失败。\n\n"
            "解决(推荐): 把 ffmpeg.exe 拷到本程序同一个文件夹里, 再重新打开。\n"
            "整个文件夹连同 videos 一起拷走, 换台电脑也能直接用。\n\n"
            "或者: 装 ffmpeg 并加进系统 PATH。")
        r.destroy()

    root = tk.Tk()
    App(root)
    root.mainloop()


if __name__ == "__main__":
    main()
