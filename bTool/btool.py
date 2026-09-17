#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
bTool — ESP32 系列图形化烧写 / 文件管理 / REPL 工具

功能:
  1. 串口自动检测, 下拉选择
  2. 芯片型号选择 (ESP32 / ESP32-S3 / ESP32-C3 / ESP32-C6)
  3. 本地 .bin 固件烧写 (可多文件+多地址), 带进度条, 可选先擦除
  4. 本地文件 ↔ 设备 VFS 双向传输, 支持上传/下载/删除/重命名
  5. 底部 REPL 交互终端, 支持 Ctrl+C 中断、raw REPL 切换

依赖:
  pip install pyserial esptool

运行:
  python btool.py
"""

import base64
import os
import queue
import sys
import threading
import time
import traceback

import tkinter as tk
from tkinter import filedialog, messagebox, ttk

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("缺少 pyserial:  pip install pyserial")

try:
    import esptool
    import esptool.cmds
    from esptool.logger import EsptoolLogger
except ImportError:
    esptool = None
    EsptoolLogger = None


APP_TITLE = "bTool — ESP32 图形化工具"
DEFAULT_BAUD = 115200
REPL_BAUD = 115200

# 按文件名猜烧写地址 (bPuppy / ESP-IDF 常见布局)
ADDR_GUESS = (
    ("bootloader", 0x0),
    ("partition-table", 0x8000),
    ("partition_table", 0x8000),
    ("boot_app0", 0xE000),
)


def guess_addr(path):
    """按文件名猜烧写地址; 猜不出就给 app 区默认值 0x10000"""
    name = os.path.basename(path).lower()
    for key, addr in ADDR_GUESS:
        if key in name:
            return addr
    return 0x10000


# ======================================================================
# esptool 进度钩子
# ======================================================================
# esptool 5.x 把日志/进度条收在一个单例 logger 里, 默认直接往 stdout 打。
# 子类化 EsptoolLogger 覆写 print/progress_bar, 把进度送回 Tkinter。

class BridgeLogger(EsptoolLogger if EsptoolLogger else object):
    """把 esptool 的输出和进度转发给回调 (由 UI 层注入)。"""

    sink = None          # callable(text: str)
    progress = None      # callable(cur: int, total: int)

    def print(self, *args, **kwargs):
        try:
            text = " ".join(str(a) for a in args)
        except Exception:
            return
        if BridgeLogger.sink:
            BridgeLogger.sink(text.rstrip())

    def note(self, message):
        if BridgeLogger.sink:
            BridgeLogger.sink("· " + str(message))

    def warning(self, message):
        if BridgeLogger.sink:
            BridgeLogger.sink("⚠ " + str(message))

    def error(self, message):
        if BridgeLogger.sink:
            BridgeLogger.sink("✗ " + str(message))

    def stage(self, message=None, *a, **kw):
        if BridgeLogger.sink and message:
            BridgeLogger.sink("▶ " + str(message))

    def progress_bar(self, cur_iter, total_iters, prefix="", suffix="", bar_length=30):
        if BridgeLogger.progress and total_iters:
            BridgeLogger.progress(cur_iter, total_iters)


# ======================================================================
# 串口管理 —— 同一时刻只允许一个功能占用
# ======================================================================
# 三种用途:
#   'repl'  普通 REPL   (交互输入)
#   'raw'   raw REPL    (程序化文件操作)
#   'flash' esptool 烧写
#
# repl 和 raw 是**同一条串口的两种模式** (Ctrl-A / Ctrl-B 切换), 不用重开;
# 只有烧写要真正独占 —— esptool 自己开串口, 所以烧写前必须先关掉我们的。

class SerialError(Exception):
    pass


class SerialManager:
    def __init__(self):
        self._ser = None
        self._port = None
        self._baud = DEFAULT_BAUD
        self._mode = "none"
        self.lock = threading.RLock()

    # ---- 生命周期 ----
    @property
    def is_open(self):
        return self._ser is not None and self._ser.is_open

    @property
    def mode(self):
        return self._mode

    def open(self, port, baud=DEFAULT_BAUD):
        with self.lock:
            self.close()
            try:
                # 打开时**不要**动 DTR/RTS: 它们是复位/BOOT 控制脚,
                # 一动板子就重启, 会把正在跑的程序打断。
                self._ser = serial.Serial(port, baud, timeout=0.2,
                                          write_timeout=3.0)
            except Exception as e:
                raise SerialError("打开 %s 失败: %s" % (port, e))
            self._port = port
            self._baud = baud
            self._mode = "repl"
            time.sleep(0.15)
            self._ser.reset_input_buffer()
            return self._ser

    def close(self):
        with self.lock:
            if self._ser:
                try:
                    self._ser.close()
                except Exception:
                    pass
            self._ser = None
            self._port = None
            self._mode = "none"

    # ---- 读写 ----
    def write(self, data):
        if isinstance(data, str):
            data = data.encode("utf-8", "replace")
        with self.lock:
            if not self.is_open:
                raise SerialError("串口未连接")
            self._ser.write(data)
            self._ser.flush()

    def read_avail(self):
        with self.lock:
            if not self.is_open:
                return b""
            n = self._ser.in_waiting
            return self._ser.read(n) if n else b""

    def read_until(self, want, timeout=3.0, echo_sink=None):
        """读到出现 want (bytes) 为止。echo_sink: 收到中途数据时的回调。"""
        buf = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = self.read_avail()
            if chunk:
                buf += chunk
                if echo_sink:
                    echo_sink(chunk)
                if want in buf:
                    return buf
            else:
                time.sleep(0.01)
        return buf

    # ---- 模式切换 ----
    def interrupt(self):
        """Ctrl-C 中断正在运行的程序"""
        self.write(b"\x03\x03")

    def enter_raw(self, tries=5):
        """进 raw REPL。bPuppy 的 voice 模块 UART 回显会插进握手里, 需重试。"""
        for _ in range(tries):
            self.write(b"\x03")            # 先打断, 确保不在跑程序
            time.sleep(0.15)
            self.write(b"\x01")            # Ctrl-A
            got = self.read_until(b"raw REPL", timeout=1.5)
            if b"raw REPL" in got:
                self._mode = "raw"
                time.sleep(0.1)
                self.read_avail()
                return True
        raise SerialError("进 raw REPL 失败 (板子没响应, 检查端口/供电)")

    def enter_repl(self):
        """回普通 REPL"""
        self.write(b"\x02")                # Ctrl-B
        time.sleep(0.15)
        self._mode = "repl"

    def raw_exec(self, code, timeout=8.0):
        """在 raw REPL 里执行一段代码, 返回 (stdout, stderr)。

        raw REPL 的返回格式: OK<stdout>\\x04<stderr>\\x04>
        """
        if self._mode != "raw":
            self.enter_raw()
        self.read_avail()                  # 丢掉残留
        self.write(code.encode("utf-8") + b"\x04")

        buf = b""
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = self.read_avail()
            if chunk:
                buf += chunk
                if buf.rstrip().endswith(b"\x04>"):
                    break
            else:
                time.sleep(0.01)

        if not buf:
            raise SerialError("raw REPL 无响应 (超时 %.0fs)" % timeout)
        # 去掉开头的回显/杂讯, 从第一个 OK 开始
        i = buf.find(b"OK")
        if i < 0:
            raise SerialError("raw REPL 返回异常: %r" % buf[:120])
        body = buf[i + 2:]
        end = body.rfind(b"\x04>")
        if end >= 0:
            body = body[:end]
        parts = body.split(b"\x04")
        out = parts[0] if len(parts) > 0 else b""
        err = parts[1] if len(parts) > 1 else b""
        return (out.decode("utf-8", "replace"),
                err.strip().decode("utf-8", "replace"))


# ======================================================================
# 设备文件操作 (基于 raw REPL)
# ======================================================================

def dev_list(sm, path="/"):
    """列出设备目录 → [(名字, 大小或 None)]"""
    code = (
        "import os\n"
        "try:\n"
        "    ns = os.listdir(%r)\n"
        "except OSError as e:\n"
        "    print('ERR', e); ns = []\n"
        "for n in sorted(ns):\n"
        "    p = %r + ('/' if not %r.endswith('/') else '') + n\n"
        "    try:\n"
        "        st = os.stat(p)\n"
        "        print('%%s\\t%%d' %% (n, st[6]))\n"
        "    except OSError:\n"
        "        print('%%s\\t-1' %% n)\n"
    ) % (path, path, path)
    out, err = sm.raw_exec(code)
    if err and "ERR" in out:
        raise SerialError(out.strip())
    items = []
    for line in out.splitlines():
        line = line.strip()
        if not line:
            continue
        name, _, size = line.rpartition("\t")
        if not name:
            continue
        try:
            sz = int(size)
        except ValueError:
            sz = -1
        items.append((name, sz if sz >= 0 else None))
    return items


def dev_stat(sm, path):
    out, err = sm.raw_exec(
        "import os\ntry:\n    st=os.stat(%r)\n    print(st[6])\n"
        "except OSError as e:\n    print(-1)\n" % path)
    try:
        return int(out.strip().splitlines()[-1])
    except Exception:
        return -1


def dev_read(sm, path, chunk=768, on_progress=None):
    """读设备文件 → bytes。分块 hex 传输, 避免超出 raw REPL 缓冲。"""
    size = dev_stat(sm, path)
    if size < 0:
        raise SerialError("读不到 %s (不存在?)" % path)
    parts = []
    off = 0
    while off < size:
        n = min(chunk, size - off)
        code = ("import binascii\n"
                "f=open(%r,'rb'); f.seek(%d)\n"
                "print(binascii.hexlify(f.read(%d)).decode())\n"
                "f.close()\n") % (path, off, n)
        out, err = sm.raw_exec(code, timeout=6.0)
        hexs = "".join(out.split())
        if not hexs:
            raise SerialError("读 %s 偏移 %d 返回空" % (path, off))
        parts.append(bytes.fromhex(hexs))
        off += n
        if on_progress:
            on_progress(off, size)
    return b"".join(parts)


def dev_write(sm, path, data, chunk=768, on_progress=None):
    """写设备文件。base64 分块, 每块长度是 4 的倍数才能独立解码。"""
    b64 = base64.b64encode(data).decode()
    b64_chunk = chunk * 4 // 3
    b64_chunk -= b64_chunk % 4              # 必须是 4 的倍数
    sm.raw_exec("f=open(%r,'wb')\n" % path)
    try:
        total = len(b64)
        for i in range(0, total, b64_chunk):
            piece = b64[i:i + b64_chunk]
            code = ("import binascii\n"
                    "f.write(binascii.a2b_base64(%r))\n") % piece
            out, err = sm.raw_exec(code, timeout=6.0)
            if err:
                raise SerialError("写入失败: %s" % err.splitlines()[-1])
            if on_progress:
                on_progress(min(i + b64_chunk, total), total)
    finally:
        sm.raw_exec("f.close()")
    return len(data)


def dev_remove(sm, path):
    out, err = sm.raw_exec(
        "import os\nos.remove(%r)\nprint('OK')\n" % path)
    return "OK" in out


def dev_rename(sm, old, new):
    out, err = sm.raw_exec(
        "import os\nos.rename(%r, %r)\nprint('OK')\n" % (old, new))
    return "OK" in out


def dev_mkdir(sm, path):
    out, err = sm.raw_exec(
        "import os\ntry:\n    os.mkdir(%r)\n    print('OK')\n"
        "except OSError as e:\n    print('E', e)\n" % path)
    return "OK" in out


# ======================================================================
# 主界面
# ======================================================================

class App(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title(APP_TITLE)
        self.geometry("1080x760")
        self.minsize(900, 640)

        self.sm = SerialManager()
        self.msgq = queue.Queue()          # 工作线程 → UI
        self.busy = False
        self.local_dir = os.getcwd()
        self.dev_dir = "/"

        self._build_ui()
        self.after(60, self._drain)

        # esptool 日志钩子:
        # set_logger 会把单例的 __class__ 换掉, 所以先装好, 之后 esptool 所有
        # 输出都走 BridgeLogger。sink/progress 在烧写前动态注入即可。
        if EsptoolLogger:
            try:
                EsptoolLogger().set_logger(BridgeLogger())
            except Exception:
                pass

        self.refresh_ports()

    # ------------------------------------------------------------------
    # UI 构建
    # ------------------------------------------------------------------
    def _build_ui(self):
        st = ttk.Style()
        try:
            st.theme_use("vista")
        except Exception:
            pass

        # ---- 顶部: 连接栏 ----
        top = ttk.LabelFrame(self, text="连接", padding=8)
        top.pack(fill="x", padx=8, pady=(8, 4))

        ttk.Label(top, text="串口:").grid(row=0, column=0, sticky="w")
        self.cb_port = ttk.Combobox(top, width=22, state="readonly")
        self.cb_port.grid(row=0, column=1, padx=(4, 6))

        ttk.Button(top, text="刷新", width=6,
                   command=self.refresh_ports).grid(row=0, column=2)

        ttk.Label(top, text="芯片:").grid(row=0, column=3, padx=(16, 0), sticky="w")
        self.cb_chip = ttk.Combobox(
            top, width=14, state="readonly",
            values=["ESP32", "ESP32-S3", "ESP32-C3", "ESP32-C6"])
        self.cb_chip.current(1)
        self.cb_chip.grid(row=0, column=4, padx=(4, 6))
        ttk.Label(top, text="(烧写时按此选 ROM, 选错会连不上)",
                  foreground="#666").grid(row=0, column=5, sticky="w")

        self.btn_conn = ttk.Button(top, text="连接", width=8, command=self.on_connect)
        self.btn_conn.grid(row=0, column=6, padx=(16, 4))
        self.btn_disc = ttk.Button(top, text="断开", width=8,
                                   command=self.on_disconnect, state="disabled")
        self.btn_disc.grid(row=0, column=7)

        # ---- 中部: 选项卡 ----
        nb = ttk.Notebook(self)
        nb.pack(fill="both", expand=True, padx=8, pady=4)

        self.tab_flash = ttk.Frame(nb)
        self.tab_files = ttk.Frame(nb)
        nb.add(self.tab_flash, text="  烧写  ")
        nb.add(self.tab_files, text="  文件管理  ")
        self.nb = nb

        self._build_flash_tab()
        self._build_files_tab()

        # ---- 底部: REPL ----
        bottom = ttk.LabelFrame(self, text="REPL 终端", padding=6)
        bottom.pack(fill="both", expand=False, padx=8, pady=(4, 4))

        self.txt_repl = tk.Text(bottom, height=12, wrap="word",
                                bg="#1e1e1e", fg="#d4d4d4",
                                insertbackground="#d4d4d4",
                                font=("Consolas", 10))
        self.txt_repl.pack(fill="both", expand=True)
        self.txt_repl.configure(state="disabled")

        row = ttk.Frame(bottom)
        row.pack(fill="x", pady=(6, 0))
        ttk.Label(row, text="命令:").pack(side="left")
        self.ent_cmd = ttk.Entry(row, font=("Consolas", 10))
        self.ent_cmd.pack(side="left", fill="x", expand=True, padx=6)
        self.ent_cmd.bind("<Return>", lambda e: self.on_send())

        ttk.Button(row, text="发送", width=7, command=self.on_send).pack(side="left")
        ttk.Button(row, text="Ctrl+C", width=8,
                   command=self.on_interrupt).pack(side="left", padx=4)
        ttk.Button(row, text="清空", width=6,
                   command=self.clear_repl).pack(side="left", padx=4)
        self.btn_raw = ttk.Button(row, text="切到 Raw", width=10,
                                  command=self.on_toggle_raw)
        self.btn_raw.pack(side="left", padx=4)

        # ---- 状态栏 ----
        self.var_status = tk.StringVar(value="就绪")
        bar = ttk.Frame(self)
        bar.pack(fill="x", side="bottom")
        ttk.Separator(bar, orient="horizontal").pack(fill="x")
        ttk.Label(bar, textvariable=self.var_status,
                  anchor="w").pack(fill="x", padx=8, pady=3)

    # ---- 烧写页 ----
    def _build_flash_tab(self):
        p = self.tab_flash

        bar = ttk.Frame(p)
        bar.pack(fill="x", padx=8, pady=8)
        ttk.Button(bar, text="添加固件…", command=self.on_add_fw).pack(side="left")
        ttk.Button(bar, text="移除选中", command=self.on_del_fw).pack(side="left", padx=6)
        ttk.Button(bar, text="清空", command=self.on_clear_fw).pack(side="left")

        ttk.Label(bar, text="烧写波特率:").pack(side="left", padx=(20, 2))
        self.cb_fbaud = ttk.Combobox(
            bar, width=10, state="readonly",
            values=["115200", "230400", "460800", "921600"])
        self.cb_fbaud.set("921600")
        self.cb_fbaud.pack(side="left")

        cols = ("addr", "file", "size")
        self.tv_fw = ttk.Treeview(p, columns=cols, show="headings", height=8)
        self.tv_fw.heading("addr", text="地址")
        self.tv_fw.heading("file", text="固件文件")
        self.tv_fw.heading("size", text="大小")
        self.tv_fw.column("addr", width=100, anchor="center")
        self.tv_fw.column("file", width=560)
        self.tv_fw.column("size", width=110, anchor="e")
        self.tv_fw.pack(fill="both", expand=True, padx=8)
        self.tv_fw.bind("<Double-1>", self.on_edit_addr)

        note = ttk.Label(
            p, foreground="#a33", justify="left",
            text=("⚠ 「擦除」会清空整片 Flash —— 包括 NVS 里的舵机标定/几何参数。\n"
                  "   只想更新程序时, 取消勾选下面的「先擦除」, 直接烧写即可。"))
        note.pack(fill="x", padx=8, pady=(6, 0))

        run = ttk.Frame(p)
        run.pack(fill="x", padx=8, pady=8)
        self.var_erase = tk.BooleanVar(value=False)
        ttk.Checkbutton(run, text="先擦除整片 Flash",
                        variable=self.var_erase).pack(side="left")
        self.btn_flash = ttk.Button(run, text="擦除并烧写", command=self.on_flash)
        self.btn_flash.pack(side="right")

        self.pb = ttk.Progressbar(p, mode="determinate", maximum=100)
        self.pb.pack(fill="x", padx=8, pady=(0, 4))
        self.var_pb = tk.StringVar(value="等待操作")
        ttk.Label(p, textvariable=self.var_pb).pack(anchor="w", padx=8)

        self.txt_flash = tk.Text(p, height=9, wrap="word",
                                 bg="#f6f6f6", font=("Consolas", 9))
        self.txt_flash.pack(fill="both", expand=True, padx=8, pady=(4, 8))

    # ---- 文件管理页 ----
    def _build_files_tab(self):
        p = self.tab_files

        pan = ttk.PanedWindow(p, orient="horizontal")
        pan.pack(fill="both", expand=True, padx=8, pady=8)

        # 左: 本地
        left = ttk.LabelFrame(pan, text="本地文件", padding=6)
        pan.add(left, weight=1)

        lb = ttk.Frame(left)
        lb.pack(fill="x")
        self.var_local = tk.StringVar(value=self.local_dir)
        ttk.Entry(lb, textvariable=self.var_local).pack(
            side="left", fill="x", expand=True)
        ttk.Button(lb, text="…", width=3,
                   command=self.on_pick_dir).pack(side="left", padx=(4, 0))

        self.tv_local = ttk.Treeview(left, columns=("name", "size"),
                                     show="headings", selectmode="extended")
        self.tv_local.heading("name", text="名称")
        self.tv_local.heading("size", text="大小")
        self.tv_local.column("name", width=260)
        self.tv_local.column("size", width=90, anchor="e")
        self.tv_local.pack(fill="both", expand=True, pady=(6, 0))
        self.tv_local.bind("<Double-1>", self.on_local_open)

        # 中: 操作按钮
        mid = ttk.Frame(pan)
        pan.add(mid, weight=0)
        ttk.Label(mid, text="").pack(pady=30)
        ttk.Button(mid, text="上传 →", width=10,
                   command=self.on_upload).pack(pady=4)
        ttk.Button(mid, text="← 下载", width=10,
                   command=self.on_download).pack(pady=4)
        ttk.Separator(mid, orient="horizontal").pack(fill="x", pady=10)
        ttk.Button(mid, text="重命名", width=10,
                   command=self.on_rename).pack(pady=4)
        ttk.Button(mid, text="删除", width=10,
                   command=self.on_delete).pack(pady=4)
        ttk.Separator(mid, orient="horizontal").pack(fill="x", pady=10)
        ttk.Button(mid, text="刷新", width=10,
                   command=self.refresh_device).pack(pady=4)

        # 右: 设备
        right = ttk.LabelFrame(pan, text="设备 VFS", padding=6)
        pan.add(right, weight=1)

        rb = ttk.Frame(right)
        rb.pack(fill="x")
        self.var_dev = tk.StringVar(value="/")
        ttk.Entry(rb, textvariable=self.var_dev).pack(
            side="left", fill="x", expand=True)
        ttk.Button(rb, text="转到", width=5,
                   command=self.refresh_device).pack(side="left", padx=(4, 0))

        self.tv_dev = ttk.Treeview(right, columns=("name", "size"),
                                   show="headings", selectmode="extended")
        self.tv_dev.heading("name", text="名称")
        self.tv_dev.heading("size", text="大小")
        self.tv_dev.column("name", width=260)
        self.tv_dev.column("size", width=90, anchor="e")
        self.tv_dev.pack(fill="both", expand=True, pady=(6, 0))
        self.tv_dev.bind("<Double-1>", self.on_dev_open)

    # ------------------------------------------------------------------
    # 线程 → UI 消息泵
    # ------------------------------------------------------------------
    def post(self, kind, **kw):
        self.msgq.put((kind, kw))

    def _drain(self):
        try:
            while True:
                kind, kw = self.msgq.get_nowait()
                try:
                    self._handle(kind, kw)
                except Exception:
                    self.log_repl("[UI错误] " + traceback.format_exc())
        except queue.Empty:
            pass
        self.after(60, self._drain)

    def _handle(self, kind, kw):
        if kind == "repl":
            self.log_repl(kw["text"])
        elif kind == "repl_raw":
            self.log_repl(kw["text"], raw=True)
        elif kind == "flash_log":
            self.txt_flash.insert("end", kw["text"] + "\n")
            self.txt_flash.see("end")
        elif kind == "progress":
            cur, total = kw["cur"], kw["total"]
            pct = 100.0 * cur / total if total else 0
            self.pb["value"] = pct
            self.var_pb.set("进度 %.1f%%  (%d / %d)" % (pct, cur, total))
        elif kind == "status":
            self.var_status.set(kw["text"])
        elif kind == "pb_text":
            self.var_pb.set(kw["text"])
        elif kind == "busy":
            self.set_busy(kw["on"])
        elif kind == "connected":
            self._after_connect()
        elif kind == "disconnected":
            self._after_disconnect()
        elif kind == "files_dev":
            self._fill_dev(kw["items"])
        elif kind == "files_local":
            self._fill_local(kw["items"])
        elif kind == "refresh_local":
            self.refresh_local()
        elif kind == "msgbox":
            messagebox.showinfo(APP_TITLE, kw["text"])
        elif kind == "msgbox_err":
            messagebox.showerror(APP_TITLE, kw["text"])

    # ------------------------------------------------------------------
    # 小工具
    # ------------------------------------------------------------------
    def log_repl(self, text, raw=False):
        self.txt_repl.configure(state="normal")
        if raw:
            self.txt_repl.insert("end", text)
        else:
            self.txt_repl.insert("end", text + "\n")
        self.txt_repl.see("end")
        self.txt_repl.configure(state="disabled")

    def clear_repl(self):
        self.txt_repl.configure(state="normal")
        self.txt_repl.delete("1.0", "end")
        self.txt_repl.configure(state="disabled")

    def set_busy(self, on):
        self.busy = on
        state = "disabled" if on else "normal"
        for b in (self.btn_flash, self.btn_conn):
            b.configure(state=state)
        self.configure(cursor="watch" if on else "")

    def need_conn(self):
        if not self.sm.is_open:
            messagebox.showwarning(APP_TITLE, "请先连接设备")
            return False
        return True

    def run_bg(self, fn, *a, **kw):
        """把耗时操作丢到后台线程, 异常统一报给 UI"""
        def wrap():
            try:
                fn(*a, **kw)
            except Exception as e:
                self.post("msgbox_err", text="%s" % e)
                self.post("status", text="失败: %s" % e)
            finally:
                self.post("busy", on=False)
        self.set_busy(True)
        threading.Thread(target=wrap, daemon=True).start()

    # ------------------------------------------------------------------
    # 连接
    # ------------------------------------------------------------------
    def refresh_ports(self):
        ports = []
        for p in serial.tools.list_ports.comports():
            desc = p.description or ""
            ports.append("%s  (%s)" % (p.device, desc) if desc else p.device)
        self.cb_port["values"] = ports
        if ports and not self.cb_port.get():
            self.cb_port.current(0)
        self.var_status.set("检测到 %d 个串口" % len(ports))

    def _port_name(self):
        raw = self.cb_port.get().strip()
        return raw.split()[0] if raw else ""

    def on_connect(self):
        port = self._port_name()
        if not port:
            messagebox.showwarning(APP_TITLE, "没有可用串口")
            return

        def work():
            self.post("status", text="正在连接 %s ..." % port)
            sm = self.sm
            sm.open(port, REPL_BAUD)
            sm.interrupt()                      # 打断, 拿干净提示符
            time.sleep(0.3)
            sm.read_avail()
            sm.write(b"\r")
            greet = sm.read_until(b">>>", timeout=2.0)
            self.post("repl", text="—— 已连接 %s @%d ——" % (port, REPL_BAUD))
            if greet:
                self.post("repl_raw",
                          text=greet.decode("utf-8", "replace"))
            else:
                self.post("repl", text="(没看到 >>> 提示符; 若板子在跑程序请按 Ctrl+C)")
            self.post("connected")

        self.run_bg(work)

    def _after_connect(self):
        self.btn_conn.configure(state="disabled")
        self.btn_disc.configure(state="normal")
        self.var_status.set("已连接 %s (REPL)" % self.sm._port)
        self.btn_raw.configure(text="切到 Raw")
        self.refresh_device()
        self.refresh_local()

    def on_disconnect(self):
        try:
            if self.sm.is_open and self.sm.mode == "raw":
                self.sm.enter_repl()
        except Exception:
            pass
        self.sm.close()
        self._after_disconnect()

    def _after_disconnect(self):
        self.btn_conn.configure(state="normal")
        self.btn_disc.configure(state="disabled")
        self.var_status.set("已断开")
        self.tv_dev.delete(*self.tv_dev.get_children())

    # ------------------------------------------------------------------
    # REPL
    # ------------------------------------------------------------------
    def on_send(self):
        if not self.need_conn():
            return
        cmd = self.ent_cmd.get()
        self.ent_cmd.delete(0, "end")
        if not cmd.strip():
            return

        def work():
            sm = self.sm
            if sm.mode == "raw":
                out, err = sm.raw_exec(cmd)
                if out:
                    self.post("repl_raw", text=out)
                if err:
                    self.post("repl", text=err)
                return
            # 普通 REPL: 会回显输入, 只显示设备返回的部分
            sm.read_avail()
            sm.write(cmd.encode("utf-8") + b"\r")
            buf = b""
            last = time.time()
            deadline = time.time() + 3.0
            while time.time() < deadline:
                chunk = sm.read_avail()
                if chunk:
                    buf += chunk
                    last = time.time()
                elif buf and time.time() - last > 0.35:
                    break                  # 停了一会儿没新数据 → 认为输出完了
                else:
                    time.sleep(0.01)
            text = buf.decode("utf-8", "replace")
            # 去回显: 第一行通常是原样返回的输入
            lines = text.replace("\r\n", "\n").split("\n")
            if lines and lines[0].strip() == cmd.strip():
                lines = lines[1:]
            self.post("repl_raw", text=">>> " + cmd + "\n")
            self.post("repl_raw", text="\n".join(lines))

        self.run_bg(work)

    def on_interrupt(self):
        if not self.need_conn():
            return
        try:
            self.sm.interrupt()
            self.log_repl("\n[已发送 Ctrl+C]")
        except Exception as e:
            messagebox.showerror(APP_TITLE, str(e))

    def on_toggle_raw(self):
        if not self.need_conn():
            return
        try:
            if self.sm.mode == "raw":
                self.sm.enter_repl()
                self.btn_raw.configure(text="切到 Raw")
                self.var_status.set("普通 REPL 模式")
                self.log_repl("[已切回普通 REPL]")
            else:
                self.sm.enter_raw()
                self.btn_raw.configure(text="切回 REPL")
                self.var_status.set("raw REPL 模式 (文件操作)")
                self.log_repl("[已切到 raw REPL]")
        except Exception as e:
            messagebox.showerror(APP_TITLE, str(e))

    # ------------------------------------------------------------------
    # 文件: 本地
    # ------------------------------------------------------------------
    def refresh_local(self):
        d = self.var_local.get().strip() or os.getcwd()
        if not os.path.isdir(d):
            return
        self.local_dir = d
        items = []
        # 不在根目录时给一个"上级目录"入口 (双击 ../ 上去)
        up = os.path.dirname(os.path.abspath(d))
        if up and up != os.path.abspath(d):
            items.append(("../", None))
        try:
            for n in sorted(os.listdir(d), key=str.lower):
                full = os.path.join(d, n)
                if os.path.isdir(full):
                    items.append((n + "/", None))
                else:
                    try:
                        items.append((n, os.path.getsize(full)))
                    except OSError:
                        items.append((n, None))
        except OSError as e:
            self.log_repl("[本地目录读取失败] %s" % e)
        self.post("files_local", items=items)

    def _fill_local(self, items):
        self.tv_local.delete(*self.tv_local.get_children())
        for name, size in items:
            self.tv_local.insert("", "end", values=(
                name, "" if size is None else self.fmt_size(size)))

    def on_pick_dir(self):
        d = filedialog.askdirectory(initialdir=self.local_dir)
        if d:
            self.var_local.set(d)
            self.refresh_local()

    def on_local_open(self, _evt=None):
        sel = self.tv_local.selection()
        if not sel:
            return
        name = self.tv_local.item(sel[0], "values")[0]
        # 注意先判 "../": 它也以 "/" 结尾, 顺序反了会被当成要进去的目录
        if name == "../":
            self.var_local.set(os.path.dirname(os.path.abspath(self.local_dir)))
            self.refresh_local()
        elif name.endswith("/"):
            self.var_local.set(os.path.join(self.local_dir, name.rstrip("/")))
            self.refresh_local()

    # ------------------------------------------------------------------
    # 文件: 设备
    # ------------------------------------------------------------------
    def refresh_device(self):
        if not self.need_conn():
            return
        path = self.var_dev.get().strip() or "/"

        def work():
            self.post("status", text="读取设备目录 %s ..." % path)
            items = dev_list(self.sm, path)
            self.post("files_dev", items=items)
            self.post("status", text="设备目录 %s: %d 项" % (path, len(items)))

        self.run_bg(work)

    def _fill_dev(self, items):
        self.tv_dev.delete(*self.tv_dev.get_children())
        self.tv_dev.insert("", "end", values=("../", ""))
        for name, size in items:
            self.tv_dev.insert("", "end", values=(
                name, "" if size is None else self.fmt_size(size)))

    def on_dev_open(self, _evt=None):
        sel = self.tv_dev.selection()
        if not sel:
            return
        name = self.tv_dev.item(sel[0], "values")[0]
        if name == "../":
            cur = self.var_dev.get().strip().rstrip("/")
            self.var_dev.set(os.path.dirname(cur) or "/")
            self.refresh_device()

    def _dev_selected(self):
        sel = self.tv_dev.selection()
        return [self.tv_dev.item(s, "values")[0] for s in sel
                if self.tv_dev.item(s, "values")[0] != "../"]

    def _local_selected(self):
        sel = self.tv_local.selection()
        return [self.tv_local.item(s, "values")[0] for s in sel]

    # ------------------------------------------------------------------
    # 上传 / 下载 / 删除 / 重命名
    # ------------------------------------------------------------------
    def on_upload(self):
        if not self.need_conn():
            return
        names = self._local_selected()
        if not names:
            messagebox.showwarning(APP_TITLE, "先在左边选文件")
            return
        ddir = self.var_dev.get().strip().rstrip("/") or ""

        def work():
            for name in names:
                if name.endswith("/"):
                    continue
                src = os.path.join(self.local_dir, name)
                dst = ddir + "/" + name
                size = os.path.getsize(src)
                self.post("status", text="上传 %s (%d 字节)..." % (name, size))
                data = open(src, "rb").read()
                dev_write(self.sm, dst,
                          data,
                          on_progress=lambda c, t: self.post(
                              "pb_text", text="上传 %s: %.0f%%" % (name, 100.0 * c / t)))
                self.post("repl", text="✓ 上传 %s → %s" % (name, dst))
            self.post("status", text="上传完成")
            self.post("files_dev", items=dev_list(self.sm, self.var_dev.get()))

        self.run_bg(work)

    def on_download(self):
        if not self.need_conn():
            return
        names = self._dev_selected()
        if not names:
            messagebox.showwarning(APP_TITLE, "先在右边选文件")
            return
        ddir = self.var_dev.get().strip().rstrip("/") or ""

        def work():
            for name in names:
                src = ddir + "/" + name
                dst = os.path.join(self.local_dir, name)
                self.post("status", text="下载 %s ..." % name)
                data = dev_read(self.sm, src,
                                on_progress=lambda c, t: self.post(
                                    "pb_text", text="下载 %s: %.0f%%" % (name, 100.0 * c / t)))
                with open(dst, "wb") as f:
                    f.write(data)
                self.post("repl", text="✓ 下载 %s → %s (%d 字节)"
                          % (src, dst, len(data)))
            self.post("status", text="下载完成")
            self.post("refresh_local")          # 下载完刷新本地列表

        self.run_bg(work)

    def on_delete(self):
        if not self.need_conn():
            return
        names = self._dev_selected()
        if not names:
            messagebox.showwarning(APP_TITLE, "先在右边选文件")
            return
        if not messagebox.askyesno(
                APP_TITLE, "确认删除设备上的这 %d 个文件?\n%s"
                % (len(names), "\n".join(names))):
            return
        ddir = self.var_dev.get().strip().rstrip("/") or ""

        def work():
            for name in names:
                if dev_remove(self.sm, ddir + "/" + name):
                    self.post("repl", text="✓ 删除 %s" % name)
                else:
                    self.post("repl", text="✗ 删除失败 %s" % name)
            self.post("status", text="删除完成")
            self.post("files_dev", items=dev_list(self.sm, self.var_dev.get()))

        self.run_bg(work)

    def on_rename(self):
        if not self.need_conn():
            return
        names = self._dev_selected()
        if len(names) != 1:
            messagebox.showwarning(APP_TITLE, "请选中恰好一个文件")
            return
        old = names[0]
        dlg = tk.Toplevel(self)
        dlg.title("重命名")
        dlg.transient(self)
        dlg.grab_set()
        ttk.Label(dlg, text="新名称:").pack(padx=12, pady=(12, 4))
        ent = ttk.Entry(dlg, width=36)
        ent.pack(padx=12)
        ent.insert(0, old)
        ent.focus_set()

        def ok():
            new = ent.get().strip()
            dlg.destroy()
            if not new or new == old:
                return
            ddir = self.var_dev.get().strip().rstrip("/") or ""
            def work():
                if dev_rename(self.sm, ddir + "/" + old, ddir + "/" + new):
                    self.post("repl", text="✓ 重命名 %s → %s" % (old, new))
                else:
                    self.post("repl", text="✗ 重命名失败")
                self.post("files_dev", items=dev_list(self.sm, self.var_dev.get()))
            self.run_bg(work)

        ent.bind("<Return>", lambda e: ok())
        ttk.Button(dlg, text="确定", command=ok).pack(pady=10)

    # ------------------------------------------------------------------
    # 烧写
    # ------------------------------------------------------------------
    def on_add_fw(self):
        paths = filedialog.askopenfilenames(
            title="选择固件 (.bin)",
            filetypes=[("固件", "*.bin"), ("所有文件", "*.*")],
            initialdir=self.local_dir)
        have = {self.tv_fw.item(i, "values")[1]
                for i in self.tv_fw.get_children()}
        for p in paths:
            if p in have:
                continue
            addr = guess_addr(p)
            self.tv_fw.insert("", "end", values=(
                "0x%X" % addr, p, self.fmt_size(os.path.getsize(p))))

    def on_del_fw(self):
        for i in self.tv_fw.selection():
            self.tv_fw.delete(i)

    def on_clear_fw(self):
        self.tv_fw.delete(*self.tv_fw.get_children())

    def on_edit_addr(self, _evt=None):
        sel = self.tv_fw.selection()
        if not sel:
            return
        item = sel[0]
        cur = self.tv_fw.item(item, "values")[0]

        dlg = tk.Toplevel(self)
        dlg.title("修改烧写地址")
        dlg.transient(self)
        dlg.grab_set()
        ttk.Label(dlg, text="地址 (十六进制, 如 0x10000):").pack(padx=12, pady=(12, 4))
        ent = ttk.Entry(dlg, width=24)
        ent.pack(padx=12)
        ent.insert(0, cur)
        ent.focus_set()

        def ok():
            txt = ent.get().strip()
            dlg.destroy()
            try:
                v = int(txt, 16)
                self.tv_fw.set(item, "addr", "0x%X" % v)
            except ValueError:
                messagebox.showerror(APP_TITLE, "地址格式不对: %s" % txt)

        ent.bind("<Return>", lambda e: ok())
        ttk.Button(dlg, text="确定", command=ok).pack(pady=10)

    def on_flash(self):
        if esptool is None:
            messagebox.showerror(APP_TITLE, "缺少 esptool:  pip install esptool")
            return
        rows = self.tv_fw.get_children()
        if not rows:
            messagebox.showwarning(APP_TITLE, "先添加固件文件")
            return
        addr_data = []
        for i in rows:
            addr_s, path, _ = self.tv_fw.item(i, "values")
            if not os.path.isfile(path):
                messagebox.showerror(APP_TITLE, "文件不存在: %s" % path)
                return
            addr_data.append((int(addr_s, 16), path))

        port = self._port_name()
        if not port:
            messagebox.showwarning(APP_TITLE, "没有可用串口")
            return
        do_erase = self.var_erase.get()
        chip = self.cb_chip.get()
        baud = int(self.cb_fbaud.get())

        self.txt_flash.delete("1.0", "end")
        self.pb["value"] = 0

        def log(t):
            self.post("flash_log", text=t)

        def work():
            # ---- 串口互斥: esptool 要自己开串口, 先让出 ----
            was_connected = self.sm.is_open
            if was_connected:
                try:
                    if self.sm.mode == "raw":
                        self.sm.enter_repl()
                except Exception:
                    pass
                self.sm.close()
                log("[已释放串口供烧写使用]")

            BridgeLogger.sink = log
            BridgeLogger.progress = lambda c, t: self.post("progress", cur=c, total=t)

            try:
                log("连接芯片 (%s @ %d)..." % (chip, baud))
                with esptool.cmds.detect_chip(port, baud,
                                              connect_mode="default-reset") as esp:
                    log("芯片: %s" % esp.CHIP_NAME)
                    if do_erase:
                        log("擦除整片 Flash ...")
                        self.post("pb_text", text="擦除中…")
                        esptool.cmds.erase_flash(esp, force=True)
                        log("擦除完成")
                    log("烧写 %d 个文件 ..." % len(addr_data))
                    esptool.cmds.write_flash(esp, addr_data, force=True)
                    log("校验 ...")
                    esptool.cmds.verify_flash(esp, addr_data)
                    log("复位 ...")
                    esptool.cmds.reset_chip(esp, "hard-reset")
                log("✓ 烧写完成")
                self.post("pb_text", text="烧写完成")
                self.post("status", text="烧写完成")
            finally:
                BridgeLogger.sink = None
                BridgeLogger.progress = None
                # ---- 恢复 REPL 连接 ----
                if was_connected:
                    try:
                        time.sleep(1.2)
                        self.sm.open(port, REPL_BAUD)
                        self.sm.interrupt()
                        time.sleep(0.3)
                        self.sm.read_avail()
                        log("[串口已重新打开]")
                    except Exception as e:
                        log("[串口重开失败: %s]" % e)

        self.run_bg(work)

    # ------------------------------------------------------------------
    @staticmethod
    def fmt_size(n):
        if n is None:
            return ""
        for u in ("B", "K", "M"):
            if n < 1024:
                return "%d %s" % (n, u) if u == "B" else "%.1f %s" % (n, u)
            n /= 1024.0
        return "%.1f G" % n


def main():
    if esptool is None:
        print("警告: 没装 esptool, 烧写功能不可用 (pip install esptool)")
    app = App()
    app.mainloop()


if __name__ == "__main__":
    main()
