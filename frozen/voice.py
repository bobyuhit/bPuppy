"""
bPuppy 语音控制 — Hiwonder CI-33T 语音识别/发声模块 (UART2, 9600)

⚠ 设计原则 (2026-08-19): 本模块对语音指令**只转发事件, 不做任何动作**。
   动作全部交由 KittenBlock 程序完成 (事件积木 → 用户编程), 固件 py 代码不动作。

接线 (2026-08-19, UART2 引脚反转 TX=GPIO19 / RX=GPIO20):
    CI-33T PA2 (UART1_TX) ──→ GPIO20 (UART2 RX)   语音指令进 ESP32
    CI-33T PA3 (UART1_RX) ←── GPIO19 (UART2 TX)   ESP32 发指令给模块
    VCC 5V 外部供电, GND 共地

⚠ GPIO19/20 = ESP32-S3 原生 USB 引脚 (D-/D+):
   MicroPython 组件默认启用 TinyUSB (mpy_startup.c 的 usb_init()),
   会初始化 USB-OTG PHY 接管 GPIO19/20 → UART2 TX 发不出、machine.Pin 无效。
   已在 components/mr9you__micropython-helper/mpy_startup.c 注释掉 usb_init() 释放。
   代价: USB-CDC 虚拟串口不可用 (REPL/烧录走 UART0=COM14, 不受影响)。

2 字节数据区协议 <CMD> <PARAM>:
  上行 (本系统 → CI-33T): AA 55 <CMD> <PARAM> 55 AA   (发声/反馈)
  下行 (CI-33T → 本系统): 数据区命令码 0x30-0x3C (运动/姿态)
                           实测 (2026-08-19) = 裸 2 字节, 无帧头帧尾
                           → 打印 VOICE RX hex + 宽容扫描命令码转发事件

用法:
    import voice            # 上电默认: import 即启动 (UART2 + 后台线程)
    voice.play('汪汪')      # 播放预置声音 (汪汪/嘤嘤, 映射见 SND_WANG/SND_YING)
    voice.say(0x70, 1)      # 发狗叫声 2 号 (AA 55 70 01 55 AA)
    voice.on_cmd(0x30, fn)  # 注册回调: 收到停止指令时执行 fn (KittenBlock 事件积木用)
    voice.stop()            # 停止 (后台线程退出, 下次 start 可重启)

KittenBlock「语音」组事件积木 (2026-08-19 新增):
    事件积木生成末尾函数 def voiceWhenX(): (X = Stop/Fwd/Back/...),
    本模块后台线程扫描 __main__ 全局按名字 (voiceWhenX → 命令码)
    自动注册为事件回调 → 收到指令只触发用户程序, 固件自身不做动作。
"""

import time
import _thread
import bpuppy_uart

# ---- 串口 ----
UART_NUM = 2
# UART2 引脚约定 (2026-08-19 起): TX=GPIO19, RX=GPIO20
# 原因: CI-33T 实际接线 PA2(UART1_TX)→GPIO20、PA3(UART1_RX)←GPIO19,
#       为交叉对接 (发对收), 故 ESP32 侧把 TX 配在 GPIO19、RX 配在 GPIO20。
#   CI-33T PA2(TX) → GPIO20 (UART2 RX)    PA3(RX) ← GPIO19 (UART2 TX)
UART_TX  = 19
UART_RX  = 20
BAUD     = 9600

# ---- 线上帧封装 (上行) ----
FRAME_HEAD = b'\xAA\x55'
FRAME_TAIL = b'\x55\xAA'

# ---- 下行: 运动/姿态命令 (数据区第一字节, 仅作事件信号, 不触发动作) ----
CMD_STOP   = 0x30   # 停止
CMD_FWD    = 0x31   # 前进
CMD_BACK   = 0x32   # 后退
CMD_LEFT   = 0x33   # 左转
CMD_RIGHT  = 0x34   # 右转
CMD_FASTER = 0x35   # 加速
CMD_SLOWER = 0x36   # 减速
CMD_JUMP   = 0x37   # 跳跃
CMD_STAND  = 0x38   # 站立 (姿态)
CMD_CROUCH = 0x39   # 蹲下
CMD_SIT    = 0x3A   # 坐下
CMD_WAVE   = 0x3B   # 摇手
CMD_PLAY   = 0x3C   # 邀玩

# ---- 上行: 发声/反馈命令 (数据区第一字节) ----
SND_BARK = 0x70     # 狗叫声类 (PARAM: 0=1号, 1=2号, ...; 实测可用)
SND_TTS  = 0x71     # 平台自定义发声段 (实测 2026-08-19: 0x01 = 嘤嘤)
SND_EXT  = 0x72     # 预留: 其他反馈

# ---- 预置声音 (KittenBlock「语音播放汪汪/嘤嘤」) ----
# 实测确认 (2026-08-19): 0x70 0x01 = 汪汪, 0x71 0x01 = 嘤嘤。
# 若平台固件另有映射, 改下面常量即可, voice.play / 积木代码不用动。
SND_WANG = (0x70, 0x01)     # 汪汪 (用户实测确认: 0x70 1 能响)
SND_YING = (0x71, 0x01)     # 嘤嘤 (用户实测确认)

# 下行命令码范围 (宽容扫描用; 帧头帧尾 AA/55/5A/A5 均不在段内, 不误触发)
_CMD_MIN = 0x30
_CMD_MAX = 0x3C

_started = False

# ================================================================
# 事件回调注册表 — 只转发信号, 不做动作 (2026-08-19)
# ================================================================
# 收到下行命令码 → 触发用户回调 (KittenBlock「语音」组事件积木)。
# 固件侧无任何内置动作: 命令来了要么触发用户函数, 要么什么都不做。

_handlers = {}    # cmd -> [fn, ...]

def on_cmd(cmd, fn):
    """注册回调: 收到命令码 cmd 时调用 fn() (幂等, 重复注册只保留一份)"""
    lst = _handlers.setdefault(cmd, [])
    if fn not in lst:
        lst.append(fn)

def off_cmd(cmd, fn=None):
    """注销回调: fn=None 时清空该命令码的全部回调"""
    if cmd not in _handlers:
        return
    if fn is None:
        del _handlers[cmd]
    else:
        _handlers[cmd] = [f for f in _handlers[cmd] if f is not fn]

# KittenBlock 事件积木函数名 → 命令码 (扫描 __main__ 自动注册)
_EVT_FUNCS = {
    'voiceWhenStop':   CMD_STOP,
    'voiceWhenFwd':    CMD_FWD,
    'voiceWhenBack':   CMD_BACK,
    'voiceWhenLeft':   CMD_LEFT,
    'voiceWhenRight':  CMD_RIGHT,
    'voiceWhenFaster': CMD_FASTER,
    'voiceWhenSlower': CMD_SLOWER,
    'voiceWhenJump':   CMD_JUMP,
    'voiceWhenStand':  CMD_STAND,
    'voiceWhenCrouch': CMD_CROUCH,
    'voiceWhenSit':    CMD_SIT,
    'voiceWhenWave':   CMD_WAVE,
    'voiceWhenPlay':   CMD_PLAY,
}

def _scan_events():
    """扫描 __main__ 全局 (frozen main.py exec 用户程序的作用域),
    把 voiceWhen* 函数注册为对应命令码的事件回调 (幂等, 后台线程周期性调用)。
    只注册不调用 → 用户函数体不会在开机时执行一次。"""
    try:
        import sys
        mod = sys.modules.get('__main__')
        if mod is None:
            return
        for fname, cmd in _EVT_FUNCS.items():
            fn = getattr(mod, fname, None)
            if fn is not None and callable(fn) and fn not in _handlers.get(cmd, []):
                on_cmd(cmd, fn)
                print("voice: event 0x%02x -> %s" % (cmd, fname))
    except Exception:
        pass

# ================================================================
# 后台轮询线程
# ================================================================

def _pump():
    while _started:
        try:
            _scan_events()          # 注册 KittenBlock 事件积木函数 (voiceWhen*)
            if bpuppy_uart.any():
                data = bpuppy_uart.read(16)
                if data:
                    # 调试: 打印原始帧, 便于摸清 CI-33T 下行封装格式
                    print("VOICE RX: %s" % data.hex())
                    for b in data:
                        if _CMD_MIN <= b <= _CMD_MAX:
                            _dispatch(b)
        except Exception:
            pass
        time.sleep_ms(20)

def _dispatch(cmd):
    """只转发事件信号给用户回调, 固件自身不做任何动作"""
    handlers = _handlers.get(cmd)
    if not handlers:
        return
    print("VOICE CMD: 0x%02x -> event" % cmd)
    for fn in handlers:
        try:
            fn()
        except Exception as e:
            print("voice: event 0x%02x error: %s" % (cmd, e))

# ================================================================
# 对外接口
# ================================================================

def say(category, code):
    """上行: 发送发声/反馈指令. say(0x70, 1) = 狗叫声 2 号 = 汪汪"""
    try:
        bpuppy_uart.send(FRAME_HEAD + bytes((category, code)) + FRAME_TAIL)
    except Exception as e:
        print("voice: say error: %s" % e)

def play(name):
    """按名字播放预置声音: play('汪汪') / play('嘤嘤')。映射见 SND_WANG/SND_YING。"""
    m = {'汪汪': SND_WANG, '嘤嘤': SND_YING}
    cat, code = m.get(name, SND_WANG)
    say(cat, code)

def start():
    """启动 UART2(9600) + 后台轮询线程 (幂等)"""
    global _started
    if _started:
        return
    bpuppy_uart.init(UART_NUM, UART_TX, UART_RX, BAUD)
    _started = True
    _thread.start_new_thread(_pump, ())
    print("voice: CI-33T ready  UART2 %d baud  (AA 55 <cmd> <param> 55 AA)" % BAUD)

def stop():
    """停止后台轮询线程 (下次 start 可重启)"""
    global _started
    _started = False


start()   # 上电默认: import 即启动
