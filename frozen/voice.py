"""
bPuppy 语音控制 — Hiwonder CI-33T 语音识别/发声模块 (UART2, 9600)

⚠ 设计原则 (2026-08-19, 2026-09-27 修订): 本模块对语音指令**默认只转发事件, 不做任何动作**。
   动作交由 KittenBlock 程序完成 (事件积木 → 用户编程), 固件 py 代码不动作。

   **唯一例外: 0x3F「播报电压」** —— 收到该指令时固件**内置**播报电量百分比
   (见 _say_batt_pct)。破例原因: 这个动作是"把固件自己采到的电池数据念出来",
   属于固件自身状态的播报, 不是运动/姿态动作; 交给用户程序做的话还得让用户
   自己拼电量计算 —— 而电压的唯一真源是 C 层, Python 抄不到。
   ⚠ 因为是**无条件**内置: 用户程序里若再写一个「当收到 [播报电压] 指令」的事件
   积木, 固件播报和用户动作**会同时触发** (听感上"报两次")。要避免就只用其中一个。

接线 (2026-08-19, UART2 引脚反转 TX=GPIO19 / RX=GPIO20):
    CI-33T PA2 (UART1_TX) ──→ GPIO20 (UART2 RX)   语音指令进 ESP32
    CI-33T PA3 (UART1_RX) ←── GPIO19 (UART2 TX)   ESP32 发指令给模块
    VCC 5V 外部供电, GND 共地

⚠ GPIO19/20 = ESP32-S3 原生 USB 引脚 (D-/D+), 且已归 UART2 用 (UART_TX=19, UART_RX=20):
   MicroPython 组件默认启用 TinyUSB (mpy_startup.c 的 usb_init()),
   会初始化 USB-OTG PHY 接管 GPIO19/20 → UART2 TX 发不出、machine.Pin 无效。
   已在 components/mr9you__micropython-helper/mpy_startup.c 注释掉 usb_init(),
   把两个脚让给 UART2。USB-CDC 虚拟串口因此不可用, 不是可以打开的功能 ——
   想恢复 USB 串口就得放弃本模块的 UART2 (REPL/烧录走 UART0=COM14, 不受影响)。

协议 (2026-09-26 改):
  上行 (本系统 → CI-33T): AA 55 <CMD> <PARAM> 55 AA   (发声/反馈, 6 字节)

  下行 (CI-33T → 本系统): 4 字节定长帧, 两种, 靠**首字节取值范围**区分 ——
    命令帧:  BB <CMD> <PARAM> EE    CMD = 0x30-0x3F (运动/姿态), PARAM 预留
    角度帧:  <角度> 00 00 00        声源角度 (DOA), 0-180 度
                                    ⚠ 模块侧格式固定, 本仓库不能改

  判据: 首字节 0xBB = 命令帧; <= 0xB4 (=180) = 角度帧。两段取值不重叠
  (0xBB=187 > 180), 所以**不可能互相误判**。既不改角度帧格式, 也彻底
  根除了旧版逐字节扫 0x30-0x3C 时"角度 48-60 度被误当成运动指令"的 bug。

  收不够 4 字节就留在缓冲里等下一轮 (_parse), 不做任何猜测 —— 因此
  字节被 UART 切分/粘连都不影响解析。

用法:
    import voice            # 上电默认: import 即启动 (UART2 + 后台线程)
    voice.play('汪汪')      # 播放预置声音 (汪汪/嘤嘤, 映射见 SND_WANG/SND_YING)
    voice.say(0x70, 1)      # 发狗叫声 1 号 (AA 55 70 01 55 AA)
    voice.say_num(73)       # 播报数字 73 (AA 55 72 49 55 AA, 0-100 越界钳位)
    voice.SoundAngle        # 最近一次声源角度 (度, 0-180); -1 = 开机后还没收到过
    voice.on_cmd(0x30, fn)  # 注册回调: 收到停止指令时执行 fn (KittenBlock 事件积木用)
    voice.stop()            # 停止 (后台线程退出, 下次 start 可重启)

  ★ 收到「播报电压」(0x3F) 时固件会**自动**给 CI-33T 发「播报数字」报电量百分比
    (来自 voltage.read_pct(), 7.4V=100% / 6.6V=0% / 读不到=0)。不需要任何用户代码。
    前提是平台侧给 CI-33T 的【串口输入】配好 AA 55 72 <数据> 55 AA 的 0-100 词条,
    否则模块收到帧也没声音可放 —— 见 docs/README.md:595。

KittenBlock「语音」组事件积木 (2026-08-19 新增, 2026-09-26 加声音角度,
    2026-09-27 加「点头」/「播报电压」):
    事件积木生成末尾函数 def voiceWhenX(): (X = Stop/Fwd/Back/.../SoundDir),
    本模块后台线程扫描 __main__ 全局按名字 (voiceWhenX → 命令码)
    自动注册为事件回调 → 收到指令只触发用户程序, 固件自身不做动作。
    例外只有一个: 0x3F「播报电压」有内置动作 (见文件头原则 + _say_batt_pct),
    voiceWhenVolt 事件**照常**能注册 —— 两者会同时生效。

    声音角度做成**事件 + 变量**两件套 (用户要求):
      事件「当收到 [声音角度] 指令」→ def voiceWhenSoundDir()
      变量「(声音角度)」          → voice.SoundAngle
    ⚠ 声源持续存在时模块会**连发**角度帧, 所以这个事件会连续触发 ——
      要"一有声音就响应"用事件, 要"持续跟随声源"轮询 SoundAngle 变量。
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

# ---- 下行帧封装 (2026-09-26 新增) ----
# 命令帧 BB <CMD> <PARAM> EE。0xBB/0xEE 都 > 180(0xB4), 与角度帧首字节
# 取值范围不重叠 —— 这是两种帧唯一且充分的区分依据。
# ⚠ 命名带 _CMD_: 上面的 FRAME_HEAD/FRAME_TAIL 是**上行**的 bytes 常量,
#   这两个是**下行**的单字节 int, 别混。
_FRAME_CMD_HEAD = 0xBB
_FRAME_CMD_TAIL = 0xEE

# 声源角度上限 (度)。角度帧首字节 <= 它 —— 用于和命令帧区分。
ANGLE_MAX = 180         # = 0xB4

# 角度事件的内部派发编号 —— 只在本模块内用, **不上线**。
# 刻意选在单字节命令码之外 (命令帧 CMD 是 1 字节, 现有 0x30-0x3F,
# 以后还会往后扩); 用 0x100 保证永不与任何线上 CMD 冲突。
CMD_SOUND_DIR = 0x100

# ---- 下行: 运动/姿态命令 (数据区第一字节, 仅作事件信号, 不触发动作) ----
CMD_STOP   = 0x30   # 停止
CMD_FWD    = 0x31   # 前进
CMD_BACK   = 0x32   # 后退
CMD_LEFT   = 0x33   # 左转
CMD_RIGHT  = 0x34   # 右转
CMD_FASTER = 0x35   # 加速
CMD_SLOWER = 0x36   # 减速
CMD_NOD    = 0x37   # 点头 (原「跳跃」; 跳跃步态 2026-09-27 已从固件删除, 码位复用)
CMD_STAND  = 0x38   # 站立 (姿态)
CMD_CROUCH = 0x39   # 蹲下
CMD_SIT    = 0x3A   # 坐下
CMD_WAVE   = 0x3B   # 摇手
CMD_PLAY   = 0x3C   # 邀玩
CMD_VOLT   = 0x3F   # 播报电压 ★ 有内置动作: 固件自动播报电量百分比 (_say_batt_pct)
                    #   (0x3D/0x3E 暂空, 留着以后用)

# ---- 上行: 发声/反馈命令 (数据区第一字节) ----
SND_BARK = 0x70     # 狗叫声类 (第 4 字节 = 声音编号, 实测可用)
SND_TTS  = 0x71     # 平台自定义发声段 (预留, 当前没有声音用它)
SND_NUM  = 0x72     # 播报数字 (第 4 字节 = 数字本身, 范围见 NUM_MIN/NUM_MAX)

# 播报数字的取值范围 (KittenBlock「播报数字 [NUM]」滑块也是 0-100)。
# 越界**钳位**到边界并打印一行 —— 不静默: 平台侧只会为 0-100 配词条,
# 发个 200 过去那边匹配不上, 听感上就是"什么都没发生"。
NUM_MIN = 0
NUM_MAX = 100

# ---- 预置声音 (KittenBlock「狗叫 [汪汪/嘤嘤]」积木) ----
# 狗叫类**第一字节统一 0x70, 靠第 4 字节区分具体声音**。以后加新声音、或平台侧
# 调了某个声音的编号, 只动这里的 (0x70, n) + 扩展下拉一行 (kblock.json5 的
# soundMenu), 积木 pycode 不用动。
# 实测确认 (2026-09-27 更新):
#   0x70 0x01 = 汪汪   (2026-08-19 实测)
#   0x70 0x02 = 嘤嘤   (2026-09-27 由 0x71 0x02 改来 —— 两个声音同属 0x70 狗叫类)
SND_WANG = (0x70, 0x01)     # 汪汪
SND_YING = (0x70, 0x02)     # 嘤嘤

# 命令帧 CMD 字节的合法范围。2026-09-26 前是"逐字节扫描段", 现在是
# **合法性校验**: 命令帧除帧头/帧尾外, 中间两个字节也得对得上才算数。
_CMD_MIN = 0x30
_CMD_MAX = 0x3F     # 0x3D/0x3E 暂未使用, 但已落在合法区间内 (收下后无事件, 静默)

# ---- 声源角度 (KittenBlock 变量积木「(声音角度)」读它) ----
# 初值 -1 而非 0: 0 度是**有效方向** (用户确认), 用 0 做初值的话用户没法
# 区分"正前方"和"开机后一次都没收到过"。
SoundAngle = -1

_started = False

# ---- 主脚本全局 dict (事件积木函数所在作用域) ----
# 实测 (2026-08-19): 本固件 MicroPython 的 sys.modules['__main__'] 为 None,
# 不能用它找 voiceWhen* 函数。由 frozen main.py / KittenBlock afterConnect
# 显式传入 globals() (主脚本/REPL 共享的全局 dict), 扫描事件函数用它。
_MAIN_GLOBALS = None

def set_main_globals(g):
    """传入主脚本全局 dict (frozen main.py 或 afterConnect 调用: voice.set_main_globals(globals()))"""
    global _MAIN_GLOBALS
    _MAIN_GLOBALS = g

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
    'voiceWhenNod':    CMD_NOD,
    'voiceWhenStand':  CMD_STAND,
    'voiceWhenCrouch': CMD_CROUCH,
    'voiceWhenSit':    CMD_SIT,
    'voiceWhenWave':   CMD_WAVE,
    'voiceWhenPlay':   CMD_PLAY,
    'voiceWhenVolt':   CMD_VOLT,
    # 声源角度 (2026-09-26): value 不是线上命令码, 是内部派发编号 CMD_SOUND_DIR。
    # 角度值靠 SoundAngle 变量带出去 —— 14 条既有回调的 fn() 签名保持不变。
    'voiceWhenSoundDir': CMD_SOUND_DIR,
}

def _scan_events():
    """扫描主脚本全局 (frozen main.py exec 用户程序 / REPL 的作用域),
    把 voiceWhen* 函数注册为对应命令码的事件回调 (幂等, 后台线程周期性调用)。
    只注册不调用 → 用户函数体不会在开机时执行一次。
    全局 dict 来源: set_main_globals() (frozen main.py/afterConnect) > sys.modules['__main__']。"""
    try:
        g = _MAIN_GLOBALS
        if g is None:
            import sys
            m = sys.modules.get('__main__')
            g = m.__dict__ if m is not None else None
        if g is None:
            return
        for fname, cmd in _EVT_FUNCS.items():
            fn = g.get(fname)
            if fn is not None and callable(fn) and fn not in _handlers.get(cmd, []):
                on_cmd(cmd, fn)
                print("voice: event 0x%02x -> %s" % (cmd, fname))
    except Exception:
        pass

# ================================================================
# 后台轮询线程
# ================================================================

_buf = b''

def _parse():
    """按结构切帧。**收不够 4 字节就原样留着等下一轮, 绝不猜。**

    两种帧靠首字节取值范围区分 (见文件头):
      0xBB            → 命令帧 BB <CMD> <PARAM> EE (CMD 还要在 0x30-0x3F 内)
      <= ANGLE_MAX    → 角度帧 <角度> 00 00 00
    都不是 → 丢掉 1 个字节重同步 (这样夹在中间的噪声能自动滑过去)。

    旧版是逐字节扫 0x30-0x3C, 角度 48-60 度会被误当运动指令; 现在按帧结构
    判定, 从根上没有了这个 bug。顺带: 帧被 UART 切分/粘连也不影响 ——
    切开了就等字节到齐, 粘连了 while 循环自然逐个切出来。
    """
    global _buf
    while len(_buf) >= 4:
        b0 = _buf[0]
        if b0 == _FRAME_CMD_HEAD and _buf[3] == _FRAME_CMD_TAIL \
                and _CMD_MIN <= _buf[1] <= _CMD_MAX:
            _dispatch(_buf[1], _buf[2])         # 命令帧
            _buf = _buf[4:]
        elif b0 <= ANGLE_MAX and _buf[1:4] == b'\x00\x00\x00':
            _dispatch(CMD_SOUND_DIR, b0)        # 角度帧 → 存 SoundAngle + 触发事件
            _buf = _buf[4:]
        else:
            _buf = _buf[1:]                     # 重同步
    # 兜底清理。正常情况上面 while 每轮要么消费 4 字节要么丢 1 字节, 到这里
    # 必 <4 字节 —— 所以这段几乎不会触发, 只在异常路径攒下垃圾时兜底。
    # ⚠ 必须放在**解析之后**: 放前面会从头部截断, 把残帧自己的帧头切掉。
    if len(_buf) > 64:
        _buf = _buf[-3:]        # 3 = 最长可能的残帧前缀 (4 字节帧差 1 字节)

def _pump():
    global _buf
    while _started:
        try:
            _scan_events()          # 注册 KittenBlock 事件积木函数 (voiceWhen*)
            if bpuppy_uart.any():
                data = bpuppy_uart.read(64)
                if data:
                    # 调试: 打印本次收到的原始字节 (可能是半帧 —— 拼起来才判定),
                    # 保留原有的 VOICE RX 日志格式便于对照实测。
                    print("VOICE RX: %s" % data.hex())
                    _buf += data
            _parse()
        except Exception:
            pass
        time.sleep_ms(20)

def _say_batt_pct():
    """内置: 收到「播报电压」(0x3F) → 给 CI-33T 发「播报数字」报电量百分比。

    ⚠ 本模块唯一的内置动作 —— 其余指令仍然只转发事件, 见文件头原则。
    跑在语音后台线程里 (_pump → _parse → _dispatch)。
    不加锁: say_num → bpuppy_uart.send 底层是**一次** uart_write_bytes
    (uart_driver.c:54), IDF 带驱动互斥量 ⇒ 6 字节帧整帧发出, 不会跟主线程里
    KittenBlock 调的 say_num 交错。

    读不到数据时 batt_pct() 是 0, 按用户定的**照样播报 0**, 但多打一行日志 ——
    否则 PWM_EXT 抢了 GPIO3 这种情况在板上完全看不出来 (听到的就是个 0)。
    """
    try:
        import bpuppy_led
        if bpuppy_led.batt_v() < 0.0:
            print("voice: 播报电压 —— 电池读数无效 (ADC 未跑 / PWM_EXT 抢了 GPIO3), 按 0 播报")
        say_num(bpuppy_led.batt_pct())
    except Exception as e:
        print("voice: 播报电压失败: %s" % e)

def _dispatch(cmd, param=0):
    """转发事件信号给用户回调, 固件自身不做任何动作 —— 唯一例外见 _say_batt_pct。

    param 目前只有角度帧用。CMD_SOUND_DIR 时**先存变量再触发事件** ——
    顺序很重要: 用户事件函数体里读 voice.SoundAngle 得读到本次的值。
    存变量这一步不依赖有没有注册回调, 所以「(声音角度)」积木单独用也能读到值。

    回调签名保持 fn() 不变 —— 角度靠变量带出去, 不给 14 条既有回调加参数。
    """
    global SoundAngle
    if cmd == CMD_SOUND_DIR:
        SoundAngle = param
    if cmd == CMD_VOLT:
        # ★ 全模块唯一的内置动作 (见文件头原则)。**无条件**执行 —— 放在事件
        #   派发之前: 没注册 voiceWhenVolt 时也要播报 (空板说"播报电压"得有反应);
        #   注册了则固件播报 + 用户动作都会跑 (用户已知并接受, 见文件头)。
        _say_batt_pct()
    handlers = _handlers.get(cmd)
    if not handlers:
        return
    if cmd == CMD_SOUND_DIR:
        # 角度单独一行: 0x100 不在线上, 打成 "VOICE CMD: 0x100" 会让人对着协议表找不到。
        # 也便于日志里区分"运动指令"和"声源角度"(角度是连发的, 别把 VOICE CMD 淹了)。
        print("VOICE ANGLE: %d -> event" % param)
    else:
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

def say_num(n):
    """上行: 播报数字 → 帧 AA 55 72 <n> 55 AA (KittenBlock「播报数字 [NUM]」积木走这里)。

    滑块本身已限 0-100, 这里再兜一道 —— REPL 手敲、变量传进来都可能越界。
    越界钳位到边界并打印; 非数字直接忽略并打印, 不抛异常 (后台线程/积木调用都不能炸)。
    """
    try:
        v = int(round(float(n)))
    except Exception:
        print("voice: say_num 参数不是数字: %r" % (n,))
        return
    if v < NUM_MIN or v > NUM_MAX:
        clamped = NUM_MIN if v < NUM_MIN else NUM_MAX
        print("voice: say_num %d 超出 %d-%d, 已钳位到 %d" % (v, NUM_MIN, NUM_MAX, clamped))
        v = clamped
    say(SND_NUM, v)

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
