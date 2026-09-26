# UART2 loopback 测试: GPIO19(TX) 短接 GPIO20(RX)
# 2026-09-26 扩展: 除上行帧回显外, 增加**裸下行帧注入** —— 不依赖 CI-33T 模块
#                  就能验证新的按结构切帧解析 (_parse) 和声源角度 (SoundAngle)。
#
# ⚠ 注入命令帧 (BB <CMD> <PARAM> EE) 会真的派发事件: 若用户程序里注册了
#   voiceWhenFwd 等回调, 狗会动。测前请把狗**架空** (或先删掉 /main.py)。
#   角度帧不会动狗, 只会写 SoundAngle。
#
# ⚠ 跑之前确认 CI-33T 没在发数据 (别对麦克风说话), 否则会和注入的字节混进
#   同一个帧缓冲 _buf, 打乱下面的分段注入用例。
import sys, time, re
import serial

PORT = 'COM14'
BAUD = 115200

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

s = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(1)
s.reset_input_buffer()


def wait_prompt(timeout=5):
    buf = b''
    end = time.time() + timeout
    while time.time() < end:
        chunk = s.read(256)
        if chunk:
            buf += chunk
            if b'>>>' in buf:
                return buf
    return buf


def drain(seconds=0.8):
    """抓一段串口输出并解码"""
    buf = b''
    end = time.time() + seconds
    while time.time() < end:
        chunk = s.read(512)
        if chunk:
            buf += chunk
        else:
            time.sleep(0.05)
    return buf.decode('utf-8', errors='replace')


def repl(cmd, wait=0.8):
    """在 REPL 里执行一行, 返回这段输出"""
    s.write(cmd.encode() + b'\r\n')
    return drain(wait)


def inject(raw, wait=0.8):
    """把裸字节直接灌进 UART2 的发送口 → 短接 → 被 _pump 当"下行"读回"""
    hexs = raw.hex()
    print(f"  注入 {hexs}")
    s.write(f"bpuppy_uart.send(bytes.fromhex('{hexs}')); print('injected')\r\n".encode())
    return drain(wait)


def rxs(text):
    """提取这段输出里的 VOICE RX 帧"""
    return re.findall(r'VOICE RX: (\S+)', text)


def cmds(text):
    """提取这段输出里派发过的运动指令, 返回可读字符串"""
    return ', '.join(re.findall(r'VOICE CMD: \S+', text)) or '无'


def angles(text):
    """提取这段输出里派发过的声源角度 (固件打成 VOICE ANGLE: <度> -> event)"""
    return ', '.join(re.findall(r'VOICE ANGLE: (\d+)', text)) or '无'


def angle_now():
    """读板上当前的 voice.SoundAngle"""
    out = repl("print('SoundAngle=%d' % voice.SoundAngle)")
    m = re.search(r'SoundAngle=(-?\d+)', out)
    return int(m.group(1)) if m else None


s.write(b'\r\n')
wait_prompt(2)

# ============================================================
# A. 上行帧回显 (TX 通路) —— 注意: 上行帧**不应**再派发任何命令
# ============================================================
print("=" * 60)
print("A. 上行帧回显 (验证 TX 通路 + 确认上行帧不会被误判成下行)")
for cat, code in ((0x70, 0), (0x70, 1), (0x31, 0)):
    print(f">> voice.say(0x{cat:02x}, 0x{code:02x})  ->  应发 AA 55 {cat:02x} {code:02x} 55 AA")
    repl(f'voice.say(0x{cat:02x}, {code}); print("sent")')

up_text = drain(1.0)
print("  收到的 VOICE RX:", rxs(up_text))
expected = {'aa55700055aa', 'aa55700155aa', 'aa55310055aa'}
got = set(rxs(up_text))
print("  匹配预期上行帧:", got & expected or "无")
# 关键回归: 上行帧里含 0x31, 旧解析会误派发"前进"; 新解析必须一条都不派发
print("  误派发的命令:", cmds(up_text), "(应为 无)")

# ============================================================
# B. 下行命令帧
# ============================================================
print("=" * 60)
print("B. 命令帧 BB <CMD> <PARAM> EE")
t = inject(b'\xBB\x31\x00\xEE')
print("  RX:", rxs(t))
print("  派发:", cmds(t), "(期望 0x31; 若为'无'检查是否注册了 voiceWhenFwd)")

# ============================================================
# C. 角度帧
# ============================================================
print("=" * 60)
print("C. 角度帧 <角度> 00 00 00")
for ang in (120, 0, 180):
    t = inject(bytes((ang, 0, 0, 0)))
    val = angle_now()
    print("   注入 %d 度 -> SoundAngle=%s %s; VOICE ANGLE=%s; 误派发: %s (应为 无)"
          % (ang, val, "OK" if val == ang else "❌ 不符", angles(t), cmds(t)))

# ============================================================
# D. 撞码回归 (本次核心) —— 旧解析会把这些角度误报成运动指令
# ============================================================
print("=" * 60)
print("D. 撞码回归: 角度 48-60 度落在旧扫描段 0x30-0x3C 内, 绝不能派发")
for ang in (0x30, 0x35, 0x3C):
    t = inject(bytes((ang, 0, 0, 0)))
    val = angle_now()
    print("   角度 %d (0x%02x) -> SoundAngle=%s %s; 误派发: %s (应为 无)"
          % (ang, ang, val, "OK" if val == ang else "❌ 不符", cmds(t)))

# ============================================================
# E. 切分回归: 一帧分两次到
# ============================================================
print("=" * 60)
print("E. 切分回归: 一帧分两次注入, 应等拼齐后才派发")
inject(b'\xBB\x31', 0.4)
t = inject(b'\x00\xEE', 0.8)
print("    拼齐后派发:", cmds(t), "(期望 0x31, 无 = ❌ 丢帧了)")

# ============================================================
# F. 重同步回归: 前面夹垃圾字节
# ============================================================
print("=" * 60)
print("F. 重同步回归: 垃圾字节 + 正常帧, 垃圾应被滑过")
t = inject(b'\x01\x02\xBB\x31\x00\xEE')
print("    派发:", cmds(t), "(期望 0x31, 无 = ❌ 重同步失败)")
# 垃圾字节 0x01/0x02 不该被当成角度帧 (角度 1 度)
val = angle_now()
print("    SoundAngle 未被垃圾污染:", "OK" if val != 1 else "❌ 垃圾被误判成角度 1")

# ============================================================
print("=" * 60)
print("结论: A 段应看到 3 条上行帧回显且 0 条误派发;")
print("      B/E/F 应派发 0x31; C/D 应逐条对上 SoundAngle 且 0 条误派发。")
print("⚠ D 段若出现 VOICE CMD: 0x30/0x35/0x3c 说明解析退回了旧版逐字节扫描。")
print("ℹ 固件对角度事件打的是 VOICE ANGLE: <度> -> event, 不用 VOICE CMD 前缀")
print("  (0x100 不在线上, 混在 VOICE CMD 里会对着协议表找不到)。")
s.close()
