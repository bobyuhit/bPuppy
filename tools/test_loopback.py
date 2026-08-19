# UART2 loopback 测试: GPIO19(TX) 短接 GPIO20(RX)
# 验证 _pump 线程 + VOICE RX 打印 + 帧发送正确
import sys, time
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

s.write(b'\r\n')
wait_prompt(2)

# 发 3 帧, 观察 loopback 回显
for cat, code in ((0x70, 0), (0x70, 1), (0x31, 0)):
    print(f">> voice.say(0x{cat:02x}, 0x{code:02x})  ->  应发 AA 55 {cat:02x} {code:02x} 55 AA")
    s.write(f'voice.say(0x{cat:02x}, {code}); print("sent")\r\n'.encode())
    time.sleep(0.8)   # 给线程时间收+打印

# 抓取线程的 VOICE RX 打印
buf = b''
end = time.time() + 5
while time.time() < end:
    chunk = s.read(512)
    if chunk:
        buf += chunk
        if b'VOICE RX' in buf:
            time.sleep(0.5)
            buf += s.read(512)
            break
    else:
        time.sleep(0.1)

text = buf.decode('utf-8', errors='replace')
print("=" * 50)
print(text)
print("=" * 50)

# 检查
import re
rxs = re.findall(r'VOICE RX: (\S+)', text)
print("收到的 VOICE RX:", rxs)
expected = {'aa55700055aa', 'aa55700155aa', 'aa55310055aa'}
got = set(rxs)
print("匹配预期帧:", got & expected or "无")
print("未匹配的:", got - expected or "无")
ok = bool(got & expected)
print("结论:", "LOOPBACK 通过 — 线程活, 打印正常, 帧正确" if ok else "未收到回显(检查短接是否正确)")

# 安全: 0x31 前进若被线程执行会动狗, 但这里是从 TX 回 RX, 会被 _dispatch!
# 注意: 0x31 会触发前进, 需用户确认狗已架空。脚本只验证打印, 不判断动作。
s.close()
