# 测试上行发声 (2026-08-19 实测确认):
#   - voice.play('汪汪') → AA 55 70 01 55 AA   (0x70 0x01 = 汪汪, 用户确认)
#   - voice.play('嘤嘤') → AA 55 71 01 55 AA   (0x71 0x01 = 嘤嘤, 用户确认)
#   - voice.say(0x70, 0/1)                     狗叫声 1号/2号
#   - voice.say(0x71, 0)                       平台自定义发声段 0 (探测)
# 用法: python tools/say_test.py [COM14]
import sys, time
import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else 'COM14'
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

tests = [
    ("voice.play('汪汪')   → AA 55 70 01 55 AA (已确认: 汪汪)", "voice.play('汪汪')"),
    ("voice.play('嘤嘤')   → AA 55 71 01 55 AA (已确认: 嘤嘤)", "voice.play('嘤嘤')"),
    ("voice.say(0x70, 0)   → AA 55 70 00 55 AA (狗叫1号)", "voice.say(0x70, 0)"),
    ("voice.say(0x70, 1)   → AA 55 70 01 55 AA (狗叫2号=汪汪)", "voice.say(0x70, 1)"),
    ("voice.say(0x71, 0)   → AA 55 71 00 55 AA (发声段0 探测)", "voice.say(0x71, 0)"),
    ("voice.say(0x71, 1)   → AA 55 71 01 55 AA (发声段1=嘤嘤)", "voice.say(0x71, 1)"),
]

for desc, code in tests:
    print(">> %s" % desc)
    s.write(('%s; print("sent")\r\n' % code).encode())
    time.sleep(2)   # 留出发声时间
    buf = s.read(512)
    print(buf.decode('utf-8', errors='replace'))

print("== 完成. 若映射与标注不符, 改 frozen/voice.py 的 SND_WANG/SND_YING ==")
s.close()
