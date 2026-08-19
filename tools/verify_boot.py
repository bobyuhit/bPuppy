# 软复位并抓取启动日志, 验证 voice.py 上电自启
import sys, time
import serial

PORT = 'COM14'
BAUD = 115200

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

def try_open(retries=5):
    for i in range(retries):
        try:
            s = serial.Serial(PORT, BAUD, timeout=1)
            return s
        except Exception as e:
            print(f"  打开失败 (第{i+1}次): {e}")
            time.sleep(2)
    return None

s = try_open()
if not s:
    print("FATAL: 无法打开 COM14")
    sys.exit(1)

time.sleep(1)

# 清空输入缓冲
s.reset_input_buffer()

# 触发软复位: Ctrl+D (0x04)
print(">> 发送 Ctrl+D 软复位")
s.write(b'\x04')
s.flush()

boot = b''
deadline = time.time() + 15
while time.time() < deadline:
    chunk = s.read(1024)
    if chunk:
        boot += chunk
        # 出现 Ready 或 >>> 说明启动完成
        if b'Ready' in boot or b'>>>' in boot:
            time.sleep(0.5)
            boot += s.read(2048)
            break

try:
    text = boot.decode('utf-8', errors='replace')
except Exception:
    text = repr(boot)

print("=" * 60)
print("启动日志:")
print("=" * 60)
print(text)
print("=" * 60)

# 关键字检查
checks = [
    ("voice 自启", "voice: CI-33T ready"),
    ("UART2 ready", "UART2 ready"),
    ("Ready 待命", "Ready."),
]
print("\n== 验证结果 ==")
ok = True
for name, kw in checks:
    hit = kw in text
    print(f"  [{'OK' if hit else 'FAIL'}] {name}  ({kw!r})")
    if not hit:
        ok = False
for bad in ("voice: .*error", "Traceback", "Fatal error", "Guru Meditation", "CONFLICT!"):
    import re
    if re.search(bad, text):
        print(f"  [WARN] 检测到错误模式: {bad}")
        ok = False

print("\n结论:", "通过 — voice 上电自启正常" if ok else "存在问题, 需检查")
s.close()
