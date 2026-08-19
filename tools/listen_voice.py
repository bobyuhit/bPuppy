# 监听 60 秒: 打印 voice 模块收到的 VOICE RX / 执行的 VOICE CMD
# 用法: 脚本运行期间对 CI-33T 麦克风说话, 观察输出
import sys, time
import serial

PORT = 'COM14'
BAUD = 115200
SECONDS = 60
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

s = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(1)
s.reset_input_buffer()

# 进入 REPL prompt
s.write(b'\r\n')
buf = b''
end = time.time() + 5
while time.time() < end:
    chunk = s.read(256)
    if chunk:
        buf += chunk
        if b'>>>' in buf:
            break
    else:
        time.sleep(0.05)

print(f"== 监听开始 ({SECONDS}s). 请对 CI-33T 麦克风说话 ==")
print("== 预期看到 VOICE RX: <hex> (下行帧) 与 VOICE CMD: 0x.. (触发的命令) ==")
print("==" + "=" * 50)

t0 = time.time()
buf = b''
while time.time() - t0 < SECONDS:
    chunk = s.read(256)
    if chunk:
        buf += chunk
        # 实时打印 VOICE 相关行
        text = chunk.decode('utf-8', errors='replace')
        for line in text.splitlines():
            l = line.strip()
            if l and ('VOICE' in l or 'voice' in l or 'Traceback' in l or 'error' in l):
                print(f"  [{time.strftime('%H:%M:%S')}] {l}")
    else:
        time.sleep(0.1)

print("== 监听结束 ==")
s.close()
