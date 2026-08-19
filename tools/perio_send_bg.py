# 后台线程周期性向 UART2 TX(GPIO19) 发送 AA 55 70 01 55 AA, 每 1s 一帧
# 供外部测量; REPL 保持响应, 用户测完可让脚本停掉线程
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
s.write(b'import bpuppy_uart; bpuppy_uart.init(2,19,20,9600)\r\n')
wait_prompt(3)

print(">> 启动后台发送线程 (每 1s 一帧 AA 55 70 01 55 AA)...")
print(">> 测量点: GPIO19 (UART2 TX) 对 GND。空闲=高电平, 发送期=波形/低脉冲")
code = (
    "import bpuppy_uart, time, _thread\n"
    "_voice_tx_stop = False\n"
    "def _voice_tx():\n"
    "    frm = b'\\xAA\\x55\\x70\\x01\\x55\\xAA'\n"
    "    n = 0\n"
    "    while not _voice_tx_stop:\n"
    "        bpuppy_uart.send(frm); n += 1\n"
    "        time.sleep_ms(950)\n"
    "    print('TX thread stopped, total frames =', n)\n"
    "_thread.start_new_thread(_voice_tx, ())\n"
    "print('TX thread started (每1s一帧)')\n"
)
s.write(b'\x05')
time.sleep(0.2)
s.write(code.encode())
s.write(b'\x04')

# 确认线程已启动 (5 秒窗口内抓 TX thread started), 然后退出
# 线程在 ESP 上独立持续发送, 不依赖本进程; 测完用 tools/stop_tx.py 停止
print(">> 确认线程启动...")
buf = b''
end = time.time() + 8
while time.time() < end:
    chunk = s.read(256)
    if chunk:
        buf += chunk
        if b'TX thread started' in buf:
            break
    else:
        time.sleep(0.2)
text = buf.decode('utf-8', errors='replace')
print(text)
if 'TX thread started' in text:
    print("== 发送线程已启动 (每1s一帧 AA 55 70 01 55 AA)。请测量 GPIO19 (UART2 TX) 对 GND。")
    print("== 测完运行 tools/stop_tx.py 停止线程 ==")
else:
    print("!! 未确认线程启动, 请检查")
s.close()
