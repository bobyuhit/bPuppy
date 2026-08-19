# 周期性向 UART2 TX(GPIO19) 发送数据, 供外部测量 (示波器/逻辑分析仪/万用表/串口监听)
# 每 1 秒发一帧 AA 55 70 01 55 AA, 共 20 秒
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

# 确保 UART2 ready (幂等)
s.write(b'import bpuppy_uart; bpuppy_uart.init(2,19,20,9600)\r\n')
wait_prompt(3)

print(">> 开始周期性发送 (20 秒, 每 1s 一帧 AA 55 70 01 55 AA)...")
print(">> 测量点: GPIO19 (UART2 TX) 对 GND。空闲=高电平, 发送期=波形/低脉冲")
code = (
    "import bpuppy_uart, time\n"
    "frm = b'\\xAA\\x55\\x70\\x01\\x55\\xAA'\n"
    "print('PERIO START 20s')\n"
    "t0 = time.ticks_ms(); n = 0\n"
    "while time.ticks_diff(time.ticks_ms(), t0) < 20000:\n"
    "    bpuppy_uart.send(frm); n += 1\n"
    "    time.sleep_ms(950)\n"
    "    if n % 10 == 0:\n"
    "        print('PERIO sent', n)\n"
    "print('PERIO DONE', n)\n"
)
# MicroPython paste mode: Ctrl+E 进入, 粘代码, Ctrl+D 执行
s.write(b'\x05')
time.sleep(0.2)
s.write(code.encode())
s.write(b'\x04')
buf = b''
end = time.time() + 25
while time.time() < end:
    chunk = s.read(256)
    if chunk:
        buf += chunk
        if b'PERIO DONE' in buf:
            break
    else:
        time.sleep(0.1)

text = buf.decode('utf-8', errors='replace')
print(text)
import re
n = re.findall(r'PERIO DONE (\d+)', text)
print("== 发送完成: 共发 %s 帧 AA 55 70 01 55 AA ==" % (n[0] if n else '?'))
s.close()
