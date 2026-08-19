# 手动诊断 UART2 loopback: 绕开 voice 线程, 直接测 bpuppy_uart 收发
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

def cmd(c, t=3):
    s.write((c + '\r\n').encode())
    buf = b''
    end = time.time() + t
    while time.time() < end:
        chunk = s.read(256)
        if chunk:
            buf += chunk
            if b'>>>' in buf:
                return buf
        else:
            time.sleep(0.05)
    return buf

s.write(b'\r\n')
wait_prompt(2)

# 0) 停掉 voice 线程, 避免抢数据
print(">> voice.stop()")
print(cmd('import voice; voice.stop()').decode('utf-8', errors='replace'))

# 1) 重新初始化 UART2 (确保干净)
print(">> bpuppy_uart.init(2,19,20,9600)")
print(cmd('import bpuppy_uart; bpuppy_uart.init(2,19,20,9600)').decode('utf-8', errors='replace'))

# 2) 手动发一帧 (TX GPIO19 发出)
print(">> send AA 55 70 01 55 AA")
s.write(b"bpuppy_uart.send(b'\\xAA\\x55\\x70\\x01\\x55\\xAA')\r\n")
time.sleep(0.3)

# 3) 读回看有没有 loopback 回显
print(">> any() + read()")
print(cmd('print("any=", bpuppy_uart.any()); d = bpuppy_uart.read(16); print("read=", d.hex() if d else None)').decode('utf-8', errors='replace'))

# 4) 连续发3次再读 (RX 也许需要等)
print(">> 连发3帧再读")
s.write(b"for _ in range(3): bpuppy_uart.send(b'\\xAA\\x55\\x70\\x00\\x55\\xAA')\r\n")
time.sleep(0.5)
print(cmd('print("any=", bpuppy_uart.any()); d = bpuppy_uart.read(16); print("read=", d.hex() if d else None)').decode('utf-8', errors='replace'))

s.close()
