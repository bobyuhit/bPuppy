# 查询 voice 线程 + UART2 状态
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
print(cmd('import voice; print("voice._started =", voice._started)').decode('utf-8', errors='replace'))
print(cmd('import bpuppy_uart; print("uart any =", bpuppy_uart.any())').decode('utf-8', errors='replace'))
s.close()
