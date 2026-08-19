# 停止 GPIO19/20 翻转定位线程, 并恢复 UART2
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
s.write(b'_loc_stop = True\r\n')
time.sleep(0.8)
s.write(b'import bpuppy_uart; bpuppy_uart.init(2,19,20,9600)\r\n')
wait_prompt(3)
buf = s.read(512)
print(buf.decode('utf-8', errors='replace'))
print("== 已停止翻转, UART2 已恢复 (RX=20 TX=19 9600) ==")
s.close()
