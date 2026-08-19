# 停止周期性发送线程 (_voice_tx_stop = True)
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
s.write(b'_voice_tx_stop = True\r\n')
time.sleep(1)
buf = s.read(512)
print(buf.decode('utf-8', errors='replace'))
print("== 已请求停止线程 ==")
s.close()
