# GPIO 物理连通性自测: GPIO20(输出) 驱动 GPIO19(输入), 验证短接是否真的通
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

def cmd(c, t=4):
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

print(">> 停 voice 线程 + 释放 UART2 引脚")
print(cmd('import voice; voice.stop()').decode('utf-8', errors='replace'))
print(cmd('import bpuppy_uart; bpuppy_uart.stop()').decode('utf-8', errors='replace'))

print(">> GPIO20 配输出, GPIO19 配输入, 拉高/拉低对比")
out = cmd(
    "import machine; "
    "tx=machine.Pin(20, machine.Pin.OUT); "
    "rx=machine.Pin(19, machine.Pin.IN); "
    "tx.value(1); v1=rx.value(); "
    "tx.value(0); v0=rx.value(); "
    "print('RESULT tx=1 rx=%d, tx=0 rx=%d' % (v1, v0))"
).decode('utf-8', errors='replace')
print(out)

print(">> 恢复 UART2 (stop 后重新 init)")
print(cmd('bpuppy_uart.init(2,19,20,9600)').decode('utf-8', errors='replace'))

s.close()
