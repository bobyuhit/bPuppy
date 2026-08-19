# 定位 GPIO19/GPIO20: 交替翻转 (500ms), 供万用表扫板子排针
# p19 高时 p20 低, 反之亦然; 找到相位相反、半秒互换的两个脚 = GPIO19 与 GPIO20
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

# 释放 UART2 引脚 (配成普通 GPIO 输出)
print(">> 释放 UART2, GPIO19/20 配成输出交替翻转")
s.write(b'import bpuppy_uart; bpuppy_uart.stop()\r\n')
wait_prompt(3)

code = (
    "import machine, time, _thread\n"
    "_loc_stop = False\n"
    "p19 = machine.Pin(19, machine.Pin.OUT)\n"
    "p20 = machine.Pin(20, machine.Pin.OUT)\n"
    "def _flip():\n"
    "    while not _loc_stop:\n"
    "        p19.value(1); p20.value(0)\n"
    "        time.sleep_ms(500)\n"
    "        p19.value(0); p20.value(1)\n"
    "        time.sleep_ms(500)\n"
    "_thread.start_new_thread(_flip, ())\n"
    "print('LOCATE: GPIO19/20 交替翻转中 (500ms)')\n"
)
s.write(b'\x05')
time.sleep(0.2)
s.write(code.encode())
s.write(b'\x04')

buf = b''
end = time.time() + 8
while time.time() < end:
    chunk = s.read(256)
    if chunk:
        buf += chunk
        if b'LOCATE' in buf:
            break
    else:
        time.sleep(0.2)
print(buf.decode('utf-8', errors='replace'))
if b'LOCATE' in buf:
    print("== 定位模式已启动。")
    print("== 万用表 DC 档: 黑笔接 GND, 红笔扫板子排针, 找两个引脚:")
    print("==   GPIO19: 0.5s 3.3V → 0.5s 0V 循环")
    print("==   GPIO20: 与 GPIO19 相位相反 (19=3.3V 时 20=0V)")
    print("== 定位结束运行 tools/stop_locate.py 停止, 再恢复 UART2")
else:
    print("!! 未确认启动")
s.close()
