# REPL 测试 voice.say() 上行帧 (安全, 不动狗)
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

# 进入 REPL prompt
s.write(b'\r\n')
wait_prompt(2)

# 1) 检查 voice 模块状态
s.write(b'import voice; print("voice._started =", voice._started)\r\n')
out = wait_prompt(5)
print(out.decode('utf-8', errors='replace'))

# 2) 发送狗叫2号 (AA 55 70 01 55 AA)
print(">> voice.say(0x70, 1)  -> 应发 AA 55 70 01 55 AA")
s.write(b'voice.say(0x70, 1); print("say OK")\r\n')
out = wait_prompt(5)
print(out.decode('utf-8', errors='replace'))

# 3) 测试事件转发: 注册回调 + 手动 _dispatch(0x30) (不动狗, 只验证信号路径)
s.write(b'voice.on_cmd(0x30, lambda: print("EVT 0x30 fired")); voice._dispatch(0x30)\r\n')
out = wait_prompt(5)
print(out.decode('utf-8', errors='replace'))

s.close()
