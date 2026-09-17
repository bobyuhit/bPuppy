"""
bPuppy 电池电压检测 + WS2812 指示灯 (上电默认模块)

开机后默认检测电池电压, 通过 GPIO48 WS2812 显示 (颜色随电压):
    ≥7.4V 蓝常亮 | 渐变紫 | ≤6.6V 红常亮 | <6.4V 红闪烁

标定系数与颜色阈值都在 C 层 led_driver.c —— 本模块**不做任何换算**:
read_v() 直接返回 C 缓存的同一个值。所以 LED 显示的颜色和这里读到的电压
必然一致 (同一条代码路径), 不会出现"两套系数算出两个电压"。

用法:
    import voltage          # 上电默认: import 即自动启动
    v = voltage.read_v()    # 已标定电压 (V); 未就绪返回 -1.0
    voltage.stop()          # 停止监控 (LED 熄灭)
    voltage.start()         # 重新启动 (幂等)

改标定 (换板子后重新采集, 不用重编译固件):
    import batt; batt.verify(); batt.apply_to_board()     # 见 mpy_modules/batt.py
"""

import bpuppy_adc
import bpuppy_led

_started = False


def start():
    """启动电压检测 + WS2812 指示灯 (幂等)"""
    global _started
    if not _started:
        bpuppy_adc.init()    # ADC + GPIO48 WS2812 电池指示灯一并激活
        _started = True


def stop():
    """停止监控 (LED 熄灭)"""
    global _started
    if _started:
        bpuppy_adc.stop()
        _started = False


def read_v():
    """已标定电池电压 (V); 监控未跑或 ADC 未就绪返回 -1.0"""
    return bpuppy_led.batt_v()


start()   # 上电默认: import 即自动启动
