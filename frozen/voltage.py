"""
bPuppy 电池电压检测 + WS2812 指示灯 (上电默认模块)

开机后默认检测电池电压, 通过 GPIO48 WS2812 显示 (颜色随电压):
    ≥7.4V 蓝常亮 | 渐变紫 | ≤6.6V 红常亮 | <6.4V 红闪烁

换算与阈值与 C 层 led_driver.c 一致 (同一套标定系数)。

用法:
    import voltage          # 上电默认: import 即自动启动
    v = voltage.read_v()    # 已标定电压 (V)
    voltage.stop()          # 停止监控 (LED 熄灭)
    voltage.start()         # 重新启动 (幂等)
"""

import bpuppy_adc

# 标定 (mpy_modules/batt.py 最小二乘拟合, 与 led_driver.c 常量一致)
_CAL_A = 1.0379     # 实际 = a × 显示 + b
_CAL_B = 0.4660
_DIVIDER = 6.1      # 51k/10k 分压换算: read_mv() × 6.1 / 1000 → 显示值 V

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
    """已标定电池电压 (V); ADC 未就绪返回 -1"""
    mv = bpuppy_adc.read_mv()
    if mv < 0:
        return -1.0
    v = mv * _DIVIDER / 1000.0
    return _CAL_A * v + _CAL_B


start()   # 上电默认: import 即自动启动
