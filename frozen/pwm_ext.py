"""
pwm_ext.py — PWM_EXT1/2/3 扩展舵机的**底层封装**

只做三件事:
  1. 编号映射: PWM_EXTn (1/2/3) → MCPWM 通道 (0/1/2) + GPIO (3/47/48)
  2. 按需初始化: 第一次用到某路时自动 init; EXT1/EXT3 与电池检测同脚, 先停 ADC
  3. 角度读写的薄封装 (立即生效, 不缓动)

⚠ **本模块不含缓动。** 平滑逼近统一在 poses.py 里, 主舵机和扩展舵机共用同一套
   实现 —— 缓动算法跟"驱动哪个舵机"无关, 只跟"读角度/写角度"有关。
   要平滑移动请用:
       import poses
       poses.set_servo(8|9|10, deg)     # 8/9/10 = PWM_EXT1/2/3
       poses.set_step(3)
       poses.commit()

用法 (裸控制):
    import pwm_ext
    pwm_ext.set_angle(2, 90)     # PWM_EXT2 → 90°, 立即生效 (硬切)
    pwm_ext.get_angle(2)         # 读当前角度
    pwm_ext.get_pulse_us(2)      # 读当前脉宽
    pwm_ext.is_on(2)             # 该路是否已启用 (不触发初始化)
    pwm_ext.off(2)               # 释放该路

⚠ 按需自动初始化: PWM_EXT1(GPIO3) / PWM_EXT3(GPIO48) 与电池检测**同脚**,
   会自动先调 bpuppy_adc.stop()。副作用: 电池电压读数和 WS2812 指示灯停掉,
   直到手动 bpuppy_adc.init() 恢复。PWM_EXT2(GPIO47) 是空闲脚, 无副作用。
"""

import bpuppy_pwm_ext

# PWM_EXTn → GPIO (板上固定接线, 见 docs/硬件连接.md)
_PIN = {1: 3, 2: 47, 3: 48}
# 与电池检测同脚的两路 — 用之前必须先停 ADC, 否则固件仍在驱动/读同一个脚
_NEED_ADC_STOP = (1, 3)
_MAX = 3


def _chk(n):
    """PWM_EXT 编号 (1~3) → MCPWM 通道号 (0~2)"""
    n = int(n)
    if n < 1 or n > _MAX:
        raise ValueError("PWM_EXT 编号必须是 1~%d, 收到 %s" % (_MAX, n))
    return n - 1


def ensure(n):
    """按需初始化该路 (幂等); 返回 MCPWM 通道号"""
    ch = _chk(n)
    if bpuppy_pwm_ext.status(ch)[0]:
        return ch
    if int(n) in _NEED_ADC_STOP:
        import bpuppy_adc
        bpuppy_adc.stop()        # 同脚, 不停的话固件会跟 PWM 抢引脚
    bpuppy_pwm_ext.init(ch, _PIN[int(n)])
    return ch


def is_on(n):
    """该路是否已启用 —— **不会**触发初始化 (只查状态)"""
    return bool(bpuppy_pwm_ext.status(_chk(n))[0])


def set_angle(n, deg):
    """立即转到该角度 (硬切; 未初始化则自动初始化)"""
    bpuppy_pwm_ext.set_angle(ensure(n), deg)


def get_angle(n):
    """读该路当前角度 (deg); 未初始化则自动初始化"""
    return bpuppy_pwm_ext.get_angle(ensure(n))


def get_pulse_us(n):
    """读该路当前脉宽 (500~2500us); 未初始化则自动初始化"""
    return bpuppy_pwm_ext.get_pulse_us(ensure(n))


def off(n):
    """释放该路: 停脉冲, 引脚回到普通 GPIO"""
    bpuppy_pwm_ext.deinit(_chk(n))
