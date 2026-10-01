"""
pwm_ext.py — PWM_EXT1/2/3 扩展舵机的**底层封装**

只做三件事:
  1. 编号映射: PWM_EXTn (1/2/3) → MCPWM 通道 (0/1/2) + GPIO (3/47/48)
  2. 显式启用: on(n) / off(n) —— 关 ADC、抢管脚都收在这里
  3. 角度读写的薄封装 (立即生效, 不缓动)

⚠ **本模块不自动启用。** 开哪几路由开机脚本 /pwm_ext_on.py 的宏决定
   (源文件 mpy_modules/pwm_ext_on.py, 传到板子根目录即生效)。
   管脚功能开机定死, 运行中不变 —— 未启用的路读写**直接报错**, 不会偷偷把它打开。

⚠ **本模块不含缓动。** 平滑逼近统一在 poses.py 里, 主舵机和扩展舵机共用同一套
   实现 —— 缓动算法跟"驱动哪个舵机"无关, 只跟"读角度/写角度"有关。
   要平滑移动请用:
       import poses
       poses.set_servo(8|9|10, deg)     # 8/9/10 = PWM_EXT1/2/3
       poses.set_step(3)
       poses.commit()

用法 (裸控制):
    import pwm_ext
    pwm_ext.on(2)                # 启用 PWM_EXT2 —— 只在开机脚本里调, 见下
    pwm_ext.set_angle(2, 90)     # PWM_EXT2 → 90°, 立即生效 (硬切)
    pwm_ext.get_angle(2)         # 读当前角度
    pwm_ext.get_pulse_us(2)      # 读当前脉宽
    pwm_ext.is_on(2)             # 该路是否已启用 (只查状态, 不触发动作)
    pwm_ext.off(2)               # 释放该路

⚠ 同脚冲突: PWM_EXT1(GPIO3) 与电池 ADC 同脚, PWM_EXT3(GPIO48) 与 WS2812
   电池指示灯同脚。on() 会**自动先调 bpuppy_adc.stop() 让路** —— 副作用是
   电池电压读数和指示灯停掉, 直到手动 bpuppy_adc.init() 恢复。
   PWM_EXT2(GPIO47) 是空闲脚, 无副作用。
"""

import bpuppy_pwm_ext

# PWM_EXTn → GPIO (板上固定接线, 见 PCB/硬件连接.md)
# ★ 通道号 = n-1, 不能改: poses.py 把舵机编号 8/9/10 映射到 MCPWM 通道 0/1/2
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


def _on_ch(n):
    """校验 + 确认该路已启用。未启用抛能看懂的错 (C 层只会说"通道未 init")"""
    ch = _chk(n)
    if not bpuppy_pwm_ext.status(ch)[0]:
        raise ValueError(
            "PWM_EXT%d 未启用 —— 开机时 /pwm_ext_on.py 里 EXT%d 是 0。"
            "改成 1 并重新上电。" % (int(n), int(n)))
    return ch


def on(n):
    """启用该路 (幂等)。EXT1/EXT3 与电池检测同脚, 会先停 ADC。

    ★ 本模块唯一抢管脚的地方 —— 只在开机脚本 /pwm_ext_on.py 里调用。
      读写 (set_angle / get_angle / get_pulse_us) 不会替你开: 未启用直接报错,
      否则"读一下角度"就会停掉电池检测并让舵机跳到中位。"""
    ch = _chk(n)
    if bpuppy_pwm_ext.status(ch)[0]:
        return                       # 幂等: 已开就什么都不做
    if int(n) in _NEED_ADC_STOP:
        import bpuppy_adc
        bpuppy_adc.stop()            # 同脚, 不停的话固件会跟 PWM 抢引脚
    bpuppy_pwm_ext.init(ch, _PIN[int(n)])


def off(n):
    """释放该路: 停脉冲, 引脚回到普通 GPIO"""
    bpuppy_pwm_ext.deinit(_chk(n))


def is_on(n):
    """该路是否已启用 —— **不会**触发任何动作 (只查状态)"""
    return bool(bpuppy_pwm_ext.status(_chk(n))[0])


def set_angle(n, deg):
    """立即转到该角度 (硬切); 该路未启用则报错"""
    bpuppy_pwm_ext.set_angle(_on_ch(n), deg)


def get_angle(n):
    """读该路当前角度 (deg); 该路未启用则报错"""
    return bpuppy_pwm_ext.get_angle(_on_ch(n))


def get_pulse_us(n):
    """读该路当前脉宽 (500~2500us); 该路未启用则报错"""
    return bpuppy_pwm_ext.get_pulse_us(_on_ch(n))
