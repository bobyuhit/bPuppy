"""
bPuppy 运动控制 — Python 层 API

封装 bpuppy_motion C 模块，提供:
- 逆运动学 (IK) 解算
- 步态生成器
- 身体姿态控制

TODO: 完善步态算法
"""

import math

# ---- 机器人几何参数（默认值，根据实际机械结构调整）----
LEG_COUNT = 4           # 腿数
HIP_LENGTH = 40.0       # 髋关节长度 (mm)
THIGH_LENGTH = 40.0     # 大腿长度 (mm)
CALF_LENGTH = 45.0      # 小腿长度 (mm)
BODY_WIDTH = 100.0      # 身体半宽 (mm)
BODY_LENGTH = 150.0     # 身体半长 (mm)


class Leg:
    """单腿控制接口"""
    def __init__(self, index, hip_ch, thigh_ch, calf_ch):
        self.index = index
        self.hip_ch = hip_ch      # 髋关节舵机通道
        self.thigh_ch = thigh_ch  # 大腿关节舵机通道
        self.calf_ch = calf_ch    # 小腿关节舵机通道

    def set_angles(self, hip, thigh, calf):
        """设置单腿三个关节角度"""
        import bpuppy_servo
        bpuppy_servo.set_angle(self.hip_ch, hip)
        bpuppy_servo.set_angle(self.thigh_ch, thigh)
        bpuppy_servo.set_angle(self.calf_ch, calf)


class IK:
    """逆运动学解算"""
    @staticmethod
    def solve(leg_index, foot_x, foot_y, foot_z):
        """
        给定足端坐标 (x, y, z) 相对于髋关节，返回关节角度
        TODO: 实现实际 IK 算法
        """
        # 占位 - 返回中立位角度
        return 90.0, 90.0, 90.0


class GaitGenerator:
    """步态生成器"""
    def __init__(self):
        self.phase = 0.0
        self.gait = "stand"

    def set_gait(self, gait_name):
        """设置步态: stand, trot, walk, crawl"""
        self.gait = gait_name

    def update(self, dt):
        """
        更新步态相位，返回每条腿的足端偏移
        TODO: 实现实际步态逻辑
        """
        self.phase += 2 * math.pi * dt
        if self.phase > 2 * math.pi:
            self.phase -= 2 * math.pi
        # 占位
        return [(0, 0, -80)] * LEG_COUNT


class bPuppyController:
    """
    bPuppy 主控制器
    封装底层驱动，提供统一的机器人控制接口
    """
    def __init__(self):
        self.legs = []
        self.gait_gen = GaitGenerator()
        self._running = False

    def init_legs(self, pin_map):
        """
        初始化所有腿
        pin_map: [(hip_gpio, thigh_gpio, calf_gpio), ...] x4
        """
        import bpuppy_servo
        for i, (hip_pin, thigh_pin, calf_pin) in enumerate(pin_map):
            ch_base = i * 3
            bpuppy_servo.init(ch_base + 0, hip_pin)
            bpuppy_servo.init(ch_base + 1, thigh_pin)
            bpuppy_servo.init(ch_base + 2, calf_pin)
            self.legs.append(Leg(i, ch_base, ch_base + 1, ch_base + 2))

    def start(self):
        """启动运动控制"""
        import bpuppy_motion
        bpuppy_motion.start()
        self._running = True

    def stop(self):
        """停止运动"""
        import bpuppy_motion
        bpuppy_motion.emergency_stop()
        self._running = False

    def walk(self, speed=0.5):
        """前进"""
        import bpuppy_motion
        bpuppy_motion.set_gait("walk")
        bpuppy_motion.set_params(speed=speed, stride=30, height=80)

    def trot(self, speed=0.5):
        """小跑"""
        import bpuppy_motion
        bpuppy_motion.set_gait("trot")
        bpuppy_motion.set_params(speed=speed, stride=50, height=80)

    def stand(self):
        """站立"""
        import bpuppy_motion
        bpuppy_motion.set_gait("stand")

    def sit(self):
        """蹲下"""
        import bpuppy_motion
        bpuppy_motion.set_gait("crouch")


# 创建全局控制器实例
dog = bPuppyController()
