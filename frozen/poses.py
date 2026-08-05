"""
poses.py — bPuppy 姿态与动态动作 (Python 层)

用法:
    import poses
    poses.set(0, 135)           # 设置舵机 0 目标 135°
    poses.commit()              # 平滑逼近并执行
    poses.crouch()              # 预定义: 蹲伏
    poses.sit()                 # 预定义: 猫坐
    poses.osc(3, 10, 4, 8)      # 舵机3 ±10° 4Hz 8次摆动
    poses.stand()               # 恢复 C 层 IK 站姿
    poses.fwd()                 # 前进 (KittenBlock 极短接口)

KittenBlock 在线 REPL 模式串口易丢字, 所有积木 pycode 必须 ≤15 字符,
因此提供短名函数: fwd/bwd/tl/tr/stp/spd/stride/hgt/lift/gait/step/pose/osc/dly
"""

import bpuppy_servo
import bpuppy_motion
import time
import math

D2R = 0.0174533

# ---- 8 路舵机命名常量 ----
LF_HIP, LF_KNEE = 0, 1
LH_HIP, LH_KNEE = 2, 3
RF_HIP, RF_KNEE = 4, 5
RH_HIP, RH_KNEE = 6, 7

# ---- 全局状态 ----
_pose_buf = [None] * 8      # 用户逐通道设置的目标角度，None=不修改
_pose_step = 3.0             # 过渡速度 (°/帧)，与 C SERVO_MAX_DEG_PER_FRAME 一致
_speed = 2.5
_stride = 70
_height = 70


# ============================================================
# 运动控制 (KittenBlock 极短接口)
# ============================================================

def ensure_motion():
    """启动 motion task (幂等), 平滑站起"""
    if not bpuppy_motion.is_running():
        bpuppy_motion.start()
        bpuppy_motion.stand_up()
        time.sleep(1)


def go(speed=2.5, stride=70, height=70, turn=0):
    """快捷: 启动 motion + 自适应前进"""
    ensure_motion()
    bpuppy_motion.set_params(speed, stride, height)
    bpuppy_motion.set_turn(turn)
    bpuppy_motion.set_gait('go')


def stop_motion():
    """快捷: 停止 motion"""
    bpuppy_motion.emergency_stop()


# ---- KittenBlock 短名 ----
def fwd():  go(_speed, _stride, _height, 0)
def bwd():  go(_speed, -_stride, _height, 0)
def tl():   go(_speed, _stride, _height, -0.8)
def tr():   go(_speed, _stride, _height, 0.8)
def stp():  bpuppy_motion.emergency_stop()
def spd(n):
    global _speed
    _speed = float(n)
def stride(n):
    global _stride
    _stride = float(n)
def hgt(n):
    global _height
    _height = float(n)
def lift(n):
    bpuppy_motion.set_lift(float(n))
def gait(n):
    """切换步态: 数字参数 (1=go 2=walk 3=trot 4=stand), 统一重置 turn"""
    name = {1: 'go', 2: 'walk', 3: 'trot', 4: 'stand'}.get(int(float(n)), 'go')
    if name == 'stand':
        stand()
        return
    ensure_motion()
    bpuppy_motion.set_params(_speed, _stride, _height)
    bpuppy_motion.set_turn(0)
    bpuppy_motion.set_gait(name)
def dly(n):
    time.sleep(n)


# ============================================================
# 原子操作
# ============================================================

def set(ch, deg):
    """设置某路舵机的目标角度（不立即执行，等 commit）"""
    _pose_buf[ch] = float(deg)


def set_step(n):
    """设置过渡速度 (°/帧)"""
    global _pose_step
    _pose_step = float(n)


def step(n):
    set_step(n)


def read_pose():
    """读取所有舵机当前位置填入 buffer"""
    for ch in range(8):
        _pose_buf[ch] = bpuppy_servo.get_angle(ch)


def pose():
    read_pose()


# ============================================================
# 平滑逼近 — Python 版 servo_step_toward
# ============================================================

def _get_cur():
    """返回 8 路舵机当前角度"""
    return [bpuppy_servo.get_angle(ch) for ch in range(8)]


def _move_to(targets, step=3.0):
    """
    阻塞式平滑逼近 8 路舵机到 targets。
    targets 中 None 的通道保持原位不动。
    """
    cur = _get_cur()
    resolved = [t if t is not None else cur[i] for i, t in enumerate(targets)]

    while True:
        done = True
        for ch in range(8):
            diff = resolved[ch] - cur[ch]
            if abs(diff) < 0.3:
                continue
            done = False
            cur[ch] += max(-step, min(step, diff))

        bpuppy_servo.group_begin()
        for ch in range(8):
            bpuppy_servo.group_add(ch, cur[ch])
        bpuppy_servo.group_commit()

        if done:
            break
        time.sleep_ms(20)


def commit():
    """
    执行姿态: 先急停 motion task, 再从当前位置平滑逼近 _pose_buf。
    没被 set() 修改的通道保持原位。
    """
    bpuppy_motion.emergency_stop()
    time.sleep_ms(50)
    _move_to(_pose_buf, _pose_step)


def go_to(targets, step=3.0):
    """
    一步到位: 急停 → 直接逼近 targets（8 个浮点数列表）。
    不修改 _pose_buf，适合代码直调。
    """
    bpuppy_motion.emergency_stop()
    time.sleep_ms(50)
    _move_to(list(targets), step)


# ============================================================
# 动态摆动
# ============================================================

def oscillate(ch, amp, hz, cycles):
    """单舵机正弦摆动: 先急停, 在当前位置 ±amp°, 频率 hz, 循环 cycles 次"""
    bpuppy_motion.emergency_stop()
    time.sleep_ms(30)
    center = bpuppy_servo.get_angle(ch)
    frames = int(round(50.0 * hz * cycles))  # 50帧/秒 × 秒数
    for i in range(frames):
        val = center + amp * math.sin(2.0 * math.pi * hz * i / 50.0)
        bpuppy_servo.set_angle(ch, val)
        time.sleep_ms(20)


def osc(ch, amp, hz, cycles):
    oscillate(ch, amp, hz, cycles)


# ============================================================
# 恢复 C 层控制
# ============================================================

def stand():
    """恢复 C 层 motion task，平滑过渡到 IK 站姿"""
    bpuppy_motion.resume()
    bpuppy_motion.set_gait("stand")


# ============================================================
# 预定义姿态 (语法糖，直接调 go_to)
# ============================================================

# 蹲伏: 四腿折叠
CROUCH = [135, 45,  45, 135,  45, 135,  135, 45]

# 猫坐: 前腿撑, 后腿折 (实测角度)
SIT    = [120, 160, 10, 165,  60, 20,  170, 15]

# 邀玩: 前低后高
PLAY   = [50, 125,  70, 50,   130, 55,  110, 130]


def crouch():
    go_to(CROUCH)


def sit():
    go_to(SIT)


def play():
    """邀玩: 移动到位 → 4Hz 摇后腿 8 次 → 保持"""
    go_to(PLAY)
    time.sleep_ms(100)

    cur_lh = bpuppy_servo.get_angle(LH_KNEE)
    cur_rh = bpuppy_servo.get_angle(RH_KNEE)
    frames = int(round(50.0 * 4.0 * 2.0))  # 4Hz × 2s = 8 次
    for i in range(frames):
        wiggle = math.sin(2.0 * math.pi * 4.0 * i / 50.0) * 10.0
        bpuppy_servo.group_begin()
        bpuppy_servo.group_add(LH_KNEE, cur_lh + wiggle)
        bpuppy_servo.group_add(RH_KNEE, cur_rh + wiggle)
        bpuppy_servo.group_commit()
        time.sleep_ms(20)


def wave():
    """挥手: sit → 后腿到位 → RF 膝摆动 3 次 → 回 sit"""
    go_to(SIT)
    time.sleep_ms(300)

    # 后腿大腿到位 + 右前腿抬起 (限速平滑)
    # [LF_HIP, LF_KNEE, LH_HIP, LH_KNEE, RF_HIP, RF_KNEE, RH_HIP, RH_KNEE]
    go_to([None, None, 48, None, 150, 45, 132, None], step=3.0)
    time.sleep_ms(600)

    # RF_KNEE 1Hz 摆动 3 次 (从 45° 起)
    frames = int(round(50.0 * 1.0 * 3.0))
    for i in range(frames):
        wave_knee = math.sin(2.0 * math.pi * 1.0 * i / 50.0) * 10.0
        bpuppy_servo.set_angle(RF_KNEE, 45 + wave_knee)
        time.sleep_ms(20)

    # 回到 sit
    time.sleep_ms(300)
    go_to(SIT)
