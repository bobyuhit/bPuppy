"""
poses.py — bPuppy 姿态与动态动作 (Python 层)

用法:
    import poses
    poses.set(0, 135)           # 设置舵机 0 目标 135°
    poses.set(1, 45)            # 设置舵机 1 目标 45°
    poses.commit()              # 平滑逼近并执行
    poses.crouch()              # 预定义: 蹲伏
    poses.sit()                 # 预定义: 猫坐
    poses.oscillate(1, 10, 4, 8)  # 舵机1 ±10° 4Hz 8次摆动
    poses.stand()               # 恢复 C 层 IK 站姿
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


def read_pose():
    """读取所有舵机当前位置填入 buffer"""
    for ch in range(8):
        _pose_buf[ch] = bpuppy_servo.get_angle(ch)


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
    """
    单舵机正弦摆动: 在当前位置基础上 ±amp°, 频率 hz, 循环 cycles 次。
    多次摆动之间不归零——起始值=当前舵机实际角度。
    """
    center = bpuppy_servo.get_angle(ch)
    period = 1.0 / hz
    frames_per_cycle = int(period / 0.02)  # 50Hz
    total_frames = frames_per_cycle * int(cycles)

    for i in range(total_frames):
        t = i / frames_per_cycle * 2.0 * math.pi * cycles / cycles
        val = center + amp * math.sin(t * cycles / frames_per_cycle * 2.0 * math.pi)
        bpuppy_servo.set_angle(ch, val)
        time.sleep_ms(20)


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

    # 后腿双膝同步摆动: LH_KNEE=3, RH_KNEE=7
    cur_lh = bpuppy_servo.get_angle(LH_KNEE)
    cur_rh = bpuppy_servo.get_angle(RH_KNEE)
    period = 1.0 / 4.0  # 4Hz
    frames = int(period / 0.02) * 8  # 8 cycles

    for i in range(frames):
        t = i / 50.0  # 秒
        wiggle = math.sin(t * 4.0 * 2.0 * math.pi) * 10.0
        bpuppy_servo.group_begin()
        bpuppy_servo.group_add(LH_KNEE, cur_lh + wiggle)
        bpuppy_servo.group_add(RH_KNEE, cur_rh + wiggle)
        bpuppy_servo.group_commit()
        time.sleep_ms(20)


def wave():
    """挥手: sit → 后腿到位 → RF 膝摆动 3 次 → 回 sit"""
    go_to(SIT)
    time.sleep_ms(300)

    # 后腿到位 (sit 基底下调大腿)
    go_to([None, None, 48, None, None, None, 132, None], step=3.0)
    time.sleep_ms(600)

    # 右前腿抬起 + 摆动 3 次
    cur_rf = bpuppy_servo.get_angle(RF_KNEE)
    bpuppy_servo.set_angle(RF_HIP, 150)
    bpuppy_servo.set_angle(RF_KNEE, 45)

    period = 1.0 / 1.0  # 1Hz
    frames = int(period / 0.02) * 3  # 3 cycles
    for i in range(frames):
        t = i / 50.0
        wave_knee = math.sin(t * 1.0 * 2.0 * math.pi) * 10.0
        bpuppy_servo.set_angle(RF_KNEE, 45 + wave_knee)
        time.sleep_ms(20)

    # 后腿恢复
    bpuppy_servo.set_angle(RF_KNEE, cur_rf)
    time.sleep_ms(500)

    go_to(SIT)
