"""
poses.py — bPuppy 姿态与舵机编辑 (Python 层)

分工: 运动/参数/步态用 bpuppy_motion 原语句 (与操作指南一致);
      姿态 (C 层已删) + 舵机编辑 + 动态动作由本模块实现。

用法:
    import poses
    poses.set_servo(0, 135)     # 设置舵机 0 目标 135°
    poses.commit()              # 平滑逼近并执行
    poses.crouch()              # 预定义: 蹲伏
    poses.sit()                 # 预定义: 猫坐
    poses.oscillate(3, 10, 4, 8)  # 舵机3 ±10° 4Hz 8次摆动
    poses.stand()               # POSE_STAND 站姿 (固定高度)
"""

import bpuppy_servo
import bpuppy_motion
import bpuppy_ik
import time
import math

D2R = 0.0174533

# ---- POSE_STAND 机械参数 (Python IK 站姿) ----
L1 = float(bpuppy_ik.L1)
L2 = float(bpuppy_ik.L2)
CENTER = 0.0
POSE_STAND_HEIGHT = 70.0   # 固定高度 (不随 set_params 运动高度变化)
LEGS = [
    (0, 1, bpuppy_ik.LEFT,  bpuppy_ik.FRONT),   # LF
    (2, 3, bpuppy_ik.LEFT,  bpuppy_ik.REAR),    # LH
    (4, 5, bpuppy_ik.RIGHT, bpuppy_ik.FRONT),   # RF
    (6, 7, bpuppy_ik.RIGHT, bpuppy_ik.REAR),    # RH
]

# ---- 8 路舵机命名常量 ----
LF_HIP, LF_KNEE = 0, 1
LH_HIP, LH_KNEE = 2, 3
RF_HIP, RF_KNEE = 4, 5
RH_HIP, RH_KNEE = 6, 7

# ---- 全局状态 ----
_pose_buf = [None] * 8      # 用户逐通道设置的目标角度，None=不修改
_pose_step = 3.0             # 过渡速度 (°/帧)，与 C SERVO_MAX_DEG_PER_FRAME 一致


# ============================================================
# 舵机编辑 (姿态)
# ============================================================

def set_servo(ch, deg):
    """设置某路舵机的目标角度（不立即执行，等 commit）"""
    _pose_buf[ch] = float(deg)


def set_step(n):
    """设置过渡速度 (°/帧)"""
    global _pose_step
    _pose_step = float(n)


def read_pose():
    """读取所有舵机当前位置填入 buffer, 并打印显示"""
    for ch in range(8):
        _pose_buf[ch] = bpuppy_servo.get_angle(ch)
    print("Pose: [" + ", ".join("%.1f" % (a if a is not None else 0.0)
          for a in _pose_buf) + "]")


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
    执行姿态: 写舵机自动切 POSE (C 层检测), 再从当前位置平滑逼近 _pose_buf。
    没被 set_servo() 修改的通道保持原位。
    """
    time.sleep_ms(30)   # 等 C 层自动切 POSE 生效
    _move_to(_pose_buf, _pose_step)
    # ★ 执行后清空 buffer, 防止残留污染下次 (否则之前设过的通道会被意外写入)
    for ch in range(8):
        _pose_buf[ch] = None


def go_to(targets, step=3.0):
    """
    一步到位: 写舵机自动切 POSE, 直接逼近 targets（8 个浮点数列表）。
    不修改 _pose_buf，适合代码直调。
    """
    time.sleep_ms(30)
    _move_to(list(targets), step)


# ============================================================
# 动态摆动
# ============================================================

def oscillate(ch, amp, hz, cycles):
    """单舵机正弦摆动: 写舵机自动切 POSE, 在当前位置 ±amp°, 频率 hz, 循环 cycles 次"""
    time.sleep_ms(30)
    center = bpuppy_servo.get_angle(ch)
    # 帧数 = 50帧/秒 × 次数 / 频率 (每周期 50/hz 帧)
    frames = int(round(50.0 * cycles / hz))
    for i in range(frames):
        val = center + amp * math.sin(2.0 * math.pi * hz * i / 50.0)
        bpuppy_servo.set_angle(ch, val)
        time.sleep_ms(20)


# ============================================================
# 恢复 C 层控制
# ============================================================

def stand():
    """POSE_STAND: Python IK 站姿 (固定高度 70), 留在姿态模式"""
    targets = [None] * 8
    for hip_ch, knee_ch, side, leg_pair in LEGS:
        hip, knee = bpuppy_ik.solve(CENTER, POSE_STAND_HEIGHT, L1, L2, side, leg_pair)
        targets[hip_ch] = hip
        targets[knee_ch] = knee
    go_to(targets)


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
    frames = int(round(50.0 * 8.0 / 4.0))  # 8次 @ 4Hz = 100帧 = 2秒
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
    frames = int(round(50.0 * 3.0 / 1.0))  # 3次 @ 1Hz = 150帧 = 3秒
    for i in range(frames):
        wave_knee = math.sin(2.0 * math.pi * 1.0 * i / 50.0) * 10.0
        bpuppy_servo.set_angle(RF_KNEE, 45 + wave_knee)
        time.sleep_ms(20)

    # 回到 sit
    time.sleep_ms(300)
    go_to(SIT)
