"""
balance.py — 站立自平衡 (绕过 motion task, 直接舵机控制)
角度优先, 高度自适应

用法:
    import balance
    balance.start()
    balance.stop()

默认参数: kp=0.06, ki=0.0, kd=0.43
"""

import bpuppy_imu
import bpuppy_motion
import bpuppy_ik
import bpuppy_servo
import math
import time

# ---- 机械参数 ----
L1 = float(bpuppy_ik.L1)
L2 = float(bpuppy_ik.L2)
HALF_L = 62.5
HALF_W = 59.0
MIN_Z = 15.0   # 最小足端高度
MAX_Z = 82.0   # L1+L2-3
CENTER = 0.0

LEFT  = bpuppy_ik.LEFT
RIGHT = bpuppy_ik.RIGHT
FRONT = bpuppy_ik.FRONT
REAR  = bpuppy_ik.REAR

LEGS = [
    (0, 1, LEFT,  FRONT),  # LF
    (2, 3, LEFT,  REAR),   # LH
    (4, 5, RIGHT, FRONT),  # RF
    (6, 7, RIGHT, REAR),   # RH
]

D2R = 0.0174533


def _clip(v, lim):
    if v > lim: return lim
    if v < -lim: return -lim
    return v


def start(kp=0.06, ki=0.0, kd=0.43, deadband=0.5, max_body=30.0, height=60.0):
    if not bpuppy_imu.is_ready():
        bpuppy_imu.init(0, 3, 14, 0x68)   # 自动启动 IMU
    bpuppy_imu.set_mag_fusion(False)      # 磁力计只修yaw, 不参与 roll/pitch (避免残差拉偏)
    time.sleep_ms(30)   # group 写舵机会自动切 POSE (C 层检测), motion 停止

    dt = 0.02
    br, bp = 0.0, 0.0
    i_r, i_p = 0.0, 0.0
    prev_er, prev_ep = 0.0, 0.0

    # 记录初始姿态当零点
    ri, pi, _ = bpuppy_imu.read_angles()

    print("Balance ON  kp=%.3f  max=%.0f  h=%.0f  ofs=%+.1f/%+.1f" %
          (kp, max_body, height, ri, pi))

    while True:
        r, p, y = bpuppy_imu.read_angles()

        er = ri - r     # 误差 = 初始姿态 − 当前姿态
        ep = pi - p

        # 补偿增量 = kp×err + ki×∫err + kd×(err−err_prev)
        if abs(er) > deadband:
            i_r += er; d_r = er - prev_er
            inc_r = kp * er + ki * i_r + kd * d_r
            br += inc_r
        else:
            i_r *= 0.9; inc_r = 0.0
        if abs(ep) > deadband:
            i_p += ep; d_p = ep - prev_ep
            inc_p = -(kp * ep + ki * i_p + kd * d_p)
            bp += inc_p
        else:
            i_p *= 0.9; inc_p = 0.0
        prev_er = er; prev_ep = ep

        if br > max_body: br = max_body
        if br < -max_body: br = -max_body
        if bp > max_body: bp = max_body
        if bp < -max_body: bp = -max_body

        # ---- 四足高度 ----
        z_roll  = HALF_W * math.tan(br * D2R)
        z_pitch = HALF_L * math.tan(bp * D2R)
        dz = [-z_roll+z_pitch, -z_roll-z_pitch, +z_roll+z_pitch, +z_roll-z_pitch]
        c_min = max(MIN_Z - d for d in dz)
        c_max = min(MAX_Z - d for d in dz)
        if c_min <= c_max:
            center = max(c_min, min(c_max, height))
        else:
            center = (c_min + c_max) * 0.5

        fz_vals = [center + d for d in dz]

        bpuppy_servo.group_begin()
        for i, (hip_ch, knee_ch, side, leg_pair) in enumerate(LEGS):
            hip, knee = bpuppy_ik.solve(CENTER, fz_vals[i], L1, L2, side, leg_pair)
            bpuppy_servo.group_add(hip_ch, hip)
            bpuppy_servo.group_add(knee_ch, knee)
        bpuppy_servo.group_commit()

        print('IMU P%+.1f R%+.1f | Err Pe%+.1f Re%+.1f | Inc iP%+.2f iR%+.2f | Tar Pt%+.1f Rt%+.1f' %
              (p, r, ep, er, inc_p, inc_r, bp, br))
        time.sleep_ms(20)


def stop():
    bpuppy_imu.set_mag_fusion(True)       # 恢复 磁力计参与 roll/pitch (9轴)
    bpuppy_motion.set_gait("stop")        # GAIT_STOP 站好 (自动进 MOTION)
    bpuppy_motion.set_body_pose(0, 0, 0)
    print("Balance OFF")
