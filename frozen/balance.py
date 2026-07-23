"""
balance.py — 站立自平衡 (绕过 motion task, 直接舵机控制)
角度优先, 高度自适应

用法:
    import balance
    balance.start()
    balance.stop()

默认参数: kp=0.08, ki=0.0015, kd=0.5
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


def start(kp=0.08, ki=0.0015, kd=0.5, deadband=0.5, max_body=45.0, height=60.0):
    bpuppy_motion.emergency_stop()
    time.sleep(0.05)

    dt = 0.02
    br, bp = 0.0, 0.0
    zr_prev, zp_prev = 0.0, 0.0
    i_r, i_p = 0.0, 0.0
    prev_er, prev_ep = 0.0, 0.0

    # 记录初始姿态当零点
    ri, pi, _ = bpuppy_imu.read_angles()

    print("Balance ON  kp=%.1f  max=%.0f  h=%.0f  ofs=%+.1f/%+.1f" %
          (kp, max_body, height, ri, pi))

    while True:
        r, p, y = bpuppy_imu.read_angles()
        r = -(r - ri)   # 去偏置 + 方向翻转
        p -= pi

        # 从上帧足高反算实际身体-足平面角
        ra = math.atan2(zr_prev, HALF_W) / D2R
        pa = math.atan2(zp_prev, HALF_L) / D2R

        # 误差 = IMU − 实际
        er = r - ra
        ep = p - pa

        # 增量式 PID: br += kp×err + ki×∫err + kd×derr
        if abs(er) > deadband:
            i_r += er; d_r = er - prev_er
            br += kp * er + ki * i_r + kd * d_r
        else: i_r *= 0.9
        if abs(ep) > deadband:
            i_p += ep; d_p = ep - prev_ep
            bp += kp * ep + ki * i_p + kd * d_p
        else: i_p *= 0.9
        prev_er = er; prev_ep = ep

        if br > max_body: br = max_body
        if br < -max_body: br = -max_body
        if bp > max_body: bp = max_body
        if bp < -max_body: bp = -max_body

        # ---- 四足高度 ----
        zr_prev = HALF_W * math.tan(br * D2R)
        zp_prev = HALF_L * math.tan(bp * D2R)
        dz = [-zr_prev+zp_prev, -zr_prev-zp_prev, +zr_prev+zp_prev, +zr_prev-zp_prev]
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

        print('IMU %+5.1f %+5.1f | act %+5.1f %+5.1f | err %+5.1f %+5.1f | tgt %+5.1f %+5.1f' %
              (r, p, ra, pa, er, ep, br, bp))
        time.sleep(dt)


def stop():
    bpuppy_motion.resume()
    bpuppy_motion.set_gait("stand")
    bpuppy_motion.set_body_pose(0, 0, 0)
    print("Balance OFF")
