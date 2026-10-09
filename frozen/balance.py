"""
balance.py — 站立自平衡 (绕过 motion task, 直接舵机控制)
角度优先, 高度自适应

用法:
    import balance
    balance.start()          # 立即返回; 控制循环在后台线程里跑
    balance.stop()           # 随时可调: 停循环 + 收尾

    balance.VERBOSE = True   # 调试打印开关 (默认关, 打印每拍数据)

自平衡期间**写类指令被拒绝**(返回 False, 不执行):
    bpuppy_motion / bpuppy_servo / poses 的运动·姿态·舵机写入口全部拦下,
    只放 balance.stop 通过。被拒的 KittenBlock 积木会自动嘤一声。
    语音触发的动作走同一批入口, 同样被拒 (动作不发生)。

默认参数: kp=0.07, ki=0.0, kd=0.05
"""

import bpuppy_imu
import bpuppy_motion
import bpuppy_ik
import bpuppy_servo
import math
import time
import _thread

# ---- 机械参数 ----
# ⚠ 腿长 L1/L2 不在这里写死: start() 里从 bpuppy_motion.get_geometry() 现读。
#   bpuppy_ik.L1/L2 是编译期常量, cal_ik() 改不到它 (静默偏差 2°~16°)。
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

VERBOSE = False     # 每拍调试打印 (默认关: KittenBlock 调用时通道干净; REPL 里置 True 可开)
_running = False    # 循环继续标志 (主线程写, 后台线程读)

# ---- 自平衡期间的拦截名单 ----
# 只拦"写类"入口 (写舵机 / 切模式 / 改参数); 读类 (get_* / read_* / is_*) 全放行。
# 被拦的调用直接返回 False, 不执行。
_GUARD_SPEC = (
    ("bpuppy_motion", ("cal_ik", "load_geometry", "set_body_dims", "set_body_pose",
                       "set_center", "set_direction", "set_gait", "set_joint_limits",
                       "set_omega", "set_params", "set_speed", "set_turn", "start")),
    ("bpuppy_servo",  ("cal", "cal_point", "init", "init_all", "load_cal",
                       "set_angle", "stop",
                       "group_begin", "group_add", "group_commit")),
    ("poses",         ("set_servo", "set_step", "commit", "go_to", "oscillate",
                       "stand", "crouch", "sit", "play", "wave")),
)

_guarded = None     # None = 未装; list[(module, name, original)] = 已装


def _deny(*a, **k):
    return False


def _install_guard():
    """把写类入口替换成拒绝桩 (原函数留着, stop 时恢复)。"""
    global _guarded
    if _guarded is not None:
        return
    saved = []
    for mod_name, names in _GUARD_SPEC:
        try:
            mod = __import__(mod_name)
        except ImportError:
            continue
        for n in names:
            if hasattr(mod, n):
                saved.append((mod, n, getattr(mod, n)))
                try:
                    setattr(mod, n, _deny)
                except Exception:
                    saved.pop()          # 该模块属性不可写 → 跳过这一项
    _guarded = saved


def _remove_guard():
    """恢复被替换的入口。"""
    global _guarded
    if not _guarded:
        _guarded = None
        return
    for mod, n, orig in _guarded:
        try:
            setattr(mod, n, orig)
        except Exception:
            pass
    _guarded = None


def _clip(v, lim):
    if v > lim: return lim
    if v < -lim: return -lim
    return v


def _loop(kp, ki, kd, deadband, max_body, height, ri, pi, L1, L2, gb, ga, gc):
    """控制循环 — 跑在后台线程里。stop() 清 _running 后, 本循环下一拍退出。
    gb/ga/gc = group_begin/add/commit 的原始引用 (守卫装前取出, 绕过被替换的模块属性)。"""
    global _running
    br, bp = 0.0, 0.0
    i_r, i_p = 0.0, 0.0
    prev_er, prev_ep = 0.0, 0.0
    try:
        while _running:
            r, p, y = bpuppy_imu.read_angles()

            er = r - ri     # 误差 = 当前姿态 − 初始姿态
            ep = p - pi

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

            gb()
            for i, (hip_ch, knee_ch, side, leg_pair) in enumerate(LEGS):
                hip, knee = bpuppy_ik.solve(CENTER, fz_vals[i], L1, L2, side, leg_pair)
                ga(hip_ch, hip)
                ga(knee_ch, knee)
            gc()

            if VERBOSE:
                print('IMU P%+.1f R%+.1f | Err Pe%+.1f Re%+.1f | Inc iP%+.2f iR%+.2f | Tar Pt%+.1f Rt%+.1f' %
                      (p, r, ep, er, inc_p, inc_r, bp, br))
            time.sleep_ms(20)
    finally:
        _running = False    # 异常退出也清标志, 否则下次 start 会误判"还在跑"


def start(kp=0.07, ki=0.0, kd=0.05, deadband=0.5, max_body=30.0, height=60.0):
    """启动自平衡。准备(IMU/零点/几何/守卫)在主线程同步做完, 控制循环丢后台线程 —
    本函数立即返回, REPL / KittenBlock 照常可用; 期间写类指令被拒, stop() 随时能停。"""
    global _running
    if _running:                 # 已在跑 → 先停掉旧的
        stop()
    if not bpuppy_imu.is_ready():
        bpuppy_imu.init(0, 14, 21, 0x68)  # V3.0 硬件: SDA=14, SCL=21 (电池检测走 GPIO3=ADC1)
    bpuppy_imu.set_mag_fusion(False)      # 磁力计只修yaw, 不参与 roll/pitch (避免残差拉偏)
    time.sleep_ms(30)   # group 写舵机会自动切 POSE (C 层检测), motion 停止

    # 腿长从板子现读 (cal_ik() 改的就是这一份)
    g = bpuppy_motion.get_geometry()
    L1 = g[0]; L2 = g[1]

    # 记录初始姿态当零点 (本函数返回后零点不再改变)
    ri, pi, _ = bpuppy_imu.read_angles()

    # 守卫装前取出原始引用 — 控制循环用自己的引用, 不受拦截影响
    gb = bpuppy_servo.group_begin
    ga = bpuppy_servo.group_add
    gc = bpuppy_servo.group_commit

    _install_guard()                       # 写类入口 → 拒绝桩

    print("Balance ON  kp=%.3f  kd=%.3f  max=%.0f  h=%.0f  ofs=%+.1f/%+.1f" %
          (kp, kd, max_body, height, ri, pi))

    _running = True
    _thread.stack_size(8192)             # ⚠ 全局默认值, 用完必须还原
    _thread.start_new_thread(_loop, (kp, ki, kd, deadband, max_body,
                                     height, ri, pi, L1, L2, gb, ga, gc))
    _thread.stack_size(0)                # 0 = 端口默认


def stop():
    """停止自平衡: 清标志 → 等循环退出 → 卸守卫 → 主线程收尾。可随时调用。"""
    global _running
    _running = False
    time.sleep_ms(60)                     # 等循环退出 (最多一拍 20ms + 余量)
    _remove_guard()                       # 先放行写类入口, 收尾动作才走得通
    bpuppy_imu.set_mag_fusion(True)       # 恢复 磁力计参与 roll/pitch (9轴)
    bpuppy_motion.set_gait("stop")        # GAIT_STOP 站好 (自动进 MOTION)
    bpuppy_motion.set_body_pose(0, 0)     # (俯仰, 横滚)
    print("Balance OFF")
