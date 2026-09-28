"""
set_heading.py — 航向锁定: 保持 57°

用法:
    import set_heading
    set_heading.start()
    set_heading.stop()
"""

import bpuppy_imu
import bpuppy_motion
import time

_target = 57.0
_running = False
_KP = 0.03
_DEADBAND = 5.0   # ±5° 内不转


def _angle_error(current, target):
    """最短转角, 返回 [-180, 180]"""
    err = target - current
    while err > 180.0:
        err -= 360.0
    while err <= -180.0:
        err += 360.0
    return err


def start(target=57.0):
    global _target, _running
    _target = target
    _running = True

    if not bpuppy_imu.is_ready():
        bpuppy_imu.init(0, 14, 21, 0x68)  # V3.0 硬件: SDA=14, SCL=21 (电池检测走 GPIO3=ADC1)

    # 步频与步长/站高各归各的函数 (set_params 现在收的是 步长/抬脚/站高)。
    # 抬脚高度从板上读回 —— 本函数以前根本不碰 lift, 写死 30 会把用户设的值静默重置。
    # ★ 方向必须**显式**说成朝前: 步长解耦成幅度之后, set_params(70,...) 不再隐含"前进"。
    #   本脚本是"朝目标航向走过去", 方向由 turn 控制, 前进方向永远是 +1。
    _lift = bpuppy_motion.get_params()[3]
    bpuppy_motion.set_speed(2.5)
    bpuppy_motion.set_direction(1)
    bpuppy_motion.set_params(70, _lift, 70)
    bpuppy_motion.set_gait("go")

    print("Heading lock: target=%.0f°  (ctrl-C to stop)" % _target)

    while _running:
        try:
            roll, pitch, yaw = bpuppy_imu.read_angles()
            err = _angle_error(yaw, _target)
            turn = -err * _KP
            if turn > 1.0:
                turn = 1.0
            elif turn < -1.0:
                turn = -1.0

            bpuppy_motion.set_turn(turn)

            if abs(err) < _DEADBAND:
                print("  yaw=%.1f  err=%+.1f  ✓ 到达, 停下" % (yaw, err))
                bpuppy_motion.set_turn(0)
                bpuppy_motion.set_gait("stop")   # 必须用合法名: "stand" 不是步态, 会走未知步态兜底
                break
            else:
                print("  yaw=%.1f  err=%+.1f  turn=%.2f  TURN" %
                      (yaw, err, turn))

        except Exception as e:
            print("ERR:", e)

        time.sleep_ms(50)  # 20Hz


def stop():
    global _running
    _running = False
    bpuppy_motion.set_turn(0)
    bpuppy_motion.set_gait("stop")   # 同上: 合法步态名
    print("Heading lock stopped")
