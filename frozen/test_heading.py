"""
test_heading.py — 航向锁定: 保持 57°

用法:
    import test_heading
    test_heading.start()
    test_heading.stop()
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
    while err > 180:
        err -= 360
    while err < -180:
        err += 360
    return err


def start(target=57.0):
    global _target, _running
    _target = target
    _running = True

    bpuppy_motion.set_params(2.5, 70, 70)
    bpuppy_motion.set_gait("go")

    print("Heading lock: target=%.0f°  (ctrl-C to stop)" % _target)

    while _running:
        try:
            roll, pitch, yaw = bpuppy_imu.read_angles()
            err = _angle_error(yaw, _target)
            turn = err * _KP
            if turn > 1.0:
                turn = 1.0
            elif turn < -1.0:
                turn = -1.0

            bpuppy_motion.set_turn(turn)

            if abs(err) < _DEADBAND:
                bpuppy_motion.set_gait("stand")
            else:
                bpuppy_motion.set_params(2.5, 70, 70)
                bpuppy_motion.set_gait("go")

            print("  yaw=%.1f  err=%+.1f  turn=%.2f  %s" %
                  (yaw, err, turn, "HOLD" if abs(err) < _DEADBAND else "TURN"))

        except Exception as e:
            print("ERR:", e)

        time.sleep(0.05)  # 20Hz, 跟 motion 任务同步


def stop():
    global _running
    _running = False
    bpuppy_motion.set_turn(0)
    bpuppy_motion.set_gait("stand")
    print("Heading lock stopped")
