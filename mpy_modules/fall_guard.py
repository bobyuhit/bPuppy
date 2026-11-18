"""
fall_guard.py — 跌倒保护 (VFS 模块，不冻结)

行为:
  - 本文件放在板子根目录 /fall_guard.py；import 时自动 start()。
  - 上电: 不释放。frozen/main.py 在 poses.stand() 和 voice 就绪后 import,
    这时才打开 C 层检测; 检测器先要求约 0.5s 稳定才武装。
  - 运动中或站立中 (C 层判断) 倾角 > 60° 或角速度 > 400°/s 连续 3 帧:
    C 层立即锁存、关 8 路主舵机 + PWM_EXT 扩展舵机、停运动。
    Python 只负责听到后嘤嘤 (扩展舵机再按实际启用状态兜底释放一次)。
  - 恢复: 保护锁存期间, 收到语音模块的声源角度帧, 且机身放平、稳定
    约 0.3s → 主舵机按 poses.CROUCH 重新上力，扩展舵机按释放前保存的
    最后脉宽自动恢复；不自动站起。不再有“扶正 5 秒自动恢复”。

REPL:
    import fall_guard
    fall_guard.on()           # 打开自动检测 (import 已自动执行)
    fall_guard.off()          # 关自动检测; 手动/语音恢复仍可用
    fall_guard.release()      # 手动进入保护: 松开主舵机 + 扩展舵机
    fall_guard.recover()      # 手动从卧姿恢复 (要求机身放平)
    fall_guard.rearm()        # recover() 别名
    fall_guard.latched()      # 当前是否锁存
    fall_guard.is_enabled()   # 自动检测是否打开
    fall_guard.set(angle=60, rate=400, frames=3,
                   level_angle=15, level_ms=300)

注意:
  - 检测在 C 的 50Hz motion task 里, 不受用户 Python 程序拖慢。
  - 只有运动模式或 poses.stand() 站姿在检测范围; crouch/sit/play/wave
    等静态姿势不检测。
  - 恢复姿势是 poses.CROUCH, 不读取“舵机实际角度” (模拟舵机没有反馈)。
  - off() 只关自动检测; release()/recover()/语音恢复仍然可用。
"""

import time
import _thread

import bpuppy_imu
import bpuppy_motion

# ---- 默认阈值 ----
ANGLE        = 60.0     # 倾角阈值 (deg)
RATE         = 400.0    # 角速度阈值 (deg/s)
FRAMES       = 3        # 连续帧数 @50Hz (约 60ms)
LEVEL_ANGLE  = 15.0     # 判定“身体放平”的倾角上限 (deg)
LEVEL_RATE   = 30.0     # 判定“身体放平”的角速度上限 (deg/s)
LEVEL_MS     = 300      # 放平需要持续多久 (ms)
PERIOD_MS    = 20       # 后台线程周期

_running      = False   # 恢复线程是否已起
_enabled      = False   # 自动检测是否打开 (C 层 safety_enable)
_latched      = False
_sound_event  = False
_level_count  = 0
_level_need   = LEVEL_MS // PERIOD_MS
_ext_saved    = []      # Python 兜底: [(PWM_EXT 编号, 最后角度), ...]


def _on_sound():
    """voice 的声源角度回调 (在 voice 后台线程里跑): 只置位, 不在这里动舵机。"""
    global _sound_event
    _sound_event = True


def _beep(name):
    try:
        import voice
        voice.say(*getattr(voice, name))
    except Exception:
        pass


def _stop_balance():
    """自平衡线程还在的话停掉; 它的写舵机本来也会被 C 锁存拒绝。"""
    try:
        import balance
        if getattr(balance, "_running", False):
            balance.stop()
    except Exception:
        pass


def _stop_ext():
    """松开 PWM_EXT1/2/3 扩展舵机。C 层 motion_safety_release 已调
    pwm_ext_stop_all(); 这里再按 Python 侧的实际启用状态兜底一次，
    并保存最后角度供 _rearm_ext() 使用 (C 层保存的是脉宽)。"""
    global _ext_saved
    _ext_saved = []
    try:
        import pwm_ext
    except Exception:
        return
    for n in (1, 2, 3):
        try:
            if pwm_ext.is_on(n):
                try:
                    ang = pwm_ext.get_angle(n)
                except Exception:
                    ang = None
                _ext_saved.append((n, ang))
                pwm_ext.off(n)
        except Exception:
            pass


def _rearm_ext():
    """恢复 Python 兜底保存的扩展舵机; C 层已恢复的通道 is_on 为 True，
    只需要把角度再写回保存值 (通常与 C 保存的脉宽一致)。"""
    global _ext_saved
    if not _ext_saved:
        return
    try:
        import pwm_ext
    except Exception:
        _ext_saved = []
        return
    for n, ang in _ext_saved:
        try:
            if not pwm_ext.is_on(n):
                pwm_ext.on(n)
            if ang is not None:
                pwm_ext.set_angle(n, ang)
        except Exception:
            pass
    _ext_saved = []


def _read_level():
    """返回 (tilt, rate, level)。读不到 IMU 时按“不平”处理。"""
    try:
        if not bpuppy_imu.is_ready():
            return 999.0, 999.0, False
        r, p, _y = bpuppy_imu.read_angles()
        raw = bpuppy_imu.read_raw()
        gx, gy, gz = raw[1]
        rate = (gx * gx + gy * gy + gz * gz) ** 0.5 * 57.29578
        tilt = max(abs(r), abs(p))
        return tilt, rate, (tilt < LEVEL_ANGLE and rate < LEVEL_RATE)
    except Exception:
        return 999.0, 999.0, False


def _recover(reason):
    """按 CROUCH 直接重新上力。不再走 poses 的缓动起点 (没有真实角度反馈)。"""
    global _latched, _sound_event, _level_count
    tilt, rate, level = _read_level()
    if not level:
        print("[fall_guard] recover refused: body not level (tilt=%.1f rate=%.0f)" %
              (tilt, rate))
        return False
    try:
        import poses
        bpuppy_motion.safety_rearm(poses.CROUCH)
        _rearm_ext()
        _latched = False
        _sound_event = False
        _level_count = 0
        # 重新上力后软件当前角就是 CROUCH; 这一步同时把 pose 范围置为静态。
        try:
            poses.crouch()
        except Exception as e:
            print("[fall_guard] crouch state update failed: %s" % e)
        print("[fall_guard] recovered to crouch (%s)" % reason)
        return True
    except Exception as e:
        print("[fall_guard] recover failed: %s" % e)
        return False


def _loop():
    global _running, _latched, _sound_event, _level_count

    while _running:
        try:
            # 持续维护“放平”计时, 供声音角度事件使用
            tilt, rate, level = _read_level()
            if level:
                if _level_count < _level_need:
                    _level_count += 1
            else:
                _level_count = 0

            latched_now = bpuppy_motion.safety_latched()
            if not _latched:
                if latched_now:
                    _latched = True
                    _sound_event = False
                    _level_count = 0
                    _stop_balance()
                    _stop_ext()
                    _beep("SND_YING")   # 嘤嘤
                    print("[fall_guard] RELEASED: servos off, waiting for sound angle")
            else:
                if not latched_now:
                    # 被外部 (例如 REPL safety_rearm) 恢复了
                    _latched = False
                    _sound_event = False
                elif _sound_event:
                    _sound_event = False
                    if _level_count >= _level_need:
                        _recover("sound angle")
                    else:
                        print("[fall_guard] sound angle ignored: body not level/stable")
        except Exception as e:
            print("[fall_guard] loop error: %s" % e)

        time.sleep_ms(PERIOD_MS)


def start():
    """打开 C 层检测并起恢复线程。import 时自动执行; 幂等。"""
    global _running, _enabled, _sound_event, _level_count

    if _running:
        # main.py 在 import 之后还会再调一次 start(); 已经开着就不要重复
        # safety_enable(True), 否则会重置武装计时并多打一行日志。
        if not _enabled:
            _enabled = True
            bpuppy_motion.safety_enable(True)
        return True

    try:
        if not bpuppy_imu.is_ready():
            bpuppy_imu.init(0, 14, 21, 0x68)
        if not bpuppy_imu.is_ready():
            _enabled = False
            print("[fall_guard] IMU not ready, protection disabled")
            return False
    except Exception as e:
        _enabled = False
        print("[fall_guard] IMU init failed: %s" % e)
        return False

    try:
        import voice
        voice.on_cmd(voice.CMD_SOUND_DIR, _on_sound)
    except Exception as e:
        print("[fall_guard] voice angle hook failed: %s" % e)

    # 运动任务平时只在 set_gait/motion.start() 时才创建; 站立保护也需要
    # 它的 50Hz 循环来跑 C 检测, 所以这里主动启动 (幂等, enabled 仍为 false)。
    try:
        bpuppy_motion.start()
    except Exception as e:
        print("[fall_guard] motion task start failed: %s" % e)

    _sound_event = False
    _level_count = 0
    _enabled = True
    bpuppy_motion.safety_enable(True)
    _running = True

    try:
        _thread.stack_size(8192)
        _thread.start_new_thread(_loop, ())
        _thread.stack_size(0)
    except Exception as e:
        _running = False
        _enabled = False
        bpuppy_motion.safety_enable(False)
        print("[fall_guard] thread start failed: %s" % e)
        return False

    print("[fall_guard] started (C 50Hz detector, sound-angle recovery)")
    return True


def on():
    """打开自动检测 (import 时已自动执行; 可重复调用)。"""
    return start()


def off():
    """只关自动检测; 手动 release/recover 和语音恢复仍然可用。"""
    global _enabled, _sound_event
    _enabled = False
    _sound_event = False
    try:
        bpuppy_motion.safety_enable(False)
    except Exception:
        pass
    print("[fall_guard] auto detection OFF (manual/voice recovery still active)")


def release():
    """手动进入保护: 锁存 + 停运动 + 关主舵机和 PWM_EXT 扩展舵机。不要求自动检测已打开。"""
    global _latched, _sound_event, _level_count
    if latched():
        print("[fall_guard] already latched")
        return True
    try:
        bpuppy_motion.safety_release()
        _stop_balance()
        _stop_ext()
        bpuppy_motion.safety_release()   # balance 最后一帧可能已写舵机
        _latched = True
        _sound_event = False
        _level_count = 0
        _beep("SND_YING")
        print("[fall_guard] manual RELEASE: servos off, waiting for recover/voice")
        return True
    except Exception as e:
        print("[fall_guard] release failed: %s" % e)
        return False


def recover():
    """手动从卧姿恢复控制: 要求已锁存且机身放平, 直接进入 CROUCH。"""
    if not latched():
        print("[fall_guard] not latched")
        return False
    return _recover("manual")


def rearm():
    """recover() 的别名。"""
    return recover()


def latched():
    try:
        return _latched or bpuppy_motion.safety_latched()
    except Exception:
        return _latched


def is_enabled():
    """自动检测是否打开 (不等于当前是否锁存)。"""
    return _enabled


def set(angle=60.0, rate=400.0, frames=3, level_angle=15.0, level_ms=300):
    """运行时改阈值 (C 检测参数 + Python 放平判据)。"""
    global ANGLE, RATE, FRAMES, LEVEL_ANGLE, LEVEL_MS, _level_need
    ANGLE = float(angle)
    RATE = float(rate)
    FRAMES = int(frames)
    LEVEL_ANGLE = float(level_angle)
    LEVEL_MS = int(level_ms)
    _level_need = max(1, LEVEL_MS // PERIOD_MS)
    try:
        bpuppy_motion.safety_config(ANGLE, RATE, FRAMES)
    except Exception as e:
        print("[fall_guard] config failed: %s" % e)
        return False
    print("[fall_guard] thresholds: angle=%.1f rate=%.0f frames=%d level=%.1f/%dms" %
          (ANGLE, RATE, FRAMES, LEVEL_ANGLE, LEVEL_MS))
    return True


# import / 启动即打开自动检测。off() 之后 MicroPython 不会重复执行模块,
# 所以这里只在第一次加载时生效; 之后再开用 on()。
if not _running:
    start()
