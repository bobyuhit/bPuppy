"""方向锚定 (step1) —— IMU 航向闭环

**它自己 import heading_follow 并把闭环接上** —— 用户不需要看见 step2。
用户日常就这几下:

    import heading_anchor as anc

    anc.on()        # 打开: 保持当前朝向 (闭环在后台跑起来, 立即返回)
    anc.on(30)      # 打开: 转 30° 后保持
    anc.read()      # 读状态: (是否打开, 目标角, 当前yaw, 偏差)
    anc.off()       # 关闭: 停闭环 + 松转向

⭐ **行进中照样能调** —— 已经在跑的时候 on() 只是**换个目标**, 闭环下一拍自己跟着走
  (不重起线程、turn 不掉 0)。所以"走着走着改主意, 再转 30°"就再敲一次 anc.on(30)。

要调参数才用得着 set() —— **全都有缺省, 不设也能跑**:

    anc.cfg                        # 看当前参数
    anc.set(kp=2.5)                # 改一个
    anc.set(deadband=3.0, period_ms=40)
    anc.set()                      # 无参 = 全部恢复缺省

★ on() 收的是**角度量**("要转多少"), 不是绝对航向 —— 调用方不用知道当前朝几度,
  ±180 回绕在这边统一处理。

★ on() 把 IMU 的 **init 和收敛等待**都包在里面了 —— 冷启动时 yaw 要 ~2.5s 才稳,
  不等就会把一个还没收敛的假值锁成目标(实测偏差能到 100°)。调用方不用管。

⚠ 闭环跑在**后台线程**里 ⇒ **Ctrl-C 停不住它**(中断只投递主线程)。停它用 off()。
  好处是 REPL 空着, 你随时能敲 off() / read()。

⚠ 本模块**不打印**。要读状态用 read(); 要打点自己在调用方打。
"""
import time

import bpuppy_imu as imu

import heading_follow as flw        # ← step1 连接 step2 的地方

_I2C_PORT = 0
_SDA_PIN  = 14
_SCL_PIN  = 21
_ADDR     = 0x68

# 等姿态收敛的判据: 连续 _SETTLE_HITS 拍、每拍(间隔 _SETTLE_POLL_MS)变化都小于
# _SETTLE_TOL 度 ⇒ 算稳。_SETTLE_MIN_MS 是下限保护: 万一 IMU 任务还没起来、
# read_angles() 恒返回一个死值, 光看"变化小"会 300ms 就误判成收敛。
_SETTLE_TOL        = 0.5
_SETTLE_HITS       = 3
_SETTLE_POLL_MS    = 100
_SETTLE_MIN_MS     = 1000
_SETTLE_TIMEOUT_MS = 6000

# 快判 —— 只在开机后**第一次** on() 跑一下, 决定要不要走上面那套完整等待。
# ⭐ 判据是**净漂移**而不是瞬时变化率: 走路时 yaw 也在摆(±2.4°), 但那是**对称摆动**,
#    一个 _PROBE_MS 窗口下来净值 ≈0; 而冷启动的姿态收敛是**单调爬升**,
#    同样长的窗口净漂 ~20°(实测 30~48°/s)。两者差一个数量级, 判据很干净。
# 为什么要它: 光看 `is_ready()` 是不够的 —— 程序开头预初始化过 IMU 的话它是 True,
#    但滤波可能才刚起步, 锁一个还在爬的 yaw ⇒ 闭环追着这个漂移把狗转过去(静默错)。
_PROBE_MS      = 600
_PROBE_POLL_MS = 100
_PROBE_DRIFT   = 5.0

# ── 可调参数: 全部有缺省, 不设也能跑 ──────────────────────────────
# ⚠ 这份是**缺省值**, 用户改的是下面的 cfg。`anc.set()` 无参 = 把 cfg 拉回这份。
_DEF = {
    'kp':        2.0,   # 每个步态周期收掉误差的比例。地面滑移 ~1/3, 所以比理论值放大
    'ki':        0.0,   # 积分; 只有跟**移动**目标才需要
    'deadband':  2.0,   # 死区(度)。走路时 yaw 本身在 ±2.4° 摆, 比这小就是拿噪声当误差
    'period_ms': 50,    # 采样周期(20Hz)。闭环带宽只有 ~0.8Hz, 再快也吸收不了
    'i_limit':   0.3,   # 积分项抗饱和
}
cfg = {}
cfg.update(_DEF)

_on     = False
_target = 0.0
_warm   = False     # IMU 的姿态滤波收敛过了吗 —— 只有开机后**第一次** on() 才需要等


def on(offset_deg=0.0):
    """打开锚定 / **换目标**: 目标 = **调用这一刻**的 yaw + offset_deg(要转多少, 不是转到几度)。

    **不传参就是 0** ⇒ 保持当前朝向。用的参数是 `anc.cfg`(缺省即可, 不用先设)。

    ⭐ **行进中随时可以调。** 已经在跑的话这里只做两件事: 重新锁目标 + 什么都不打断 ——
      跑着的闭环下一拍自己就会读到新目标。**不重起线程, 所以 turn 不会掉一下 0**,
      狗是平滑地把弯拉过去, 不是"先直一下再拐"。

    首次调用会顺手 init IMU(幂等), 然后**先快判再决定要不要等**:
      · 姿态早就稳了(程序开头预初始化过 IMU) ⇒ 只花 _PROBE_MS, 立刻锁目标
      · 还在冷启动爬升                      ⇒ 走完整等待(实测 ~4.5s), **绝不锁没收敛的 yaw**
    之后所有调用**都是秒回**(走路时 yaw 一直在摆, 再判也没意义)。

    返回 True/False = 锁目标的时候 IMU 是稳的吗(REPL 会回显, 免得模块自己打印)。
    """
    global _on, _target, _warm
    imu.init(_I2C_PORT, _SDA_PIN, _SCL_PIN, _ADDR)     # 幂等
    ok = True
    if not _warm:
        # ⚠ 只有开机后**第一次**才判 —— 之后绝不能再判: 走路时 yaw 天然摆 ±2.4°,
        #   "连续几拍变化小"这类判据在行进中永远不成立, 每次 on() 都会卡满超时。
        # ⚠ 也**不能只看 is_ready()**: 程序开头预初始化过 IMU 的话它是 True, 但滤波可能
        #   才刚起步 —— 那样会锁一个还在爬的 yaw, 闭环追着这个漂移把狗转过去, 静默错。
        #   所以先快判净漂移, 不稳才退回完整等待。
        ok = _settled_fast() or _settle()
    _warm = True
    _target = _yaw() + offset_deg
    _on = True
    if not flw.running():          # 已在跑 ⇒ 只换目标, 别碰线程
        flw.start(err, **cfg)      # ← 接上 step2
    return ok


def off():
    """关闭锚定: 停掉 step2 的闭环 + **松转向**(turn 归零)。

    ⚠ 只管这一头 —— 狗还在走, 只是不再纠方向。要停狗是 `set_gait("stop")`。
    """
    global _on
    _on = False
    flw.stop()


def read():
    """读状态: (是否打开, 目标角度, 当前yaw, 偏差)。

    没打开 ⇒ 偏差是 None; IMU 没就绪 ⇒ 后三个都是 None。

    ⚠ 报出去的目标角是**折到 ±180 的** —— 内部 `_target` 存的是没折的原值
      (yaw_at_on + offset, 可以越过 180)。跨过 ±180 后不折会显示成
      `(True, 189.0, -171.0, 0.0)`: 数学没错(189 ≡ −171), 但那个 189 看着像越界;
      折了就是 `(True, -171.0, -171.0, 0.0)`, 一眼"到位了"。
      **只影响这一行的输出** —— 控制律从不读 `_target` 的原值(`err()` 里本来就
      `_wrap(差)`), 而"当初要的是 +189、走的穿过 180 那条短路"这个信息, `err` 的
      符号已经给了。
    """
    t = _wrap(_target)
    if not imu.is_ready():
        return (_on, t, None, None)
    y = _yaw()
    return (_on, t, y, _wrap(_target - y) if _on else None)


def set(**kw):
    """调参数 —— **全部有缺省, 不调也能跑**。无参调用 = 全部恢复缺省。

    可调: kp / ki / deadband / period_ms / i_limit。当前值读 `anc.cfg`。
    已经打开的话**立即生效**: 闭环拿新参数重起, **目标不变**(不重新锚定)。
    """
    if not kw:
        cfg.clear()
        cfg.update(_DEF)
    else:
        for k in kw:
            if k not in _DEF:
                raise ValueError('没有这个参数: %s (可调 %s)' % (k, ', '.join(_DEF)))
            cfg[k] = float(kw[k])
    if _on:
        flw.start(err, **cfg)      # 重起闭环; 目标不动


def err():
    """闭环每拍调这个取偏差(度) = 目标 − 当前yaw, ±180 回绕。**内部接口**。

    返回 None = "这拍没数据" ⇒ step2 当 0 处理 ⇒ 直走。
    ⚠ 没打开、或 IMU 没就绪时都返回 None。不这么拦的话: **未 init 时
      read_angles() 恒返回 (0,0,0)**, 闭环会把这个假 0 当成"当前朝向 0 度",
      一直输出恒定转角转到天荒地老 —— 实测踩过(err 恒 30 跑了整整 6 秒)。
    """
    if not _on or not imu.is_ready():
        return None
    return _wrap(_target - _yaw())


# ── 以下内部 ────────────────────────────────────────────────────────

def _yaw():
    """当前航向 (度, ±180)。⚠ 未 init/未就绪时恒返回 0, 不能当读数用。"""
    return imu.read_angles()[2]


def _wrap(a):
    """把角度折到 ±180。"""
    return (a + 180.0) % 360.0 - 180.0


def _settled_fast():
    """快判姿态稳不稳: 采 _PROBE_MS 一个窗口, 看 yaw 的**净漂移**大不大。

    稳 ⇒ True(滤波早就收敛了, 不用等); 还在爬 ⇒ False(交给 _settle 慢慢等)。
    ⚠ 判据用净漂移不用瞬时抖动 —— 理由见顶部 _PROBE_* 那段的注释。
    """
    if not imu.is_ready():
        return False
    y0 = _yaw()
    t0 = time.ticks_ms()
    while time.ticks_diff(time.ticks_ms(), t0) < _PROBE_MS:
        time.sleep_ms(_PROBE_POLL_MS)
    return abs(_wrap(_yaw() - y0)) < _PROBE_DRIFT


def _settle():
    """等姿态收敛 ⇒ True; 超时 ⇒ False。

    IMU 任务和 Mahony 滤波都在后台自己跑, 这里只做"看它稳了没"的判据, 不驱动它。
    """
    t0 = time.ticks_ms()
    end = time.ticks_add(t0, _SETTLE_TIMEOUT_MS)
    prev, hits = None, 0
    while time.ticks_diff(end, time.ticks_ms()) > 0:
        if imu.is_ready():
            y = _yaw()
            if prev is not None and abs(_wrap(y - prev)) < _SETTLE_TOL:
                hits += 1
                if hits >= _SETTLE_HITS and time.ticks_diff(time.ticks_ms(), t0) >= _SETTLE_MIN_MS:
                    return True
            else:
                hits = 0
            prev = y
        time.sleep_ms(_SETTLE_POLL_MS)
    return False
