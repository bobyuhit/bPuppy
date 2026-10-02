"""角度跟随 (step2) —— 转向闭环的**执行体**

    输入:  err_fn() → 当前偏差(度)     ← 谁来算 err 都行(方向锚定 / 视觉 / 声源…)
    输出:  bpuppy_motion.set_turn(...)

    import heading_follow as flw

    flw.run(err_fn)      # 阻塞跑, 直到 max_ticks / stop() / Ctrl-C
    flw.start(err_fn)    # 丢后台线程跑, 立即返回
    flw.stop()           # 停: 置停止标志 + 松转向
    flw.running()        # 后台还在跑吗

⚠ 这个模块**不知道 err 从哪来** —— 它只认识"一个无参函数, 返回度数"。
  所以 IMU 锚定 / 视觉 / 声源角全都喂得进来, 这边一行都不用改。
  ⚠ 它也**不 import 任何 err 源** —— 连接由调用方(或 err 源那一侧)搭。

✨ `run()` 退出时**自动松转向**(`turn` 归零)。所以不管是跑够、被打断还是出错,
  都不会留下一只"还在拐"的狗。想继续用同一个 err 源就再跑一次。

────────────────────────────────────────────────────────────────
控制律 —— 为什么是 `turn = kp · err_rad / G_c`
────────────────────────────────────────────────────────────────
`set_turn()` 收的是**几何系数**(左右步长比), 不是"转多快"。同一个 turn=0.5:

    walk (speed 2.5) → 实际 0.59 rad/s
    trot (speed 6)   → 实际 1.35 rad/s      ← 差 2.3 倍

⇒ 同一套 kp, 在 trot 下等于 2.3 倍增益(walk 稳、trot 振, 或反过来)。
所以按**每周期**下指令, 并把几何换算掉:

    一个步态周期里, turn 这个几何系数能让航向转
        G_c · turn   [rad]        G_c = eff_stride / ((1 − eff_duty) · half_w)
                                  ↑ "每周期, turn=1 能转多少弧度"
    要求"每周期收掉 err 的 kp 比例" ⇒  G_c·turn = kp·err_rad
    ⇒  turn = kp · err_rad / G_c

⭐ **speed 和 omega 全都约掉了** —— 因为"每周期就是每周期", 与走多快无关。
   所以 kp 是个**无因次**的量:「每个步态周期收掉误差的百分之多少」, 默认 0.6。
   ⇒ **改速度不用重调 kp**, 换步态也不用。

⚠ `eff_stride`/`eff_duty` 必须用 **get_effective()** —— 那是 GO 覆盖+步长淡入淡出
  之后**腿循环真正在用**的值。用 get_params() 的读数算**在 GO 下会算错**
  (实测: 用户设 stride=30, GO 下实际用 70)。

⚠ kp 别超 1.0 —— "一周期收完"已经是这条机构允许的最快(每条腿一个周期才有一个
  落点事件), 再高只是要求它跑得比腿落地还快, 只会抖。

⚠ 归一化保证的是"**指令算出来的**转速一致"; 实际转到多少要看地面抓不抓得住
  (腿只能前后摆, 身体要转就得让脚横向搓) —— 实测滑移让实际只有模型的 ~1/3。
  ⇒ kp 要比"理论看起来对"的值大一些, 实测调。
"""
import _thread
import time

import bpuppy_motion as m

_BG_STACK = 16 * 1024      # 后台线程栈 (内部 RAM)。见 AGENTS.md 易错点 12:
                           # 设完**必须**还原成 0, 否则后面每个模块起线程都按这个尺寸要

_TWO_PI  = 6.283185307179586
_DEG2RAD = 0.017453292519943295

# ⚠⚠ **物理符号 —— 落地实测定的, 别凭直觉改**
#
#   turn > 0 = 狗往**右**拐 (俯视顺时针) ⇒ **yaw 增大**。
#   闭环要的是 `err = target − yaw > 0 ⇔ 需要 yaw 增大` ⇒ turn 取**正**。
#
#   ⚠ 取 −1 会变成**正反馈**: 狗越偏越往错方向冲, turn 饱和到 ±1。
#     别凭"右转 yaw 应该减小"的直觉改它 —— 本机 yaw 是**顺时针为正**。
#
#   ⇒ 这一行就是以前固件里的 `HEADING_TURN_SIGN`。**改它不用重编译** ——
#     这正是把闭环搬到 Python 的最大收益。
TURN_SIGN = 1.0

_i_term    = 0.0        # 积分项, 单位 rad/周期 (与 G_c·turn 同域)
_stop_flag = False      # stop() 置 True ⇒ 跑着的循环下一拍退出
_thread_id = None       # 后台线程在跑时非 None


def g_c():
    """本帧的 `G_c` = 一个步态周期里, turn=1 能让航向转多少弧度。

    拿不到有效值时返回 0.0(此时无法换算, 调用方按"转弯无意义"处理)。
    """
    try:
        eff_stride, eff_duty = m.get_effective()
        half_w = m.get_geometry()[3]
    except Exception:
        return 0.0
    denom = (1.0 - eff_duty) * half_w
    if denom < 1e-6 or eff_stride < 1e-6:
        return 0.0
    return eff_stride / denom


def stop():
    """停闭环: 置停止标志 + **松转向**(turn 归零) + 清积分项。

    **阻塞到后台线程真的退出**(最多 500ms) —— 不这样的话, 紧接着的 `start()`
    可能在旧线程还没看到标志时就把标志清回去, 变成两个循环同时写 turn。
    没有后台线程在跑时立即返回。
    """
    global _i_term, _stop_flag
    _stop_flag = True
    _i_term    = 0.0
    m.set_turn(0.0)
    t0 = time.ticks_ms()
    while _thread_id is not None and time.ticks_diff(time.ticks_ms(), t0) < 500:
        time.sleep_ms(10)


def running():
    """后台闭环还在跑吗。"""
    return _thread_id is not None


def start(err_fn, kp=0.6, ki=0.0, deadband=2.0, period_ms=50, i_limit=0.3):
    """在**后台线程**里跑闭环, 立即返回。已经在跑就先停掉再起(幂等)。

    ⚠ 后台跑的**Ctrl-C 停不住**(中断只投递主线程) —— 停它要用 `stop()`。
      好处是 REPL 空出来了, 你能随时敲 `stop()` / 看状态。
    """
    global _stop_flag, _thread_id
    stop()                          # 幂等; 已经在跑会等到旧线程真的退出 (见 stop 的说明)
    _stop_flag = False
    _thread.stack_size(_BG_STACK)
    try:
        _thread_id = _thread.start_new_thread(
            _bg, (err_fn, kp, ki, deadband, period_ms, i_limit))
    finally:
        _thread.stack_size(0)       # 0 = 还原端口默认 (esp32: 5KB)


def _bg(err_fn, kp, ki, deadband, period_ms, i_limit):
    global _thread_id
    try:
        _loop(err_fn, kp, ki, deadband, period_ms, i_limit, None)
    except Exception:
        pass                        # ⚠ 不打印 —— 控制模块不输出。异常只反映在"线程没了"
    finally:
        _thread_id = None


def run(err_fn, kp=0.6, ki=0.0, deadband=2.0, period_ms=50,
        i_limit=0.3, max_ticks=None):
    """阻塞跑闭环。三种收场: `max_ticks` 跑够 / 别的线程调 `stop()` / Ctrl-C 打断。

    err_fn     无参函数, 返回当前偏差(度)。返回 None 表示"这一拍没数据" ⇒ 当 0 处理
               (别拿着旧值一直转 —— 目标被遮挡 / 模块掉线时的基本保护)。
    kp         每个**步态周期**收掉误差的百分之多少。建议 0.5~0.8, **别超 1.0**。
               ⚠ 实测滑移让实际转角只有模型的 ~1/3, 所以可能需要比理论值大些。
    ki         积分增益 [1/周期], 默认 0(纯 P)。跟**移动**目标才需要 ——
               纯 P 追动目标有稳态滞后(滞后角 ≈ 目标每周期移动量 / kp)。
    deadband   死区(度)。带内不纠, 免得在收敛点附近来回蹭。
    period_ms  采样周期, 默认 50ms(20Hz)。
               ⚠ **20Hz 已经绰绰有余** —— 闭环固有带宽只有 ~0.8Hz(每半周期一次
                 "落点事件"), 喂再快也吸收不了。
    i_limit    积分项钳位 [rad/周期] (抗饱和)。0.3 约合"最多再补 17° 的额外转角"。
    max_ticks  跑够这么多拍就返回(返回实际拍数)。默认 None = 无限。

    ⇒ 返回跑过的拍数。**退出时一定把 turn 归零**(包括 Ctrl-C)。
    """
    global _stop_flag
    _stop_flag = False          # ⚠ 只有**顶层**调用才开闸 —— 理由见 _loop 的说明
    return _loop(err_fn, kp, ki, deadband, period_ms, i_limit, max_ticks)


def _loop(err_fn, kp, ki, deadband, period_ms, i_limit, max_ticks):
    """闭环本体。

    ⚠ **绝对不碰 `_stop_flag`。** 后台线程走的是这条路径, 闸门只能由 start()/stop()
      说了算。在这里清闸会和"start() 刚把线程起起来、stop() 紧接着就到"打架:
      新线程还没来得及进循环, 就把 stop() 设的标志擦掉 ⇒ **停不下来**, 而且
      `_thread_id` 永远回不到 None(`running()` 一直为真)。
      实测踩过: `set()` 重起之后紧接着 `off()`, 后台线程就没死掉。
    """
    global _i_term
    _i_term = 0.0
    n = 0

    try:
        while (max_ticks is None or n < max_ticks) and not _stop_flag:
            n += 1
            # ---- 取偏差 ----
            try:
                err = err_fn()
            except Exception:
                err = None      # err_fn 抛异常也当"这一拍没数据"。⚠ 不打印 —— 控制模块不输出
            if err is None:
                err = 0.0                      # 没数据 ⇒ 直走
            err = (err + 180.0) % 360.0 - 180.0     # ±180 回绕

            # ---- 换算系数 ----
            g = g_c()
            if g <= 0.0:
                _i_term = 0.0
                m.set_turn(0.0)
                time.sleep_ms(period_ms)
                continue

            # ---- 死区 ----
            err_c = 0.0 if abs(err) < deadband else err
            err_rad = err_c * _DEG2RAD

            # ---- PI: 两项都在"每周期转角"域 [rad/周期] ----
            _i_term += ki * err_rad
            if _i_term >  i_limit: _i_term =  i_limit
            if _i_term < -i_limit: _i_term = -i_limit
            turn = TURN_SIGN * (kp * err_rad + _i_term) / g
            if turn >  1.0: turn =  1.0
            if turn < -1.0: turn = -1.0

            m.set_turn(turn)
            time.sleep_ms(period_ms)
    finally:
        # ⭐ 无论怎么退出(跑够 / stop() / Ctrl-C / err_fn 炸了)都松转向 ——
        #    不然 turn 会冻在最后一拍, 狗保持那个转向一直拐。
        _i_term = 0.0
        m.set_turn(0.0)

    return n
