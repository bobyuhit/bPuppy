"""角度跟随 (step2) —— 转向闭环的**执行体**

    输入:  err_fn() → 当前偏差(度)     ← 谁来算 err 都行(方向锚定 / 视觉 / 声源…)
    输出:  bpuppy_motion.set_turn(...)

用法(阻塞跑; `/main.py` 本来就在后台线程里跑, 不挡 REPL):

    import heading_follow as flw
    flw.run(lambda: 某传感器.read_err())        # 场景1: 外部源
    flw.run(heading_anchor.anchor(30))          # 场景2: 转 30° 后保持

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
import time

import bpuppy_motion as m

_TWO_PI  = 6.283185307179586
_DEG2RAD = 0.017453292519943295

# ⚠⚠ **物理符号 —— 落地实测定的, 别凭直觉改**
#
#   实测 (2026-10-01, 落地走起来): **turn > 0 时 yaw 减小** ⇒ 狗往**右**拐。
#   而闭环要的是 `err = target − yaw > 0 ⇔ 需要 yaw 增大` ⇒ turn 必须取**负**。
#
#   凭什么信这个数: 两次落地实测都是 err **单调上升**(30→114 / 30→64.7)、无跳变;
#   而"走起来时 yaw 的自然漂移"实测只有 ~0.1°/s ⇒ 12°/s 的下降是**真转**不是磁干扰。
#   ⚠ 早先"目视看到向左拐"那个判断是**错的**(大概率是站位导致的左右混淆)。
#
#   ⇒ 这一行就是以前固件里的 `HEADING_TURN_SIGN`。**改它不用重编译** ——
#     这正是把闭环搬到 Python 的最大收益。
TURN_SIGN = -1.0

_i_term = 0.0        # 积分项, 单位 rad/周期 (与 G_c·turn 同域)


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
    """停掉闭环: 松开转向 (turn 回 0)。"""
    global _i_term
    _i_term = 0.0
    m.set_turn(0.0)


def run(err_fn, kp=0.6, ki=0.0, deadband=2.0, period_ms=50,
        i_limit=0.3, max_ticks=None, verbose=False):
    """阻塞跑闭环。默认**一直跑**, 直到被 Ctrl-C / 异常打断; 给 max_ticks 则跑够就返回。

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
               **调参/自检用** —— 在 REPL 里量几拍看看 turn 算得对不对, 不用 Ctrl-C 硬停。
    verbose    每拍打印一行, 调参时开。
    """
    global _i_term
    _i_term = 0.0
    n = 0

    while max_ticks is None or n < max_ticks:
        n += 1
        # ---- 取偏差 ----
        try:
            err = err_fn()
        except Exception as e:
            if verbose:
                print('heading_follow: err_fn 抛异常, 当无数据处理:', e)
            err = None
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

        if verbose:
            print('err=%7.2f  turn=%7.3f  G_c=%6.3f' % (err, turn, g))

        m.set_turn(turn)
        time.sleep_ms(period_ms)

    return n
