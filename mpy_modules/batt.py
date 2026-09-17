"""
bPuppy 电池标定工具 — 最小二乘线性拟合 (PC/REPL 侧用, 不是运行时读数接口)

基于多点实测 (万用表 vs 程序显示) 做最小二乘拟合: 实际 = a × 显示 + b。
校正 ADC 量程 / 分压电阻偏差。

⚠ 本模块是**标定工具**, 不是运行时读数接口。板子上真正生效的标定在
   C 层 led_driver.c (系数存 NVS)。日常读电压请用:

       import bpuppy_led
       bpuppy_led.batt_v()        # 已标定电压 (V), 与 LED 颜色同一条路径
       # 或 frozen/voltage.py 的 voltage.read_v() (同一来源的薄封装)

   本模块的 read_batt_v() 只是工具自己按本地 CAL_POINTS 算的值, 用于
   写回板子**之前**预览 —— apply_to_board() 之后两者才会一致。

用法:
    import batt
    batt.init()                    # 初始化 ADC (GPIO3 = ADC1_CH2)
    r = batt.raw_v()               # 未标定显示值 (V), 与万用表对照
    batt.read_batt_v()             # 按本地拟合算的电压 (V) — 预览用

标定流程 (换板子后重新采集):
    1. 万用表量实际电压, 与 batt.raw_v() 对照, 记下成对的值
    2. batt.add_cal_point(万用表V)   # 自动取当前 raw_v() 作显示值; 可多次
       (或直接编辑下方 CAL_POINTS)
    3. batt.verify()                # 自查拟合残差
    4. batt.apply_to_board()        # ★ 写进板子 NVS — 此后 LED 和读数都用新系数
       失败会抛 OSError, 系数本次仍生效但重启后丢失

撤销: bpuppy_led.reset_cal() 清空 NVS, 恢复出厂默认系数。
"""

import bpuppy_adc

# 实测标定点 (程序显示 V, 万用表 V) — 2026-08-19 采集
CAL_POINTS = [
    (5.50, 6.17),
    (6.27, 7.00),
    (7.21, 7.90),
    (7.80, 8.59),
]

DIVIDER = 6.1   # 51k/10k 分压换算: read_mv() × DIVIDER / 1000 → 显示值 (V)

_fit = None     # (a, b) 拟合系数缓存


def init():
    """初始化 ADC (V3.0: GPIO3 = ADC1_CH2)"""
    bpuppy_adc.init()


def raw_v():
    """未标定程序显示值 (V) — 采集新标定点时与万用表对照"""
    init()
    mv = bpuppy_adc.read_mv()
    return mv * DIVIDER / 1000.0 if mv >= 0 else -1.0


def _least_squares(pts):
    """最小二乘 y = a·x + b → (a, b)"""
    n = len(pts)
    if n == 0:
        return (1.0, 0.0)          # 无标定点: 显示值即结果
    mx = sum(p[0] for p in pts) / n
    my = sum(p[1] for p in pts) / n
    sxx = sum((p[0] - mx) ** 2 for p in pts)
    sxy = sum((p[0] - mx) * (p[1] - my) for p in pts)
    if sxx == 0:
        # 所有 x 相同 (通常=只有 1 个标定点) → 斜率无从确定, 取 a=1 只修偏置:
        # 单点标定能提供的全部信息就是"偏了多少", 增益本来就测不出来。
        # ⚠ 绝不能返回 a=0 —— 那会让 batt_v() 恒等于 b, 电池从满用到空读数都不变,
        #    LED 颜色也永不变化。这是个静默失效, 看不出来。
        return (1.0, my - mx)
    a = sxy / sxx
    return (a, my - a * mx)


def fit():
    """重新计算拟合系数 (a, b) 并返回"""
    global _fit
    _fit = _least_squares(sorted(CAL_POINTS))
    return _fit


def read_batt_v():
    """已标定的电池电压 (V)"""
    global _fit
    r = raw_v()
    if r < 0:
        return -1.0
    if _fit is None:
        _fit = _least_squares(sorted(CAL_POINTS))
    a, b = _fit
    return a * r + b


def add_cal_point(actual_v, reported_v=None):
    """追加标定点并重拟合: actual_v=万用表读数, reported_v=当前 raw_v() (缺省自动取)"""
    global _fit
    if reported_v is None:
        reported_v = raw_v()
    CAL_POINTS.append((reported_v, actual_v))
    CAL_POINTS.sort()
    _fit = None                    # 失效缓存, 下次自动重算
    return list(CAL_POINTS)


def clear_cal_points():
    """清空标定点 (含出厂那 4 个)。

    ⚠ 给**新板子**标定前必须先调这个 —— add_cal_point() 是追加,
    不清空会把出厂点 (属于另一块板) 和新点混在一起拟合。
    清空后 CAL_POINTS 为空, _least_squares 退化成 (1.0, 0.0), 即暂时不做标定。
    """
    global _fit
    CAL_POINTS.clear()
    _fit = None
    print("标定点已清空 (拟合退化为 a=1.0 b=0.0, 即不标定)")
    return list(CAL_POINTS)


def verify():
    """打印每个标定点的拟合结果与残差"""
    a, b = _least_squares(sorted(CAL_POINTS))
    for x, y in sorted(CAL_POINTS):
        yf = a * x + b
        print("显示 %.2fV → 拟合 %.2fV (万用表 %.2fV) 残差 %+.3fV"
              % (x, yf, y, yf - y))
    print("系数: a=%.4f  b=%.4f" % (a, b))


def apply_to_board(a=None, b=None):
    """把拟合系数写进板子 NVS —— 这一步之后 LED 和读数才用新系数。

    不传参数则用当前 CAL_POINTS 的拟合值。
    写入后由 C 层 led_driver.c 生效 (监控任务下一拍即用), 掉电保留、固件升级不丢。
    """
    import bpuppy_led
    if a is None or b is None:
        a, b = fit()
    bpuppy_led.set_cal(a, b)      # NVS 写失败会抛 OSError
    print("已写回板子: a=%.4f b=%.4f (bpuppy_led.batt_v() 现在用新系数)" % (a, b))
    return (a, b)
