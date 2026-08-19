"""
bPuppy 电池电压读取 — 最小二乘线性拟合标定

基于多点实测 (万用表 vs 程序显示) 最小二乘拟合: 实际 = a × 显示 + b。
校正 ADC 量程 / 分压电阻偏差。比分段插值更平滑, 抗采集噪声。

用法:
    import batt
    batt.init()                    # 初始化 ADC (GPIO3 = ADC1_CH2)
    v = batt.read_batt_v()         # 已标定的电池电压 (V)
    r = batt.raw_v()               # 未标定显示值 (V), 采集新标定点时对照
    batt.verify()                  # 对照标定点自查 (拟合残差)

标定数据更新 (换电池/换机/重新采集):
    batt.add_cal_point(6.0)        # 万用表 6.0V, 自动取当前 raw_v() 作显示值
    # 或直接编辑下方 CAL_POINTS 后执行 batt.verify()
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
        return (0.0, my)           # 所有 x 相同 → 取平均
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


def verify():
    """打印每个标定点的拟合结果与残差"""
    a, b = _least_squares(sorted(CAL_POINTS))
    for x, y in sorted(CAL_POINTS):
        yf = a * x + b
        print("显示 %.2fV → 拟合 %.2fV (万用表 %.2fV) 残差 %+.3fV"
              % (x, yf, y, yf - y))
    print("系数: a=%.4f  b=%.4f" % (a, b))
