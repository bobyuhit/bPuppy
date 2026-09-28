#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""复刻 traj_combo_bad(), 扫出"默认参数下姿态能压多大"

d = √(x² + z²) 必须 ≤ L1+L2-1 = 40+45-1 = 84
    x, z 是**叠加姿态补偿后**的足端位置 (补偿公式与固件同一套)
    取 5 个 ease 采样点 × 4 条腿的最坏值
"""
import sys, math
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

BHL, BHW = 62.5, 59.0
L1, L2 = 40.0, 45.0
D_MAX = L1 + L2 - 1.0          # 84
LEFT = [True, True, False, False]

STRIDE, HEIGHT, LIFT, CENTER = 70.0, 70.0, 30.0, 0.0
EASE_PTS = (0.0, 0.25, 0.5, 0.75, 1.0)


def worst_d(pitch, roll, mode='new'):
    x = abs(STRIDE) * 0.5
    z_roll = BHW * math.tan(math.radians(roll))
    tan_p = math.tan(math.radians(pitch))
    zp_const = BHL * math.tan(math.radians(pitch))
    worst = 0.0
    for ease in EASE_PTS:
        xv = -x + 2.0 * x * ease + CENTER
        zv = HEIGHT - LIFT * math.sin(ease * math.pi)
        for k in range(4):
            base = zv + (-z_roll if LEFT[k] else z_roll)
            hx = BHL if k in (0, 2) else -BHL
            if mode == 'new':
                zf = base + (hx + xv) * tan_p       # 足端在机身系里的绝对 x
            else:                                   # 旧: 按前后腿分档的常量
                zf = base + (zp_const if k in (0, 2) else -zp_const)
            if zf < 0.0:
                continue                            # 入地由另一条判据报
            d = math.sqrt(xv * xv + zf * zf)
            if d > worst:
                worst = d
    return worst


def limit(axis, mode='new'):
    """二分找边界: 该轴单独能压到多少度 (d 恰好 = 84)"""
    lo, hi = 0.0, 45.0
    for _ in range(60):
        mid = (lo + hi) * 0.5
        d = worst_d(mid, 0.0, mode) if axis == 'pitch' else worst_d(0.0, mid, mode)
        if d <= D_MAX:
            lo = mid
        else:
            hi = mid
    return lo


print('默认参数: stride=%.0f height=%.0f lift=%.0f center=%.0f  (L1=%.0f L2=%.0f, d 上限 %.0f)'
      % (STRIDE, HEIGHT, LIFT, CENTER, L1, L2, D_MAX))
print('=' * 72)
print('%-22s %-14s %-14s' % ('', '旧公式(前后分档)', '新公式(按各自 x)'))
print('%-22s %-14s %-14s'
      % ('俯仰 |pitch| 上限', '%.2f°' % limit('pitch', 'old'), '%.2f°' % limit('pitch', 'new')))
print('%-22s %-14s %-14s'
      % ('横滚 |roll| 上限', '%.2f°' % limit('roll', 'old'), '%.2f°' % limit('roll', 'new')))
print()

# 顺带看几个档位的 stride / height
print('换几组 stride / height 看上限怎么变 (新公式, 俯仰):')
print('%-28s %-10s' % ('', '|pitch| 上限'))
for s, h, l in ((70, 70, 30), (60, 70, 30), (50, 70, 30), (40, 70, 30),
                (70, 60, 20), (70, 70, 5), (50, 70, 5)):
    STRIDE, HEIGHT, LIFT = float(s), float(h), float(l)
    print('  stride=%-4d height=%-4d lift=%-4d    %.2f°' % (s, h, l, limit('pitch', 'new')))
STRIDE, HEIGHT, LIFT = 70.0, 70.0, 30.0

print()
print('俯仰上限 vs stride / height  (抬脚固定 30mm; 表内是"够得着"这一条约束)')
print('%-10s' % '', end='')
HS = (20, 30, 40, 50, 60, 70)
for h in HS:
    print('%9s' % ('h=%d' % h), end='')
print()
for s in (0, 10, 20, 30, 40, 50, 60, 70):
    print('%-10s' % ('s=%d' % s), end='')
    for h in HS:
        STRIDE, HEIGHT = float(s), float(h)
        print('%8.1f°' % limit('pitch', 'new'), end='')
    print()
STRIDE, HEIGHT = 70.0, 70.0
print()
print('注: height 很小时还要过"足端入地"那一关 (height - lift < 0 直接拒), 表里没体现。')
