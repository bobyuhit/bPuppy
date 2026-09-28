#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""支撑三足平面 vs 机身平面 —— 全量扫描 (pitch / roll 各自与组合)

量: 三足平面在**机身坐标系**里的 (pitch, roll) 两个**带符号**分量。
    补偿正确时, 它就恒等于"机身相对地面的姿态" = (pitch设定值, roll设定值)。
    走路一周期里的**摆幅** = 狗身被周期性顶歪的幅度。

三种补偿:
    now      fz = h ± bhl·tan(p) ∓ bhw·tan(r)   现状: 按"前/后腿"分两档的常量
    fx_tan   fz = h + fx·tan(p) - fy·tan(r)     按该腿自己的 fx (用 tan)
    fx_sin   fz = h + fx·sin(p) - fy·sin(r)     按该腿自己的 fx (用 sin, 从旋转矩阵推出来的正确形式)
"""
import sys, math
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

from stop_audit import (GaitState, go_params, settle_to_walk, advance_legs,
                        DT, TWO_PI, OMEGA_BASE, LEG_NAMES, compute_offsets)

BHL, BHW = 62.5, 59.0
FY = [BHW, BHW, -BHW, -BHW]
IS_FRONT = [True, False, True, False]
# ★ 髋在**机身坐标系**里的 x。腿循环算出的 foot_x 是**相对髋**的,
#   而平面方程用的是**绝对** x ⇒ 必须加上这个, 否则站着 (fx=0) 时补偿会整个消失。
HX = [BHL, -BHL, BHL, -BHL]


def plane_angles(pts):
    (x1, y1, z1), (x2, y2, z2), (x3, y3, z3) = pts
    ux, uy, uz = x2 - x1, y2 - y1, z2 - z1
    vx, vy, vz = x3 - x1, y3 - y1, z3 - z1
    nx = uy * vz - uz * vy
    ny = uz * vx - ux * vz
    nz = ux * vy - uy * vx
    ln = math.sqrt(nx * nx + ny * ny + nz * nz)
    if ln < 1e-9:
        return None
    nx, ny, nz = nx / ln, ny / ln, nz / ln
    if nz < 0.0:
        nx, ny, nz = -nx, -ny, -nz
    if abs(nz) < 1e-9:
        return None
    return (math.degrees(math.atan2(-nx, nz)),
            math.degrees(math.atan2(ny, nz)))


def fz_of(mode, i, fx, h, p_rad, r_rad):
    tp, tr = math.tan(p_rad), math.tan(r_rad)
    if mode == 'now':
        # 原始: 按"前 / 后腿"分档的常量 (= 假设足端恒在髋正下方)
        return h + (BHL * tp if IS_FRONT[i] else -BHL * tp) - FY[i] * tr
    if mode == 'myfix':
        # 我上一版误改的: 用**相对髋**的 fx ⇒ 站着 (fx=0) 补偿归零
        return h + fx * tp - FY[i] * tr
    if mode == 'absolute':
        # 正确: 用**足端在机身系里的绝对 x** = 髋的 x + 相对 x
        return h + (HX[i] + fx) * tp - FY[i] * tr
    raise ValueError(mode)


MODES = ('now', 'myfix', 'absolute')


def run(pitch_deg, roll_deg, speed=2.5, cycles=2):
    p = go_params(speed)
    duty, gap, S = p['duty'], p['gap'], p['stride']
    h, lift = p['height'], p['lift']
    p_rad, r_rad = math.radians(pitch_deg), math.radians(roll_deg)

    st = GaitState()
    settle_to_walk(st, speed, 12)
    dphase = (OMEGA_BASE * speed) * DT
    offs = compute_offsets(duty, gap, 1)

    out = {m: [] for m in MODES}
    n = int(cycles * (TWO_PI / (OMEGA_BASE * speed)) / DT)
    for _ in range(n):
        st.phase += dphase
        if st.phase > TWO_PI:
            st.phase -= TWO_PI
        advance_legs(st, S, duty, gap, h, lift, speed, 1)
        sw = []
        for i in range(4):
            lp = st.phase + offs[i] * TWO_PI
            if lp > TWO_PI:
                lp -= TWO_PI
            sw.append(lp / TWO_PI < duty)
        supp = [i for i in range(4) if not sw[i]]
        if len(supp) != 3:
            continue
        for m in MODES:
            pts = [(HX[i] + st.prev_fx[i], FY[i],
                    fz_of(m, i, st.prev_fx[i], h, p_rad, r_rad)) for i in supp]
            r = plane_angles(pts)
            if r:
                out[m].append(r)
    return out


CASES = [
    ('pitch +10, roll  0', 10.0, 0.0),
    ('pitch -10, roll  0', -10.0, 0.0),
    ('pitch   0, roll +10', 0.0, 10.0),
    ('pitch   0, roll -10', 0.0, -10.0),
    ('pitch +10, roll +10', 10.0, 10.0),
    ('pitch -10, roll -10', -10.0, -10.0),
]

print('=' * 100)
print('三足平面在机身系里的 (pitch, roll) 角 —— 目标就是"设定值本身", 摆幅 = 狗身被周期性顶歪的幅度')
print('=' * 100)

for label, pd, rd in CASES:
    res = run(pd, rd)
    print()
    print('【%s】   目标 (pitch %+.1f°, roll %+.1f°)' % (label, pd, rd))
    print('   %-8s %-34s %-34s' % ('', 'pitch 分量  范围 / 摆幅', 'roll 分量  范围 / 摆幅'))
    for m in MODES:
        rows = res[m]
        if not rows:
            continue
        ps = [r[0] for r in rows]
        rs = [r[1] for r in rows]
        ok = (max(abs(min(ps) - pd), abs(max(ps) - pd)) < 0.5 and
              max(abs(min(rs) - rd), abs(max(rs) - rd)) < 0.5)
        mark = '✅' if ok else '❌'
        print('   %-8s %+7.2f°~%+7.2f° 摆幅%6.2f°   %+7.2f°~%+7.2f° 摆幅%6.2f°  %s'
              % (m, min(ps), max(ps), max(ps) - min(ps),
                 min(rs), max(rs), max(rs) - min(rs), mark))
