#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""换向审计 v2 —— 把「翻转帧的瞬移」和「之后的滑脚」分开, 并加对照组

对照组用来说明: 出问题的到底是
    (a) 取反四个锚定量 (翻转块本身), 还是
    (b) 方向一变, need_rev 从 false 翻 true ⇒ offset 从 fwd 切到 rev ⇒ 相位当帧跳变。

四组:
    negate            固件现状 (取反 + offset 切换)
    negate_nooffswap  取反, 但 offset 不跟着方向换 (仍用 offsets_fwd)
    none              不取反, 但 offset 切换 (方向切换本身)
    none_nooffswap    都不做  ← 基线, 应该全程正常
"""
import sys, math
sys.stdout.reconfigure(encoding='utf-8', errors='replace')

from stop_audit import (GaitState, go_params, all_legs_in_stance,
                        settle_to_walk, compute_offsets, DT, TWO_PI,
                        OMEGA_BASE, LEG_NAMES)


def clone(st):
    n = GaitState()
    n.phase = st.phase
    n.b = list(st.b); n.b_land = list(st.b_land)
    n.x_land = list(st.x_land); n.x_lift = list(st.x_lift)
    n.swing = list(st.swing); n.prev_fx = list(st.prev_fx)
    return n


def step(st, S, duty, gap, h, lift, speed, direction, offswap):
    """一帧腿循环, 可关掉 need_rev 的 offset 切换"""
    fphi = (OMEGA_BASE * speed) * DT / TWO_PI
    if offswap:
        offs = compute_offsets(duty, gap, direction)
    else:
        offs = compute_offsets(duty, gap, 1)
    sw_ratio = duty
    inv = 1.0 - duty
    for i in range(4):
        lp = st.phase + offs[i] * TWO_PI
        if lp > TWO_PI:
            lp -= TWO_PI
        pn = lp / TWO_PI
        in_sw = pn < sw_ratio
        prev_sw = st.swing[i]
        if in_sw and not prev_sw:
            b_at = st.b[i] - (S / inv) * pn
            st.x_lift[i] = st.x_land[i] - (b_at - st.b_land[i])
        elif (not in_sw) and prev_sw:
            over = (pn - sw_ratio) / inv
            st.b_land[i] = st.b[i] - S * over
            st.x_land[i] = S * 0.5
        st.swing[i] = in_sw
        if in_sw:
            t = pn / sw_ratio
            ease = t * t * (3.0 - 2.0 * t)
            x = st.x_lift[i] + (S * 0.5 - st.x_lift[i]) * ease
        else:
            x = st.x_land[i] - (st.b[i] - st.b_land[i])
        if direction < 0:
            x = -x
        st.prev_fx[i] = x
        st.b[i] += (S / inv) * fphi


def run(speed, mode, samples=63, frames=120):
    p = go_params(speed)
    duty, gap, S, h, lift = p['duty'], p['gap'], p['stride'], p['height'], p['lift']
    dphase = (OMEGA_BASE * speed) * DT
    negate = mode.startswith('negate')
    offswap = not mode.endswith('nooffswap')

    base = GaitState()
    settle_to_walk(base, speed, 12)

    rows = []
    for k in range(samples):
        st = clone(base)
        # 跑到第一个全踩地窗口 (旧方向)
        fp = None
        for _ in range(600):
            st.phase += dphase
            if st.phase > TWO_PI:
                st.phase -= TWO_PI
            if all_legs_in_stance(st.phase, duty, gap, 1):
                fp = st.phase
                break
            step(st, S, duty, gap, h, lift, speed, 1, True)
        if fp is None:
            continue
        # ★ 翻转帧 = 判定窗口的那一帧。固件里 取反 与 腿循环 在**同一帧**:
        #   先判 all_legs_in_stance (用旧方向), 再取反, 再用**本帧相位**跑腿循环。
        #   这里必须照做 —— 少了本帧的 step, 相位和 b 就差一帧, 取反抵消不掉。
        prev_x = list(st.prev_fx)
        prev_sw = list(st.swing)
        if negate:
            for i in range(4):
                st.b[i] = -st.b[i]; st.b_land[i] = -st.b_land[i]
                st.x_land[i] = -st.x_land[i]; st.x_lift[i] = -st.x_lift[i]

        step(st, S, duty, gap, h, lift, speed, -1, offswap)
        flip_jump = max(abs(st.prev_fx[i] - prev_x[i]) for i in range(4))
        slip_after = 0.0      # 之后支撑腿单帧位移
        net = [None] * 4
        sf = [None] * 4
        last_st = [None] * 4
        prev_sw = list(st.swing)
        prev_x = list(st.prev_fx)

        for f in range(1, frames):
            st.phase += dphase
            if st.phase > TWO_PI:
                st.phase -= TWO_PI
            step(st, S, duty, gap, h, lift, speed, -1, offswap)
            mx = max(abs(st.prev_fx[i] - prev_x[i]) for i in range(4))
            if f >= 2:
                for i in range(4):
                    if (not prev_sw[i]) and (not st.swing[i]):
                        d = abs(st.prev_fx[i] - prev_x[i])
                        if d > slip_after:
                            slip_after = d
            for i in range(4):
                if (not prev_sw[i]) and (not st.swing[i]):
                    last_st[i] = st.prev_fx[i]
                if st.swing[i] and not prev_sw[i]:
                    sf[i] = last_st[i]
                if (not st.swing[i]) and prev_sw[i] and net[i] is None and sf[i] is not None:
                    net[i] = st.prev_fx[i] - sf[i]
            prev_sw = list(st.swing)
            prev_x = list(st.prev_fx)
        rows.append((fp, flip_jump, slip_after, net))
        base.phase += dphase
        if base.phase > TWO_PI:
            base.phase -= TWO_PI
        step(base, S, duty, gap, h, lift, speed, 1, True)
    return rows, S


def summarize(speed, mode):
    rows, S = run(speed, mode)
    if not rows:
        print('%-18s 无样本' % mode)
        return
    fj = [r[1] for r in rows if r[1] is not None]
    sa = [r[2] for r in rows]
    devs = []
    for _, _, _, net in rows:
        if all(n is not None for n in net):
            devs.append(max(abs(n - (-S)) for n in net))
    print('%-18s 翻转帧跳变 max=%6.2f mm | 之后滑脚 max=%6.2f mm | '
          '摆动净位移偏差 max=%6.2f mm  (期望净位移 %+.0f)'
          % (mode, max(fj) if fj else -1, max(sa), max(devs) if devs else -1, -S))
    return rows, S


print('=' * 100)
print('speed 2.5 (go: duty=0.20 gap=0.04 stride=70)  —— 全踩地窗口内 64 个翻转相位')
print('=' * 100)
for m in ('none_nooffswap', 'none', 'negate_nooffswap', 'negate'):
    summarize(2.5, m)

print()
print('=' * 100)
print('明细: mode=negate (固件现状), 按翻转相位排序')
print('=' * 100)
rows, S = run(2.5, 'negate')
rows.sort(key=lambda r: r[0])
print('%9s | %8s | %8s | %s' % ('翻转相位', '翻转帧跳', '之后滑脚', '摆动净位移 LF/LH/RF/RH'))
print('-' * 100)
for fp, fj, sa, net in rows:
    print('%8.1f° | %8.2f | %8.2f | %s'
          % (fp / TWO_PI * 360.0, fj if fj is not None else -1, sa,
             ' '.join('%+7.1f' % n if n is not None else '    n/a' for n in net)))
