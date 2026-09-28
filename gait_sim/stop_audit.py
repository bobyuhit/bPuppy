#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
停步残留审计 — 离线复刻固件 (drivers/motion_task.cpp) 的逐帧计算,
扫「淡出窗口 FADE_OUT_TURNS」对停步残留的影响。

尺子: 切 GAIT_STOP 那一刻 max|足端 x| (机体系, mm)。
      锚定规则下, 脚一落地世界坐标就定死, 落点 x_land = 步长/2 ——
      所以「残留」= 四只脚离站姿点 x=0 的最大距离, 也就是随后 0.3s 停步缓动
      (pose_trans=2) 必须**拖着地面**走完的距离。
      除以 0.3s 就是搓地速度。作者的判据: 39~40mm -> 130mm/s 是"搓地",
      2.2~4.0mm -> 13mm/s 可接受。

只复刻轨迹层 (足端 x), 不复刻 IK / 舵机角 —— 残留是纯几何量。

对应固件版本: 3407771 之后的 motion_task.cpp (支撑相锚定 + 平方淡出)。
"""

import math
import sys

sys.stdout.reconfigure(encoding='utf-8', errors='replace')

DT = 0.02                     # MOTION_LOOP_MS = 20
TWO_PI = 2.0 * math.pi
OMEGA_BASE = 2.0              # ik.h: OMEGA_DEFAULT
GO_LIFT_LOW = 30.0            # motion_task.cpp 顶部常量
GO_LIFT_HIGH = 5.0
STOP_TRANS_TIME = 0.3         # TRANS_TIME: 切静态后的缓动时长

# 腿顺序与固件 leg_hip_ch = {0,2,4,6} 一致: LF, LH, RF, RH
LEG_NAMES = ['LF', 'LH', 'RF', 'RH']


# ---------------------------------------------------------------- 固件公式

def go_params(speed):
    """motion_task.cpp 的 GO 分支: speed -> duty/gap/stride/height/lift"""
    s = speed
    if s <= 4.0:
        return dict(duty=0.20, gap=0.04, stride=70.0, height=70.0, lift=GO_LIFT_LOW)
    if s >= 6.0:
        return dict(duty=0.40, gap=0.10, stride=50.0, height=70.0, lift=GO_LIFT_HIGH)
    t = (s - 4.0) / 2.0
    return dict(duty=0.20 + t * 0.20,
                gap=0.04 + t * 0.06,
                stride=70.0 - t * 20.0,
                height=70.0,
                lift=GO_LIFT_LOW + t * (GO_LIFT_HIGH - GO_LIFT_LOW))


def compute_offsets(duty, gap, direction):
    """motion_task.cpp 的 compute_offsets: 输出 [LF, LH, RF, RH] 的相位偏移"""
    if direction > 0:
        starts = [duty + gap, 0.0, 0.50 + duty + gap, 0.50]
    else:
        starts = [0.0, duty + gap, 0.50, 0.50 + duty + gap]
    offs = []
    for st in starts:
        while st >= 1.0:
            st -= 1.0
        offs.append((1.0 - st) if st > 0.001 else 0.0)
    return offs


def all_legs_in_stance(phase_rad, duty, gap, direction):
    """motion_task.cpp 的 all_legs_in_stance: 此刻四腿是否全在支撑相"""
    need_rev = (direction < 0) and (duty + gap < 0.50)
    for o in compute_offsets(duty, gap, -1 if need_rev else 1):
        pn = phase_rad / TWO_PI + o
        if pn >= 1.0:
            pn -= 1.0
        if pn < duty:
            return False
    return True


# ---------------------------------------------------------------- 状态与步进

class GaitState:
    """支撑相锚定的全部 per-leg 状态 (对应固件里的 g_gait_b / g_leg_* / g_leg_swing)"""

    def __init__(self):
        self.phase = 0.0
        self.b = [0.0] * 4
        self.b_land = [0.0] * 4
        self.x_land = [0.0] * 4
        self.x_lift = [0.0] * 4
        self.swing = [False] * 4
        self.prev_fx = [0.0] * 4


def advance_legs(st, eff_stride, duty, gap, height, lift, speed, direction=1):
    """一帧的腿循环。返回本帧四腿足端 x (机体系)。

    ★ 顺序必须是 锁存 -> 算 x -> 累加 b, 三处都不能换
      (见 foot_trajectory 顶部注释的 ①②)。
    """
    frame_dphi = (OMEGA_BASE * speed) * DT / TWO_PI
    offs = compute_offsets(duty, gap, direction)
    swing_ratio = duty
    inv_stance = 1.0 - duty
    landed = [False] * 4

    for i in range(4):
        leg_phase = st.phase + offs[i] * TWO_PI
        if leg_phase > TWO_PI:
            leg_phase -= TWO_PI
        pn = leg_phase / TWO_PI
        in_swing = pn < swing_ratio
        prev_swing_i = st.swing[i]

        if in_swing and not st.swing[i]:
            # 刚离地: 从支撑相最后一帧的位置起步 (b 要回退到理想跨越点)
            b_at_lift = st.b[i] - (eff_stride / inv_stance) * pn
            st.x_lift[i] = st.x_land[i] - (b_at_lift - st.b_land[i])
        elif (not in_swing) and st.swing[i]:
            # 刚落地: 定死锚点 (b_land 同样回退)
            over = (pn - duty) / inv_stance
            st.b_land[i] = st.b[i] - eff_stride * over
            st.x_land[i] = eff_stride * 0.5
        st.swing[i] = in_swing

        if in_swing:
            t = pn / swing_ratio
            ease = t * t * (3.0 - 2.0 * t)
            x = st.x_lift[i] + (eff_stride * 0.5 - st.x_lift[i]) * ease
        else:
            x = st.x_land[i] - (st.b[i] - st.b_land[i])
        if direction < 0:
            x = -x

        st.prev_fx[i] = x
        # ★ b 最后推进 (它必须对应**本帧**的 phase_norm)
        st.b[i] += (eff_stride / inv_stance) * frame_dphi
        landed[i] = (not in_swing) and prev_swing_i

    return list(st.prev_fx), landed


def settle_to_walk(st, speed, cycles=10):
    """跑到稳态行走 (系数=1), 顺带让锚定状态收敛"""
    p = go_params(speed)
    total = int(cycles * (TWO_PI / (OMEGA_BASE * speed)) / DT)
    for _ in range(total):
        st.phase += (OMEGA_BASE * speed) * DT
        if st.phase > TWO_PI:
            st.phase -= TWO_PI
        advance_legs(st, p['stride'], p['duty'], p['gap'],
                     p['height'], p['lift'], speed)


def run_stop(st0, speed, fade_out_turns, direction=1, criterion='window'):
    """从 st0 开始执行停步, 返回 (残留 mm, 淡出帧数, 等窗口帧数)。

    残留 = 切 GAIT_STOP 那一帧 max|足端 x|。
    """
    p = go_params(speed)
    duty, gap, stride = p['duty'], p['gap'], p['stride']

    st = GaitState()
    st.phase = st0.phase
    st.b = list(st0.b)
    st.b_land = list(st0.b_land)
    st.x_land = list(st0.x_land)
    st.x_lift = list(st0.x_lift)
    st.swing = list(st0.swing)
    st.prev_fx = list(st0.prev_fx)

    fade = 1.0
    active = False
    fade_frames = 0
    frames = 0
    prev_x = list(st.prev_fx)
    max_jump = 0.0
    max_jump_all = 0.0
    landed_since_zero = [False] * 4

    while True:
        frames += 1
        if frames > 5000:
            return None, fade_frames, frames - fade_frames, max_jump, max_jump_all

        st.phase += (OMEGA_BASE * speed) * DT
        if st.phase > TWO_PI:
            st.phase -= TWO_PI

        # ---- 闸门: arming + 平方淡出 (对应固件 478-552 行) ----
        if not active:
            if fade > 0.001:            # stride_stopping 恒真 (g_stop_decel = true)
                active = True
        if active:
            step = ((OMEGA_BASE * speed) * DT / TWO_PI) / fade_out_turns
            root = math.sqrt(fade) - step
            fade = root * root if root > 0.0 else 0.0
            if fade <= 0.0:
                fade = 0.0
                active = False
                fade_frames = frames

        fade_closed = (not active) and fade <= 0.001

        # ---- 腿循环 ----
        prev_swing = list(st.swing)
        _, landed = advance_legs(st, stride * fade, duty, gap,
                                 p['height'], p['lift'], speed, direction)

        # 逐腿判据: 记录"步长已归零之后重新落地过"的腿
        if fade_closed:
            for i in range(4):
                if landed[i]:
                    landed_since_zero[i] = True

        # 滑移只看**两帧都踩在地上**的腿: 摆动腿一帧走 5~8mm 是正常轨迹
        for i in range(4):
            d = abs(st.prev_fx[i] - prev_x[i])
            if d > max_jump_all:
                max_jump_all = d
            if (not prev_swing[i]) and (not st.swing[i]) and d > max_jump:
                max_jump = d
        prev_x = list(st.prev_fx)

        # ---- 淡出完成 -> 切换 (对应固件 604-622 行) ----
        if fade_closed:
            if criterion == 'window':
                ok = all_legs_in_stance(st.phase, duty, gap, direction)
            else:                       # 'perleg': 每条腿都重新落地过 (+ 仍在窗口内)
                ok = all(landed_since_zero) and \
                     all_legs_in_stance(st.phase, duty, gap, direction)
            if ok:
                return (max(abs(x) for x in st.prev_fx), fade_frames,
                        frames - fade_frames, max_jump, max_jump_all)


# ---------------------------------------------------------------- 旧版 (3407771^, 无锚定)

def traj_old_x(pn, S, duty, direction):
    """旧式足端轨迹 —— **位置系数**: x 只由 相位 + 当前步长 决定, 与历史无关。
    支撑相 x = S/2 - S·t  ⇒  S 一归零, 所有腿的 x 当帧变 0。
    """
    if pn < duty:
        t = pn / duty
        ease = t * t * (3.0 - 2.0 * t)
        x = -S * 0.5 + S * ease
    else:
        t = (pn - duty) / (1.0 - duty)
        x = S * 0.5 - S * t
    return -x if direction < 0 else x


def frame_old(phase, S, duty, gap, direction=1):
    offs = compute_offsets(duty, gap, direction)
    out = []
    for i in range(4):
        lp = phase + offs[i] * TWO_PI
        if lp > TWO_PI:
            lp -= TWO_PI
        out.append(traj_old_x(lp / TWO_PI, S, duty, direction))
    return out


def run_stop_old(phase0, speed, gait, direction=1):
    """旧版停步。返回 (残留, 总帧数, 单帧足端最大跳变 mm)。

    gait='go'       : 立刻切 1/3 → 下个半周期边界归零 → 切静态
    gait='walk'/'trot': stop_smooth ≡ 0 ⇒ 当帧就切 (0 周期, 0 步)
    """
    p = go_params(speed)
    duty, gap, S0 = p['duty'], p['gap'], p['stride']
    phase = phase0
    S = S0
    prev_x = frame_old(phase, S, duty, gap, direction)
    max_jump = 0.0
    frames = 0
    while True:
        frames += 1
        if frames > 5000:
            return None, frames, max_jump
        prev_phase = phase
        phase += (OMEGA_BASE * speed) * DT
        if phase > TWO_PI:
            phase -= TWO_PI
        cross_half = int(prev_phase / math.pi) != int(phase / math.pi)

        if gait == 'go':
            one_third = abs(S0) / 3.0
            if S > one_third + 0.1:
                S = one_third            # 立刻切 1/3 (不在半周期边界)
            if cross_half:
                S = 0.0                  # 下个半周期归零
        else:
            S = 0.0                      # TROT/WALK: 当帧归零

        x = frame_old(phase, S, duty, gap, direction)
        # 滑移只看两帧都在支撑相的腿
        offs = compute_offsets(duty, gap, direction)

        def _st(ph):
            f = []
            for i in range(4):
                lp = ph + offs[i] * TWO_PI
                if lp > TWO_PI:
                    lp -= TWO_PI
                f.append(lp / TWO_PI >= duty)
            return f

        ps, cs = _st(prev_phase), _st(phase)
        for i in range(4):
            if (not ps[i]) and (not cs[i]):
                d = abs(x[i] - prev_x[i])
                if d > max_jump:
                    max_jump = d
        prev_x = x

        if abs(S) <= 0.1:
            return max(abs(v) for v in x), frames, max_jump


# ---------------------------------------------------------------- 扫描

def audit(speed, fade_out_turns, samples=240, warm_cycles=10, criterion='window'):
    """扫 samples 个停止相位, 取最坏残留"""
    st = GaitState()
    settle_to_walk(st, speed, warm_cycles)

    worst = -1.0
    worst_at = 0
    worst_fade_frames = 0
    worst_wait_frames = 0
    max_wait = 0
    waited_cnt = 0
    max_jump = 0.0
    max_jump_all = 0.0

    snap = GaitState()
    for k in range(samples):
        # 记录当前状态快照
        snap.phase = st.phase
        snap.b = list(st.b)
        snap.b_land = list(st.b_land)
        snap.x_land = list(st.x_land)
        snap.x_lift = list(st.x_lift)
        snap.swing = list(st.swing)
        snap.prev_fx = list(st.prev_fx)

        res, ff, wf, jm, jma = run_stop(snap, speed, fade_out_turns, criterion=criterion)
        if res is not None:
            if wf > max_wait:
                max_wait = wf
            if wf > 0:
                waited_cnt += 1
            if jm > max_jump:
                max_jump = jm
            if jma > max_jump_all:
                max_jump_all = jma
            if res > worst:
                worst = res
                worst_at = k
                worst_fade_frames = ff
                worst_wait_frames = wf

        # 往前走一帧, 取样下一个停止相位
        p = go_params(speed)
        st.phase += (OMEGA_BASE * speed) * DT
        if st.phase > TWO_PI:
            st.phase -= TWO_PI
        advance_legs(st, p['stride'], p['duty'], p['gap'],
                     p['height'], p['lift'], speed)

    return dict(residual=worst, at=worst_at,
                fade_frames=worst_fade_frames, wait_frames=worst_wait_frames,
                max_wait=max_wait, waited=waited_cnt, samples=samples,
                max_jump=max_jump, max_jump_all=max_jump_all)


def audit_old(speed, gait, samples=240, warm_cycles=10):
    """旧版停步的 240 相位扫描 (无锚定, 位置系数)"""
    p = go_params(speed)
    phase = 0.0
    for _ in range(int(warm_cycles * (TWO_PI / (OMEGA_BASE * speed)) / DT)):
        phase += (OMEGA_BASE * speed) * DT
        if phase > TWO_PI:
            phase -= TWO_PI
    worst, mj, wf = -1.0, 0.0, 0
    for _ in range(samples):
        res, frames, jump = run_stop_old(phase, speed, gait)
        if res is not None:
            worst = max(worst, res)
            mj = max(mj, jump)
            wf = max(wf, frames)
        phase += (OMEGA_BASE * speed) * DT
        if phase > TWO_PI:
            phase -= TWO_PI
    return dict(residual=worst, max_jump=mj, frames=wf, duty=p['duty'], stride=p['stride'])


def main():
    speeds = [2.5, 8.5]
    windows = [0.5, 0.75, 0.8, 1.0, 1.15, 1.25, 1.5]

    print('=' * 96)
    print('停步审计 — 离线复刻固件逐帧计算')
    print('  残留 = 切 GAIT_STOP 那刻 max|足端 x| (mm) —— 随后 0.3s 缓动必须拖掉的距离')
    print('  滑移 = 收窄期间**支撑腿**单帧最大位移 (mm/帧) —— 正常走路本身约 1.4mm/帧')
    print('作者判据: 39~40mm/130mm-s = 搓地 (否掉)   2.2~4.0mm/13mm-s = 可接受')
    print('每格扫 240 个停止相位取最坏')
    print('=' * 96)

    # ---- 对照表: 旧版 vs 新版各窗口 ----
    for speed in speeds:
        p = go_params(speed)
        cyc = TWO_PI / (OMEGA_BASE * speed)
        print()
        print('【旧 vs 新】 speed %.1f   (GO: duty %.2f stride %.0f, 周期 %.3fs)'
              % (speed, p['duty'], p['stride'], cyc))
        print('%-28s %9s %7s %11s %12s %13s'
              % ('方案', '时长(s)', '步数', '残留(mm)', '拖地(mm/s)', '支撑腿滑移(mm/帧)'))
        print('-' * 96)
        fpd = (OMEGA_BASE * speed) * DT / TWO_PI
        for gait, label in (('walk', '旧版 WALK/TROT (当帧切)'),
                            ('go',   '旧版 GO (切1/3→半周期归零)')):
            r = audit_old(speed, gait)
            print('%-28s %9.2f %7d %11.2f %12.1f %13.2f'
                  % (label, r['frames'] * DT, round(4 * r['frames'] * fpd),
                     r['residual'], r['residual'] / STOP_TRANS_TIME, r['max_jump']))
        for w in windows:
            r = audit(speed, w)
            if r['residual'] < 0:
                continue
            total = (r['fade_frames'] + r['wait_frames']) * DT
            print('%-28s %9.2f %7d %11.2f %12.1f %13.2f'
                  % ('新版 淡出 %.2f 周期' % w, total,
                     round(4 * (r['fade_frames'] + r['wait_frames']) * fpd),
                     r['residual'], r['residual'] / STOP_TRANS_TIME, r['max_jump']))
        for w in (0.5, 0.75, 1.0, 1.25):
            r = audit(speed, w, criterion='perleg')
            if r['residual'] < 0:
                continue
            total = (r['fade_frames'] + r['wait_frames']) * DT
            print('%-28s %9.2f %7d %11.2f %12.1f %13.2f'
                  % ('★逐腿判据 淡出 %.2f' % w, total,
                     round(4 * (r['fade_frames'] + r['wait_frames']) * fpd),
                     r['residual'], r['residual'] / STOP_TRANS_TIME, r['max_jump']))
    print()


if __name__ == '__main__':
    main()
