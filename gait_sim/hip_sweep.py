"""髋距扫描：80~150mm，腿长 L1=40 L2=45，找 CoM 最稳的参数"""
import csv, math, sys
from pathlib import Path

L1 = 40.0
L2 = 45.0
BODY_HALF_W = 59.0  # 固定
IK_KNEE_REAR_FORWARD = 1  # 与 ik.h 保持一致
DUTY, GAP, STRIDE, HEIGHT, LIFT = 0.20, 0.02, 70.0, 60.0, 30.0

def compute_offsets(duty, gap):
    starts = [duty+gap, 0.0, 0.50+duty+gap, 0.50]
    return [(0.0 if s < 0.001 else (1.0 - s)) for s in [  # LF LH RF RH
        starts[0]-1.0 if starts[0]>=1.0 else starts[0],
        starts[1], starts[2]-1.0 if starts[2]>=1.0 else starts[2], starts[3]]]

def ik_solve(fx, fz, side, leg_pair='front'):
    d_sq = fx*fx + fz*fz
    d = math.sqrt(d_sq)
    l_sum, l_diff = L1+L2, abs(L2-L1)
    d = max(l_diff+1, min(l_sum-1, d))
    ck = max(-1, min(1, (L1*L1+L2*L2-d*d)/(2*L1*L2)))
    knee = math.degrees(math.acos(ck))
    alpha = math.atan2(fz, fx)
    cb = max(-1, min(1, (L1*L1+d*d-L2*L2)/(2*L1*d)))
    beta = math.acos(cb)
    if IK_KNEE_REAR_FORWARD and leg_pair == 'rear':
        hip = math.degrees(alpha - beta)
    else:
        hip = math.degrees(alpha + beta)
    knee_mirror = (side == 'right')
    if IK_KNEE_REAR_FORWARD and leg_pair == 'rear':
        knee_mirror = not knee_mirror
    if side == 'right':
        hip = 180 - hip
    knee = 180 - knee if knee_mirror else knee
    return max(0, min(180, hip)), max(10, min(170, knee))

from matplotlib.path import Path

def simulate_one(bl):
    offsets = compute_offsets(DUTY, GAP)
    hip_pos = {'LF':(bl, BODY_HALF_W), 'LH':(-bl, BODY_HALF_W),
               'RF':(bl, -BODY_HALF_W), 'RH':(-bl, -BODY_HALF_W)}
    unstable, total = 0, 0
    for i in range(200):
        g_phase = 2*math.pi*i/200
        pts = []; swing_legs = []
        leg_pairs = ['front','rear','front','rear']
        for idx, name in enumerate(['LF','LH','RF','RH']):
            side = 'left' if idx < 2 else 'right'
            leg_phase = g_phase + offsets[idx]*2*math.pi
            if leg_phase > 2*math.pi: leg_phase -= 2*math.pi
            pn = leg_phase/(2*math.pi)
            if pn < DUTY:
                t = pn / DUTY; e = t*t*(3-2*t)
                fz = LIFT*math.sin(e*math.pi)
                fx = -STRIDE*0.5 + STRIDE*e
                swing_legs.append(name)
            else:
                t = (pn-DUTY)/(1-DUTY)
                fz = 0.0
                fx = STRIDE*0.5 - STRIDE*t
            foot_z_ik = HEIGHT - fz
            hp, kp = ik_solve(fx, foot_z_ik, side, leg_pairs[idx])
            hx, hy = hip_pos[name]
            if name not in swing_legs:
                pts.append((hx+fx, hy))
        if len(pts) >= 3:
            cx = sum(p[0] for p in pts)/len(pts)
            cy = sum(p[1] for p in pts)/len(pts)
            pts.sort(key=lambda p: math.atan2(p[1]-cy, p[0]-cx))
            if not Path(pts).contains_point((0,0)):
                unstable += 1
        total += 1
    return unstable, total

print(f"{'髋距mm':>8} {'不稳帧':>6} {'占比%':>7}")
print("-"*24)
best_bl, best_rate = None, 100.0
for bl in range(40, 76):  # 半距, 全距80~150
    body_l = bl * 2
    unstable, total = simulate_one(bl)
    rate = unstable/total*100
    tag = ""
    if unstable == 0:
        tag = " ★"
        if best_rate > rate:
            best_rate = rate; best_bl = body_l
    print(f"{body_l:>8} {unstable:>6} {rate:>6.1f}%{tag}")

print(f"\n最优髋距: {best_bl}mm  (不稳率 {best_rate:.1f}%)")
