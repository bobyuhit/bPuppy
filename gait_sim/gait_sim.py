#!/usr/bin/env python3
"""
bPuppy 四足机器人步态仿真 — 单周期足尖轨迹
==============================================
直接移植 drivers/ik.c 和 drivers/motion_task.cpp 的公式，
确保仿真结果与实机固件一致。

用法:
  python gait_sim.py                          # 默认 walk 步态
  python gait_sim.py --gait trot              # trot 步态
  python gait_sim.py --gait walk --stride 90 --height 110  # 自定义参数
  python gait_sim.py --csv my_gait.csv        # 指定 CSV 输出路径
"""

import argparse
import csv
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

# ==============================================================================
# 常量 — 对应 ik.h / motion_task.h
# ==============================================================================

L1 = 40.0         # 大腿长度 mm (ik.h: IK_L1_DEFAULT)
L2 = 45.0         # 小腿长度 mm (ik.h: IK_L2_DEFAULT)
BODY_HALF_L = 60.0  # 前后髋半距 mm (motion_task.cpp: BODY_HALF_L=60, 全距 120mm)
BODY_HALF_W = 59.0  # 左右髋半宽 mm (motion_task.cpp: BODY_HALF_W=59.0, 全距 118mm)

HIP_MIN, HIP_MAX = 0.0, 180.0
KNEE_MIN, KNEE_MAX = 10.0, 170.0  # 对应 ik.h ik_knee_min / ik_knee_max

IK_KNEE_REAR_FORWARD = 1   # 与 ik.h 保持一致: 1=后腿前弯, 0=四腿后弯

LEG_NAMES = ["LF", "LH", "RF", "RH"]
LEG_SIDES = ["left", "left", "right", "right"]
LEG_PAIRS = ["front", "rear", "front", "rear"]

# 默认步态参数 — 对应 motion_task.cpp g_motion 初始值 + motion_apply_gait_params()
# C 代码中 stride/height/lift 对所有步态使用同一默认值，只有 duty/gap 随步态变化
GAIT_PRESETS = {
    "walk": {"duty": 0.20, "gap": 0.02, "stride": 70.0, "height": 60.0, "lift": 30.0},
}


# ==============================================================================
# 数据结构
# ==============================================================================

@dataclass
class GaitParams:
    """步态参数 — 对应 motion_state_t 中的核心字段"""
    duty: float       # 摆动相占比 [0~1]
    gap: float        # 同侧相位间隙
    stride: float     # 步长 mm
    height: float     # 站立高度 mm
    lift: float       # 抬腿高度 mm
    direction: int = 1  # 1=前进, -1=后退

    @classmethod
    def from_preset(cls, name: str, **overrides):
        """从预设加载，允许覆盖"""
        preset = GAIT_PRESETS.get(name, GAIT_PRESETS["walk"]).copy()
        preset.update(overrides)
        return cls(**preset)


# ==============================================================================
# 相位偏移计算 — 移植自 motion_task.cpp compute_offsets()
# ==============================================================================

def compute_offsets(duty: float, gap: float, direction: int = 1) -> list[float]:
    """
    计算四条腿的相位偏移。

    前进时 LH 领头:
      start_LH = 0,  start_LF = duty + gap
      start_RH = 0.50, start_RF = 0.50 + duty + gap
    后退时 LF 和 LH 角色互换。

    返回 [LF_offset, LH_offset, RF_offset, RH_offset]，归一化到 [0, 1)。
    """
    if direction > 0:
        starts = [
            duty + gap,          # LF
            0.0,                 # LH
            0.50 + duty + gap,   # RF
            0.50,                # RH
        ]
    else:
        starts = [
            0.0,                 # LF
            duty + gap,          # LH
            0.50,                # RF
            0.50 + duty + gap,   # RH
        ]

    offsets = []
    for s in starts:
        s = s - 1.0 if s >= 1.0 else s
        off = 0.0 if s < 0.001 else (1.0 - s)
        offsets.append(off)
    return offsets


# ==============================================================================
# 足端轨迹 — 移植自 motion_task.cpp foot_trajectory()
# ==============================================================================

def foot_trajectory(phase_norm: float, stride: float, height: float,
                    lift: float, duty: float, direction: int = 1) -> tuple[float, float]:
    """
    给定归一化相位 [0, 1)，返回足端坐标 (foot_x, foot_z)。

    坐标系: x 正=前, z 以地面为 0 (正=上)。
    支撑相 z=0 (着地), 摆动相 z>0 (抬离地面)。
    """
    if phase_norm < duty:
        # 摆动相 — cycloidal arc with smoothstep easing
        t = phase_norm / duty
        ease = t * t * (3.0 - 2.0 * t)
        fz = lift * math.sin(ease * math.pi)   # 0 → lift → 0, 正=上
        fx = -stride * 0.5 + stride * ease
    else:
        # 支撑相 — linear push-back
        t = (phase_norm - duty) / (1.0 - duty)
        fz = 0.0                                 # 着地
        fx = stride * 0.5 - stride * t

    if direction < 0:
        fx = -fx
    return fx, fz


# ==============================================================================
# IK 解算 — 移植自 ik.c ik_solve_2dof()
# ==============================================================================

def ik_solve(foot_x: float, foot_z: float, side: str,
             leg_pair: str = "front") -> tuple[float, float]:
    """
    2-DOF 平面逆运动学。

    foot_x: 前后 (正=前), foot_z: 上下 (正=下)
    side: "left" | "right", leg_pair: "front" | "rear"
    返回 (hip_deg, knee_deg) 舵机角度。
    """
    d_sq = foot_x * foot_x + foot_z * foot_z
    d = math.sqrt(d_sq)

    # 钳位到机械可达范围
    l_sum = L1 + L2
    l_diff = abs(L2 - L1)
    if d > l_sum - 1.0:
        d = l_sum - 1.0
    if d < l_diff + 1.0:
        d = l_diff + 1.0

    # 膝角 (大腿-小腿夹角, 0°=折叠, 180°=伸直)
    cos_knee = (L1 * L1 + L2 * L2 - d * d) / (2.0 * L1 * L2)
    cos_knee = max(-1.0, min(1.0, cos_knee))
    knee_angle = math.acos(cos_knee)

    # 髋角: 膝后弯 hip=alpha+beta, 膝前弯 hip=alpha-beta
    alpha = math.atan2(foot_z, foot_x)
    cos_beta = (L1 * L1 + d * d - L2 * L2) / (2.0 * L1 * d)
    cos_beta = max(-1.0, min(1.0, cos_beta))
    beta = math.acos(cos_beta)
    if IK_KNEE_REAR_FORWARD and leg_pair == "rear":
        hip_angle = alpha - beta
    else:
        hip_angle = alpha + beta

    hip_deg = math.degrees(hip_angle)
    knee_deg = math.degrees(knee_angle)

    # 膝角映射: 后腿前弯时左右互换
    knee_mirror = (side == "right")
    if IK_KNEE_REAR_FORWARD and leg_pair == "rear":
        knee_mirror = not knee_mirror

    if side == "right":
        hip_deg = 180.0 - hip_deg
    knee_deg = 180.0 - knee_deg if knee_mirror else knee_deg

    # 钳位
    hip_deg = max(HIP_MIN, min(HIP_MAX, hip_deg))
    knee_deg = max(KNEE_MIN, min(KNEE_MAX, knee_deg))

    return hip_deg, knee_deg


# ==============================================================================
# 单周期仿真
# ==============================================================================

def simulate_cycle(params: GaitParams, num_steps: int = 200) -> list[dict]:
    """
    仿真一个完整步态周期。

    全局相位 g_phase 从 0 → 2π，均匀采样 num_steps 帧。
    返回 list[dict]，每帧包含各腿的足端坐标和关节角度。
    """
    offsets = compute_offsets(params.duty, params.gap, params.direction)

    rows = []
    for i in range(num_steps):
        g_phase = 2.0 * math.pi * i / num_steps  # [0, 2π)
        row = {"step": i, "g_phase_deg": round(math.degrees(g_phase), 1)}

        for leg_idx, name in enumerate(LEG_NAMES):
            side = LEG_SIDES[leg_idx]

            # 该腿在当前时刻的相位
            leg_phase = g_phase + offsets[leg_idx] * 2.0 * math.pi
            if leg_phase > 2.0 * math.pi:
                leg_phase -= 2.0 * math.pi
            phase_norm = leg_phase / (2.0 * math.pi)

            # 足端轨迹 (z: 地面=0, 正=上)
            fx, fz = foot_trajectory(
                phase_norm, params.stride, params.height,
                params.lift, params.duty, params.direction,
            )

            # IK 需要髋关节坐标系: z 正=下, hip→ground = height
            foot_z_ik = params.height - fz
            hip_deg, knee_deg = ik_solve(fx, foot_z_ik, side,
                                          LEG_PAIRS[leg_idx])

            # 摆动/支撑标识
            in_swing = 1 if phase_norm < params.duty else 0

            row[f"{name}_fx"] = round(fx, 3)
            row[f"{name}_fz"] = round(fz, 3)
            row[f"{name}_hip"] = round(hip_deg, 3)
            row[f"{name}_knee"] = round(knee_deg, 3)
            row[f"{name}_swing"] = in_swing

        rows.append(row)

    return rows


# ==============================================================================
# 可视化
# ==============================================================================

def plot_trajectories(rows: list[dict], params: GaitParams, gait_name: str):
    """绘制 t-z 曲线 + t-x 曲线 + 顶视图快照 + 相位甘特图"""
    num_steps = len(rows)

    def col(name):
        return np.array([r[name] for r in rows])

    colors = ["#E74C3C", "#F39C12", "#2980B9", "#27AE60"]  # LF LH RF RH
    x_axis = np.linspace(0, 100, num_steps)  # 相位百分比

    from matplotlib.gridspec import GridSpec

    fig = plt.figure(figsize=(18, 18))
    fig.suptitle(
        f"{gait_name.upper()} gait simulation\n"
        f"L1={L1:.0f}mm  L2={L2:.0f}mm  body={2*BODY_HALF_L:.0f}×{2*BODY_HALF_W:.0f}mm\n"
        f"duty={params.duty:.2f}  gap={params.gap:.2f}\n"
        f"stride={params.stride:.0f}mm  height={params.height:.0f}mm  lift={params.lift:.0f}mm",
        fontsize=13, fontweight="bold",
    )

    gs = GridSpec(4, 1, figure=fig, height_ratios=[1.0, 1.0, 2.2, 0.8],
                  hspace=0.35)

    # ---- Row 1: t–Z 曲线 — 四足对齐 ----
    ax_tz = fig.add_subplot(gs[0])
    for idx, name in enumerate(LEG_NAMES):
        ax_tz.plot(x_axis, col(f"{name}_fz"), color=colors[idx],
                   linewidth=1.8, alpha=0.85, label=name)
    ax_tz.axhline(y=0, color="gray", linewidth=0.8, linestyle=":", alpha=0.5)
    ax_tz.set_title("t–Z: Foot Z Position vs Phase %  (z=0 = ground)", fontsize=13, fontweight="bold")
    ax_tz.set_ylabel("Foot Z (mm)")
    ax_tz.grid(True, alpha=0.3)
    ax_tz.legend(fontsize=10, loc="upper right")

    # ---- Row 2: t–X 曲线 — 四足对齐 ----
    ax_tx = fig.add_subplot(gs[1])
    for idx, name in enumerate(LEG_NAMES):
        ax_tx.plot(x_axis, col(f"{name}_fx"), color=colors[idx],
                   linewidth=1.8, alpha=0.85, label=name)
    ax_tx.axhline(y=0, color="gray", linewidth=0.8, linestyle=":", alpha=0.5)
    ax_tx.set_title("t–X: Foot X Position vs Phase %  (x=0 = hip)", fontsize=13, fontweight="bold")
    ax_tx.set_ylabel("Foot X (mm)")
    ax_tx.grid(True, alpha=0.3)
    ax_tx.legend(fontsize=10, loc="lower left")

    # ---- Row 3: 顶视图快照 — RH + RF 抬脚瞬间 (左右各一) ----
    gs_top = gs[2].subgridspec(1, 2, wspace=0.25)
    ax_rh = fig.add_subplot(gs_top[0])
    _draw_topdown_snapshot(ax_rh, rows, params, colors, leg="RH")
    ax_rh.set_title("Top-Down — RH Lift-off  (●=stance, ▷=swing, ■=hip)",
                     fontsize=9, fontweight="bold")
    ax_rf = fig.add_subplot(gs_top[1])
    _draw_topdown_snapshot(ax_rf, rows, params, colors, leg="RF")
    ax_rf.set_title("Top-Down — RF Lift-off",
                     fontsize=9, fontweight="bold")

    # ---- Row 4: 相位甘特图 ----
    ax_gantt = fig.add_subplot(gs[3])
    gantt_order = ["LH", "LF", "RH", "RF"]  # 从上到下
    gantt = np.zeros((4, num_steps))
    for idx, name in enumerate(gantt_order):
        gantt[idx, :] = col(f"{name}_swing")

    ax_gantt.imshow(gantt, aspect="auto", cmap="RdYlGn_r",
                    extent=[0, 100, -0.5, 3.5], interpolation="nearest")
    ax_gantt.set_yticks(range(4))
    ax_gantt.set_yticklabels(gantt_order)
    ax_gantt.set_xlabel("Phase %")
    ax_gantt.set_title("Phase Gantt Chart (green=stance, red=swing)",
                       fontsize=13, fontweight="bold")

    return fig


def _point_in_polygon(px: float, py: float, polygon: list) -> bool:
    """判断点是否在多边形内 (matplotlib Path.contains_point)"""
    if len(polygon) < 3:
        return False
    from matplotlib.path import Path
    # 确保多边形闭合
    verts = [(x, y) for x, y in polygon]
    verts.append(verts[0])
    path = Path(verts)
    return path.contains_point((px, py))


def _draw_topdown_snapshot(ax, rows: list[dict], params: GaitParams, colors: list, leg: str = "RH"):
    """在顶视图中绘制指定腿抬脚瞬间的身体矩形 + 四足位置"""
    # 找指定腿进入摆动相的第一帧
    swing_col = np.array([r[f"{leg}_swing"] for r in rows])
    swing_start_idx = np.where(swing_col == 1)[0]
    if len(swing_start_idx) == 0:
        frame_idx = 0
    else:
        frame_idx = swing_start_idx[0]
    row = rows[frame_idx]

    # 身体矩形: 长=前后髋距, 宽=左右髋距
    body_l = 2 * BODY_HALF_L  # 128mm 前后
    body_w = 2 * BODY_HALF_W  # 118mm 左右

    rect = plt.Rectangle((-body_l / 2, -body_w / 2), body_l, body_w,
                          fill=False, edgecolor="black", linewidth=2, linestyle="-")
    ax.add_patch(rect)

    # 四髋关节位置 (顶点)
    hip_positions = {
        "LF": (+BODY_HALF_L, +BODY_HALF_W),
        "LH": (-BODY_HALF_L, +BODY_HALF_W),
        "RF": (+BODY_HALF_L, -BODY_HALF_W),
        "RH": (-BODY_HALF_L, -BODY_HALF_W),
    }

    for idx, name in enumerate(LEG_NAMES):
        hx, hy = hip_positions[name]
        fx = hx + row[f"{name}_fx"]   # foot_x 是相对髋的前后偏移
        fy = hy                        # 无侧向运动 (2-DOF 平面腿)
        is_swing = row[f"{name}_swing"] == 1

        # 髋关节 — 小圆点 (轻量化)
        ax.plot(hx, hy, marker="o", color="gray", markersize=6, zorder=4)

        # 足端 — 摆动相用三角 (顶角指向前进方向), 支撑相用圆
        if is_swing:
            # 等腰三角, 顶角指右 (前进方向 x+)
            tri = np.array([[8, 0], [-4, 5], [-4, -5]])  # 顶点在右
            ax.fill(fx + tri[:, 0], fy + tri[:, 1],
                    color=colors[idx], edgecolor="black", linewidth=1.2, zorder=6)
        else:
            ax.plot(fx, fy, marker="o", color=colors[idx], markersize=14,
                    markeredgecolor="black", markeredgewidth=1, zorder=6)

        # 连线: 髋 → 足
        ax.plot([hx, fx], [hy, fy], color=colors[idx], linewidth=1.5, alpha=0.5)

    # ---- 支撑多边形: 连接所有着地足端 ----
    stance_feet = []
    for name in LEG_NAMES:
        if row[f"{name}_swing"] == 0:
            hx, hy = hip_positions[name]
            fx = hx + row[f"{name}_fx"]
            fy = hy
            stance_feet.append((fx, fy))

    if len(stance_feet) >= 2:
        # 按绕重心角度排序，确保凸多边形
        cx = sum(p[0] for p in stance_feet) / len(stance_feet)
        cy = sum(p[1] for p in stance_feet) / len(stance_feet)
        stance_feet.sort(key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
        poly_x = [p[0] for p in stance_feet] + [stance_feet[0][0]]
        poly_y = [p[1] for p in stance_feet] + [stance_feet[0][1]]

        ax.fill(poly_x, poly_y, facecolor="none", edgecolor="gray",
                linewidth=1.5, linestyle="--", hatch="///", alpha=0.4, zorder=3)
        # 顶点小点
        ax.scatter(poly_x[:-1], poly_y[:-1], color="black", s=10, zorder=7)

    # ---- 重心标记 (身体中心 = CoM) ----
    com_x, com_y = 0.0, 0.0
    in_polygon = _point_in_polygon(com_x, com_y, stance_feet)
    com_color = "green" if in_polygon else "red"
    ax.plot(com_x, com_y, marker="+", color=com_color, markersize=20,
            markeredgewidth=3, zorder=8)

    # 图例
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker="o", color="w", markerfacecolor="gray",
               markersize=10, label="stance foot"),
        Line2D([0], [0], marker=">", color="w", markerfacecolor="gray",
               markersize=10, label="swing foot (apex = forward)"),
    ]
    ax.legend(handles=legend_elements, loc="lower right", fontsize=9)

    ax.set_xlim(-body_l, body_l)
    ax.set_ylim(-body_w * 1.5, body_w * 1.5)
    ax.set_xlabel("X (mm)  [forward →]")
    ax.set_ylabel("Y (mm)  [left ←]")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)

    phase_deg = row["g_phase_deg"]
    ax.text(0.02, 0.98, f"{leg} lift-off  g_phase = {phase_deg:.0f}°", transform=ax.transAxes,
            ha="left", va="top", fontsize=10,
            bbox=dict(boxstyle="round", facecolor="wheat"))


# ==============================================================================
# 顶视图动图
# ==============================================================================

def make_animation(rows: list[dict], params: GaitParams, gait_name: str,
                   output_path: str = None, fps: int = 20):
    """生成单周期顶视图 GIF 动图"""
    from matplotlib.animation import FuncAnimation, PillowWriter

    num_steps = len(rows)
    body_l = 2 * BODY_HALF_L
    body_w = 2 * BODY_HALF_W
    colors = ["#E74C3C", "#F39C12", "#2980B9", "#27AE60"]
    hip_positions = {
        "LF": (+BODY_HALF_L, +BODY_HALF_W),
        "LH": (-BODY_HALF_L, +BODY_HALF_W),
        "RF": (+BODY_HALF_L, -BODY_HALF_W),
        "RH": (-BODY_HALF_L, -BODY_HALF_W),
    }

    fig, ax = plt.subplots(figsize=(8, 8))
    ax.set_xlim(-body_l * 1.1, body_l * 1.1)
    ax.set_ylim(-body_w * 1.6, body_w * 1.6)
    ax.set_aspect("equal")
    ax.set_xlabel("X (mm)  [forward →]")
    ax.set_ylabel("Y (mm)  [left ←]")
    ax.grid(True, alpha=0.3)

    title = ax.set_title("", fontsize=13, fontweight="bold")

    # 静态元素：身体矩形
    rect = plt.Rectangle((-body_l / 2, -body_w / 2), body_l, body_w,
                          fill=False, edgecolor="black", linewidth=2)
    ax.add_patch(rect)

    # 动态元素占位
    hip_dots = [ax.plot([], [], "o", color="gray", markersize=6)[0] for _ in range(4)]
    foot_artists = []  # (polygon/triangle fill or circle, line_to_hip)
    for idx in range(4):
        line, = ax.plot([], [], color=colors[idx], linewidth=1.5, alpha=0.5)
        marker, = ax.plot([], [], "o", color=colors[idx], markersize=14,
                          markeredgecolor="black", markeredgewidth=1)
        tri_fill = None
        foot_artists.append({"line": line, "marker": marker, "tri_fill": tri_fill})

    poly_fill = None
    poly_line = None
    poly_dots = None
    com_dot, = ax.plot([], [], "+", color="green", markersize=20, markeredgewidth=3)
    phase_text = ax.text(0.02, 0.98, "", transform=ax.transAxes,
                         ha="left", va="top", fontsize=10,
                         bbox=dict(boxstyle="round", facecolor="wheat"))

    # 图例
    from matplotlib.lines import Line2D
    legend_elements = [
        Line2D([0], [0], marker="o", color="gray", markersize=10,
               label="stance foot"),
        Line2D([0], [0], marker=">", color="gray", markersize=10,
               label="swing foot"),
        Line2D([0], [0], marker="+", color="green", markersize=10,
               markeredgewidth=3, label="CoM (stable)", linestyle="none"),
        Line2D([0], [0], marker="+", color="red", markersize=10,
               markeredgewidth=3, label="CoM (unstable)", linestyle="none"),
    ]
    ax.legend(handles=legend_elements, loc="lower right", fontsize=8)

    def update(frame_idx):
        row = rows[frame_idx]
        phase_deg = row["g_phase_deg"]

        title.set_text(
            f"{gait_name.upper()} gait simulation\n"
            f"L1={L1:.0f}mm  L2={L2:.0f}mm  body={2*BODY_HALF_L:.0f}×{2*BODY_HALF_W:.0f}mm\n"
            f"duty={params.duty:.2f}  gap={params.gap:.2f}\n"
            f"stride={params.stride:.0f}mm  height={params.height:.0f}mm  lift={params.lift:.0f}mm"
        )
        phase_text.set_text(f"phase = {phase_deg:.0f}°")

        # 收集着地足
        stance_feet = []

        for idx, name in enumerate(LEG_NAMES):
            hx, hy = hip_positions[name]
            fx = hx + row[f"{name}_fx"]
            fy = hy
            is_swing = row[f"{name}_swing"] == 1

            # 髋关节
            hip_dots[idx].set_data([hx], [hy])

            # 足端
            fa = foot_artists[idx]
            fa["line"].set_data([hx, fx], [hy, fy])

            if is_swing:
                # 三角
                tri = np.array([[10, 0], [-5, 6], [-5, -6]])
                if fa["tri_fill"] is None:
                    fa["tri_fill"] = ax.fill(
                        fx + tri[:, 0], fy + tri[:, 1],
                        color=colors[idx], edgecolor="black", linewidth=1.2, zorder=6)[0]
                else:
                    fa["tri_fill"].set_xy(np.column_stack([fx + tri[:, 0], fy + tri[:, 1]]))
                fa["tri_fill"].set_visible(True)
                fa["marker"].set_visible(False)
            else:
                if fa["tri_fill"] is not None:
                    fa["tri_fill"].set_visible(False)
                fa["marker"].set_data([fx], [fy])
                fa["marker"].set_visible(True)
                stance_feet.append((fx, fy))

        # 支撑多边形
        nonlocal poly_fill, poly_line, poly_dots
        if poly_fill is not None:
            poly_fill.remove()
            poly_fill = None
        if poly_line is not None:
            poly_line.remove()
            poly_line = None
        if poly_dots is not None:
            poly_dots.remove()
            poly_dots = None

        if len(stance_feet) >= 2:
            cx = sum(p[0] for p in stance_feet) / len(stance_feet)
            cy = sum(p[1] for p in stance_feet) / len(stance_feet)
            sorted_feet = sorted(stance_feet,
                                 key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
            poly_x = [p[0] for p in sorted_feet] + [sorted_feet[0][0]]
            poly_y = [p[1] for p in sorted_feet] + [sorted_feet[0][1]]

            poly_fill = ax.fill(poly_x, poly_y, facecolor="none", edgecolor="gray",
                                linewidth=1.5, linestyle="--", hatch="///",
                                alpha=0.4, zorder=3)[0]
            poly_dots = ax.scatter(poly_x[:-1], poly_y[:-1], color="black",
                                   s=10, zorder=7)

        # CoM
        in_polygon = _point_in_polygon(0.0, 0.0, stance_feet)
        com_color = "green" if in_polygon else "red"
        com_dot.set_data([0.0], [0.0])
        com_dot.set_color(com_color)

        artists = [title, phase_text, com_dot]
        artists.extend(hip_dots)
        artists.append(poly_fill) if poly_fill else None
        artists.append(poly_dots) if poly_dots else None
        for fa in foot_artists:
            artists.append(fa["line"])
            if fa["tri_fill"] and fa["tri_fill"].get_visible():
                artists.append(fa["tri_fill"])
            if fa["marker"].get_visible():
                artists.append(fa["marker"])
        return artists

    skip = max(1, num_steps // 80)  # 控制帧数，约 80 帧
    ani = FuncAnimation(fig, update, frames=range(0, num_steps, skip),
                        interval=1000 / fps, blit=False)

    if output_path is None:
        output_path = str(Path(__file__).parent / f"gait_{gait_name}.gif")

    writer = PillowWriter(fps=fps)
    ani.save(output_path, writer=writer)
    plt.close(fig)
    print(f"[OK] Animation saved: {output_path}")
    return output_path

def export_csv(rows: list[dict], filepath: str):
    """导出仿真结果到 CSV"""
    if not rows:
        return
    fieldnames = list(rows[0].keys())
    with open(filepath, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n[OK] CSV exported: {filepath}")
    print(f"     {len(rows)} frames, {len(fieldnames)} columns")


# ==============================================================================
# 主入口
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="bPuppy 四足机器人步态仿真 — 单周期足尖轨迹",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  python gait_sim.py                              # 输出 CSV + PNG + GIF
  python gait_sim.py --show                       # 同上，并弹出图表窗口
  python gait_sim.py --stride 80 --height 70      # 自定义参数
  python gait_sim.py --no-gif --no-png            # 只输出 CSV
        """,
    )
    parser.add_argument("--gait", type=str, default="walk",
                        choices=["walk"],
                        help="步态类型 (default: walk)")
    parser.add_argument("--duty", type=float, default=None,
                        help="摆动相占比 (walk=0.20, trot=0.40)")
    parser.add_argument("--gap", type=float, default=None,
                        help="同侧相位间隙 (walk=0.04, trot=0.10)")
    parser.add_argument("--stride", type=float, default=None,
                        help="步长 mm (walk=80, trot=60)")
    parser.add_argument("--height", type=float, default=None,
                        help="站立高度 mm (walk=100, trot=80)")
    parser.add_argument("--lift", type=float, default=None,
                        help="抬腿高度 mm (default=25)")
    parser.add_argument("--steps", type=int, default=200,
                        help="单周期采样帧数 (default=200)")
    parser.add_argument("--csv", type=str, default=None,
                        help="CSV 输出路径 (default: gait_walk.csv)")
    parser.add_argument("--no-csv", action="store_true",
                        help="不输出 CSV")
    parser.add_argument("--no-png", action="store_true",
                        help="不输出静态图表 PNG")
    parser.add_argument("--no-gif", action="store_true",
                        help="不输出顶视图 GIF 动图")
    parser.add_argument("--show", action="store_true",
                        help="显示静态图表窗口（默认只保存文件）")
    parser.add_argument("--fps", type=int, default=8,
                        help="动图帧率 (default: 8, 越小越慢)")

    args = parser.parse_args()

    # 构建参数：预设 + 命令行覆盖
    overrides = {}
    if args.duty is not None:
        overrides["duty"] = args.duty
    if args.gap is not None:
        overrides["gap"] = args.gap
    if args.stride is not None:
        overrides["stride"] = args.stride
    if args.height is not None:
        overrides["height"] = args.height
    if args.lift is not None:
        overrides["lift"] = args.lift

    params = GaitParams.from_preset(args.gait, **overrides)

    # 打印参数
    print("=" * 60)
    print(f"  bPuppy Gait Simulation — {args.gait.upper()}")
    print("=" * 60)
    print(f"  duty={params.duty:.2f}  gap={params.gap:.2f}  "
          f"stride={params.stride:.0f}mm  height={params.height:.0f}mm  "
          f"lift={params.lift:.0f}mm")
    print(f"  L1={L1:.0f}mm  L2={L2:.0f}mm  "
          f"body_half_L={BODY_HALF_L:.0f}mm  body_half_W={BODY_HALF_W:.0f}mm")
    offsets = compute_offsets(params.duty, params.gap, params.direction)
    print(f"  offsets: LF={offsets[0]:.3f}  LH={offsets[1]:.3f}  "
          f"RF={offsets[2]:.3f}  RH={offsets[3]:.3f}")
    print(f"  frames: {args.steps}")
    print("=" * 60)

    # 仿真
    rows = simulate_cycle(params, num_steps=args.steps)

    # 摘要
    for name in LEG_NAMES:
        swing_vals = [r[f"{name}_swing"] for r in rows]
        fx_vals = [r[f"{name}_fx"] for r in rows]
        hip_vals = [r[f"{name}_hip"] for r in rows]
        knee_vals = [r[f"{name}_knee"] for r in rows]
        swing_pct = sum(swing_vals) / len(swing_vals) * 100
        print(f"  {name}: swing={swing_pct:.0f}%  "
              f"fx=[{min(fx_vals):.1f}, {max(fx_vals):.1f}]  "
              f"hip=[{min(hip_vals):.1f}, {max(hip_vals):.1f}]  "
              f"knee=[{min(knee_vals):.1f}, {max(knee_vals):.1f}]")

    base_dir = Path(__file__).parent

    # CSV
    if not args.no_csv:
        csv_path = args.csv or str(base_dir / f"gait_{args.gait}.csv")
        export_csv(rows, csv_path)

    # PNG 静态图表
    if not args.no_png:
        png_path = str(base_dir / f"gait_{args.gait}.png")
        print("\n[INFO] Generating static plots...")
        fig = plot_trajectories(rows, params, args.gait)
        fig.savefig(png_path, dpi=150, bbox_inches="tight")
        print(f"[OK] PNG saved: {png_path}")
        if args.show:
            plt.show()
        else:
            plt.close(fig)

    # GIF 动图
    if not args.no_gif:
        gif_path = str(base_dir / f"gait_{args.gait}.gif")
        print("\n[INFO] Generating animation...")
        make_animation(rows, params, args.gait, fps=args.fps, output_path=gif_path)

    print("\n[DONE]")


if __name__ == "__main__":
    main()
