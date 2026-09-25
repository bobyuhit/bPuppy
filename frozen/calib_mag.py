"""
calib_mag.py — 磁力计 3D 椭球拟合引导式校准

拿起机器狗, 按提示分三轴旋转, 实时显示覆盖进度。
全部达标后自动拟合, 持久化到 NVS。

用法:
    import calib_mag
    calib_mag.start()
"""

import bpuppy_imu
import time

_TARGETS = (60.0, 60.0, 60.0)  # X/Y/Z 最小覆盖范围 (μT)
MIN_SAMPLES = 300              # 最小样本数 (椭球拟合质量, 太少拟合不准)
_AXIS_NAMES = ('X (Roll)', 'Y (Pitch)', 'Z (Yaw)')
_HINTS = (
    '请左右倾斜机器狗，覆盖 Roll 范围',
    '请前后倾斜机器狗，覆盖 Pitch 范围',
    '请水平旋转机器狗，覆盖 Yaw 360°',
)


_STALL_MS = 5000   # 连续多久读不到有效样本算卡死 (正常 ~50Hz, 5s ≈ 250 次)


def _stall_abort(count):
    """采集卡死的兜底出路。

    两个 while 都靠 mag_cal_collect() 返回 ok=True 推进。一旦磁力计读不到
    (掉线 / start_mag_cal 没生效), ok 就**恒为 False**, 循环永远不会结束 ——
    脚本卡住且不报错。所以连续 _STALL_MS 没样本就直接退出。

    ⚠ 只在样本不足时调 finish_mag_cal(): 它的作用是把 g_mc_active 清掉、
      把被 start_mag_cal 停掉的 AHRS 任务重启起来 (样本 <30 时返回 -1, 不写 NVS)。
      样本已经够多时**不能调** —— 那会拿半圈数据拟合出一个假椭球存进 NVS, 比卡死更坏。
    """
    print('\n✗ 连续 %d 秒读不到磁力计数据 —— 校准中止。' % (_STALL_MS // 1000))
    if count < 30:
        bpuppy_imu.finish_mag_cal()   # 样本不足 → 返回 -1, 不写 NVS, 只做清理
        print('  已放弃, 没有写入任何数据。检查磁力计接线后重试。')
    else:
        print('  已采 %d 个样本但中途断了 —— 未写入 (半圈数据拟合出来是错的)。' % count)
        print('  注意 IMU 任务此时仍是停的, 重新跑一次校准即可恢复。')


def _bar(pct, width=16):
    filled = int(pct / 100.0 * width)
    if filled > width:
        filled = width
    return '█' * filled + '░' * (width - filled)


def _show_stats(ri, rx, ry, rz, count):
    """打印三轴进度条"""
    for i, (name, r) in enumerate(zip(_AXIS_NAMES, (rx, ry, rz))):
        pct = min(r / _TARGETS[i] * 100.0, 100.0)
        marker = '>' if i == ri else ' '
        print(' %s %s: %s %3.0f%%  %5.0f/%3.0f uT' %
              (marker, name, _bar(pct), pct, r, _TARGETS[i]))
    print('  samples: %d' % count)


def start():
    if not bpuppy_imu.is_ready():
        bpuppy_imu.init(0, 14, 21, 0x68)  # V3.0 硬件: SDA=14, SCL=21 (电池检测走 GPIO3=ADC1)

    # 什么都没接 (或没认出来) —— 说清楚, 别往下走
    if not bpuppy_imu.is_ready():
        print('\n⚠ 没检测到 IMU —— 校准跳过。')
        print('  检查 SDA=14 / SCL=21 接线, 以及模块是否插稳。')
        return

    # ★ 判据必须是 has_mag(), **不是** get_chip() == 'mpu9250'
    #   三块板子换着插, 芯片和"有没有磁力计"是两件事:
    #     - 有模块 WHO_AM_I=0x70 (6500 核心) 却带着真的 AK8963, 9 轴齐全
    #     - 真 6500 没有磁力计; 6050 也确定没有
    #   只有 ak8963_init() 真问出 0x48 才算数, has_mag() 就是回答这个的。
    #   ⚠ 判错的后果不是"多跑一遍"而是**死循环**: start_mag_cal() 开头
    #     `if (!g_imu_ready || !g_mag_ready) return;` 直接返回, 于是
    #     mag_cal_collect() 恒返回 ok=False, 下面的 while 永远转下去。
    if not bpuppy_imu.has_mag():
        print('\n⚠ 当前 IMU = %s, 没有磁力计 → 校准跳过。' % bpuppy_imu.get_chip())
        print('  只有 6 轴 (加速度+陀螺仪), yaw 会漂, 修不了。')
        return

    print('\n===== 磁力计 3D 椭球校准 =====\n')
    print('拿起机器狗，在空中自由旋转。')
    print('依次完成 Roll / Pitch / Yaw 三轴覆盖。\n')

    bpuppy_imu.start_mag_cal()
    time.sleep_ms(100)

    for axis_idx in range(3):
        name = _AXIS_NAMES[axis_idx]
        hint = _HINTS[axis_idx]
        print('--- 阶段 %d/3: %s ---' % (axis_idx + 1, name))
        print('  %s\n' % hint)

        stall_since = time.ticks_ms()   # 读到有效样本就重置, 见 _stall_abort
        while True:
            result = bpuppy_imu.mag_cal_collect()
            ok, count = result[0], result[1]
            rx, ry, rz = result[2], result[3], result[4]

            if ok:
                stall_since = time.ticks_ms()
                r_vals = (rx, ry, rz)
                r_current = r_vals[axis_idx]
                pct = min(r_current / _TARGETS[axis_idx] * 100.0, 100.0)
                print('\n 当前轴 %s: %s %3.0f%%  %.0f/%.0f uT' %
                      (name, _bar(pct), pct, r_current, _TARGETS[axis_idx]))
                _show_stats(axis_idx, rx, ry, rz, count)
                if pct >= 100.0:
                    print('  ✓ 达标!\n')
                    break

            elif time.ticks_diff(time.ticks_ms(), stall_since) > _STALL_MS:
                _stall_abort(count)
                return

            time.sleep_ms(20)  # ~50Hz 采集

    # 覆盖达标后, 补足最小样本数 (保证椭球拟合质量)
    if count < MIN_SAMPLES:
        print('\n--- 补充采样: 请继续自由旋转, 采够 %d 样本 (当前 %d) ---'
              % (MIN_SAMPLES, count))
        stall_since = time.ticks_ms()
        while count < MIN_SAMPLES:
            result = bpuppy_imu.mag_cal_collect()
            ok, count = result[0], result[1]
            if ok:
                stall_since = time.ticks_ms()
                if count % 25 == 0:
                    pct = min(count / MIN_SAMPLES * 100.0, 100.0)
                    print('  样本: %s %3.0f%%  %d/%d' %
                          (_bar(pct), pct, count, MIN_SAMPLES))
            elif time.ticks_diff(time.ticks_ms(), stall_since) > _STALL_MS:
                _stall_abort(count)
                return
            time.sleep_ms(20)
        print('  样本已够: %d\n' % count)

    print('正在拟合椭球...')
    time.sleep_ms(200)
    resid = bpuppy_imu.finish_mag_cal()

    if resid < 0:
        print('✗ 校准失败! 请重试。')
    else:
        print('✓ 校准完成! 残差: %.4f' % resid)
        if resid > 0.08:
            print('  ⚠ 残差偏大 (理想 <0.05), 拟合质量差, 建议重新校准')
        else:
            print('  硬铁 + 软铁校正已保存到 NVS')
