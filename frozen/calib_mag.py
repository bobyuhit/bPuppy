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
_AXIS_NAMES = ('X (Roll)', 'Y (Pitch)', 'Z (Yaw)')
_HINTS = (
    '请左右倾斜机器狗，覆盖 Roll 范围',
    '请前后倾斜机器狗，覆盖 Pitch 范围',
    '请水平旋转机器狗，覆盖 Yaw 360°',
)


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
    print('\n===== 磁力计 3D 椭球校准 =====\n')
    print('拿起机器狗，在空中自由旋转。')
    print('依次完成 Roll / Pitch / Yaw 三轴覆盖。\n')

    bpuppy_imu.start_mag_cal()
    time.sleep(0.1)

    for axis_idx in range(3):
        name = _AXIS_NAMES[axis_idx]
        hint = _HINTS[axis_idx]
        print('--- 阶段 %d/3: %s ---' % (axis_idx + 1, name))
        print('  %s\n' % hint)

        while True:
            result = bpuppy_imu.mag_cal_collect()
            ok, count = result[0], result[1]
            rx, ry, rz = result[2], result[3], result[4]

            if ok:
                r_vals = (rx, ry, rz)
                r_current = r_vals[axis_idx]
                pct = min(r_current / _TARGETS[axis_idx] * 100.0, 100.0)
                print('\n 当前轴 %s: %s %3.0f%%  %.0f/%.0f uT' %
                      (name, _bar(pct), pct, r_current, _TARGETS[axis_idx]))
                _show_stats(axis_idx, rx, ry, rz, count)
                if pct >= 100.0:
                    print('  ✓ 达标!\n')
                    break

            time.sleep(0.02)  # ~50Hz 采集

    print('正在拟合椭球...')
    time.sleep(0.2)
    resid = bpuppy_imu.finish_mag_cal()

    if resid < 0:
        print('✗ 校准失败! 请重试。')
    else:
        print('✓ 校准完成! 残差: %.4f' % resid)
        print('  硬铁 + 软铁校正已保存到 NVS')
