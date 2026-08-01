"""
plot_imu.py — IMU 姿态角实时曲线显示

用法:
    python plot_imu.py COM8

ESP32 端先运行采集循环:
    import bpuppy_imu,time
    while True:
     r,p,y=bpuppy_imu.read_angles()
     print('%6.1f,%6.1f,%7.1f'%(r,p,y))
     time.sleep(0.02)

依赖: pip install pyserial matplotlib
"""

import sys
import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from collections import deque
import re

MAX_POINTS = 300
BAUDRATE = 921600

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM8'
    ser = serial.Serial(port, BAUDRATE, timeout=0.1)
    print(f"已连接 {port}, 等待数据...")

    t_data = deque(maxlen=MAX_POINTS)
    r_data = deque(maxlen=MAX_POINTS)
    p_data = deque(maxlen=MAX_POINTS)
    y_data = deque(maxlen=MAX_POINTS)
    counter = [0]

    fig, ax = plt.subplots(figsize=(14, 6), facecolor='white')
    fig.suptitle('IMU Roll / Pitch / Yaw', fontsize=15, fontweight='bold')

    ax.set_ylabel('Angle (°)', fontsize=12)
    ax.set_ylim(-180, 180)
    ax.grid(True, alpha=0.25)
    ax.axhline(y=0, color='#ccc', alpha=0.5, lw=1, ls='--')
    ax.set_facecolor('#fafafa')
    ax.set_xlabel('samples', fontsize=11)

    colors = {'Roll': '#e74c3c', 'Pitch': '#2ecc71', 'Yaw': '#3498db'}
    line_r, = ax.plot([], [], color=colors['Roll'], lw=1.0, label='Roll')
    line_p, = ax.plot([], [], color=colors['Pitch'], lw=1.0, label='Pitch')
    line_y, = ax.plot([], [], color=colors['Yaw'], lw=1.0, label='Yaw')
    ax.legend(loc='upper right', fontsize=10, ncol=3)

    # 数值标注
    val_text = ax.text(0.02, 0.96, '', transform=ax.transAxes,
                       fontsize=13, fontfamily='monospace', va='top',
                       bbox=dict(boxstyle='round,pad=0.4', facecolor='white',
                                 edgecolor='#ccc', alpha=0.92))

    csv_re = re.compile(r'^\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)')

    def poll_serial():
        try:
            raw = ser.read(ser.in_waiting or 1).decode('utf-8', errors='ignore')
        except:
            return
        for line in raw.splitlines():
            m = csv_re.match(line.strip())
            if m:
                counter[0] += 1
                t_data.append(counter[0])
                r_data.append(float(m.group(1)))
                p_data.append(float(m.group(2)))
                y_data.append(float(m.group(3)))

    def update(frame):
        poll_serial()
        if t_data:
            t = list(t_data)
            line_r.set_data(t, list(r_data))
            line_p.set_data(t, list(p_data))
            line_y.set_data(t, list(y_data))
            c = counter[0]
            ax.set_xlim(max(0, c - MAX_POINTS), max(MAX_POINTS, c + 5))

            r, p, y = r_data[-1], p_data[-1], y_data[-1]
            val_text.set_text(
                f'  Roll  {r:+6.1f}°    Pitch  {p:+6.1f}°    Yaw  {y:+7.1f}°  ')

        return line_r, line_p, line_y, val_text

    ani = animation.FuncAnimation(fig, update, interval=30, blit=False, cache_frame_data=False)
    plt.subplots_adjust(left=0.06, right=0.98, top=0.90, bottom=0.08)
    plt.show()
    ser.close()

if __name__ == '__main__':
    main()
