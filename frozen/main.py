"""
bPuppy 机器狗 — MicroPython 启动脚本
在固件烧录后自动执行（frozen 模式）
"""

import gc
import sys

# 构建版本
try:
    import bpuppy
    version = bpuppy.version()
except Exception:
    version = "unknown"

print("=" * 44)
print("  bPuppy Robot Dog - ESP32-S3 MicroPython")
print(f"  Build: {version}")
print("  WROOM-1 N16R8 | uPy v1.22.1 | IDF v5.1.2")
print("=" * 44)

# ---- 加载 C 驱动模块 ----
modules_loaded = []
modules_failed = []

# 舵机驱动 — 自动初始化 + 从 NVS 加载校准
try:
    import bpuppy_servo
    bpuppy_servo.init_all()
    bpuppy_servo.load_cal()
    modules_loaded.append("servo")
except ImportError as e:
    modules_failed.append(("servo", e))

# IMU 传感器 — QMI8658 (暂时屏蔽，待引脚确认)
# try:
#     import bpuppy_imu
#     bpuppy_imu.init(0, 1, 2, 0x6A)  # I2C0, SDA=GPIO1, SCL=GPIO2, addr=0x6A
#     modules_loaded.append("imu")
# except ImportError as e:
#     modules_failed.append(("imu", e))

# UART 通信（骨架）
try:
    import bpuppy_uart
    modules_loaded.append("uart")
except ImportError as e:
    modules_failed.append(("uart", e))

# 运动控制 — 上电自动站立
try:
    import bpuppy_motion
    # 先将舵机设到蹲姿，避免 start() 瞬间跳变
    # 注意: 改 ik.h IK_KNEE_REAR_FORWARD 后需同时切换下面两行
    # 四腿后弯 (IK_KNEE_REAR_FORWARD=0):
    # bpuppy_servo.set_angle(0, 135);  bpuppy_servo.set_angle(1, 45)   # LF
    # bpuppy_servo.set_angle(2, 135);  bpuppy_servo.set_angle(3, 45)   # LH
    # bpuppy_servo.set_angle(4, 45);   bpuppy_servo.set_angle(5, 135)  # RF
    # bpuppy_servo.set_angle(6, 45);   bpuppy_servo.set_angle(7, 135)  # RH
    # 后腿前弯 (IK_KNEE_REAR_FORWARD=1):
    bpuppy_servo.set_angle(0, 135);  bpuppy_servo.set_angle(1, 45)   # LF: hip后指 knee后折
    bpuppy_servo.set_angle(2, 45);   bpuppy_servo.set_angle(3, 135)  # LH: hip前指 knee前凸
    bpuppy_servo.set_angle(4, 45);   bpuppy_servo.set_angle(5, 135)  # RF: hip前指 knee后折
    bpuppy_servo.set_angle(6, 135);  bpuppy_servo.set_angle(7, 45)   # RH: hip后指 knee前凸
    bpuppy_motion.start()
    bpuppy_motion.stand_up()
    modules_loaded.append("motion")
except ImportError as e:
    modules_failed.append(("motion", e))

# BLE 遥控 — Hiwonder Wonderbot App 兼容 (NimBLE C 驱动)
try:
    import ble_hiwonder
    ble = ble_hiwonder.HiwonderBLE()

    # 后台线程轮询 BLE 指令, REPL 不受影响
    try:
        import _thread
        _thread.start_new_thread(ble_hiwonder.run_ble_task, (ble, 50))
        print("  [BLE] 后台轮询已启动")
    except Exception as e:
        print("  [BLE] 后台线程启动失败: %s (需手动调用 ble.check())" % e)

    modules_loaded.append("ble")
except Exception as e:
    modules_failed.append(("ble", e))

# ---- 状态报告 ----
print(f"  Loaded:   {', '.join(modules_loaded) if modules_loaded else '(none)'}")
if modules_failed:
    for name, err in modules_failed:
        print(f"  [WARN] {name}: {err}")

# ---- 内存信息 ----
gc.collect()
try:
    import micropython
    micropython.mem_info()
except Exception:
    pass

print("Ready.")
print(">>> bpuppy_motion.set_gait('go')   # 自适应前进")
print(">>> bpuppy_motion.set_gait('sit')  # 蹲下")
print()
