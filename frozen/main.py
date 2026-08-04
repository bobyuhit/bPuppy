"""
bPuppy 机器狗 — MicroPython 启动脚本
在固件烧录后自动执行（frozen 模式）

启动顺序:
  1. 挂载 VFS (Flash 文件系统)
  2. 启动 BLE (KittenBlock 蓝牙 / Hiwonder, 由固件编译模式决定)
  3. 检查 /main.py (KittenBlock 上传的用户程序)
  4. 有 → 执行用户程序
  5. 无 → 执行原厂默认逻辑 (站立)

WiFi 热点: 上电默认不开 (KittenBlock 蓝牙优先)。需要时手动
  import camera_stream; camera_stream.start()
"""

import gc
import sys
import os, uos

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

# ---- 挂载 VFS ----
_vfs_mounted = False
try:
    from esp32 import Partition
    _bdev = Partition.find(1, label='vfs')  # TYPE_DATA=1
    if _bdev:
        try:
            uos.mount(uos.VfsFat(_bdev[0]), '/')   # 已格式化, 直接挂载
        except:
            uos.VfsFat.mkfs(_bdev[0])              # 首次使用, 先格式化
            uos.mount(uos.VfsFat(_bdev[0]), '/')
        _vfs_mounted = True
except Exception:
    pass

# ---- 启动 BLE (固件编译模式决定: KittenBlock Nordic / Hiwonder FFE0) ----
# KittenBlock 模式时 C 层 (ble_driver_mpy.c) 自动注册 dupterm REPL 通道
try:
    import bpuppy_ble
    bpuppy_ble.start()
except Exception:
    pass

# ---- 检查用户程序 (KittenBlock 上传为 /main.py) ----
if _vfs_mounted:
    try:
        with open('/main.py', 'r') as f:
            _user_code = f.read()
        print("[bPuppy] 运行用户程序...")
        exec(_user_code)
        print("Ready.")
        # 用户程序执行完, 不再跑原厂默认
        raise SystemExit
    except OSError:
        pass  # 无用户程序
    except SystemExit:
        raise  # 传递出去, 真正退出

# ---- 原厂默认: 加载 C 驱动模块 ----
if not _vfs_mounted:
    print("[bPuppy] VFS 未挂载, 使用原厂默认")
else:
    print("[bPuppy] 无用户程序, 使用原厂默认")

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

# IMU / UART / ADC / BLE — 手动或按需启动, 上电不 import
#   bpuppy_imu  由 balance / set_heading / calib_mag 的 start() 自动 init
#   bpuppy_uart / bpuppy_adc  用的时候手动 import + init()
#   BLE         由 ble_hiwonder.HiwonderBLE() 构造时自动启动
#   WiFi 遥控   上电自动开启 (见下), 纯遥控不开摄像头

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

# WiFi 遥控 — 上电默认不开 (KittenBlock 蓝牙优先, 避免 RF 争用)
# 需要 WiFi 时手动: import camera_stream; camera_stream.start()
# 图传: camera_stream.start(stream=True) 或网页点「图传 开」
try:
    import camera_stream  # 仅 import, 不启动热点
    modules_loaded.append("wifi_off")
except Exception as e:
    modules_failed.append(("wifi", e))

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
