"""
bPuppy 机器狗 — MicroPython 启动脚本
在固件烧录后自动执行（frozen 模式）

启动顺序:
  1. 挂载 VFS (Flash 文件系统)
  2. 启动 BLE (KittenBlock 蓝牙 / Hiwonder, 由固件编译模式决定)
  3. 原厂无条件初始化: IDLE → POSE → 站姿待命
  4. 电池电压检测 + WS2812 指示灯 (frozen/voltage.py, 上电默认)
  5. 语音控制 — CI-33T 语音模块 (frozen/voice.py, UART2/9600, 上电默认)
  6. 检查 /main.py (用户程序)
  7. 有 → exec 用户程序 (从站姿切入)
  8. 无 → Ready 待命

WiFi 热点: 上电默认不开 (KittenBlock 蓝牙优先)。需要时手动
  import camera_stream; camera_stream.start()
"""

import gc
import sys
import os, uos
import time

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

# ============================================================
# 原厂初始化 (无条件执行) — IDLE → POSE → 站姿待命
# ============================================================
# 舵机初始化 (init_all 不触发切 POSE, 保持 IDLE)
try:
    import bpuppy_servo
    bpuppy_servo.init_all()
    bpuppy_servo.load_cal()
except Exception as e:
    print("  [WARN] servo: %s" % e)

# 加载 motion / poses
try:
    import bpuppy_motion
    import poses
except Exception as e:
    print("  [WARN] motion: %s" % e)

time.sleep(0.5)   # 初始化稳定

# 蹲姿 (set_angle 放最后, 触发 IDLE→POSE 自动进入姿态模式)
# 后腿前弯 (IK_KNEE_REAR_FORWARD=1)
bpuppy_servo.set_angle(0, 135);  bpuppy_servo.set_angle(1, 45)   # LF
bpuppy_servo.set_angle(2, 45);   bpuppy_servo.set_angle(3, 135)  # LH
bpuppy_servo.set_angle(4, 45);   bpuppy_servo.set_angle(5, 135)  # RF
bpuppy_servo.set_angle(6, 135);  bpuppy_servo.set_angle(7, 45)   # RH

# 默认姿态 POSE_STAND (Python 站姿, 留在姿态模式)
poses.stand()

# 电池电压检测 + WS2812 指示灯 (frozen/voltage.py, 上电默认; 逻辑在模块内, import 即启动)
import voltage

# 语音控制 — CI-33T 语音模块 (frozen/voice.py, UART2/9600, 上电默认; import 即启动)
import voice

# ============================================================
# 用户程序 (从站姿切入)
# ============================================================
if _vfs_mounted:
    try:
        with open('/main.py', 'r') as f:
            _user_code = f.read()
        print("[bPuppy] 运行用户程序...")
        exec(_user_code)
        print("Ready.")
        # 用户程序执行完, 不再跑后续
        raise SystemExit
    except OSError:
        pass  # 无用户程序
    except SystemExit:
        raise  # 传递出去, 真正退出

# ---- Ready (站姿待命) ----
gc.collect()
try:
    import micropython
    micropython.mem_info()
except Exception:
    pass

print("Ready.")
print(">>> poses.crouch() / poses.stand()  # 姿态模式")
print(">>> bpuppy_motion.set_gait('go')    # 运动模式")
print()
