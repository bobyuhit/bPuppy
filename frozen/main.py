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
def _vfs_is_virgin(part):
    """分区首扇区全是 0xFF = 擦除态 = 出厂新片, 可以安全格式化。

    只要有任何一个字节不是 0xFF, 说明上面**有东西** (哪怕文件系统已损坏),
    这时绝不能格 —— mkfs 会抹掉整个 8MB vfs, 用户的程序全在里面,
    而挂载失败的原因往往只是瞬时的 (分区正被写 / 上次没干净卸载)。
    """
    try:
        buf = bytearray(512)
        part.readblocks(0, buf)
        return all(b == 0xFF for b in buf)
    except Exception as e:
        # 读不出来就更不该格
        print("[bPuppy] VFS 首扇区读取失败: %s (按'有数据'处理)" % e)
        return False


_vfs_mounted = False
try:
    from esp32 import Partition
    _bparts = Partition.find(1, label='vfs')  # TYPE_DATA=1
    if not _bparts:
        print("[bPuppy] [WARN] 找不到 vfs 分区, 用户程序不可用")
    else:
        _bdev = _bparts[0]
        try:
            uos.mount(uos.VfsFat(_bdev), '/')   # 已格式化, 直接挂载
            _vfs_mounted = True
        except Exception as e:
            # 实测 (2026-09-16): 原先是裸 except + 无条件 mkfs —— 任何挂载失败都会
            # 格式化整个 vfs, 把用户程序抹光。改成只在"确认是全新分区"时才格。
            if _vfs_is_virgin(_bdev):
                print("[bPuppy] VFS 是未格式化的新分区, 正在格式化...")
                uos.VfsFat.mkfs(_bdev)
                uos.mount(uos.VfsFat(_bdev), '/')
                _vfs_mounted = True
            else:
                print("[bPuppy] [ERROR] VFS 挂载失败: %s" % e)
                print("[bPuppy]         分区上有数据, 已拒绝自动格式化 (避免抹掉用户程序)")
                print("[bPuppy]         本次用户程序不可用; 确需重建时手动执行 (⚠ 会清空 vfs):")
                print("[bPuppy]         import esp32, os; os.VfsFat.mkfs(esp32.Partition.find(1,label='vfs')[0])")
except Exception as e:
    print("[bPuppy] [WARN] VFS 初始化异常: %s" % e)

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
# 实测 (2026-08-19): 本固件 sys.modules['__main__'] 为 None, 事件积木函数
# (voiceWhen*) 必须从主脚本全局 dict 扫描。exec(_user_code) 在下方同一全局执行,
# 函数定义后会进入这个 dict, voice 后台线程周期性扫描即能注册。
voice.set_main_globals(globals())

# ============================================================
# 用户程序 (从站姿切入)
# ============================================================
# 实测 (2026-09-16): 用户程序必须放在**后台线程**里跑, 不能在主线程 exec。
# 原因: KittenBlock 的「重复执行」会生成顶格 while True:, 而键盘积木「按下x键?」
# 下载后又退化成恒假的 if False: (按键检测在浏览器侧) → 循环体空转、无 sleep。
# 在主线程 exec 它 → exec 永不返回 → 主线程走不到 REPL (mpy_startup.c 的
# for(;;) pyexec_friendly_repl()) → KittenBlock 整个失联, 且每次复位都卡,
# 只能串口 Ctrl-C + 删 /main.py 才救得回来。
# 放线程后 exec 不再阻塞主线程, 失败模式从"板子变砖"降级为"程序没反应"。
def _run_user(_code):
    try:
        # 显式传 globals(): 让用户程序的顶层赋值/def 落进 main.py 的模块全局 dict
        # (MicroPython 的 mp_locals_get() 是线程级而非函数帧级, 不传其实也等价;
        #  但显式写出来才不会因日后重构而悄悄丢掉语音事件的注册链路)
        exec(_code, globals())
        print("[bPuppy] 用户程序结束, 回到 REPL")
    except BaseException as e:  # 必须 BaseException: SystemExit/KeyboardInterrupt 不在 Exception 下
        print("[bPuppy] 用户程序异常: %s" % e)


if _vfs_mounted:
    try:
        with open('/main.py', 'r') as f:
            _user_code = f.read()
    except OSError:
        _user_code = None  # 无用户程序 → 走下面 Ready 兜底

    if _user_code:
        print("[bPuppy] 运行用户程序 (后台线程)...")
        import _thread
        # 线程默认栈只有 5KB (mpthreadport.c MP_THREAD_DEFAULT_STACK_SIZE)。用户程序
        # 里 import VFS 上的 .py 会在线程栈上编译 → 栈不够是 FreeRTOS panic 重启
        # (MICROPY_STACK_CHECK 护不住解析器)。抬到与主任务栈相同的 16KB;
        # stack_size 只影响此后新建线程, 已启动的 voice 线程不受影响。
        _thread.stack_size(16 * 1024)
        # 线程继承 main.py 的模块全局 dict (= 上面 voice.set_main_globals 传的同一个),
        # exec 新定义的 voiceWhen* 仍会被 voice 后台线程扫到并注册。
        _thread.start_new_thread(_run_user, (_user_code,))
        # 立刻还原默认栈: stack_size 是**全局状态**, 上面那个 16KB 只该给用户线程。
        # 不还原的话, 此后每个模块起线程都按 16KB 要 (camera_stream 就有 3 处:
        # _accept_loop / _dns_server / 每个客户端一个 _send_stream), 内部 RAM
        # 不够时 start_new_thread 直接抛 OSError: can't create thread —— 而且
        # 是间歇性的 (看当时碎片), 比必然失败更难查。实测踩过。
        # 上面的 start_new_thread 是同步的: 16KB 在它返回前就已分配好, 所以这行
        # 不影响用户线程, 只影响"之后"新建的线程。
        _thread.stack_size(0)          # 0 = 端口默认 (esp32: MP_THREAD_DEFAULT_STACK_SIZE)
        # 主线程继续往下, 打印 Ready 并进入 REPL

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
