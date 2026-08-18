# bPuppy — ESP32-S3 四足机器狗

## 产品概述

bPuppy 是基于 ESP32-S3 的 8 自由度四足机器狗（4腿 × 2DOF：髋+膝），运行 MicroPython v1.22.1 + ESP-IDF v5.1.2。
底层 C 驱动（舵机、IMU、BLE、IK、步态），上层 Python 应用，兼容 Hiwonder Wonderbot App 蓝牙遥控。

| 项目 | 规格 |
|------|------|
| 主控 | ESP32-S3 WROOM-1 N16R8 (16MB Flash, 8MB Octal PSRAM) |
| 舵机 | 8× 模拟舵机 (每条腿 2DOF: 髋 + 膝) |
| IMU | MPU6050/MPU9250 双芯片自适应 (V2.0: I2C0 SDA=GPIO3, SCL=GPIO21, addr=0x68; WHO_AM_I 自动识别: 6050=6轴无磁力计, 9250=9轴含 AK8963; Mahony 姿态) |
| 通信 | BLE (NimBLE, 编译互斥: KittenBlock Nordic UART 或 Hiwonder FFE0) + UART2 (GPIO19/20, CI-33T/micro:bit) |
| 控制台 | USB-JTAG CDC (921600bps, 直连 USB) |
| 供电 | 7.4V 2S LiPo |

**当前固件参数（默认值，均可运行时修改 + NVS 持久化）：**

| 参数 | 默认值 | 运行时修改 |
|------|--------|----------|
| 大腿 L1 | 40mm | `cal_ik(L1, L2)` → NVS |
| 小腿 L2 | 45mm | `cal_ik(L1, L2)` → NVS |
| 前后髋距 | 125mm | `set_body_dims(bl, bw)` → NVS |
| 左右髋宽 | 118mm | `set_body_dims(bl, bw)` → NVS |
| 膝角范围 | 10°~170° | `set_joint_limits()` → NVS |
| 髋角范围 | 0°~180° | `set_joint_limits()` → NVS |
| 速度范围 | 0~10 | `set_params()` |
| 抬腿默认 | 30mm | `set_lift()` |
| 脚中位偏移 | 0mm | `set_center()` |
| Walk 最优 | speed=2.5, stride=70, height=70 | 实测 |
| Trot 最优 | speed=8.5, stride=70, height=60 | 实测 |

---

## 编译环境

### 版本铁律

```
MicroPython v1.22.1  ──只能搭配──▶  ESP-IDF v5.1.2
                                    ⚠ 不可用 v5.3 / v5.5
```

### Docker 编译（唯一方式，带 ccache 加速）

**编译准则（AI 和人类共同遵守）：**

1. 始终在**宿主机 Git Bash** 中运行 `build.sh`，不要在容器内手动编译
2. `build.sh` 已自动处理 `MSYS_NO_PATHCONV=1`（防止 Git Bash 路径转换错误）
3. `build/` 目录不提交 git
4. 每台电脑首次使用前建一次 ccache 目录
5. **编译和烧录过程必须实时向用户汇报进度**（每秒一次），不汇报用户会终止操作
6. **关键成功节点立即 git commit**，方便崩了回退
7. **参数调优前先 commit**，避免意外全量重编

```bash
# === 首次设置（新电脑上只需一次） ===
mkdir -p ~/.ccache_bpuppy

# === 日常编译 ===
bash build.sh

# === 改了 CMakeLists.txt / sdkconfig / idf_component.yml ===
rm -rf build && bash build.sh
```

**ccache 缓存原理**：`~/.ccache_bpuppy` 挂载到容器内 `/root/.ccache`，容器销毁后缓存不丢。增量编译从 1356 步降到 ~10 步，几秒完成。

**多台电脑**：每台电脑各自维护 `~/.ccache_bpuppy`，互不影响。`build/` 目录也在各自电脑上独立存在。

**产物**: `build/micropython_bpuppy.bin`

### Windows 烧录 (PowerShell)

```powershell
# ESP32-S3 原生 USB-JTAG，直连 USB 即可，无需 USB-UART 转接
# 端口: 插入 USB 后从设备管理器查看 (因机器而异)
esptool --chip esp32s3 --port COM3 --baud 921600 write-flash `
  0x10000 build/micropython_bpuppy.bin
```

> **日常增量**只写 app 分区 (0x10000)。改了 bootloader/分区表、或首次烧录时才需要**全烧**:
>
> ```powershell
> esptool --chip esp32s3 --port COM3 --baud 921600 write-flash `
>   0x0 build/bootloader/bootloader.bin `
>   0x8000 build/partition_table/partition-table.bin `
>   0x10000 build/micropython_bpuppy.bin
> ```

> 成功标志: 三行 `Wrote xxx bytes` + `Hash of data verified`。烧完按 RESET 或重新上电。

### 串口连接

- 波特率: **921600**
- 端口: 设备管理器查看 (USB-JTAG CDC，直连 USB 即可)
- 工具: PuTTY / Tera Term / VS Code Serial Monitor

烧录后重启，应看到:
```
============================================
  bPuppy Robot Dog - ESP32-S3 MicroPython
  Build: ...
  WROOM-1 N16R8 | uPy v1.22.1 | IDF v5.1.2
============================================
  Loaded:   servo, motion
Ready.
>>>
```

### 改了哪些文件需要怎么构建

| 改了什么 | 怎么构建 |
|---------|---------|
| `frozen/*.py` | `bash build.sh` → 只烧 app 分区 |
| `drivers/*.c/.cpp` | `bash build.sh` → 只烧 app 分区 |
| `CMakeLists.txt` / `sdkconfig.*` / `partitions.csv` | `rm -rf build && bash build.sh` → **全烧** |

---

## 舵机角度标定

**髋部 (HIP):**

| 腿侧 | 0° | 90° | 180° |
|------|-----|-----|------|
| 左腿 (LF/LH) | 指向前 | 指向下 | 指向后 |
| 右腿 (RF/RH) | 指向后 | 指向下 | 指向前 |

**膝部 (KNEE):** IK `knee_angle = 0°` 完全折叠, `180°` 完全伸直。

| 左膝 servo | IK knee | 状态 | 右膝 servo | IK knee | 状态 |
|-----------|---------|------|-----------|---------|------|
| 0° | 0° | 折叠 | 180° | 0° | 折叠 |
| 90° | 90° | 直角 | 90° | 90° | 直角 |
| 180° | 180° | 伸直 | 0° | 180° | 伸直 |

---

## 软件架构

```
MicroPython 层:   frozen/main.py → 上电自动站立 + WiFi 遥控 (IMU/BLE/UART/ADC 手动或按需启动)
                       ↑ import
C Extension API:  bpuppy_servo / bpuppy_imu / bpuppy_uart / bpuppy_adc /
                  bpuppy_ik / bpuppy_motion / bpuppy_camera / bpuppy_ble
                       ↑ MP_REGISTER_MODULE
C 驱动层:
  servo_driver.c    — LEDC PWM 8路舵机 (S3 统一 LS mode) + NVS 校准
  imu_driver.c      — I2C MPU6050/MPU9250 双芯片自适应 (WHO_AM_I 识别, 6050=6轴无磁力计, 9250=9轴 Mahony + 磁力计椭球校准)
  uart_driver.c     — UART2 通信口 + UART1 摄像头复用口 + I2C1 摄像头复用口
  adc_driver.c      — ADC 电池检测 (V2.0 停用: 电池=GPIO14/ADC2 BLE 下不可用; V3.0 改 GPIO3/ADC1_CH2)
  ik.h / ik.c       — 2-DOF 逆运动学
  ble_driver.c      — NimBLE GATT (编译互斥: KittenBlock Nordic / Hiwonder FFE0)
  ble_stream.c      — BLE 流对象 (dupterm REPL 桥接, KittenBlock 模式)
  motion_task.cpp   — 50Hz FreeRTOS 步态控制 (core 0, priority 6)
  motion_task_mpy.c — motion 的 MicroPython 绑定
                       ↑
FreeRTOS:          ESP-IDF v5.1.2
                       ↑
硬件:              ESP32-S3 WROOM-1 N16R8
```

### 关键文件

| 文件 | 说明 |
|------|------|
| `drivers/motion_task.cpp` | **核心** — 步态算法、相位框架、足端轨迹、IK、GO自适应 |
| `drivers/ik.c` | 2-DOF 逆运动学, L1/L2/髋距/限位 均运行时可变 + NVS 持久化 |
| `drivers/servo_driver.c` | LEDC PWM + NVS 校准 (`cal(ch, ref_deg)`) |
| `drivers/imu_driver.c` | MPU6050/MPU9250 双芯片自适应 (WHO_AM_I 识别, 6050 跳过磁力计), Mahony 姿态融合, 校准存 NVS |
| `drivers/uart_driver.c` | UART2 (GPIO19/20) + UART1 (GPIO4/5) 通信驱动 |
| `drivers/adc_driver.c` | ADC 电池检测 (V2.0 停用; V3.0: GPIO3=ADC1_CH2, 分压 51k/10k) |
| `drivers/ble_driver.c` | NimBLE GATT 服务 — 编译互斥 (KittenBlock Nordic / Hiwonder FFE0) |
| `drivers/ble_stream.c` | BLE 流对象 — dupterm REPL 桥接 (KittenBlock 蓝牙) |
| `drivers/micropython.cmake` | `BPUPPY_BLE_KEBLOCK` / `BPUPPY_BLE_HIWONDER` 编译宏 |
| `components/mr9you__micropython-helper` | MicroPython 移植层 (mphalport.c 补 dupterm 输入) |
| `kext-bpuppy/` | KittenBlock 硬件扩展 (15 积木 + 蓝牙配置 + 开发文档) |
| `frozen/main.py` | 启动脚本 — 原厂初始化 → 站姿待命 (POSESTAND), 用户程序从站姿切入 |
| `frozen/balance.py` | 站立自平衡 — 增量式 PID, 50Hz 闭环 (绕过 motion task) |
| `frozen/camera_stream.py` | WiFi 热点 MJPEG 图传 + 网页遥控器 |
| `frozen/ble_hiwonder.py` | BLE 遥控协议 — GO 自适应, speed 0~10 |
| `frozen/camera_serial.py` | 串口拍照回传 — 通过 REPL 触发拍照，base64 回传 PC |
| `drivers/camera_driver.c` | OV2640 DVP 驱动 + MicroPython 绑定 (`bpuppy_camera`) |
| `tools/capture.py` | PC 端拍照工具 — 通过串口命令拍照并自动保存/预览 |
| `gait_sim/gait_sim.py` | PC 端步态仿真 — CSV/PNG/GIF |
| `docs/操作指南.md` | 日常操作手册 |
| `docs/camera_design.md` | 摄像头功能设计文档 |

---

## 步态算法概要 (motion_task.cpp)

### GO 自适应

| speed | duty | gap | stride | height | pitch | 实际 |
|-------|------|-----|--------|--------|-------|------|
| ≤4 | 0.20 | 0.04 | 70 | 70 | 0° | walk |
| 4~6 | 插值 | 插值 | 70→50 | 70 | 0° | 混合 |
| ≥6 | 0.40 | 0.10 | 50 | 70 | 0° | trot |

lift 继承 `g_motion.lift_height` (默认 30mm)。实际 speed 经半周期平滑跟随 `target_speed`。

### 姿态过渡

- **静态↔静态** (GAIT_STOP ↔ 运动步态): 每帧限速 3° smoothstep, 自然平滑
- **变速**: 每半周期 ±3.0 步进跟随 target_speed, 避免突变
- **姿态模式 (crouch/sit/play/wave/stand)**: 由 Python `poses` 模块 `_move_to` 限速逼近, 与 C 层机制一致

---

### GO 起步

1. **kick**: 静态→GO 瞬间速度从 0 → min(目标, 2.5), 打破相位死锁
2. **首值注入**: 不等半周期边界, 立刻 stride = 目标/5 (速度 5 时 ~12mm, 第一脚就迈)
3. **速度爬升**: 每半周期 +3 到目标 (速度 5: 2.5→5, 1 半周期)
4. **步长爬升**: 每半周期 +目标/5, **5 半周期**从 1/5 爬到目标

**示例 (速度 5, 目标步长 60)**:

| 半周期 | 速度 | 步长 | 时长 |
|--------|------|------|------|
| kick+首值 | 0→2.5 | →12 | 即时 |
| HP1 | 2.5→5 | 12→24 | ~0.6s |
| HP2 | 5 | 24→36 | ~0.3s |
| HP3 | 5 | 36→48 | ~0.3s |
| HP4 | 5 | 48→60 | ~0.3s |

> 起步总长约 1.5s, 全程小步平滑展开。pose_trans=1 (0.3s) 足端从站立缓动到轨迹。

### GO 停步

1. **步长急刹**: 收停信号 → 立刻切 stride = 目标/3 (不等半周期)
2. **收尾**: 1 个半周期后 stride → 0
3. **静态切换**: stride=0 → 切静态 (stop 等), 速度清零
4. **速度不变**: 停步期间速度照常跟随 target, 不人为减速

**示例 (速度 5, 步长 60)**:

| 阶段 | 步长 | 时长 |
|------|------|------|
| 即刻切 1/3 | 60→20 | 即时 |
| 1 半周期 | 20→0 | ~0.3s |
| → 切 stop 收脚 | — | ~0.3s |

> 停步干脆, 全程 ~0.6s。最后一步 stride=20 很短, 接近 stop 姿态。

### GO 换向 (前进↔后退)

1. **步长急刹**: 检测到方向翻转 → 立刻切 1/3 → 1 半周期后归零
2. **方向保持**: 减速期 stride_sign 保持原方向 (防腿打架猛停)
3. **直接反向**: stride=0 立刻新方向走, **不经过 stop 中间态** (省 0.6s)
4. **反向起步**: 首值注入, 步长从 1/5 重新爬升 (速度保持)

> 换向间隔 = 急刹 ~0.3s + 收尾 ~0.3s = ~0.6s, 无冗余停顿。

### GO 暂停恢复 (滑块 0→N)

滑块拖到 0: stride→0 → 自动切 stop (等同于停步)。
滑块拖回 >0: 新起步, kick + 首值注入, 步长从 1/5 爬升。

### 网页方向键

未拖滑块时用系统 target_speed (默认 2.5), 不覆盖预设。
方向键 URL: /cmd?stride=±70&turn=X, 经 `_effective_speed()` 获取实际速度。

### 静态姿态

| gait | 说明 | 特点 |
|------|------|------|
| `"stop"` | 停止站好 (GAIT_STOP) | IK 计算, 高度由 `set_params` 设定, 留运动模式 |
| `"walk"` | 猫步 | 持续步态 |
| `"trot"` | 小跑 | 持续步态 |
| `"go"` | 自适应 | 持续步态, 推荐 |

**姿态模式 (Python `poses`):**

| 调用 | 说明 |
|------|------|
| `poses.stand()` | 站姿 POSE_STAND, 固定高度, 留姿态模式 |
| `poses.crouch()` | 蹲伏, 固定角度 |
| `poses.sit()` | 猫坐 |
| `poses.play()` | 邀玩 (前低后高 + 4Hz 摇臀 8 次) |
| `poses.wave()` | 挥手 (坐下 + 右前膝摆动 3 次, 回坐) |

> `play` / `wave` 属于"静态姿势 + 单次动态动作": 姿势先就位, 动作执行完回到静态。
> 模式自动切换: `set_gait(go/walk/trot/stop)` → MOTION; Python 写舵机 (set_angle/group_commit/cal) → POSE。
> 代码中二者归为 static gait (`is_static_gait`), 不应用行走参数。

### 足端轨迹 (smoothstep 摆线)

```
摆动相: ease = t²(3-2t), z = H - lift·sin(ease·π), x = -S/2 + S·ease
支撑相: z = H, x = S/2 - S·(p-duty)/(1-duty)
```

---

## 上电行为

1. `servo_init_all()` + `load_cal()` — 初始化 8 路 LEDC + 从 NVS 加载校准值 (保持 IDLE)
2. `sleep(0.5)` — 初始化稳定
3. 舵机设到蹲姿 (set_angle) — 触发 IDLE→POSE 自动进入姿态模式
4. `poses.stand()` — POSE_STAND 站姿待命 (Python IK, 固定高度)
5. `bpuppy_ble.start()` — 启动 BLE 广播 (模式由固件编译决定, KittenBlock 模式自动注册 dupterm REPL)

上电自动站姿待命 + **BLE 广播**（KittenBlock 蓝牙编程）。用户程序 (main.py) 从**站姿切入**。**WiFi / 摄像头 / IMU / UART / ADC 均手动或按需启动**:
- WiFi 热点: 手动 `import camera_stream; camera_stream.start()`（上电默认不开, 把 RF 让给蓝牙）
- WiFi 图传: 网页点「图传 开」或 `camera_stream.start(stream=True)`
- IMU: balance / set_heading / calib_mag 的 `start()` 自动 `init()`（`imu_init` 幂等）
- BLE 协议层: KittenBlock 模式走 dupterm REPL（C 层自动）; Hiwonder 模式 `HiwonderBLE()` 构造时启动
- UART / ADC: 手动 `import` + `init()`

> **蓝牙编译互斥**：两个蓝牙模式（KittenBlock Nordic / Hiwonder FFE0）**不要同时编译**，同一固件只能启用其一。由 `drivers/micropython.cmake` 的 `BPUPPY_BLE_KEBLOCK` / `BPUPPY_BLE_HIWONDER` 宏二选一，详见 `kext-bpuppy/KittenBlock扩展开发.md` 第 11 节。

### KittenBlock 平台支持

| 平台 | 方式 | 说明 |
|------|------|------|
| **安卓** | Chrome 打开 `https://kblock.kittenbot.cc/` | Web Bluetooth 原生支持 |
| **iPad** | **Bluefy** 浏览器打开 kblock.kittenbot.cc | ⚠ iPad Safari/Chrome 的 Web Bluetooth 被 Apple 限制，必须用 Bluefy |
| **PC 桌面版** | KittenBlock 桌面版 | 串口 (115200) 或蓝牙（PC 需蓝牙适配器） |
| **iPad KittenBlock App** | 不推荐 | App 无法加载 URL 导入的自定义主板扩展 |

> 蓝牙无线连接走 **Nordic UART + dupterm REPL**（固件内置），KittenBlock 把它当串口用。无线上传 main.py 到 VFS 同样支持。

---

## 步态仿真 (PC 端)

```bash
cd gait_sim
pip install matplotlib numpy
python gait_sim.py                    # 输出 CSV + PNG + GIF
python gait_sim.py --stride 80 --height 70 --fps 4
```


## OV2640 摄像头

### 硬件

| 项目 | 规格 |
|------|------|
| 型号 | OV2640 (200万像素) |
| 接口 | DVP 8-bit 并行 |
| 最大分辨率 | UXGA 1600×1200 |
| 输出格式 | JPEG (硬件编码) / RGB565 / GRAYSCALE |
| XCLK | 20MHz (LCD_CAM 内部分频，不占 LEDC) |
| 引脚 | GPIO 4~18（与舵机无冲突，详见 GPIO 表） |

### 软件接口

```python
import bpuppy_camera

# 默认初始化 (JPEG, 1600×1200, q=10)
bpuppy_camera.init()

# 自定义格式和分辨率
bpuppy_camera.init_adv(bpuppy_camera.QVGA, 0, 2, 20000000, bpuppy_camera.RGB565)

# 拍照 → 返回 (data, width, height, format)
data, w, h, fmt = bpuppy_camera.capture()

# 释放
bpuppy_camera.deinit()
```

| 函数 | 说明 |
|------|------|
| `init()` | 默认: JPEG UXGA, q=10, 双缓冲 |
| `init_adv(fs, q, fb, xclk, fmt)` | 自定义: 分辨率/画质/缓冲数/时钟/格式 |
| `capture()` | 拍照，返回 `(bytes, w, h, fmt)` 或 `None` |
| `deinit()` | 释放摄像头 |
| `is_ready()` | 是否已初始化 |

**格式常量**: `JPEG`(4) / `RGB565`(1) / `GRAYSCALE`(5)
**分辨率常量**: `QQVGA`(160×120) / `QVGA`(320×240) / `VGA`(640×480) / `SVGA`(800×600) / `XGA`(1024×768) / `UXGA`(1600×1200)

### PC 端串口拍照

```powershell
pip install pyserial
python tools/capture.py COM3   # 端口换成实际值 (设备管理器查看)
# 连接后按 Enter 拍照，自动打开图片，q 退出
```

> 原理: PC 通过串口发送命令，ESP32 拍照后 base64 回传，PC 解码保存为 JPEG。

---
---

## 开发注意事项 — 易错点总结

### 1. 膝角约定

**IK `knee_angle`**: 0°=折叠, 180°=伸直。
**舵机**: 左膝 `servo = knee_deg`, 右膝 `servo = 180 - knee_deg`。
两边 servo=90° 时均为直角。修改任何膝角相关代码前必须确认方向。

### 2. `motion_set_params` 的参数语义

- `speed` = 步频 (0~10, 0=停), 纯 magnitude
- `stride` = 步长+方向 (正=前, 零=原地踏步, 负=后)
- `height` = 站立高度 (mm)

三个参数始终直接写入，无哨兵。

### 3. 运动→静止的 `pose_trans`

运动步态切到 `GAIT_STOP` 时走减速停逻辑 (`g_stop_decel` → `g_pending_gait=GAIT_STOP`), 需保证 `g_was_moving` 状态正确, 否则触发 `pose_trans=2` (停步过渡) 导致跳变。

### 4. `servo_init` 初始 duty

`servo_init` 设 `duty=0`, 导致初始化后舵机失能(随机位置)。开机 `init_all` 后必须用 `set_angle` 设定蹲姿再进入姿态模式 (main.py 已处理)。

### 5. 蹲姿角度定义

crouch 姿态角度在 Python `poses.py` 定义 (`CROUCH = [135,45,...]`)。若修改腿结构, 需同步 `poses.py` 的蹲姿角度 (且 `servo_init_all` 非幂等, 用户程序勿重复调用)。

### 6. 校准公式

`cal(ch, ref_deg)` → offset = `ref_deg - 90`。`set_angle(90)` → 发送 `90 + offset` 到舵机。
含义: "舵机要转到 ref_deg° 腿才垂直" → offset 补偿后 set_angle(90) 腿正好垂直。

### 7. GO 自适应中的 eff_speed

GO 的 duty/gap/stride/height 查表使用 `eff_speed` (实际 speed 的绝对值, 经过半周期平滑), 不是 `target_speed`。BLE 写 `target_speed`, 实际 speed 逐步跟随。

### 8. BLE 停止

停止时设 `speed=0, stride=0`。d=0 调 `set_params(0, 0, 70)` + `set_gait("stop")`。

### 9. GPIO 引脚映射 (已确定)

| 舵机 | GPIO | 舵机 | GPIO |
|------|------|------|------|
| LF_HIP 左前大腿 | 1 | RF_HIP 右前大腿 | 40 |
| LF_KNEE 左前小腿 | 42 | RF_KNEE 右前小腿 | 38 |
| LH_HIP 左后大腿 | 2 | RH_HIP 右后大腿 | 39 |
| LH_KNEE 左后小腿 | 41 | RH_KNEE 右后小腿 | 45 |

IMU: I2C0 (V2.0: SDA=GPIO3, SCL=GPIO21, addr=0x68; V3.0: SDA 改 GPIO14)。双芯片自适应: WHO_AM_I 识别 MPU6050(6轴)/MPU9250(9轴), REPL 可用 `bpuppy_imu.get_chip()` / `has_mag()` 查询。
UART2: GPIO19=RX, 20=TX (CI-33T / micro:bit, 手动 init)。
UART1: GPIO4=TX, 5=RX (与摄像头 SCCB SDA/SCL 复用, 手动 init)。
I2C1: GPIO9=SDA, 10=SCL (与摄像头 D1/D3 复用, 手动 init)。
ADC: 电池检测 V2.0 停用 (电池=GPIO14/ADC2, BLE 下不可用); V3.0 改 GPIO3=ADC1_CH2 (分压 51k/10k, 软件 ×6.1)。
完整 GPIO 分配表见 `docs/硬件连接.md`。

### OV2640 摄像头 DVP 引脚 (小智 ESP32-S3 板载)

| 信号 | GPIO | 信号 | GPIO |
|------|------|------|------|
| SIOD (SDA) | 4 | SIOC (SCL) | 5 |
| VSYNC | 6 | HREF | 7 |
| XCLK | 15 | PCLK | 13 |
| Y2 (D0) | 11 | Y6 (D4) | 12 |
| Y3 (D1) | 9 | Y7 (D5) | 18 |
| Y4 (D2) | 8 | Y8 (D6) | 17 |
| Y5 (D3) | 10 | Y9 (D7) | 16 |

> 与舵机 GPIO 无冲突。PWDN/RESET 未接。

### 10. ADC 驱动必须用 legacy API

`bpuppy_adc` 的 C 驱动**只能用 legacy driver** (`adc1_config_width` / `adc1_config_channel_atten` / `adc1_get_raw`, `#include "driver/adc.h"`)。

**绝不能**用 new driver (`adc_oneshot_*`, driver_ng) — MicroPython 的 `machine.ADC` 使用 legacy driver，ESP-IDF 5.x 中两者互斥，混用会触发 `CONFLICT! driver_ng is not allowed to be used with the legacy driver` 断言并**上电无限重启**。

---

## 首次编译问题排查

### `idf.py: command not found`
未激活 ESP-IDF 环境。容器内 `source /opt/esp/idf/export.sh`。

### `mpy.cmake not found` / `micropython-helper not found`
组件未下载。需先 `idf.py set-target esp32s3` → `idf.py reconfigure`，等待组件管理器拉取。

### `sdkconfig` 冲突 / Flash 大小不对
旧 build 缓存: `rm -rf build && idf.py set-target esp32s3 && idf.py build`

### 构建产物名不对
确认 `CMakeLists.txt` 中 `project(micropython_bpuppy)`，产物为 `build/micropython_bpuppy.bin`。

---

## sdkconfig 三层覆盖机制

```
第 1 层: ESP32_GENERIC_S3 板级默认 (8MB Flash, 自动 PSRAM)
第 2 层: SPIRAM_OCT variant (240MHz, Octal PSRAM)
第 3 层: sdkconfig.defaults + sdkconfig.bpuppy (16MB Flash, 16MB 分区表)
```

后加载的覆盖先加载的。查看生效配置: `grep CONFIG_ESPTOOLPY build/sdkconfig`

---

## 修改代码指引

| 需求 | 改哪个文件 |
|------|-----------|
| 修改舵机 GPIO 引脚 | `drivers/servo_driver.c` → `servo_init_all()` |
| 修改步态参数 | `drivers/motion_task.cpp` |
| 修改 IK 腿长/髋距/限位 | 推荐运行 `cal_ik()` / `set_body_dims()` / `set_joint_limits()` → NVS 持久化；改默认值则 `drivers/ik.h` |
| 添加 MicroPython C 函数 | 对应 `drivers/*.c` + 注册到模块表 |
| 修改 Python 启动逻辑 | `frozen/main.py` |
| 修改 BLE 协议 | `frozen/ble_hiwonder.py` |
| 修改摄像头参数/格式 | `drivers/camera_driver.c` → `init_adv()` 或 MicroPython `bpuppy_camera.init_adv()` |
| 修改 PC 拍照工具 | `tools/capture.py` |
| 修改构建参数 | `CMakeLists.txt` + `sdkconfig.defaults` |
| 更新版本号 | `drivers/bpuppy_version.c` → `BP_VERSION` |
| 修改分区表 | `partitions.csv` |

---

## ESP-IDF 常用命令

```bash
idf.py clean              # 清理编译产物
idf.py fullclean           # 清理所有 (含 cmake 缓存)
idf.py menuconfig          # 图形化配置
idf.py size                # 固件各组件大小
idf.py flash monitor       # 烧录并监控
```

---

## 验证清单

- [ ] `idf.py build` 编译成功
- [ ] `build/micropython_bpuppy.bin` 存在
- [ ] 烧录后 USB CDC 串口可连接 (921600, 端口见设备管理器)
- [ ] 启动 banner 显示 "bPuppy Robot Dog - ESP32-S3"
- [ ] 上电自动站立，无跳动
- [ ] `import bpuppy; bpuppy.version()` 返回版本号
- [ ] KittenBlock 模式 (`BPUPPY_BLE_KEBLOCK`): 广播 `bPuppy_XXXX`，KittenBlock 蓝牙可连（安卓/iPad Bluefy/PC）
- [ ] Hiwonder 模式 (`BPUPPY_BLE_HIWONDER`): 广播 `mechdog_XX`，Wonderbot App 可连
- [ ] 蓝牙 REPL：`os.dupterm(None)` 返回 BLE 流对象（C 层自动注册）

---

## 已知问题 / 待解决

### IMU 校准后有固定偏差 —— ✅ 已修复
`calibrate(300)` 原先只补偿 Z 轴（1g），X/Y 零偏未标定 → 水平放置姿态角偏 ~12°。现已补 X/Y 零偏（**校准时机身必须水平**）。修复后水平放置读数应 ≈0。

### 自平衡起始方向依赖 —— 已修偏置, 待验证
`balance.start()` 时，若机身起始方向与启动瞬间不一致，会稳定在肉眼不平的角度。根因是加速度计 X/Y 偏置（上述），偏置修复后应缓解，**待实测验证**。

---

| 组件 | 版本 |
|------|------|
| MicroPython | v1.22.1 |
| ESP-IDF | v5.1.2 |
| 固件版本 | V1.0_2026071001 |
| 芯片 | ESP32-S3 WROOM-1 N16R8 |
