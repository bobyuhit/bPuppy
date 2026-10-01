# bPuppy — ESP32-S3 四足机器狗

## 产品概述

bPuppy 是基于 ESP32-S3 的 8 自由度四足机器狗（4腿 × 2DOF：髋+膝），运行 MicroPython v1.22.1 + ESP-IDF v5.1.2。
底层 C 驱动（舵机、IMU、BLE、IK、步态），上层 Python 应用，支持 KittenBlock 图形化编程（蓝牙 / USB）。

| 项目 | 规格 |
|------|------|
| 主控 | ESP32-S3 WROOM-1 N16R8 (16MB Flash, 8MB Octal PSRAM) |
| 舵机 | 8× 模拟舵机 (每条腿 2DOF: 髋 + 膝) |
| IMU | MPU6050/MPU9250 双芯片自适应 (I2C0 SDA=GPIO14, SCL=GPIO21, addr=0x68; WHO_AM_I 自动识别: 6050=6轴无磁力计, 9250=9轴含 AK8963; Mahony 姿态) |
| 通信 | BLE (NimBLE, 编译互斥: KittenBlock Nordic UART 或 Hiwonder FFE0) + UART2 (GPIO19/20, CI-33T/micro:bit) |
| 控制台 | CH343 USB-UART 桥 → UART0 (GPIO43/44), 115200bps (USB-CDC 已关闭) |
| 供电 | 7.4V 2S LiPo |

**当前固件参数（默认值，均可运行时修改 + NVS 持久化）：**

| 参数 | 默认值 | 运行时修改 |
|------|--------|----------|
| 大腿 L1 | 40mm | `cal_ik(L1, L2)` → NVS |
| 小腿 L2 | 45mm | `cal_ik(L1, L2)` → NVS |
| 前后髋距 | 半距 62.5mm（全长 125mm） | `set_body_dims(bl, bw)` → NVS ⚠ 传**半距** |
| 左右髋宽 | 半宽 59mm（全宽 118mm） | `set_body_dims(bl, bw)` → NVS ⚠ 传**半宽** |
| 膝角范围 | 10°~170° | `set_joint_limits()` → NVS |
| 髋角范围 | 0°~180° | `set_joint_limits()` → NVS |
| 速度范围 | 0~10 | `set_speed()` |
| 抬腿默认 | 30mm | `set_params()` 第 2 参 |
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

板载 CH343 USB-UART 桥接芯片，走 **UART0 (GPIO43/44)**，**不是** ESP32-S3 原生 USB-JTAG。
ESP32-S3 的原生 USB 脚 **GPIO19/20 已被语音模块的 UART2 占用**（`frozen/voice.py`），
所以 MicroPython 的 `usb_init()` 必须注释掉 —— 不关它，USB-OTG PHY 会接管这两个脚，
UART2 发不出波形。**USB-CDC 虚拟串口因此不可用，这不是可以打开的功能**
（详见 [硬件连接.md](PCB/硬件连接.md) 的 GPIO19/20 条目）。端口从设备管理器看，因机器而异。

```powershell
# 日常增量: 只写 app 分区
esptool --chip esp32s3 --port COM14 --baud 921600 write-flash `
  0x10000 build/micropython_bpuppy.bin
```

> **只烧 app 是安全的**，即使板子上还是旧分区表 —— 固件在**运行时**按 label 找分区
> （`frozen/main.py` 的 `Partition.find(1, label='vfs')`），不写死偏移。所以换固件不必换分区表。
>
> 改了 bootloader / 分区表、或首次烧录时才需要**全烧**:

```powershell
# 全烧: 板子从零开始 (或分区表有变) 时用
esptool --chip esp32s3 --port COM14 --baud 921600 write-flash `
  0x0 build/bootloader/bootloader.bin `
  0x8000 build/partition_table/partition-table.bin `
  0x10000 build/micropython_bpuppy.bin
```

> ⚠ **全烧写了 `0x8000`（分区表），vfs 就必须重建一次** —— 位置/大小变了，旧 FAT 失效。
> 做法见下面「Flash 分区布局」的 `erase-region` 步骤。只烧 app 则不需要。

> 成功标志: 三行 `Wrote xxx bytes` + `Hash of data verified`。烧完按 RESET 或重新上电。

### 串口连接

- 波特率: **115200**（实测 REPL 必须用 115200 才能读到干净输出；旧文档误标 921600）
- 端口: 设备管理器查看（板载外部 USB-UART 桥接, 接 UART0 GPIO43/44，非 USB-JTAG CDC）
- 工具: PuTTY / Tera Term / VS Code Serial Monitor

烧录后重启，应看到:
```
============================================
  bPuppy Robot Dog - ESP32-S3 MicroPython
  Build: ...
  WROOM-1 N16R8 | uPy v1.22.1 | IDF v5.1.2
============================================
[bPuppy] 运行 /camera_on.py (后台线程)...
Ready.
>>> poses.crouch() / poses.stand()  # 姿态模式
>>> bpuppy_motion.set_gait('go')    # 运动模式
```

`[bPuppy] 运行 /camera_on.py` 这行**只在板子上存在该文件时**才打（见「上电行为」）。
中间还会夹着 `micropython.mem_info()` 的内存报告和 servo/voice 的告警，属正常。

### 改了哪些文件需要怎么构建

| 改了什么 | 怎么构建 |
|---------|---------|
| `frozen/*.py` | `bash build.sh` → 只烧 app 分区 |
| `drivers/*.c/.cpp` | `bash build.sh` → 只烧 app 分区 |
| `partitions.csv` | `bash build.sh` → **全烧**（增量即会重生成分区表, **不需** `rm -rf build`） |
| `CMakeLists.txt` / `sdkconfig.*` | `rm -rf build && bash build.sh` → **全烧** |

> `partitions.csv` 是 `partition-table.bin` 的**显式 ninja 依赖**（`build/build.ninja` 里
> `build partition_table/partition-table.bin: CUSTOM_COMMAND ... partitions.csv ...`），
> 改它普通增量 `idf.py build` 一定会重生成，不必全量重编。
> 判断有没有生效：构建日志里会打出 `Partition table binary generated. Contents:` 那段表。
> 想二次确认就解析 `build/partition_table/partition-table.bin`（32 字节一项，
> 第 5 项是 magic `0xEBEB` 的 MD5 伪项、不是分区，之后是 0xFF 填充，文件固定 3072 字节）。

### Flash 分区布局 (16MB)

`partitions.csv` 是唯一权威来源：

| 分区 | 类型 | 偏移 | 大小 | 用途 |
|------|------|------|------|------|
| *(bootloader)* | — | `0x0` | 32KB | 二级引导 |
| *(分区表本身)* | — | `0x8000` | 4KB | 3072 字节实体 + 填充 |
| `nvs` | data/nvs | `0x9000` | 24KB | **舵机三点标定 / 几何 / IMU 标定** |
| `phy_init` | data/phy | `0xF000` | 4KB | RF 校准数据 |
| `factory` | app/factory | `0x10000` | `0x7F0000` (7.94MB) | 固件 + frozen `.py`（两者都在这里） |
| `vfs` | data/fat | `0x800000` | 8MB | 用户程序（KittenBlock 下载的 `/main.py`） |

两条改分区前必读：

- **2026-09-16 前是 `factory` 2MB / `vfs` 14MB**。那时固件 1.63MB 已占满应用分区的 84%（只剩 320KB），
  而 vfs 只装一个几 KB 的 `/main.py` —— 纯浪费。现已调成 8MB/8MB，应用剩 6.3MB。
- ⚠ **`nvs` 在 `factory` 之前，所以改 `factory`/`vfs` 的尺寸永远碰不到它** —— 前提是 `factory`
  起始仍是 `0x10000`、且不动 `nvs` 那两行。烧录**绝不要用 `--erase-all` / `erase-flash`**：
  那是唯一会毁掉舵机标定的操作。也**别把「升级 ESP-IDF」和「改分区表」放在同一次操作里** ——
  `mpy_startup.c` 里 `nvs_flash_init()` 的失败分支会调 `nvs_flash_erase()`，
  NVS 格式版本一变就静默清空全部标定。

**改过分区表后 vfs 必须重建一次**（位置/大小变了，旧 FAT 失效）。`frozen/main.py` 只在首扇区
全 `0xFF`（处女分区）时才自动格式化，所以要先把新 vfs 的首扇区擦掉让它自愈：

```powershell
esptool --chip esp32s3 --port COM3 erase-region 0x800000 0x1000
```

复位后应看到 `[bPuppy] VFS 是未格式化的新分区, 正在格式化...`，然后正常挂载。
若看到 `[bPuppy] [ERROR] VFS 挂载失败 ... 已拒绝自动格式化`，就是漏了这一步 —— 补擦再复位即可。
（只擦 1 个扇区就够：guard 只读首扇区，`mkfs` 不要求 flash 是擦除态。）

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
MicroPython 层:   frozen/main.py → 上电自动站立 + BLE 广播 + 电池指示灯 + 语音 (WiFi/摄像头/IMU 手动或按需启动)
                       ↑ import
C Extension API:  bpuppy_servo / bpuppy_imu / bpuppy_uart / bpuppy_adc /
                  bpuppy_ik / bpuppy_motion / bpuppy_camera / bpuppy_ble / bpuppy_led
                       ↑ MP_REGISTER_MODULE
C 驱动层:
  servo_driver.c    — LEDC PWM 8路舵机 (S3 统一 LS mode) + NVS 校准
  imu_driver.c      — I2C MPU6050/MPU9250 双芯片自适应 (WHO_AM_I 识别, 6050=6轴无磁力计, 9250=9轴 Mahony + 磁力计椭球校准)
  uart_driver.c     — UART2 通信口 + UART1 摄像头复用口 (I2C1 无固件模块, 用原生 machine.I2C)
  adc_driver.c      — ADC 电池检测 (电池=GPIO3/ADC1_CH2, 分压 51k/10k)
  led_driver.c      — WS2812 电池指示灯 (GPIO48, RMT chan 0, adc_init 后自动激活)
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
| `drivers/adc_driver.c` | ADC 电池检测 (GPIO3=ADC1_CH2, 分压 51k/10k) |
| `drivers/led_driver.c` | WS2812 电池指示灯 (GPIO48, `bpuppy_adc.init()` 自动激活; 蓝=满电/红=低压/闪烁=危险)。**标定唯一实现** — 系数存 NVS, `bpuppy_led.batt_v()` 读电压, `bpuppy_led.batt_pct()` 读电量 0-100 (7.4V=100%/6.6V=0%, 与 LED 分界点同源; 读不到返回 0), `set_cal/reset_cal` 改标定 |
| `drivers/ble_driver.c` | NimBLE GATT 服务 — 编译互斥 (KittenBlock Nordic / Hiwonder FFE0) |
| `drivers/ble_stream.c` | BLE 流对象 — dupterm REPL 桥接 (KittenBlock 蓝牙) |
| `drivers/micropython.cmake` | `BPUPPY_BLE_KEBLOCK` / `BPUPPY_BLE_HIWONDER` 编译宏 |
| `components/mr9you__micropython-helper` | MicroPython 移植层 (mphalport.c 补 dupterm 输入) |
| `kext-bpuppy/` | KittenBlock 硬件扩展 (39 积木 + 蓝牙配置 + 开发文档) |
| `frozen/main.py` | 启动脚本 — 原厂初始化 → 站姿待命 (POSESTAND), 用户程序在**后台线程**里 exec (不阻塞 REPL) |
| `frozen/balance.py` | 站立自平衡 — 增量式 PID, 50Hz 闭环 (绕过 motion task) |
| `frozen/camera_stream.py` | WiFi 热点 MJPEG 图传 + 网页遥控器 |
| `frozen/voice.py` | 语音「事件」转发核心 — UART2 收发 + 后台线程 + 事件注册/分发（无内置动作，见下方「语音事件系统」节） |
| `frozen/camera_serial.py` | 串口拍照回传 — 通过 REPL 触发拍照，base64 回传 PC |
| `frozen/heading_anchor.py` | **航向锁定 step1** — IMU 航向的 err 源。对外 `on(偏移度)` / `off()` / `read()` / `set(**cfg)`，`anc.cfg` 看参数。**自己 import step2 并把闭环接到后台线程**（用户看不见 `heading_follow`）。KittenBlock 的「航向锁定 偏转 / 解除航向锁定」两个积木靠它 |
| `frozen/heading_follow.py` | **航向锁定 step2** — 闭环执行体 `err_fn → set_turn`。`run`(阻塞) / `start`(后台线程) / `stop` / `running` / `g_c`。**不 import 任何 err 源**，所以视觉 / 声源都能喂进来。`TURN_SIGN` 在这里（改它不用重编译） |
| `drivers/camera_driver.c` | OV2640 DVP 驱动 + MicroPython 绑定 (`bpuppy_camera`) |
| `tools/capture.py` | PC 端拍照工具 — 通过串口命令拍照并自动保存/预览 |
| `gait_sim/gait_sim.py` | PC 端步态仿真 — CSV/PNG/GIF |
| `docs/操作指南.md` | 日常操作手册 |

---

## 步态算法概要 (motion_task.cpp)

### GO 自适应

| speed | duty | gap | stride | height | lift | pitch | 实际 |
|-------|------|-----|--------|--------|------|-------|------|
| ≤4 | 0.20 | 0.04 | 70 | 70 | 30 | 0° | walk |
| 4~6 | 插值 | 插值 | 70→50 | 70 | 30→5 | 0° | 混合 |
| ≥6 | 0.40 | 0.10 | 50 | 70 | 5 | 0° | trot |

★ **GO 下用户设的 stride / lift / height 三个值不参与运动** —— 上表几项都由 speed 决定
(`motion_task.cpp` 的 GO 分支里 `eff_stride`/`eff_height`/`eff_lift` 被直接覆盖)。
三个值**会被接受并保存**, 之后切到 `walk` / `trot` 立刻按它们走。
**方向不受 GO 影响** —— 它是独立参数 (`motion_set_direction(±1)`), 三种步态一视同仁,
不再靠 stride 的正负号决定前后。

抬脚的两个端点值在 `motion_task.cpp` 顶部: `GO_LIFT_LOW 30.0f` (speed≤4) 和
`GO_LIFT_HIGH 5.0f` (speed≥6), 4~6 之间线性过渡。低速端 30 与 `LIFT_DEFAULT` /
KittenBlock `_lift` 初值一致; 高速端 5 是**几乎贴地**的走法 —— 小跑步长收到 50、
duty 到 0.40, 抬脚压低换更小的上下起伏。**抬脚 5mm ⇒ 摆动腿离地只有 5mm**,
若实测发现刮地/异响/堵转, 先怀疑这里。

实际 speed 经半周期平滑跟随 `target_speed`。

### 姿态过渡

- **静态↔静态** (GAIT_STOP ↔ 运动步态): 每帧限速 3° smoothstep, 自然平滑
- **变速**: 每半周期 ±3.0 步进跟随 target_speed, 避免突变
- **姿态模式 (crouch/sit/play/wave/stand)**: 由 Python `poses` 模块 `_move_to` 限速逼近, 与 C 层机制一致

### 运动参数改变时的过渡

在**运动中**改这五个参数, 目标值立刻写进固件, 但喂给腿循环的**实际值每帧只走固定一步**:

| 参数 | 每帧最多走 | 典型变化 → 到位 | 换算成角度 |
|---|---|---|---|
| 站立高度 | 2.0 mm | 30mm → 0.3s | ≈2.1°/帧 |
| 抬脚高度 | 2.0 mm | 25mm → 0.25s | ≈2.2°/帧 |
| 俯仰 / 横滚 | 0.7° | 10° → 0.3s | ≈2.3° / 0.9° |
| 重心偏移 | 1.5 mm | 20mm → 0.28s | ≈3.2°/帧 |

**手感跟站着改同量级** —— 站着改另有"每帧最多 3°"的限制器兜着, 现在运动中改也差不多。

> ⚠ **限的是"参数挪的速度", 不是"关节转速"。** 走路该多快还是多快 (摆动腿正常就 15~23°/帧) ——
> 全局限 3°/帧会把走路本身废掉。
>
> ⚠ **静止时不限速。** 站着 / 回正步时实际值直接对齐目标 —— 那时本来就有 3°/帧 的限制器, 不需要再来一层。
>
> **步长不在此列**: 支撑相锚定让它从"足端位置"变成了"身体快慢", 改了脚不会挪, 本来就没有跳变。
> **转弯率同理** (它只是左右两侧步长的缩放比例)。

---

### GO 起步 —— 站起来 + 步长淡入

起步是**两段式**, 不是"步长一档档爬":

**阶段 A — 站起来** (`g_stand_up`)
从**实际舵机角**起步 (不管当前是站着 / 坐着 / 蹲着 / 被 Python 摆过), 每帧限速 3°
向站姿 `(x=0, eff_height)` 过渡, 一直跑到 8 个关节全部进入容差。
期间相位**钉在"四腿全踩地"的中点**, 速度也不跟随 —— 否则腿会在舵机还没到位时就开始迈。

**阶段 B — 步长淡入**
站好之后, 步长系数从 0 **线性**淡到 1, 走 `FADE_IN_TURNS_WALK` = 1.0 个步态周期
(trot 段用 `FADE_IN_TURNS_TROT`, 也是 1.0)。
自变量是**走过的步态周期数** (`frame_dphi`) 而不是时间 ⇒ 与速度解耦, 拖快拖慢都是
同样几个周期走完。逐帧插值而不是每半周期跳一档 —— 跳一档 = 已踩地的脚被当场重下位置,
一档 Δstride/2 最坏 7mm ≈ 髋 8.1°, 逐帧只有 0.36mm (speed 4)。

同时:
- **kick**: 静态→GO 的瞬间把速度给到 `min(目标, 2.5)`, 打破相位死锁 (speed=0 时相位不动)
- **速度爬升**: 之后每半周期最多变 ±3.0 (`SPEED_FOLLOW_STEP`) 跟随目标

> 阶段 A 的目标高度取 `eff_height` (这条步态**将要用的**高度), 不是用户设的 `height` ——
> GO 的 `eff_height` 恒为 70, 若按 `height` 站起来, 站高设成 50 时交接那一帧要跳 20mm
> (髋 21.8° / 膝 39.0°)。按 `eff_height` 起 ⇒ 交接点与阶段 B 首帧**逐位相等**。

### GO 停步 —— 回正步

1. **等窗口**: 收停信号 → **保持当前步态继续走**, 等下一个"四腿全踩地"窗口
2. **硬切步长**: 在那个窗口把步长直接切到 0 (全踩地时切, 不会甩腿)
3. **回正步**: 对角两两抬脚 (抬 5mm), 把四条腿从当前 x 挪回 0
4. **站好**: 回正结束 → 切静态步态

> 为什么不用"步长淡出到 0": 淡出要等 1.25 个周期 (得让每条腿都在步长归零后重新落地
> 一次), 而回正步直接把脚**抬起来挪**, 不需要等。省掉的那 1.25 周期正是它快的地方。

### GO 换向 (前进↔后退) —— 对齐点翻转

1. **挂起**: 检测到方向变化 → 记下意图, **保持原方向继续走**
2. **等对齐点**: 等到"四脚全踩地 **且相位对齐**"的那一帧
3. **当帧翻转**: 把四个锚定量 (`g_gait_b` / `g_leg_b_land` / `g_leg_x_land` / `g_leg_x_lift`)
   全部取反 —— 与"方向翻转导致的 `out_x = -out_x`"精确抵消, **位置零跳变**
4. **身体照常走**: 步长/相位/速度一个都不动, 所以也没有"起步"要重跑

**为什么必须等到对齐点**: 翻转同时会把相位表的标签换掉 (LF↔LH / RF↔RH),
每条腿的支撑相进度跟着从 `t_old` 变成 `t_new`。若某条腿正好在这一帧跨过
"摆动↔支撑"边界, 落地锁存会把它的 `x_land` 写死成 `+stride/2` —— 那个锁存**不管方向**,
而翻转刚把 `x_land` 取成负数, 两者一撞位置就跳:

   跳变量 = stride × (t_old + t_new − 1)

离线逐帧实测 (go speed 2.5, stride 70): 好相位 2.2mm, 坏相位 **48.0mm**, 与公式吻合到 0.1mm。

跳变只在 `t_old + t_new = 1` 时为 0, 而这个方程在整个相位空间里**只有两个解**:
`duty + gap/2` 和 `duty + gap/2 + 0.5` —— 也就是全踩地窗口**正中间**那两个点。
四条腿此刻的姿势, 恰好就是新方向步态走到这里时本该有的姿势, 接上去严丝合缝。

> 代价: 不在对齐点上就等下一个, 最坏多等 **0.46 个步态周期**
> (speed 2.5 约 0.58s, speed 1 约 1.45s)。**等待期间照原方向继续走, 不停不减速。**
> `trot` 族 (duty+gap = 0.50) 只有两个全踩地窗口, 且本来就落在解上 ⇒ 等于不用等。

### GO 暂停恢复 (速度滑块 0→N)

滑块拖到 0 ⇒ 目标速度归零 ⇒ 步长按**平方曲线淡出** (`FADE_OUT_TURNS` = 1.25 个步态周期),
淡完切静态步态; 滑块拖回 >0 ⇒ 重新起步 (kick + 淡入)。

> ⚠ 淡出期间**速度被锁住** (`target_eff` 设成当前值, 增量恒为 0):
> 系数的自变量是"走了多少周期", 而周期 ∝ speed —— 速度真的归零则周期不走,
> 系数永远淡不完 (死锁)。这条路恰好会把速度降到 0, 所以必须锁。
>
> ⚠ **三条"停下来"的路只有这一条走淡出。** 「停步」(`set_gait("stop")`) 走**回正步**,
> 「换向」走**对齐点翻转**, 两条都不经过淡出 —— 它们的收尾方式见上面各自的章节。

### 网页方向键

未拖滑块时用系统 target_speed (默认 2.5), 不覆盖预设。
方向键 URL: /cmd?dir=±1&stride=70&turn=X, 经 `_effective_speed()` 获取实际速度。
(`stride` 只带幅度, 前后由独立的 `dir` 决定; 左右两个原地转键仍是 `stride=0`。)

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

`frozen/main.py` 的实际执行顺序:

1. 挂载 VFS (`vfs` 分区, 用户程序所在)
2. `bpuppy_ble.start()` — 启动 BLE 广播 (模式由固件编译决定, KittenBlock 模式自动注册 dupterm REPL)
3. `servo_init_all()` + `load_cal()` — 初始化 8 路 LEDC + 从 NVS 加载校准值 (保持 IDLE)
4. `import bpuppy_motion` / `poses` — 加载运动与姿态模块
5. `sleep(0.5)` — 初始化稳定
6. 舵机设到蹲姿 (set_angle) — 触发 IDLE→POSE 自动进入姿态模式
7. `poses.stand()` — POSE_STAND 站姿待命 (Python IK, 固定高度)
8. `import voltage` — 电池电压检测 + WS2812 指示灯 (import 即启动)
9. `import voice` — 语音模块 (UART2/9600, import 即启动) + `set_main_globals(globals())`
10. 开关文件存在 → 执行 (见下)
    - `/camera_on.py` — 上电自动开"网页+摄像头" (**后台线程**, 起服务器慢, 不能拖住开机)
    - `/pwm_ext_on.py` — 上电自动启扩展舵机 (**同步**, 必须早于 `/main.py`)
11. `/main.py` 存在 → 后台线程执行 (KittenBlock 下载的用户程序)

> **开关文件分两类跑**：`/pwm_ext_on.py` 必须**同步**，因为它要在用户程序之前把管脚抢好。
> `_thread.start_new_thread` 立即返回，后台线程和 `/main.py` 是**并发**的 —— 用户程序若在
> 头几行就 `poses.set_servo(9, ...)`，可能撞上 `pwm_ext` 还没 `on(2)` 而报"未启用"，
> 一个只在快慢上碰运气的假报错。它只有几行、无阻塞操作，同步跑代价可忽略。

**上电自动**: 站姿待命 + BLE 广播 + 电池指示灯 (ADC) + 语音 (UART2)。用户程序 (main.py) 从**站姿切入**。
**手动或按需启动**的只有 WiFi / 摄像头 / IMU:
- WiFi 热点: 手动 `import camera_stream; camera_stream.start()`（上电默认不开, 把 RF 让给蓝牙）
- WiFi 图传: 网页点「图传 开」或 `camera_stream.start(stream=True)`
- **想上电就自动开**: 把上面两行写进板子的 `/camera_on.py`（仓库源文件 `mpy_modules/camera_on.py`）。
  **文件在 = 开，从板子删掉 = 不开**，不用重编译固件，也不影响 KittenBlock 下载的 `/main.py`。
  ⚠ 该文件必须**纯 ASCII**（蓝牙上传会丢非 ASCII 字节，中文注释会截断文件）。
  实现: `frozen/main.py` 每次开机读 `/camera_on.py` 并丢进后台线程执行。用法见 [操作指南.md](操作指南.md) 2.2。
- IMU: balance / set_heading / calib_mag 的 `start()` 自动 `init()`（`imu_init` 幂等）
- BLE 协议层: KittenBlock 模式走 dupterm REPL（C 层自动）; Hiwonder 模式原由 `ble_hiwonder.py` 驱动，**该文件已删除**，故当前只有 KittenBlock 模式可用

> **蓝牙编译互斥**：两个蓝牙模式（KittenBlock Nordic / Hiwonder FFE0）**不要同时编译**，同一固件只能启用其一。由 `drivers/micropython.cmake` 的 `BPUPPY_BLE_KEBLOCK` / `BPUPPY_BLE_HIWONDER` 宏二选一，详见 `kext-bpuppy/KittenBlock扩展开发.md` 第 11 节。

### KittenBlock 平台支持

| 平台 | 方式 | 说明 |
|------|------|------|
| **安卓** | Chrome 打开 `https://kblock.kittenbot.cc/` | Web Bluetooth 原生支持 |
| **iPad** | **Bluefy** 浏览器打开 kblock.kittenbot.cc | ⚠ iPad Safari/Chrome 的 Web Bluetooth 被 Apple 限制，必须用 Bluefy |
| **PC 桌面版** | KittenBlock 桌面版 | 串口 (115200) 或蓝牙（PC 需蓝牙适配器） |
| **iPad KittenBlock App** | 不推荐 | App 无法加载 URL 导入的自定义主板扩展 |

> 蓝牙无线连接走 **Nordic UART + dupterm REPL**（固件内置），KittenBlock 把它当串口用。无线上传 main.py 到 VFS 同样支持。

> ⚠ **指令发送不全的修复**（2026-08 实测确认）：KittenBlock 的 JS 库自身按 20 字节硬编码分包 + 设备侧逐字符 echo 通知洪峰饿死 NimBLE mbuf 池，导致长命令第二包被丢。修复为 `ble_stream.c` **echo 批量打包**（攒一行/超时 30ms 发一个通知）。详见 [硬件连接.md 蓝牙节](PCB/硬件连接.md)。

---

## 语音「事件」系统（CI-33T）— 给接手者的全链路交接

> 本文回答一个问题：**"事件"响应型的语音模块是怎么做出来的，改起来要动哪些文件？**
> 面向后续接手的 AI / 开发者，自包含、可照着改。2026-08-19 实测定稿。

### 0. 一句话架构

```
CI-33T 语音模块 ──UART2──▶ frozen/voice.py（纯事件转发，不做动作）
                                      │ 按命令码触发
                                      ▼
                用户程序 def voiceWhenX()（KittenBlock 事件积木生成）
                                      │
                                      ▼
                bpuppy_motion / poses（用户积木里的动作）
```

**设计铁律（2026-08-19 起，2026-09-27 修订）**：固件收到语音指令**默认只转发事件信号，不做任何动作**。要不要动、怎么动，完全由 KittenBlock 用户程序决定。这条铁律让"语音功能"和"机器狗动作"彻底解耦——换动作只改积木，不动固件。

> ⭐ **唯一例外：`0x3F` 播报电压**。收到它时固件内置播报电量百分比（`_say_batt_pct`：读
> `bpuppy_led.batt_pct()` → `say_num()` 发 `AA 55 72 <pct> 55 AA`）。
> 破例理由：这是"把固件自己采到的电池数据念出来"，属于固件自身状态的播报，不是运动/姿态动作；
> 而电压的唯一真源在 C 层 `led_driver.c`，用户程序侧抄不到那份标定。
> **后果（已接受）**：因为是**无条件**执行，用户程序里若也写「当收到 [播报电压] 指令」积木，
> 固件播报与用户动作**会同时触发**（听感上"报两次"）。
> 平台侧前提：CI-33T 的【串口输入】要配 `AA 55 72 <数据> 55 AA` 的 0–100 词条。

### 1. 三层各自管什么

| 层 | 文件 | 职责 |
|----|------|------|
| 硬件/协议 | 接线 + CI-33T 平台配置 | 语音词 ↔ 串口字节的转换 |
| 固件 | `frozen/voice.py`（frozen 模块） | UART2 收发 + 后台线程扫描 + **事件注册与分发**（默认无内置动作，`0x3F` 除外） |
| 用户 | KittenBlock 扩展 + 用户程序 | `voiceWhenX` 事件函数 → 动作积木 |

#### 1.1 硬件层（接线 + 协议）

- **接线**（2026-08-19 起引脚反转）：`CI-33T PA2(TX)→GPIO20(UART2 RX)`、`PA3(RX)←GPIO19(UART2 TX)`、**9600 波特率**、5V 外部供电共地。
- ⚠ **GPIO19/20 是 ESP32-S3 原生 USB_D-/USB_D+，且已归语音模块的 UART2 用**（`frozen/voice.py` 的 `UART_TX=19` / `UART_RX=20`）。MicroPython 默认启用 TinyUSB 会接管它们 → UART2 发不出，所以已在 `components/mr9you__micropython-helper/mpy_startup.c` 注释掉 `usb_init()`，把两个脚让给 UART2。**USB-CDC 虚拟串口因此不可用，这不是可以打开的功能** —— 想恢复 USB 串口就得放弃 UART2（REPL/烧录走 UART0=COM14 不受影响）。
- **下行**（CI-33T→ESP32）= **4 字节定长帧，两种**（2026-09-26 改，此前是裸 2 字节 `<CMD> <PARAM>`）：
  - **命令帧** `BB <CMD> <PARAM> EE`（例：`BB 31 00 EE` = 前进）。CMD 表见下，PARAM 预留。
  - **角度帧** `<角度> 00 00 00` = **声源角度（DOA）**，0–180 度。⚠ 模块侧格式固定，本仓库不能改。
  - 判据只有首字节：`0xBB`(187)=命令帧；`≤0xB4`(180)=角度帧。**两段不重叠**，不可能互判。
- **上行**（ESP32→CI-33T，发声/反馈）= 帧 `AA 55 <CMD> <PARAM> 55 AA`（例：`AA 55 70 01 55 AA` = 汪汪，`AA 55 70 02 55 AA` = 嘤嘤）。
  ⭐ **狗叫类第一字节统一 `0x70`，靠第 4 个数字选声音**（见 §3.2）。
  另有 `AA 55 72 <数字> 55 AA` = **播报数字**（数字 0–100，见 §3.2.2）。
- 命令码表（下行命令帧的 CMD 字节，0x30–0x3F；`0x3D`/`0x3E` 暂未使用）：

| CMD | 语音 | CMD | 语音 |
|-----|------|-----|------|
| 0x30 | 停止 | 0x38 | 站立 |
| 0x31 | 前进 | 0x39 | 蹲下 |
| 0x32 | 后退 | 0x3A | 坐下 |
| 0x33 | 左转 | 0x3B | 摇手 |
| 0x34 | 右转 | 0x3C | 邀玩 |
| 0x35 | 加速 | 0x3D | （未用） |
| 0x36 | 减速 | 0x3E | （未用） |
| 0x37 | 点头 | 0x3F | 播报电压 ★固件内置动作 |

- CI-33T 平台（智能公元）配置：每个命令词配【串口发送】输出**命令帧** `BB <CMD> 00 EE`；**角度输出保持原样 `<角度> 00 00 00` 不动**；要狗发声时配【串口输入】词条匹配 `AA 55 <数据> 55 AA` 帧触发音效。波特率两边都 9600。
- > ⚠ **模块侧与固件必须同为 4 字节命令帧**：新固件认不出旧的裸 2 字节命令（拼不成帧，当残帧留着），14 条命令词会全哑。反向兼容（平台先改、固件还是旧的）没问题 —— 旧固件逐字节扫到帧内 CMD 照样派发。

#### 1.2 固件层 — `frozen/voice.py`（事件转发核心）

启动即 `import voice` → 模块底部 `start()` → `bpuppy_uart.init(2,19,20,9600)` + 起后台线程 `_pump()`。

`_pump()` 每 20ms：
1. `_scan_events()` — 扫描主全局 dict，把新出现的 `voiceWhenX` 函数注册为回调（**只注册不调用**）。
2. `bpuppy_uart.any()` → `read(64)` → `print("VOICE RX: <hex>")` → 追加到帧缓冲 `_buf`。
3. `_parse()` — **按结构切帧**（2026-09-26 改，原先是逐字节扫 0x30–0x3C）：
   命令帧 `BB CMD PARAM EE` 且 CMD 在 0x30–0x3F 内 → `_dispatch(CMD, PARAM)`；
   角度帧 `<角度> 00 00 00`（首字节 ≤ 0xB4）→ `_dispatch(CMD_SOUND_DIR, 角度)`；
   都不是 → 丢 1 字节重同步。**收不够 4 字节就 `while` 退出、留到下一轮**，所以切分/粘连都无害。
4. `_dispatch(cmd, param=0)` — 若是 `CMD_SOUND_DIR`，**先写 `SoundAngle = param` 再触发事件**
   （顺序要紧：用户函数体里读 `voice.SoundAngle` 得读到本次值；没有事件积木时变量也照写）；
   然后查 `_handlers[cmd]` 逐个调用用户函数（`print("VOICE CMD: 0x%02x -> event")`）。无回调则什么都不做。
   回调签名保持 `fn()` —— 角度靠变量带出去，不给既有回调加参数。

对外接口：`voice.on_cmd(cmd, fn)` / `off_cmd` / `play('汪汪'|'嘤嘤')` / `say(cat, code)` / `start` / `stop` / 变量 `SoundAngle`。

> **为什么改掉逐字节扫描**：角度帧第一字节就是角度值，逐字节扫命令码段时 **角度 48–63 度会被误当运动指令**
> （53° → `0x35` = 加速，48–63 度覆盖整个 `0x30`–`0x3F` 段）。现在靠首字节取值范围（命令 0xBB vs 角度 ≤0xB4）区分，
> 不重叠 → 不可能误判，且不依赖时序/长度/延时。

#### 1.3 用户层 — KittenBlock 扩展

- 1 个语音事件积木（hat，`kblock.json5` `## $$cat_voice` 组）：`pycode: ['def voiceWhen[VOICE]()']`，下拉选指令（`type:'value'` 参数**裸代入**函数名，KittenBlock 不加引号）。下拉 value 必须与 `_EVT_FUNCS` 的 15 个后缀完全一致（14 条指令 + `SoundDir`）。**加下拉项不用新增积木** —— `$$voiceCmdSoundDir` 那一项就复用了同一个 hat。
- 1 个变量积木（reporter）：`getSoundAngle` → `pycode: 'voice.SoundAngle'`（读最近一次声源角度，度，0–180；初值 `-1` = 还没收到过）。
- KittenBlock 离线代码生成：hat 积木把 `def voiceWhen<指令>():` 放**生成文件开头（正文之前）**，用户积木体做函数体。**没有任何代码调用它**——注册全靠固件 `_scan_events()` 按函数名找到它。
- 1 个发声积木（`voiceSound`，积木文字「狗叫 [SOUND]」）：下拉 `soundMenu`（值 `WANG`/`YING`）**裸代入**属性名，
  `pycode: 'voice.say(*voice.SND_[SOUND])'`（2026-09-27 由「语音播放汪汪」/「语音播放嘤嘤」两个积木合并成一个下拉）。
  ⚠ **`pycode` 必须纯 ASCII** —— KittenBlock 生成代码时会抹掉非 ASCII 字符，写成
  `voice.play('汪汪')` 会变成 `voice.play('')`，点哪个声音都汪汪。用 `*voice.SND_*`
  取值而非写死 hex，是为了保住"换声音只改固件常量和下拉一行、积木不用动"这条性质。
- 1 个播报数字积木（`voiceSayNum`，「播报数字 [NUM]」）：滑块 0–100，`pycode: 'voice.say_num([NUM])'`
  → 发 `AA 55 72 <数字> 55 AA`。范围与钳位见 §3.2.2。
- ⭐ **没有"播报电量"积木**（2026-09-27 决定不做）：说「播报电压」由**固件内置**自动播报电量百分比
  （`voice._say_batt_pct`），不需要 KittenBlock 参与 —— 见上文铁律的唯一例外。

### 2. 事件注册机制（核心难点，含坑）

#### 2.1 完整数据流（"前进"为例）

```
用户说"前进"
 → CI-33T 识别 → 串口发命令帧 BB 31 00 EE
 → GPIO20 (UART2 RX) → bpuppy_uart 缓冲
 → _pump() 轮询到 → print "VOICE RX: bb3100ee" → 追加到 _buf
 → _parse() 切出命令帧 → _dispatch(0x31, 0x00)
 → print "VOICE CMD: 0x31 -> event"
 → 调用户 def voiceWhenFwd()  ← 已由 _scan_events 注册
 → 函数体（积木翻译的动作）→ bpuppy_motion.set_gait('go') 等
```

声源角度走同一条链路，只是不触发 `voiceWhen*` 那一层：

```
拍手（声源在 120° 方向）
 → CI-33T DOA 算法算出角度 → 串口发角度帧 78 00 00 00
 → _pump() → print "VOICE RX: 78000000" → _parse() 切出角度帧
 → _dispatch(CMD_SOUND_DIR, 120)
 → voice.SoundAngle = 120          ← 先写变量（没事件积木也照写）
 → 调用户 def voiceWhenSoundDir()  ← 有就触发
```

#### 2.2 关键机制：函数在哪、怎么被找到

事件函数 `def voiceWhenFwd()` 定义在**主脚本全局作用域**（frozen main.py 在后台线程里 `exec(/main.py, globals())` 用的那个 dict，或 REPL 的 `globals()`）。`_scan_events()` 按 `_EVT_FUNCS` 表（函数名→命令码）在**主全局 dict** 里找函数、注册为回调。

#### 2.3 ⚠ 最大的坑：`sys.modules['__main__']` 是 `None`

这个 MicroPython 固件里 `sys.modules.get('__main__')` 返回 **None**（实测），早期实现靠它找事件函数 → 永远找不到 → 语音指令收到但狗不动（`event` 打印缺失）。

**解决方案（已在代码里）**：`voice.set_main_globals(globals())` 显式传入主全局 dict。`_scan_events()` 的 dict 来源优先级：`set_main_globals()` 传入的 > `sys.modules['__main__']`。

传入的三个路径（**新接手的 AI 加路径时别漏**）：
1. `frozen/main.py`（L132）—— 开机/物理 RESET 路径。在 `import voice` 之后、把用户程序交给后台线程之前调用；线程里 `exec(...)` 新定义的函数会进入同一个 dict（`py/modthread.c` 把创建者的 globals 传给新线程）。
2. `kext-bpuppy/extension.json` 的 `afterConnect`（L33）—— KittenBlock 在线连接/软复位路径。
3. `kext-bpuppy/kblock.json5` 的 `libs."*".import`（L3 末尾）—— KittenBlock 代码生成注入路径。

**凡是改了 `frozen/voice.py` 或 `frozen/main.py`，必须重编译固件 + 烧录**（它们打进固件，不是传 `/main.py`）。只改扩展侧只需重新打包 zip + 推送。

#### 2.4 `_scan_events` 幂等扫描

每 20ms 扫描一次；已注册的不重复注册；只注册不调用 → **用户函数体不会在开机时执行一次**。开机日志里出现 `voice: event 0x31 -> voiceWhenFwd` 即注册成功。

### 3. 怎么改（Recipe）

#### 3.1 加一个全新语音指令（例："转圈"）

动 4 处，缺一不可：

1. **CI-33T 平台**（智能公元，用户侧）：配命令词"转圈" → 串口发送**命令帧** `BB 40 00 EE`（帧头帧尾别漏）。
2. **固件** `frozen/voice.py`：
   - 加常量（可选，注释更清晰）`CMD_SPIN = 0x40`；
   - `_CMD_MAX` 从 `0x3F` 扩到 `0x40`（命令帧 CMD 字节的合法性校验，**漏了这条指令会被当成残帧丢弃**）。
     ⚠ 现有的 `0x3D`/`0x3E` 已经落在合法区间内 —— 想省事可以直接用这两个码，不用动 `_CMD_MAX`；
   - `_EVT_FUNCS` 加一行 `'voiceWhenSpin': CMD_SPIN`。
   - 重编译固件 + 烧录。
3. **扩展** `kext-bpuppy/kblock.json5`：**不再新增积木**，只在 `menus.voiceMenu` 加一项 `{ text: '$$voiceCmdSpin', value: 'Spin' }`。
   ⚠ **value 必须和函数名后缀一致**（`Spin` → `def voiceWhenSpin()`），`_EVT_FUNCS` 才能映射到命令码。
4. **本地化** `kext-bpuppy/bpuppy.l10n.json`：加 `voiceCmdSpin` 的显示文本（中文"转圈"）。
   **重新打包 zip**（extension.json + kblock.json5 + bpuppy.png + bpuppy.l10n.json 4 个文件）→ 推送 GitHub（raw 链接 `https://raw.githubusercontent.com/bobyuhit/bPuppy/master/bpuppy-kittenblock.zip`）→ KittenBlock 里清本地扩展重新导入。

#### 3.2 换/加声音映射

改 `frozen/voice.py` 的 `SND_WANG` / `SND_YING`（= 上行帧里**第一个数据字节**和**第 4 个数字**）。

⭐ **狗叫类第一字节统一 `0x70`，靠第 4 个数字区分具体声音**：`(0x70, 0x01)` = 汪汪、`(0x70, 0x02)` = 嘤嘤
（2026-09-27 用户实测：嘤嘤从 `(0x71, 0x02)` 改为 `(0x70, 0x02)`，两个声音归到同一类）。

加新声音 = 固件加一个 `SND_xxx = (0x70, n)` + `kblock.json5` 的 `soundMenu` 加一行 + `bpuppy.l10n.json` 加三语文案；
`voiceSound` 积木的 `pycode` 不用动。改完重编译烧录 + 重新打包扩展。

#### 3.2.1 加一个"读值型"积木（reporter，例：声源角度）

1. **固件** `frozen/voice.py`：把值做成**模块级变量**（如 `SoundAngle`），在 `_dispatch` 里更新它。
   重编译固件 + 烧录。
2. **扩展** `kext-bpuppy/kblock.json5`：加 reporter 积木，`pycode` 直接读变量：
   ```js
   { opcode: 'getSoundAngle', blockType: 'reporter', text: '$$getSoundAngle', pycode: 'voice.SoundAngle' }
   ```
3. **本地化** `bpuppy.l10n.json`：加 `getSoundAngle` 的三语文案。重新打包 zip + 推送。

> 与事件积木的分工：**事件**（hat）用于"一发生就响应"，**变量**（reporter）用于"随时读最新值"。
> 声源角度两者都有，因为模块会连发角度帧 —— 要"持续跟随"就轮询变量，避免事件被连发打爆。

#### 3.2.2 播报数字（上行 `0x72`）

「播报数字 [NUM]」积木 → `voice.say_num(NUM)` → 帧 `AA 55 72 <数字> 55 AA`（数字就是第 4 个字节）。

- 范围 **0–100**：积木用 `slider`（`min: 0` / `max: 100`），固件 `say_num()` 里再钳一道 ——
  REPL 手敲、变量传值都可能越界。越界**钳到边界 + 打印**，非数字**忽略 + 打印**，不抛异常
  （积木/后台线程调用都不能炸）。
- 码值真源在 `frozen/voice.py`：`SND_NUM = 0x72` / `NUM_MIN = 0` / `NUM_MAX = 100`。
  ⚠ 改范围要**三处同步**：这两个常量 + `kblock.json5` 里滑块的 `min`/`max`。
- **平台侧必须配对应词条**：CI-33T 的【串口输入】匹配 `AA 55 72 <数据> 55 AA` 并把数据当数字播报。
  没配 = 帧照发、没声音（和 §3.2 的声音同理）。

#### 3.3 让固件对某指令有"默认动作"（不推荐）

破坏「纯事件转发」铁律：固件动作和用户积木会竞争/叠加，用户无法覆盖。如果确实要：在 `_dispatch` 里对某 cmd 加了内置动作，就必须同时保证**有用户回调时不动作**，否则两边打架。目前设计下，正确的"默认动作"做法 = 在**文档/示例程序**里给出一段 KittenBlock 模板，而不是进固件。

#### 3.4 调试

- REPL 查注册状态：`voice._handlers`（dict cmd→[fn]）确认 0x31 已挂上用户函数。
- REPL 模拟触发（不靠真语音）：`voice._dispatch(0x31)` → 应该触发 `voiceWhenFwd`。
- 真机：说指令看串口 `VOICE RX: <hex>`（确认 CI-33T 发的字节）→ `VOICE CMD: 0x%02x -> event`（确认分发）→ 狗动（确认函数体）。

### 4. 踩坑清单（一页速查）

1. **`sys.modules['__main__']` 为 None** → 必须 `set_main_globals(globals())`，三条路径都传（见 2.3）。
2. **下拉 value ≠ `_EVT_FUNCS` 后缀** → 注册不上。函数名 = `pycode` 里的 `def voiceWhen<value>()`（value 裸代入，如 `Fwd` → `def voiceWhenFwd()`）。
3. **命令码超出 `_CMD_MIN`–`_CMD_MAX`**（现 0x30–0x3F，其中 `0x3D`/`0x3E` 尚未使用）→ 命令帧被判非法、当残帧丢掉。加指令必须同步扩 `_CMD_MAX`。
4. **GPIO19/20 被 TinyUSB 占** → UART2 发不出。必须关 `usb_init()`（已在固件里关了）。
5. **下行命令帧必须是 4 字节 `BB <CMD> <PARAM> EE`** → 帧头帧尾缺一不可，CMD 还得在 0x30–0x3F 内。
   ⚠ **旧的裸 2 字节 `<CMD> <PARAM>` 已不被识别**（2026-09-26 起）：2 字节拼不成 4 字节帧，会被当残帧
   留在 `_buf` 里直到被后来的字节挤掉 —— **平台侧不改就会全部失灵**。反之平台先改、固件还是旧的则无碍。
6. **上行必须带帧** `AA 55 <CMD> <PARAM> 55 AA` → CI-33T 才认。
7. **别再用"逐字节扫命令码"那套解析**（2026-09-26 前的做法）→ 角度帧第一字节就是角度值，
   **48–63 度会落在 0x30–0x3F 里被误当运动指令**。必须按帧结构判定（首字节 `0xBB` vs `≤0xB4`）。
8. **角度帧格式归 CI-33T 管，本仓库改不了** → 只能 `≤0xB4` 一个字节，别指望加帧头或改长度；
   真要改就得动模块侧配置，届时两个判据要一起重算。
9. **KittenBlock 在线绿旗不传 hat def** → 语音事件**只能 upload+RESET**（或手动贴函数）；在线调试只对 command 积木有效。
10. **Ctrl-D 软复位不重跑 frozen app** → 改了固件后必须**物理 RESET**（或重新烧录）。
11. **改 frozen 文件≠传 /main.py** → frozen 打进固件，重编译 + 烧录才生效。
12. **`afterConnect` / `libs` import 里维护同一份参数与 `set_main_globals`** → 两处都动，别只改一处。

**用户程序执行类（与语音无关，但都会撞上）**：

13. **键盘积木（「按下x键?」「当按下x键」）只能在线** → 按键检测在**浏览器**里做，板子上没有键盘也没有这个功能。点绿旗在线跑正常；**点「下载」后这些积木退化成恒假的 `if False:`**（`lib.min.js` 的 `control_if`：`valueToCode(...) || 'False'`），按键全废。键盘遥控类程序**只能在线玩，不要下载**。
14. **下载的程序里若有顶格「重复执行」→ 板子失联** → 生成顶格 `while True:`，且循环体常全是恒假的 `if False:`（空转、无 sleep）→ 原实现里主线程 `exec` 永不返回、REPL 起不来 → KittenBlock 点什么都没反应，**每次复位都卡**。
    - **2026-09-16 起已加固**：用户程序改在**后台线程**执行（`frozen/main.py`）—— `while True:` 只空转，REPL 照常可用。失败模式从"板子变砖"降级为"程序没反应"，KittenBlock 一直连得上、能重新下载覆盖。
    - **加固前的救援步骤**（老固件 / 仍遇到失联时）：串口发 `Ctrl-C`（`\x03`）打断 → 进 REPL → `import os; os.remove('/main.py')` → 复位。
    - ⚠ **重烧 app 分区（`write-flash 0x10000`）不会清 `/main.py`** —— 它住在 `vfs` 分区，所以重烧救不回来，必须走 REPL 删文件。
15. **加固带来的行为变化（2026-09-16）** → ① **`Ctrl-C` 不再能停住后台跑的用户程序**（`py/scheduler.c` 的 KeyboardInterrupt 只投递主线程）—— **停止程序的唯一手段是复位**（物理 RESET / `machine.reset()` / REPL 里 `machine.soft_reset()`）。② 用户程序里的 `machine.soft_reset()` 变成空操作（SystemExit 只在线程内被吞）。③ 用户程序与 REPL **共享同一个全局 dict**，REPL 里改同名变量会直接影响正在跑的程序（调试时是特性，也是坑）。④ 顶格死循环会拖慢（GIL 每 32 个 VM 分支换手一次），空转循环尤其明显。

> 📌 **语音程序的推荐写法**：绿旗下面**只放初始化**（如「站立」），其余全用「当收到xx指令」回调，**不要用「重复执行」** —— 语音事件是回调式的，本来就不需要循环。这是 2026-09-16 实测可用并下载验证过的范式。

### 5. 相关文件索引

| 文件 | 角色 |
|------|------|
| `frozen/voice.py` | 事件转发核心（UART2 + 后台线程 + 注册/分发） |
| `frozen/main.py` | 启动脚本，L132 `set_main_globals`，用户程序在**后台线程**里 `exec(/main.py, globals())` |
| `frozen/manifest.py` | frozen 模块清单（加 frozen 文件要注册） |
| `kext-bpuppy/kblock.json5` | 积木定义 + `libs.import` 注入串 |
| `kext-bpuppy/extension.json` | 扩展元数据 + `afterConnect` |
| `kext-bpuppy/bpuppy.l10n.json` | 积木文本本地化 |
| `kext-bpuppy/KittenBlock扩展开发.md` | 扩展开发全指南（§13 事件积木机制） |
| `docs/操作指南.md` | §7.2.1 用户侧语音用法 |
| `PCB/硬件连接.md` | UART2/CI-33T 接线 |

### 6. 与其他部分的关系

- **voice 与 KittenBlock 蓝牙**互不干扰：语音走 UART2（GPIO19/20），蓝牙走射频，REPL/烧录走 UART0(COM14)。
- **voice 与 `bpuppy_motion`**：voice **不直接调** motion（铁律）；动作由用户程序通过扩展积木间接调。
- **事件模型**：本系统的"事件" = 固件后台线程扫描主全局 dict 按名注册 + 收到串口指令分发。这是纯板侧事件（详见 `kext-bpuppy/KittenBlock扩展开发.md` §13 关于在线/离线事件模型的讨论）。

---

## 步态仿真 (PC 端)

```bash
cd gait_sim
pip install matplotlib numpy
python gait_sim.py                    # 输出 CSV + PNG + GIF
python gait_sim.py --stride 80 --height 70 --fps 4
```

另外几个是**离线审计**用的 —— 逐帧复刻固件的状态机, 用来在动代码**之前**先把账算清楚
(它们都 `import stop_audit`, 那是共同的逐帧内核):

| 脚本 | 算什么 |
|---|---|
| `stop_audit.py` | 逐帧复刻腿循环 / 相位 / 步长淡入淡出 / 锚定锁存 —— 其余脚本的共用内核 |
| `flip_audit2.py` | 换向: 扫遍全踩地窗口的每个相位, 量翻转后位置连不连续 |
| `support_plane_scan.py` | 支撑三足平面 vs 机身平面的 pitch/roll 分量 (对比几种俯仰补偿写法) |
| `validate_limits.py` | 复刻 `traj_combo_bad()`, 算重心 / 俯仰 / 横滚的可用上限 |


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

### 内存：帧缓冲在 PSRAM，不占内部 RAM

- 帧缓冲在 **PSRAM**（`drivers/camera_driver.c:68` 的 `.fb_location = CAMERA_FB_IN_PSRAM`），
  SVGA 双缓冲下为 2 × 97KB
- DMA 中转缓冲**已取消**（`sdkconfig.bpuppy` 开了 `CONFIG_CAMERA_PSRAM_DMA=y`）。
  不开的话 `cam_hal.c:520` 会额外在**内部 RAM** 要一块连续 16KB，跑久了碎片化
  凑不出来 → 相机**当场初始化失败**（见易错点 11）
- **结论：相机完全不依赖内部 RAM，碎片化影响不到它**

自检（**开图传后**才有，走 **UART0 串口**，蓝牙看不到）：

```
I cam_hal: PSRAM DMA mode enabled
I cam_hal: Allocating 97040 Byte frame buffer in PSRAM
I cam_hal: Allocating 97040 Byte frame buffer in PSRAM
```

`enabled` + 帧缓冲地址在 `0x3C......`（PSRAM 段）才算对。若是 `disabled`，
见「sdkconfig 三层覆盖机制」那节的坑。

> 相机参数改 `drivers/camera_driver.c`（C 层），MicroPython 侧改不了默认值。

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

- `stride` = 步长**幅度** (mm, ≥0; 0 = 原地踏步) —— 方向由 `motion_set_direction(±1)` 独立设,
  给负数会被拒 (返回 false, 日志提示改用 `set_direction`)
- `lift`   = 抬脚高度 (mm), 抬腿最高点 z = height − lift
- `height` = 站立高度 (mm)

三个参数**一起校验、一起写入**（`motion_validate_params(stride, height, lift)` 一次判完），
始终直接写入、无哨兵；超限时整组拒绝并保持原值，返回 `false`。

**`speed` 不在这里** —— 它是步频 (0~10)，跟腿部轨迹无关，由独立的 `motion_set_speed(speed)` 设置
（0~10 范围检查，含 NaN 护栏）。

> 历史：`speed` 曾是本函数第 1 参、`lift` 曾住在一个单独的 `motion_set_lift()` 里。两个函数互相拿
> 对方的**当前值**校验，产生顺序耦合（`docs/error.md` §2.2 #7）；`lift` 并入本函数后该耦合消失，
> `motion_set_lift` 与 `motion_check_params` 一并删除。
>
> ⚠ 读写的**参数顺序不一致**：`get_params()` 返回
> `(speed, stride, height, lift, omega, turn, gait, direction)`，
> 而写入是 `set_params(stride, lift, height)` —— lift 在读里排第 4、在写里排第 2。
> `[1]` 是**带符号**的步长 (= 幅度 × 方向, 负号表示后退), `[7]` 是方向本身 ——
> 想知道"迈多大步"看 `abs([1])`, "朝哪边走"看 `[7]`。

### 3. 运动→静止的 `pose_trans`

运动步态切到 `GAIT_STOP` 的那一帧会启动 `pose_trans=2` (停步过渡: 0.3s smoothstep, 把足端从
当前位置缓动到站姿), 需保证 `g_was_moving` 状态正确, 否则该过渡不启动、足端**硬切**。

> 停步有两条路, 都会经过这一帧, 但收尾方式不同:
> - `set_gait("stop")` → **回正步**: 先抬脚把四腿挪回 `x=0` 再切 `GAIT_STOP` ⇒ 到这一帧时
>   足端已经在 `(0, height)`, 过渡几乎无事可做。
> - 目标速度归零 (网页滑块拖到 0) → **步长淡出**: 淡完直接切静态, 残留的足端偏移交给
>   这 0.3s 缓动滑掉 ⇒ 那个时长常数 (`FADE_OUT_TURNS`) 和平方曲线是跟它配套选的
>   (线性淡出会留 39~40mm, 全靠这 0.3s 搓地滑掉)。

### 4. `servo_init` 初始 duty

`servo_init` 设 `duty=0`, 导致初始化后舵机失能(随机位置)。开机 `init_all` 后必须用 `set_angle` 设定蹲姿再进入姿态模式 (main.py 已处理)。

### 5. 蹲姿角度定义

crouch 姿态角度在 Python `poses.py` 定义 (`CROUCH = [135,45,...]`)。若修改腿结构, 需同步 `poses.py` 的蹲姿角度 (且 `servo_init_all` 非幂等, 用户程序勿重复调用)。

### 6. 校准公式

**三点校准 (现行, `cal_point`)**: 每通道存 0°/90°/180° 三个点 `c[0],c[1],c[2]` —— 值为"命令该角度时舵机实际应转的角度", 中间**分段线性插值** (`servo_driver.c:223-233`):

```
set_angle(ch, A) → A ≤ 90 时发 c[0] + (c[1]-c[0])×A/90
                   A > 90 时发 c[1] + (c[2]-c[1])×(A-90)/90
```

同时补偿零点偏移和斜率/非线性误差。

**旧单点法 (`cal`, 已不推荐, 接口保留)**: `cal(ch, ref_deg)` ≡ `cal_point(ch, 1, ref_deg)`, 只设 90° 点, 0°/180° 保持恒等 → `set_angle(90)` 发送 `ref_deg`, 只补偿零点、不补偿斜率。上述公式**只对旧的单点法成立**; 三点校准后除 90° 点外一般 `set_angle(A) ≠ A`。

### 7. GO 自适应中的 eff_speed

GO 的 duty/gap/stride/height/lift 查表使用 `eff_speed` (实际 speed 的绝对值, 经过半周期平滑), 不是 `target_speed`。BLE 写 `target_speed`, 实际 speed 逐步跟随。

### 8. BLE 停止

停止时设 `speed=0, stride=0`。d=0 调 `set_speed(0)` + `set_params(0, 30, 70)` + `set_gait("stop")`。

### 9. GPIO 引脚映射 (已确定)

| 舵机 | GPIO | 舵机 | GPIO |
|------|------|------|------|
| LF_HIP 左前大腿 | 1 | RF_HIP 右前大腿 | 40 |
| LF_KNEE 左前小腿 | 42 | RF_KNEE 右前小腿 | 38 |
| LH_HIP 左后大腿 | 2 | RH_HIP 右后大腿 | 39 |
| LH_KNEE 左后小腿 | 41 | RH_KNEE 右后小腿 | 45 |

IMU: I2C0 (SDA=GPIO14, SCL=GPIO21, addr=0x68)。芯片自适应: WHO_AM_I 识别 MPU6050(0x68)/MPU6500(0x70)/MPU9250(0x71,0x73), 也可以什么都不接。**有没有磁力计不看芯片型号** —— 一律由 `bpuppy_imu.has_mag()` 回答 (它看 AK8963 真被认出来没有); REPL 可用 `bpuppy_imu.get_chip()` / `has_mag()` 查询。
UART2: GPIO20=RX, 19=TX (CI-33T / micro:bit, ⚠ 2026-08-19 起反转 TX=19/RX=20; ⚠ GPIO19/20=USB_D-/D+, 固件已关 TinyUSB 释放, 见 PCB/硬件连接.md)。
UART1: GPIO4=TX, 5=RX (与摄像头 SCCB SDA/SCL 复用, 手动 init)。
I2C1: GPIO9=SDA, 10=SCL (与摄像头 D1/D3 复用, 手动 init)。
ADC: 电池检测启用 (电池=GPIO3=ADC1_CH2, 分压 51k/10k, 软件 ×6.1)。`bpuppy_adc.init()` 同时激活 GPIO48 WS2812 电池指示灯 (≥7.4V 蓝 / 6.6~7.4V 渐变 / ≤6.6V 红 / <6.4V 闪烁)。电量定标与之同源: **7.4V=100%, 6.6V=0%** (`bpuppy_led.batt_pct()` / `voltage.read_pct()`)。
完整 GPIO 分配表见 `PCB/硬件连接.md`。

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

### 11. 内部 RAM 只看**最大连续块**，不看总空闲

判断内部内存够不够，看 `heap_caps_get_largest_free_block()`（最大**连续**块），
不看 `free`（总空闲）。两个数经常差很远，而**要连续块的分配只看前一个**。

- 内部 RAM 是这个板子上**最稀缺**的资源（总 D/IRAM 才 ~222KB），
  BLE / WiFi / 所有线程栈 / 各种缓冲都从它出
- 往内部 RAM 要大块连续内存之前先问：**这块能不能挪到 PSRAM？**
  （8MB，且不跟上面那些抢）
- 碎片化导致的失败特征是「**间歇性 + 和时间相关**」（跑几分钟没事、十几分钟才挂），
  比必然失败难查 —— 所以**别把大块内存放在内部 RAM 上赌碎片**

> **实例**：`cam_hal.c:520` 默认会在内部 RAM 要一块连续 16KB 做 DMA 中转缓冲，
> 跑久了凑不出来就**当场初始化失败**（`Camera init_adv failed: 0xffffffff`）。
> 已在 `sdkconfig.bpuppy` 开 `CONFIG_CAMERA_PSRAM_DMA=y` 取消该块 ——
> 相机现在**完全不依赖内部 RAM**。

### 12. `_thread.stack_size()` 是**全局默认值**，设完必须还原

两条硬规则（`py/modthread.c`）：

1. **它设的是"此后新建线程"的默认值，是全局状态。** 设完不还原，后面每个模块
   起线程都按这个尺寸要 —— 内部 RAM 不够时 `start_new_thread` 直接抛
   `OSError: can't create thread`，而且看当时碎片，**时好时坏**。
2. **不带参数调用 `stack_size()` 不是安全的读** —— 它返回旧值的同时会把设置
   **重置成 0**。别拿它当调试探针。

固定写法（设完立刻还原；`start_new_thread()` 返回前就已读走该值，还原不影响它）：

```python
_thread.stack_size(16 * 1024)
_thread.start_new_thread(fn, args)
_thread.stack_size(0)          # 0 = 端口默认 (esp32: 5120)
```

尺寸：esp32 端口默认 `MP_THREAD_DEFAULT_STACK_SIZE = 5KB`，下限
`MP_THREAD_MIN_STACK_SIZE = 4KB`（传更小的值会被**向上**夹到 4KB）。
线程栈从**内部 RAM** 分配，所以这条和易错点 11 是同一个约束。

### 13. KittenBlock 积木的 `pycode` 里**不许出现复合语句**（`if` / `def` / `for` …）

**现象**：积木在线执行（点积木 / 绿旗）点了"没反应" —— 设备侧状态一点没变，
**不嘤、不报错、控制台连 Traceback 都没有**。极易误判成"固件没实现这个功能"。
（2026-09-28 `setSpeed` 方向改不了，读回 `get_params()[7]` 恒为 `1.0`；
2026-09-29 查实「参数被拒时嘤嘤叫」从来听不到 —— 同一个根因的两次表现。）

**机制（两半，缺一不可）**：`pycode` 数组被 KittenBlock 用 `\r\n` 拼好后**逐行原样下发**到友善 REPL。

*第一半 —— 缩进只加不减*：`readline_auto_indent()`（`shared/readline/readline.c:483-527`）
每收一个 `\r`，就按"缓冲区里最后一行的整段缩进"补等量空格，该行以 `:` 结尾再多补 4：

```
本行最终缩进 = 上一行最终缩进 + 4×(上一行以 ':' 结尾) + 本行自带的前导空格
```

**缩进单调不减，永远回不去** ⇒ `else:` 被顶到与 `if` 体同列 ⇒ `SyntaxError`。

*第二半 —— 复合语句闭不了合*：REPL 判"这段说完没有"只看**缓冲区最后一个字符是不是 `\n`**
（`py/repl.c:149`：`if (starts_with_compound_keyword && i[-1] != '\n') return true;`）。
而自动缩进**连"空行"都给填空格**（`readline.c:503-527`）：`if not _ok:` 之后那行被填 4 格、
body（自带 4 格）之后那行被填 **8 格** ⇒ 你敲的那个"空行"`i[-1] == ' '` ⇒ **仍算没说完**，
要**连按两次回车**（第二行因"连续两个全空格行不再加缩进"的规则才真空）才执行。
**KittenBlock 逐行下发时不补那个空行** ⇒ 这类块**永远停在续行状态、一行都不执行**。
此时在续行里按 **Ctrl-C 是静默整段丢弃**（`readline.c:158-160` → `pyexec.c:446-450`）——
不执行、不报错、直接回 `>>>`，这就是"没声音也没提示"的来源。
（续行里 Ctrl-D 是"删除光标处字符"（`readline.c:162-165`），不是执行。）

*实锤对照*（2026-09-29，拿模拟脚本跑 `HEAD` 版与修后版，同一份判据）：

| | 逐行发不安全（bTool 判据） | 段末卡在续行 ⇒ 永不执行 |
|---|---|---|
| 改前 | 5 个 `if` 块 | **`advMotion` / `bodyPose` / `setCenter` / `setSpeed` / `setTurn`** |
| 改后 | 无 | 无（只剩 `voiceWhen`，见文末例外） |

用户板上真实回显 `>>>` / `...`+8 空格，与模拟出的缩进列 `[0, 0, 8]` **逐列对上**。
另一条独立印证：`d:\bTool` 的终端粘贴早就在防这件事（`btool.py:1791-1804`），
注释写着「`if True: pass` 这种**单行**复合语句也会被判成"没输完" → 板子**卡在续行状态等空行**」。

**硬规则：整段 `pycode` 一条复合语句都不许有** —— 不写 `if` / `for` / `while` / `try` / `def` /
`with` / `class` / `async` / `@`，不写 `else:` / `elif:` / `except:` / `finally:`，不写缩进。
**只要有一行以复合关键字开头，整段就被黏住**，前面那些本来正常的行也一起不执行 ——
所以不存在"只改一部分"的折中写法。

**安全写法 = 顶格单句，失败反馈用 `or` 短路**（`advMotion` / `bodyPose` / `setCenter` /
`setSpeed` / `setTurn` 五个块现在都是这个形状）：

```json5
pycode: [
  '_ok = bpuppy_motion.set_center([OFFSET])',
  '_ok = _ok or voice.say(*voice.SND_YING) or False'
]
```

`_ok` 真 ⇒ `or` 短路、不求值右边（**不嘤**）；假 ⇒ 求值 `voice.say(...)` ⇒ **嘤一声**。
尾部 `or False` 把 `say` 返回的 `None` 收回真 bool（「设置成功？」读数照旧），
写成赋值也免得 REPL 把 `True` 回显到串口。

**自查**（几秒验一遍，不用上板）：直接套 bTool 那条判据 —— 逐行检查**空行 / 有缩进 /
以 `:` 结尾 / `@` 开头 / 以复合关键字开头**，任一命中就不能逐行发：

```python
KEYWORDS = ("if", "while", "for", "try", "with", "def", "class", "async")
def pycode_ok(lines):                    # lines = pycode 数组展开后的各行
    for ln in lines:
        if not ln.strip() or ln[0] in ' \t' or ln[0] == '@': return False
        if ln.rstrip().endswith(':'): return False
        if ln.split(None, 1)[0].rstrip('(') in KEYWORDS: return False
    return True                          # 全过 = 逐行下发安全
```

**确实需要块逻辑时的两个逃生口**：

1. 把多语句下沉成板上**一个函数调用**，`pycode` 保持单行
   （例：`_ok = bpuppy_motion.set_speed(_speed)`）。
2. 走**上传到板子跑** —— 整文件编译，不经过 REPL，完全不受这条约束。
   （`frozen/main.py` 这类"文件内容"同理安全；`tools/` 里那几个多行脚本走的是
   paste 模式 `Ctrl-E`…`Ctrl-D`，也不过 readline 的自动缩进。）

> **同类位置**：`操作指南.md` 里给人手敲/粘贴的片段（`:82`、`:102`、`:850`、`:863`）带 2 行以上块体，
> 直接粘进 REPL 会踩同一条 —— 改用粘贴模式（`Ctrl-E` 粘贴，`Ctrl-D` 结束），或把块体压成 1 行。
> **唯一的例外是 `voiceWhen[VOICE]` 那个 hat 块**（`kblock.json5:100` 的 `def voiceWhen[VOICE]()`）：
> 它**故意靠"不闭合"**接用户叠在它下面的积木，投递路径也与普通块不同 —— **别照抄它的形状**。

### 14. **VFS 优先于 frozen** —— 板子上的同名 `.py` 会**盖住**固件里的模块

2026-10-01 实测：把 `heading_anchor.py` / `heading_follow.py` 冻结进固件、编译烧录，
板上 `import heading_anchor` **加载的仍是 VFS 里那份陈旧副本**（`hasattr(anc,'on')` 为 `False`，
还在用早已删掉的 `anchor()`）。删掉 VFS 那两份之后立刻正常。

**查找顺序：`sys.modules` → VFS 根目录 → frozen。** 由此：

- **冻结 ≠ 一定生效。** 板子根目录只要有同名文件，它说了算。表现是"编了、烧了、没生效"，
  而且**版本串也会骗你** —— 这次固件时间戳确实是新的，模块照样是被旧的那份顶着。
- 冻结仍然要做：**新板子／没传过的板子开箱即用**（KittenBlock 用户不会手工传模块）。
- 判据一眼看穿：`import heading_anchor as a; print(hasattr(a, 'on'))` —— 有 `on` 才是新版。
- 修：`import os; os.remove('heading_anchor.py')`。
  ⚠ **`os.listdir("/")` 返回的名字不带斜杠**，别拿 `"/xxx.py"` 去比对 —— 比不中、静默不删，
  今天就这么白跑一轮。
- ⚠ **只烧 app 分区（`0x10000`）不擦 VFS**，所以旧副本会一直在，跨固件版本存活。

> **反面用法（这也是个特性）**：想快速验一版改动，直接把它传到 VFS 盖住固件版，
> **不用重编译**；`os.remove` 掉就退回固件版。

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

### ⚠ 改 `sdkconfig.bpuppy` 后必须删掉 `build/sdkconfig`

**`build/sdkconfig` 已存在时，往 `sdkconfig.bpuppy` 新增的配置不会生效** ——
编译不报错，烧进去没有任何提示，那个 `CONFIG_*` 一直保持关闭。

改配置后固定走这三步 + 一次核对：

```bash
cp build/sdkconfig /tmp/sdkconfig.before   # 备份
rm build/sdkconfig                          # 从 defaults 重新生成
bash build.sh
diff /tmp/sdkconfig.before build/sdkconfig  # 应当只有你改的那几行不同
```

那个 `diff` 是关键：它同时验证「新配置生效了」和「没有别的配置被意外改动」。
**改完一定要 `grep <你的 CONFIG_*> build/sdkconfig` 亲眼确认**，别假定它生效了。

> `build/` 整个是产物目录，`sdkconfig` 不在 git 里，删掉是安全的。
> 要保留的配置全在 `sdkconfig.bpuppy` / `sdkconfig.defaults`（这两个在 git 里）。

---

## 修改代码指引

| 需求 | 改哪个文件 |
|------|-----------|
| 修改舵机 GPIO 引脚 | `drivers/servo_driver.c` → `servo_init_all()` |
| 修改步态参数 | `drivers/motion_task.cpp` |
| 修改 IK 腿长/髋距/限位 | 推荐运行 `cal_ik()` / `set_body_dims()` / `set_joint_limits()` → NVS 持久化；改默认值则 `drivers/ik.h` |
| 添加 MicroPython C 函数 | 对应 `drivers/*.c` + 注册到模块表 |
| 修改 Python 启动逻辑 | `frozen/main.py` |
| 修改 BLE 协议 | `drivers/ble_driver.c`（GATT 服务）+ `drivers/ble_stream.c`（dupterm 桥接）。原 `frozen/ble_hiwonder.py` 已删除 |
| 修改语音事件/命令码映射 | `frozen/voice.py`（固件侧，需重编译烧录）+ `kext-bpuppy/kblock.json5`（扩展侧，重打包 zip） |
| 修改 KittenBlock 扩展/积木 | `kext-bpuppy/`（重打包 zip + 推送）。⚠ 积木若引用新模块，`kblock.json5` 的 `libs` 和 `extension.json` 的 `afterConnect` **两处都要加 import**，漏一处在线就是 `NameError` |
| 修改航向锁定闭环（`anc` / 「航向锁定 偏转」积木） | 改 `frozen/heading_{anchor,follow}.py` → 重编译。**源只有这一份**（2026-10-01 起删掉了 `mpy_modules/` 的副本：两份会漂，而且 VFS 优先 —— 传上去的那份会盖住固件版）。⚠⚠ 板子根目录若**残留** `/heading_anchor.py`，照样会盖住固件版、表现为"编了烧了没生效" ⇒ 用 `os.remove` 删掉。⭐ `TURN_SIGN` / `kp` / 死区这些**运行时**用 `anc.set()` / `anc.cfg` 改，**不用重编译** |
| 修改摄像头参数/格式 | `drivers/camera_driver.c` → `init_adv()` 或 MicroPython `bpuppy_camera.init_adv()` |
| 改相机内存走向 (内部 RAM ↔ PSRAM) | `sdkconfig.bpuppy` → `CONFIG_CAMERA_PSRAM_DMA`（改完**必须删 `build/sdkconfig`**，见下节） |
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
- [ ] 烧录后 UART0 串口可连 REPL (115200, CH343 端口见设备管理器; **USB-CDC 不可用**, 见上文「Windows 烧录」)
- [ ] 启动 banner 显示 "bPuppy Robot Dog - ESP32-S3"
- [ ] 上电自动站立，无跳动
- [ ] `import bpuppy; bpuppy.version()` 返回版本号
- [ ] KittenBlock 模式 (`BPUPPY_BLE_KEBLOCK`): 广播 `bPuppy_XXXX`，KittenBlock 蓝牙可连（安卓/iPad Bluefy/PC）
- [ ] ~~Hiwonder 模式 (`BPUPPY_BLE_HIWONDER`): 广播 `mechdog_XX`，Wonderbot App 可连~~ —— 已废弃：`ble_hiwonder.py` 已删除，切到该模式也没人解析 App 报文
- [ ] 蓝牙 REPL：`os.dupterm(None)` 返回 BLE 流对象（C 层自动注册）
- [ ] 语音: 开机日志出现 `voice: CI-33T ready`；说"前进" → 串口 `VOICE RX: bb3100ee` + `VOICE CMD: 0x31 -> event` → 狗走
      （⚠ 前提：平台侧 14 条命令词已改成 `BB <CMD> 00 EE`；旧裸 2 字节命令不再被识别，见踩坑 #5）
- [ ] 语音: 开机日志出现 `voice: event 0x31 -> voiceWhenFwd`（事件函数已注册）
- [ ] 语音: **声源角度** —— 拍手/说话 → 串口 `VOICE RX: <角度>000000`，REPL 里 `voice.SoundAngle` 变 0–180
      （没收到过是 `-1`）。**关键回归**：角度 48–60 度时**不应**出现 `VOICE CMD: 0x30/0x35/0x3c` 之类误派发
- [ ] 电量: REPL 里 `voltage.read_pct()` 与手算 `(read_v()-6.6)/0.8*100` 对得上（0–100 整数）；
      `voltage.stop()` 后 `read_pct()` 是 **0**（不是 -1）
- [ ] 语音: **播报电量内置动作** —— `voice._buf = bytes([0xBB,0x3F,0x00,0xEE]); voice._parse()`
      或直接说"播报电压" → CI-33T 出声报出百分比。**关键回归**：`0x37`/`0x30` 等其它码
      **仍然只打 `VOICE CMD` 日志、不出声**（例外只开了 `0x3F` 一个）
- [ ] 相机: 开图传后串口出现 `cam_hal: PSRAM DMA mode enabled` + 两行 `frame buffer in PSRAM`（**串口**看，蓝牙看不到）

---

## 已知问题 / 待解决

### 图传「关了再开」必挂、久开后开不了图传 —— ✅ 已修复

两个**互相独立**的原因：

1. **关流时在消费者还活着的情况下 `deinit()` 相机** —— 关流只等 100ms，而
   `capture()` 可以阻塞 4000ms。改成「最后一个退出的流线程负责释放」
   （`frozen/camera_stream.py`）。
2. **相机要在内部 RAM 要一块连续 16KB** 做 DMA 中转缓冲，跑久了碎片化凑不出来。
   已取消该块（`sdkconfig.bpuppy` 的 `CONFIG_CAMERA_PSRAM_DMA=y`）。

排查要点：**别拿「以前这个版本是好的」当线索**（相机代码 6~8 周没动过，是以前
没测过「关了再开」、也没跑过那么久），要拿**日志里的数**当线索 ——
`largest free block:` 那个值。详见易错点 11 / 12 与「OV2640 摄像头」节。

### IMU 校准后有固定偏差 —— ✅ 已修复
`calibrate(300)` 原先只补偿 Z 轴（1g），X/Y 零偏未标定 → 水平放置姿态角偏 ~12°。现已补 X/Y 零偏（**校准时机身必须水平**）。修复后水平放置读数应 ≈0。

### 自平衡起始方向依赖 —— 已修偏置, 待验证
`balance.start()` 时，若机身起始方向与启动瞬间不一致，会稳定在肉眼不平的角度。根因是加速度计 X/Y 偏置（上述），偏置修复后应缓解，**待实测验证**。

---

| 组件 | 版本 |
|------|------|
| MicroPython | v1.22.1 |
| ESP-IDF | v5.1.2 |
| 固件版本 | `202607090001`（`bpuppy.version()` 另附编译日期时间） |
| 芯片 | ESP32-S3 WROOM-1 N16R8 |
