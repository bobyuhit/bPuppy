# bPuppy — ESP32-S3 四足机器狗

## 产品概述

bPuppy 是基于 ESP32-S3 的 12 自由度四足机器狗，运行 MicroPython v1.22.1 + ESP-IDF v5.1.2。
底层 C 驱动（舵机、IMU、BLE、IK、步态），上层 Python 应用，兼容 Hiwonder Wonderbot App 蓝牙遥控。

| 项目 | 规格 |
|------|------|
| 主控 | ESP32-S3 WROOM-1 N16R8 (16MB Flash, 8MB Octal PSRAM) |
| 舵机 | 8× 模拟舵机 (每条腿 2DOF: 髋 + 膝) |
| IMU | QMI8658 6轴 (I2C, 暂未启用) |
| 通信 | BLE (NimBLE, Hiwonder Wonderbot 协议) |
| 控制台 | USB-JTAG CDC (115200bps, 直连 USB) |
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

### Windows 烧录

```powershell
# ESP32-S3 原生 USB-JTAG，直连 USB 即可
esptool --chip esp32s3 --port COM10 --baud 115200 write-flash `
  0x10000 build/micropython_bpuppy.bin
```

> 日常增量烧录只写 app 分区 (0x10000)。首次或改了 bootloader/分区表才需要全烧。

### 改了哪些文件需要怎么构建

| 改了什么 | 怎么构建 |
|---------|---------|
| `frozen/*.py` | `idf.py build` → 只烧 app 分区 |
| `drivers/*.c/.cpp` | `idf.py build` → 只烧 app 分区 |
| `CMakeLists.txt` / `sdkconfig.*` / `partitions.csv` | `rm -rf build && idf.py build` → **全烧** |

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
MicroPython 层:   frozen/main.py → 上电自动站立 + BLE 后台
                       ↑ import
C Extension API:  bpuppy_servo / bpuppy_imu / bpuppy_ble / bpuppy_motion
                       ↑ MP_REGISTER_MODULE
C 驱动层:
  servo_driver.c    — LEDC PWM 8路舵机 (S3 统一 LS mode)
  imu_driver.c      — I2C QMI8658 IMU (暂未启用)
  ik.h / ik.c       — 2-DOF 逆运动学
  ble_driver.c      — NimBLE GATT (Hiwonder Wonderbot 协议)
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
| `drivers/ble_driver.c` | NimBLE GATT 服务 |
| `frozen/main.py` | 启动脚本 — 初始化舵机 → 自动 stand_up → 启动 BLE |
| `frozen/ble_hiwonder.py` | BLE 遥控协议 — GO 自适应, speed 0~12 |
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

- **静态↔静态** (stand/crouch/sit/play): 每帧限速 3° smoothstep, 自然平滑
- **启动** (stand→go): 相位锁到 all_stance_mid, 0.3s smoothstep 足端从站立缓动到轨迹
- **停止** (go→stand): 0.3s smoothstep 足端从轨迹缓动回 `(0, height)`
- **变速**: 每半周期 ±3.0 步进跟随 target_speed, 避免突变

### 静态姿态

| gait | 说明 | 特点 |
|------|------|------|
| `"stand"` | 站姿 | IK 计算, 高度由 `set_params` 设定 |
| `"crouch"` | 蹲伏 | 固定角度, 四腿折叠, 关机前放松舵机 |
| `"sit"` | 猫坐 | 前腿撑地、后腿折藏身下 |
| `"play"` | 邀玩 | 前低后高 + 4Hz 摇臀 8 次 ±10° |

### 足端轨迹 (smoothstep 摆线)

```
摆动相: ease = t²(3-2t), z = H - lift·sin(ease·π), x = -S/2 + S·ease
支撑相: z = H, x = S/2 - S·(p-duty)/(1-duty)
```

---

## 上电行为

1. `servo_init_all()` — 初始化 8 路 LEDC（duty=0，舵机松）
2. `load_cal()` — 从 NVS 加载校准值
3. 等待 REPL 手动控制（motion 未启动，需手动 `start()`）
4. `motion.start()` — 创建 50Hz FreeRTOS 任务
5. `motion.stand_up()` — 蹲姿 0.3s → 3s smoothstep 站立
6. BLE 后台线程启动

上电即自动站立, 无需手动指令。

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
python tools/capture.py COM14
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
- `stride` = 步幅+方向 (正=前, 零=原地踏步, 负=后)
- `height` = 站立高度 (mm)

三个参数始终直接写入，无哨兵。

### 3. `stand_up` 完成后的 `pose_trans`

`stand_up` 结束切到 `GAIT_STAND` 时, 必须同时设 `g_was_moving = false` 并更新 `g_prev_fz` 为当前 height, 否则下一帧触发 `pose_trans=2` (停步过渡), 导致腿瞬间跳起再落下。

### 4. `servo_init` 初始 duty

`servo_init` 设 `duty=0`, 导致初始化后舵机失能(随机位置)。如需上电即稳定, 应在 `init_all` 后立即用 `set_angle` 设定所有舵机到安全姿态, 或在 `servo_init` 中设非零初始 duty。

### 5. `is_crouch` 与 `stand_up` 蹲姿一致性

`is_crouch` 和 `stand_up` 起点的蹲姿必须使用相同舵机角:
- 左腿: hip=135°, knee=45° (折叠)
- 右腿: hip=45°, knee=135° (折叠)

### 6. 校准公式

`cal(ch, ref_deg)` → offset = `ref_deg - 90`。`set_angle(90)` → 发送 `90 + offset` 到舵机。
含义: "舵机要转到 ref_deg° 腿才垂直" → offset 补偿后 set_angle(90) 腿正好垂直。

### 7. GO 自适应中的 eff_speed

GO 的 duty/gap/stride/height 查表使用 `eff_speed` (实际 speed 的绝对值, 经过半周期平滑), 不是 `target_speed`。BLE 写 `target_speed`, 实际 speed 逐步跟随。

### 8. BLE 停止

停止时设 `speed=0, stride=0`。d=0 调 `set_params(0, 0, 70)` + `set_gait("stand")`。

### 9. GPIO 引脚映射 (已确定)

| 舵机 | GPIO | 舵机 | GPIO |
|------|------|------|------|
| LF_HIP 左前大腿 | 1 | RF_HIP 右前大腿 | 42 |
| LF_KNEE 左前小腿 | 2 | RF_KNEE 右前小腿 | 41 |
| LH_HIP 左后大腿 | 47 | RH_HIP 右后大腿 | 45 |
| LH_KNEE 左后小腿 | 21 | RH_KNEE 右后小腿 | 48 |

IMU 已暂时屏蔽。USB D+/D- (GPIO19/20) 已恢复，UART0 (GPIO43/44) 可用作备用串口。

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
- [ ] 烧录后 USB CDC 串口可连接 (COM10, 115200)
- [ ] 启动 banner 显示 "bPuppy Robot Dog - ESP32-S3"
- [ ] 上电自动站立，无跳动
- [ ] `import bpuppy; bpuppy.version()` 返回版本号
- [ ] BLE 遥控正常 (Wonderbot App 可连接)

---

| 组件 | 版本 |
|------|------|
| MicroPython | v1.22.1 |
| ESP-IDF | v5.1.2 |
| 固件版本 | V1.0_2026071001 |
| 芯片 | ESP32-S3 WROOM-1 N16R8 |
