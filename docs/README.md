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

**当前固件参数（C 代码实际值）：**

| 参数 | 值 | 来源 |
|------|-----|------|
| 大腿 L1 | 40mm | `ik.h: IK_L1_DEFAULT` |
| 小腿 L2 | 40mm | `ik.h: IK_L1_DEFAULT` |
| 前后髋距 | 120mm | `motion_task.cpp: BODY_HALF_L=60` |
| 左右髋距 | 118mm | `motion_task.cpp: BODY_HALF_W=59` |
| 膝角范围 | 10°~170° | `ik.h: IK_KNEE_MIN/MAX` |
| 速度范围 | 0~12 | `motion_set_params()` |
| Walk 最优 | speed=2.5, stride=70, height=60 | 实测 |
| Trot 最优 | speed=8, stride=50, height=70 | 实测 |

---

## 编译环境

### 版本铁律

```
MicroPython v1.22.1  ──只能搭配──▶  ESP-IDF v5.1.2
                                    ⚠ 不可用 v5.3 / v5.5
```

### Docker 编译（唯一推荐方式）

```bash
# 使用官方 ESP-IDF 镜像
docker run --rm -v "d:\HiWonder\CODE\Dog\bPuppy:/workspace" \
  espressif/idf:v5.1.2 bash -c "cd /workspace && idf.py build"
```

> 首次需先 `idf.py set-target esp32s3`。日常增量编译只需 `idf.py build`。
> 产物: `build/micropython_bpuppy.bin`

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
| `drivers/ik.c` | 2-DOF 逆运动学, L1=L2=40mm |
| `drivers/servo_driver.c` | LEDC PWM + NVS 校准 (`cal(ch, ref_deg)`) |
| `drivers/ble_driver.c` | NimBLE GATT 服务 |
| `frozen/main.py` | 启动脚本 — 初始化舵机 → 自动 stand_up → 启动 BLE |
| `frozen/ble_hiwonder.py` | BLE 遥控协议 — GO 自适应, speed 0~12 |
| `gait_sim/gait_sim.py` | PC 端步态仿真 — CSV/PNG/GIF |
| `docs/操作指南.md` | 日常操作手册 |

---

## 步态算法概要 (motion_task.cpp)

### GO 自适应

| speed | duty | gap | stride | height | 实际 |
|-------|------|-----|--------|--------|------|
| ≤4 | 0.20 | 0.04 | 70 | 60 | walk |
| 4~6 | 插值 | 插值 | 70→50 | 60→70 | 混合 |
| ≥6 | 0.40 | 0.10 | 50 | 70 | trot |

所有参数基于**实际 speed**(经半周期平滑跟随), 非 target_speed。

### 姿态过渡

- **启动** (stand→go): 相位锁到 all_stance_mid, 0.3s smoothstep 足端从站立缓动到轨迹
- **停止** (go→stand): 0.3s smoothstep 足端从轨迹缓动回 `(0, height)`
- **变速**: 每半周期 ±3.0 步进跟随 target_speed, 避免突变

### 足端轨迹 (smoothstep 摆线)

```
摆动相: ease = t²(3-2t), z = H - lift·sin(ease·π), x = -S/2 + S·ease
支撑相: z = H, x = S/2 - S·(p-duty)/(1-duty)
```

---

## 上电行为

1. `servo_init_all()` — 初始化 8 路 LEDC
2. `load_cal()` — 从 NVS 加载校准值
3. `set_angle × 8` — 预设蹲姿舵机角
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

---

## 开发注意事项 — 易错点总结

### 1. 膝角约定

**IK `knee_angle`**: 0°=折叠, 180°=伸直。
**舵机**: 左膝 `servo = knee_deg`, 右膝 `servo = 180 - knee_deg`。
两边 servo=90° 时均为直角。修改任何膝角相关代码前必须确认方向。

### 2. `motion_set_params` 的参数语义

- `stride >= 0` → 写入新值 (0=原地踏步)
- `stride < 0` → 不更新 (保持原值)
- `height > 0` → 写入新值
- `height <= 0` → 不更新

用 `set_params(0, 0, 0)` 来停止但不改 stride/height。**切勿用 `set_params(0, 0, 0)` 作为"清零"**，因为 stride=0 是有效的（原地踏步）。

### 3. `stand_up` 完成后的 `pose_trans`

`stand_up` 结束切到 `GAIT_STAND` 时, 必须同时设 `g_was_moving = false` 并更新 `g_prev_fz` 为当前 height, 否则下一帧触发 `pose_trans=2` (停步过渡), 导致腿瞬间跳起再落下。

### 4. `servo_init` 初始 duty

`servo_init` 设 `duty=0`, 导致初始化后舵机失能(随机位置)。如需上电即稳定, 应在 `init_all` 后立即用 `set_angle` 设定所有舵机到安全姿态, 或在 `servo_init` 中设非零初始 duty。

### 5. `is_sit` 与 `stand_up` 蹲姿一致性

`is_sit` 和 `stand_up` 起点的蹲姿必须使用相同舵机角:
- 左腿: hip=135°, knee=45° (折叠)
- 右腿: hip=45°, knee=135° (折叠)

### 6. 校准公式

`cal(ch, ref_deg)` → offset = `ref_deg - 90`。`set_angle(90)` → 发送 `90 + offset` 到舵机。
含义: "舵机要转到 ref_deg° 腿才垂直" → offset 补偿后 set_angle(90) 腿正好垂直。

### 7. GO 自适应中的 eff_speed

GO 的 duty/gap/stride/height 查表使用 `eff_speed` (实际 speed 的绝对值, 经过半周期平滑), 不是 `target_speed`。BLE 写 `target_speed`, 实际 speed 逐步跟随。

### 8. BLE 停止

停止时只设 `speed=0`, stride/height 不更新。d=0 调 `set_params(0, 0, 0)` + `set_gait("stand")`。

### 9. GPIO 引脚为占位值

`servo_init_all()` 中的 GPIO (1,2,47,21,42,41,45,48) 和 README 文档中的 (4-7,15-18) **不一致**, 均为占位值, 待 PCB 确定后统一。

---

## 版本

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
| 修改 IK 腿长 | `drivers/ik.h` + `motion_task.cpp` 默认值 |
| 添加 MicroPython C 函数 | 对应 `drivers/*.c` + 注册到模块表 |
| 修改 Python 启动逻辑 | `frozen/main.py` |
| 修改 BLE 协议 | `frozen/ble_hiwonder.py` |
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
