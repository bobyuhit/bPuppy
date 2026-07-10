# AI_Read — bPuppy 项目工作指导

> **面向对象**: 后续 AI 助手（Claude、Copilot 等）
> **目的**: 让 AI 能够独立完成项目编译环境配置、构建、烧录和调试

---

## 第零步: 本项目是怎么到你手上的

这个 bPuppy 项目是由前一个 AI（Claude）从 bDog (ESP32) 项目迁移到 ESP32-S3 创建的。用户将从 Linux 容器的 `/workspace/bPuppy/` 手动拷贝到 Windows 主机 `D:\HiWonder\CODE\Dog\bPuppy\`，然后交给你。

### 你需要确认的第一件事

```bash
# 检查项目完整性 — 关键文件必须存在
ls CMakeLists.txt sdkconfig.defaults sdkconfig.bpuppy partitions.csv
ls main/main.c main/idf_component.yml
ls drivers/servo_driver.c drivers/imu_driver.c drivers/micropython.cmake
ls frozen/main.py frozen/ble_hiwonder.py
ls docs/README.md
```

如果以上文件都齐全，**立即跳过所有代码修改，直接进入 §3 或 §4 搭建编译环境**。所有代码迁移工作已由上一个 AI 完成。

### 你不需要做的事

- ❌ 不需要修改任何 `.c` / `.h` / `.py` 源码（已适配 ESP32-S3）
- ❌ 不需要考虑 ESP32 向后兼容（此项目仅支持 S3）
- ❌ 不需要创建新文件（项目结构已完整）
- ✅ 唯一任务: **搭环境 → 编译 → 烧录 → 验证**



---

## ⚠️ 硬性约束（不可违反）

### 编译环境: 必须使用 Docker

**禁止**在 Windows 裸机直接编译。ESP-IDF 工具链版本敏感、路径依赖强，裸机安装极易出现 Python 版本冲突、工具链缺失等问题。

**必须**使用 Docker 容器，容器内需包含:
- ESP-IDF **v5.1.2**（精确版本，不可用 v5.3/v5.5）
- ESP32-S3 工具链 (`xtensa-esp32s3-elf-gcc`)
- CMake ≥ 3.16 + Ninja

如果用户尚未准备 Docker 镜像，你的第一步是帮用户创建它。参考以下 Dockerfile:

```dockerfile
FROM ubuntu:22.04

RUN apt-get update && apt-get install -y \
    git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util \
    libusb-1.0-0 curl && apt-get clean

RUN mkdir -p /opt/esp && cd /opt/esp && \
    git clone --branch v5.1.2 --depth 1 \
    https://github.com/espressif/esp-idf.git idf-v5.1.2 && \
    cd idf-v5.1.2 && ./install.sh esp32s3

RUN echo 'source /opt/esp/idf-v5.1.2/export.sh > /dev/null 2>&1' >> /root/.bashrc
WORKDIR /workspace
MicroPython 组件: 必须走 micropython-helper 官方流程
本项目的 MicroPython 集成不是手动移植，是通过 mr9you/micropython-helper 组件（IDF Component Manager）自动管理的。

官方文档: https://github.com/mr9you/micropython-helper

关键规则:

不要手动下载 MicroPython 源码放到 components/ 下
不要用 make 构建（ESP-IDF v5.x 已废弃 make）
依赖声明在 main/idf_component.yml → 首次 idf.py reconfigure 会自动下载到 managed_components/
CMakeLists.txt 通过 include(mpy.cmake) 接入 MicroPython 构建系统
首次构建流程:


# 进入容器后
cd /workspace

# ① 先让组件管理器下载 micropython-helper
idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 \
       -D MICROPY_BOARD_VARIANT=SPIRAM_OCT \
       reconfigure

# ② 确认组件已下载
ls managed_components/mr9you__micropython-helper/

# ③ 构建
idf.py build

⚠ 常见错误: 如果跳过 reconfigure 直接 build，会因为 micropython-helper 
   未下载而报 "mpy.cmake not found"。这不是你的问题，是组件管理器还没拉取依赖。


### 如果你在 Windows 上


项目路径是 `D:\HiWonder\CODE\Dog\bPuppy\`。编译需要 **ESP-IDF v5.1.2** 并已安装 ESP32-S3 工具链。如果用户还未安装，参考 §4 或建议用户使用 Docker 方案 (§3) 更可靠。

---

## 1. 项目身份

| 属性 | 值 |
|------|-----|
| 项目名 | **bPuppy** |
| 描述 | ESP32-S3 四足机器狗 (12-DOF) |
| 前代项目 | bDog (ESP32-WROOM-32D)，在同一 workspace 的 `../` 目录 |
| 芯片 | **ESP32-S3 WROOM-1 N16R8** |
| Flash | 16MB QIO 80MHz |
| PSRAM | 8MB Octal |
| 固件框架 | MicroPython v1.22.1 + ESP-IDF v5.1.2 |
| MicroPython 组件 | `mr9you/micropython-helper` v1.22.1 |
| 开发语言 | C (驱动层) + C++ (运动控制) + Python (应用层) |

| 属性 | 值 |
|------|-----|
| 项目名 | **bPuppy** |
| 描述 | ESP32-S3 四足机器狗 (12-DOF) |
| 前代项目 | bDog (ESP32-WROOM-32D)，在同一 workspace 的 `../` 目录 |
| 芯片 | **ESP32-S3 WROOM-1 N16R8** |
| Flash | 16MB QIO 80MHz |
| PSRAM | 8MB Octal |
| 固件框架 | MicroPython v1.22.1 + ESP-IDF v5.1.2 |
| MicroPython 组件 | `mr9you/micropython-helper` v1.22.1 |
| 开发语言 | C (驱动层) + C++ (运动控制) + Python (应用层) |
| 构建系统 | CMake (ESP-IDF 标准) |

---

## 2. 环境要求（铁律）

```
MicroPython v1.22.1  ←→  ESP-IDF v5.0.4 / v5.1.2  （精确匹配）
                           ❌ 不可用 v5.3 / v5.5
```

| 工具 | 版本 | 说明 |
|------|------|------|
| ESP-IDF | **v5.1.2** | 编译工具链 + SDK |
| CMake | ≥ 3.16 | ESP-IDF 自带 |
| Python | ≥ 3.8 | ESP-IDF 自带 |
| esptool | ≥ 4.0 | 烧录工具，ESP-IDF 自带 |
| Docker (推荐) | 任意 | 容器化构建，避免环境差异 |

---

## 3. 容器化构建环境（推荐）

### 3a. 使用已有 Docker 镜像

如果团队提供了 ESP-IDF 容器镜像，直接使用。镜像应包含:

- `/opt/esp/idf-v5.1.2/` — ESP-IDF v5.1.2 完整安装
- `/opt/esp/tools/` — 工具链 (cmake, ninja, xtensa-esp32s3-elf 等)

启动容器时挂载项目目录:

```bash
docker run -it --rm \
  -v /path/to/bPuppy:/workspace \
  <esp-idf-image> \
  /bin/bash
```

### 3b. 从零搭建 Docker 构建环境

如果不存在现成镜像，按以下步骤搭建:

```dockerfile
# Dockerfile.bpuppy — bPuppy ESP32-S3 构建环境
FROM ubuntu:22.04

# 系统依赖
RUN apt-get update && apt-get install -y \
    git wget flex bison gperf python3 python3-pip python3-venv \
    cmake ninja-build ccache libffi-dev libssl-dev dfu-util \
    libusb-1.0-0 curl jq && \
    apt-get clean

# 克隆 ESP-IDF v5.1.2
RUN mkdir -p /opt/esp && cd /opt/esp && \
    git clone --branch v5.1.2 --depth 1 \
    https://github.com/espressif/esp-idf.git idf-v5.1.2

# 安装 ESP32-S3 工具链
RUN cd /opt/esp/idf-v5.1.2 && \
    ./install.sh esp32s3

# 设置环境变量（每次启动时 source）
RUN echo 'source /opt/esp/idf-v5.1.2/export.sh' >> /root/.bashrc

WORKDIR /workspace
```

构建镜像:
```bash
docker build -t bpuppy-builder -f Dockerfile.bpuppy .
```

### 3c. 容器内一键编译

```bash
# 进入容器后:
cd /workspace
./build.sh
```

`build.sh` 内容:
```bash
#!/bin/bash
set -e
export IDF_PATH=/opt/esp/idf-v5.1.2
export IDF_TOOLS_PATH=/opt/esp
source /opt/esp/idf-v5.1.2/export.sh > /dev/null 2>&1
export PATH="/opt/esp/tools/cmake/3.30.2/bin:/opt/esp/tools/ninja/1.12.1/bin:$PATH"

idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 \
       -D MICROPY_BOARD_VARIANT=SPIRAM_OCT \
       build

echo "Firmware: build/micropython_bpuppy.bin"
```

---

## 4. 裸机构建环境（Windows）

### 4a. 安装 ESP-IDF

1. 下载 **ESP-IDF v5.1.2** 离线安装器: https://dl.espressif.com/dl/esp-idf/
2. 安装时选择 **ESP32-S3** 芯片支持
3. 安装路径建议: `C:\esp\idf-v5.1.2`

### 4b. 编译

打开 **ESP-IDF Command Prompt** (开始菜单)，然后:

```batch
cd D:\HiWonder\CODE\Dog\bPuppy
setup_and_build.bat
```

`setup_and_build.bat` 会自动:
1. `idf.py set-target esp32s3`
2. 下载依赖组件 (`micropython-helper` 等)
3. 构建固件

---

## 5. CMake 构建参数速查

所有必需参数都在 `CMakeLists.txt` 中设定，但也可以通过命令行覆盖:

| 参数 | 值 | 说明 |
|------|-----|------|
| `IDF_TARGET` | `esp32s3` | 芯片型号 |
| `MICROPY_BOARD` | `ESP32_GENERIC_S3` | MicroPython 板级配置 |
| `MICROPY_BOARD_VARIANT` | `SPIRAM_OCT` | Octal PSRAM + 240MHz |

等效命令行:
```bash
idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 \
       -D MICROPY_BOARD_VARIANT=SPIRAM_OCT \
       build
```

---

## 6. 构建产物

| 文件 | 说明 |
|------|------|
| `build/micropython_bpuppy.bin` | **主固件** (MicroPython + C模块 + frozen Python) |
| `build/bootloader/bootloader.bin` | 二级 bootloader |
| `build/partition_table/partition-table.bin` | 分区表 (16MB 布局) |

---

## 7. 烧录

### ESP32-S3 USB-JTAG 直连（推荐）

S3 原生 USB 同时支持烧录和串口控制台，**无需 USB-UART 转接器**。

```bash
# 完整烧录 (首次)
esptool.py --chip esp32s3 --port /dev/ttyACM0 --baud 921600 \
  write_flash \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x10000 build/micropython_bpuppy.bin

# 仅更新固件 (日常)
esptool.py --chip esp32s3 --port /dev/ttyACM0 --baud 921600 \
  write_flash 0x10000 build/micropython_bpuppy.bin
```

### Windows

```powershell
esptool --chip esp32s3 --port COMx --baud 921600 write-flash `
  0x0 bootloader.bin `
  0x8000 partition-table.bin `
  0x10000 micropython_bpuppy.bin
```

> 端口: Linux `/dev/ttyACM0`，Windows `COMx`（设备管理器中查看）

---

## 8. 串口控制台

```bash
# Linux
picocom -b 115200 /dev/ttyACM0
# 或
screen /dev/ttyACM0 115200

# Windows
# 使用 PuTTY / MobaXterm / VS Code Serial Monitor
# 波特率: 115200
```

---

## 9. 首次编译问题排查

### 问题: `idf.py: command not found`

**原因**: 未激活 ESP-IDF 环境

**解决**:
```bash
source /opt/esp/idf-v5.1.2/export.sh
# 或 Windows: 打开 "ESP-IDF Command Prompt"
```

### 问题: `CMake Error: MICROPY_BOARD not found`

**原因**: 未传递 `-D MICROPY_BOARD=ESP32_GENERIC_S3`

**解决**: 构建命令必须包含所有三个 `-D` 参数，或确认 `CMakeLists.txt` 中已设置

### 问题: `components not found / mr9you/micropython-helper`

**原因**: 首次构建时依赖组件未下载

**解决**:
```bash
rm -rf build managed_components dependencies.lock
idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 \
       -D MICROPY_BOARD_VARIANT=SPIRAM_OCT \
       reconfigure
# 等待组件管理器下载 micropython-helper
idf.py build
```

### 问题: `LEDC_HIGH_SPEED_MODE undeclared`

**原因**: 代码仍在引用 ESP32 的高低速模式，未适配 S3

**解决**: 确认 `drivers/servo_driver.c` 中使用的是统一 `LEDC_LOW_SPEED_MODE`，无 `LEDC_HIGH_SPEED_MODE` 引用

### 问题: `sdkconfig` 冲突 / Flash 大小不对

**原因**: 旧 `build/` 目录缓存了 ESP32 配置

**解决**:
```bash
rm -rf build
idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 \
       -D MICROPY_BOARD_VARIANT=SPIRAM_OCT \
       build
```

### 问题: 构建产物名不对

**原因**: `CMakeLists.txt` 中 `project()` 名称为 `micropython_bpuppy`

确认产物路径: `build/micropython_bpuppy.bin`

---

## 10. sdkconfig 三层覆盖机制（重要）

理解 sdkconfig 加载顺序有助于调试配置问题:

```
第 1 层: ESP32_GENERIC_S3 板级默认
  ├── boards/sdkconfig.base          (ESP-IDF 基础)
  ├── boards/sdkconfig.usb           (USB-JTAG 控制台)
  ├── boards/sdkconfig.ble           (BLE 支持)
  ├── boards/sdkconfig.spiram_sx     (SPIRAM 自动检测)
  └── boards/ESP32_GENERIC_S3/sdkconfig.board  (QIO, 8MB, 分区表)

第 2 层: SPIRAM_OCT variant (CMakeLists.txt 中 set)
  ├── boards/sdkconfig.240mhz        (CPU 240MHz)
  └── boards/sdkconfig.spiram_oct    (Octal PSRAM mode)

第 3 层: 项目覆盖 (CMakeLists.txt 中 list APPEND)
  ├── sdkconfig.defaults             (项目通用)
  └── sdkconfig.bpuppy               (16MB Flash + 16MB 分区表)
```

**规则**: 后加载的覆盖先加载的。第 3 层的 `CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y` 会覆盖第 1 层的 `CONFIG_ESPTOOLPY_FLASHSIZE_8MB=y`。

查看最终生效配置:
```bash
cat build/sdkconfig | grep CONFIG_ESPTOOLPY
```

---

## 11. 项目文件导航

### 如果 AI 需要修改代码，从这里开始:

| 需求 | 改哪个文件 |
|------|-----------|
| 修改舵机 GPIO 引脚 | `drivers/servo_driver.c` → `servo_init_all()` |
| 修改 IMU I2C 引脚 | `frozen/main.py` → `bpuppy_imu.init(...)` |
| 修改步态参数 | `drivers/motion_task.cpp` |
| 添加新的 MicroPython C 函数 | 对应的 `drivers/*.c` + `drivers/micropython.cmake` |
| 修改 Python 启动逻辑 | `frozen/main.py` |
| 修改 BLE 协议处理 | `frozen/ble_hiwonder.py` |
| 修改构建参数 | `CMakeLists.txt` + `sdkconfig.bpuppy` |
| 更新版本号 | `drivers/bpuppy_version.c` → `BP_VERSION` |
| 修改分区表 | `partitions.csv` (或在 `sdkconfig.bpuppy` 切换) |

### 修改后如何构建:

| 改了什么 | 构建命令 |
|---------|---------|
| `frozen/*.py` | `idf.py build` |
| `*.c` / `*.cpp` / `*.h` | `idf.py build` |
| `*.cmake` / `CMakeLists.txt` / `sdkconfig.*` | `rm -rf build && idf.py ... build` |

---

## 12. 验证清单（AI 在完成环境配置后逐项检查）

- [ ] `idf.py --version` 输出 ESP-IDF v5.1.2
- [ ] `idf.py set-target esp32s3` 成功
- [ ] `managed_components/mr9you__micropython-helper/` 存在（组件已下载）
- [ ] `idf.py build` 编译成功，exit code = 0
- [ ] `build/micropython_bpuppy.bin` 文件存在
- [ ] 烧录后 USB CDC 串口可连接 (`/dev/ttyACM0` 或 `COMx`)
- [ ] 启动 banner 显示 "bPuppy Robot Dog - ESP32-S3"
- [ ] `import bpuppy_servo; bpuppy_servo.init_all()` 无报错
- [ ] `import bpuppy_imu; bpuppy_imu.init(0, 1, 2, 0x6A)` WHO_AM_I 正确
- [ ] `import bpuppy; bpuppy.version()` 返回版本号

---

## 13. 与 bDog 的关系

bDog 在 `../` 目录（同一 workspace）。两个项目**完全独立**:

- bDog: ESP32 + `bdog_*` 模块 + 4MB Flash
- bPuppy: ESP32-S3 + `bpuppy_*` 模块 + 16MB Flash

bDog 的任何修改不影响 bPuppy，反之亦然。构建缓存 (`build/`) 和组件 (`managed_components/`) 各自独立。

---

## 附录: 常用 ESP-IDF 命令

```bash
# 清理
idf.py clean           # 清理编译产物
idf.py fullclean       # 清理所有（包括 cmake 缓存）

# 配置
idf.py menuconfig      # 图形化 Kconfig 配置
idf.py reconfigure     # 重新运行 CMake（不改 sdkconfig）

# 烧录 + 监控
idf.py flash           # 烧录 (需设 ESPPORT)
idf.py monitor         # 串口监控 (Ctrl+] 退出)
idf.py flash monitor   # 烧录并监控

# 查看项目大小
idf.py size            # 固件各组件大小
idf.py size-components # 详细组件大小

# 环境变量
export ESPPORT=/dev/ttyACM0   # 烧录端口
export ESPBAUD=921600         # 烧录波特率
```
