# bPuppy 摄像头功能设计文档

## 概述

ESP32-S3 通过 DVP 并行接口驱动 OV2640 摄像头，提供两个独立功能：

| | 功能 1：手机实时图传 | 功能 2：色球识别 |
|---|---|---|
| **文件** | `frozen/camera_stream.py` ✅ 已完成 | `frozen/color_track.py` |
| **摄像头格式** | JPEG（硬件编码） | RGB565（原始像素） |
| **分辨率** | SVGA 800×600 | QVGA 320×240 |
| **帧率** | ~15 fps | ~25 fps |
| **运行方式** | WiFi AP 后台 + `_thread` | 主循环阻塞 |
| **输出目标** | 手机/PC 浏览器 | 串口坐标 / 舵机跟随 |
| **网络** | WiFi AP 热点 | 无 |

两个功能**不同时运行**，共用同一摄像头硬件，通过 `init()` / `deinit()` 切换。

---

## 硬件基础

### OV2640 引脚映射

| 信号 | GPIO | 说明 |
|------|------|------|
| SIOD (SDA) | 4 | SCCB 串行数据 |
| SIOC (SCL) | 5 | SCCB 串行时钟 |
| VSYNC | 6 | 帧同步 |
| HREF | 7 | 行有效 |
| XCLK | 15 | 主时钟 20MHz |
| PCLK | 13 | 像素时钟 |
| D0 (Y2) | 11 | 数据位 0 |
| D1 (Y3) | 9 | 数据位 1 |
| D2 (Y4) | 8 | 数据位 2 |
| D3 (Y5) | 10 | 数据位 3 |
| D4 (Y6) | 12 | 数据位 4 |
| D5 (Y7) | 18 | 数据位 5 |
| D6 (Y8) | 17 | 数据位 6 |
| D7 (Y9) | 16 | 数据位 7 |
| PWDN | — | 未连接 |
| RESET | — | 未连接 |

### 时钟方案

ESP32-S3 通过 **LCD_CAM 外设内部时钟分频器** 生成 XCLK：`160MHz ÷ 8 = 20MHz`。不使用 LEDC，与舵机 8 路 PWM 无冲突。

### GPIO 冲突分析

- 摄像头 13 个 GPIO (4–18) 与舵机 8 个 GPIO (1, 2, 21, 41, 42, 45, 47, 48) 完全不重叠
- IMU I2C (GPIO 1, 2) 与舵机有冲突，但 IMU 暂未启用
- XCLK 不使用 LEDC，不占用舵机的 8 个 LEDC 通道

---

## 功能 1：手机实时图传

### 数据流

```
OV2640 传感器
   │  硬件 JPEG 编码器
   │  输出: 压缩好的 JPEG 字节流
   ▼
esp_camera_fb_get()          # C 层帧缓冲
   │  camera_fb_t { .buf, .len=~28KB, .format=PIXFORMAT_JPEG }
   ▼
bpuppy_camera.capture()      # MicroPython 接口
   │  返回: (bytes, 800, 600, 4)
   ▼
socket.send()                # HTTP multipart 包装发送
   │  --boundary
   │  Content-Type: image/jpeg
   │  <JPEG bytes 原样>
   ▼
WiFi AP (192.168.4.1)
   │
   ▼
手机浏览器 → http://192.168.4.1
```

### 为什么用 JPEG

OV2640 内置硬件 JPEG 编码器，800×600 一帧约 28KB。CPU 不做任何编码工作，只负责 `socket.send()`。WiFi 带宽需求：28KB × 15fps ≈ 420KB/s，远低于 802.11n 上限。

### 为什么不用 RGB565 传图

QVGA RGB565 一帧 150KB，发送需要 150KB × 15fps ≈ 2.25MB/s，WiFi 能跑但帧率会降。且浏览器不认 RGB565 原始数据，仍需软件转 JPEG，CPU 瓶颈反而更差。

### AP 配置

| 参数 | 值 |
|------|-----|
| SSID | `bPuppy` |
| 密码 | `12345678` |
| IP | `192.168.4.1` |
| 信道 | 1 |
| DHCP | 内置，自动分配 |
| 加密 | WPA2-PSK |

### HTTP 端点

| 路径 | Content-Type | 说明 |
|------|-------------|------|
| `/` | text/html | 简单页面，`<img src="/stream">` 嵌入实时流 |
| `/stream` | multipart/x-mixed-replace | MJPEG 视频流 |

### 并发模型

HTTP accept 循环跑在 `_thread` 后台线程。主线程（REPL）不受影响，可正常输入命令。每次 accept 一个客户端连接，发送帧然后等待下一个连接。

### 帧率估算

```
摄像头 JPEG 输出:     ≈ 5ms   (DMA 硬件搬运)
socket.send():        ≈ 20ms  (28KB TCP 发送)
WiFi 传输到手机:       ≈ 30ms  (WiFi 延迟)
合计:                 ≈ 55ms  → ~18fps 上限
实际稳定帧率:          ≈ 10-15fps
```

---

## 功能 2：色球识别

### 数据流

```
OV2640 传感器
   │  RGB565 模式, QVGA 320×240
   │  输出: 76800 个像素, 每个 2 字节
   ▼
esp_camera_fb_get()
   │  camera_fb_t { .buf=150KB, .format=PIXFORMAT_RGB565 }
   ▼
像素遍历
   │  位运算提取 R/G/B
   │  例: R = (pixel >> 11) & 0x1F
   ▼
颜色阈值过滤
   │  红色球: R>20 AND G<10 AND B<10  (0-31 范围)
   │  绿色球: G>40 AND R<15 AND B<15 (0-63 范围)
   ▼
连通域分析
   │  扫描行找连续色块 → 合并相邻行 → 计算包围盒
   ▼
决策
   │  最大色块中心坐标 (cx, cy)
   │  cx < 140 → 左转
   │  cx > 180 → 右转
   │  否则     → 前进
   ▼
bpuppy_motion 控制
   │  set_turn(offset)
   │  set_gait("go")
   ▼
舵机执行
```

### 为什么用 RGB565

做颜色识别的核心操作是"读每个像素的颜色值"。RGB565 直接存储原始像素，一次位运算即可提取 R/G/B 分量。JPEG 存储的是 DCT 频率系数，必须先软件解码——ESP32-S3 上没有硬件 JPEG 解码器，CPU 软件解码一帧 VGA JPEG 约 100-200ms，加上识别算法就太慢了。

### RGB565 像素操作

```python
# 每个像素 2 字节，大端序
pixel = (buf[i] << 8) | buf[i + 1]

# 位布局: RRRRR GGGGGG BBBBB
r = (pixel >> 11) & 0x1F    # 高 5 位 = 红色 (0~31)
g = (pixel >> 5)  & 0x3F    # 中 6 位 = 绿色 (0~63)
b =  pixel        & 0x1F    # 低 5 位 = 蓝色 (0~31)
```

### 颜色标定

阈值不能写死，需要根据实际光照和色球标定。两种方式：

**方式 A — 交互式校准（推荐）：**
```python
# 运行 calibrate() → 把球放到画面中间
# 自动在球的区域采样 100 个点 → 统计均值 ± 容差
# 保存到 NVS，下次直接读
```

**方式 B — 预设常量：**
```python
# 根据不同颜色球预先测好
RED_BALL   = {"r_min": 18, "r_max": 31, "g_max": 12, "b_max": 12}
GREEN_BALL = {"r_max": 12, "g_min": 40, "g_max": 63, "b_max": 12}
BLUE_BALL  = {"r_max": 12, "g_max": 12, "b_min": 18, "b_max": 31}
```

### 色块检测算法

```
1. 下采样 (可选): 隔行隔列读，320×240 → 160×120，提速 4 倍
2. 遍历像素: 每个像素判是否在颜色范围内
3. 标记: 满足条件的像素写入 mask 位图
4. 连通域: 两遍扫描法 (Two-Pass) 标记连通区域
5. 筛选: 去掉像素数 < 阈值的小噪点
6. 输出: 最大色块的中心坐标 (cx, cy) 和包围盒 (x, y, w, h)
```

### 运动控制策略

```
色块中心 X 偏移量 = cx - 160  (画面中心 = 160)
                    -160      0        +160
                    ←←← 左    |    右 →→→

turn = 偏移量 / 160 * 系数
  例: cx=120 → turn=-0.25  (左转)
  例: cx=200 → turn=+0.25  (右转)
  例: cx=160 → turn=0      (直行)

色块面积 < 阈值 → 可能是远处 → 前进靠近
色块面积 > 阈值 → 太近了 → 后退
```

### 帧率估算

```
摄像头 RGB565 输出:      ≈ 3ms   (150KB DMA 搬运)
像素遍历 + 颜色过滤:      ≈ 8ms   (76800 像素)
连通域分析:               ≈ 15ms  (Two-Pass)
运动控制:                 ≈ 2ms   (IK + 舵机)
合计:                     ≈ 28ms  → ~35fps 上限
实际稳定帧率:              ≈ 20-25fps
```

### C 层前置改动

当前 `camera_driver.c` 的 `init_adv()` 硬编码了 `PIXFORMAT_JPEG`：

```c
// 第 63 行
.pixel_format = PIXFORMAT_JPEG,  // 只有 JPEG
```

需改为可配置：在函数签名加 `pixformat_t format` 参数，MicroPython 侧 `init_adv()` 新增第 5 个参数 `format`。

---

## 文件清单

| 文件 | 类型 | 说明 |
|------|------|------|
| `drivers/camera_driver.h` | C 头文件 | 摄像头驱动声明（已完成） |
| `drivers/camera_driver.c` | C 源码 | 摄像头驱动 + MicroPython 绑定（需加 format 参数） |
| `frozen/camera_stream.py` | Python | WiFi AP + MJPEG 图传（✅ 已完成） |
| `frozen/color_track.py` | Python | 色球识别 + 跟随（新增） |
| `frozen/main.py` | Python | 启动脚本（不变） |
| `frozen/manifest.py` | Python | 冻结模块清单（需加 2 个新文件） |

---

## 构建与烧录

仅 `camera_driver.c` 改动需要重编译。两个 Python 文件放在 `frozen/` 下，编译时自动打包进固件。

```bash
# Docker 编译
docker run --rm -v "d:/Hiwonder/CODE/dog/bPuppy:/workspace" \
  espressif/idf:v5.1.2 bash -c "source /opt/esp/idf/export.sh && cd /workspace && idf.py build"

# 烧录
esptool --chip esp32s3 --port COM14 --baud 115200 write-flash \
  0x0 build/bootloader/bootloader.bin \
  0x8000 build/partition_table/partition-table.bin \
  0x10000 build/micropython_bpuppy.bin
```

---

## 验证计划

### 功能 1 验证

```
1. import camera_stream
2. camera_stream.start()
3. 手机连接 WiFi "bPuppy" (密码 12345678)
4. 手机浏览器打开 http://192.168.4.1
5. 确认看到实时画面（~10-15fps）
6. camera_stream.stop()
7. 确认热点关闭，摄像头释放
```

### 功能 2 验证

```
1. import color_track
2. color_track.start(color="red")  # 追踪红色球
3. 串口打印: "Found at (120, 80) size=450px"
4. 移动球 → 确认坐标跟随变化
5. 连上舵机 → 确认机器人追踪球移动
6. color_track.stop()
```
