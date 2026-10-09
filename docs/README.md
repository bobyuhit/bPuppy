# bPuppy 机器狗

> 🌐 [English](README.en.md)

bPuppy 是一款使用普通 MG90S 舵机的八自由度四足机器狗开源教育平台，基于 ESP32-S3，整机成本控制在 200 元以内。结构极其简单，易于动手安装制作。

具备 walk、trot、自适应三种四足步态，可以完成行走、转弯等运动动作，以及站立、蹲下、坐下、挥手、邀玩等姿态动作。

- 支持 **MicroPython 编程 / REPL 指令集**
- 支持 **KittenBlock 图形化编程**（USB / 蓝牙连接）
- 支持 **WiFi 实时图传与网页遥控**
- 板载语音识别模块（CI-33T）实现离线语音交互（可选）
- 板载姿态传感器（IMU / 地磁）（可选）
- 具备扩展 **UART / IIC 接口**，扩展 **PWM 接口 x3**

本项目代码全部由 DeepSeek 生成，此简介由本人类撰写（DeepSeek 写的实在是太沙雕了）。

## 相关链接

- **代码开源**（持续升级中，含物料采购清单）—— https://github.com/bobyuhit/bPuppy
- **电路开源**（原理图 / PCB / BOM 在立创开源硬件平台，可直接下单打板）—— https://oshwhub.com/northerntree/project_fddruuyj
- **3D 打印结构开源** —— https://makerworld.com/zh/models/3409921-bpuppy-8-dof-servo-quadruped-robot-dog
- **相关视频**（持续更新中）—— https://space.bilibili.com/404024959/lists/9244762?type=season

## KittenBlock 扩展地址速查

KittenBlock 用户需要复制这一行，粘到 KittenBlock 的「URL 导入」里：

```
https://raw.githubusercontent.com/bobyuhit/bPuppy/master/bpuppy-kittenblock.zip
```

## 接下来看哪份文档

| 文档 | 给谁看 |
|---|---|
| [wifi设备遥控指南.md](wifi设备遥控指南.md) | **用手机/电脑遥控机器狗**的用户 —— 图传 + 网页遥控器 |
| [kittenblock图形化编程指南.md](kittenblock图形化编程指南.md) | **图形化编程**用户 —— 每个积木怎么用、参数被拒了怎么知道 |
| [micropython编程指南.md](micropython编程指南.md) | **使用 MicroPython 控制机器狗**的用户 —— 所有 MicroPython 接口、REPL、传 .py 上去跑 |
| [新板上电操作指南.md](新板上电操作指南.md) | **新板第一件事** —— 烧固件 → 传文件 → 标定 → 验收 |

## 工具下载

**bTool**（Windows 免安装小程序：烧固件 / 传文件 / 连终端）：

[⬇ 下载 bTool.exe](https://github.com/bobyuhit/bTool/releases/latest/download/bTool.exe)

**固件烧录文件**（3 个 .bin；烧写地址与步骤见[新板上电操作指南.md](新板上电操作指南.md)）：

| 文件 | 作用 | 烧写地址 |
|---|---|---|
| [micropython_bpuppy.bin](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/firmware/micropython_bpuppy.bin) | 应用程序，**每次都要烧** | `0x10000` |
| [bootloader.bin](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/firmware/bootloader.bin) | 引导，只有首次 / 改过 bootloader | `0x0` |
| [partition-table.bin](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/firmware/partition-table.bin) | 分区表，只有首次 / 改过分区表 | `0x8000` |

**新板要传到板子上的 3 个 .py 文件**（上传到板子根目录 / VFS，方法见[新板上电操作指南.md](新板上电操作指南.md)）：

| 文件 | 作用 |
|---|---|
| [pwm_ext_on.py](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/mpy_modules/pwm_ext_on.py) | 开机启用扩展舵机 |
| [camera_on.py](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/mpy_modules/camera_on.py) | 开机自动开网页图传 |
| [batt.py](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/mpy_modules/batt.py) | 电池标定工具 |

## 物料清单与组装教程

| 资源 | 说明 |
|---|---|
| [⬇ bPuppy物料采购清单.xlsx](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/bPuppy%E7%89%A9%E6%96%99%E9%87%87%E8%B4%AD%E6%B8%85%E5%8D%95.xlsx) | 整机物料。按「基础功能(必选)」+ 语音 / 自稳与导航 / 摄像头 / AI 视觉等「选配」分类，含数量、单价、购买备注和淘宝链接 |
| [🔧 视频教程（组装 / 演示，持续更新）](https://space.bilibili.com/404024959/lists/9244762?type=season) | B 站合集 —— 动手装配全过程 |

