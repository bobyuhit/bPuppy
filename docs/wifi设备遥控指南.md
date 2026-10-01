# bPuppy WiFi 设备遥控指南

机器狗内置 WiFi 热点与网页遥控器：手机连接后，浏览器即可查看实时画面并遥控运动。无需路由器或外部网络。

> **本文分两部分：**「怎么用」不涉及编程；「原理与接口」面向开发者。

---

# 一、怎么用

## 1.1 三步

| 步骤 | 操作 | 频次 |
|---|---|---|
| 一 | 将 `camera_on.py` 传入机器狗 | 仅一次 → 1.2 |
| 二 | 手机连接机器狗的 WiFi | 每次 → 1.3 |
| 三 | 打开网页操控 | 每次 → 1.4 |

完成第一步后，此后每次仅需第二、三步 —— 机器狗上电即自动开启热点。

## 1.2 传入开关文件（仅一次）

出厂默认**不**自动开启 WiFi：热点与摄像头耗电，按需开启更合理。若希望每次上电自动开启，只需一步：

> 将仓库中的 `camera_on.py` 传入机器狗板载存储。

- 此后每次上电，热点与网页**自动就绪**
- 不再需要时，**删除该文件**即可，下次上电不再开启
- 不影响已下载的用户程序，无需重新烧录固件

**传输步骤：**

> 📖 见《[新板上电操作指南](新板上电操作指南.md)》**第 4 节「传文件」** ——
> 所用工具、连线方式、操作按钮，以及传输时的三项限制。

## 1.3 连接 WiFi（每次）

1. 手机 WiFi 设置中连接 **`bPuppy_XXXX`**（`XXXX` 为设备标识，每台不同）
2. 密码 **`12345678`**

> 手机可能提示「此 WiFi 无法访问互联网」，属正常现象（本热点不接入外网），选择保持连接即可。

## 1.4 打开网页（每次）

多数手机连接后**自动弹出**遥控页；未弹出时，用浏览器访问：

```
192.168.4.1
```

## 1.5 界面说明

| 控件 | 功能 |
|---|---|
| 速度滑块 | 调整步频，数值越大越快 |
| 方向键（8 向） | 控制行进方向 |
| 停 | 立即停止并站好 |
| 站立 / 坐下 / 蹲下 | 切换姿态 |
| 玩 | 邀玩动作（前低后高 + 摇臀） |
| 挥手 | 坐下并挥右前爪 3 次 |
| 图传 开 / 关 | 仅切换画面，不影响遥控 |

## 1.6 常见情况

| 情况 | 处理 |
|---|---|
| 仅关闭画面以省电 | 网页点「图传 关」 |
| 不再自动开启 | 删除板上的 `camera_on.py`，重新上电 |
| 立即彻底关闭 | 删文件并重新上电（仅点「图传 关」不会停热点）；或见 2.2 |
| 搜不到 `bPuppy_XXXX` | 依次确认：设备已上电 → `camera_on.py` 已传入（1.2）→ 手机 WiFi 重开后再搜 |
| 已连接但网页打不开 | 手动访问 `192.168.4.1`；无效则重新上电 |

---

# 二、原理与接口

> 面向需要**编写代码或改动本功能**的开发者。只想使用的话，上面已足够。

## 2.1 速查（可直接执行）

```python
import camera_stream
import bpuppy_motion
bpuppy_motion.set_gait("stop")     # 先停止站好 (start 不会自动停, 避免狗继续运动)
camera_stream.start(stream=True)   # 开热点 + 视频流 + 网页遥控 → http://192.168.4.1
# camera_stream.start()            # 纯遥控, 不开图传 (默认)
camera_stream.stop()               # 关: 热点关闭, 摄像头释放
camera_stream.state()              # 查看状态: "running" / "stopped"
```

## 2.2 接口

| 调用 | 说明 |
|---|---|
| `camera_stream.start(stream=False, ssid=…, password=…)` | 开启热点并启动 HTTP 服务。`stream=True` 才初始化摄像头（SVGA 800×600 JPEG）；默认仅提供遥控页 |
| `camera_stream.stop()` | 关闭热点并释放摄像头（自动 `deinit()`） |
| `camera_stream.state()` | 返回 `"running"` / `"stopped"` |

> `start(stream=True)` 内部自动调用 `init_adv()`，`stop()` 自动 `deinit()` ——
> 图传与拍照互切时**无需手动管理摄像头**。
> 摄像头本身的 API（分辨率 / 格式 / 拍照）见 [micropython编程指南.md](micropython编程指南.md)。

## 2.3 上电开关文件机制

1.2 中传入的 `camera_on.py`，是**「上电开关文件」**机制的一个实例：

```python
# /camera_on.py 的全部内容
import camera_stream
camera_stream.start(stream=True)     # 只想开网页不开相机 → 去掉 (stream=True)
```

`frozen/main.py` 每次开机扫描板子根目录下的固定文件名：**文件存在即执行，不存在则跳过**。
因此**文件名本身就是开关** —— 无需重编译固件、无需敲命令，下次上电即生效。

> 通用规则（共有哪几个开关文件、放置位置、为何**必须为纯 ASCII**、
> 与 KittenBlock 下载的 `/main.py` 为何互不干扰）见
> [micropython编程指南.md](micropython编程指南.md) 的「**2.2 上电开关文件**」节。

## 2.4 自定义热点名 / 密码

```python
import camera_stream
camera_stream.start(ssid="MyDog", password="mypassword", stream=True)
```

## 2.5 图传 ↔ 拍照互切

两者均独占摄像头，同一时间只能用其一：

```python
# 图传 → 拍照
camera_stream.stop()
import camera_serial
camera_serial.snap()

# 拍照 → 图传
import camera_stream
camera_stream.start(stream=True)
```

## 2.6 实现要点

- **热点 IP 固定为 `192.168.4.1`**；手机连接后自动弹窗依赖 Captive Portal，无需手动输入网址
- **图传为 SVGA 800×600 JPEG**，约 10~15 fps。**按需开启**是刻意取舍 —— 热点与摄像头均耗电
- 连接后**无外网**（热点由机器狗自身提供），系统提示「无法访问互联网」可忽略
