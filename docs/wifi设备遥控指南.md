# bPuppy WiFi 设备遥控指南

给**用手机 / 电脑连狗**的人：MJPEG 实时图传 + 网页遥控器。热点名 `bPuppy_XXXX`。

> 摄像头本身的 API（分辨率 / 格式 / 拍照）在 [micropython编程指南.md](micropython编程指南.md)；
> 「上电自动开」那套开关文件的通用规则也在那份里 —— 本文只讲**怎么用**。
> 硬件接线、摄像头引脚看 [硬件连接.md](../PCB/硬件连接.md)。

---

## 〇、速查


```python
import camera_stream
import bpuppy_motion
bpuppy_motion.set_gait("stop")     # 先停止站好 (start 不会自动停, 避免狗继续运动)
camera_stream.start(stream=True)    # 开启热点 + 视频流 + 网页遥控 → http://192.168.4.1
# camera_stream.start()             # 纯遥控, 不开图传 (默认)
camera_stream.stop()                # 关闭: 热点关闭, 摄像头释放
camera_stream.state()               # 查看状态: "running" / "stopped"
```

> **想上电就自动开？** 把上面两行放进板子的 `/camera_on.py`（源文件 `mpy_modules/camera_on.py`）。
> **文件在 = 开，删掉 = 不开**，不用重编译、不影响 KittenBlock 下载的 `/main.py`。见下面 §二。


## 一、启动与停止

`camera_stream.py` 同时提供两个功能：MJPEG 实时视频流 + 网页遥控器。

**启动:**

```python
import camera_stream
import bpuppy_motion
bpuppy_motion.set_gait("stop")     # 先停止站好, 再开图传 (start 不会自动停)
camera_stream.start(stream=True)    # stream=True 开图传; 默认 start() 纯遥控不开图传
# → 串口打印: camera_stream: started — http://192.168.4.1
```


## 二、上电自动开（开关文件）

推荐：把 `mpy_modules/camera_on.py` 传到板子根目录，之后**每次上电自动**开「网页 + 图传」。

```python
# /camera_on.py 的全部内容
import camera_stream
camera_stream.start(stream=True)     # 只开网页不开相机 → 去掉 (stream=True)
```

**文件在 = 开，删掉 = 关**，不用重编译固件、也不影响 KittenBlock 下载的 `/main.py`。

> 这类「上电开关文件」的**通用规则**（还有哪些、放哪、为什么必须纯 ASCII、和 `/main.py` 的关系）
> 见 [micropython编程指南.md](micropython编程指南.md) 的「上电开关文件」节。

## 三、连上热点 + 网页遥控



> 上电**不会**自动开热点 —— 热点和摄像头都费电，多数时候用不到，所以设计成按需开。

ESP32-S3 开启 WiFi 热点（热点名 `bPuppy_XXXX`，XXXX=MAC 后四位）。`start(stream=True)` 才初始化摄像头（SVGA 800×600 JPEG）。**运行时可在网页点「图传 开/关」切换**，遥控始终可用。

**手机/PC 连接:**

| 步骤 | 内容 |
|------|------|
| 1. 打开 WiFi | 搜索 `bPuppy_XXXX` 热点 (XXXX = MAC 后四位) |
| 0. 自动弹窗 | 连上后手机会**自动弹出遥控页** (Captive Portal, 无需手动输网址) |
| 2. 输入密码 | `12345678` |
| 3. 打开浏览器 | 输入 `http://192.168.4.1` |
| 4. 看到画面 | 实时视频流 (~10-15fps) + 遥控界面 |

**网页遥控器功能:**

| 控件 | 操作 | 效果 |
|------|------|------|
| 速度滑块 | 拖动 0~10 | 调步频 |
| 方向按钮 (8方向) | 点击 | 转向 + 前进/后退 |
| 停 | 点击 | 立即停止站好 |
| 站立/坐下/蹲下 | 点击 | 切换姿态 |
| 玩 | 点击 | 邀玩动作 (前低后高+摇臀) |
| 挥手 | 点击 | 坐下 + 右前膝摆动 3 次 |
| 图传 开/关 | 点击 | 运行时开关视频流 (页面刷新生效) |

**停止:**

```python
import camera_stream
camera_stream.stop()
# → 串口打印: camera_stream: stopped
# 热点关闭, 摄像头释放
```

**切换摄像头用途:**

```python
# 图传 → 拍照
camera_stream.stop()
import camera_serial
camera_serial.snap()

# 拍照 → 图传
import camera_stream
camera_stream.start(stream=True)
```

`start(stream=True)` 会自动 `init_adv()` 初始化摄像头，`stop()` 会自动 `deinit()` 释放。两个功能互相切换无需手动管摄像头。

**查看状态:**

```python
import camera_stream
camera_stream.state()  # → "running" 或 "stopped"
```

**自定义热点:**

```python
import camera_stream
camera_stream.start(ssid="MyDog", password="mypassword", stream=True)
```

