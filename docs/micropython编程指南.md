# bPuppy MicroPython 编程指南

> 🌐 [English](micropython-guide.en.md)

给**写代码的人**：在 REPL 里敲，或写 `.py` 传到板子上跑。**所有 MicroPython 接口都在这份里**。

> - 用积木的 → [kittenblock图形化编程指南.md](kittenblock图形化编程指南.md)
> - 连 WiFi 图传 / 网页遥控 → [wifi设备遥控指南.md](wifi设备遥控指南.md)
> - 刚拿到板子（烧固件 / 标定 / 验收）→ [新板上电操作指南.md](新板上电操作指南.md)
> - 硬件接线 → [硬件连接.md](../PCB/硬件连接.md)　原理与源码全貌 → [AGENTS.md](../AGENTS.md)

**怎么把 `.py` 传到板子上**：见 [新板上电操作指南.md](新板上电操作指南.md) 的 bTool「文件管理」页。
串口 / REPL 的连接参数（**115200**、CH343、走 UART0）见 [AGENTS.md](../AGENTS.md) 的「串口连接」。

---

## 目录

- [〇、速查](#〇速查)
  - [① 常用运动组合](#-常用运动组合)
  - [③ 站立自平衡 + 惯导读数](#-站立自平衡--惯导读数)
  - [⑥ 持久化参数（NVS 几何 / 重心）](#-持久化参数nvs-几何--重心)
- [一、运动](#一运动)
  - [1.1 常用运动组合](#11-常用运动组合)
  - [1.2 运动指令 bpuppy_motion](#12-运动指令-bpuppy_motion)
    - [1.2.1 运行模式（自动切换）](#121-运行模式自动切换)
    - [1.2.2 参数校验（超限拒绝）](#122-参数校验超限拒绝)
  - [1.3 站立自平衡 balance](#13-站立自平衡-balance)
- [二、通信](#二通信)
  - [2.1 串口通信 bpuppy_uart](#21-串口通信-bpuppy_uart)
  - [2.2 上电开关文件](#22-上电开关文件)
- [三、视觉](#三视觉)
  - [3.1 摄像头操作 bpuppy_camera](#31-摄像头操作-bpuppy_camera)
- [四、设置与校准](#四设置与校准)
  - [4.1 舵机校准 bpuppy_servo](#41-舵机校准-bpuppy_servo)
  - [4.2 持久化参数（★NVS，掉电保留）](#42-持久化参数nvs掉电保留)
- [五、传感器与姿态](#五传感器与姿态)
  - [5.1 IMU 调试 bpuppy_imu](#51-imu-调试-bpuppy_imu)
  - [5.2 IMU 校准（陀螺 + 加速度计）](#52-imu-校准陀螺--加速度计)
  - [5.3 磁力计校准 calib_mag](#53-磁力计校准-calib_mag)
  - [5.4 航向锁定 heading_anchor](#54-航向锁定-heading_anchor)
  - [5.5 电池电压 bpuppy_adc](#55-电池电压-bpuppy_adc)
  - [5.6 扩展舵机 PWM_EXT](#56-扩展舵机-pwm_ext)
- [六、PC 端工具](#六pc-端工具)
  - [6.1 串口拍照 capture.py](#61-串口拍照-capturepy)
- [附录 · 编译 / 烧录 / 串口](#附录--编译--烧录--串口)

## 〇、速查

每块都能**独立拷贝就跑**（已含全部 import）。上电默认站姿待命（POSE 模式）。


### ① 常用运动组合


```python
import bpuppy_motion
# 最快
bpuppy_motion.set_params(70, 10, 60)      # 步长/抬腿/身高 (一次设完, 顺序无所谓)
bpuppy_motion.set_speed(8.5)              # 步频
bpuppy_motion.set_gait("trot")

# 这也不错
bpuppy_motion.set_params(50, 10, 70)
bpuppy_motion.set_speed(8.5)
bpuppy_motion.set_gait("trot")

# 最稳
bpuppy_motion.set_params(70, 30, 70)
bpuppy_motion.set_speed(2.5)
bpuppy_motion.set_gait("walk")
```

> **`set_params(步长, 抬腿, 身高)` 三个值一次判、一次写**，返回值 `True`/`False` 告诉你写没写进去。
> 以前抬腿归另一个函数 `set_lift`，两个函数互相拿对方的**旧值**校验，于是有顺序要求
> （先 `set_lift` 再 `set_params`）—— 现在合并成一个调用，那个坑没有了。`set_lift` 已删除。
>
> 速度是**步频**，跟腿的运动轨迹无关，所以单独一个 `set_speed(0~10)`，任何时候都能单独改，
> 不动步长/抬腿/身高。同样返回 `True`/`False`。
>
> ⚠ **升固件后必须重新下载程序**：旧程序里的 `set_lift(...)` 会报 `AttributeError`，
> 旧写法的 `set_params(速度, 步长, 身高)` 参数会错位。


### ③ 站立自平衡 + 惯导读数

```python
import bpuppy_imu
bpuppy_imu.init(0, 14, 21, 0x68)      # ① 先初始化 IMU (AHRS 需先收敛, 上电初期可能需重试)

import time
time.sleep_ms(2000)                        # ② 等姿态稳定 (AHRS 收敛)

import balance
balance.start()                      # ③ 三选一 (自动急停)
# balance.start(kp=0.06, ki=0.001, kd=0.5)       # 自定义 PID
balance.stop()                       # 停止, 恢复运动任务
```

**辅助 · 循环输出惯导数据:**

```python
import bpuppy_imu
bpuppy_imu.init(0, 14, 21, 0x68)      # 初始化 (幂等)
import time
for i in range(20):
    r, p, y = bpuppy_imu.read_angles()
    print('roll=%.2f pitch=%.2f yaw=%.2f' % (r, p, y))
    time.sleep_ms(500)
```

**辅助 · 磁力计数据输出:**

```python
import bpuppy_imu
bpuppy_imu.init(0, 14, 21, 0x68)      # 初始化 (幂等)
import time
while True:
    a, g, m, t = bpuppy_imu.read_raw()
    print('mag=%.3f %.3f %.3f uT' % (m[0], m[1], m[2]))
    time.sleep_ms(200)
```

### ⑥ 持久化参数（NVS 几何 / 重心）


```python
import bpuppy_motion
bpuppy_motion.cal_ik(40, 45)                     # 腿长 (L1, L2 mm)
bpuppy_motion.set_body_dims(62.5, 59)            # 髋距 (前后半距, 左右半宽 mm)
bpuppy_motion.set_joint_limits(0, 180, 10, 170)  # 限位 (髋min/max, 膝min/max deg)
bpuppy_motion.set_center(0)                      # 重心偏移 (正=脚前移)；还要跟 stride/height 组合够得着, 默认参数下只到 ±11.4mm
bpuppy_motion.show_geometry()                    # 查看全部
```


## 一、运动

### 1.1 常用运动组合

三个实测最优方案（完整代码见上面的速查①，拷过来就能跑）：

| 方案 | speed | stride | height | lift | gait |
|------|-------|--------|--------|------|------|
| 最快 | 8.5 | 70 | 60 | 20 | trot |
| 这也不错 | 8.5 | 50 | 70 | 30 | trot |
| 最稳 | 2.5 | 70 | 70 | 30 | walk |

> 关机前先 `poses.crouch()` 让舵机放松。

### 1.2 运动指令 bpuppy_motion

**函数（调用即用）:**

| 调用示例 | 说明 |
|------|------|
| `bpuppy_motion.start()` | 启动 50Hz 运动任务（自动加载几何参数） |
| `bpuppy_motion.set_gait("walk")` | 切换步态（见下方步态列表） |
| `bpuppy_motion.set_gait("stop")` | 停止站好 (GAIT_STOP, 留运动模式) |
| `bpuppy_motion.get_mode()` | 查询运行模式 (0=IDLE, 1=POSE, 2=MOTION) |
| `bpuppy_motion.show_geometry()` | 查看全部持久化 + 运行时参数 |
| `bpuppy_motion.load_geometry()` | 从 NVS 重新加载持久化参数 |

**运行时参数（掉电丢失，需每次设置）：**

| 调用示例 | 默认值 | 说明 |
|------|--------|------|
| `bpuppy_motion.set_params(70, 30, 70)` | 70, 30, 70 | (stride, lift, height): **步长 / 抬腿高度 / 站立高度**。stride 只表**幅度**（≥0，0 = 原地踏步），**方向另设** —— 给负数会被拒。三个值一起校验一起写，返回 `True`/`False` |
| `bpuppy_motion.set_direction(1)` | 1 | **方向**：`+1` = 前 / `-1` = 后。与步长/速度/步态**完全解耦** —— 改方向不会碰步长的幅度，改步长也不会让狗掉头。给 `0` 或 `NaN` 被拒。返回 `True`/`False` |
| `bpuppy_motion.set_speed(2.5)` | 2.5 | **步频** 0~10，与腿部轨迹无关，可单独改。返回 `True`/`False` |
| `bpuppy_motion.set_omega(2.0)` | 2.0 | 基准角频率 (rad/s), 调步态快慢 |
| `bpuppy_motion.set_body_pose(0, 0)` | 0, 0 | **(俯仰, 横滚)** 身体倾角 (deg)。**纯手动设定，无 IMU 反馈**；go/walk/trot 三个步态都生效，设了就保持。**够不着时整条被拒**（俯仰横滚一个都不写、保持原值），返回 `True`/`False`。默认参数下 \|俯仰\| ≤ 3.7°、\|横滚\| ≤ 6.2° —— 想压更大先减小步长 |
| `bpuppy_motion.set_turn(0)` | 0 | 转弯率 -1~+1。超出 ±1 **会被钳到 ±1**（仍写入），此时返回 `False` |

> **运动中改 站高 / 抬脚 / 俯仰 / 横滚 / 重心 是平滑过渡的** —— 每帧走一小步，典型变化 0.3 秒到位，
> 不会一帧跳过去。静止时改则直接生效（那时另有「每帧最多 3°」的限制器兜着）。
>
> `set_lift()` 已删除（并入 `set_params` 第 2 参）；`set_body_pose` 的第 3 参 `yaw` 是死字段，已删除。
> 读回用 `get_params()`，顺序是 `(speed, stride, height, lift, omega, turn, gait, direction)`
> —— 注意跟 `set_params(stride, lift, height)` 的**参数顺序不一样**。
> `[1]` 是**带符号**的步长（= 幅度 × 方向，负号表示后退），`[7]` 是方向本身。
>
> ⚠ **`go` 步态会接管 stride/lift/height。** 走 go 时步长、站高、抬脚由 speed 自己算
> （speed≤4 → 步长 70 / 站高 70 / 抬脚 30；speed≥6 → 步长 50 / 站高 70 / 抬脚 **5**；中间线性插值）。
> 用户设的三个值**会被接受并保存、但不参与 go 的运动** —— 之后切到 walk / trot 立刻按它们走。
> 于是：四个方向块（前进/后退/左转/右转）用的都是 go ⇒「运动参数」积木对它们**没有影响**，
> 想让「运动参数」起作用得先用「切换步态」选 walk 或 trot。
> **方向不受 go 影响** —— 它是独立参数，三种步态一视同仁。

**实测最优参数:**

| 步态 | speed | stride | height | lift | pose | 备注 |
|------|-------|--------|--------|------|------|------|
| walk | 2.5 | 70 | 70 | 30 | pitch=-5 | 猫步, 稳而安静 |
| trot | 8~8.5 | 50~70 | 70 | 30 | pitch=-3 | 小跑, 最快 |

> 降低身高 (height<70) 时需减小抬腿 (lift): height=60 → lift≤20, height=50 → lift≤10, 否则髋角超限被参数校验拒绝。

**不管哪种步态，起步前都是同样四件事：**

```python
bpuppy_motion.set_gait("go")             # ① 选步态
bpuppy_motion.set_direction(1)           # ② 定方向 (+1 前 / -1 后, 上电默认 +1)
bpuppy_motion.set_speed(2.5)             # ③ 定步频 0~10
bpuppy_motion.set_params(70, 30, 70)     # ④ 高级参数三兄弟: 步长幅度 / 抬脚高度 / 站立高度
```

四者**互不干扰**：改方向不会碰步长的幅度，改步长不会让狗掉头，改速度不影响步态。
`go` 是唯一的例外 —— 它的步长/站高/抬脚由 `speed` 自己推算，`set_params` 设的值
**会被接受并保存、但不参与 go 的运动**（切到 walk / trot 立刻生效）。
但**方向仍然独立**：go 一样听 `set_direction`，不再靠步长的正负号决定前后。

**步态列表:**

| 调用示例 | 说明 |
|-----------|------|
| `bpuppy_motion.set_gait("stop")` | 停止站好 (GAIT_STOP, 高度随参数) |
| `bpuppy_motion.set_gait("walk")` | 猫步 (方向由 `set_direction` 定；speed 只管步频) |
| `bpuppy_motion.set_gait("trot")` | 小跑 (方向由 `set_direction` 定；speed 只管步频) |
| `bpuppy_motion.set_gait("go")` | 自适应 (推荐): speed≤4→walk, speed≥6→trot, 4~6 全参数插值。★ go 下 **stride/lift/height 三个值全部无效**，只有 speed 管用（见下方警告）|
| `poses.crouch()` | 蹲伏 (姿态模式, 固定角度) |
| `poses.sit()` | 猫坐 (姿态模式) |
| `poses.play()` | 邀玩 (前低后高 + 4Hz摇臀8次) |
| `poses.wave()` | 挥手 (坐下 + 右前膝摆动 3 次, 完成回坐) |
| `poses.stand()` | 站立 (姿态模式 POSE_STAND, 固定高度) |

> **未知步态名 → 停车，这是刻意的 fail-safe，不是缺陷。**
> `set_gait()` 内部 `gait_type_t g = GAIT_STOP;` 是初值，名字不匹配任何一项时不做赋值，
> 于是直接落到 STOP。所以拼错一个字母（比如 `"tr"`）= 狗原地停下。
> 这样设计是因为**指令可能真的会损坏** —— 例如蓝牙传中文/长串时会整段丢字节
> （蓝牙传含中文的代码会整段丢字节，`"trot"` 可能传成 `"tr"`），此时停车是唯一安全的结局，好过拿损坏的字符串去猜步态。
> 从 v3.0-13 起，未知名字会在 REPL 打印 `⚠ 未知步态 "xxx" → 停车`，行为不变，只是不再静默。
>
> ⚠ 旧别名 `"walkfwd"` / `"walkbck"` / `"trotfwd"` / `"trotbck"` **已删除**，现在会走上面的兜底（停车 + 警告）。
> 它们本来就不该存在 —— 四个名字映射到同样两个步态，方向全靠**当时的步长符号**，所以
> `set_gait("walkbck")` 在前进步长下照样往前走。方向现在用 `set_direction(±1)` 显式设。

### 1.2.1 运行模式（自动切换）

系统有**三种运行模式**，由 C 层状态机自动切换，**无需手动操作**：

| 模式 | 值 | 说明 |
|------|-----|------|
| IDLE | 0 | 上电未初始化（短暂） |
| POSE | 1 | 姿态模式，Python 接管舵机（蹲/坐/站立/自定义） |
| MOTION | 2 | 运动模式，C 层 50Hz 控制（go/walk/trot/stop） |

**自动切换规则：**
- **进 MOTION**：`set_gait('go'/'walk'/'trot'/'stop')` → 自动切运动模式
- **进 POSE**：Python 写舵机（`set_angle`/`group_commit`/`cal`）→ 自动切姿态模式
- 查询当前模式：`bpuppy_motion.get_mode()`

**停止（GAIT_STOP）vs 站立（POSE_STAND）：**

| | `set_gait('stop')` | `poses.stand()` |
|---|---|---|
| 模式 | MOTION | POSE |
| 控制器 | C 层 IK | Python IK |
| 高度 | 随 `set_params` 高度 | 固定 70mm |
| 用途 | 停止站好 | 姿态区站立 |

### 1.2.2 参数校验（超限拒绝）

`set_params` 设置参数时自动校验，**超限/干涉则整组拒绝写入并保持原值**（REPL 输出提示，返回值 `False`）。
**速度不参与几何判断**（仅 0~10 范围检查），它由独立的 `set_speed` 设置。

| 参数 | 判断条件 | 拒绝原因 |
|------|---------|---------|
| **速度** | 0 ~ 10 | 超范围 |
| **步长** | \|stride\| ≤ 2×body_half_l×0.85 ≈ **106mm** | 前后脚干涉 |
| **高度** | 名义 `height ≥ 15mm`，但**实际先卡在髋角**（默认参数下 ≈ **20.6mm**）| 过低腿折叠干涉 / 髋角超限 |
| **抬腿** | height - lift ≥ 0 | 抬腿过度（足端到髋上方） |
| **髋/膝角** | 遍历足端实际摆动轨迹，任一点髋∈[0,180]° 膝∈[10,170]° | 超限（含抬腿最高点） |

**足端轨迹遍历**（髋/膝角核心判断），采样 `ease = 0, 0.25, 0.5, 0.75, 1.0`：

```
x = -|stride|/2 + |stride|·ease + center_offset
z = height - lift·sin(ease·π)
```

对 4 条腿（左/右 × 前/后）IK 解算，检查髋/膝角是否在限位内，**任一点超限即拒绝**。

> 判断只涉及 **stride / height / lift 三个参数**，速度单独做 0~10 范围检查，不参与几何判断。
> 降低身高时需减小抬腿（如 height=60→lift≤20，height=50→lift≤10），否则髋角超限被拒。校验日志显示具体哪条腿和角度。
> 默认参数：speed=2.5, stride=70, height=70, lift=30。

### 1.3 站立自平衡 balance

绕过运动任务，直接舵机控制。增量式 PID，50Hz 闭环。`start()` 会自动将磁力计融合切为「只修 yaw」模式（`set_mag_fusion(False)`），保证 roll/pitch 水平准确，`stop()` 时恢复。

> ⚠ `start()` 会自动初始化 IMU，但**刚 init 时 AHRS 未收敛就记录零点，可能导致自平衡发散**。建议**先手动初始化 IMU 并等姿态稳定**再启动：

```python
import bpuppy_imu
bpuppy_imu.init(0, 14, 21, 0x68)      # 先初始化 IMU (上电初期可能需重试)

import time
time.sleep_ms(2000)                        # 等 AHRS 收敛

import balance
balance.start()                                  # ← 三选一 (默认参数)
# balance.start(kp=0.06, kd=0.2)                 # 自定义 PD
# balance.start(kp=0.06, ki=0.001, kd=0.5)       # 自定义 PID
```

| 参数 | 默认 | 含义 |
|------|------|------|
| kp | 0.06 | 比例增益 |
| ki | 0.0 | 积分增益 |
| kd | 0.43 | 微分增益 |
| deadband | 0.5 | 死区 (°) |
| max_body | 30.0 | 身体倾角上限 (°) |
| height | 60.0 | 目标中心高度 (mm) |

**停止:**

```python
import balance
balance.stop()
```

> balance 的 `start()` 会通过写舵机自动切入姿态模式（POSE），**Ctrl-C 中断后**需手动 `balance.stop()`（内部 `set_gait("stop")` 恢复运动）恢复。

**串口输出:**

```
IMU P+0.1 R-2.4 | Err Pe+0.0 Re+0.0 | Inc iP+0.00 iR+0.00 | Tar Pt+0.0 Rt+0.0
```

| 字段 | 含义 |
|------|------|
| IMU P/R | IMU 姿态角 (Pitch/Roll) |
| Err Pe/Re | 误差 (初始−当前) |
| Inc iP/iR | 本帧补偿增量 |
| Tar Pt/Rt | 目标身体倾角 |

---


## 二、通信

### 2.1 串口通信 bpuppy_uart

**UART2 (默认通信口)** — GPIO 20=RX, 19=TX，接 CI-33T 语音模块 / micro:bit。⚠ 2026-08-19 起**引脚反转**（TX=GPIO19/RX=GPIO20），原因见 [硬件连接.md](../PCB/硬件连接.md) UART2 节。

```python
import bpuppy_uart
bpuppy_uart.init(2, 19, 20, 115200)   # UART2, TX=19, RX=20
bpuppy_uart.send(b"hello")            # 发送 bytes
bpuppy_uart.sendline("AT")            # 发送字符串+换行
data = bpuppy_uart.read(64)           # 读取 bytes (非阻塞)
n = bpuppy_uart.any()                 # 缓冲字节数
```

**UART1 (摄像头复用口)** — GPIO 4=TX, 5=RX，**与摄像头 SCCB SDA/SCL 共享引脚，不拍照时可用**。

```python
import bpuppy_uart
bpuppy_uart.u1_init(4, 5, 115200)      # TX=4, RX=5
bpuppy_uart.u1_send(b"hello")
bpuppy_uart.u1_sendline("AT")
data = bpuppy_uart.u1_read(64)
n = bpuppy_uart.u1_any()
```

| 调用示例 | 说明 |
|------|------|
| `bpuppy_uart.init(2, 19, 20, 115200)` | 初始化 UART2 (num, tx, rx, baud) |
| `bpuppy_uart.send(b"hello")` | 发送 bytes 或 str |
| `bpuppy_uart.read(64)` | 读取最多 64 字节 (非阻塞) |
| `bpuppy_uart.any()` | 缓冲中可读字节数 |
| `bpuppy_uart.sendline("AT")` | 发送字符串 + CRLF |
| `bpuppy_uart.stop()` | 停止 UART2 (释放外设, 可重新 init) |
| `bpuppy_uart.u1_init(17, 18, 115200)` | 初始化 UART1 (摄像头复用脚) |
| `bpuppy_uart.u1_send(b"hello")` / `u1_read(n)` / `u1_any()` / `u1_sendline(s)` | UART1 对应操作 |
| `bpuppy_uart.u1_stop()` | 停止 UART1 (释放外设, 可重新 init) |

> 两者均**手动初始化**，上电不自动启动（避免与摄像头抢引脚）。

**语音控制（CI-33T 模块，上电默认开启）** — 由 `frozen/voice.py` 实现，`import voice` 即自动启动 UART2（**波特率 9600**）并后台轮询。CI-33T 接线：**PA2(TX)→GPIO20(UART2 RX)、PA3(RX)←GPIO19(UART2 TX)**、波特率 9600（详见 [硬件连接.md](../PCB/硬件连接.md)）。

> ⚠ **GPIO19/20 = USB_D-/USB_D+，必须关闭 TinyUSB 才能当 UART2 用**（2026-08-19 实测根因）：
> ESP32-S3 的 GPIO19=USB_D-、GPIO20=USB_D+ 是**原生 USB 引脚**。MicroPython 组件默认启用 **TinyUSB CDC**（`mpy_startup.c` 的 `usb_init()` → `tinyusb_driver_install()`），启动即初始化 USB-OTG PHY，**接管 GPIO19/20**——表现为：
> - `machine.Pin(19)` 驱动无效、UART2 TX 从 GPIO19 发不出波形（示波器看不到）
> - 实测引脚电平 = USB 空闲态：**D-(19) 低、D+(20) 高**
> - 下行（GPIO20 收）偶尔能通（输入路径幸存），上行（GPIO19 发）彻底失效
>
> **修复（固件定制）**：`components/mr9you__micropython-helper/mpy_startup.c` 中注释掉 `usb_init()` 调用，跳过 TinyUSB 初始化，GPIO19/20 回归普通 GPIO 归 UART2 用。**USB-CDC 虚拟串口因此不可用，这不是可以打开的功能** —— 两个脚已经给了语音模块，想恢复 USB 串口就得放弃 UART2（REPL/烧录走 UART0=COM14，BLE 走射频，均不受影响）。

协议：**上行** = `AA 55 <CMD> <PARAM> 55 AA`；**下行** = **4 字节定长帧，两种**
（2026-09-26 改，此前下行是裸 2 字节 `<CMD> <PARAM>`）。

**下行（CI-33T → 本系统）** — 两种帧，靠**首字节取值范围**区分：

| 帧 | 格式 | 说明 |
|---|---|---|
| **命令帧** | `BB <CMD> <PARAM> EE` | 运动/姿态指令，CMD 见下表；PARAM 预留（目前恒 `00`） |
| **角度帧** | `<角度> 00 00 00` | **声源角度（DOA）**，0–180 度。⚠ 模块侧格式固定，本仓库不能改 |

判据只有首字节一个字节：`0xBB`(=187) → 命令帧；`≤ 0xB4`(=180) → 角度帧。
**两段取值不重叠**（角度不可能是 187），所以不可能互相误判。

命令帧的 CMD 字节（`0x30`–`0x3F`；`0x3D`/`0x3E` 暂未使用）：

| CMD | 动作 | CMD | 动作 |
|--------|------|--------|------|
| `0x30` | 停止 | `0x38` | 站立 |
| `0x31` | 前进 | `0x39` | 蹲下 |
| `0x32` | 后退 | `0x3A` | 坐下 |
| `0x33` | 左转 | `0x3B` | 摇手 |
| `0x34` | 右转 | `0x3C` | 邀玩 |
| `0x35` | 加速 | `0x3D` | （未用） |
| `0x36` | 减速 | `0x3E` | （未用） |
| `0x37` | 点头 | `0x3F` | 播报电压 ★固件内置动作 |

voice.py 收到即打印 `VOICE RX: <hex>`，然后按帧结构切分（`voice._parse`）：
命令帧派发事件、角度帧存进 `voice.SoundAngle`。**收不够 4 字节就留着等下一轮**，
所以帧被串口切分或粘连都不影响解析。

> ⚠ **这次改动修掉了一个真 bug（2026-09-26 前）**：旧版是**逐字节扫描**命令码段，
> 而角度帧 `<角度> 00 00 00` 的第一字节就是角度值 —— **角度 48–63 度时低字节正好落在
> 扫描段内，会被误当成运动指令执行**（53° → `0x35` = 加速，48–63 度覆盖整个 `0x30`–`0x3F` 段）。
> 现在按帧结构判定，从根上没有这个问题。判决依据是首字节取值范围不重叠，不靠时序、
> 不靠长度、不靠延时。

**上行（本系统 → CI-33T）** — 发声/反馈，帧 = `AA 55 <CMD> <PARAM> 55 AA`：

| 帧内容 | 含义 |
|--------|------|
| `AA 55 70 00 55 AA` | 狗叫声 0 号 |
| `AA 55 70 01 55 AA` | 狗叫声 1 号 = **「汪汪」**（实测确认） |
| `AA 55 70 02 55 AA` | 狗叫声 2 号 = **「嘤嘤」**（实测确认；2026-09-27 由 `AA 55 71 02` 改来） |
| `AA 55 71 00 55 AA` | 平台自定义发声段 0 |
| `AA 55 72 <数字> 55 AA` | **播报数字**（数字 0–100，用「播报数字 [NUM]」积木发） |

⭐ **狗叫类第一字节统一 `0x70`，靠第 4 个数字区分具体声音** —— 加新声音就改第 4 个数字，
固件里对应 `voice.py` 的 `SND_xxx` 常量。

```python
import voice
voice.play('汪汪')      # 播放预置声音（汪汪/嘤嘤, 映射见 voice.SND_WANG/SND_YING）
voice.say(0x70, 1)      # 发狗叫声 1 号 = 汪汪 (AA 55 70 01 55 AA)
voice.say(0x70, 2)      # 发狗叫声 2 号 = 嘤嘤 (AA 55 70 02 55 AA)
voice.SoundAngle        # 最近一次声源角度（度, 0-180）; -1 = 开机后还没收到过
voice.on_cmd(0x30, fn)  # 注册回调: 收到停止指令时执行 fn
voice.stop()            # 停止后台轮询
```

> ⚠ **只转发事件、不做动作（2026-08-19 起）**：固件收到语音指令**只打印 + 触发事件回调**（KittenBlock 事件积木），**不内置任何动作**（前进/后退等均已移除），动作全由用户在 KittenBlock 里编程。
>
> ⭐ **唯一例外：`0x3F` 播报电压（2026-09-27 起）** —— 收到它时固件**自动**给 CI-33T 发
> 「播报数字」报电量百分比（`AA 55 72 <pct> 55 AA`，pct 来自 `voltage.read_pct()`）。
> 破例原因：这是"把固件自己采到的电池数据念出来"，属于固件自身状态的播报，不是运动/姿态；
> 交给用户程序做的话，电压的唯一真源在 C 层，Python 侧抄不到。**不需要写任何用户代码。**
>
> - **平台前提**：CI-33T 的【串口输入】要配好 `AA 55 72 <数据> 55 AA` 的 **0–100 词条**，
>   否则帧收到了也没声音可放（见 [AGENTS.md](../AGENTS.md) 的语音事件系统 §3.2.2）。
> - **无条件执行**：`voiceWhenVolt` 事件**照常**能注册。所以若用户程序里也写了
>   「当收到 [播报电压] 指令」积木，**固件播报和用户动作会同时触发**（听感上"报两次"）。
> - 读不到电池数据时按 **0** 播报，同时打一行日志说明原因。

**CI-33T 端配置（智能公元平台）**：在【离线命令词与应答语自定义】为每个命令词（"前进"、"停止"…）配置**串口发送**，输出对应的**命令帧** `BB <CMD> 00 EE`（如前进 = `BB 31 00 EE`，CMD 见上表）；**声源角度的输出保持原样 `<角度> 00 00 00` 不要改**（改了就解析不出来了）。要本系统发声时，配置【串口输入】词条匹配 `AA 55 <数据> 55 AA` 帧触发对应音效/播报。波特率两边都设 9600。

> ⚠ **模块侧和固件必须同时是新格式**：命令帧从 2 字节变 4 字节后，**新固件认不出旧的 2 字节命令**
> （2 字节拼不成 4 字节帧，会被当成残帧留着），14 条命令词会全部失效。
> 所以刷入新固件前，先把平台侧 14 条命令的输出格式改成 `BB <CMD> 00 EE`；
> 或者接受"先刷固件、期间语音指令不响应"，改完平台侧即恢复。
> 反向（平台先改、固件还是旧的）**不影响使用** —— 旧固件逐字节扫到帧里的 CMD 字节照样派发。


### 2.2 上电开关文件

`frozen/main.py` 每次开机都会去板子上找这些开关文件：**文件在就执行，删掉就什么都不做**。
所以文件本身就是开关 —— 不用重编译固件，也不用敲指令，下次上电即生效。

| 开关文件 | 作用 | 源文件 |
|---------|------|--------|
| `/camera_on.py` | 上电自动开「网页+摄像头」 | `mpy_modules/camera_on.py` |
| `/pwm_ext_on.py` | 上电自动启扩展舵机 (PWM_EXT) | `mpy_modules/pwm_ext_on.py` |

```python
# /camera_on.py 的全部内容
import camera_stream
camera_stream.start(stream=True)     # 只开网页不开相机 → 去掉 (stream=True)
```

```python
# /pwm_ext_on.py 的全部内容 (当前启用 PWM_EXT2)
EXT1 = 0        # GPIO 3  与电池 ADC 同脚 → 电池读数失效
EXT2 = 1        # GPIO 47 空闲脚 → 无副作用
EXT3 = 0        # GPIO 48 与 WS2812 同脚 → 电池灯灭
ANGLE = 90      # 启用那几路的初始角度

import pwm_ext
for _n, _want in {1: EXT1, 2: EXT2, 3: EXT3}.items():
    if _want:
        pwm_ext.on(_n)
        pwm_ext.set_angle(_n, ANGLE)
```

> **改哪一路只需动上面那几个 0/1。** 哪个 GPIO、哪个 MCPWM 通道、要不要先停电池
> ADC —— 全在固件里（`frozen/pwm_ext.py` 的 `pwm_ext.on()`），不用管也改不到。
> ⚠ 注释保持**纯英文**：蓝牙上传会丢非 ASCII 字节。

- **开**：把 `mpy_modules/<对应文件>` 传到板子（ViperIDE 拖进去，或 `mpremote cp ...`）——**文件名必须和上表一致**，放在根目录
- **关**：从板子删掉那个文件即可
- **和 `/main.py` 互不影响** —— KittenBlock 每次下载只重写 `/main.py`（它下载的用户程序），不会碰这些文件
- ⚠ 这些文件必须是**纯 ASCII**（别写中文注释）：蓝牙传含中文的内容会丢字节。仓库里那几份都是纯 ASCII 的
- ⚠ 若改启用 **PWM_EXT1(GPIO3)** 或 **PWM_EXT3(GPIO48)**，它们与电池检测同脚，**必须先 `bpuppy_adc.stop()`**，
  否则固件仍在驱动/读同一个脚。代价是电池电压读数和 WS2812 指示灯会停掉，直到 `bpuppy_adc.init()` 恢复。
  **PWM_EXT2(GPIO47) 是空闲脚，不需要这一步**（样例文件用的就是它）

## 三、视觉

### 3.1 摄像头操作 bpuppy_camera

**拍照:**

```python
import bpuppy_camera

# 初始化（上电后首次需手动 init）
bpuppy_camera.init()

# 拍照 → 返回 (data, width, height, format)
data, w, h, fmt = bpuppy_camera.capture()
# data: JPEG 字节数据（默认 1600×1200）
# fmt:  4=JPEG, 1=RGB565, 5=GRAYSCALE

# 释放
bpuppy_camera.deinit()
```

**格式切换:**

默认 `init()` 输出 JPEG 1600×1200。如需其他格式或分辨率：

```python
import bpuppy_camera
# QVGA 320×240 RGB565（色球识别用）
bpuppy_camera.init_adv(bpuppy_camera.QVGA, 0, 2, 20000000, bpuppy_camera.RGB565)

# SVGA 800×600 JPEG（WiFi 图传用）
bpuppy_camera.init_adv(bpuppy_camera.SVGA, 10, 2, 20000000, bpuppy_camera.JPEG)
```

| 分辨率 | 常量 | 像素 |
|--------|------|------|
| 160×120 | `QQVGA` | — |
| 320×240 | `QVGA` | — |
| 640×480 | `VGA` | — |
| 800×600 | `SVGA` | — |
| 1024×768 | `XGA` | — |
| 1600×1200 | `UXGA` | 默认 |

| 格式 | 常量 | 用途 |
|------|------|------|
| JPEG | `JPEG` (4) | 拍照/图传，硬件编码 |
| RGB565 | `RGB565` (1) | 图像识别，原始像素 |
| GRAYSCALE | `GRAYSCALE` (5) | 灰度图 |

**已知问题:**

- 初始化后前几帧可能是旧画面（双缓冲残留），`camera_serial.snap()` 已自动丢弃前 3 帧
- 图像上下颠倒：已在驱动中通过 `set_vflip(1)` 修正

---


## 四、设置与校准

### 4.1 舵机校准 bpuppy_servo

**舵机命名常量:**

| 常量 | 值 | 含义 |
|------|-----|------|
| `bpuppy_servo.LF_HIP` | 0 | 左前大腿 |
| `bpuppy_servo.LF_KNEE` | 1 | 左前小腿 |
| `bpuppy_servo.LH_HIP` | 2 | 左后大腿 |
| `bpuppy_servo.LH_KNEE` | 3 | 左后小腿 |
| `bpuppy_servo.RF_HIP` | 4 | 右前大腿 |
| `bpuppy_servo.RF_KNEE` | 5 | 右前小腿 |
| `bpuppy_servo.RH_HIP` | 6 | 右后大腿 |
| `bpuppy_servo.RH_KNEE` | 7 | 右后小腿 |

**函数（调用即用）:**

| 调用示例 | 说明 |
|------|------|
| `bpuppy_servo.init_all()` | 按硬件映射初始化全部 8 路舵机 |
| `bpuppy_servo.set_angle(0, 90)` | (通道, 角度) 设置舵机角度 (当前映射 -35~215) |
| `bpuppy_servo.get_angle(0)` | (通道) 返回当前角度 (float) |
| `bpuppy_servo.stop()` | 紧急停止所有舵机 (关闭 PWM) |
| `bpuppy_servo.cal_point(0, 0, 3)` ★ | (通道, 点, 参考角) **三点校准**: 点 0/1/2 = 0°/90°/180° |
| `bpuppy_servo.get_cal_point(0, 1)` | (通道, 点) 读取指定点校准参考角 |
| `bpuppy_servo.cal(0, 90)` | (通道, 参考角) 兼容旧接口 = 只标 90° 点 |
| `bpuppy_servo.get_cal(0)` | (通道) 读取 90° 点校准参考角 |
| `bpuppy_servo.load_cal()` ★ | 从 NVS 加载校准值 (开机自动调用) |

**三点校准说明（★推荐）:**

每通道在 **0°/90°/180°** 三个点各存一个"命令该角度时舵机实际应转的角度"，中间分段线性插值——同时补偿**零点偏移和斜率/非线性误差**（单点 `cal` 只补偿零点）。

**标定流程**（每个舵机三点；`cal_point` 一步到位：写 NVS + 当场按原始角度转舵机）:

```python
bpuppy_servo.cal_point(0, 0,   3)   # ① 试 3   → 看腿是否在 0° 理想位 → 不对就换数重发
bpuppy_servo.cal_point(0, 1,  95)   # ② 试 95  → 腿垂直 (90° 位)
bpuppy_servo.cal_point(0, 2, 178)   # ③ 试 178 → 腿在 180° 理想位
```

每次调用都写一次 NVS，所以**最后一次试对的值就是最终值**，没有单独的「保存」步骤。`cal_point` 发的是你给的**原始角度**（跳过校准表）；`set_angle` 相反，发的是经三点插值后的值 —— 所以**试的时候别用 `set_angle`**（校准表边试边变，会让它输出的角度跟你输入的对不上），标完再用 `set_angle(0/90/180)` 验证各点是否精确到位。掉电保留；中间角度（45°/135° 等）由插值补。`cal(ch, ref)` 保留为兼容旧接口（等价 `cal_point(ch, 1, ref)`，只标 90° 点）。

> 方向约定（髋 0/90/180 指向、膝左右镜像）见 [AGENTS.md](../AGENTS.md) 的「舵机角度标定」表。

### 4.2 持久化参数（★NVS，掉电保留）

| 组 | 调用示例 | 默认值 |
|----|------|--------|
| 几何 | `bpuppy_motion.cal_ik(40, 45)` ★ | 大腿 40, 小腿 45 (mm) |
| 几何 | `bpuppy_motion.set_body_dims(62.5, 59)` ★ | 前后半距 62.5, 左右半宽 59 (mm) |
| 几何 | `bpuppy_motion.set_joint_limits(0, 180, 10, 170)` ★ | 髋 0~180, 膝 10~170 (deg) |
| 舵机 | `bpuppy_servo.cal(ch, 90)` ★ | 8 路校准参考角 (deg) |
| 重心 | `bpuppy_motion.set_center(0)` ★ | 脚中位偏移 (mm)，正=前移，**两道闸门**：① ≤ 大腿长/2（默认 ±20）② 跟当前 stride/height 组合够得着 —— 默认参数下 ② 更紧，实际只到 **±11.4mm** |

> 以上参数设定后自动写入 NVS（几何/重心 9 个 + 每路舵机 3 点 × 8 路 = 24 个校准值），`motion.start()` 自动恢复。无 NVS 数据时回退 `ik.h` 宏默认值。
>
> `show_geometry()` 打印几何参数 + 各路舵机的 **90° 点**校准值，**不显示 0°/180° 点** —— 那两点要用 `bpuppy_servo.get_cal_point(ch, 0/2)` 单独读。
>
> `get_geometry()` 返回 `(L1, L2, 前后半距, 左右半宽)`，给程序读回当前几何（`show_geometry()` 只是打印，塞不进变量）。
> `poses.stand()`（含上电站姿、「站立」积木）和 `balance` 都从它现读，所以 **`cal_ik()` 改完腿长，站姿和自平衡会立刻跟上**。
> ⚠ 别用 `bpuppy_ik.L1/L2` 算站姿 —— 那是**编译期默认值**，`cal_ik()` 改不到它，混用会让站姿和步态按两套腿长解算。
>
> **非法值会被拒**（保持原值不变，返回 `False`，repl 打印 `⚠ ... 被拒`）：`cal_ik` / `set_body_dims` 需 1~500 mm；`set_joint_limits` 需 `min < max` 且在舵机行程内（当前 270° 舵机 = −35~215）；`set_center` / `set_body_pose` 见下。
> 因为这几个值写进 NVS 且**跨固件升级存活**，写入和开机读取两处都校验 —— 已写坏的板子刷完新固件会在开机日志看到 `⚠ NVS 里 ... 非法 -> 回退默认` 并自动恢复，不会一直坏下去。
>
> **`set_center` 和 `set_body_pose` 都是两道闸门**（2026-09-28 起）：① 单独看自己 —— 重心 ≤ 大腿长的一半（默认 ±20mm）；② 跟当前 stride / height / lift / 姿态**组合起来足端够不够得着**。够不着时 `ik_solve_2dof` 只会把距离钳到可达上限、**静默失去伸展**（脚在地上蹭、狗歪着走，**不报错**），所以两道都拦。**顺序不再有影响**：先压姿态再设参数、还是反过来，都拦得住。
>
> ② 比 ① 紧得多。默认 stride=70 / height=70 时脚已经伸到 78.3mm、离可达上限 84 只剩 5.7mm ⇒ **重心实际最多挪 ±11.4mm**，**机身姿态 \|俯仰\| ≤ 3.7°、\|横滚\| ≤ 6.2°**。想挪更多 / 压更大就**先减小步长**：stride 60 → 重心 15mm 能过；stride 40 → 俯仰能过 8°。
>
> ⚠ `set_center` 被拒时**不写** flash（两道校验都在写 NVS 之前返回）。开机回读 NVS **只复查 ① 量级**（那时 stride/height 还是默认值，② 组合校验没法算）—— 所以别指望开机时拦住"存进去合法、配上后来改的 stride 才够不着"的组合。


## 五、传感器与姿态

### 5.1 IMU 调试 bpuppy_imu


> 手动初始化: `bpuppy_imu.init(0, 14, 21, 0x68)`。balance / heading_anchor / calib_mag 的 `start()` / `on()` 会自动启动，无需手动。停止: `bpuppy_imu.stop()`（停 AHRS 任务，可重新 init）。

**磁力计融合开关 `set_mag_fusion`:**

```python
bpuppy_imu.set_mag_fusion(False)   # 磁力计只修 yaw, 不参与 roll/pitch
bpuppy_imu.set_mag_fusion(True)    # 9轴完整融合 (默认)
```

> 磁力计校准残差可能把 roll/pitch 拉偏（绕 Z 旋转时姿态漂移）。`False` 让磁力计**只管航向（yaw）**，roll/pitch 由加速度计决定（水平更准），yaw 仍稳定。**balance 自动使用此模式**；需要完整 9 轴（含磁力计参与水平）时切回 `True`。

**查看原始数据 (加速度/陀螺/磁力计/温度):**

```python
import bpuppy_imu
bpuppy_imu.init(0, 14, 21, 0x68)      # 初始化 (幂等)
import time
while True:
    a, g, m, t = bpuppy_imu.read_raw()
    print('acc=%.2f %.2f %.2f  gyro=%.2f %.2f %.2f  mag=%.1f %.1f %.1f  temp=%.1f' %
          (a[0],a[1],a[2], g[0],g[1],g[2], m[0],m[1],m[2], t))
    time.sleep_ms(500)
```

**查看姿态角 (roll, pitch, yaw):**

```python
import bpuppy_imu
bpuppy_imu.init(0, 14, 21, 0x68)      # 初始化 (幂等)
import time
while True:
    r, p, y = bpuppy_imu.read_angles()
    print('roll=%.2f pitch=%.2f yaw=%.2f' % (r, p, y))
    time.sleep_ms(500)
```


### 5.2 IMU 校准（陀螺 + 加速度计）

```python
bpuppy_imu.calibrate(300)     # 拿住不放约 5 秒; 校准完自动存 NVS, 掉电保留
```

⚠ **校准时机身必须水平**。什么时候需要、不校会怎样 → [新板上电操作指南.md](新板上电操作指南.md) 的标定节。

### 5.3 磁力计校准 calib_mag

```python
import calib_mag
calib_mag.start()             # 引导式 3D 椭球校准, 跟着串口提示转; 结果存 NVS
```

有磁力计的板子才需要（6 轴模块会自己跳过）。残差怎么判 → [新板上电操作指南.md](新板上电操作指南.md) 的磁力计节。

### 5.4 航向锁定 heading_anchor

闭环拆成两个文件，各管一半：

| 文件 | 角色 |
|---|---|
| `heading_anchor.py` | **step1** — IMU 航向的 err 源。对外就四个：`on` / `off` / `read` / `set` |
| `heading_follow.py` | **step2** — 闭环执行体，`err → turn`。**它不知道 err 从哪来**，所以视觉 / 声源也能喂进来 |

**两个都冻结进固件了**，直接 import 就能用。

> ⚠ **别再把这两个 `.py` 手工传到板子上。** MicroPython 的查找顺序是
> `sys.modules` → **板子根目录** → 固件，所以板上留着一份同名的 `.py` 会**盖住固件版**——
> 表现是"固件明明升级了，行为还是老的"。真遇到了就 `import os; os.remove('heading_anchor.py')`。
> （传 `pwm_ext_on.py` / `camera_on.py` 那些**不冲突**，照旧。）

```python
import heading_anchor as anc

anc.on()        # 打开：锁住**当前朝向**
anc.on(30)      # 打开：从现在这个方向**再转 30°**，然后锁住
anc.read()      # (是否打开, 目标角, 当前yaw, 偏差)
anc.off()       # 关闭：停闭环 + 松转向
```

> ⚠ **`on()` 收的是"要转多少"，不是"转到几度"** —— 是**相对量**。
> 磁力计不可靠时绝对航向本来就不可信（实测在屋里它跟着**位置**变、不跟着**朝向**变），
> 只有相对量站得住。

> ⭐ **行进中随时能调** —— 已经在跑时再调 `anc.on(30)` 只是**换个目标**，闭环下一拍自己跟上：
> 不重起线程、`turn` 不掉 0，弯是平滑拉过去的。

> ⚠ **首次**调用会先花 0.6 秒**快判**姿态稳不稳：稳了（比如程序开头已经初始化过 IMU）就直接锁目标；
> 还在冷启动爬升才走完整等待（约 5 秒）。**绝不会锁一个还没收敛的 yaw** —— 那会让闭环追着漂移把狗转过去。
> 开机后先在**站着**的时候调一次把它"预热"掉，之后行进中调就都是瞬间返回。

> ⚠ **`off()` 只是松开转向，不停狗** —— 它停的是"纠方向"这件事。要停狗用「停止」/ `set_gait("stop")`。

调参 —— **全都有缺省，不调也能跑**：

```python
anc.cfg                    # 看当前参数
anc.set(kp=2.5)            # 改一个（已锁定时立即生效，目标不变）
anc.set()                  # 无参 = 全部恢复缺省
```

**KittenBlock**：两个积木在「**传感器功能**」里 —— **「航向锁定 偏转 [ ] 度」** 和 **「解除航向锁定」**。
**KittenBlock 生成的程序开头会自动初始化 IMU**（`libs` 注入头和 `afterConnect` 里都加了
`bpuppy_imu.init(...)`）—— 等用户点积木时姿态早就收敛了，那 0.6 秒快判直接通过，**基本无感**。
（不用再手动点「初始化 IMU」；点了也只是空调用，幂等。）


### 5.5 电池电压 bpuppy_adc

电池分压接 **GPIO3 = ADC1_CH2** (ADC1 在 BLE/WiFi 下可用) → 启用 (`BPUPPY_ADC_ENABLE=1`)。硬件接线见 `PCB/硬件连接.md` 电池电压测量节 (分压 51k/10k)。

**读取 (推荐):**

```python
import bpuppy_led
v = bpuppy_led.batt_v()       # 已标定电池电压 (V); 监控未跑 → -1.0
p = bpuppy_led.batt_pct()     # 电量百分比 0-100; 监控未跑 → 0
print("电池电压: %.2f V  电量: %d%%" % (v, p))
```

或者用薄封装 (推荐后者 — 它会顺带确保 ADC + 指示灯已启动):

```python
import voltage                # 上电默认已自动 import, 无需手动
v = voltage.read_v()          # 与 bpuppy_led.batt_v() 同源, 不做二次换算
p = voltage.read_pct()        # 电量百分比 0-100 (整数)
```

**电量百分比 `read_pct()` (0-100):**

定标 **7.4V = 100%, 6.6V = 0%** —— 两个端点就是 LED 的蓝/红分界点, 所以
「百分比到 0」和「灯变红」是**同一时刻**, 不可能对不上。低于 6.6V (含红闪区) 一律 0,
高于 7.4V (真实满充约 8.4V) 钳到 100。

```python
import voltage
print(voltage.read_pct())     # 0-100; 监控未跑/ADC 未就绪 → 0 (不是 -1)
```

> 读不到数据时返回 **0**, 跟"电量真的 0%"无法区分 —— 要区分就同时看 `read_v()`
> (`< 0` 表示无有效读数)。百分比从 C 层 `s_batt_v` 现算, 不另存缓存。

> **只有一个标定实现。** 系数 (a, b) 存在 C 层 `drivers/led_driver.c`, 持久化在 NVS。
> `voltage.read_v()` / `read_pct()` 直接返回 `bpuppy_led.batt_v()` / `batt_pct()` 的
> 同一个值 —— 所以 **LED 颜色和读到的电压/电量不可能不一致** (同一条代码路径)。

**底层原理 (未标定):**

```python
import bpuppy_adc
bpuppy_adc.init()                      # 初始化 (V3.0: ADC1_CH2 GPIO3)
                                       #   ↳ 同时激活 GPIO48 WS2812 电池指示灯
mv = bpuppy_adc.read_mv()              # ADC 引脚电压 (mV, 分压后)
batt_v = mv * 6.1 / 1000               # 未标定电池电压 (V), 51k/10k 分压换算
```


> **WS2812 电池指示灯**: `bpuppy_adc.init()` 后 GPIO48 的 WS2812 自动按电压亮色 —
> ≥7.4V 蓝 (满电) / 6.6~7.4V 蓝→紫→红 渐变 / ≤6.6V 红 (低压) / **<6.4V 红色闪烁 (危险)**。
> 颜色两个分界点**就是**电量的 100% / 0% (都取 C 层同一组常量)。
> 手动控制: `import bpuppy_led; bpuppy_led.set_color(r,g,b)` / `bpuppy_led.off()`。阈值见 `drivers/led_driver.c` 顶部常量。

---


### 5.6 扩展舵机 PWM_EXT

主板预留 3 路扩展舵机输出（接第 9 个及以后的小舵机）。接线、复用脚、MCPWM 原理见 [硬件连接.md](../PCB/硬件连接.md) 的「12. PWM_EXT 扩展舵机输出」。

**平滑控制 (推荐 — 用 `poses`, 与主舵机同一套 API):**

统一编号：`0`~`7` = 主舵机，`8`/`9`/`10` = **PWM_EXT1/2/3**。

```python
import poses
poses.set_servo(9, 90)        # 9 = PWM_EXT2 (只写缓冲, 不动)
poses.set_step(3)             # 过渡速度 °/帧 (默认 3.0 = 150°/s)
poses.commit()                # 一起平滑逼近, 主舵机和扩展舵机同时到位
poses.get_servo_angle(9)      # 读当前角度
poses.oscillate(9, 15, 1)     # 正弦摆动 (ch, amp, hz, cycles)
```

> **缓动只有一份实现**（`poses._move_to`）—— 缓动算法跟"驱动哪个舵机"无关，
> 只跟"读角度/写角度"有关。主舵机和扩展舵机在**同一个循环**里逼近。
> `pwm_ext` 模块只负责编号映射和读写直通，**不含缓动**。

**裸控制 (不缓动, 立即生效):**

```python
import pwm_ext
pwm_ext.on(2)                # 启用 PWM_EXT2 —— 只在开机脚本里调 (见 2.2)
pwm_ext.set_angle(2, 90)     # PWM_EXT2 立即转到 90° (硬切)
pwm_ext.get_angle(2)         # 读当前角度
pwm_ext.get_pulse_us(2)      # 读当前脉宽 500~2500
pwm_ext.off(2)               # 释放该路
```

> KittenBlock 的「舵机 [X] 设为 [Y]°」「舵机 [X] 的角度」「摆动 [X] …」三个积木的下拉里
> 已经加了 **扩展舵机1/2/3**, 直接用积木即可, 不用手敲。

⚠ **未启用的路读写会报错**，不会替你打开 —— 管脚功能**开机时就定死了**。

**上电自动启用**: 把 `mpy_modules/pwm_ext_on.py` 传到板子根目录（文件在 = 开，删掉 = 关，
不用重编译）。开哪几路改文件顶部那几个 0/1 即可；哪个 GPIO、哪个 MCPWM 通道、要不要先停
电池 ADC —— 都在固件里，不用管。

⚠ PWM_EXT1(GPIO3) 与电池 ADC 同脚，PWM_EXT3(GPIO48) 与 WS2812 同脚 —— 启用这两路会
**先自动停掉电池检测**（副作用：电压读数变 -1.0、电池灯灭）。PWM_EXT2(GPIO47) 是空闲脚，无副作用。

---


## 六、PC 端工具

### 6.1 串口拍照 capture.py


ESP32 上电后，在 PC 终端运行：

```powershell
pip install pyserial
cd tools
python capture.py COM3   # 端口换成实际值 (设备管理器查看)
```

连接后按 **Enter** 拍照（自动保存并打开图片预览），输入 **q** 退出。全程不重启 ESP32。

> 原理: PC 串口发送 `import camera_serial; camera_serial.snap()` → ESP32 拍照 → base64 回传 → PC 解码保存为 JPEG。

---


## 附录 · 编译 / 烧录 / 串口

**编译**：Docker 编译、版本铁律（MicroPython v1.22.1 **只能**配 ESP-IDF v5.1.2）→ [AGENTS.md](../AGENTS.md) 的「编译环境」。
**烧录**：新板第一次走 bTool 图形界面 → [新板上电操作指南.md](新板上电操作指南.md)；命令行 / 只烧 app 分区 → [AGENTS.md](../AGENTS.md) 的「Windows 烧录」。
**串口**：115200、CH343、PuTTY / Tera Term / VS Code → [AGENTS.md](../AGENTS.md) 的「串口连接」。
（REPL 必须用 **friendly REPL**，C 侧 `ESP_LOGx` 在 raw REPL 下会被丢弃。）

产物：`build/micropython_bpuppy.bin`。
