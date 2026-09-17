# 问题复盘：只动扩展舵机，却把步态停掉了

> 发现日期：2026-09-17
> 状态：**根因已定位，改法已确定，代码尚未修改**（本文只记录，实施见 §6）

---

## 1. 现象

最小复现：

```python
import poses, bpuppy_motion

bpuppy_motion.set_gait("walk")     # 先让狗走起来
# ……狗正常行走中……
poses.set_servo(8, 90)             # 只点扩展舵机 PWM_EXT1
poses.commit()
```

**结果：狗停住，原地冻住。**

最迷惑的一点是：

> **看起来像什么都没发生 —— 主舵机纹丝不动 —— 但狗停了。**

扩展舵机确实转到了 90°，主舵机保持在原位一动不动，可步态没了。
`bpuppy_motion.motion_is_running()` 返回 `False`。

---

## 2. 根因

### 2.1 `_move_to()` 每帧做两件**互不相干**的事

`frozen/poses.py` 的 `_move_to()`（约 :144~:154）在一个循环里同时处理主舵机和扩展舵机：

```python
        # ---- 事 A：主舵机 ----
        bpuppy_servo.group_begin()
        for ch in range(8):                                     # ← 无条件！跟 targets 无关
            v = cur[ch] if cur[ch] is not None else bpuppy_servo.get_angle(ch)
            bpuppy_servo.group_add(ch, v)
        bpuppy_servo.group_commit()                             # ★ 元凶

        # ---- 事 B：扩展舵机 ----
        for ch in range(_EXT_BASE, _CH_COUNT):
            if cur[ch] is not None:
                _write(ch, cur[ch])                             # ← 你真正点的那个
```

- **事 B** 走 `pwm_ext.set_angle` → MCPWM 驱动，**从不经过 `bpuppy_servo`**，本身完全无害。
- **事 A** 是 `for ch in range(8)`，**无条件执行** —— 哪怕 8 路主舵机一个都没被点。

所以：**停步态跟"动扩展舵机"没有因果关系。它是同一段代码里另一件事顺带触发的。**

### 2.2 事 A 的终点是"跳过整个循环"

```
servo_driver.c:392   mp_servo_group_commit()            ← MicroPython 包装层
servo_driver.c:393       motion_python_servo_write();   ★ 先切模式，在角度被看之前
servo_driver.c:394       servo_group_commit();
         ↓
motion_task.cpp:878   motion_python_servo_write()
motion_task.cpp:880       if (g_mode != MODE_POSE) motion_set_mode(MODE_POSE);
         ↓
motion_task.cpp:861   case MODE_POSE:
motion_task.cpp:862       g_motion.enabled        = false;
motion_task.cpp:863       g_motion.emergency_stop = true;
         ↓
motion_task.cpp:212   motion_task_main():
                          while (1) {
                              if (!g_motion.enabled || g_motion.emergency_stop) {
                                  vTaskDelayUntil(&last_wake, period);   // 睡 20ms
                                  continue;                              // ★ 回开头
                              }
                              // ↓ 下面整段才是"走路"：姿态过渡 → 相位 → 足端 → IK → 写舵机
```

**"不动了"就是 `motion_task.cpp:212` 那个 `if`。没有任何"步态转换"代码。**

### 2.3 三个后果

| # | 后果 | 说明 |
|---|------|------|
| 1 | **任务没死，在空转** | 每 20ms 醒来、判断、睡回去。不是 `vTaskDelete` 也不是 `vTaskSuspend` → 所以置回标志即可立刻恢复（`motion_task.cpp:852-859` 还专门同步 `g_smooth_angles` 防跳变） |
| 2 | **舵机"保持"而不"松掉"** | `servo_group_commit` 不再被调 → 没有新 duty 写进 LEDC → 保持上一帧脉宽 → 位置舵机停在最后角度 |
| 3 | ⚠ **原地冻结，可能冻在抬腿中途** | `motion_task.cpp:237-258` 那段 0.3s **优雅停步过渡**（`set_gait('stop')` → `GAIT_STOP` 才走）位于 `:212` **之后**，被一起跳过了。若恰好在一条腿抬在空中时被冻结，它就保持着那条腿悬空的姿势定住 |

---

## 3. 为什么"写回原值"也照样停

`cur[0..7]` 在这次调用里全是 `None`（你没设过），所以事 A 走 `else` 分支：
`bpuppy_servo.get_angle(ch)` 读回当前角度，再原样写回去。

我核对过 `servo_driver.c:250-267` 的 `servo_get_angle` —— 它是 `servo_apply_cal`
（三点分段线性，`:225-233`）的**精确逆运算**，所以这个往返是恒等的，
主舵机**真的一动不动**。

但没用。因为：

> `motion_python_servo_write()` 在包装函数的**第一行**（`servo_driver.c:393`），
> 它不看值，也不看有没有值。它只回答一个问题：
> **"有人从 Python 调 `group_commit` 了吗？"**

是 → 切 POSE。至于传了什么，不在它的考虑范围内。

---

## 4. 为什么加上扩展舵机之前不发作

`_pose_buf` 当时只有 8 个槽，8 个**全是**主舵机。于是：

```
"buffer 非空"  ⇒  必然蕴含  ⇒  "有主舵机参与"
```

无条件提交**恰好等价于**正确行为。逻辑漏洞存在，但**触发不了**。

从 8 路扩到 11 路（`_CH_COUNT = 11`，`_EXT_BASE = 8`）的那一刻起，
才第一次出现这种情况：**确实设了目标值，但设的是主舵机之外的通道。**

> 这个 bug 不是被"写出来"的，是被"扩出来"的 —— 扩展舵机把一段原本无害的
> 陈旧逻辑变成了真缺陷。

---

## 5. 哪些是设计、不该改

发现这个 bug 后最容易顺手"修错"的地方 —— 下面这些都是**故意的**，改了就坏。

### 5.1 切 POSE 的钩子只在**写**路径上，读不切

`bpuppy_servo` 全部导出里，调用 `motion_python_servo_write()` 的只有 4 处：

| 导出函数 | 行 | 钩子 | 切 POSE |
|---|---|---|---|
| `set_angle(ch, deg)` | `servo_driver.c:368` | ✅ | 是 |
| `group_commit()` | `servo_driver.c:393` | ✅ | 是 |
| `cal(ch, deg)` | `servo_driver.c:437` | ✅ | 是 |
| `cal_point(ch, p, deg)` | `servo_driver.c:455` | ✅ | 是 |
| `get_angle(ch)` | `servo_driver.c:399` | ❌ | 否 |
| `get_cal(ch)` | `servo_driver.c:444` | ❌ | 否 |
| `get_cal_point(ch, p)` | `servo_driver.c:462` | ❌ | 否 |

**读随便读，写才交接。**

### 5.2 这根钩子是**交接棒**，不是 bug

motion 任务在 50Hz 里也在驱动**同样这 8 路** LEDC 通道
（`motion_task.cpp:403` / `:517-521`），走的是 **C 层**的 `servo_group_add/commit` ——
**那条路上没有钩子**。

如果 Python 这时也能同时写同样 8 路，两边就会互相打架。
所以"Python 动主舵机 = 交出控制权"是**正确规则，保留不动**。

**不对称之处：**

> 同一个 `servo_group_commit()`，从 C 调不停步态，从 Python 调解停 ——
> 差别只因为钩子挂在 MicroPython 包装层上。

### 5.3 ⚠ `motion_set_mode` 这名字容易误导

它**根本不碰** `g_motion.gait` / `g_phase`，只是把 `enabled` / `emergency_stop`
两个标志翻了一下。设计意图是**"暂停 + 交棒"**，不是"停步态"。

| 想要的效果 | 正确做法 | 机制 |
|---|---|---|
| **优雅停步**（四腿踩实站住） | `set_gait('stop')` → `GAIT_STOP` | `motion_task.cpp:243-258` 的 0.3s smoothstep 退到站姿 ✅ |
| **Python 临时接管舵机** | Python 写主舵机 | `motion_task.cpp:212` 跳过循环，原地冻结 ⚠ |

---

## 6. 改法

**改一个文件、一处：`frozen/poses.py` 的 `_move_to()`。**

思路：让事 A 跟事 B 一样**只处理本次真正参与的通道**。

### 改动 1 —— 进入循环前，算一次"本次有没有主舵机参与"

```python
    # ★ 本次是否有主舵机参与 —— 决定要不要碰 bpuppy_servo。
    #   主舵机走 group_commit, 而它会强制切 MODE_POSE (停步态):
    #       group_commit → motion_python_servo_write → motion_set_mode(MODE_POSE)
    #   所以只动扩展舵机时不该付这个代价 (oscillate() 已经是这个行为)。
    has_main = any(t is not None for t in targets[:_EXT_BASE])
```

### 改动 2 —— 给事 A 加守卫

```python
        # 主舵机: 一帧内批量提交 (group), 未参与的通道从舵机读回原位。
        # 没有主舵机参与就整段跳过 —— 不写就不会切 POSE, 步态继续跑。
        if has_main:
            bpuppy_servo.group_begin()
            for ch in range(8):
                v = cur[ch] if cur[ch] is not None else bpuppy_servo.get_angle(ch)
                bpuppy_servo.group_add(ch, v)
            bpuppy_servo.group_commit()
```

`_EXT_BASE`（=8）和 `targets` 都是现有变量，不需要新增任何东西。

### 为什么这样改不会引入新问题

| 理由 | 说明 |
|---|---|
| **有先例** | 同文件的 `oscillate()` 已经是这个行为 —— 它只用单通道 `_write`，扩展舵机路径不碰 `bpuppy_servo`，所以摆动扩展舵机从来不会打断行走。本次只是把 `_move_to()` 对齐到已有先例 |
| **跳过是安全的** | 主舵机是 PWM 位置舵机，不写就保持原位（脉冲照常发）。而在 MODE_MOTION 下 motion_task 本来就在驱动它们 —— 跳过提交正是"让步态继续管主舵机"这个本意 |
| **主舵机行为零变化** | 只要有主舵机（0~7）参与，走的还是原来那段代码，一字不改 |
| **`has_main` 是局部变量** | 纯内部实现，不改变任何现有 API |

### 变量命名

`has_main` 里的 `main` 指"主舵机"（相对于扩展舵机）。若觉得不够直白，
可改名为 `has_main_servo` 或 `touches_main` —— 纯内部命名，随意。

---

## 7. 验证

⚠ **`frozen/poses.py` 在 `frozen/manifest.py` 里 = 冻进固件**。
改完必须重新编译并烧录才生效，单纯上传 `mpy_modules/` 不起作用。

编译：`bash build.sh`（宿主机 Git Bash，Docker + ccache，见 `docs/README.md` 编译环境节）。

烧录后按下表实测：

| # | 操作 | 期望 |
|---|------|------|
| **A** | 走起来 → `poses.set_servo(8, 90); poses.commit()` | **狗继续走**，扩展舵机转到 90° ← 本次修的就是这条 |
| **B** | `poses.set_servo(0, 90); poses.commit()` | 照常：停步态、切 POSE、主舵机平滑移动（回归） |
| **C** | `poses.go_to([8], 5)` | 同 A，扩展舵机平滑到位，不停步态 |
| **D** | 什么都不 `set_servo` 直接 `poses.commit()` | 无任何动作、**不停步态**（改前这个也会停） |
| **E** | `set_servo(0,...)` 与 `set_servo(8,...)` 同时下 | 两种舵机同时到位；主舵机那组照常切 POSE |

> `bpuppy_led.batt_v()` 可用来辅助确认：A / C / D 三条里**电池读数不应变成 -1.0**。
> 若变成 -1.0 说明 ADC 被 PWM_EXT 抢了 —— 注意 PWM_EXT1/EXT3 抢 GPIO3/48 是
> **预期行为**，PWM_EXT2 走空闲脚 GPIO47，用第 2 路测 A/C/D 不会触发该副作用。

---

## 8. 遗留（不在本次范围）

### 8.1 切 POSE 是原地冻结，可能冻在抬腿中途

见 §2.3 后果 3。这是**主舵机交接路径的固有行为**，不是本次这个 bug 的一部分。
真要做，需要在切 POSE 前先让步态退到站姿 —— 那是一个**新功能**，单独立项。

### 8.2 相关但独立的发现

同一次审计还发现另外三个问题，**与本文无关，另行处理**：

- `led_batt_stop()` 只 `vTaskDelete` + 关灯，没有 `rmt_driver_uninstall`、也没释放 GPIO48
- `voltage.read_v()` 改为直接返回 `bpuppy_led.batt_v()` —— 电压读数与电池 LED 监控任务耦合
- `frozen/pwm_ext.py` 模块注释关于"与电池检测同脚"的描述不准确
