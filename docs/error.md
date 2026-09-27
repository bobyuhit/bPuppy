# bPuppy：不许改的设计 & 未解决的问题

> 这个文件只放两类东西：**故意的设计**（改了就坏）和**还没解决的问题**。
> 已修好的复盘不留在这里，去 git 历史翻。上一条是「只动扩展舵机，却把步态停掉了」
> （2026-09-17 发现；改法落在 `frozen/poses.py` 的 `_move_to()`，加 `has_main` 守卫，
> 板上按 A~E 实测通过）。
> 最近一次整理：2026-09-27 —— 删掉已修复的正文，补上 IK 限位现状与待办。

---

## 1. 不许改的设计

最容易顺手"修错"的地方 —— 下面这些都是**故意的**，改了就坏。

### 1.1 切 POSE 的钩子只在**写**路径上，读不切

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

### 1.2 这根钩子是**交接棒**，不是 bug

motion 任务在 50Hz 里也在驱动**同样这 8 路** LEDC 通道
（`motion_task.cpp:491-495`：两次 `servo_group_add` + `servo_group_commit()`），
走的是 **C 层**的 `servo_group_add/commit` —— **那条路上没有钩子**。

如果 Python 这时也能同时写同样 8 路，两边就会互相打架。
所以"Python 动主舵机 = 交出控制权"是**正确规则，保留不动**。

**不对称之处：**

> 同一个 `servo_group_commit()`，从 C 调不停步态，从 Python 调解停 ——
> 差别只因为钩子挂在 MicroPython 包装层上。

### 1.3 ⚠ `motion_set_mode` 这名字容易误导

它**根本不碰** `g_motion.gait` / `g_phase`，只是把 `enabled` / `emergency_stop`
两个标志翻了一下。设计意图是**"暂停 + 交棒"**，不是"停步态"。

| 想要的效果 | 正确做法 | 机制 |
|---|---|---|
| **优雅停步**（四腿踩实站住） | `set_gait('stop')` → `GAIT_STOP` | `motion_task.cpp:229-262` 的 0.3s smoothstep 退到站姿 ✅ |
| **Python 临时接管舵机** | Python 写主舵机 | `motion_task.cpp:211` 跳过循环，原地冻结 ⚠ |

---

## 2. IK 限位

### 2.1 现在怎么限的（三层）

**第 1 层：写前校验 —— 唯一的"闸门"**

`motion_validate_params()`（`motion_task.cpp:597`）被 `motion_set_params()` 和
`motion_set_lift()` 调用，**无条件跑**（2026-09-27 起去掉了 `stride != 0` 那层前置条件）。
它把足端**实际会走到的轨迹点**采样出来，逐点丢给 `ik_pos_check()`（`motion_task.cpp:547`）：

- 采 5 个 ease 点（0 / 0.25 / 0.5 / 0.75 / 1）：
  - `x = −stride/2 + stride·ease + center_offset`
  - `z = height − lift·sin(ease·π)`
  - 再**逐腿**叠机体姿态补偿：左腿 `−z_roll` / 右腿 `+z_roll`，前腿（0/2）`+z_pitch` / 后腿 `−z_pitch`
  - `z_roll = body_half_w·tan(roll)`、`z_pitch = body_half_l·tan(pitch)`
- 每点判三条：`d = √(x²+z²)` 必须落在 `[|L2−L1|+1, L1+L2−1]` = **[6, 84] mm**
  （L1=40、L2=45）；hip ∈ `[0, 180]`；knee ∈ `[10, 170]`（后两者来自 NVS，可改）
- 另两条硬边界：`|stride| ≤ 2·body_half_l·0.85` = **106.25 mm**、`height ≥ MIN_HEIGHT` = **15**
- 补偿后 `z < 0`（脚插到地下）在循环后统一报一次，不刷屏

任一条不过 → **整组参数不写入**，`get_params()` 保持原值。

**第 2 层：每个几何/标定 setter 各自的范围**

| setter | 判据 | 位置 |
|---|---|---|
| `cal_ik(L1, L2)` | 各自 ∈ `[IK_LEN_MIN, IK_LEN_MAX]` = [1, 500] mm | `motion_task.cpp:751` |
| `set_body_dims(bl, bw)` | 同上范围 | `:767` |
| `set_joint_limits(...)` | `min < max` **且**整段落在舵机行程 [−35, 215] 内 | `:784` |
| `set_center(off)` | `\|off\| ≤ L1/2`（大腿的一半） | `:746` |
| `set_lift(lift)` | 与第 1 层同一套采样校验 | `:952` |

判据收在 `geom_limits_valid()`（`:726`）—— 用 NaN 安全写法 `!(a >= lo && a <= hi)`，
所以 NaN 会落进"非法"一侧。从 NVS 读回老值时走的是**同一句**，写入口径一致。

**第 3 层：运行层兜底 —— 一律不报错，只保证狗不散架**

| 兜底 | 位置 | 效果 |
|---|---|---|
| 入口挡非法 L1/L2、非有限足端坐标 | `ik.c:31` 起 | 直接返回，不产生错误角度 |
| `d` 钳到 [6, 84] | `ik.c:57-58` | height 设成 100 也只是"腿比你设的短一截"，不报错 |
| hip/knee 角度钳到限位 | `ik.c:98-101` | ⚠ 限位本身 `min > max` 时全被压成 max，狗彻底不动 |
| stride 钳到 `max_stride` | `motion_task.cpp:436` | |
| 脉宽钳到 `[SERVO_ANGLE_MIN, SERVO_ANGLE_MAX]` | `servo_driver.c:101-103` | 最后一道网，静默 |
| 每帧最多 3°（`SERVO_MAX_DEG_PER_FRAME`） | `motion_task.cpp:49`，仅用于 `:485` | **只在 `is_stand` 时生效**，走着治不了 |

### 2.2 还没解决的 8 条（按危险程度排序）

**1. ⚠ NaN 全穿透** —— `stride` / `height` / `lift` 走 `mp_obj_get_float` 进来，
所有比较对 NaN 恒假：`fabsf(NaN) > max_stride` 假、`height < MIN_HEIGHT` 假、
`ik_pos_check` 里 `if (d < lo || d > hi)` 也假 → 判定"通过" → NaN 写进 `g_motion`。
运行时 `ik.c` 的钳位同样是 `if (a > max)` 写法，对 NaN 一样失效，最终 `(int)` 转换
NaN 是未定义行为。至少要在这三个入口加 `isfinite` 挡一道。

**2. ⚠ `set_body_pose()` 完全不校验**（`motion_task.cpp:965`）—— roll/pitch/yaw 直接进
`g_motion`。实测 `pitch = −12°` → `z_pitch = 62.5·tan12° = 13.3 mm` → 后腿
`(x=±35, z=70+13.3)` → `d = 90.3 mm > 84`。而校验只在 `set_params`/`set_lift`
**被调用的那一刻**跑，所以**顺序决定拦不拦得住**：先压姿态再设参数 → 拦得住；
先设参数再压姿态 → 静默发生，狗当场把腿折短。

**3. ⚠ `ik_pos_check()` 是 `ik.c` 公式的手抄副本**（`motion_task.cpp:546` 注释自述）——
两边没有共享代码、没有一致性测试。`ik.c` 的公式改了而这里忘了跟，闸门就静默失灵，
而且失灵的后果是"以为校验过了"。

**4. `set_omega()` 无任何校验**（`:946`）—— 相位增量是 `omega·speed·0.02`/帧，
写多大走多快，没有上限。`set_turn()` 同样没有任何范围检查。

**5. 5 点采样是抽查，不是证明** —— d 的极值恰好落在采样点上
（stride=70 / height=70 / lift=30 → 各点 `d` = 78.2 / 51.8 / 40.0 / 51.8 / 78.2），
对 d 够用；但**髋角**没有这个保证。5 点之间如果有个更陡的中间态，闸门看不见。

**6. 低站姿的真实限制是髋角，不是 `MIN_HEIGHT`** —— 实测 `height = 20` 时
hip 已经 180.9° > 180，是 `ik_pos_check` 的**髋角判定**拦住的，不是 `MIN_HEIGHT`。
"能站多低"取决于 joint limits 和 L1/L2，那个常量给不出这个数。

**7. 顺序依赖** —— `set_lift` / `set_params` 都是跟**当时的其余参数**一起判，
所以"先降 lift 再改 stride/height"和反过来结果不同。典型的：
`set_params(sp, 0, 20)` 被拒是因为当场 `lift = 30`，`20 − 30 = −10` 入地 ——
想改到位得先 `set_lift(0)`。

**8. 过校验 ≠ 姿态好看** —— 0.85 这个 stride 系数、`center_offset ≤ L1/2`、
`d ≥ |L2−L1|+1` 的"+1"，都是人定的舒适区，不是物理极限。反过来说，
参数在范围内也不代表走起来稳（重心、摩擦、舵机力矩都不在校验里）。

---

## 3. 其他遗留

### 3.1 切 POSE 是原地冻结，可能冻在抬腿中途

Python 写主舵机 → `motion_set_mode(MODE_POSE)` → 把 `enabled = false` /
`emergency_stop = true` 翻掉 → `motion_task_main()` 里 `motion_task.cpp:211` 那个 `if`
每 20ms 睡一觉、**跳过整段控制循环**。

注意这跟 `set_gait('stop')` **不是一回事**：后者（`GAIT_STOP`）会走
`motion_task.cpp:229-262` 那段 0.3s smoothstep 先退到站姿，而切 POSE 是**立刻停手** ——
若恰有一条腿抬在空中，它就保持悬空定住。

这是**主舵机交接路径的固有行为**。真要改成"先退站姿再交棒"，那是一个**新功能**，
单独立项。

### 3.2 相关但独立的发现

- `led_batt_stop()` 只 `vTaskDelete` + 关灯，没有 `rmt_driver_uninstall`、也没释放 GPIO48
- `voltage.read_v()` 改为直接返回 `bpuppy_led.batt_v()` —— 电压读数与电池 LED 监控任务耦合

### 3.3 每次 `git commit` 都会触发 265 步真重编（ccache 救不了）

**现象**：只改了 `drivers/imu_driver.c` 一个文件，`bash build.sh` 却跑了 1409 步、
耗时约 54 分钟。而 `docs/README.md:67` 写的是「增量编译从 1356 步降到 ~10 步，几秒完成」。

**先排除的**：ccache 本身没问题。用 `compile_commands.json` 里**原封不动**的命令跑两次：

```
第 1 次  Result: cache miss
第 2 次  Result: cache hit (direct)
```

ccache 3.7.7（镜像 `espressif/idf:v5.1.2` 自带，走 ESP-IDF 的 `CCACHE_ENABLE=1`）
工作完全正常 —— 注意它用 `RULE_LAUNCH_COMPILE` 挂载，所以 `compile_commands.json`
和 `CMakeCache.txt` 里**看不到** `ccache` 前缀，但 `build.ninja` 的命令里有。
排查时别据此以为 ccache 没接上。

**根因**：`build.ninja` 把 **`.git/refs/heads/master` 当成了依赖**：

```
$ ninja -C build -n -d explain
ninja explain: output build.ninja older than most recent input
               /workspace/.git/refs/heads/master (1790352448289128600 vs 1790352822453215600)
[0/1] Re-running CMake...
```

`.git` 就在挂进容器的 `/workspace` 里，所以**每次 commit 都会改这个文件的 mtime**，
于是 ninja 重跑 CMake；而 `py/makeversionhdr.py` 会把 `git describe` 的结果写进
`build/genhdr/mpversion.h`：

```
#define MICROPY_GIT_TAG "v3.0-41.g0af06b8.dirty"   ← 每 commit 都变
#define MICROPY_BUILD_DATE "2026-09-25"            ← 每跨一天都变
```

`mpversion.h` 被 `py/mpconfig.h` 包含，等于被所有 MicroPython 源文件包含。
它一变，`ninja -d explain` 显示：

```
239  build/genhdr/qstrdefs.generated.h is dirty
216  build/genhdr/root_pointers.h is dirty
  6  build/genhdr/moduledefs.h is dirty
  7  build/genhdr/mpversion.h is dirty
```

合计 **265 步**。这一步 ccache **救不了** —— 不是缓存失效，是源码文本真的变了，
必须重编。实测：重跑一次 CMake 本身只要几秒，重跑之后脏步数正好 **265**（不是 1409）。

**修法（已验证，未实施）**：`py/makeversionhdr.py:89` 和 `:101` 本来就支持环境变量覆盖：

```python
git_tag = None
if "MICROPY_GIT_TAG" in os.environ:          # ← 设了就完全跳过 git 调用
    git_tag = os.environ["MICROPY_GIT_TAG"]
if git_tag is None:
    git_tag = get_version_info_from_git(repo_path)
...
build_date = datetime.date.today()
if "SOURCE_DATE_EPOCH" in os.environ:        # ← 钉死日期
    build_date = datetime.datetime.utcfromtimestamp(int(os.environ["SOURCE_DATE_EPOCH"])).date()
```

脚本本身**内容没变就不重写文件**（`:114` 起 `write_file` 判断），所以只要
`build.sh` 的 `docker run` 加两个 `-e` 就能让 `mpversion.h` 恒定居。

实测（容器内直接跑 `makeversionhdr.py`）：

```
不设环境变量 → MICROPY_GIT_TAG "v3.0-42.gd3508df.dirty"   （commit 一变就变）
设两个变量   → MICROPY_GIT_TAG "bPuppy-fixed"
               MICROPY_BUILD_DATE "1970-01-01"
               连跑两次不重写文件（mtime 不变）
```

**取舍（2026-09-26 决定：先不改）**：钉死 tag 的代价是启动 banner 不再显示
commit 号（`MicroPython v3.0-41.g0af06b8.dirty on 2026-09-25` 是排查"烧进去的
到底是不是这份代码"时的直接依据）。要提速又不想丢 commit 号，可以让
`build.sh` 写成 `MICROPY_GIT_TAG="${MICROPY_GIT_TAG:-bPuppy}"`，平时走固定值、
需要时可 `MICROPY_GIT_TAG=v3.0-42.gd3508df bash build.sh` 手动带真 tag 全量重编。

**未解释的部分**：2026-09-26 那次 1409 步比 265 大得多，多出来的约 1144 步是
ESP-IDF 组件全量重编（`xtensa` / `efuse` / `driver` … 从第 1 步就开始）。这是
**一次性**的，之后 `ninja -n` 只剩 265 步，触发源没有定位到 —— 不编故事。
若再次出现，第一步查 `ninja -C build -n -d explain` 里 "is dirty" 的都是谁。

**副作用**：`docs/README.md:67` 那句「增量编译从 1356 步降到 ~10 步，几秒完成」
是**不准确**的 —— 1356 → ~10 只在"没有 commit、也没跨天"的理想情况下成立。
