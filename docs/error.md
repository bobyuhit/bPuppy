# bPuppy：不许改的设计 & 未解决的问题

> 这个文件只放两类东西：**故意的设计**（改了就坏）和**还没解决的问题**。
> 已修好的复盘不留在这里，去 git 历史翻。上一条是「只动扩展舵机，却把步态停掉了」
> （2026-09-17 发现；改法落在 `frozen/poses.py` 的 `_move_to()`，加 `has_main` 守卫，
> 板上按 A~E 实测通过）。
> 最近一次整理：2026-09-28 —— 新增 §4 运动平滑审计（八种变化的每帧跳变量），
> 订正 §2.2 第 2 条（顺序依赖的 setter 是两个不是一处；**同日修掉**
> `set_center` / `set_body_pose` 的组合校验，附新旧边界对照表），全文行号重新核对。

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
（`motion_task.cpp:500-504`：两次 `servo_group_add` + `servo_group_commit()`），
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
| **优雅停步**（四腿踩实站住） | `set_gait('stop')` → `GAIT_STOP` | `motion_task.cpp:227-257` 的 0.3s smoothstep 退到站姿 ✅ |
| **Python 临时接管舵机** | Python 写主舵机 | `motion_task.cpp:216` 跳过循环，原地冻结 ⚠ |

---

## 2. IK 限位

### 2.1 现在怎么限的（三层）

**第 1 层：写前校验 —— 唯一的"闸门"**

`motion_validate_params()`（`motion_task.cpp:606`）只被 `motion_set_params()` 调用，
**无条件跑**（2026-09-27 起去掉了 `stride != 0` 那层前置条件；同日 `set_lift` 并入 `set_params`，
`motion_set_lift` / `motion_check_params` 删除，所以现在这是唯一的调用点）。
它把足端**实际会走到的轨迹点**采样出来，逐点丢给 `ik_pos_check()`（`motion_task.cpp:556`）：

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
| `cal_ik(L1, L2)` | 各自 ∈ `[IK_LEN_MIN, IK_LEN_MAX]` = [1, 500] mm | `motion_task.cpp:766` |
| `set_body_dims(bl, bw)` | 同上范围 | `:782` |
| `set_joint_limits(...)` | `min < max` **且**整段落在舵机行程 [−35, 215] 内 | `:797` |
| `set_center(off)` | `\|off\| ≤ L1/2`（大腿的一半）**再加组合校验**（2026-09-28 起，见 2.2 第 2 条） | `:761` |
| `set_speed(speed)` | ∈ [0, 10]，NaN 安全写法 | `motion_set_speed` |
| `set_turn(turn)` | 无范围检查，**静默钳到 ±1**（现返回 `false` 告知已钳位） | `:983` |

判据收在 `geom_limits_valid()`（`:739`）—— 用 NaN 安全写法 `!(a >= lo && a <= hi)`，
所以 NaN 会落进"非法"一侧。从 NVS 读回老值时走的是**同一句**，写入口径一致。

**第 3 层：运行层兜底 —— 一律不报错，只保证狗不散架**

| 兜底 | 位置 | 效果 |
|---|---|---|
| 入口挡非法 L1/L2、非有限足端坐标 | `ik.c:31` 起 | 直接返回，不产生错误角度 |
| `d` 钳到 [6, 84] | `ik.c:57-58` | height 设成 100 也只是"腿比你设的短一截"，不报错 |
| hip/knee 角度钳到限位 | `ik.c:98-101` | ⚠ 限位本身 `min > max` 时全被压成 max，狗彻底不动 |
| stride 钳到 `max_stride` | `motion_task.cpp:445` | |
| 脉宽钳到 `[SERVO_ANGLE_MIN, SERVO_ANGLE_MAX]` | `servo_driver.c:101-103` | 最后一道网，静默 |
| 每帧最多 3°（`SERVO_MAX_DEG_PER_FRAME`） | `motion_task.cpp:54`，仅用于 `:494-497` | **只在 `is_stand` 时生效**，走着治不了 —— 代价见 §4.2 第 3 条 |

### 2.2 还没解决的 8 条（按危险程度排序；第 2 条 2026-09-28 已修，剩 KittenBlock 侧没接返回值）

**1. ⚠ NaN 全穿透** —— `stride` / `height` / `lift` 走 `mp_obj_get_float` 进来，
所有比较对 NaN 恒假：`fabsf(NaN) > max_stride` 假、`height < MIN_HEIGHT` 假、
`ik_pos_check` 里 `if (d < lo || d > hi)` 也假 → 判定"通过" → NaN 写进 `g_motion`。
运行时 `ik.c` 的钳位同样是 `if (a > max)` 写法，对 NaN 一样失效，最终 `(int)` 转换
NaN 是未定义行为。至少要在这三个入口加 `isfinite` 挡一道。

**2. ⚠ 顺序依赖的 setter 不止 `set_body_pose()`** —— 校验把 **5 个量**算进轨迹，
但**只有 3 个能触发校验**：`stride` / `height` / `lift`（走 `motion_set_params`）。
`center_offset` 和 `pitch` / `roll` 只是被**读**去算，自己没有闸门。

| setter | 自己的校验 | 反例 |
|---|---|---|
| `set_body_pose(pitch, roll)`（`motion_task.cpp:974`） | 无 | `pitch = −12°` → `z_pitch = 62.5·tan12° = 13.3 mm` → **前腿**（`leg_pair[0]/[2]` 拿 `+z_pitch`）`(x=±35, z=83.3)` → `d = 90.3 mm > 84` |
| `set_center(off)`（`:992`） | 只判 `\|off\| ≤ L1/2` = 20 mm | 先 `set_params(70, 30, 70)` 再 `set_center(20)` → 足端 `x = 55` → `d = 89.0 mm > 84` |

两边都**静默**：`ik_solve_2dof()` 把 `d` 钳到 [6, 84]（`ik.c:57-58`），
腿够不着就**走短一截**，不报错、不 NaN。而校验只在 `set_params` **被调用的那一刻**跑，
所以**顺序决定拦不拦得住**：先压姿态 / 先挪重心再设参数 → 拦得住；
反过来 → 静默发生，狗当场把腿折短 / 步长被悄悄吃掉。
（`set_body_pose` 旧的第 3 参 `yaw` 是死字段，2026-09-27 已删。）

⚠ **但 `set_center` 不只是"顺序"问题 —— 不用凑顺序就会撞。** 默认参数下
（stride 70 / height 70）够得着的边界是 `x ≤ √(84²−70²) = 46.4`，即 **center 最多只能到 11.4mm**，
而闸门放到 20mm —— **允许的范围里有 43% 是到不了的**。一条「设置重心 15」
（`x = 50` → `d = 86.0`）就超了，不需要先设参数再设重心那种刁钻顺序。
`geom_center_offset_valid()` 头上的注释把现象写得很准（"腿静默失去伸展 (狗歪着走)"）
—— 作者知道**现象**，只是选了一个拦不住**组合**的闸门：量级闸门管不了组合。
所以修的时候别只是把 0.5 调小（`L1/2` 改小会连带砍掉本来合法的用法 ——
stride=50 时 center=15 是能站住的，见下表），要让它进同一个 helper 一起算。

**✅ 已修复（2026-09-28）** —— 采样校验抽成 `traj_combo_bad(stride, height, lift, center, roll, pitch)`，
`set_center` / `set_body_pose` 也拿**新值**跑同一条轨迹，被拒就整组不写、返回 `false`
（`motion_task.cpp`；两个 mpy 绑定也跟着返回 bool，`set_center` 以前被 `mp_const_none` 吞掉了）。
离线复刻固件公式算出的新边界（**算的，不是量的**）：

| (stride, height) | 旧的 center 上限 | 新的 center 上限 | 卡在哪 |
|---|---|---|---|
| 70, 70（默认） | 20 | **11.4** | 够不着：`d=86.0` @ center=15 |
| 60, 70 | 20 | **16.4** | 够不着 |
| 50, 70 | 20 | **19.4** | 髋角：`hip=−0.4` @ center=20 |
| 40 / 30, 70 | 20 | **19.4** | 髋角（同上，后腿摆动中点 `hip<0`） |

| setter | 旧上限 | 新上限（默认 stride 70 / height 70） |
|---|---|---|
| 俯仰 | 无限 | **±5.8°**（−5° 仍合法 —— 源码注释里"实测在用"的那个值刚好过） |
| 横滚 | 无限 | **±6.2°** |

⚠ **这是行为变更，会撞到现有用法**：默认步长下「机身姿态」压到 −10° 会被**拒**（以前是静默折腿），
「设置重心 15」也会被拒。绕法是先减小步长：`stride=40` 时俯仰 −10° 就合法了。
KittenBlock 那两个块现在**还看不见拒绝**（pycode 没接返回值，见下面"还没做"）。

**残留的一个洞**：NVS 里的 `center_offset` 载入时**只判量级**（`motion_load_geometry`），
刻意如此 —— 载入发生在 stride/height 还是默认值的时候，若按组合判，一个当初合法的 center
会在用户改小步长后于下次开机被静默丢掉。代价是 NVS 里可能存着一个"跟默认步长组合起来
够不着"的值；好处是它下次调 `set_params` 时会以"被拒 + 日志点名 center=15"的形式**自己暴露出来**，
而不是继续静默吃步幅。

正解是同一个：把姿态补偿和重心偏移从 `motion_validate_params` 抽成
`(stride, height, lift, roll, pitch, center_offset)` 的 helper，
让这两个 setter 也拿**新值**跑一遍，并返回 `bool`。**⇒ 2026-09-28 已做，见本节末尾。**

**还没做的**：KittenBlock 的「机身姿态」/「设置重心」两个块的 pycode 还是
`bpuppy_motion.set_body_pose([PITCH], [ROLL])` 这种**不接返回值**的写法（`kblock.json5:103` / `:116`），
所以被拒时用户看到的是"点了没反应"。C 侧和 mpy 已经能报出 `False` 了
（`set_center` 以前连 `bool` 都被 `mp_const_none` 吞掉，这次一并修了），
只差积木那句 `_ok = ...; if not _ok: voice.say(*voice.SND_YING)` —— 跟 `6b2f6cb` 给
「运动参数」加的是同一套。改动小，但要让用户**重新导入扩展**，单独立项。

**3. ⚠ `ik_pos_check()` 是 `ik.c` 公式的手抄副本**（`motion_task.cpp:556` 注释自述）——
两边没有共享代码、没有一致性测试。`ik.c` 的公式改了而这里忘了跟，闸门就静默失灵，
而且失灵的后果是"以为校验过了"。

**4. `set_omega()` 无任何校验**（`:959`）—— 相位增量是 `omega·speed·0.02`/帧，
写多大走多快，没有上限。它**没有任何调用者**，2026-09-27 复查时仍未动。
`set_turn()` 也没有范围检查，但超范围会**静默钳到 ±1**（行为不变，现在返回 `false` 告知）。

**5. 5 点采样是抽查，不是证明** —— d 的极值恰好落在采样点上
（stride=70 / height=70 / lift=30 → 各点 `d` = 78.2 / 51.8 / 40.0 / 51.8 / 78.2），
对 d 够用；但**髋角**没有这个保证。5 点之间如果有个更陡的中间态，闸门看不见。

**6. 低站姿的真实限制是髋角，不是 `MIN_HEIGHT`** —— 实测 `height = 20` 时
hip 已经 180.9° > 180，是 `ik_pos_check` 的**髋角判定**拦住的，不是 `MIN_HEIGHT`。
"能站多低"取决于 joint limits 和 L1/L2，那个常量给不出这个数。

**7. 顺序依赖 —— ✅ 已修复（2026-09-27）**

原状：`set_lift` / `set_params` 都跟**当时的其余参数**一起判，于是"先降 lift 再改 stride/height"
和反过来结果不同。典型：`set_params(sp, 0, 20)` 被拒是因为当场 `lift = 30`，`20 − 30 = −10` 入地
—— 想改到位得先 `set_lift(0)`。KittenBlock 的「运动参数」块为此打了 `set_lift → params → 读回 →
补一次` 的三段式补丁。

修法：stride / lift / height 是**同一条足端轨迹的三个维度**（地面位移、抬起来多高、站多高），
本来就必须联合校验 —— 但也可以**同一个来源**。把 lift 并进 `motion_set_params(stride, lift, height)`
第 2 参后，三个值一次判、一次写入，互相之间不再有"旧值"可用，耦合从根上消失。
`motion_set_lift` / `motion_check_params` 一并删除。

剩下的顺序依赖在 `set_body_pose` / `set_center`，见上面第 2 条。

**8. 过校验 ≠ 姿态好看** —— 0.85 这个 stride 系数、`center_offset ≤ L1/2`、
`d ≥ |L2−L1|+1` 的"+1"，都是人定的舒适区，不是物理极限。反过来说，
参数在范围内也不代表走起来稳（重心、摩擦、舵机力矩都不在校验里）。

---

## 3. 其他遗留

### 3.1 切 POSE 是原地冻结，可能冻在抬腿中途

Python 写主舵机 → `motion_set_mode(MODE_POSE)` → 把 `enabled = false` /
`emergency_stop = true` 翻掉 → `motion_task_main()` 里 `motion_task.cpp:216` 那个 `if`
每 20ms 睡一觉、**跳过整段控制循环**。

注意这跟 `set_gait('stop')` **不是一回事**：后者（`GAIT_STOP`）会走
`motion_task.cpp:227-257` 那段 0.3s smoothstep 先退到站姿，而切 POSE 是**立刻停手** ——
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

---

## 4. 运动平滑（2026-09-28 审计）

> 起因：站立《-》运动、运动中途前后换向、运动中途改高度，这三类变化要不要平滑。
> **下面所有角度都是离线按本仓库自己的公式和参数算出来的，不是板上量的** ——
> 舵机对阶跃指令的真实响应（有无机械缓冲、会不会堵转）没测过。

### 4.1 对账表

尺子取"每帧髋/膝最多变多少度"，即相邻两帧舵机**目标角**的跳变。

| 变化 | 平滑手段 | 覆盖 | 单帧跳变 髋/膝 |
|---|---|---|---|
| 站立 ↔ 运动 | `pose_trans` 1/2 + 0.3s smoothstep | 全部步态 | **1.7° / 0.7°** ✅ |
| 运动中途前后换向 | `g_stop_decel` / `g_decel_sign` / `g_stride_smooth` | **仅 go** | go 32.2° / 55.3°；walk·trot **48.6° / 64.2°** |
| 运动中途改站高 | 无 | — | 31.5° / 42.4° |
| 运动中途改抬脚 | 无 | — | 27.4° / 29.4° |
| 运动中途改俯仰 | 无 | — | 33.1° / 57.4°（pitch 0 → −10°） |
| 运动中途改横滚 | 无 | — | 12.5° / 20.5° |
| 运动中途改重心 | 无 | — | 42.0° / 56.2° |
| 运动中途改转弯率 | 无 | — | 45.3° / 64.2° |

同期**正常走路本身**每帧变多少（对照用，别拿站姿的尺子去量走路）：

| 场景 | 单帧 髋/膝 |
|---|---|
| go speed 2.5 | 14.9° / 18.2° |
| **go speed 4 ← 峰值** | **23.2° / 28.7°** |
| go speed 10 | 12.9° / 10.5° |
| 站着时的硬上限 `SERVO_MAX_DEG_PER_FRAME` | 3.0° |

### 4.2 三条结论

**1. 站↔运动的平滑是过头，不是不足。** 1.7°/帧 只有站姿限速（3°）的一半，
而正常走路是 15~23°/帧 —— 起步那 0.3s 慢到几乎看不出来。不是坏事，
但"站↔运动没做平滑"这个印象是错的。

**2. go 的"减速到零"缺了它换向会崩。** 换向时 `eff_duty + eff_gap` 决定用 `offsets_fwd`
还是 `offsets_rev` 那组相位偏移 —— 等于四条腿的落足次序**整体换一套**。go 靠
`g_stride_smooth` 把步长先退到 0 再反向，**步长 0 时两套时序在位置上没差别**，
切换就无从产生跳变。但这不是免费的：

- **只有 go 有**。`stop_smooth = (gait == GAIT_GO) ? g_stride_smooth : 0.0f`，
  于是 walk / trot 的"减速完成"条件**当帧就成立**，`g_stop_decel` / `g_reverse`
  在腿循环之前被清掉 —— **零帧**方向保持，48.6° / 64.2° 一步拉过去。
- go 自己也在步长涨回 14mm 之后才切相位偏移，残余 32.2° / 55.3°。
  想再小，得让 `need_rev` 那个判据也等步长回到 0 附近，
  而不是等"减速完成"这个事件。

**3. 站高站着改被限速、走着改不被 —— 同一件事两个待遇。** 限速那三行
（`motion_task.cpp:494-497`）写在 `if (is_stand)` 里面：站着改站高 3°/帧（0.28s 走完），
走着改站高**一帧跳 42°**（膝）。§2.1 第 3 层表最后一行早已记了
"只在 `is_stand` 时生效，走着治不了" —— 这里补上它的**代价**：
不是"少一层保护"，是 **42° 对 3°**。

### 4.3 设计结论：有"姿态过渡"，没有"参数过渡"

`pose_trans` 过渡的是**姿态**（站立 ↔ 运动这种状态切换）。
而站高 / 抬脚 / 俯仰 / 横滚 / 重心 / 转弯率这六个**参数**是裸写的 ——
setter 一调，下一帧就拿新值算 IK，六个里没有一个有过渡。

正解方向：加一层**参数缓动**（~0.3s，跟 `pose_trans` 同量级），
每帧让"当前值"朝目标值靠近、IK 用当前值。顺带白捡两件事：

- 转弯率穿 0 变无害 —— `turn` 的正负号是左右腿步长的缩放比例，穿 0 就是换转方向，
  跟 go 的减速停是**同一个道理**（值过 0 时另一套时序在位置上无差别）；
- `need_rev` 的相位偏移切换同样因为步长已缓动到 0 而变无害。

⚠ **错解**：把 `is_stand` 那 3°/帧推广到"所有时候"。正常走路本身是 15~23°/帧，
限到 3° 等于把走路废掉。3°/帧是**站姿**的合理速度，不是通用速度。
