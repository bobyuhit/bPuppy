/*
 * bDog 运动控制任务 — 50Hz 实时步态控制
 *
 * 步态统一框架:
 *   给定 duty (摆动占比) 和 gap (同侧间隙), 自动推导四条腿的相位偏移。
 *   walk 和 trot 的区别纯靠参数切换。
 *
 *   每 20ms 执行一次:
 *     1. 全局相位累加
 *     2. 对每条腿: 查偏移 → 足端轨迹 → IK → 批量舵机同步
 */

#include "motion_task.h"
#include "ik.h"
#include "servo_driver.h"
#include "imu_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include <math.h>
#include <string.h>

// 扩展舵机 (PWM_EXT1/2/3, MCPWM) 的跌倒释放/恢复入口
extern "C" void pwm_ext_stop_all(void);
extern "C" void pwm_ext_rearm_all(void);

static const char *TAG = "motion";

/* ---- 任务参数 ---- */
#define MOTION_STACK       4096
#define MOTION_PRIORITY    6
#define MOTION_CORE        0
#define MOTION_LOOP_MS     20      // 50Hz (舵机 PWM 同频)

#define MIN_HEIGHT        15.0f    // 最低站立高度 (mm), 过低腿折叠干涉

#define TWO_PI             (2.0f * (float)M_PI)
#define TURN_STRIDE_FACTOR 0.15f
#define SPEED_FOLLOW_STEP 3.0f  // 每半周速度跟随最大变化量
// GO 两个插值端点的抬脚高度 (mm) —— 跟 duty/gap/stride/height 一样按 speed 插值,
// 4~6 之间线性过渡。低速端 30 (= KittenBlock 里 _lift 的初值 / LIFT_DEFAULT),
// 高速端 5: 小跑步长收到 50、duty 到 0.40, 抬脚压低换更小的上下起伏。
#define GO_LIFT_LOW       30.0f
#define GO_LIFT_HIGH      5.0f

/* ---- 起步过渡 (任意姿态 → 站姿 → 满额步态) ---- */
// 阶段 A: 任意姿态 → 站姿, 每帧 SERVO_MAX_DEG_PER_FRAME 度, 跑到收敛为止
#define STAND_UP_TOL      0.3f   // 收敛判据 (deg) —— 与 Python poses._move_to 同值
// 阶段 B (步长淡入淡出): 起步 0→1, 停步/换向 1→0 —— 三件事共用同一个系数, 见 g_stride_fade.
// 窗口用**累计步态周期数**做自变量 (与速度解耦): 无论 speed 多少都是 fade_in_turns 个周期.
// ★ 淡出方向 (1→0) 至少要 1 个整周期: 支撑相锚定后脚一落地就不再回 x=0, 必须让
//   **每条腿都在步长→0 之后重新落地一次**, 四脚才会都收敛到 x≈0 的站姿等价点.
#define FADE_IN_TURNS_WALK   1.0f   // 淡入 walk 段 (duty 0.20). ★暂时与 TROT 同值 ⇒ 下面的分档失效 (试手感)
#define FADE_IN_TURNS_TROT   1.0f   // 淡入 trot 段 (duty 0.40): 对角两腿同时离地 ⇒ 原地踏步晃, 窗口要短

/* ---- 淡出专用: 统一 1.25 周期 + 平方曲线 ----
 * (只有"目标速度归零"这一条路走它 —— 停步走回正步, 换向走对齐点翻转, 都不经过这里) */
// 淡出**不能**跟着 fade_in_turns 走. 跟着走时 trot/GO≥6 用 1.0 太短 —— 淡完那刻四脚离
// 站姿还差 **39~40mm**, 全交给最后 0.3s 的 pose_trans=2 缓动去滑 ⇒ ≈130mm/s 搓地.
// 平方曲线 fade = (1-p)² 把同一段时间重新分配成"前段就小下去, 后段只剩一点点" ⇒
// **同样时长里前冲更短, 且淡完那刻残留小一个数量级**.
// 离线扫遍 duty 0.20/0.30/0.40 × speed 1~10 (每格扫 240 个停止相位取最坏):
//   线性 2.0/1.0 → walk 1.74~2.78s / 前冲 86mm / 残留 4.8mm, 但 trot·GO 残留 34~40mm ⚠
//   平方 1.25    → walk 1.16~1.84s / 前冲 35mm / 残留 4.3~4.5mm, trot·GO 残留 2.2~4.0mm
// 再短不行: 平方 1.0 时残留回到 22~28mm, 又会搓地 ⇒ 1.25 是"又短又干净"那个点, 三种步态共用.
#define FADE_OUT_TURNS        1.25f

/* ---- 运动实参的跟随限速 ----
 * 用户在运动中改 站高 / 抬脚 / 俯仰 / 横滚 / 重心 时, 目标值立刻写进 g_motion, 但
 * **喂给腿循环的实参每帧只走固定一步** —— 目标不会当帧砸到腿上.
 *
 * 为什么需要: 这五个量直接改足端轨迹 (height→z 基准, lift→摆动相抬升,
 *   pitch/roll→逐腿 z 偏置, center→x 常量偏置). 当帧生效 = 一帧跳 27~57°(髋/膝),
 *   而正常走路摆动腿才 15~23°/帧 ⇒ 看着像抽搐. 而站着改同样这几个参数反而是平滑的
 *   (有 3°/帧 的限制器兜着) —— 同一件事两个待遇, 这一层就是来拉平的.
 *
 * 为什么用固定步长而不是"按比例 / 定时到位": 变化幅度差一个数量级 (改站高 5mm vs 30mm).
 *   按比例分配会让小变化慢得没道理; 固定步长天然自适应 —— 小变化几帧到位, 大变化也不会变猛.
 * 为什么限的是"参数"而不是"关节": 全局限 3°/帧会把走路本身废掉 (摆动腿正常 15~23°/帧),
 *   见 docs/error.md §4.2 第 3 条.
 *
 * 步数按"每帧 ≈ 3° 等效"反推 (error.md §4.1 的离线实测: 站高 30mm ≈ 髋 31.5°、
 *   抬脚 25mm ≈ 27.4°、俯仰 10° ≈ 33.1°) ⇒ 典型变化量都在 0.25~0.3s 到位,
 *   与 pose_trans 的 0.3s 同量级.
 *
 * ★ 转弯率**在此列** (2026-10-14): 原以为"它只是左右步长的缩放比例, 不用管" —— 那
 *   只对**支撑腿**成立 (锚定吸收). **摆动腿**的 x = x_lift + (S/2 - x_lift)*ease 里
 *   S 当帧生效, turn 一变就跳, 台阶 ∝ eff_stride*Δturn. 航向锁定闭环 20Hz 频繁写
 *   turn 把它放大成可见现象, 这一层把它按帧摊开.
 *   与闭环低通 (heading_follow 的 TURN_LPF_A) 的分工: **低通压噪声的抖动, 限速压
 *   变化的速度** —— 噪声幅度远小于限速步长, 低通不能省.
 *
 * ★ 步长**不在此列** —— 支撑相锚定之后它是"身体快慢"而不是"足端位置", 改了脚不会挪.
 * ★ 顺带治一个: GO 的抬脚由 speed 插值算出, 而 speed 每半周期能变 ±3 ⇒ 抬脚一次能跳
 *   25mm (速度跨过 4~6 时). 这一层放在 GO 分支**之后**, 那一项也一并抹平.
 */
#define RAMP_HEIGHT_STEP   2.0f    // mm/帧  30mm → 15 帧 (0.3s)
#define RAMP_LIFT_STEP     2.0f    // mm/帧  25mm → 13 帧 (0.25s)
#define RAMP_CENTER_STEP   1.5f    // mm/帧  20mm → 14 帧 (0.28s)
#define RAMP_ATTITUDE_STEP 0.7f    // deg/帧 10°  → 15 帧 (0.3s)
#define RAMP_TURN_STEP     0.04f   // /帧   turn 0→1 约 25 帧 (0.5s);
                                   // 摆动腿单帧台阶 = eff_stride*0.04 ≈ 2.8mm@S70 / 4.2mm@S106

/* ---- 相位联动摆动 (body sway) ----
 * 为什么需要: walk 抬起任意一条腿时, 支撑三足的斜边几乎从机身中心穿过 ——
 * 四脚站在髋正下方 ⇒ 站姿是矩形 ⇒ 那条对角线过中心。逐相位核算
 * (gait_sim/sway_solver.py, 口径与 ik.c 同源):
 *   t=0.00 (LH 抬) / t=0.44 (LF 落) / t=0.50 (RH 抬) / t=0.94 (RF 落)
 *   —— 重心到支撑边界只有 **1.98mm** (默认 S70/H70/L30, 重心在正中时)。
 *   两个"落地"瞬间最容易被忽略 (只查抬脚瞬间会漏掉)。
 * 做法: 给整幅足端图案叠加一个前后平移:
 *     sway_x(t) = -A · dir · cos(4π(t - δ)),   t = g_phase/2π (0 = LH 抬脚, 正向)
 *   ★ 足端只能前后动 —— 本机构 ik 只有 (x,z), 足端 y 恒为 ±body_half_w
 *     (见下面 roll 补偿的注释), 左右方向没有可用自由度。
 *   符号: 足端图案 +Δ ⇒ 机身/重心相对支撑 -Δ ⇒ 重心沿 +A·cos 方向挪。
 *   A>0 时后腿抬的瞬间重心前移、前腿落的瞬间重心后移, 正对上面四个临界点。
 *   δ=0.095 由求解器给出: 让 t=0 与 t=0.44 两个瞬时各得 ±0.368A 的裕量 (相等)。
 *   逆走: 足端图案是正向的镜像 ⇒ 摆动随 direction 取反。
 * 幅值受**两条**约束, 都要看:
 *  ① 可达预算 (足端前伸/后伸端点) —— 组合校验拦它: S70/H70/L30 下 A≤22 能过;
 *     低站高 + 大间距 (如 H45/sp66) 时可达余量只剩 2.5mm, A 只能到 ~6。
 *  ② ★ **步速** (2026-10-11 实测补): 摆动等效于让机身相对支撑前后挪, 速度峰值 = A·4π/T;
 *     而步行本身的速度 = stride/((1-duty)·T) —— 同一个 T ⇒ **比值只取决于 A/stride**:
 *         不倒退 ⇒ A ≤ stride/(4π(1-duty)) ≈ 0.0995·stride   (S70 → 7.0mm)
 *     A=12 (S70) 时摆动速度 120mm/s > 步行 70mm/s ⇒ 机身约 40% 的时间在**倒退**
 *     ⇒ 实测观感"每一步停一下/一卡一卡"。3~7 是平滑区间; 换波形救不了 (锯齿"慢退快冲"
 *     要拿到同样裕量需 ~1g 机身加速度, 脚会打滑)。
 * ⚠ 口径注: 上面 1.98mm 是"摆动腿当完全不存在"的算法; 四个临界瞬间摆动腿还贴地
 *   (0~3mm), 真"裸奔"的最低裕量是 t≈0.4 的 **4.6mm** (另一腿离地 ~10mm 时)。
 *   按有效口径: A=0 → 4.6mm; A=5 → ~7.5mm; A=12 → ~9.5mm (代价见②)。
 * 生效范围: 只在 walk 族 (walkness = (0.5-duty-gap)/0.26): walk=1, trot/GO 高速段=0,
 *   GO 4~6 段线性跟随; 再乘步长淡入淡出系数 ⇒ 起步/停车/回正自动收放。 */
#define SWAY_DELTA          0.095f // 相位提前量 (周期): 两个临界瞬时增益相等 (求解器)
#define RAMP_SWAY_STEP      0.5f   // mm/帧 (满幅 12mm → 24 帧 ≈ 0.5s)
#define SWAY_DIR_STEP       0.10f  // 换向时方向符号斜坡 /帧 (≈0.2s, 过零无跳变)
/* SWAY_DEFAULT / SWAY_MAX_AMP 在 motion_task.h (mpy 绑定也要用) */

/* ---- 回正步 (停步收尾) ----
 * 停步不走淡出了: 收停信号后**保持当前步态**走到下一个"四腿全踩地"窗口, 在那里把步长
 * 硬切到 0, 然后进 GAIT_REPOS —— 身体不再移动, 由**对角两两抬脚**把四腿挪回 x=0.
 *
 * 为什么能在那一刻硬切步长: 全踩地时**没有腿在空中**, 所以不存在"摆动腿落点目标当帧塌掉"
 * 的甩腿问题 (离线扫描过: 淡出窗口短于一个摆动相就会甩). 这是选这个时机唯一也是关键的理由.
 *
 * 为什么对角抬是安全的: 对角抬时身体中心偏离支撑线 16.5mm, 但**抬起的脚离支撑线 44.2mm**
 * ⇒ 身体歪到 lift/44.2 弧度时抬起的脚就触地把它止住。**抬脚高度本身就是歪角的安全阀**:
 *   REPOS_LIFT 5mm → 最多歪 6.5° / 身体下沉 ~1.9mm;  8mm → 10.4° / 3.0mm.
 * 所以这个值不是"够用就好", 而是**越小越稳**。
 */
#define REPOS_LIFT            5.0f    // 抬脚高度 (mm) —— 只需离地; 同时是歪角上限
#define REPOS_STEP_TIME       0.20f   // 一组抬落时长 (秒)
#define REPOS_GAP             0.05f   // 两组之间的间隙 (秒)
#define REPOS_TOTAL           (2.0f * REPOS_STEP_TIME + REPOS_GAP)

/* ---- 身体几何 (运行时变量, 默认值来自 ik.h, NVS 可覆盖) ---- */

/* ---- 腿配置: LF, LH, RF, RH ---- */
static const int leg_hip_ch[]  = {0, 2, 4, 6};
static const int leg_knee_ch[] = {1, 3, 5, 7};
static const int leg_side[]    = {IK_SIDE_LEFT, IK_SIDE_LEFT,
                                   IK_SIDE_RIGHT, IK_SIDE_RIGHT};
static const int leg_pair[]    = {IK_LEG_FRONT, IK_LEG_REAR,
                                   IK_LEG_FRONT, IK_LEG_REAR};

/* ---- 舵机角度限速 (每帧最大变化) ----
 *
 * ⚠ **两个常量必须分开, 不能互相套用**:
 *   3°/帧 = 150 °/s —— 姿态过渡够用, 而且不伤舵机。
 *   但套到步态循环上会把摆动相砍死: 摆动相要转 152° (knee 133→57→133),
 *   3°/帧 在一个 0.25s 的摆动相里只够转 37.5° ⇒ 步态直接瘫。
 *
 * 步态循环此前**一道限幅都没有** —— 离线逐帧复刻算出 walk (speed2.5/stride70/lift30)
 * 的摆动相峰值是 髋 806 / 膝 900 °/s, 而 MG90S 在 7.4V 直供下空载也只有约 900 °/s
 * (空载 ⇒ 力矩为零)。即摆动相全程贴着物理极限, 舵机自己饱和 ⇒ 实际轨迹不可知。
 * 加上下面这道限幅后, 只要它**【一帧都不触发】**, 指令 = 实际, 落地打滑那类分析才成立。
 */
#define SERVO_MAX_DEG_PER_FRAME  3.0f   // 姿态过渡 (站姿 / 起步阶段A / 回正步): 150 °/s
#define GAIT_MAX_DEG_PER_FRAME  14.0f   // 步态循环 (walk/trot/go): 700 °/s
static float g_smooth_angles[8] = {90, 90, 90, 90, 90, 90, 90, 90};
static int   g_dir_last = 1;          // 上次运动方向 (+1 前进 / -1 后退), 用于检换向
static float g_stride_smooth = 0.0f;  // GO 平滑步长 (起步从 0 爬升)
static bool  g_half_pulse = false;    // 半周期脉冲 (步长平滑触发)
static bool  g_stand_up = false;      // 阶段 A: 起步过渡 (任意姿态 → 站姿) 进行中

/* ---- 步长淡入淡出 (起步 / 停步 / 换向 共用一套) ---- */
// g_stride_fade ∈ [0,1] 乘在满额步长上: 1=满额, 0=零步长 (原地踏步).
//   起步 = 0 → 1;  停步 / 换向淡出 = 1 → 0.
// 速率 ⇒ 自变量是**相位走过的周期数**, 与速度解耦 (和 g_phase 用同一个积分量: omega*dt/2π).
// 淡入: 线性, fade_in_turns 个周期 (walk 2.0 / trot 1.0, 跟 duty 分档).
// 淡出: **平方曲线**, 统一 FADE_OUT_TURNS 个周期 (不分档, 见上面的常量注释).
// ★ 淡出期间速度必须锁住 (见速度跟随块): 速度归零 ⇒ 周期不走 ⇒ 系数永远淡不完.
static float g_stride_fade  = 1.0f;   // 当前系数值 (1=满额, 0=零步长)
static float g_fade_target  = 1.0f;   // 系数目标 (0 或 1)
static bool  g_fade_active  = false;  // 系数正在朝目标运动

/* ---- 支撑相锚定 (阶段 C) ---- */
// 支撑相不再是"按步长现算位置", 而是"脚一落地世界坐标定死, 之后腿角只随身体前进量变"
// ⇒ 步长变化 = **身体速度变化**, 而不是把已踩住的脚当场重下位置 ⇒ 支撑脚零滑移.
// 常步长时与旧式逐位相同 (x_lift == -stride/2, b - b_land == stride*t) —— 但要求
// foot_trajectory 注释里那两条配套条件 (b 最后推进 + 锁存回退到分数帧跨越点), 缺一即差 mm.
// 全部是 per-leg: turn 会让左右两侧步长不同, 身体前进量因此也得逐腿记.
/* ---- 换向状态 (全踩地窗口直接翻转锚定量) ---- */
static bool g_pending_flip = false;   // 已收到换向信号, 等下一个全踩地窗口
static int  g_flip_sign    = 1;       // 翻转前的运动方向 (等待期间保持它, 防抽腿)

/* ---- 回正步状态 ---- */
static bool  g_pending_repos = false;      // 已收到停步信号, 等下一个全踩地窗口
static gait_type_t g_repos_after = GAIT_STOP;  // 回正完切成哪个步态 (通常 GAIT_STOP)
static float g_repos_timer = 0.0f;         // 回正步计时 (秒)
static float g_repos_x0[4]  = {0, 0, 0, 0};// 进回正步那帧的四腿 x (挪动起点)
// 两组对角的处理顺序 —— 进回正步时按"该运动方向下谁更早进入摆动"排定, 见 motion_enter_repos()
static int   g_repos_pair[2][2] = {{0, 3}, {1, 2}};   // 默认 A={LF,RH} 先

static float g_gait_b[4]     = {0, 0, 0, 0};  // 本腿的虚拟身体前进量 (mm)
static float g_leg_b_land[4] = {0, 0, 0, 0};  // 落地瞬间的 g_gait_b
static float g_leg_x_land[4] = {0, 0, 0, 0};  // 落地瞬间的足端 x (前向约定, 未取反)
static float g_leg_x_lift[4] = {0, 0, 0, 0};  // 离地瞬间的足端 x (前向约定, 未取反)
static bool  g_leg_swing[4]  = {false, false, false, false};  // 上帧是否摆动相 (做边沿)

/* ---- turn 跨 ±0.5 的单腿方向翻转 (per-leg 挂起 + 对齐窗口, 2026-10-14) ----
 * turn 让一侧步长按 s = 1-2|turn| 缩放: |turn|<0.5 同向、=0.5 原地、>0.5 该侧倒走。
 * 跨 0.5 时该腿 leg_stride 变号 → 相位表换一套 (walk) + x 取反 —— 两个动作都是
 * **离散**的, 当帧生效会产生: 位置台阶 / 相位平移 0.24 周期 / 误触边沿锁存。
 * 做法 (与全局换向 "全踩地+相位对齐" 同思路, 单腿版条件更宽):
 *   1. 目标符号 (sign of s) 与生效符号 g_leg_dir 不一致 ⇒ **挂起**:
 *      该腿继续按旧方向走, 幅度仍随 |s| 连续变化;
 *   2. 等**安全窗口**: 该腿切换前(旧表 pn)与切换后(新表 pn)都落在支撑相 ——
 *      支撑相 x 只由锚定量决定、不含相位/ease, 两边同一公式 ⇒ 位置连续;
 *      切换前后都非摆动 ⇒ 不触发落地/离地边沿;
 *   3. 窗口到了才翻: 更新 g_leg_dir + **取反该腿 4 个锚定量** (与 out_x=-out_x
 *      精确抵消, 位置零跳变)。
 * 挂起期间目标又穿回 0.5 (抖动) ⇒ 目标==生效, 挂起自动取消 (天然滞回)。
 * trot 族两套相位表相同 (need_rev 不成立) ⇒ 窗口退化为"在支撑相即可", 更宽。
 * ★ 顺序约束: 翻转必须在**边沿锁存之后**、foot_trajectory 之前。顺序反了,
 *   "落地锁存的 +S/2" 会绕过取反 → 单帧跳 S 量级。离线实测 (gait_sim/
 *   turn_flip_audit.py, 240 起始相位 × 4 场景): 现状当帧翻最坏 23.5mm;
 *   顺序反 9.6mm; 正序 3.5mm ≈ 正常行走本身 (2.5mm)。
 * ★ 与全局换向的关系: g_leg_dir 是"turn 缩放系数的符号", 与全局步进方向
 *   (stride_sign) 正交 —— 全局换向不动它; 两者同帧叠加时, 每次 direction
 *   变化都配一次该腿锚定量取反, 数学上闭合。
 */
static int   g_leg_dir[4]    = {1, 1, 1, 1};   // s(turn) 的**生效符号** (±1)

// 限速: 往 target 方向最多走 max_step 度
static float servo_step_toward(int ch, float target, float max_step) {
    float cur = g_smooth_angles[ch];
    if (target > cur + max_step) return cur + max_step;
    if (target < cur - max_step) return cur - max_step;
    return target;
}

static bool is_static_gait(gait_type_t g) {
    // 回正步也算"不在走路": 它身体不动, 只是把脚挪回站姿点。
    // 算进来的作用: 回正途中收到 set_gait('go') 会走 from_static 分支 (阶段 A + 淡入), 正确。
    return g == GAIT_STOP || g == GAIT_REPOS;
}

/* ---- 全局状态 ---- */
static motion_mode_t g_mode = MODE_IDLE;   // 运行模式状态机

/* ---- 跌倒保护 (C 层 50Hz 检测) ----
 *
 * 默认 monitor=false: 上电阶段不检测, 避免 IMU 收敛/开机姿态造成误释放。
 * frozen/main.py 在 poses.stand() + voice 就绪后调用 fall_guard.start(),
 * 它再 bpuppy_motion.safety_enable(True) 武装。
 *
 * 检测范围 = 运动模式 (MODE_MOTION) 或 poses.stand() 置的 POSE 站立标志。
 * 触发后 motion_safety_release() 置锁存 + 关 PWM + 停运动; 之后只等
 * Python 收到声音角度帧并确认身体放平, 调 safety_rearm(CROUCH) 恢复。
 */
static bool  g_safety_monitor_enabled = false;
static bool  g_safety_pose_active = false;   // poses.stand() 置 true, 其它姿态置 false
static bool  g_safety_armed = false;
static int   g_safety_arm_count = 0;
static int   g_safety_fall_count = 0;
static float g_safety_angle_deg = 60.0f;
static float g_safety_rate_dps = 400.0f;
static int   g_safety_frames = 3;

__attribute__((weak)) motion_state_t g_motion = {
    .gait           = GAIT_STOP,
    .speed          = 0.0f,          // 实际速度静止为 0
    .target_speed   = SPEED_DEFAULT, // 目标速度默认 2.5
    .stride         = STRIDE_DEFAULT,
    .direction      = 1.0f,          // 上电默认朝前
    .height         = HEIGHT_DEFAULT,
    .lift_height    = LIFT_DEFAULT,
    .body_roll      = 0.0f,
    .body_pitch     = 0.0f,
    .ik_L1          = IK_L1_DEFAULT,
    .ik_L2          = IK_L2_DEFAULT,
    .omega_base     = 2.0f,
    .gait_duty      = 0.20f,
    .gait_gap       = 0.04f,
    .turn           = 0.0f,
    .center_offset  = CENTER_OFFSET_DEFAULT,
    .center_spacing = CENTER_SPACING_DEFAULT,
    .sway_amp       = SWAY_DEFAULT,
    .body_half_l    = IK_BODY_HALF_L_DEFAULT,
    .body_half_w    = IK_BODY_HALF_W_DEFAULT,
    .emergency_stop = false,
    .enabled        = false,
    .pose_trans     = 0,
    .pose_timer     = 0.0f,
};

static TaskHandle_t g_task_handle = NULL;
__attribute__((weak)) float g_phase = 0.0f;
static float g_prev_fx[4] = {0, 0, 0, 0};
static float g_prev_fz[4] = {100, 100, 100, 100};
static bool g_was_moving = false;

/* 运动实参的跟随限速 (见文件上方 RAMP_*_STEP):
 * 喂给腿循环的**实际值**, 每帧朝 g_motion 里的目标挪一步.
 * 初值与 g_motion 初始化一致; motion_task_start() 里还会再同步一次, 免得
 * Python 在任务启动前就把某几个参数改掉 (那时没人推进限速层). */
static float g_ramp_height = HEIGHT_DEFAULT;
static float g_ramp_lift   = LIFT_DEFAULT;
static float g_ramp_center = CENTER_OFFSET_DEFAULT;
static float g_ramp_spacing = CENTER_SPACING_DEFAULT;  // 中位点间距 (前腿 +s/2 / 后腿 -s/2)
static float g_ramp_sway   = 0.0f;   // 摆动幅值实参 (mm, 永远斜坡, 从 0 淡入 —— 见 SWAY_*)
static float g_sway_dir    = 1.0f;   // 摆动方向符号实参 (±1, 换向时斜坡过渡)
static float g_ramp_pitch  = 0.0f;
static float g_ramp_roll   = 0.0f;
static float g_ramp_turn   = 0.0f;   // 转弯率实参 (运动中每帧限速; 见 RAMP_TURN_STEP)

/* ---- 本帧**实际用**的步态参数 (只读上报) ----
 *
 * 为什么要在任务里存一份: Python 侧要做转弯归一化 (`turn` 这个几何系数 → 与步态无关的
 * 转速指令), 得先知道"turn=1 在当前步态下对应多少偏航角速度", 而那个换算依赖
 * **实际生效的** eff_stride / eff_duty。
 *
 * ⚠ 光看 get_params() 是不够的 —— **GO 步态会覆盖用户设的 stride/duty**
 *   (见下面 GO 分支), 用户读数里那两个是"设进去的", 不是"真在用的"。
 * ⇒ 由任务每帧把定稿值存出来, 供 `motion_get_effective()` 上报。**纯上报, 不参与任何计算。**
 */
static volatile float g_eff_stride_out = STRIDE_DEFAULT;
static volatile float g_eff_duty_out   = 0.20f;

/* ---- 相位偏移计算 (统一框架) ---- */
// 规则:
//   LH_start = 0
//   LF_start = duty + gap
//   RH_start = 0.50
//   RF_start = 0.50 + duty + gap
//   direction<0 时 LF 和 LH 角色互换 (后退)
// 输出 offset[LF,LH,RF,RH], 其中 start_time = (1 - offset) mod 1
static void compute_offsets(float duty, float gap, int direction,
                            float offsets[4])
{
    float start_LF, start_LH, start_RF, start_RH;

    if (direction > 0) {
        // 前进: LH 领头
        start_LH = 0.0f;
        start_LF = duty + gap;
        start_RH = 0.50f;
        start_RF = 0.50f + duty + gap;
    } else {
        // 后退: LF 领头
        start_LF = 0.0f;
        start_LH = duty + gap;
        start_RF = 0.50f;
        start_RH = 0.50f + duty + gap;
    }

    // 归一化到 [0, 1)
    if (start_LF >= 1.0f) start_LF -= 1.0f;
    if (start_LH >= 1.0f) start_LH -= 1.0f;
    if (start_RF >= 1.0f) start_RF -= 1.0f;
    if (start_RH >= 1.0f) start_RH -= 1.0f;

    // offset = (1 - start) mod 1
    offsets[0] = (start_LF > 0.001f) ? (1.0f - start_LF) : 0.0f;
    offsets[1] = (start_LH > 0.001f) ? (1.0f - start_LH) : 0.0f;
    offsets[2] = (start_RF > 0.001f) ? (1.0f - start_RF) : 0.0f;
    offsets[3] = (start_RH > 0.001f) ? (1.0f - start_RH) : 0.0f;
}

// 预备位 = 全踩地相位中点: 四条腿全部在支撑相 (着地), z=height
// 适合作为站立↔行走之间的过渡姿态, 平移足端不抬腿
static float all_stance_mid(float duty, float gap)
{
    // 四条腿的摆动结束点 (start + duty)
    float ends[4];
    ends[0] = duty + gap + duty;        // LF
    ends[1] = 0.0f + duty;              // LH
    ends[2] = 0.50f + duty + gap + duty;// RF
    ends[3] = 0.50f + duty;             // RH
    float latest = 0.0f;
    for (int i = 0; i < 4; i++) {
        if (ends[i] > 1.0f) ends[i] -= 1.0f;
        if (ends[i] > latest) latest = ends[i];
    }
    float mid = (latest + 1.0f) * 0.5f;
    if (mid >= 1.0f) mid -= 1.0f;
    return mid;
}

/* ---- 此刻四条腿是不是都踩在地上 ----
 * 换向要换偏移组标签 (LF↔LH, RF↔RH 的相位互换). 这是一次**离散的重新贴标签**,
 * 不是某个量的大小变化 ⇒ 缓动治不了 (实测: 把偏移组缓动 0.3s, 最坏 31.5°/55.3°
 * 只降到 31.6°/50.4°, 几乎没动 —— z 对相位的斜率在摆动相入口最大).
 * 只能**挑时候**: 步长 0 时两套标签在 x 上完全相同 (都 ≡0), z 上也相同 ——
 * 当且仅当那帧四腿全在支撑相 (全踩地时两套标签都给 z=height) ⇒ 此刻换标签精确零跳变.
 * 窗口占比: walk (0.20,0.24)(0.44,0.50)(0.70,0.74)(0.94,1.00), trot (0.40,0.50)(0.90,1.00).
 *
 * dir 用**当前实际方向** (换向等窗口期间是原方向, 见 g_flip_sign), 不能直接用 g_motion.direction,
 * 否则会按新标签去判、判出来的窗口是错的.
 */
static bool all_legs_in_stance(float phase_rad, float duty, float gap, int dir)
{
    bool need_rev = (dir < 0) && (duty + gap < 0.50f);
    float offs[4];
    compute_offsets(duty, gap, need_rev ? -1 : 1, offs);
    for (int i = 0; i < 4; i++) {
        // phase_rad ∈ [0,2π) 且 offs ∈ [0,1) ⇒ pn ∈ [0,2), 一次减法就够
        // (跟腿循环里相位归一化同一个写法, 这里是唯一权威的定义处)
        float pn = phase_rad / TWO_PI + offs[i];
        if (pn >= 1.0f) pn -= 1.0f;
        if (pn < duty) return false; // 这条腿在摆动相 (抬着)
    }
    return true;
}

/* ---- 翻转对齐点: 此刻翻转, 四条腿的姿势才正好接得上新方向 ----
 *
 * 翻转会把相位表的标签换掉 (LF↔LH / RF↔RH, 见 compute_offsets 的 direction 分支),
 * 于是每条腿的支撑相进度从 t_old (旧方向的表) 变成 t_new (新方向的表)。
 * 若翻转那一帧某条腿正好跨过"摆动↔支撑"边界, 落地锁存会把它的 x_land 写死成
 * +stride/2 —— 那个锁存**不管方向**, 而翻转刚把 x_land 取成负数, 两者一撞位置就跳:
 *
 *      跳变量 = stride * (t_old + t_new - 1)
 *
 * 离线逐帧实测 (go speed 2.5, stride 70): 好相位 2.2mm, 坏相位 48.0mm, 与公式吻合到 0.1mm。
 * 四个全踩地窗口里 (0.20,0.24) 和 (0.70,0.74) 两段的和 ≈ 1, 另外两段 (0.44,0.50)
 * (0.94,1.00) 整段都在 0.3 附近 ⇒ **只有一半的窗口是能用的**, 这就是"一半几率好"的来源。
 *
 * 跳变只在 t_old + t_new == 1 时为 0。这个方程在整个相位空间里只有两个解:
 *      phase_norm = duty + gap/2   和   duty + gap/2 + 0.5
 * 即全踩地窗口**正中间**那两个点 —— 四条腿此刻的姿势, 恰好就是新方向步态走到这里时
 * 本该有的姿势, 接上去严丝合缝。不在解上就等下一个 (最多半个周期), 等待期间照旧方向
 * 走, 不停不减速 —— 比旧的"减速停 2.83~3.16s"仍快 5 倍以上。
 *
 * 两个解相差正好半个周期, 所以判据是 "(pn - duty) 对 0.5 取模后落在 [0, gap)"。
 * trot 族 (duty+gap == 0.50) 只有两个窗口, 且本来就落在解上 ⇒ 等于不用等。
 */
static bool flip_phase_aligned(float phase_rad, float duty, float gap)
{
    float d = phase_rad / TWO_PI - duty;
    d -= 0.5f * floorf(d / 0.5f);   // 对 0.5 取模 (floorf 写法对负数也对)
    if (d < 0.0f) d += 0.5f;
    return d < gap;
}

/* ---- 步态参数表 ---- */
static void motion_apply_gait_params(gait_type_t gait)
{
    switch (gait) {
    case GAIT_WALK:        g_motion.gait_duty = 0.20f; g_motion.gait_gap = 0.04f;  break;
    case GAIT_TROT:        g_motion.gait_duty = 0.40f; g_motion.gait_gap = 0.10f;  break;
    case GAIT_GO:          g_motion.gait_duty = 0.20f; g_motion.gait_gap = 0.04f;  break;  // 运行时根据 speed 动态调整
    default: break; // stand/stand_up 不调参数
    }
    // ⚠ 打的是 g_motion.turn (当前转弯系数), 不是"本次步态带来的转弯"
    ESP_LOGI(TAG, "Gait: %d (duty=%.2f gap=%.2f turn=%.2f)",
             gait, g_motion.gait_duty, g_motion.gait_gap,
             g_motion.turn);
}

/* ---- 进回正步 ---- */
// 调用时机: 全踩地窗口那一帧 (没有腿在空中 ⇒ 步长硬切不会甩腿).
// 进来之后身体不再移动 (speed=0 ⇒ 相位不累加; stride=0 ⇒ b 不推进), 只剩抬脚挪位。
static void motion_enter_repos(gait_type_t after, int dir, float duty, float gap)
{
    g_motion.gait  = GAIT_REPOS;
    g_motion.speed = 0.0f;
    g_repos_timer  = 0.0f;
    g_repos_after  = after;

    for (int i = 0; i < 4; i++) {
        g_repos_x0[i] = g_prev_fx[i];   // 挪动起点 = 当前足端 x
    }

    /* 两组对角: A = {LF(0), RH(3)},  B = {LH(1), RF(2)}
     * ★ 谁先抬, 按**刚才那个运动方向**下的相位偏移定 —— 偏移小的更早进入摆动, 就是"领先"的那组。
     *   这样回正步的节拍跟走路一致 (前进: LH 领先 → B 先; 后退: LF 领先 → A 先),
     *   看起来是"顺着刚才的方向又走了一步", 而不是倒着走一步。
     *   trot 两个方向同序 (need_rev 在 duty+gap==0.50 时不成立, 后退也用 offsets_fwd)。
     * ⚠ 若实测发现方向反了, 把下面那个比较改成 `leaderB > leaderA` 即可。 */
    bool need_rev = (dir < 0) && (duty + gap < 0.50f);
    float offs[4];
    compute_offsets(duty, gap, need_rev ? -1 : 1, offs);

    float leaderA = fminf(offs[0], offs[3]);   // A = LF, RH
    float leaderB = fminf(offs[1], offs[2]);   // B = LH, RF

    if (leaderB < leaderA) {
        g_repos_pair[0][0] = 1; g_repos_pair[0][1] = 2;   // B 先
        g_repos_pair[1][0] = 0; g_repos_pair[1][1] = 3;
    } else {
        g_repos_pair[0][0] = 0; g_repos_pair[0][1] = 3;   // A 先
        g_repos_pair[1][0] = 1; g_repos_pair[1][1] = 2;
    }

    ESP_LOGI(TAG, "Repos: pair1=%d,%d pair2=%d,%d (dir=%d%s)  x0=%.1f/%.1f/%.1f/%.1f",
             g_repos_pair[0][0], g_repos_pair[0][1],
             g_repos_pair[1][0], g_repos_pair[1][1],
             dir, need_rev ? " rev-offsets" : "",
             g_repos_x0[0], g_repos_x0[1], g_repos_x0[2], g_repos_x0[3]);
}

/* 实参限速: 每帧朝目标挪一步, 够近就直接到位 (不留残差) */
static float ramp_to(float cur, float target, float step)
{
    float d = target - cur;
    if (fabsf(d) <= step) return target;
    return cur + (d > 0.0f ? step : -step);
}

/* ---- 足端轨迹生成器 (支撑相锚定, 阶段 C) ----
 *
 * 旧式把步长当**位置系数**: 摆动 x = -S/2 + S*ease, 支撑 x = S/2 - S*t.
 * S 是四腿共用的全局量、每帧现算 ⇒ 步长一变, 已踩在地上的脚当帧就被重下位置
 * (= 在地上拖). 逐腿算过: 一档 ΔS=14mm 时三条支撑腿要求 -2.80 / +5.95 / +1.75 mm,
 * 互相矛盾 (跨度 8.75mm) ⇒ 要么打滑要么身体被推扭.
 *
 * 改成**锚定**: 一只脚一落地, 它的世界坐标就定死了; 之后腿角变化唯一的理由是
 * 身体在往前走. 即 x_i(t) = p_i - b(t), 所有支撑腿共用同一个 b ⇒ 任意两脚之差
 * = p_i - p_j = 常数, **无论步长怎么变都自动成立**. 步长于是从"位置系数"变成
 * "速度": 脚踩在原处不动, 身体快一点慢一点.
 *
 * 常步长时与旧式**逐位相同** (离线实测最大差 0.000000 mm) —— 但这**要求两个配套条件**,
 * 缺一个就会差到 mm 级 (都是实测出来的, 不是推的):
 *
 *   ① b 必须在**算完本帧 x 之后**才累加. b 的含义是"当前这一帧相位下的身体前进量",
 *      若先累加再算 x, b 就比相位超前整整一帧 ⇒ 常步长下也差一帧的量
 *      (walk speed 4 差 2.23mm, trot speed 10 差 5.31mm), 且是四腿一致的常量偏移.
 *   ② 两个锁存都要**回退到理想的分数帧跨越点**, 不能就用"检测到边沿那一帧"的值:
 *        落地 — 理想落地是 pn = duty, 本帧已越过 over = (pn-duty)/(1-duty)
 *        离地 — 理想离地是 pn = 0, 本帧已走过 pn
 *      不回退的话, 摆动起点 x_lift 比 -S/2 差一点 (walk 2.23mm / trot 5.31mm).
 *   只做 ① 不做 ② (或反之) 都仍有 mm 级差异; 两个都做才是逐位相同.
 *
 *   手推: 摆动 — x_lift 恒等于 -S/2, 代入即 -S/2 + S*ease;
 *         支撑 — b-b_land = (S/(1-duty))*(自理想落地起的周期数), 而 t 正是那个周期数/(1-duty)
 *                ⇒ b-b_land = S*t, 代入即 S/2 - S*t.
 * ⇒ 对调好的步态是纯重构, 正常行走一字不变.
 *
 * 边界 (诚实): b 是**虚拟**身体前进量, 真实前进由动力学决定. 地板滑 / 狗被架起来 /
 * 被挡住 —— 开环轨迹步态无从知道. 它修的是"指令自己前后矛盾", 不是物理滑移.
 *
 * 另有**不是本机制引入**的既有误差: 摆动↔支撑拐角处 (落地/离地那一帧) 新旧两式都有
 * 3~4mm 的单帧台阶, 那是帧量化 (步态被 20ms 采样) 造成的, 现行固件同样有. 实测:
 * 拐角台阶 旧 3.87 / 新 3.64 mm —— 两式相当, 不是回归.
 *
 * x_lift / b_now / b_land / x_land 由腿循环在离地、落地边沿锁存并传入 (全是**前向约定**,
 * 未按 direction 取反 —— 取反统一放在最后一步, 这样锁存值本身就是连续的).
 */
static void foot_trajectory(float phase_norm, float stride, float height,
                            float lift, float swing_ratio, float direction,
                            float x_lift, float b_now, float b_land, float x_land,
                            float *out_x, float *out_z)
{
    if (phase_norm < swing_ratio) {
        float t = phase_norm / swing_ratio;
        float ease = t * t * (3.0f - 2.0f * t);
        *out_z = height - lift * sinf(ease * (float)M_PI);
        // 摆动: 从**实际离地位置** x_lift 起步, 走到 +stride/2
        // (旧式从写死的 -stride/2 起步 —— 步长变了就等于把空中的脚也挪一下)
        *out_x = x_lift + (stride * 0.5f - x_lift) * ease;
    } else {
        *out_z = height;
        // 支撑: 落地锁存点 − 落地以来的身体前进量 (脚不动, 是身体在走)
        *out_x = x_land - (b_now - b_land);
    }

    if (direction < 0) {
        *out_x = -*out_x;
    }
}

/* ---- 跌倒保护检测 (在 motion task 的 50Hz 循环里跑) ----
 *
 * 放在 C 层而不是 Python 线程: 即使 KittenBlock 下载的用户程序在
 * while True 里空转, 这里的检测和关 PWM 也不会被 GIL 拖慢。
 */
static void motion_safety_tick(void)
{
    if (!g_safety_monitor_enabled || servo_safety_latched() || !imu_is_ready()) {
        return;
    }

    imu_angles_t ang = {0};
    imu_raw_data_t raw = {0};
    imu_read_angles(&ang);
    imu_read_raw(&raw);

    float tilt = fmaxf(fabsf(ang.roll), fabsf(ang.pitch));
    float rate = sqrtf(raw.gyro_x * raw.gyro_x +
                       raw.gyro_y * raw.gyro_y +
                       raw.gyro_z * raw.gyro_z) * 57.29578f;

    bool scope = (g_mode == MODE_MOTION) || g_safety_pose_active;
    if (!scope) {
        // 坐/蹲/玩/挥手等静态姿势不在保护范围
        g_safety_armed = false;
        g_safety_arm_count = 0;
        g_safety_fall_count = 0;
        return;
    }

    if (!g_safety_armed) {
        // 上电/恢复后先要求约 0.5s 稳定, 避免 IMU 收敛或刚上力时的姿态误触发。
        // 15° 而不是 10°: 实测蹲姿/地面轻微不平时 roll 可到 10.6°, 太紧会永远不武装。
        if (tilt < 15.0f && rate < 50.0f) {
            if (++g_safety_arm_count >= 25) {
                g_safety_armed = true;
                g_safety_arm_count = 0;
                ESP_LOGI(TAG, "Safety armed (tilt=%.1f rate=%.0f)", tilt, rate);
            }
        } else {
            g_safety_arm_count = 0;
        }
        return;
    }

    if (tilt > g_safety_angle_deg || rate > g_safety_rate_dps) {
        if (++g_safety_fall_count >= g_safety_frames) {
            g_safety_fall_count = 0;
            ESP_LOGE(TAG, "FALL detected (tilt=%.1f rate=%.0f), release servos",
                     tilt, rate);
            motion_safety_release();
        }
    } else {
        g_safety_fall_count = 0;
    }
}

/* ---- 主控制循环 ---- */
static void motion_task_main(void *pvParam)
{
    ESP_LOGI(TAG, "Motion control started (50Hz)");
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MOTION_LOOP_MS);
    float dt = MOTION_LOOP_MS * 0.001f;

    while (1) {
        // 先做安全检测: 即使运动被禁用 (POSE 站立) 也要能释放跌落的舵机
        motion_safety_tick();

        if (!g_motion.enabled || g_motion.emergency_stop) {
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        bool is_stand    = (g_motion.gait == GAIT_STOP);

        /*
         * ============================================================
         * 姿态过渡: 停步过渡 (起步过渡已改由 g_stand_up 承担)
         * ============================================================
         *
         * 站立 ←→ 预备位 ←→ 行走
         *
         * 预备位 = 全踩地相位中点 (all_stance_mid), 此时四腿着地不抬,
         *         从站立移动到预备位 (或反向) 只平移足端, 不会歪倒.
         *
         * 起步过渡: **pose_trans=1 不可用**. 原因: 它从**写死的** (0, eff_height) 起步
         *   (等于"假设狗站着"), 从坐姿按前进时第一帧就命令髋/膝跳 95.7°;
         *   且 GO 的 eff_height 被强制成 70, 站高设成非 70 时会白跳一段.
         *   现在改由 g_stand_up (阶段 A) 从**实际舵机角**起步, 见下面腿循环的限速器.
         *
         * 停步过渡 (pose_trans=2):
         *   正常行走 → (足端 smoothstep 0.3s 退回站姿) → 站立姿态
         *
         * 过渡期 0.3s, 用 smoothstep 缓动: ease = t²(3-2t)
         */
        #define TRANS_TIME 0.3f
        // 停住 → 阶段 A 立即作废 (交给停步过渡)
        if (is_stand) g_stand_up = false;

        bool now_static = is_stand;
        if (now_static && g_was_moving) {
            // 行走 → 静止: 启动停步过渡
            g_motion.pose_trans = 2;
            g_motion.pose_timer = 0.0f;
            g_motion.speed = 0.0f;
        }
        // 姿态过渡计时: 到期清零
        if (g_motion.pose_trans == 2) {
            g_motion.pose_timer += dt;
            if (g_motion.pose_timer >= TRANS_TIME)
                g_motion.pose_trans = 0;
        }
        g_was_moving = !now_static;

        // 换向检测: 运动方向变化 → 挂起, 等下一个"四腿全踩地"窗口做翻转 (见下面的翻转块)。
        // ★ 为什么不用"减速停 → 淡出 → 淡入": 那套要 2.83~3.16s, 而全踩地翻转是当帧完成的
        //   (四腿全在支撑相, 而支撑相不含相位 ⇒ 取反锚定量与方向翻转精确抵消)。
        int cur_dir = (g_motion.direction < 0.0f) ? -1 : 1;   // ★ direction 是独立字段, 与 stride 的符号无关
        if (cur_dir != g_dir_last && !now_static) {
            // ★ 阶段 A 期间不检换向: 那时还没起步, 用户先设负 stride 再按前进会被误判成换向
            if (!g_pending_flip && g_motion.pose_trans == 0 && !g_stand_up) {
                g_pending_flip = true;
                g_flip_sign    = g_dir_last;   // 旧方向 —— 等待期间保持它, 否则支撑腿当帧取反
            }
        }
        g_dir_last = cur_dir;

        // speed=步频 stride=步幅**幅度** direction=方向 (±1) —— 两者互不干扰
        float eff_speed = g_motion.speed;
        // 本帧走过的步态周期数 —— g_phase / 步长淡入淡出 / 支撑相锚定**三者共用的积分量**.
        // 同一个式子在三处用, 所以"相位走一圈"必然等于"系数走 1/fade_in_turns"、
        // 也必然等于"身体前进 stride/(1-duty)", 不需要任何额外计时器去对齐.
        float frame_dphi = (g_motion.omega_base * eff_speed) * dt / TWO_PI;
        // 系数淡出中 (朝 0 走) —— 速度跟随块要用它锁速度, 见那里的注释
        bool fade_closing = g_fade_active && (g_fade_target < 0.5f);

        /*
         * ============================================================
         * 速度跟随: 半周期更新
         * ============================================================
         *
         * 实际速度向目标速度跟随, 每半个步态周期 (g_phase 过 0 或 PI)
         * 调整一次, 每次最多变化 ±SPEED_FOLLOW_STEP. 避免每帧变速造成的步态冲击.
         *
         * 相位冻结 (speed=0) 时也触发, 防止锁死.
         *
         * ★ 阶段 A (g_stand_up) 期间不累加也不跟随速度: 相位钉在 all_stance_mid,
         *   限速器只管把 8 个关节拉到站姿角. 否则腿会在舵机还没到位时就开始迈.
         */
        // 相位累加 + 半周期速度更新
        // ★ 回正步排除在外: 它身体不动 (speed=0), 若不排除, prev_phase == g_phase 成立
        //   会走进"起步 kick"那条分支把 speed 重新点着, 相位就又开始跑了。
        if (!is_stand && !g_stand_up && g_motion.gait != GAIT_REPOS) {
            float omega = g_motion.omega_base * eff_speed;
            float prev_phase = g_phase;
            g_phase += omega * dt;
            if (g_phase > TWO_PI) g_phase -= TWO_PI;

            bool cross_half = (int)(prev_phase / (float)M_PI) !=
                              (int)(g_phase     / (float)M_PI);
            if (cross_half || prev_phase == g_phase) {
                if (cross_half) g_half_pulse = true;
                // 停步/换向停: 速度不减, 照常跟随 target_speed (只减步长)
                float target_eff = g_motion.target_speed;
                // ★ 系数淡出期间**锁住速度** (target 设成当前值 ⇒ ds=0 ⇒ 不动).
                //   原因: 系数的自变量是"走了多少周期", 而周期 = omega*dt, omega ∝ speed.
                //   速度归零 ⇒ 周期不走 ⇒ 系数永远淡不完 (死锁). 而滑块拉到 0 这条路
                //   恰好会把 speed 降到 0 —— 所以必须锁. 这也正是上一行注释的本意.
                if (fade_closing) target_eff = g_motion.speed;
                float ds = target_eff - g_motion.speed;
                if (fabsf(ds) < 0.15f) {
                    g_motion.speed = target_eff; // 接近就到位
                } else if (g_motion.speed < 0.15f && target_eff > 0.0f) {
                    // 起步 kick (仅"从静止到起步": 站立→GO / 换向反向起步 / 暂停恢复):
                    // speed=0 相位不动, 先给起步速度(≤2.5)让相位能动, 再半周期爬升
                    g_motion.speed = (target_eff < SPEED_DEFAULT)
                                   ? target_eff : SPEED_DEFAULT;
                } else {
                    float step = SPEED_FOLLOW_STEP;
                    if (ds >  step) ds =  step;
                    if (ds < -step) ds = -step;
                    g_motion.speed += ds;
                }
            }
        }

        // GAIT_GO: 速度自适应 duty/gap/stride/height/lift —— GO 下**三个运动参数全部无效**,
        // 只有 speed 与 gait 说了算 (方向是**独立字段** g_motion.direction, 见下方 stride_sign)
        float eff_duty   = g_motion.gait_duty;
        float eff_gap    = g_motion.gait_gap;
        // eff_stride 取绝对值 (方向由下方 stride_sign 决定, 与 GO 一致;
        // 否则 TROT/WALK 含符号 stride + 符号 stride_sign 双重符号 → 后退变前进)
        float eff_stride = fabsf(g_motion.stride);
        float eff_height = g_motion.height;
        float eff_lift   = g_motion.lift_height;
        // GO: speed≤4=walk  speed≥6=trot  → duty/gap/stride/height/lift 插值
        // (body_pitch 不在此列: 它由 set_body_pose 手动设定, 三个步态一视同仁地生效,
        //  在这里清零会让 GO 下"设了没反应" —— 只有 walk/trot 生效)
        if (g_motion.gait == GAIT_GO) {
            float s = eff_speed;
            if (s <= 4.0f) {
                eff_duty=0.20f; eff_gap=0.04f;
                eff_stride=70.0f; eff_height=70.0f;
                eff_lift=GO_LIFT_LOW;
            } else if (s >= 6.0f) {
                eff_duty=0.40f; eff_gap=0.10f;
                eff_stride=50.0f; eff_height=70.0f;
                eff_lift=GO_LIFT_HIGH;
            } else {
                float t = (s - 4.0f) / 2.0f;
                eff_duty   = 0.20f + t * 0.20f;
                eff_gap    = 0.04f + t * 0.06f;
                eff_stride = 70.0f - t * 20.0f;
                eff_height = 70.0f;
                eff_lift   = GO_LIFT_LOW + t * (GO_LIFT_HIGH - GO_LIFT_LOW);
            }
        }

        /* ---- 运动实参的跟随限速 (说明见文件上方 RAMP_*_STEP) ----
         * 放在 GO 分支**之后**: 对 walk/trot 限的是用户改的那几个值, 对 GO 限的是
         * speed 插值算出来的抬脚 (站高恒 70, 无所谓).
         *
         * ★ 间距只在 walk/trot (含静止/回正, 那两态保持现状) 生效 —— **GO 不叠加**:
         *   GO 的几何是固定表 (70/70/30), 而间距是按用户参数配平的, 两者组合可能超出
         *   可达 ⇒ 静默缩短 (前伸/后伸端点 d>84)。切换步态时走同一条限速层 (1.5mm/帧)
         *   平滑淡出/淡入。中途偏移 (center_offset) 不受此限 —— 它三种步态通用。 */
        float eff_center_spacing = (g_motion.gait == GAIT_GO) ? 0.0f
                                                              : g_motion.center_spacing;
        if (is_stand || g_motion.gait == GAIT_REPOS) {
            // 静止: 实参直接对齐目标 —— 站着改参数有 3°/帧 的限制器兜着, 不需要这层
            g_ramp_height  = eff_height;
            g_ramp_lift    = eff_lift;
            g_ramp_center  = g_motion.center_offset;
            g_ramp_spacing = eff_center_spacing;
            g_ramp_pitch   = g_motion.body_pitch;
            g_ramp_roll    = g_motion.body_roll;
            g_ramp_turn    = g_motion.turn;
        } else {
            g_ramp_height  = ramp_to(g_ramp_height,  eff_height,              RAMP_HEIGHT_STEP);
            g_ramp_lift    = ramp_to(g_ramp_lift,    eff_lift,                RAMP_LIFT_STEP);
            g_ramp_center  = ramp_to(g_ramp_center,  g_motion.center_offset,  RAMP_CENTER_STEP);
            g_ramp_spacing = ramp_to(g_ramp_spacing, eff_center_spacing,      RAMP_CENTER_STEP);
            g_ramp_pitch   = ramp_to(g_ramp_pitch,   g_motion.body_pitch,     RAMP_ATTITUDE_STEP);
            g_ramp_roll    = ramp_to(g_ramp_roll,    g_motion.body_roll,      RAMP_ATTITUDE_STEP);
            g_ramp_turn    = ramp_to(g_ramp_turn,    g_motion.turn,           RAMP_TURN_STEP);
        }
        eff_height = g_ramp_height;
        eff_lift   = g_ramp_lift;

        /* ---- 相位联动摆动: 目标幅值 (见文件上方 SWAY_* 注释) ----
         * walkness: walk(duty+gap=0.24)=1 → trot(0.50)=0, GO 4~6 段线性跟随;
         * × g_stride_fade: 起步淡入 / 滑块归零淡出 / 回正 (系数收 0) 都自动收放。
         * ★ 永远走限速层, **不参与**上面 is_stand/REPOS 的"静止直接对齐" ——
         *   步态切换 (walk→trot) 目标会瞬变 12→0, 直接对齐就是 12mm 的足端跳变。 */
        float walkness = (0.50f - (eff_duty + eff_gap)) / (0.50f - 0.24f);
        if (!(walkness > 0.0f)) walkness = 0.0f;   // ★ 含 NaN 护栏 (!(a>0) 对 NaN 为真)
        if (walkness > 1.0f)    walkness = 1.0f;
        float eff_sway = 0.0f;
        if (!(is_stand || g_motion.gait == GAIT_REPOS)) {
            eff_sway = walkness * g_motion.sway_amp * g_stride_fade;
        }
        g_ramp_sway = ramp_to(g_ramp_sway, eff_sway, RAMP_SWAY_STEP);

        /* 摆动方向符号: 与 stride_sign 同源 (换向等待期保持旧方向), 用斜坡过渡 ——
         * 当帧翻转会在相位非零处造成 2|sway| 的足端跳变 (翻转点不保证落在余弦零点)。 */
        float sway_dir_t = g_pending_flip ? (float)g_flip_sign : g_motion.direction;
        g_sway_dir = ramp_to(g_sway_dir, (sway_dir_t < 0.0f) ? -1.0f : 1.0f,
                             SWAY_DIR_STEP);

        /*
         * ============================================================
         * 步长淡入淡出 (只剩"起步淡入"和"目标速度归零淡出"两条路)
         * ============================================================
         *
         * g_stride_fade ∈ [0,1] 乘在满额步长上.
         *   起步 —— 0 → 1 (从站立起步时步长必须从 0 起);
         *   目标速度归零 —— 1 → 0 (步长收到 0 = 原地踏步, 然后切静态).
         * ★ 停步 (set_gait "stop") 走**回正步**、换向走**对齐点翻转**, 两条都不经过这里.
         *
         * 为什么起步必须从 0 起 (一进场就满步长有两个后果):
         *   (1) 身体从静止瞬间达到全速 (满额 stride/(1-duty) ≈ 87.5mm/周期);
         *   (2) 进场时四腿相位是错开的 (LH 已在自己支撑相的末尾, RF 才开头),
         *       而脚都在 x=0 ⇒ 首圈每腿都要"重新锚定"回自己的正确位置.
         *
         * 自变量是**本帧走过的步态周期数** (frame_dphi), 不是时间 ⇒ 与速度解耦:
         * 无论 speed 多少, 都是固定的那几个步态周期走完全程.
         *   **淡入** (起步): 线性, walk FADE_IN_TURNS_WALK / trot FADE_IN_TURNS_TROT 个周期 ——
         *     分档的理由是 walk 段 (duty 0.20, 一次只抬一条腿, 永远三条着地) 可以慢慢原地踏步,
         *     trot 段 (duty 0.40, 对角两腿同时离地) 原地踏步静态不稳, 窗口要短.
         *     (当前两档同值 1.0 ⇒ 分档暂时失效, 见常量处的注释.)
         *   **淡出** (只有"目标速度归零"这一条路): 平方曲线, 三种步态统一 FADE_OUT_TURNS 个周期.
         *     不再分档 —— 平方曲线下同一个时长对整个 duty 空间都够干净 (见常量注释).
         *     ★ 停步 (set_gait "stop") 走**回正步**、换向走**对齐点翻转**, 两条都不经过这里.
         * 逐帧插值 (不是每半周期跳一档): 跳一档 = 已踩地的脚被当场重下位置,
         * 一档 Δstride/2 最坏 7mm ≈ 髋 8.1°; 逐帧只有 0.36mm (speed 4).
         *
         * ★ 阶段 A 期间系数不动 (收在 0): 那时相位被钉死, 累加也没意义.
         * ★ "目标速度归零"的**切换时机**不在这里, 在下面的"淡出完成"块 —— 必须等
         *   "四腿全踩地"窗口, 见那里的注释.
         */
        bool stride_stopping = g_motion.target_speed <= 0.1f;
        float fade_in_turns = (eff_duty >= 0.30f) ? FADE_IN_TURNS_TROT : FADE_IN_TURNS_WALK;

        if (is_stand || g_motion.gait == GAIT_REPOS) {
            g_fade_active = false;        // 站住 / 回正: 系数闲置 (下次起步重新收到 0)
        } else if (!g_stand_up) {
            if (!g_fade_active) {
                // 系数空闲 → 朝"该去的地方"重新 armed: 该走就淡入, 该停就淡出.
                // ★ 自校正: "先 set_gait 后 set_speed" 这种顺序也不会卡死 ——
                //   系数先淡到 0 站住, 等速度一到位自己就重新淡入.
                if (stride_stopping && g_stride_fade > 0.001f) {
                    g_fade_target = 0.0f;
                    g_fade_active = true;
                } else if (!stride_stopping && g_stride_fade < 0.999f) {
                    g_fade_target = 1.0f;
                    g_fade_active = true;
                }
            }
            if (g_fade_active) {
                if (g_stride_fade < g_fade_target) {
                    /* 淡入: 线性 (系数 0→1, 走一个步态周期) */
                    float step = frame_dphi / fade_in_turns;
                    g_stride_fade += step;
                    if (g_stride_fade >= g_fade_target) {
                        g_stride_fade = g_fade_target;
                        g_fade_active = false;
                        ESP_LOGI(TAG, "Stride fade-in done (%.1f turns, stride=%.1f)",
                                 fade_in_turns, fabsf(g_motion.stride));
                    }
                } else {
                    /* 淡出: 平方曲线 fade = (1-p)² (p = 归一化进度).
                     * 不需要新增状态量 —— 恒等式 sqrt(fade) = 1-p 让逐帧递推正好是
                     *   (sqrt(fade) - dp)² = (1 - (p+dp))²
                     * 所以"开方 → 减一步 → 平方"就等价于直接算 (1-p)², p 不必存.
                     * 平方 vs 线性: 前段就小下去 ⇒ 同样时长里前冲更短, 且淡完那刻的
                     * 足端残留小一个数量级 (线性时 trot/GO 留 39~40mm, 只能靠最后 0.3s
                     * 缓动滑掉 = 搓地). 常数与实测见文件上方 FADE_OUT_TURNS 注释. */
                    float step = frame_dphi / FADE_OUT_TURNS;
                    float root = sqrtf(g_stride_fade) - step;
                    g_stride_fade = (root > 0.0f) ? root * root : 0.0f;
                    if (g_stride_fade <= g_fade_target) {
                        g_stride_fade = g_fade_target;
                        g_fade_active = false;
                        ESP_LOGI(TAG, "Stride fade-out done (%.1f turns, quadratic)", FADE_OUT_TURNS);
                    }
                }
            }
        }

        // 满额步长 (还没过淡入)
        float stride_target = eff_stride;
        float stride_pre    = stride_target;
        bool  fade_full     = (!g_fade_active && g_stride_fade >= 0.999f);

        /* GO 变速平滑 (既有行为, 只服务"speed 变化导致 GO 目标 70↔50"的跳变).
           ★ 只有"GO 目标步长变化"走这里; 减速停由淡入淡出接管, 不经过本分支.
             淡入没走完时让平滑值同步跟随满额目标, 保证交回控制权那一帧不跳. */
        if (g_motion.gait == GAIT_GO) {
            if (!fade_full) {
                g_stride_smooth = stride_target;
            } else if (g_half_pulse) {
                float diff = stride_target - g_stride_smooth;
                if (fabsf(diff) > 0.1f) {
                    float step = fabsf(stride_target) / 5.0f;
                    if (step < 1.0f) step = 1.0f;
                    if (diff >  step) diff =  step;
                    if (diff < -step) diff = -step;
                    g_stride_smooth += diff;
                }
            }
            stride_pre = g_stride_smooth;
        }
        eff_stride = stride_pre * g_stride_fade;
        g_half_pulse = false;

        /* 上报本帧**定稿**的步态参数 (纯上报, 不参与任何计算)。
         * 位置: eff_duty 在 GO 分支之后就没再动过, eff_stride 的末次赋值就是上面这一句
         * ⇒ 到这儿两个都是"本帧真正会用"的值。 */
        g_eff_stride_out = eff_stride;
        g_eff_duty_out   = eff_duty;

        // 阶段 A (起步过渡): 相位钉在全踩地中点 —— 阶段 A 结束时正好落在
        // 四腿全着地 / x=0 / z=height 的站姿等价点, 阶段 B 从这里零位移交接
        if (g_stand_up) {
            g_phase = all_stance_mid(eff_duty, eff_gap) * TWO_PI;
        }

        /*
         * ============================================================
         * 淡出完成 → 切静态 (只剩"滑块拉到 0"这一条路)
         * ============================================================
         *
         * ★ 停步和换向**都不走这里了**:
         *   停步 → 回正步 (motion_enter_repos): 全踩地窗口硬切步长, 再抬脚把四腿挪回 x=0
         *   换向 → 全踩地翻转: 取反四个锚定量, 当帧完成, 不降步长不淡出
         *   剩下的这条是"速度滑块拖到 0" —— 它的语义是"原地停下", 沿用老的淡出最省事。
         *
         * 系数淡到 0 之后还不能马上切, 必须等**落在"四腿全踩地"窗口内**:
         * 支撑相锚定后脚一落地就不再回 x=0, 所以步长收到 0 时脚留在原地;
         * 等一个整周期让每腿都在步长≈0 之后重新落地 ⇒ 四脚都收敛到 x≈0 的站姿等价点。
         * 不等窗口就会有腿还在空中 (z<height), 切过去要落下来。
         *
         * 速度已归零 (相位冻结) 时不等窗口, 直接切 —— 否则会死锁.
         * 这时若还有腿在空中, 由下面的姿态限速器 (3°/帧) 把它收下来, 不是跳变.
         */
        bool fade_closed = (!g_fade_active && g_stride_fade <= 0.001f);
        if (g_motion.target_speed <= 0.1f && fade_closed) {
            int cur_dir = (g_motion.direction < 0.0f) ? -1 : 1;
            bool window_ok    = all_legs_in_stance(g_phase, eff_duty, eff_gap, cur_dir);
            bool phase_frozen = (eff_speed < 0.05f);
            if (window_ok || phase_frozen) {
                g_motion.gait = GAIT_STOP;
                g_motion.speed = 0.0f;
            }
            // else: 留在原地踏步等窗口 (步长 0 ⇒ x≡0, 腿照抬照落但不移动)
        }

        /*
         * ============================================================
         * 停步 → 回正步
         * ============================================================
         *
         * 收停信号后**保持当前步态继续走**, 直到相位落在"四腿全踩地"的那一刻 ——
         * 那里**没有腿在空中**, 所以步长可以硬切到 0 而不甩动摆动腿。
         * (离线扫过: 淡出窗口短于一个摆动相时, 摆动腿的落点目标当帧塌掉 ⇒ 顿挫。
         *  全踩地时没有摆动腿, 这条约束自动满足 —— 这是选这个时机唯一的理由。)
         *
         * 进来之后身体完全静止: speed=0 ⇒ 相位不累加, stride=0 ⇒ b 不推进。
         * 剩下的只是"把脚挪回站姿点", 由 GAIT_REPOS 的抬脚动作完成 (不再拖地)。
         */
        int repos_dir = (g_motion.direction < 0.0f) ? -1 : 1;   // ★ 独立字段
        // 相位已冻结 (speed≈0) 时不等窗口 —— 否则永远等不到 (相位不走就没有窗口),
        // 这时若还有腿在空中, 由回正步自己的限速器把它收下来, 不是跳变。
        bool repos_frozen = (eff_speed < 0.05f);
        if (g_pending_repos && !is_stand
            && (all_legs_in_stance(g_phase, eff_duty, eff_gap, repos_dir)
                || repos_frozen)) {
            g_pending_repos = false;
            motion_enter_repos(g_repos_after, repos_dir, eff_duty, eff_gap);
        }

        // 回正步计时: 走完两组抬落就收工
        if (g_motion.gait == GAIT_REPOS) {
            g_repos_timer += dt;
            if (g_repos_timer >= REPOS_TOTAL) {
                g_motion.gait = g_repos_after;
                ESP_LOGI(TAG, "Repos done -> gait %d (%.2fs)", (int)g_repos_after, REPOS_TOTAL);
            }
        }

        /*
         * ============================================================
         * 换向: 全踩地窗口直接翻转锚定量
         * ============================================================
         *
         * 全踩地时四腿都在支撑相, 而支撑相 x = x_land - (b - b_land) 里**没有相位**。
         * 于是"四个锚定量全部取反"和"方向翻转导致的 out_x = -out_x"精确抵消:
         *
         *     -( (-x_land) - ((-b) - (-b_land)) ) = x_land - (b - b_land)   ✓ 位置连续
         *
         * ⇒ 不需要降步长、不需要淡出、不需要回正 —— 当帧完成换向, 身体照常走。
         *   步长/相位/速度一个都不动, 所以也没有"起步"要重跑。
         *
         * ★ 为什么必须全踩地: 摆动相是 x_lift + (S/2 - x_lift)*ease, 带 ease 项,
         *   取反抵消不了 ⇒ 有腿在空中时会瞬移。
         * ★ 为什么还要**等到对齐点**: 上面那个"抵消"只保证翻转**那一帧**的位置连续。
         *   翻转同时会把相位表标签换掉, 每条腿的支撑相进度跟着从 t_old 变成 t_new;
         *   若某条腿正好在这帧跨过"摆动↔支撑"边界, 落地锁存会把 x_land 写死成 +S/2,
         *   位置就跳 stride*(t_old+t_new-1) —— 见 flip_phase_aligned, 实测最坏 48mm。
         */
        if (g_pending_flip && !is_stand && !g_stand_up
            && all_legs_in_stance(g_phase, eff_duty, eff_gap, g_flip_sign)
            && flip_phase_aligned(g_phase, eff_duty, eff_gap)) {
            // ★ 决定翻不翻的**只看最终方向**, 不看中间按了几次:
            //   连按两下 (+1 → -1 → +1) 方向其实没变, 这时再翻一次就是**白翻** ——
            //   翻转是"取反", 翻两次回到原地, 但中间要等一整个全踩地窗口,
            //   且日志会说"方向变了"而实际没变, 排障时会误导。
            if (g_motion.direction != (float)g_flip_sign) {
                for (int i = 0; i < 4; i++) {
                    g_gait_b[i]     = -g_gait_b[i];
                    g_leg_b_land[i] = -g_leg_b_land[i];
                    g_leg_x_land[i] = -g_leg_x_land[i];
                    g_leg_x_lift[i] = -g_leg_x_lift[i];
                }
                ESP_LOGI(TAG, "Dir flip at all-stance: %d -> %d (stride=%.0f, 无减速无回正)",
                         g_flip_sign, -g_flip_sign, g_motion.stride);
            } else {
                ESP_LOGI(TAG, "Dir flip 取消: 方向已回到 %d, 不翻", g_flip_sign);
            }
            g_pending_flip = false;
        }

        // 计算步态偏移: 正向 (前腿迈) + 反向 (后腿迈) 各一套
        // turn 可能让不同侧走向不同方向，per-leg 按方向选用
        float offsets_fwd[4], offsets_rev[4];
        compute_offsets(eff_duty, eff_gap,  1, offsets_fwd);
        compute_offsets(eff_duty, eff_gap, -1, offsets_rev);

        servo_group_begin();

        bool stand_up_converged = true;   // 阶段 A: 8 个关节是否全部进入容差
        for (int leg = 0; leg < 4; leg++) {
            float foot_x, foot_z;

            // turn 的 per-leg 缩放系数 (带符号, 用实参 g_ramp_turn): 只有 turn 同侧
            // 的那条腿会过零/变号. 不受步态分支影响 —— 站姿/回正也要用它同步
            // g_leg_dir (见各清零处: 锚定量全 0 时改方向零代价).
            float s_side;
            if (g_ramp_turn >= 0.0f) {
                s_side = (leg_side[leg] == IK_SIDE_RIGHT) ? (1.0f - g_ramp_turn * 2.0f) : 1.0f;
            } else {
                s_side = (leg_side[leg] == IK_SIDE_LEFT)  ? (1.0f + g_ramp_turn * 2.0f) : 1.0f;
            }

            /* ★★ 回正步: 身体不动, 只有脚在动 —— 对角两两**抬起来**挪回 x=0。
             *  优先级最高 (gait=GAIT_REPOS 时下面的站立/运动分支都不该跑)。
             *
             *  与旧收尾 (pose_trans=2 把脚**滑**到站姿) 的本质差别: 这里是**抬脚**,
             *  所以脚不离地这件事根本不会发生 —— 零拖地。 */
            if (g_motion.gait == GAIT_REPOS) {
                // 身体不前进 ⇒ 锚定状态清零 (与站立分支同样处理)
                g_gait_b[leg]     = 0.0f;
                g_leg_b_land[leg] = 0.0f;
                g_leg_x_land[leg] = 0.0f;
                g_leg_x_lift[leg] = 0.0f;
                g_leg_swing[leg]  = false;
                g_leg_dir[leg]    = (s_side >= 0.0f) ? 1 : -1;   // 锚定量全 0, 方向直接对齐

                // 本条腿属于哪一组 (0 = 先抬, 1 = 后抬; 顺序见 motion_enter_repos)
                int grp = 0;
                for (int gi = 0; gi < 2; gi++)
                    if (g_repos_pair[gi][0] == leg || g_repos_pair[gi][1] == leg) grp = gi;

                // 组内归一化进度 u ∈ [0,1]: 本组还没轮到 ⇒ 0, 已做完 ⇒ 1
                float u = (g_repos_timer - grp * (REPOS_STEP_TIME + REPOS_GAP))
                          / REPOS_STEP_TIME;
                if (u < 0.0f) u = 0.0f;
                if (u > 1.0f) u = 1.0f;

                foot_x = g_repos_x0[leg];
                foot_z = g_motion.height;
                /* z 抬 [0, 0.25] / x 移 [0.25, 0.75] / z 落 [0.75, 1]
                 * ★ x 只在中间那 50% 动 —— 两头留给起落, 保证**挪的时候脚已经离地** */
                if (u < 0.25f) {
                    float e = u / 0.25f;  e = e * e * (3.0f - 2.0f * e);
                    foot_z = g_motion.height - REPOS_LIFT * e;
                } else if (u < 0.75f) {
                    float e = (u - 0.25f) / 0.50f;  e = e * e * (3.0f - 2.0f * e);
                    foot_z = g_motion.height - REPOS_LIFT;
                    foot_x = g_repos_x0[leg] * (1.0f - e);
                } else {
                    float e = (u - 0.75f) / 0.25f;  e = e * e * (3.0f - 2.0f * e);
                    foot_z = g_motion.height - REPOS_LIFT * (1.0f - e);
                    foot_x = 0.0f;
                }
                // 回正结束时 g_prev_fx 必须是 0 —— 否则随后可能触发的 pose_trans=2
                // 会从"走路时那一帧的位置"开始缓动, 又拖一次
                g_prev_fx[leg] = foot_x;
                g_prev_fz[leg] = foot_z;
            }
            // 阶段 A 与站立走同一条路: 足端直接取站姿 (0, height), 不查轨迹。
            // 差别只在下面限速器 —— 站立时角已到位, 阶段 A 要从实际角爬过去。
            else if (is_stand || g_stand_up) {
                // 支撑相锚定状态清零: 站立 / 阶段 A 期间没有轨迹, 而且此刻步长为 0
                // ⇒ b = 0, x_land = 0 让支撑式给出 x ≡ 0, 与站姿一致.
                // ★ 每帧都清是关键: 任何一帧站住都把锚点归零, 所以**起步交接点必然
                //   与站姿逐位相等**, 不需要单独写一段"交接初始化" ——
                //   b 只在运动分支里累加, 站住时归零, 天然没有残留.
                g_gait_b[leg]     = 0.0f;
                g_leg_b_land[leg] = 0.0f;
                g_leg_x_land[leg] = 0.0f;
                g_leg_x_lift[leg] = 0.0f;
                g_leg_swing[leg]  = false;   // 站姿 = 四腿全踩地
                g_leg_dir[leg]    = (s_side >= 0.0f) ? 1 : -1;   // 锚定量全 0, 方向直接对齐
                if (g_motion.pose_trans == 2) {
                    // 停止过渡: 从上一帧位置缓动到站姿
                    float t = g_motion.pose_timer / TRANS_TIME;
                    float ease = t * t * (3.0f - 2.0f * t);
                    foot_x = g_prev_fx[leg] * (1.0f - ease);
                    foot_z = g_prev_fz[leg] + (eff_height - g_prev_fz[leg]) * ease;
                } else {
                    // ★ 站 / 阶段 A 的目标高度都取 eff_height (这条步态**将要用的**高度),
                    //   不是 g_motion.height —— GO 的 eff_height 恒为 70, 站高设成 50 时若按
                    //   g_motion.height 站起来, 交接那一帧会从 50 瞬跳 70 (20mm ≈ 髋 21.8°/
                    //   膝 39.0°), 正是要消掉的那个跳变. 按 eff_height 起 ⇒ 交接点
                    //   与阶段 B 首帧 (stride=0, 四腿全支撑, z=eff_height) **逐位相等**.
                    //   静止时实参已被同步成 g_motion.height (见上面的限速层), 两者本就相等.
                    foot_x = 0;
                    foot_z = eff_height;
                }
            } else {
                // 运动步态 (stride = 幅度, 0 = 原地踏步; 前后由 direction 定)

                // turn → per-side stride: 一侧不变, 另一侧 1→0→-1 连续缩放
                // 方向符号来自**独立字段** g_motion.direction
                // 换向等窗口期间保持原方向, 翻转完成后才用新方向 (防止腿瞬间打架)
                // ★ 换向等窗口期间保持**旧方向**: 方向若提前翻掉, 支撑腿的 x 会当帧取反
                //   (瞬移 2|x|, 最坏 65mm)。真正翻转只在全踩地那一帧发生, 见下面的翻转块。
                // ★ 方向现在是**独立字段** (motion_set_direction), 不再从 stride 的符号推 ——
                //   所以"换向"和"改步长"是两件互不干扰的事。
                float stride_sign;
                if (g_pending_flip)
                    stride_sign = (float)g_flip_sign;
                else
                    stride_sign = g_motion.direction;
                // 本腿的总方向 = turn 缩放符号 (g_leg_dir, 会挂起/延迟翻转) × 全局步进方向
                float step_sign = (stride_sign < 0.0f) ? -1.0f : 1.0f;
                float direction = (float)g_leg_dir[leg] * step_sign;
                float abs_stride = fabsf(eff_stride * s_side);
                float max_stride = 2.0f * g_motion.body_half_l * 0.85f;
                if (abs_stride > max_stride) abs_stride = max_stride;

                // 每腿按自身方向选正向/反向相位
                // walk 族 (duty+gap<0.5) 方向反了需翻转相位; trot (0.5) 对称无需
                bool need_rev = (direction < 0) && (eff_duty + eff_gap < 0.50f);
                float offset = need_rev ? offsets_rev[leg] : offsets_fwd[leg];
                float leg_phase = g_phase + offset * TWO_PI;
                if (leg_phase > TWO_PI) leg_phase -= TWO_PI;
                float phase_norm = leg_phase / TWO_PI;

                /* ---- 支撑相锚定 (阶段 C) ----
                 * 顺序**必须是** 锁存 → 算 x → 累加 b, 三处都不能换 (详见 foot_trajectory
                 * 顶部注释 ①②; 离线逐帧实测: 换任一处, 常步长下就差 2.2~7.1mm).
                 * 用 per-leg 的 abs_stride ⇒ turn 让左右步长不同也各自自洽. */
                bool in_swing = (phase_norm < eff_duty);
                if (in_swing && !g_leg_swing[leg]) {
                    // 刚离地: 摆动从**支撑相最后一帧的位置**起步 ⇒ 连续无跳.
                    // 理想离地瞬间是 pn = 0, 本帧已走过 pn ⇒ 把 b 回退这一小段,
                    // 否则 x_lift 会比 -stride/2 差一点 (常步长下 walk 2.2mm).
                    // (旧式从这里开始改用写死的 -stride/2, 步长一变就把空中的脚也挪了)
                    float b_at_lift = g_gait_b[leg]
                                    - (abs_stride / (1.0f - eff_duty)) * phase_norm;
                    g_leg_x_lift[leg] = g_leg_x_land[leg]
                                      - (b_at_lift - g_leg_b_land[leg]);
                } else if (!in_swing && g_leg_swing[leg]) {
                    // 刚落地: 定死锚点. 本次摆动的终点就是 +stride/2.
                    // 理想落地瞬间是 pn = duty, 本帧已越过 over ⇒ b_land 同样要回退,
                    // 否则整个支撑相都会带一个常量偏移.
                    float over = (phase_norm - eff_duty) / (1.0f - eff_duty);
                    g_leg_b_land[leg] = g_gait_b[leg] - abs_stride * over;
                    g_leg_x_land[leg] = abs_stride * 0.5f;
                }
                g_leg_swing[leg] = in_swing;

                /* ---- turn 跨 ±0.5: 挂起 + 等支撑窗口才翻 (机制/实测见 g_leg_dir 定义处) ----
                 * ★ 必须在**边沿锁存之后**: 边沿用旧表 pn 处理真实落地/离地事件,
                 *   翻转随后取反锚定量; 顺序反了 "落地锁存的 +S/2" 会绕过取反 ⇒ 跳 S. */
                if (((s_side >= 0.0f) ? 1 : -1) != g_leg_dir[leg]) {
                    int   dn     = -g_leg_dir[leg];
                    float dirn   = (float)dn * step_sign;
                    bool  nr_new = (dirn < 0.0f) && (eff_duty + eff_gap < 0.50f);
                    float off_n  = nr_new ? offsets_rev[leg] : offsets_fwd[leg];
                    float pn_new = g_phase + off_n * TWO_PI;
                    if (pn_new > TWO_PI) pn_new -= TWO_PI;
                    pn_new /= TWO_PI;
                    if (phase_norm >= eff_duty && pn_new >= eff_duty) {
                        g_leg_dir[leg]    = dn;
                        g_gait_b[leg]     = -g_gait_b[leg];
                        g_leg_b_land[leg] = -g_leg_b_land[leg];
                        g_leg_x_land[leg] = -g_leg_x_land[leg];
                        g_leg_x_lift[leg] = -g_leg_x_lift[leg];
                        direction  = dirn;      // 本帧改用新表/新方向
                        phase_norm = pn_new;
                    }
                }

                foot_trajectory(phase_norm, abs_stride,
                                eff_height, eff_lift,
                                eff_duty, direction,
                                g_leg_x_lift[leg], g_gait_b[leg],
                                g_leg_b_land[leg], g_leg_x_land[leg],
                                &foot_x, &foot_z);

                // ★ b 最后推进: 上面读到的 g_gait_b[leg] 必须对应**本帧的 phase_norm**
                g_gait_b[leg] += (abs_stride / (1.0f - eff_duty)) * frame_dphi;
            }

            // 保存当前帧足端 (停过渡用)
            // ★ 必须存在加偏移/加补偿**之前** —— 存的是轨迹原始值。
            //   停过渡 (上面 is_stand + pose_trans==2 那段) 拿它插值, 插值完下面还会
            //   再加一次偏移和补偿; 要是这里存的是加过的值, 那一帧就会偏移+补偿各算
            //   两次 → 切 stop 时脚瞬跳 co (默认 0 看不出来) + z_pitch (pitch=-5°
            //   时 5.5mm ≈ 2~3° 的抖)。
            g_prev_fx[leg] = foot_x;
            g_prev_fz[leg] = foot_z;

            // 中位点偏移: 中点平移 (center_offset) + 前/后腿间距的一半 (±spacing/2)
            //   —— 间距正 = 展开 (前腿往前 + 后腿往后), 负 = 收窄, 见 set_center_spacing 注释。
            // + 相位联动摆动 (sway): t = g_phase/2π (0 = LH 抬脚), 见 SWAY_* 常量注释。
            //   符号: 足端图案 +Δ ⇒ 重心相对支撑 -Δ; 这里给的是足端上的偏移。
            float sway_x = -g_ramp_sway * g_sway_dir
                         * cosf(2.0f * g_phase - SWAY_DELTA * 4.0f * (float)M_PI);
            foot_x += g_ramp_center
                    + ((leg_pair[leg] == IK_LEG_FRONT) ?  g_ramp_spacing * 0.5f
                                                       : -g_ramp_spacing * 0.5f)
                    + sway_x;

            /* 身体姿态补偿: roll/pitch → 四腿高度偏置
             *
             * 机身在机身系里倾斜 p 之后, 地面在那个坐标系里是个斜面 z = h + x·tan(p)。
             * 足端要落在这个面上, z 偏置就得随 x 连续变化 —— 而 x 是**足端在机身系里的
             * 绝对坐标**, 也就是「髋的 x + 这条腿轨迹算出的相对 x」。
             *
             * ★ 容易错的两个地方, 实测都踩过 (walk, pitch=+10°, 三足平面在机身系里的角度):
             *   ① 只用 foot_x (漏掉髋) —— 站姿时 foot_x 恒为 0 ⇒ 补偿整个消失, 狗根本
             *      不倾斜, 实测只做到 **1.45°**;
             *   ② 按"前腿 / 后腿"分两档给常量 (= 假设足端恒在髋正下方) —— 步态里前后腿的
             *      x 都在 ±stride/2 之间走、**范围重叠**, 区分不开 ⇒ pitch 差 1.4° 且 roll
             *      在 ±3.2° 之间随支撑组合来回翻;
             *   加上髋之后才是 **(10.00°, 0.00°)**, 四个支撑组合全部一致。
             *
             * ★ 横滚**不用改**: 足端的 y 恒为 ±body_half_w (脚在左右方向不移动),
             *   常量偏置本来就等价于"按该腿自己的 y 算"。实测 roll=±10° 现状即精确。
             *
             * ★ 用 tan 而不是 sin: 平面方程 z = h + x·tan(p) 的法向恰好给出倾角 p
             *   (实测精确 10.00°); 换成 sin 得到的是 atan(sin p) = 9.85°, 反而偏了。
             *
             * 注: 用加过中位偏移 (中点 + 间距) 的 foot_x —— 足端实际的位置就包含那些偏移。
             */
            float deg2rad = 0.0174533f;
            float z_roll = g_motion.body_half_w * tanf(g_ramp_roll * deg2rad);
            if (leg_side[leg] == IK_SIDE_LEFT)
                foot_z -= z_roll;   else foot_z += z_roll;
            float leg_x = ((leg_pair[leg] == IK_LEG_FRONT) ? g_motion.body_half_l
                                                           : -g_motion.body_half_l)
                          + foot_x;
            foot_z += leg_x * tanf(g_ramp_pitch * deg2rad);

            ik_result_t ik = ik_solve_2dof(foot_x, foot_z,
                                g_motion.ik_L1, g_motion.ik_L2,
                                leg_side[leg], leg_pair[leg]);
            // 姿态限速: 站立/停步过渡 + 起步过渡 (阶段 A) 每帧最多 SERVO_MAX_DEG_PER_FRAME 度
            // ★ 闸门是"角度未收敛", 不是"指令是站住" —— 后者 (is_stand) 问错了问题,
            //   导致行走时整个"从真实姿态平滑起来"的机制被绕开.
            // ★ 从**实际角**出发: g_smooth_angles 在进 MODE_MOTION 时已同步真实舵机角
            //   (motion_set_mode), 所以坐/蹲/玩/被 Python 摆过都自动适用 —— 限速器
            //   不需要知道狗在哪、摆成什么样, 只需要知道当前角.
            // 回正步也套限速器: 时间曲线已经限了速率, 这是第二道保险
            // (防止 REPOS_STEP_TIME 被设得过小 ⇒ 舵机被要求跳)
            // ★ 步态循环也套限速器 (2026-10-10 加): 慢速姿态过渡用 3°/帧,
            //   步态循环用 GAIT_MAX_DEG_PER_FRAME —— **两个常量**, 见文件上方说明.
            //   支撑相只要 1~100 °/s (0.02~2°/帧), 永远碰不到步态那道限制,
            //   所以"该保护的自动保护住了": 只有摆动相的峰值会被削.
            {
                float step = (is_stand || g_stand_up || g_motion.gait == GAIT_REPOS)
                             ? SERVO_MAX_DEG_PER_FRAME
                             : GAIT_MAX_DEG_PER_FRAME;
                float hip_raw  = ik.hip_deg;
                float knee_raw = ik.knee_deg;
                ik.hip_deg  = servo_step_toward(leg_hip_ch[leg],  hip_raw,  step);
                ik.knee_deg = servo_step_toward(leg_knee_ch[leg], knee_raw, step);
                if (g_stand_up &&
                    (fabsf(ik.hip_deg  - hip_raw)  > STAND_UP_TOL ||
                     fabsf(ik.knee_deg - knee_raw) > STAND_UP_TOL)) {
                    stand_up_converged = false;
                }
            }
            g_smooth_angles[leg_hip_ch[leg]]  = ik.hip_deg;
            g_smooth_angles[leg_knee_ch[leg]] = ik.knee_deg;
            servo_group_add(leg_hip_ch[leg],  ik.hip_deg);
            servo_group_add(leg_knee_ch[leg], ik.knee_deg);
        }

        // 阶段 A 结束: 8 个关节全部进入容差 → 清 g_stand_up, 下一帧进阶段 B.
        // 此刻相位仍钉在 all_stance_mid, 步长为 0 ⇒ 与站姿精确相等, 交接零位移.
        if (g_stand_up && stand_up_converged) {
            g_stand_up = false;
            ESP_LOGI(TAG, "Stand-up done -> stride ramp (gait=%d height=%.1f)",
                     g_motion.gait, g_motion.height);
        }

        servo_group_commit();

        vTaskDelayUntil(&last_wake, period);
    }
}

/* ---- API ---- */

void motion_task_start(void)
{
    if (g_task_handle != NULL) return;
    motion_load_geometry();  // 上电自动从 NVS 加载 L1/L2/髋距
    // 实参限速层同步一次: Python 可能在任务启动**之前**就改过参数 (main.py 或用户的
    // REPL 指令), 那时没人推进限速层 ⇒ 实参还停在编译期初值. 不补这一次, 第一帧就会
    // 从初值往目标"挪", 白白多走一段.
    g_ramp_height  = g_motion.height;
    g_ramp_lift    = g_motion.lift_height;
    g_ramp_center  = g_motion.center_offset;
    g_ramp_spacing = g_motion.center_spacing;
    g_ramp_sway    = 0.0f;   // 摆动从 0 淡入 (起步时步长系数本来也是 0, 目标就是 0)
    g_ramp_pitch   = g_motion.body_pitch;
    g_ramp_roll    = g_motion.body_roll;
    g_ramp_turn    = g_motion.turn;
    xTaskCreatePinnedToCore(motion_task_main, "motion", MOTION_STACK,
                            NULL, MOTION_PRIORITY, &g_task_handle, MOTION_CORE);
}

const motion_state_t *motion_get_state(void) { return &g_motion; }

/* 本帧**实际生效**的步态参数 (只读)。
 *
 * 与 get_params() 的区别: 那里报的是**用户设进去的** stride/duty; 这里报的是经过
 * GO 覆盖 + 步长淡入淡出之后、腿循环真正在用的值。做转弯归一化的 Python 侧要的是后者
 * —— 换算 "turn=1 对应多少偏航角速度" 必须用**真值**, 否则 GO 下会算错。
 *
 * ⚠ 运动任务没跑时这两个值停在初值 (不会更新), 别拿它当"当前步态"用。 */
void motion_get_effective(float *eff_stride, float *eff_duty)
{
    if (eff_stride) *eff_stride = g_eff_stride_out;
    if (eff_duty)   *eff_duty   = g_eff_duty_out;
}

void motion_set_gait(gait_type_t gait)
{
    if (servo_safety_latched()) {
        ESP_LOGW(TAG, "safety latched: refuse set_gait");
        return;
    }
    if (gait < GAIT_COUNT) {
        // 任何步态指令 → 自动进入运动模式 (Python 无需显式切换)
        // ★ "从静止起步" 的判据 = 当前不在运动模式, 或当前步态是静态.
        //   只看 g_motion.gait 不够: 狗在坐姿 (MODE_POSE) 时 gait 保留着上次的运动步态,
        //   这时按前进会被当成"同态切换", 跳过站起来那一步.
        motion_mode_t prev_mode = g_mode;
        bool from_static = is_static_gait(g_motion.gait) || (prev_mode != MODE_MOTION);
        motion_set_mode(MODE_MOTION);
        g_motion.enabled = true;
        // 运动步态 → 静态步态 —— 停步走**回正步**:
        //   收停信号后**保持当前步态继续走**, 等下一个"四腿全踩地"窗口, 在那里硬切步长,
        //   然后进回正步 (对角抬脚把四腿挪回 x=0)。
        //   ★ 不走淡出: 淡出那 1.25 周期存在的理由是"等每条腿都在步长归零后重新落地一次"
        //     (脚一落地世界坐标就定死, 步长归零不会让它回 x=0) —— 而回正步直接把脚**抬起来挪**,
        //     不需要等。省掉的那 1.25 周期正好是它比淡出快的地方。
        //   ★ 换向也不走淡出 —— 它走"全踩地直接翻转锚定量", 连步长都不用降。
        if (is_static_gait(gait) && !is_static_gait(g_motion.gait)) {
            g_pending_repos = true;
            g_pending_flip  = false;      // 停步优先于换向
            g_repos_after   = gait;
            ESP_LOGI(TAG, "Stop requested -> 走到下一个全踩地窗口进回正步");
            return;
        }
        // ★ 回正步进行中再收到静态步态指令 ⇒ 直接忽略, 让它做完。
        //   不挡的话会走到函数尾部把 gait 改成 GAIT_STOP, 把回正打断在抬脚半路
        //   (is_static_gait 现在含 GAIT_REPOS, 所以它既进不了上面那个分支, 也拦不住尾部赋值)。
        if (g_motion.gait == GAIT_REPOS && is_static_gait(gait)) {
            return;
        }
        // 静态→运动 或 同态切换: 立即生效, 并取消挂起的回正
        if (!is_static_gait(gait)) {
            g_pending_repos = false;
            /* ★ 挂起的换向在这里**必须保留**, 不能取消。
             * 取消会留下**半完成态**: 换向指令的语义是"等下一个全踩地窗口翻锚定量", 这个等待是
             * **固有**的 (摆动腿不能翻, 翻了会瞬移)。而"取消"会留下一个**半完成态** ——
             * direction 已经是新值, 锚定量却还是旧朝向 ⇒ stride_sign 提前翻号,
             * 支撑腿当帧 x 取反, 瞬移最坏 2×(stride/2) = 70mm; 更糟的是
             * g_dir_last 在检测那一帧就跟着更新了, 所以**不会再补翻一次**, 是永久错位。
             *
             * 触发它并不难: ① 积木「后退」块 = set_direction(-1) + set_gait('go'),
             *   放在循环里连按两次, 第二次的 set_gait('go') 就把第一次挂起的翻转吃掉;
             * ② 网页方向键连点同理。**同态重复下发**是常态, 不能当成"改步态"。
             *
             * 保留挂起是对的: 翻转只是"把四个锚定量取反, 让它们跟 direction 一致",
             * 跟步态无关 —— 新步态的 duty/gap 只改变全踩地窗口**什么时候**到来, 不改翻转的
             * 正确性 (窗口每个周期都有, 最坏晚一个周期完成)。等待期间 stride_sign 一直读
             * g_flip_sign (旧方向), 与未翻转的锚定量自洽 ⇒ 全程无跳变。
             *
             * (g_pending_repos 仍然要取消: 回正是"停车收尾", 用户又给了运动指令 ⇒ 不收尾了。) */
            if (from_static) {
                // 起步两段式 (trot / walk / go 通用):
                //   阶段 A —— 不管当前是什么姿态 (站立/坐/蹲/玩/被 Python 摆过),
                //             先无条件过渡到站姿 (0, height), 每帧 3°, 跑到收敛;
                //   阶段 B —— 随后步长系数从 0 淡到 1, walk 段 2 个周期 / trot 段 1 个.
                // 目标用 eff_height 而**不是** Python stand() 的固定 70 ⇒ 没有残留高度差.
                g_stand_up    = true;
                g_stride_fade = 0.0f;      // 步长从 0 起
                g_fade_target = 1.0f;
                g_fade_active = false;     // 阶段 A 期间不动; 阶段 A 一结束淡入块自动 armed
            }
        }
        g_motion.gait = gait;
        if (gait != GAIT_STOP)
            motion_apply_gait_params(gait);
    }
}

/* ---- 参数校验 ---- */

// 足端位置角度校验 (复制 ik.c 公式, 不钳位) — 返回 true = 髋/膝超限或不可达
static bool ik_pos_check(float x, float z, int side, int leg_pair,
                         float L1, float L2)
{
    float L1L2_max = L1 + L2;
    float L2L1_min = fabsf(L2 - L1);
    float d = sqrtf(x * x + z * z);
    if (d > L1L2_max - 1.0f) return true;   // 超机械可达
    if (d < L2L1_min + 1.0f) return true;   // 足端过近 (腿叠死)

    float cos_knee = (L1 * L1 + L2 * L2 - d * d) / (2.0f * L1 * L2);
    if (cos_knee > 1.0f) cos_knee = 1.0f;
    if (cos_knee < -1.0f) cos_knee = -1.0f;
    float knee_angle = acosf(cos_knee);

    float alpha = atan2f(z, x);
    float cos_beta = (L1 * L1 + d * d - L2 * L2) / (2.0f * L1 * d);
    if (cos_beta > 1.0f) cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    float beta = acosf(cos_beta);

    float hip_angle;
#if IK_KNEE_REAR_FORWARD
    hip_angle = (leg_pair == IK_LEG_REAR) ? (alpha - beta) : (alpha + beta);
#else
    hip_angle = alpha + beta;
#endif

    float hip_deg = hip_angle * 180.0f / (float)M_PI;
    if (side == IK_SIDE_RIGHT) hip_deg = 180.0f - hip_deg;

    int knee_mirror = (side == IK_SIDE_RIGHT);
#if IK_KNEE_REAR_FORWARD
    if (leg_pair == IK_LEG_REAR) knee_mirror = !knee_mirror;
#endif
    float knee_deg = knee_angle * 180.0f / (float)M_PI;
    if (knee_mirror) knee_deg = 180.0f - knee_deg;

    bool bad = (hip_deg < ik_hip_min || hip_deg > ik_hip_max ||
                knee_deg < ik_knee_min || knee_deg > ik_knee_max);
    if (bad) {
        const char *legn = (leg_pair == IK_LEG_FRONT) ? "前腿" : "后腿";
        const char *siden = (side == IK_SIDE_LEFT) ? "左" : "右";
        ESP_LOGW(TAG, "    %s%s: hip=%.1f°(限%.0f~%.0f) knee=%.1f°(限%.0f~%.0f)",
                 siden, legn, hip_deg, ik_hip_min, ik_hip_max,
                 knee_deg, ik_knee_min, ik_knee_max);
    }
    return bad;
}

// 足端轨迹采样校验 —— 把**参与轨迹的全部 7 个量**一起判, 返回 true = 有问题, 参数不应写入。
//
// ★ 收 7 个参数而不是自己读全局, 是为了让 set_body_pose / set_center / set_center_spacing
//   也能拿**新值**跑一遍。
//   那几个量同样进轨迹 (center_offset 平移整条 x; center_spacing 前腿 +s/2、后腿 -s/2;
//   roll/pitch 逐腿叠加 z), 但它们自己
//   以前只查量级, 于是能改成一个"单看合法、组合起来够不着"的值 (docs/error.md §2.2 #2):
//   默认 stride=70 height=70 时脚已经伸到 78.3mm, 离可达上限 84 只剩 5.7mm ⇒ 重心实际
//   最多只能挪 11.4mm, 而 ±20 那个闸门会放行 15 / 20。运行时 ik_solve_2dof 只把 d 钳到
//   84 静默吞掉伸展, 表现为脚在地上蹭、狗歪着走, **绝不会报错**。
static bool traj_combo_bad(float stride, float height, float lift,
                           float center_offset, float center_spacing,
                           float sway, float roll, float pitch)
{
    float L1 = g_motion.ik_L1;
    float L2 = g_motion.ik_L2;
    float x  = fabsf(stride) * 0.5f;
    float half_spacing = center_spacing * 0.5f;   // 前腿 +half / 后腿 -half
    bool  bad = false;

    // 机械步幅上限
    float max_stride = 2.0f * g_motion.body_half_l * 0.85f;
    if (fabsf(stride) > max_stride) {
        ESP_LOGW(TAG, "⚠ 步幅过大: |stride|=%.0f > max=%.0f (前后脚干涉), 参数未写入",
                 fabsf(stride), max_stride);
        bad = true;
    }

    // 高度下限 (腿折叠干涉)
    if (height < MIN_HEIGHT) {
        ESP_LOGW(TAG, "⚠ 高度过低: height=%.0f < min=%.0fmm, 参数未写入", height, MIN_HEIGHT);
        bad = true;
    }

    // 机体姿态补偿量 —— 与腿循环里加 foot_z 的那段**同一套公式**, 改一处必须改另一处。
    //   roll : 左腿 -z_roll / 右腿 +z_roll —— 足端的 y 恒为 ±body_half_w, 常量偏置即正确。
    //   pitch: 用**足端在机身系里的绝对 x** (= 髋的 x + 相对 x) —— 地面在机身系里是斜面
    //          z = h + x·tan(p)。只写 xv (漏掉髋) 或只按前后腿分两档都是错的, 见腿循环
    //          那里的注释和实测数字。
    // ★ 必须算进来: 漏了它时校验看的是"没补偿的 z", 而狗实际走的是"补偿后的 z"。
    float deg2rad = 0.0174533f;
    float z_roll    = g_motion.body_half_w * tanf(roll * deg2rad);
    float tan_pitch = tanf(pitch * deg2rad);

    // 遍历足端实际摆动轨迹采样点 (非笛卡尔积!)
    // 摆动相: x=-S/2+S·ease, z=height-lift·sin(ease·π)
    //   x 最远时 z=height(着地), 抬腿最高时 x 在中点 — 实际轨迹
    // ★ 不加 if (stride != 0): stride=0 是**原地踏步**, 腿照样按 lift 抬起来
    //   (foot_trajectory 的抬腿项与 stride 无关)。此时 x 退化成 center_offset,
    //   正是真实足端位置。以前这层 if 把 stride=0 的校验整个跳过:
    //   z 可以要求抬到负高度、高度也可以设到够不着, 全都静默放行。
    float ease_pts[5] = { 0.0f, 0.25f, 0.5f, 0.75f, 1.0f };
    float min_zf  = height;   // 补偿后最深的足端 (循环后统一报一次, 免得刷 20 行日志)
    int   min_leg = 0;
    // 相位联动摆动的相位表: 各腿抬脚相位 (前向), 顺序与 compute_offsets 一致 [LF,LH,RF,RH]
    static const float sway_start[4] = { 0.24f, 0.0f, 0.74f, 0.50f };

    for (int i = 0; i < 5; i++) {
        float ease = ease_pts[i];
        float xv0 = -x + 2.0f * x * ease;
        xv0 += center_offset;
        float zv = height - lift * sinf(ease * (float)M_PI);
        for (int k = 0; k < 4; k++) {
            // 中位点间距: 跟运行时腿循环 (foot_x += ...) 同一套 —— 前腿 +s/2, 后腿 -s/2
            float xv = xv0 + ((leg_pair[k] == IK_LEG_FRONT) ? half_spacing
                                                            : -half_spacing);
            // 相位联动摆动: 与运行时同一相位关系 (该腿在 ease 点的时刻 = 抬脚相位 + ease·duty)。
            // 只查**前向**相位表 —— 逆向是它的镜像、裕量同值 (见 SWAY_* 注释);
            // duty 取 walk 的 0.20: 摆动只在 walk 族非零, 那时 duty 最贴近 0.20。
            xv += -sway * cosf(4.0f * (float)M_PI
                               * (sway_start[k] + ease * 0.20f - SWAY_DELTA));
            float leg_x = ((k == 0 || k == 2) ? g_motion.body_half_l
                                              : -g_motion.body_half_l) + xv;
            float zf = zv + ((leg_side[k] == IK_SIDE_LEFT) ? -z_roll : z_roll)
                          + leg_x * tan_pitch;
            if (zf < min_zf) { min_zf = zf; min_leg = k; }
            if (zf < 0.0f) continue;   // 入地由循环后统一报一次, 这里跳过角度检查免得刷屏
            if (ik_pos_check(xv, zf, leg_side[k], leg_pair[k], L1, L2)) {
                // ★ 报**全部 6 个**量: 够不着未必是 stride/height/lift 的错, 也可能是
                //   center/pitch/roll 把足端推出去了。只报前三个就是"骂错参数"。
                ESP_LOGW(TAG, "⚠ 足端 (x=%.0f z=%.0f 腿%d) 髋/膝超限, 参数未写入 "
                         "(stride=%.0f height=%.0f lift=%.0f center=%.1f spacing=%.1f "
                         "sway=%.1f roll=%.1f pitch=%.1f)",
                         xv, zf, k, stride, height, lift, center_offset,
                         center_spacing, sway, roll, pitch);
                bad = true;
            }
        }
    }

    // 补偿后的 z 必须有下限: 上面只查了 ik_pos_check (可达距离 + 髋/膝角度),
    //   它拦不住"略低于地面"的点 —— x 大、z 略负时 d=√(x²+z²) 仍在可达范围内,
    //   角度也可能合规, 于是脚被要求插到地面以下。机体姿态压得狠时会碰到。
    if (min_zf < 0.0f) {
        ESP_LOGW(TAG, "⚠ 足端入地 (抬腿过度): 最深 z=%.1fmm (腿%d, 叠加姿态补偿后) —— "
                 "lift=%.0f 超过 height=%.0f, 或姿态/间距压太狠 "
                 "(center=%.1f spacing=%.1f sway=%.1f roll=%.1f pitch=%.1f), 参数未写入",
                 min_zf, min_leg, lift, height, center_offset, center_spacing,
                 sway, roll, pitch);
        bad = true;
    }
    return bad;
}

// set_params 专用: 它只改 stride/height/lift, 其余四个量取当前存储值。
// 其余三个 setter 不走这里 —— 它们直接调 traj_combo_bad() 并传自己的新值。
static bool motion_validate_params(float stride, float height, float lift)
{
    return traj_combo_bad(stride, height, lift,
                          g_motion.center_offset, g_motion.center_spacing,
                          g_motion.sway_amp,
                          g_motion.body_roll, g_motion.body_pitch);
}

bool motion_set_params(float stride, float lift, float height)
{
    // ★ 方向已独立成参数 ⇒ 步长只剩**幅度**这一个含义, 负数没有立足之地。
    //   放在校验之前判: 负数是"用错了 API"而不是"数值超限", 提示词要对症
    //   (超限那句会说"跟姿态组合够不着", 对负数完全是误导)。
    //   写法 !(a >= 0) 同时拦住负数与 NaN —— 和下面 traj_combo_bad 的写法同源。
    if (!(stride >= 0.0f)) {
        ESP_LOGW(TAG, "⚠ 步长只表幅度 (收到 %.1f) —— 方向请用 set_direction(±1); 参数未写入", stride);
        return false;
    }

    // 步长/抬脚/站立高度是**同一条足端轨迹的三个维度**, 一起判、一起写。
    // 以前 speed 挤在这里, 且 lift 住在另一个 setter 里互相拿对方的旧值校验
    // (docs/error.md §2.2 #7 的顺序耦合), 现在三个值只有一个来源, 耦合从根上消失。
    if (motion_validate_params(stride, height, lift)) {
        ESP_LOGW(TAG, "→ 保持原值: stride=%.0f lift=%.0f height=%.0f",
                 g_motion.stride, g_motion.lift_height, g_motion.height);
        return false;
    }

    // ★ 三行必须连着走完 —— 中途被拒时一个都不写, 避免出现"新 stride + 旧 lift"的中间态
    //   (那种组合从没被 motion_validate_params 判过)。
    g_motion.stride      = stride;  // 幅度 (≥0); 0 = 原地踏步 —— 方向见 direction
    g_motion.lift_height = lift;    // 抬脚高度
    g_motion.height      = height;  // 站立高度
    ESP_LOGI(TAG, "Params: stride=%.0f lift=%.0f height=%.0f", stride, lift, height);
    return true;
}

// 速度/步频 (0~10) —— 与轨迹无关, 任何时刻都能单独改, 不碰 stride/lift/height。
// 以前它挤在 motion_set_params 里, 于是"只改速度"也得把 stride/height 一起重发。
bool motion_set_speed(float speed)
{
    // ★ 写法: !(a >= min && a <= max) 而非 (a < min || a > max)。两者等价, 但前者对 NaN
    //   也成立 (NaN 参与任何比较恒为假 ⇒ 取反为真) —— 挡住 NaN 一路乘进步态相位。
    if (!(speed >= 0.0f && speed <= 10.0f)) {
        ESP_LOGW(TAG, "⚠ 速度超限: speed=%.1f (允许 0~10), 参数未写入, 保持原值 %.2f",
                 speed, g_motion.target_speed);
        return false;
    }
    g_motion.target_speed = speed;  // 纯频率, ≥0
    ESP_LOGI(TAG, "Speed: %.2f (omega=%.3f rad/s, cycle=%.1fs)",
             speed, g_motion.omega_base * speed,
             TWO_PI / (g_motion.omega_base * speed + 0.001f));
    return true;
}

/* ---- 几何 setter: 非法输入一律拒写并保持原值 ----
 *
 * 复用运动参数那套现成模式 (见 motion_set_params): 返回 bool 表示是否写入成功,
 * 由 MicroPython 绑定层 (motion_task_mpy.c) 把"被拒"打到用户看得见的地方 —— 现在
 * 返回值还会透传给 MicroPython 调用方, 所以 KittenBlock 不必再靠 get_params() 读回比对。
 *
 * ⚠ 为什么必须校验: 这几个值直接进 NVS, 而 **NVS 跨固件升级存活** —— 一旦写坏,
 *   重刷固件也救不回来 (每次开机 motion_load_geometry() 又会把坏值读回来),
 *   所以读取那侧也必须校验, 见 motion_load_geometry()。
 *   写坏的两种典型后果:
 *     set_joint_limits(min>max) → ik_solve_2dof 的两句顺序钳位把所有角度压成 max,
 *                                 四条腿卡在同一角, 且 ik_pos_check 拒绝一切采样
 *                                 → 所有 set_params 被拒 → 狗完全不能动;
 *     cal_ik(0, 0)              → ik.c 的 cos_knee 除以 0 → NaN 一路穿到舵机占空比。
 *
 * ⚠ 关于写法: 一律用 !(a >= min && a <= max) / !(a < b), 不用 (a < min || a > max)。
 *   两者等价, 但前者对 NaN 也成立 (NaN 参与任何比较恒为假 → 取反为真), 顺带挡住 NaN。
 */

// 限位合法性: 每项都要落在舵机可达角度内, 且 min < max。
// ★ 必须**配对**校验 —— 单看一个数看不出顺序反了 (0 和 180 各自都合法)。
// setter 和 NVS 读取两条路径共用这一份判定, 避免"写入时收、重启后被盗回默认"的不对称。
static bool geom_limits_valid(float hip_min, float hip_max,
                              float knee_min, float knee_max)
{
    if (!(hip_min  >= SERVO_ANGLE_MIN && hip_min  <= SERVO_ANGLE_MAX)) return false;
    if (!(hip_max  >= SERVO_ANGLE_MIN && hip_max  <= SERVO_ANGLE_MAX)) return false;
    if (!(knee_min >= SERVO_ANGLE_MIN && knee_min <= SERVO_ANGLE_MAX)) return false;
    if (!(knee_max >= SERVO_ANGLE_MIN && knee_max <= SERVO_ANGLE_MAX)) return false;
    if (!(hip_min  < hip_max))  return false;
    if (!(knee_min < knee_max)) return false;
    return true;
}

// 脚中位偏移合法性: 上限 = 当前大腿长度的一半 (用 cal_ik 改腿长时上限跟着变)。
// ★ 这是人定的**舒适区**, 不是物理极限 —— 站高 70mm 时腿其实能挪到约 ±46mm,
//   但那样髋角已扭到 33°、腿接近伸直, 属于"够得着但姿态难看"。取 L1/2 时髋角
//   不超过 16°, 姿态好看。
// ★ 这是**量级**闸门, 判据只有一条: |off| ≤ 大腿的一半。它**拦不住"组合起来够不着"** ——
//   同一个 15mm, stride=50 时合法 (组合上限 19.4mm), stride=70 时已经超
//   (足端 x=50 → d=86.0 > 上限 84), 而运行时 ik_solve_2dof 只把 d 钳到 84 静默吞掉伸展
//   (脚在地上蹭、狗歪着走), 不报错。
//   所以 set_center 后面还有第二道 traj_combo_bad() 组合校验, 别以为这里就是全部。
//   保留本函数有两个理由: (1) 便宜, 且能给出"超了多少"这种直白的量级报错;
//   (2) NVS 载入路径 (motion_load_geometry) 也用它 —— 那里刻意**只**判量级:
//       载入发生在 stride/height 还是默认值的时候, 若按组合判, 一个当初合法的 center
//       会在用户改小步长后于下次开机被静默丢掉, 那是更糟的行为。
// NaN 也走到 false (NaN 参与比较恒为假) → 视为非法, 无需额外判断。
static bool geom_center_offset_valid(float off)
{
    return fabsf(off) <= g_motion.ik_L1 * 0.5f;
}

// 中位点**间距**的量级闸门: |s| ≤ 2×前后髋半距 (宽松; 单腿等效偏移只有 s/2)。
//   ★ 与 offset 的 ±L1/2 刻意不同 —— 间距的可用范围本来就大 (低站高配平点 s≈66,
//     H=30 时到 ~82), 真闸门是后面的 traj_combo_bad 组合校验 (够不够得着)。
//     本函数只防 NaN/荒谬值 + NVS 载入时的量级复查 (与 "co" 同策略)。
// NaN 参与比较恒假 → false, 与 geom_center_offset_valid 同源。
static bool geom_center_spacing_valid(float s)
{
    return fabsf(s) <= 2.0f * g_motion.body_half_l;
}

bool motion_cal_ik(float L1, float L2)
{
    if (!(L1 >= IK_LEN_MIN && L1 <= IK_LEN_MAX) ||
        !(L2 >= IK_LEN_MIN && L2 <= IK_LEN_MAX)) {
        ESP_LOGW(TAG, "IK cal rejected: L1=%.2f L2=%.2f 超出 [%.0f, %.0f], "
                 "保持原值 L1=%.1f L2=%.1f", L1, L2, IK_LEN_MIN, IK_LEN_MAX,
                 g_motion.ik_L1, g_motion.ik_L2);
        return false;
    }
    g_motion.ik_L1 = L1;
    g_motion.ik_L2 = L2;
    motion_save_geometry();
    ESP_LOGI(TAG, "IK cal: L1=%.1f L2=%.1f (saved)", L1, L2);
    return true;
}

bool motion_set_body_dims(float half_l, float half_w)
{
    if (!(half_l >= IK_LEN_MIN && half_l <= IK_LEN_MAX) ||
        !(half_w >= IK_LEN_MIN && half_w <= IK_LEN_MAX)) {
        ESP_LOGW(TAG, "Body dims rejected: half_l=%.2f half_w=%.2f 超出 [%.0f, %.0f], "
                 "保持原值 half_l=%.1f half_w=%.1f", half_l, half_w,
                 IK_LEN_MIN, IK_LEN_MAX,
                 g_motion.body_half_l, g_motion.body_half_w);
        return false;
    }
    g_motion.body_half_l = half_l;
    g_motion.body_half_w = half_w;
    motion_save_geometry();
    ESP_LOGI(TAG, "Body dims: half_l=%.1f half_w=%.1f (saved)", half_l, half_w);
    return true;
}

bool motion_set_joint_limits(float hip_min, float hip_max,
                              float knee_min, float knee_max)
{
    if (!geom_limits_valid(hip_min, hip_max, knee_min, knee_max)) {
        ESP_LOGW(TAG, "Joint limits rejected: hip[%.0f~%.0f] knee[%.0f~%.0f] "
                 "(需 min<max 且落在舵机行程 [%d, %d] 内), 保持原值 "
                 "hip[%.0f~%.0f] knee[%.0f~%.0f]",
                 hip_min, hip_max, knee_min, knee_max,
                 (int)SERVO_ANGLE_MIN, (int)SERVO_ANGLE_MAX,
                 ik_hip_min, ik_hip_max, ik_knee_min, ik_knee_max);
        return false;
    }
    ik_hip_min  = hip_min;
    ik_hip_max  = hip_max;
    ik_knee_min = knee_min;
    ik_knee_max = knee_max;
    motion_save_geometry();
    ESP_LOGI(TAG, "Joint limits: hip[%.0f~%.0f] knee[%.0f~%.0f] (saved)",
             hip_min, hip_max, knee_min, knee_max);
    return true;
}

/* ---- 几何参数 NVS 持久化 ---- */
#define GEOM_NVS_NS  "bpuppy_geom"

void motion_load_geometry(void)
{
    nvs_handle_t handle;
    if (nvs_open(GEOM_NVS_NS, NVS_READONLY, &handle) != ESP_OK) {
        ESP_LOGI(TAG, "Geometry NVS not found, using defaults: "
                 "L1=%.1f L2=%.1f BL=%.1f BW=%.1f hip[%.0f~%.0f] knee[%.0f~%.0f]",
                 g_motion.ik_L1, g_motion.ik_L2,
                 g_motion.body_half_l, g_motion.body_half_w,
                 ik_hip_min, ik_hip_max, ik_knee_min, ik_knee_max);
        return;
    }

    // ★ 读进来也要校验, 不能只靠 setter 挡。
    //   NVS 跨固件升级存活: 一块**旧固件**写坏的板子, 刷上新固件后每次开机照样把坏值
    //   读回来 —— 不在这里回退, 那块板子就再也救不回来了。
    int32_t val;

    if (nvs_get_i32(handle, "l1", &val) == ESP_OK) {
        float v = val / 100.0f;
        if (v >= IK_LEN_MIN && v <= IK_LEN_MAX) {
            g_motion.ik_L1 = v;
        } else {
            g_motion.ik_L1 = IK_L1_DEFAULT;
            ESP_LOGW(TAG, "NVS 里 L1=%.1f 非法 (需 %.0f~%.0f) -> 回退默认 %.1f",
                     v, IK_LEN_MIN, IK_LEN_MAX, IK_L1_DEFAULT);
        }
    }
    if (nvs_get_i32(handle, "l2", &val) == ESP_OK) {
        float v = val / 100.0f;
        if (v >= IK_LEN_MIN && v <= IK_LEN_MAX) {
            g_motion.ik_L2 = v;
        } else {
            g_motion.ik_L2 = IK_L2_DEFAULT;
            ESP_LOGW(TAG, "NVS 里 L2=%.1f 非法 (需 %.0f~%.0f) -> 回退默认 %.1f",
                     v, IK_LEN_MIN, IK_LEN_MAX, IK_L2_DEFAULT);
        }
    }
    if (nvs_get_i32(handle, "bl", &val) == ESP_OK) {
        float v = val / 100.0f;
        if (v >= IK_LEN_MIN && v <= IK_LEN_MAX) {
            g_motion.body_half_l = v;
        } else {
            g_motion.body_half_l = IK_BODY_HALF_L_DEFAULT;
            ESP_LOGW(TAG, "NVS 里 half_l=%.1f 非法 (需 %.0f~%.0f) -> 回退默认 %.1f",
                     v, IK_LEN_MIN, IK_LEN_MAX, IK_BODY_HALF_L_DEFAULT);
        }
    }
    if (nvs_get_i32(handle, "bw", &val) == ESP_OK) {
        float v = val / 100.0f;
        if (v >= IK_LEN_MIN && v <= IK_LEN_MAX) {
            g_motion.body_half_w = v;
        } else {
            g_motion.body_half_w = IK_BODY_HALF_W_DEFAULT;
            ESP_LOGW(TAG, "NVS 里 half_w=%.1f 非法 (需 %.0f~%.0f) -> 回退默认 %.1f",
                     v, IK_LEN_MIN, IK_LEN_MAX, IK_BODY_HALF_W_DEFAULT);
        }
    }

    // 限位: 四项先各自读出来, 读完再**配对**校验 (单看一个数看不出 min/max 反了 ——
    // 0 和 180 各自都合法)。整套非法就四项一起回退出厂值: 只回退其中一项会配出一个
    // 谁也没设过的组合。
    float hmin = ik_hip_min, hmax = ik_hip_max;
    float kmin = ik_knee_min, kmax = ik_knee_max;
    if (nvs_get_i32(handle, "hmin", &val) == ESP_OK) hmin = val / 100.0f;
    if (nvs_get_i32(handle, "hmax", &val) == ESP_OK) hmax = val / 100.0f;
    if (nvs_get_i32(handle, "kmin", &val) == ESP_OK) kmin = val / 100.0f;
    if (nvs_get_i32(handle, "kmax", &val) == ESP_OK) kmax = val / 100.0f;
    if (geom_limits_valid(hmin, hmax, kmin, kmax)) {
        ik_hip_min  = hmin;  ik_hip_max  = hmax;
        ik_knee_min = kmin;  ik_knee_max = kmax;
    } else {
        ik_hip_min  = IK_HIP_MIN_DEFAULT;  ik_hip_max  = IK_HIP_MAX_DEFAULT;
        ik_knee_min = IK_KNEE_MIN_DEFAULT; ik_knee_max = IK_KNEE_MAX_DEFAULT;
        ESP_LOGW(TAG, "NVS 里限位非法: hip[%.0f~%.0f] knee[%.0f~%.0f] "
                 "(需 min<max 且在 [%d, %d] 内) -> 回退默认 hip[%.0f~%.0f] knee[%.0f~%.0f]",
                 hmin, hmax, kmin, kmax, (int)SERVO_ANGLE_MIN, (int)SERVO_ANGLE_MAX,
                 ik_hip_min, ik_hip_max, ik_knee_min, ik_knee_max);
    }

    if (nvs_get_i32(handle, "co",   &val) == ESP_OK) {
        float v = val / 100.0f;
        if (geom_center_offset_valid(v)) {
            g_motion.center_offset = v;
        } else {
            g_motion.center_offset = CENTER_OFFSET_DEFAULT;
            ESP_LOGW(TAG, "NVS 里 offset=%.1f 非法 (上限 ±%.1f = 大腿 %.1f 的一半) -> 回退默认 %.0f",
                     v, g_motion.ik_L1 * 0.5f, g_motion.ik_L1, CENTER_OFFSET_DEFAULT);
        }
    }
    nvs_close(handle);

    ESP_LOGI(TAG, "Geometry loaded from NVS: "
             "L1=%.1f L2=%.1f BL=%.1f BW=%.1f hip[%.0f~%.0f] knee[%.0f~%.0f] offset=%.0f",
             g_motion.ik_L1, g_motion.ik_L2,
             g_motion.body_half_l, g_motion.body_half_w,
             ik_hip_min, ik_hip_max, ik_knee_min, ik_knee_max,
             g_motion.center_offset);
}

void motion_save_geometry(void)
{
    nvs_handle_t handle;
    if (nvs_open(GEOM_NVS_NS, NVS_READWRITE, &handle) != ESP_OK) {
        ESP_LOGW(TAG, "Failed to open NVS for geometry save");
        return;
    }

    nvs_set_i32(handle, "l1", (int32_t)(g_motion.ik_L1 * 100.0f));
    nvs_set_i32(handle, "l2", (int32_t)(g_motion.ik_L2 * 100.0f));
    nvs_set_i32(handle, "bl", (int32_t)(g_motion.body_half_l * 100.0f));
    nvs_set_i32(handle, "bw", (int32_t)(g_motion.body_half_w * 100.0f));
    nvs_set_i32(handle, "hmin", (int32_t)(ik_hip_min * 100.0f));
    nvs_set_i32(handle, "hmax", (int32_t)(ik_hip_max * 100.0f));
    nvs_set_i32(handle, "kmin", (int32_t)(ik_knee_min * 100.0f));
    nvs_set_i32(handle, "kmax", (int32_t)(ik_knee_max * 100.0f));
    nvs_set_i32(handle, "co",   (int32_t)(g_motion.center_offset * 100.0f));
    nvs_commit(handle);
    nvs_close(handle);

    ESP_LOGI(TAG, "Geometry saved to NVS: "
             "L1=%.1f L2=%.1f BL=%.1f BW=%.1f hip[%.0f~%.0f] knee[%.0f~%.0f] offset=%.0f",
             g_motion.ik_L1, g_motion.ik_L2,
             g_motion.body_half_l, g_motion.body_half_w,
             ik_hip_min, ik_hip_max, ik_knee_min, ik_knee_max,
             g_motion.center_offset);
}

void motion_ensure_geometry_loaded(void)
{
    // motion_load_geometry() 本来只在 motion_task_start() 里调一次, 而运动任务是
    // **首次运动指令**才懒创建 (见 motion_set_mode 的 MODE_MOTION 分支)。开机的
    // poses.stand() (frozen/main.py) 跑在任务创建之前, 那时 g_motion 里还是编译期
    // 默认值 —— Python 若直接读就会拿到 40/45 而不是 NVS 里的真实腿长。
    // 这里补上: 任务没起就自己加载一次; 任务已在跑就跳过 (避免重复读 flash)。
    //
    // ⚠⚠ `loaded` 这个标志是必须的, 不是优化 —— 2026-10-01 实测:
    //   只看 `g_task_handle == NULL` 的话, **任务没起时每次调用都会**
    //   重开 NVS 读一遍 + 打一行 "Geometry loaded from NVS"。
    //   而 heading_follow 的闭环 20Hz 调 get_geometry() ⇒ 狗站着开「航向锁定」
    //   时 **20 行/秒刷屏**, 走 BLE 的 KittenBlock 通道会被灌满。
    //   (motion_load_geometry 自己不幂等, 它没有"已载入"的概念。)
    static bool loaded = false;
    if (g_task_handle == NULL && !loaded) {
        motion_load_geometry();
        loaded = true;
    }
}

void motion_set_omega(float omega)
{
    g_motion.omega_base = omega;
    ESP_LOGI(TAG, "Omega base: %.2f rad/s", omega);
}

// ★ 参数顺序: 俯仰在前, 跟积木文案「俯仰 [PITCH]…滚转 [ROLL]」一致。
// 返回 true=已采纳, false=被拒 (俯仰/横滚一个都不写, 保持原值)。
// ★ 姿态补偿是**逐腿叠加进 z 的** (前腿 +z_pitch / 后腿 -z_pitch, 左 -z_roll / 右 +z_roll),
//   所以它跟 stride/height/lift 一样决定足端够不够得着。以前本函数完全不校验, 于是
//   "先设参数再压姿态"能把腿折过去: pitch=-12° → 前腿 z=83.3 → d=90.3 > 84, 静默够不着。
//   现在跟 set_params 共用 traj_combo_bad(), 拿**新姿态**跑同一条轨迹 ⇒ 顺序耦合消失。
//   (docs/error.md §2.2 #2)
bool motion_set_body_pose(float pitch, float roll)
{
    if (traj_combo_bad(g_motion.stride, g_motion.height, g_motion.lift_height,
                       g_motion.center_offset, g_motion.center_spacing,
                       g_motion.sway_amp, roll, pitch)) {
        ESP_LOGW(TAG, "→ 姿态被拒, 保持原值: pitch=%.1f roll=%.1f "
                 "(当前 stride=%.0f height=%.0f lift=%.0f center=%.1f spacing=%.1f)",
                 g_motion.body_pitch, g_motion.body_roll,
                 g_motion.stride, g_motion.height, g_motion.lift_height,
                 g_motion.center_offset, g_motion.center_spacing);
        return false;
    }
    g_motion.body_pitch = pitch;
    g_motion.body_roll  = roll;
    return true;
}

/* ---- 方向 (独立参数) ----
 * 方向以前**寄居在 stride 的符号里** (stride 正=前 负=后), 于是:
 *   ① GO 下 stride 的幅度被 speed 接管, 只剩符号有意义 ⇒ 方向看起来"挂在速度上";
 *   ② KittenBlock 的方向下拉画在「速度」块上, 生成代码却写的是 set_params 的符号 ⇒ UI 与实现错位;
 *   ③ 同一件事在 GO 和 walk/trot 下走两条路 ⇒ 教用户时没法一句话说清。
 * 拆成独立字段之后, **不管哪种步态**, 都是「步态 + 方向 + 速度 + 高级参数」四个设置。
 *
 * 返回是否原样采纳; 0 与 NaN 被拒 (取不到方向), 保持原值。
 * ★ NaN 安全: NaN 与任何数比较恒假 ⇒ 两个 if 都不进 ⇒ 落到 return false。
 *   这和几何 setter 那套 !(a >= lo && a <= hi) 是同一个道理, 写法不同但等价。
 */
bool motion_set_direction(float dir)
{
    if (dir > 0.0f) { g_motion.direction =  1.0f; return true; }
    if (dir < 0.0f) { g_motion.direction = -1.0f; return true; }
    ESP_LOGW(TAG, "⚠ 方向只能是 ±1 (收到 %.1f), 保持原值 %.0f", dir, g_motion.direction);
    return false;
}

// 返回 true=原样采纳。false 有**两种, 含义不同**:
//   超限 → 已**钳位**到 ±1, 且**写入了钳位后的值** (不是拒绝)
//   NaN  → **拒绝**, 保持原值
// ★ 刻意保持"超限钳位而不是拒写": 输入 5 的人若拿回**上一次**的转弯率(多半是 0),
//   表现就是"我设了转弯它直走", 比钳到 1 更让人困惑。
//
// ⚠ **NaN 检查必须放在两个钳位之后**:
//   NaN 与任何数比较恒假 ⇒ 两个 `if` 都不进 ⇒ 原样写进 g_motion.turn,
//   再经腿循环的 `>= 0.0f`(同样恒假)走进 else 分支 ⇒ **一路穿到 IK 和舵机占空比**。
//   放到钳位**前面**的话 `!(Inf >= -1 && Inf <= 1)` 也为真 ⇒ ±Inf 会被一起拒掉,
//   和"±Inf 照旧被钳"的既定语义打架。放对位置后, **NaN 是唯一能穿过两个钳位的值**。
bool motion_set_turn(float turn)
{
    bool clamped = false;
    if (turn < -1.0f) { turn = -1.0f; clamped = true; }   // ±Inf 在这里被钳掉
    if (turn >  1.0f) { turn =  1.0f; clamped = true; }

    if (!(turn >= -1.0f && turn <= 1.0f)) {   // 走到这儿的只有 NaN
        ESP_LOGW(TAG, "turn 收到 NaN, 拒绝并保持原值 %.2f", g_motion.turn);
        return false;
    }

    g_motion.turn = turn;
    return !clamped;
}

bool motion_set_center(float offset)
{
    if (!geom_center_offset_valid(offset)) {
        ESP_LOGW(TAG, "Center offset rejected: %.1fmm 超出 ±%.1fmm (大腿 %.1f 的一半), "
                 "保持原值 %.1fmm", offset, g_motion.ik_L1 * 0.5f, g_motion.ik_L1,
                 g_motion.center_offset);
        return false;
    }
    // ★ 第二道: **组合**校验。上面那道只量偏移自己, 看不出"跟当前 stride/height 合在一起
    //   还够不够得着" —— 默认 stride=70 height=70 时脚已经伸到 78.3mm, 离可达上限 84 只剩
    //   5.7mm, 重心实际最多只能挪 11.4mm, 而 ±20 那道闸门会放行 15 / 20。够不着时
    //   ik_solve_2dof 只把 d 钳到 84 静默吞掉伸展, 表现为脚在地上蹭、狗歪着走。
    if (traj_combo_bad(g_motion.stride, g_motion.height, g_motion.lift_height,
                       offset, g_motion.center_spacing,
                       g_motion.sway_amp,
                       g_motion.body_roll, g_motion.body_pitch)) {
        ESP_LOGW(TAG, "→ 重心被拒, 保持原值: offset=%.1fmm 单看没超 ±%.1fmm, 但跟当前 "
                 "stride=%.0f height=%.0f lift=%.0f spacing=%.1f 组合后足端够不着 "
                 "(想让重心挪更多, 先减小 stride 或降低站高)",
                 offset, g_motion.ik_L1 * 0.5f,
                 g_motion.stride, g_motion.height, g_motion.lift_height,
                 g_motion.center_spacing);
        return false;
    }
    g_motion.center_offset = offset;
    motion_save_geometry();
    ESP_LOGI(TAG, "Center offset: %.0f mm (saved)", offset);
    return true;
}

/* 中位点间距 (前/后腿对称): 前腿 +s/2、后腿 -s/2; 正=展开, 负=收窄。
 *
 * 背景 (2026-10-11): 低站高/大摆幅时前腿髋在**向后端点**、后腿髋在**向前端点**最先
 * 顶限位 (H=45/S=70 时前髋 179.6°, 余量 0.4°), 同髋向前行程整段闲置 (只用到 103.9°)。
 * 根因是几何: 髋角 = α ± β, 蹲姿下膝部张角 β≈52° 把范围整体抬高。
 * 把这对中位点对称拉开后, 髋的负担转移给膝(伸展侧): 前移时髋向后余量升、膝向前余量降,
 * 配平点 (两者相等) = 最大最小余量最优。
 *   例: S=70/H=45 间距 66 (前+33/后-33) → 余量 0.4° → 23.6°; 允许的最低站高 44.7 → ~30。
 *
 * ⚠ 代价: 向前端点吃可达圆盘预算 ⇒ 最大步长 S_max = 2·(√(84²−H²) − s/2)。
 * ⚠ 最优方向不固定: 长步长 (如 S=106/H=50) 下最优反而**收窄** (≈ -24), 需按 (S,H) 现算。
 *
 * ★ 运行时参数, **不落盘** —— 与 stride/height/lift 同类: 它只对"当时那套参数"有意义,
 *   掉电后参数回默认值, 存量间距反而会配出超可达组合 ⇒ 一起归零最安全。
 * ★ 只在 walk/trot 生效 —— **GO 不叠加** (GO 用固定几何 70/70/30, 组合校验算不到它;
 *   见限速层的 eff_center_spacing); 切换步态时按 1.5mm/帧 平滑淡出/淡入。
 */
bool motion_set_center_spacing(float spacing)
{
    if (!geom_center_spacing_valid(spacing)) {
        ESP_LOGW(TAG, "Center spacing rejected: %.1fmm 超出 ±%.1fmm (2×前后髋半距), "
                 "保持原值 %.1fmm", spacing, 2.0f * g_motion.body_half_l,
                 g_motion.center_spacing);
        return false;
    }
    if (traj_combo_bad(g_motion.stride, g_motion.height, g_motion.lift_height,
                       g_motion.center_offset, spacing,
                       g_motion.sway_amp,
                       g_motion.body_roll, g_motion.body_pitch)) {
        ESP_LOGW(TAG, "→ 中位点间距被拒, 保持原值: spacing=%.1fmm 单看没超 ±%.1fmm, 但跟"
                 "当前 stride=%.0f height=%.0f lift=%.0f center=%.1f 组合后足端够不着 "
                 "(向前端点先到可达上限, 想放更开先减小 stride 或降低站高)",
                 spacing, 2.0f * g_motion.body_half_l,
                 g_motion.stride, g_motion.height, g_motion.lift_height,
                 g_motion.center_offset);
        return false;
    }
    g_motion.center_spacing = spacing;
    ESP_LOGI(TAG, "Center spacing: %.0f mm (runtime only, not saved)", spacing);
    return true;
}

/* ---- 相位联动摆动 (body sway): 幅值 A (mm) ----
 * 背景、公式与幅值-裕量的量化见文件上方 SWAY_* 常量注释; 这里只做两道闸门 + 写入。
 *
 * ★ 运行时参数, **不落盘** —— 掉电回默认 (SWAY_DEFAULT)。理由与 center_spacing 同:
 *   幅值上限随几何变 (低站高 + 大间距时可达到只有 ~2mm), 存着的幅值配上掉电后的
 *   默认参数可能超可达 ⇒ 不存最安全。
 * ★ 只在 walk 族生效 (trot/GO 高速段自动归零), 起步/停车随步长系数收放。
 */
bool motion_set_sway(float amp)
{
    // 写法 !(a >= min && a <= max): 对 NaN 也成立 (NaN 参与比较恒假 ⇒ 取反为真)
    if (!(amp >= 0.0f && amp <= SWAY_MAX_AMP)) {
        ESP_LOGW(TAG, "⚠ 摆动幅值超限: %.1fmm (允许 0~%.0f), 保持原值 %.1fmm",
                 amp, SWAY_MAX_AMP, g_motion.sway_amp);
        return false;
    }
    if (traj_combo_bad(g_motion.stride, g_motion.height, g_motion.lift_height,
                       g_motion.center_offset, g_motion.center_spacing,
                       amp,
                       g_motion.body_roll, g_motion.body_pitch)) {
        ESP_LOGW(TAG, "→ 摆动幅值被拒, 保持原值: sway=%.1fmm 单看没超 %.0fmm, 但跟当前 "
                 "stride=%.0f height=%.0f lift=%.0f center=%.1f spacing=%.1f 组合后"
                 "足端够不着 (摆动吃可达预算, 低站高/大间距时余量很小 —— "
                 "想加摆动先减小 stride 或间距、或抬高站高)",
                 amp, SWAY_MAX_AMP, g_motion.stride, g_motion.height,
                 g_motion.lift_height, g_motion.center_offset,
                 g_motion.center_spacing);
        return false;
    }
    g_motion.sway_amp = amp;
    ESP_LOGI(TAG, "Sway: %.1f mm (runtime only, not saved)", amp);
    // 超过步速平滑上限只警告不拒 —— 裕量确实更大, 取舍由使用者定 (见 SWAY_* 注释②)
    float smooth_max = 0.0995f * fabsf(g_motion.stride);
    if (g_motion.stride > 10.0f && amp > smooth_max) {
        ESP_LOGW(TAG, "⚠ 摆动 %.1fmm > 步速平滑上限 %.1fmm (≈0.0995×stride): "
                 "机身会周期性停顿/倒退 (走一步停一下)", amp, smooth_max);
    }
    return true;
}

bool motion_is_running(void)
{
    return g_motion.enabled && !g_motion.emergency_stop;
}

void motion_set_mode(motion_mode_t mode)
{
    if (mode == MODE_MOTION && servo_safety_latched()) {
        ESP_LOGW(TAG, "safety latched: refuse MODE_MOTION");
        return;
    }
    g_mode = mode;
    switch (mode) {
    case MODE_MOTION:
        if (g_task_handle == NULL) motion_task_start();   // 幂等: 确保 task 已创建
        // 从 POSE/IDLE 切回时同步 g_smooth_angles 到真实舵机角, 防跳变
        for (int i = 0; i < 8; i++) {
            g_smooth_angles[i] = servo_get_angle(i);
        }
        g_motion.enabled = true;
        g_motion.emergency_stop = false;
        break;
    case MODE_POSE:
        g_motion.enabled = false;
        g_motion.emergency_stop = true;   // motion 停止输出, Python 接管
        break;
    case MODE_IDLE:
    default:
        g_motion.enabled = false;
        break;
    }
}

motion_mode_t motion_get_mode(void)
{
    return g_mode;
}

// MicroPython servo 绑定调用: Python 动舵机 → 自动切 POSE
void motion_python_servo_write(void)
{
    if (servo_safety_latched()) return;
    if (g_mode != MODE_POSE) {
        motion_set_mode(MODE_POSE);
    }
}

/* ================================================================
 * 跌倒保护 (MicroPython 可调用)
 * ================================================================ */

void motion_safety_release(void)
{
    g_motion.emergency_stop = true;
    g_motion.enabled = false;
    g_mode = MODE_POSE;
    g_safety_armed = false;
    g_safety_arm_count = 0;
    g_safety_fall_count = 0;
    servo_safety_stop();
    pwm_ext_stop_all();   // 扩展舵机也松开, 不只是 8 路主舵机
    ESP_LOGW(TAG, "Safety release: motion stopped, all servos off");
}

void motion_safety_rearm_at(const float *angles, int count)
{
    if (!angles || count < 8) {
        ESP_LOGE(TAG, "Safety rearm: need 8 angles, got %d", count);
        return;
    }
    servo_safety_rearm_at(angles, count);
    pwm_ext_rearm_all();   // 扩展舵机按释放前的最后脉宽恢复
    g_motion.emergency_stop = true;
    g_motion.enabled = false;
    g_mode = MODE_POSE;
    g_safety_armed = false;
    g_safety_arm_count = 0;
    g_safety_fall_count = 0;
    ESP_LOGI(TAG, "Safety rearm done (main + extension); motion stays in POSE");
}

bool motion_safety_latched(void)
{
    return servo_safety_latched();
}

void motion_safety_set_enabled(bool enabled)
{
    g_safety_monitor_enabled = enabled;
    g_safety_armed = false;
    g_safety_arm_count = 0;
    g_safety_fall_count = 0;
    ESP_LOGI(TAG, "Safety monitor %s", enabled ? "ON" : "OFF");
}

void motion_safety_set_pose_active(bool active)
{
    if (g_safety_pose_active == active) return;
    g_safety_pose_active = active;
    g_safety_arm_count = 0;
    g_safety_fall_count = 0;
    if (!active) g_safety_armed = false;
    ESP_LOGI(TAG, "Safety pose scope %s", active ? "ON (stand)" : "OFF (static pose)");
}

void motion_safety_config(float angle, float rate, int frames)
{
    if (angle < 5.0f) angle = 5.0f;
    if (angle > 80.0f) angle = 80.0f;
    if (rate < 50.0f) rate = 50.0f;
    if (rate > 1000.0f) rate = 1000.0f;
    if (frames < 1) frames = 1;
    if (frames > 50) frames = 50;
    g_safety_angle_deg = angle;
    g_safety_rate_dps = rate;
    g_safety_frames = frames;
    ESP_LOGI(TAG, "Safety config: angle=%.1f rate=%.0f frames=%d",
             angle, rate, frames);
}
