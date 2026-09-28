/*
 * bDog 运动控制任务 — 头文件
 *
 * FreeRTOS 实时任务，控制四足机器狗的步态运动。
 * 与 MicroPython 并行运行，通过全局状态结构体与 Python 层通信。
 *
 * 步态统一框架:
 *   左右两侧各有两条腿，同侧间隙 + 单腿摆动占比构成基本节拍单元。
 *   walk 和 trot 的区别仅在于 duty/gap 参数不同。
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 步态类型 ---- */
typedef enum {
    GAIT_STOP = 0,      // 停止站好 (运动模式, 高度随参数)
    GAIT_WALK,          // 猫步 (speed>0前进, speed<0后退)
    GAIT_TROT,          // 小跑 (speed>0前进, speed<0后退)
    GAIT_GO,            // 自适应 (speed≤4.0→walk, speed≥6.0→trot, 之间插值; 见 motion_task_main 的 GO 分支)
                        // ★ GO 自己定 步长/站高/抬脚 —— 用户设的 stride/lift/height 只有 stride 的符号当方向用
    GAIT_REPOS,         // ★ 回正步 (停步收尾): 身体不动, 对角两两抬脚把四腿挪回 x=0 —— 见 motion_enter_repos()
    GAIT_COUNT
} gait_type_t;

/* ---- 运行模式状态机 (自动切换) ---- */
// IDLE  = 上电未初始化;  POSE = Python 接管舵机;  MOTION = motion task 控制
typedef enum {
    MODE_IDLE = 0,
    MODE_POSE,
    MODE_MOTION,
} motion_mode_t;

/* ---- 运动状态 ---- */
typedef struct {
    gait_type_t gait;           // 当前步态
    float       speed;          // 速度 (当前, 平滑后)
    float       target_speed;   // 速度 (目标)
    float       stride;         // 步长**幅度** (mm, ≥0; 0 = 原地踏步) —— 方向见 direction
    float       direction;      // +1 = 前, -1 = 后 —— 独立参数, 上电默认 +1
    float       height;         // 站立高度 (mm)
    float       lift_height;    // 抬腿高度 (mm)
    float       body_roll;      // 身体目标横滚角 (deg)
    float       body_pitch;     // 身体目标俯仰角 (deg)
    float       ik_L1;          // IK 大腿长度校准 (mm)
    float       ik_L2;          // IK 小腿长度校准 (mm)
    float       omega_base;     // 基准角频率 (rad/s), 默认 2.0
    float       gait_duty;      // 摆动相占比 (walk:0.20, trot:0.40)
    float       gait_gap;       // 同侧间隙 (walk:0.04, trot:0.10)
    float       turn_rate;      // (旧) 转弯速率, 将被 turn 替代
    float       turn;           // 转弯系数 -1(左) ~ +1(右)
    float       center_offset;  // 脚中位偏移 (正=前移, 负=后移)
    float       body_half_l;    // 前后髋半距 (mm), 默认 62.5
    float       body_half_w;    // 左右髋半宽 (mm), 默认 59.0
    bool        emergency_stop; // 急停标志
    bool        enabled;        // 运动使能

    /* ---- 姿态过渡 (预备位切换) ---- */
    // 0=无过渡  1=已废弃 (起步过渡改由 motion_task.cpp 的 g_stand_up 阶段 A 承担,
    //            原因见那里的注释: 旧实现从写死的 (0, eff_height) 起步)
    //            2=停步过渡 (行走→预备位→站立)
    // 预备位 = gait 全踩地相位中点, 从站立进入或退回时在此缓动
    uint8_t     pose_trans;       // 姿态过渡状态
    float       pose_timer;       // 姿态过渡计时 (秒)
} motion_state_t;

/* ---- API ---- */

// 启动运动控制 FreeRTOS 任务
void motion_task_start(void);

// 获取当前运动状态（Python 层可读取）
const motion_state_t *motion_get_state(void);

// 设置步态（自动填充 duty/gap/dir/turn）
void motion_set_gait(gait_type_t gait);

// 设置运动参数: 步长(mm, 正=前 零=原地 负=后), 抬脚高度(mm), 站立高度(mm)
// 三者是同一条足端轨迹的三个维度, **一起校验、一起写入** —— 天然自洽, 无顺序耦合。
// 返回是否写入成功 (false=超限被拒, 三个字段全部保持原值)
bool motion_set_params(float stride, float lift, float height);

// 设置目标速度/步频 (0~10)。与轨迹无关, 任意时刻可单独设, 不影响 stride/lift/height。
// 返回是否写入成功 (false=超出 0~10 或 NaN 被拒, 保持原值)
bool motion_set_speed(float speed);

// IK 校准：调整大腿/小腿长度
// 返回是否写入成功 (false=非法被拒, 保持原值)
bool motion_cal_ik(float L1, float L2);

// 身体尺寸校准：调整前后/左右髋距
// 返回是否写入成功 (false=非法被拒, 保持原值)
bool motion_set_body_dims(float half_l, float half_w);

// 舵机极限校准：调整髋/膝舵机角度限位
// 返回是否写入成功 (false=min/max 反了或非法, 保持原值)
bool motion_set_joint_limits(float hip_min, float hip_max,
                              float knee_min, float knee_max);

// 几何参数持久化：从 NVS 加载 / 保存到 NVS
void motion_load_geometry(void);
void motion_save_geometry(void);

// 确保几何参数已从 NVS 载入 —— 运动任务未创建时代为加载 (Python 侧读取几何前调用)
void motion_ensure_geometry_loaded(void);

// 设置基准角频率 (rad/s)
void motion_set_omega(float omega);

// 设置身体姿态 (deg): 俯仰(前低后高为负) / 横滚
// ★ 参数顺序 = 俯仰在前 —— 跟积木文案「俯仰 [PITCH]…滚转 [ROLL]」一致。
//   旧签名是 (roll, pitch, yaw), 且 yaw 是死字段(只写不读), 已一并删掉。
// 返回是否写入成功 (false=跟当前 stride/height/lift/重心组合后足端够不着或入地, 被拒,
// 俯仰和横滚都保持原值)。
// ★ 姿态补偿是逐腿叠加进 z 的, 所以它能改变可达性 —— 必须跟 set_params 一样做组合校验
//   (pitch=-12° → 前腿 d=90.3 > 84)。2026-09-28 起补上, 顺序耦合消失 (docs/error.md §2.2 #2)。
bool motion_set_body_pose(float pitch, float roll);

// 设置转弯系数 (-1=左, +1=右, 0=直)
// 返回是否原样采纳 (false=超出 ±1 已**钳位**, 注意此时仍写入了钳位后的值, 不是拒绝)
bool motion_set_turn(float turn);

// 设置运动方向 (+1=前, -1=后) —— 独立参数, 上电默认 +1。
// 与步长/速度/步态**完全解耦**: 改方向不再连带改步长, 反之亦然。
// 返回是否原样采纳 (false=0 或 NaN 被拒, 保持原值)
bool motion_set_direction(float dir);

// 设置脚中位偏移 (正=前移, 负=后移)
// 返回是否写入成功 (false=被拒, 保持原值且不写 NVS)。被拒有两种原因:
//   (1) 量级超 ±大腿长/2 = ±20mm;
//   (2) 量级没超, 但跟当前 stride/height/lift 组合后足端够不着
//       (默认 stride=70 height=70 时实际上限只有 11.4mm, 见 docs/error.md §2.2 #2)
bool motion_set_center(float offset);

// 检查运动任务是否正在运行（enabled 且未急停）
bool motion_is_running(void);

// 设置运行模式 (自动转换入口)
void motion_set_mode(motion_mode_t mode);
motion_mode_t motion_get_mode(void);

// MicroPython servo 绑定调用: Python 动舵机 → 自动切 POSE
void motion_python_servo_write(void);

#ifdef __cplusplus
}
#endif
