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
    GAIT_GO,            // 自适应 (speed<5→walk, speed>10→trot, 之间插值)
    // 以下未暴露到 MicroPython
    GAIT_JUMP,          // 跳跃：蹲→前腿弹→后腿弹→蹲
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
    float       stride;         // 步幅 (mm)
    float       height;         // 站立高度 (mm)
    float       lift_height;    // 抬腿高度 (mm)
    float       body_roll;      // 身体目标横滚角 (deg)
    float       body_pitch;     // 身体目标俯仰角 (deg)
    float       body_yaw;       // 身体目标偏航角 (deg)
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
    float       stand_up_elapsed; // 跳跃计时 (复用, 秒)

    /* ---- 姿态过渡 (预备位切换) ---- */
    // 0=无过渡  1=起步过渡 (站立→预备位→行走)
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

// 设置运动参数
void motion_set_params(float speed, float stride, float height);

// 校验参数（不写入），返回 0=OK, 1=超限
int motion_check_params(float stride, float height);

// 分腿校准
void motion_cal_LF_HIP(float deg);
void motion_cal_LH_HIP(float deg);
void motion_cal_RF_HIP(float deg);
void motion_cal_RH_HIP(float deg);
void motion_cal_LF_KNEE(float deg);
void motion_cal_LH_KNEE(float deg);
void motion_cal_RF_KNEE(float deg);
void motion_cal_RH_KNEE(float deg);

// IK 校准：调整大腿/小腿长度
void motion_cal_ik(float L1, float L2);

// 身体尺寸校准：调整前后/左右髋距
void motion_set_body_dims(float half_l, float half_w);

// 舵机极限校准：调整髋/膝舵机角度限位
void motion_set_joint_limits(float hip_min, float hip_max,
                              float knee_min, float knee_max);

// 几何参数持久化：从 NVS 加载 / 保存到 NVS
void motion_load_geometry(void);
void motion_save_geometry(void);

// 设置基准角频率 (rad/s)
void motion_set_omega(float omega);

// 设置抬腿高度 (mm)
void motion_set_lift(float lift);

// 设置身体姿态
void motion_set_body_pose(float roll, float pitch, float yaw);

// 设置转弯系数 (-1=左, +1=右, 0=直)
void motion_set_turn(float turn);

// 设置脚中位偏移 (正=前移, 负=后移)
void motion_set_center(float offset);

// 检查运动任务是否正在运行（enabled 且未急停）
bool motion_is_running(void);

// 设置运行模式 (自动转换入口)
void motion_set_mode(motion_mode_t mode);
motion_mode_t motion_get_mode(void);

// MicroPython servo 绑定调用: Python 动舵机 → 自动切 POSE
void motion_python_servo_write(void);

// 跳跃：蹲→前腿弹→后腿弹→回蹲
void motion_jump(void);

#ifdef __cplusplus
}
#endif
