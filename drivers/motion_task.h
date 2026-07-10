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
    GAIT_STAND = 0,     // 站立
    GAIT_WALK,          // 猫步 (speed>0前进, speed<0后退)
    GAIT_TROT,          // 小跑 (speed>0前进, speed<0后退)
    GAIT_GO,            // 自适应 (speed<5→walk, speed>10→trot, 之间插值)
    GAIT_CRAWL,         // 爬行
    GAIT_BOUND,         // 跳跃
    GAIT_WALKTURN_L,    // 猫步左转
    GAIT_WALKTURN_R,    // 猫步右转
    GAIT_TROTTURN_L,    // 小跑左转
    GAIT_TROTTURN_R,    // 小跑右转
    GAIT_SIT,           // 蹲下
    GAIT_STAND_UP,      // 蹲→站过渡（缓动插值, 自动切换 GAIT_STAND）
    GAIT_COUNT
} gait_type_t;

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
    bool        emergency_stop; // 急停标志
    bool        enabled;        // 运动使能
    float       stand_up_elapsed; // 站立过渡已用时间 (秒)

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

// 设置基准角频率 (rad/s)
void motion_set_omega(float omega);

// 设置身体姿态
void motion_set_body_pose(float roll, float pitch, float yaw);

// 设置转弯系数 (-1=左, +1=右, 0=直)
void motion_set_turn(float turn);

// 急停
void motion_emergency_stop(void);

// 恢复
void motion_resume(void);

// 站立过渡：蹲姿 → 缓动 → 站立
void motion_stand_up(void);

#ifdef __cplusplus
}
#endif
