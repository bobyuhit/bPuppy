/*
 * bDog 逆运动学 (IK) — 2-DOF 单腿解算
 *
 * 输入: 足端坐标 (x, z) 相对髋关节，x=前(+), z=下(+)
 * 输出: 髋关节舵机角度, 膝关节舵机角度
 *
 * 腿参数:
 *   L1 (大腿) = 60mm, L2 (小腿) = 68mm
 *
 * 舵机映射 (标定结果):
 *   髋部: 0°前 → 90°下 → 180°后 (四腿相同)
 *   膝部 LEFT  (LF/LH): 45°直 → 135°折
 *   膝部 RIGHT (RF/RH): 45°折 → 135°直
 */

#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define IK_L1_DEFAULT  40.0f   // 大腿长度 (mm)
#define IK_L2_DEFAULT  45.0f   // 小腿长度 (mm)

#define IK_BODY_HALF_L_DEFAULT  62.5f   // 前后髋半距默认值 (mm)
#define IK_BODY_HALF_W_DEFAULT  59.0f   // 左右髋半宽默认值 (mm)

#define IK_HIP_MIN_DEFAULT   0.0f    // 髋舵机下限默认值
#define IK_HIP_MAX_DEFAULT   180.0f  // 髋舵机上限默认值
#define IK_KNEE_MIN_DEFAULT  10.0f   // 膝舵机下限默认值
#define IK_KNEE_MAX_DEFAULT  170.0f  // 膝舵机上限默认值

#define CENTER_OFFSET_DEFAULT  0.0f  // 脚中位偏移默认值 (mm)

// 运行时参数默认值 (不进 NVS，掉电丢失)
#define SPEED_DEFAULT   2.5f
#define STRIDE_DEFAULT  0.0f
#define HEIGHT_DEFAULT  70.0f
#define LIFT_DEFAULT    30.0f
#define OMEGA_DEFAULT   2.0f

// 运行时极限变量 (ik.c 中定义, 可通过 motion_set_joint_limits() 修改 + NVS 持久化)
extern float ik_hip_min;
extern float ik_hip_max;
extern float ik_knee_min;
extern float ik_knee_max;

#define IK_SIDE_LEFT  0
#define IK_SIDE_RIGHT 1

#define IK_KNEE_REAR_FORWARD  1   // 1=后腿膝前弯(真狗式) 0=四腿后弯
// ⚠ 设为1时需同步修改后腿膝关节舵机的小腿安装方向 (180°翻转)

#define IK_LEG_FRONT  0
#define IK_LEG_REAR   1

// IK 解算结果
typedef struct {
    float hip_deg;   // 髋舵机角度 (0-180)
    float knee_deg;  // 膝舵机角度 (45-135)
} ik_result_t;

// 解算单腿 IK（L1, L2 可传入校准值）
// foot_x: 足端前后坐标 (正=前), foot_z: 足端上下坐标 (正=下)
// L1: 大腿长度, L2: 小腿长度
// side: IK_SIDE_LEFT 或 IK_SIDE_RIGHT
ik_result_t ik_solve_2dof(float foot_x, float foot_z,
                          float L1, float L2, int side, int leg_pair);

#ifdef __cplusplus
}
#endif
