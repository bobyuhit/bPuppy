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
#define IK_L2_DEFAULT  40.0f   // 小腿长度 (mm)

#define IK_KNEE_MIN   10.0f   // 膝舵机最小角度
#define IK_KNEE_MAX   170.0f  // 膝舵机最大角度

#define IK_SIDE_LEFT  0
#define IK_SIDE_RIGHT 1

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
                          float L1, float L2, int side);

#ifdef __cplusplus
}
#endif
