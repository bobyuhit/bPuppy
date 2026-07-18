/*
 * bDog 逆运动学 (IK) — 2-DOF 单腿解算
 *
 * 几何模型:
 *   髋关节 O ─── L1 (大腿) ─── 膝关节 K ─── L2 (小腿) ─── 足端 F
 *
 *   已知: F 相对于 O 的坐标 (foot_x, foot_z), L1=60, L2=68
 *   求解: ∠hip (髋角), ∠knee (膝角)
 *
 * IK 公式:
 *   d² = x² + z²
 *   knee_angle  = acos((L1² + L2² - d²) / (2·L1·L2))    // 膝弯曲程度
 *   hip_angle   = atan2(z, x) + acos((L1² + d² - L2²) / (2·L1·d))
 */

#include "ik.h"
#include <math.h>
#include "esp_log.h"

static const char *TAG = "ik";

// 运行时舵机极限 (可通过 motion_set_joint_limits() 修改 + NVS 持久化)
float ik_hip_min  = IK_HIP_MIN_DEFAULT;
float ik_hip_max  = IK_HIP_MAX_DEFAULT;
float ik_knee_min = IK_KNEE_MIN_DEFAULT;
float ik_knee_max = IK_KNEE_MAX_DEFAULT;

#define DEG(x)  ((x) * 180.0f / (float)M_PI)

ik_result_t ik_solve_2dof(float foot_x, float foot_z,
                          float L1, float L2, int side, int leg_pair)
{
    ik_result_t r;
    float L1L2_max = L1 + L2;
    float L2L1_min = fabsf(L2 - L1);

    float d_sq = foot_x * foot_x + foot_z * foot_z;
    float d = sqrtf(d_sq);

    // 钳位到机械可达范围
    if (d > L1L2_max - 1.0f)  d = L1L2_max - 1.0f;
    if (d < L2L1_min + 1.0f)  d = L2L1_min + 1.0f;

    // 膝角 (大腿与小腿之间的夹角, 0°=并拢折叠, 180°=完全伸直)
    float cos_knee = (L1 * L1 + L2 * L2 - d * d) / (2.0f * L1 * L2);
    if (cos_knee > 1.0f)  cos_knee = 1.0f;
    if (cos_knee < -1.0f) cos_knee = -1.0f;
    float knee_angle = acosf(cos_knee);  // [0, π], 0=折叠  π=伸直

    // 髋角 (大腿与水平面夹角, 0=水平向前, 90°=指向下)
    float alpha = atan2f(foot_z, foot_x);               // 足端方向
    float cos_beta = (L1 * L1 + d * d - L2 * L2) / (2.0f * L1 * d);
    if (cos_beta > 1.0f)  cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    float beta = acosf(cos_beta);
    // 膝后弯: hip = alpha + beta   膝前弯: hip = alpha - beta
    float hip_angle;
#if IK_KNEE_REAR_FORWARD
    hip_angle = (leg_pair == IK_LEG_REAR) ? (alpha - beta) : (alpha + beta);
#else
    hip_angle = alpha + beta;
#endif

    // 弧度 → 舵机度
    r.hip_deg = DEG(hip_angle);

    // 膝舵机映射
    float knee_deg = DEG(knee_angle);  // 0°=直, 180°=最大折

    // 后腿前弯: 膝角映射左右互换 (0=四腿后弯, 1=后腿前弯兼真狗式)
    int knee_mirror = (side == IK_SIDE_RIGHT);
#if IK_KNEE_REAR_FORWARD
    if (leg_pair == IK_LEG_REAR) knee_mirror = !knee_mirror;
#endif

    if (side == IK_SIDE_RIGHT) {
        r.hip_deg = 180.0f - r.hip_deg;
    }
    r.knee_deg = knee_mirror ? (180.0f - knee_deg) : knee_deg;

    // 钳位 (运行时变量, NVS 可覆盖)
    if (r.hip_deg < ik_hip_min)  r.hip_deg = ik_hip_min;
    if (r.hip_deg > ik_hip_max)  r.hip_deg = ik_hip_max;
    if (r.knee_deg < ik_knee_min) r.knee_deg = ik_knee_min;
    if (r.knee_deg > ik_knee_max) r.knee_deg = ik_knee_max;

    return r;
}
