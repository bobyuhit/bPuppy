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

#define DEG(x)  ((x) * 180.0f / (float)M_PI)

ik_result_t ik_solve_2dof(float foot_x, float foot_z,
                          float L1, float L2, int side)
{
    ik_result_t r;
    float L1L2_max = L1 + L2;
    float L2L1_min = fabsf(L2 - L1);

    float d_sq = foot_x * foot_x + foot_z * foot_z;
    float d = sqrtf(d_sq);

    // 钳位到机械可达范围
    if (d > L1L2_max - 1.0f)  d = L1L2_max - 1.0f;
    if (d < L2L1_min + 1.0f)  d = L2L1_min + 1.0f;

    // 膝角 (大腿与小腿之间的角度, 0=直)
    float cos_knee = (L1 * L1 + L2 * L2 - d * d) / (2.0f * L1 * L2);
    if (cos_knee > 1.0f)  cos_knee = 1.0f;
    if (cos_knee < -1.0f) cos_knee = -1.0f;
    float knee_angle = acosf(cos_knee);  // 弧度, 0 = 完全伸直

    // 髋角 (大腿与水平面夹角, 0=水平向前)
    float alpha = atan2f(foot_z, foot_x);  // 足端方向
    float cos_beta = (L1 * L1 + d * d - L2 * L2) / (2.0f * L1 * d);
    if (cos_beta > 1.0f)  cos_beta = 1.0f;
    if (cos_beta < -1.0f) cos_beta = -1.0f;
    float beta = acosf(cos_beta);
    float hip_angle = alpha + beta;  // 弧度

    // 弧度 → 舵机度
    r.hip_deg = DEG(hip_angle);

    // 膝舵机映射
    float knee_deg = DEG(knee_angle);  // 0°=直, 180°=最大折

    if (side == IK_SIDE_LEFT) {
        // 左腿: HIP(0°前→90°下→180°后)
        //       KNEE: servo 0°=折叠 180°=伸直 → servo = knee_deg
        r.knee_deg = knee_deg;
    } else {
        // 右腿: HIP(0°后→90°下→180°前) — 反转
        //       KNEE: servo 0°=伸直 180°=折叠 → servo = 180 - knee_deg
        r.hip_deg  = 180.0f - r.hip_deg;
        r.knee_deg = 180.0f - knee_deg;
    }

    // 钳位
    if (r.hip_deg < 0.0f)   r.hip_deg = 0.0f;
    if (r.hip_deg > 180.0f) r.hip_deg = 180.0f;
    if (r.knee_deg < IK_KNEE_MIN) r.knee_deg = IK_KNEE_MIN;
    if (r.knee_deg > IK_KNEE_MAX) r.knee_deg = IK_KNEE_MAX;

    return r;
}
