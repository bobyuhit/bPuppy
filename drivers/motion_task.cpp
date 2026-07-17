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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include <math.h>
#include <string.h>

static const char *TAG = "motion";

/* ---- 任务参数 ---- */
#define MOTION_STACK       4096
#define MOTION_PRIORITY    6
#define MOTION_CORE        0
#define MOTION_LOOP_MS     20      // 50Hz (舵机 PWM 同频)

#define TWO_PI             (2.0f * (float)M_PI)
#define TURN_STRIDE_FACTOR 0.15f
#define SPEED_FOLLOW_STEP 3.0f  // 每半周速度跟随最大变化量

/* ---- 身体几何 ---- */
#define BODY_HALF_L        60.0f   // 前后髋距 120mm / 2
#define BODY_HALF_W        59.0f   // 左右髋宽 118mm / 2

/* ---- 腿配置: LF, LH, RF, RH ---- */
static const int leg_hip_ch[]  = {0, 2, 4, 6};
static const int leg_knee_ch[] = {1, 3, 5, 7};
static const int leg_side[]    = {IK_SIDE_LEFT, IK_SIDE_LEFT,
                                   IK_SIDE_RIGHT, IK_SIDE_RIGHT};

/* ---- 全局状态 ---- */
__attribute__((weak)) motion_state_t g_motion = {
    .gait           = GAIT_STAND,
    .speed          = 2.5f,
    .target_speed   = 2.5f,
    .stride         = 0.0f,
    .height         = 60.0f,
    .lift_height    = 30.0f,
    .body_roll      = 0.0f,
    .body_pitch     = 0.0f,
    .body_yaw       = 0.0f,
    .ik_L1          = IK_L1_DEFAULT,
    .ik_L2          = IK_L2_DEFAULT,
    .omega_base     = 2.0f,
    .gait_duty      = 0.20f,
    .gait_gap       = 0.04f,
    .turn_rate      = 0.0f,
    .turn           = 0.0f,
    .center_offset  = 0.0f,
    .emergency_stop = false,
    .enabled        = false,
    .stand_up_elapsed = 0.0f,
    .pose_trans     = 0,
    .pose_timer     = 0.0f,
};

static TaskHandle_t g_task_handle = NULL;
__attribute__((weak)) float g_phase = 0.0f;
static float g_prev_fx[4] = {0, 0, 0, 0};
static float g_prev_fz[4] = {100, 100, 100, 100};
static bool g_was_moving = false;

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

/* ---- 步态参数表 ---- */
static void motion_apply_gait_params(gait_type_t gait)
{
    switch (gait) {
    case GAIT_WALK:        g_motion.gait_duty = 0.20f; g_motion.gait_gap = 0.04f; g_motion.turn_rate = 0.0f;  break;
    case GAIT_TROT:        g_motion.gait_duty = 0.40f; g_motion.gait_gap = 0.10f; g_motion.turn_rate = 0.0f;  break;
    case GAIT_GO:          g_motion.gait_duty = 0.20f; g_motion.gait_gap = 0.04f; g_motion.turn_rate = 0.0f;  break;  // 运行时根据 speed 动态调整
    case GAIT_CRAWL:       g_motion.gait_duty = 0.30f; g_motion.gait_gap = 0.03f; g_motion.turn_rate = 0.0f;  break;
    case GAIT_BOUND:       g_motion.gait_duty = 0.35f; g_motion.gait_gap = 0.15f; g_motion.turn_rate = 0.0f;  break;
    default: break; // stand/sit/stand_up 不调参数
    }
    ESP_LOGI(TAG, "Gait: %d (duty=%.2f gap=%.2f turn=%.1f)",
             gait, g_motion.gait_duty, g_motion.gait_gap,
             g_motion.turn_rate);
}

/* ---- 足端轨迹生成器 ---- */
static void foot_trajectory(float phase_norm, float stride, float height,
                            float lift, float swing_ratio, float direction,
                            float *out_x, float *out_z)
{
    if (phase_norm < swing_ratio) {
        float t = phase_norm / swing_ratio;
        float ease = t * t * (3.0f - 2.0f * t);
        *out_z = height - lift * sinf(ease * (float)M_PI);
        *out_x = -stride * 0.5f + stride * ease;
    } else {
        float t = (phase_norm - swing_ratio) / (1.0f - swing_ratio);
        *out_z = height;
        *out_x = stride * 0.5f - stride * t;
    }

    if (direction < 0) {
        *out_x = -*out_x;
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
        if (!g_motion.enabled || g_motion.emergency_stop) {
            vTaskDelayUntil(&last_wake, period);
            continue;
        }

        bool is_stand    = (g_motion.gait == GAIT_STAND);
        bool is_sit      = (g_motion.gait == GAIT_SIT);
        bool is_stand_up = (g_motion.gait == GAIT_STAND_UP);
        bool is_jump     = (g_motion.gait == GAIT_JUMP);

        if (is_stand_up || is_jump) {
            g_motion.stand_up_elapsed += dt;
        }

        /*
         * ============================================================
         * 姿态过渡: 起步过渡 / 停步过渡
         * ============================================================
         *
         * 站立 ←→ 预备位 ←→ 行走
         *
         * 预备位 = 全踩地相位中点 (all_stance_mid), 此时四腿着地不抬,
         *         从站立移动到预备位 (或反向) 只平移足端, 不会歪倒.
         *
         * 起步过渡 (pose_trans=1):
         *   站立姿态 → (相位设到预备位, 足端 smoothstep 0.3s 过渡) → 正常行走
         *
         * 停步过渡 (pose_trans=2):
         *   正常行走 → (足端 smoothstep 0.3s 退回站姿) → 站立姿态
         *
         * 过渡期 0.3s, 用 smoothstep 缓动: ease = t²(3-2t)
         */
        #define TRANS_TIME 0.3f
        bool now_static = (is_stand || is_sit);
        if (now_static && g_was_moving) {
            // 行走 → 静止: 启动停步过渡
            g_motion.pose_trans = 2;
            g_motion.pose_timer = 0.0f;
            g_motion.speed = 0.0f;
        } else if (!now_static && !is_stand_up && !g_was_moving) {
            // 静止 → 行走: 启动起步过渡
            g_motion.pose_trans = 1;
            g_motion.pose_timer = 0.0f;
        }
        // 姿态过渡计时: 到期清零
        if (g_motion.pose_trans == 1 || g_motion.pose_trans == 2) {
            g_motion.pose_timer += dt;
            if (g_motion.pose_timer >= TRANS_TIME)
                g_motion.pose_trans = 0;
        }
        g_was_moving = !now_static;

        // 方向与速度: 正=前, 负=后
        float eff_speed = fabsf(g_motion.speed);
        int   eff_dir   = (g_motion.speed < 0.0f) ? -1 : 1;

        /*
         * ============================================================
         * 速度跟随: 半周期更新
         * ============================================================
         *
         * 实际速度向目标速度跟随, 每半个步态周期 (g_phase 过 0 或 PI)
         * 调整一次, 每次最多变化 ±SPEED_FOLLOW_STEP. 避免每帧变速造成的步态冲击.
         *
         * 相位冻结 (speed=0) 时也触发, 防止锁死.
         */
        // 相位累加 + 半周期速度更新
        if (!is_stand && !is_sit && !is_stand_up) {
            float omega = g_motion.omega_base * eff_speed;
            float prev_phase = g_phase;
            g_phase += omega * dt;
            if (g_phase > TWO_PI) g_phase -= TWO_PI;

            bool cross_half = (int)(prev_phase / (float)M_PI) !=
                              (int)(g_phase     / (float)M_PI);
            if (cross_half || prev_phase == g_phase) {
                float ds = g_motion.target_speed - g_motion.speed;
                if (fabsf(ds) < 0.15f) {
                    g_motion.speed = g_motion.target_speed; // 接近就到位
                } else {
                    float step = SPEED_FOLLOW_STEP;
                    if (ds >  step) ds =  step;
                    if (ds < -step) ds = -step;
                    g_motion.speed += ds;
                }
            }
        }

        // GAIT_GO: 速度自适应 duty/gap，stride/height 沿用 g_motion 默认
        float eff_duty   = g_motion.gait_duty;
        float eff_gap    = g_motion.gait_gap;
        float eff_stride = g_motion.stride;
        float eff_height = g_motion.height;
        // GO: speed≤4=walk  speed≥6=trot  → duty/gap/stride/pitch 插值
        float eff_pitch = g_motion.body_pitch;
        if (g_motion.gait == GAIT_GO) {
            float s = eff_speed;
            if (s <= 4.0f) {
                eff_duty=0.20f; eff_gap=0.04f;
                eff_stride=70.0f; eff_height=70.0f; eff_pitch=-5.0f;
            } else if (s >= 6.0f) {
                eff_duty=0.40f; eff_gap=0.10f;
                eff_stride=50.0f; eff_height=70.0f; eff_pitch=-3.0f;
            } else {
                float t = (s - 4.0f) / 2.0f;
                eff_duty   = 0.20f + t * 0.20f;
                eff_gap    = 0.04f + t * 0.06f;
                eff_stride = 70.0f - t * 20.0f;
                eff_height = 70.0f;
                eff_pitch  = -5.0f + t * 2.0f;
            }
            g_motion.body_pitch = eff_pitch;
        }

        // 计算步态偏移
        float offsets[4];
        compute_offsets(eff_duty, eff_gap, eff_dir, offsets);

        // 过渡开始时设一次相位到全踩地中点, 然后正常累计
        if (g_motion.pose_trans == 1 && g_motion.pose_timer < 0.02f) {
            float mid = all_stance_mid(eff_duty, eff_gap);
            g_phase = mid * TWO_PI;
        }

        servo_group_begin();

        for (int leg = 0; leg < 4; leg++) {
            float foot_x, foot_z;

            if (g_motion.gait == GAIT_JUMP) {
                // 跳跃时序: 蹲(1s) → 前腿弹(0.1s) → 后腿弹(0.3s) → 回蹲→站
                static const float crouch_z = 15.0f, jump_z = 78.0f;
                float jt = g_motion.stand_up_elapsed;  // 复用计时器
                float z;
                bool front = (leg == 0 || leg == 2);

                if (jt < 1.0f) {
                    z = crouch_z;  // 蹲
                } else if (jt < 1.1f) {
                    z = front ? jump_z : crouch_z;  // 前腿弹
                } else if (jt < 1.4f) {
                    z = jump_z;  // 四腿全弹
                } else {
                    z = crouch_z;  // 回蹲
                }
                ik_result_t ik = ik_solve_2dof(0, z,
                                    g_motion.ik_L1, g_motion.ik_L2,
                                    leg_side[leg]);
                servo_group_add(leg_hip_ch[leg],  ik.hip_deg);
                servo_group_add(leg_knee_ch[leg], ik.knee_deg);
                continue;
            } else if (is_sit) {
                servo_group_add(leg_hip_ch[leg],
                    (leg_side[leg] == IK_SIDE_LEFT) ? 135.0f : 45.0f);
                servo_group_add(leg_knee_ch[leg],
                    (leg_side[leg] == IK_SIDE_LEFT) ? 45.0f : 135.0f);
                continue;
            } else if (is_stand_up) {
                float t = g_motion.stand_up_elapsed;
                float hold = 0.3f;
                float ramp = 3.0f;

                float sit_hip  = (leg_side[leg] == IK_SIDE_LEFT) ? 135.0f : 45.0f;
                float sit_knee = (leg_side[leg] == IK_SIDE_LEFT) ? 45.0f : 135.0f;

                ik_result_t ik = ik_solve_2dof(0, g_motion.height,
                                    g_motion.ik_L1, g_motion.ik_L2,
                                    leg_side[leg]);

                if (t < hold) {
                    servo_group_add(leg_hip_ch[leg],  sit_hip);
                    servo_group_add(leg_knee_ch[leg], sit_knee);
                } else {
                    float raw = (t - hold) / ramp;
                    if (raw > 1.0f) raw = 1.0f;
                    float ease = raw * raw * (3.0f - 2.0f * raw);
                    servo_group_add(leg_hip_ch[leg],
                        sit_hip  + (ik.hip_deg  - sit_hip)  * ease);
                    servo_group_add(leg_knee_ch[leg],
                        sit_knee + (ik.knee_deg - sit_knee) * ease);
                }
                continue;
            } else if (is_stand) {
                if (g_motion.pose_trans == 2) {
                    // 停止过渡: 从上一帧位置缓动到站姿
                    float t = g_motion.pose_timer / TRANS_TIME;
                    float ease = t * t * (3.0f - 2.0f * t);
                    foot_x = g_prev_fx[leg] * (1.0f - ease);
                    foot_z = g_prev_fz[leg] + (g_motion.height - g_prev_fz[leg]) * ease;
                } else {
                    foot_x = 0;
                    foot_z = g_motion.height;
                }
            } else if (eff_stride < 0.0f) {
                // stride < 0: 完全站立不动
                foot_x = 0;
                foot_z = eff_height;
            } else {
                // stride >= 0: 运动步态 (方向由 speed 符号决定)
                float direction = (float)eff_dir;

                float leg_phase = g_phase + offsets[leg] * TWO_PI;
                if (leg_phase > TWO_PI) leg_phase -= TWO_PI;
                float phase_norm = leg_phase / TWO_PI;

                // 转弯 — 单侧减速 (-1左 ~ +1右)
                float leg_stride = eff_stride;
                if (g_motion.turn != 0.0f) {
                    float factor = fabsf(g_motion.turn);
                    if ((g_motion.turn < 0 && leg_side[leg] == IK_SIDE_LEFT) ||
                        (g_motion.turn > 0 && leg_side[leg] == IK_SIDE_RIGHT))
                        leg_stride = eff_stride * (1.0f - factor);
                }
                float max_stride = 2.0f * BODY_HALF_L * 0.85f;
                if (leg_stride > max_stride) leg_stride = max_stride;

                foot_trajectory(phase_norm, leg_stride,
                                eff_height, g_motion.lift_height,
                                eff_duty, direction, &foot_x, &foot_z);

                // 起步过渡: 足端从站立 (0,height) 缓动到预备位轨迹
                if (g_motion.pose_trans == 1) {
                    float t = g_motion.pose_timer / TRANS_TIME;
                    float ease = t * t * (3.0f - 2.0f * t);
                    foot_x = 0.0f + (foot_x - 0.0f) * ease;
                    foot_z = eff_height + (foot_z - eff_height) * ease;
                }
            }

            // 脚中位偏移
            foot_x += g_motion.center_offset;

            // 身体姿态补偿: roll/pitch → 四腿高度偏置
            float deg2rad = 0.0174533f;
            float z_roll  = BODY_HALF_W * tanf(g_motion.body_roll  * deg2rad);
            float z_pitch = BODY_HALF_L * tanf(g_motion.body_pitch * deg2rad);
            if (leg_side[leg] == IK_SIDE_LEFT)
                foot_z -= z_roll;   else foot_z += z_roll;
            if (leg == 0 || leg == 2)  // front legs
                foot_z += z_pitch;  else foot_z -= z_pitch;

            // 保存当前帧足端 (停过渡用)
            g_prev_fx[leg] = foot_x;
            g_prev_fz[leg] = foot_z;

            ik_result_t ik = ik_solve_2dof(foot_x, foot_z,
                                g_motion.ik_L1, g_motion.ik_L2,
                                leg_side[leg]);
            servo_group_add(leg_hip_ch[leg],  ik.hip_deg);
            servo_group_add(leg_knee_ch[leg], ik.knee_deg);
        }

        servo_group_commit();

        if (is_jump && g_motion.stand_up_elapsed >= 1.5f) {
            g_motion.gait = GAIT_STAND;
            g_was_moving = false;
            for (int i = 0; i < 4; i++) {
                g_prev_fx[i] = 0.0f;
                g_prev_fz[i] = g_motion.height;
            }
            ESP_LOGI(TAG, "Jump complete, now standing");
        }
        if (is_stand_up && g_motion.stand_up_elapsed >= 3.3f) {
            g_motion.gait = GAIT_STAND;
            g_was_moving = false;
            for (int i = 0; i < 4; i++) {
                g_prev_fx[i] = 0.0f;
                g_prev_fz[i] = g_motion.height;
            }
            ESP_LOGI(TAG, "Stand-up complete, now standing");
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

/* ---- API ---- */

void motion_task_start(void)
{
    if (g_task_handle != NULL) return;
    xTaskCreatePinnedToCore(motion_task_main, "motion", MOTION_STACK,
                            NULL, MOTION_PRIORITY, &g_task_handle, MOTION_CORE);
}

const motion_state_t *motion_get_state(void) { return &g_motion; }

void motion_set_gait(gait_type_t gait)
{
    if (gait < GAIT_COUNT) {
        g_motion.gait = gait;
        g_motion.enabled = true;
        if (gait != GAIT_STAND && gait != GAIT_SIT && gait != GAIT_STAND_UP)
            motion_apply_gait_params(gait);
    }
}

/* ---- 参数校验 ---- */
// 返回 true = 有问题, 参数不应写入
static bool motion_validate_params(float stride, float height)
{
    float L1 = g_motion.ik_L1;
    float L2 = g_motion.ik_L2;
    float x  = stride * 0.5f;
    float z  = height;
    bool  bad = false;

    // 机械步幅上限
    float max_stride = 2.0f * BODY_HALF_L * 0.85f;
    if (stride > max_stride) {
        ESP_LOGW(TAG, "⚠ 步幅过大: stride=%.0f > max=%.0f (前后脚干涉), 参数未写入",
                 stride, max_stride);
        bad = true;
    }

    // IK 膝角检查 (使用 ik.h 中的 IK_KNEE_MIN / IK_KNEE_MAX)
    float d_sq = x * x + z * z;
    float d = sqrtf(d_sq);
    float L_sum = L1 + L2 - 0.5f;

    float cos_knee = (L1 * L1 + L2 * L2 - d * d) / (2.0f * L1 * L2);
    if (cos_knee < -1.0f) cos_knee = -1.0f;
    if (cos_knee > 1.0f)  cos_knee = 1.0f;
    float knee_raw_deg = acosf(cos_knee) * 180.0f / (float)M_PI;

    // 左膝 servo = knee, 右膝 servo = 180 - knee
    float knee_left  = knee_raw_deg;
    float knee_right = 180.0f - knee_raw_deg;

    if (knee_left < IK_KNEE_MIN || knee_left > IK_KNEE_MAX ||
        knee_right < IK_KNEE_MIN || knee_right > IK_KNEE_MAX) {

        float worst = knee_left;
        if (knee_right < IK_KNEE_MIN || knee_right > IK_KNEE_MAX)
            worst = (knee_left < knee_right) ? knee_left : knee_right;

        float cos_lim_min = cosf(IK_KNEE_MIN * (float)M_PI / 180.0f);
        float cos_lim_max = cosf(IK_KNEE_MAX * (float)M_PI / 180.0f);
        float d2_max = L1 * L1 + L2 * L2 - 2.0f * L1 * L2 * cos_lim_max;
        float d2_min = L1 * L1 + L2 * L2 - 2.0f * L1 * L2 * cos_lim_min;
        float d_max = sqrtf(d2_max);
        float d_min = sqrtf(d2_min);

        if (d > L_sum) {
            ESP_LOGW(TAG, "⚠ 足端不可达! d=%.1f > L1+L2=%.0f, 参数未写入", d, L1+L2);
        } else {
            ESP_LOGW(TAG, "⚠ 膝角触限! stride=%.0f height=%.0f → 膝≈%.0f° (限 %.0f°~%.0f°), 参数未写入",
                     stride, height, worst, IK_KNEE_MIN, IK_KNEE_MAX);
            ESP_LOGW(TAG, "  建议: height %.0f~%.0fmm", d_min, d_max);
        }
        bad = true;
    }
    return bad;
}

int motion_check_params(float stride, float height)
{
    return motion_validate_params(stride, height) ? 1 : 0;
}

void motion_set_params(float speed, float stride, float height)
{
    // 速度上限
    if (fabsf(speed) > 10.0f) {
        ESP_LOGW(TAG, "⚠ 速度过大: |speed|=%.1f > 10, 参数未写入", fabsf(speed));
        ESP_LOGW(TAG, "→ 保持原值: speed=%.2f stride=%.0f height=%.0f",
                 g_motion.speed, g_motion.stride, g_motion.height);
        return;
    }

    // 校验 stride/height (stride<0=站立, 不校验)
    float new_stride = (stride >= 0) ? stride : g_motion.stride;
    float new_height = (height >= 0) ? height : g_motion.height;
    if (stride > 0 && motion_validate_params(new_stride, new_height)) {
        ESP_LOGW(TAG, "→ 保持原值: speed=%.2f stride=%.0f height=%.0f",
                 g_motion.speed, g_motion.stride, g_motion.height);
        return;
    }

    g_motion.target_speed = speed;  // 正=前进, 负=后退, 0=停
    if (stride >= 0) g_motion.stride = stride;  // stride≥0 写入 (<0 表示不更新)
    if (height > 0) g_motion.height = height;   // height>0 写入 (0 表示不更新)
    ESP_LOGI(TAG, "Params: speed=%.2f stride=%.0f height=%.0f (omega=%.3f rad/s, cycle=%.1fs)",
             g_motion.speed, g_motion.stride, g_motion.height,
             g_motion.omega_base * fabsf(g_motion.speed),
             TWO_PI / (g_motion.omega_base * fabsf(g_motion.speed) + 0.001f));
}

void motion_cal_ik(float L1, float L2)
{
    g_motion.ik_L1 = L1;
    g_motion.ik_L2 = L2;
    ESP_LOGI(TAG, "IK cal: L1=%.1f L2=%.1f", L1, L2);
}

void motion_set_omega(float omega)
{
    g_motion.omega_base = omega;
    ESP_LOGI(TAG, "Omega base: %.2f rad/s", omega);
}

void motion_set_lift(float lift)
{
    g_motion.lift_height = lift;
    ESP_LOGI(TAG, "Lift height: %.0f mm", lift);
}

void motion_set_body_pose(float roll, float pitch, float yaw)
{
    g_motion.body_roll  = roll;
    g_motion.body_pitch = pitch;
    g_motion.body_yaw   = yaw;
}

void motion_set_turn(float turn)
{
    if (turn < -1.0f) turn = -1.0f;
    if (turn >  1.0f) turn =  1.0f;
    g_motion.turn = turn;
}

void motion_set_center(float offset)
{
    g_motion.center_offset = offset;
    ESP_LOGI(TAG, "Center offset: %.0f mm", offset);
}

void motion_emergency_stop(void)
{
    g_motion.emergency_stop = true;
    g_motion.enabled = false;
    ESP_LOGW(TAG, "Emergency stop!");
}

void motion_resume(void)
{
    g_motion.emergency_stop = false;
    g_motion.enabled = true;
    ESP_LOGI(TAG, "Resumed");
}

void motion_stand_up(void)
{
    g_motion.gait = GAIT_STAND_UP;
    g_motion.enabled = true;
    g_motion.emergency_stop = false;
    g_motion.stand_up_elapsed = 0.0f;
    ESP_LOGI(TAG, "Stand-up sequence started (hold=0.3s ramp=3.0s)");
}

void motion_jump(void)
{
    g_motion.gait = GAIT_JUMP;
    g_motion.enabled = true;
    g_motion.emergency_stop = false;
    g_motion.stand_up_elapsed = 0.0f;
    ESP_LOGI(TAG, "Jump sequence started");
}
