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
#include "nvs_flash.h"
#include "nvs.h"
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

/* ---- 身体几何 (运行时变量, 默认值来自 ik.h, NVS 可覆盖) ---- */

/* ---- 腿配置: LF, LH, RF, RH ---- */
static const int leg_hip_ch[]  = {0, 2, 4, 6};
static const int leg_knee_ch[] = {1, 3, 5, 7};
static const int leg_side[]    = {IK_SIDE_LEFT, IK_SIDE_LEFT,
                                   IK_SIDE_RIGHT, IK_SIDE_RIGHT};
static const int leg_pair[]    = {IK_LEG_FRONT, IK_LEG_REAR,
                                   IK_LEG_FRONT, IK_LEG_REAR};

/* ---- 舵机角度限速 (每帧最大变化) ---- */
#define SERVO_MAX_DEG_PER_FRAME  3.0f   // 每 20ms 最多移动度数
static float g_smooth_angles[8] = {90, 90, 90, 90, 90, 90, 90, 90};
static int   g_dir_last = 1;          // 上次运动方向 (+1 前进 / -1 后退)
static bool  g_reverse = false;       // 换向过渡中 (停→stand→反向起步)
static float g_stride_smooth = 0.0f;  // GO 平滑步长 (起步从 0 爬升)
static bool  g_half_pulse = false;    // 半周期脉冲 (步长平滑触发)
static bool  g_stop_decel = false;    // 减速停进行中 (运动步态下减速, 未收脚)
static gait_type_t g_pending_gait = GAIT_STAND;  // 减速停完成后切换的静态步态
static gait_type_t g_rev_gait = GAIT_GO;         // 换向前运动步态 (反向起步用)
static int   g_decel_sign = 1;        // 换向减速停期间保持的原方向 (+1 前进 / -1 后退)

// 限速: 往 target 方向最多走 max_step 度
static float servo_step_toward(int ch, float target, float max_step) {
    float cur = g_smooth_angles[ch];
    if (target > cur + max_step) return cur + max_step;
    if (target < cur - max_step) return cur - max_step;
    return target;
}

static bool is_static_gait(gait_type_t g) {
    return g == GAIT_STAND || g == GAIT_CROUCH || g == GAIT_SIT || g == GAIT_PLAY_BOW || g == GAIT_WAVE;
}

/* ---- 全局状态 ---- */
__attribute__((weak)) motion_state_t g_motion = {
    .gait           = GAIT_STAND,
    .speed          = 0.0f,          // 实际速度静止为 0
    .target_speed   = SPEED_DEFAULT, // 目标速度默认 2.5
    .stride         = STRIDE_DEFAULT,
    .height         = HEIGHT_DEFAULT,
    .lift_height    = LIFT_DEFAULT,
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
    .center_offset  = CENTER_OFFSET_DEFAULT,
    .body_half_l    = IK_BODY_HALF_L_DEFAULT,
    .body_half_w    = IK_BODY_HALF_W_DEFAULT,
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
        bool is_crouch      = (g_motion.gait == GAIT_CROUCH);
        bool is_sit         = (g_motion.gait == GAIT_SIT);
        bool is_play_bow    = (g_motion.gait == GAIT_PLAY_BOW);
        bool is_stand_up = (g_motion.gait == GAIT_STAND_UP);
        bool is_jump     = (g_motion.gait == GAIT_JUMP);
        bool is_wave     = (g_motion.gait == GAIT_WAVE);

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
        bool now_static = (is_stand || is_crouch || is_sit || is_play_bow);
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

        // 换向检测: 运动方向变化 → 减速停 → stand → 反向起步 (平滑换向)
        int cur_dir = (g_motion.stride < 0.0f) ? -1 : 1;
        if (cur_dir != g_dir_last && !now_static && !is_stand_up) {
            if (!g_stop_decel && g_motion.pose_trans == 0) {
                g_stop_decel = true;         // 先减速停 (每半步 -3 到 0)
                g_pending_gait = GAIT_STAND; // 停稳后先到 stand
                g_reverse = true;
                g_rev_gait = g_motion.gait;  // 记住运动步态 (反向起步用)
                g_decel_sign = g_dir_last;   // 记住原方向 (减速停期间保持, 防腿打架)
            }
        }
        g_dir_last = cur_dir;
        // 换向: stand 到位后自动反向起步 (步长从 0, 速度 kick)
        if (g_reverse && g_motion.gait == GAIT_STAND && g_motion.pose_trans == 0) {
            g_reverse = false;
            g_motion.gait = g_rev_gait;      // 恢复运动步态
            g_motion.pose_trans = 1;         // 起步过渡
            g_motion.pose_timer = 0.0f;
        }

        // speed=步频 stride=步幅+方向 (正=前, 负=后)
        float eff_speed = g_motion.speed;

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
        if (!is_stand && !is_crouch && !is_stand_up) {
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
                float ds = target_eff - g_motion.speed;
                if (fabsf(ds) < 0.15f) {
                    g_motion.speed = target_eff; // 接近就到位
                } else if (!g_stop_decel && g_motion.speed < 0.15f && target_eff > 0.0f) {
                    // 起步 kick (仅"从静止到起步": 站立→GO / 换向反向起步 / 暂停恢复):
                    // speed=0 相位不动, 先给起步速度(≤2.5)让相位能动, 再半周期爬升
                    g_motion.speed = (target_eff < SPEED_DEFAULT)
                                   ? target_eff : SPEED_DEFAULT;
                    g_stride_smooth = 0.0f;   // 起步步长从 0 爬升 (4 半周期到目标)
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
                eff_stride=70.0f; eff_height=70.0f; eff_pitch=0.0f;
            } else if (s >= 6.0f) {
                eff_duty=0.40f; eff_gap=0.10f;
                eff_stride=50.0f; eff_height=70.0f; eff_pitch=0.0f;
            } else {
                float t = (s - 4.0f) / 2.0f;
                eff_duty   = 0.20f + t * 0.20f;
                eff_gap    = 0.04f + t * 0.06f;
                eff_stride = 70.0f - t * 20.0f;
                eff_height = 70.0f;
                eff_pitch  = 0.0f;
            }
            g_motion.body_pitch = eff_pitch;
        }
        // GO 步长平滑: 半周期点向目标逼近 ±目标/2 (2 半周期=1 周期到位, 与起步对称)
        // 停步 (减速停 / 目标速度≈0): 目标改为 0, 同样按 ±目标/2 渐收
        if (g_half_pulse && g_motion.gait == GAIT_GO) {
            bool stopping = g_stop_decel || g_motion.target_speed <= 0.1f;
            float stride_target = stopping ? 0.0f : eff_stride;
            float diff = stride_target - g_stride_smooth;
            if (fabsf(diff) > 0.1f) {
                float step = fabsf(eff_stride) / 2.0f;
                if (step < 1.0f) step = 1.0f;
                if (diff >  step) diff =  step;
                if (diff < -step) diff = -step;
                g_stride_smooth += diff;
            }
        }
        if (g_motion.gait == GAIT_GO) {
            eff_stride = g_stride_smooth;
        }
        g_half_pulse = false;

        // 减速停完成: 步长渐收到 0 → 切换静态步态 (最后一步最短, 收脚站定)
        // 按停 / 滑块到 0 都会触发: 步长归零, 速度清零, 切静态收脚
        if ((g_stop_decel || g_motion.target_speed <= 0.1f) && g_stride_smooth <= 0.1f) {
            g_stop_decel = false;
            g_motion.gait = g_pending_gait;
            g_motion.speed = 0.0f;          // 停稳后速度清零 (相位冻结, 让 stand 收脚)
        }

        // 计算步态偏移: 正向 (前腿迈) + 反向 (后腿迈) 各一套
        // turn 可能让不同侧走向不同方向，per-leg 按方向选用
        float offsets_fwd[4], offsets_rev[4];
        compute_offsets(eff_duty, eff_gap,  1, offsets_fwd);
        compute_offsets(eff_duty, eff_gap, -1, offsets_rev);

        // 过渡开始时设一次相位到全踩地中点, 然后正常累计
        if (g_motion.pose_trans == 1 && g_motion.pose_timer < 0.02f) {
            float mid = all_stance_mid(eff_duty, eff_gap);
            g_phase = mid * TWO_PI;
        }

        // ---- play_bow 摇屁股 (每帧计算一次) ----
        static float wiggle_phase = 0;
        static gait_type_t wiggle_last = GAIT_STAND;
        float wiggle = 0;
        if (is_play_bow) {
            if (wiggle_last != GAIT_PLAY_BOW) wiggle_phase = 0;
            wiggle_last = GAIT_PLAY_BOW;
            if (wiggle_phase < 6.283f * 8.0f) {
                wiggle = sinf(wiggle_phase) * 10.0f;
                wiggle_phase += 6.283f * 4.0f * dt;  // 4Hz
            }
        } else {
            wiggle_last = g_motion.gait;
        }

        // ---- wave 分阶段: sit → 后腿到位 → 抬RF挥动 → RF恢复 → 后腿恢复回sit ----
        static int wave_stage = 0;
        static float wave_timer = 0.0f;
        static float wave_phase = 0.0f;
        static gait_type_t wave_last = GAIT_STAND;
        float wave_knee = 0;
        if (is_wave) {
            if (wave_last != GAIT_WAVE) { wave_stage = 0; wave_timer = 0; wave_phase = 0; }
            wave_last = GAIT_WAVE;

            if (wave_stage == 0) {           // ① 先 sit (停留 0.3s)
                if (wave_timer > 0.3f) { wave_stage = 1; wave_timer = 0; }
            } else if (wave_stage == 1) {    // ② 后腿大腿到位 (0.6s)
                if (wave_timer > 0.6f) { wave_stage = 2; wave_timer = 0; wave_phase = 0; }
            } else if (wave_stage == 2) {    // ③ 抬右前腿 + 挥动 3 次
                if (wave_phase < 6.283f * 3.0f) {
                    wave_knee = sinf(wave_phase) * 10.0f;
                    wave_phase += 6.283f * 1.0f * dt;
                } else { wave_stage = 3; wave_timer = 0; }
            } else if (wave_stage == 3) {    // ④ 右前腿恢复 sit (0.5s)
                if (wave_timer > 0.5f) { wave_stage = 4; wave_timer = 0; }
            } else if (wave_stage == 4) {    // ⑤ 后腿恢复 sit (0.6s)
                if (wave_timer > 0.6f) { g_motion.gait = GAIT_SIT; }
            }
            wave_timer += dt;
        } else {
            wave_last = g_motion.gait;
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
                                    leg_side[leg], leg_pair[leg]);
                g_smooth_angles[leg_hip_ch[leg]]  = ik.hip_deg;
                g_smooth_angles[leg_knee_ch[leg]] = ik.knee_deg;
                servo_group_add(leg_hip_ch[leg],  ik.hip_deg);
                servo_group_add(leg_knee_ch[leg], ik.knee_deg);
                continue;
            } else if (is_crouch) {
                float sit_hip  = (leg_side[leg] == IK_SIDE_LEFT) ? 135.0f : 45.0f;
                float sit_knee = (leg_side[leg] == IK_SIDE_LEFT) ? 45.0f : 135.0f;
#if IK_KNEE_REAR_FORWARD
                if (leg_pair[leg] == IK_LEG_REAR) {
                    sit_hip  = 180.0f - sit_hip;
                    sit_knee = 180.0f - sit_knee;
                }
#endif
                // 静态姿态限速: 每帧最多走 SERVO_MAX_DEG_PER_FRAME 度
                sit_hip  = servo_step_toward(leg_hip_ch[leg],  sit_hip,  SERVO_MAX_DEG_PER_FRAME);
                sit_knee = servo_step_toward(leg_knee_ch[leg], sit_knee, SERVO_MAX_DEG_PER_FRAME);
                g_smooth_angles[leg_hip_ch[leg]]  = sit_hip;
                g_smooth_angles[leg_knee_ch[leg]] = sit_knee;
                servo_group_add(leg_hip_ch[leg],  sit_hip);
                servo_group_add(leg_knee_ch[leg], sit_knee);
                continue;
            } else if (is_sit) {
                // 猫坐姿态: 前腿撑, 后腿折 (实测角度)
                float s_hip, s_knee;
                if (leg_side[leg] == IK_SIDE_LEFT) {
                    s_hip  = (leg_pair[leg] == IK_LEG_FRONT) ? 120.0f : 10.0f;
                    s_knee = (leg_pair[leg] == IK_LEG_FRONT) ? 160.0f : 165.0f;
                } else {
                    s_hip  = (leg_pair[leg] == IK_LEG_FRONT) ? 60.0f : 170.0f;
                    s_knee = (leg_pair[leg] == IK_LEG_FRONT) ? 20.0f : 15.0f;
                }
                s_hip  = servo_step_toward(leg_hip_ch[leg],  s_hip,  SERVO_MAX_DEG_PER_FRAME);
                s_knee = servo_step_toward(leg_knee_ch[leg], s_knee, SERVO_MAX_DEG_PER_FRAME);
                g_smooth_angles[leg_hip_ch[leg]]  = s_hip;
                g_smooth_angles[leg_knee_ch[leg]] = s_knee;
                servo_group_add(leg_hip_ch[leg],  s_hip);
                servo_group_add(leg_knee_ch[leg], s_knee);
                continue;
            } else if (is_wave) {
                // 挥手分阶段: 0=sit 1~3=后腿到位+RF摆动 4=后腿恢复
                float s_hip, s_knee;
                // sit 基准角度
                if (leg_side[leg] == IK_SIDE_LEFT) {
                    s_hip  = (leg_pair[leg] == IK_LEG_FRONT) ? 120.0f : 10.0f;
                    s_knee = (leg_pair[leg] == IK_LEG_FRONT) ? 160.0f : 165.0f;
                } else {
                    s_hip  = (leg_pair[leg] == IK_LEG_FRONT) ? 60.0f : 170.0f;
                    s_knee = (leg_pair[leg] == IK_LEG_FRONT) ? 20.0f : 15.0f;
                }
                // 阶段 1~3: 后腿大腿到位 (LH=48, RH=132)
                if (leg_pair[leg] == IK_LEG_REAR && wave_stage >= 1 && wave_stage <= 3) {
                    s_hip = (leg_side[leg] == IK_SIDE_LEFT) ? 48.0f : 132.0f;
                }
                // 阶段 2: 右前腿抬起摆动
                if (leg_side[leg] == IK_SIDE_RIGHT && leg_pair[leg] == IK_LEG_FRONT && wave_stage == 2) {
                    s_hip  = 150.0f;
                    s_knee = 45.0f + wave_knee;
                }
                s_hip  = servo_step_toward(leg_hip_ch[leg],  s_hip,  SERVO_MAX_DEG_PER_FRAME);
                s_knee = servo_step_toward(leg_knee_ch[leg], s_knee, SERVO_MAX_DEG_PER_FRAME);
                g_smooth_angles[leg_hip_ch[leg]]  = s_hip;
                g_smooth_angles[leg_knee_ch[leg]] = s_knee;
                servo_group_add(leg_hip_ch[leg],  s_hip);
                servo_group_add(leg_knee_ch[leg], s_knee);
                continue;
            } else if (is_play_bow) {
                // 邀玩: 前低后高 (摇摆在循环外用 wiggle 变量)
                float p_hip, p_knee;
                if (leg_side[leg] == IK_SIDE_LEFT) {
                    p_hip  = (leg_pair[leg] == IK_LEG_FRONT) ? 50.0f : 70.0f;
                    p_knee = (leg_pair[leg] == IK_LEG_FRONT) ? 125.0f : 50.0f;
                } else {
                    p_hip  = (leg_pair[leg] == IK_LEG_FRONT) ? 130.0f : 110.0f;
                    p_knee = (leg_pair[leg] == IK_LEG_FRONT) ? 55.0f : 130.0f;
                }
                // 后腿同加同减
                if (leg_pair[leg] == IK_LEG_REAR)
                    p_knee += wiggle;
                p_hip  = servo_step_toward(leg_hip_ch[leg],  p_hip,  SERVO_MAX_DEG_PER_FRAME);
                p_knee = servo_step_toward(leg_knee_ch[leg], p_knee, SERVO_MAX_DEG_PER_FRAME);
                g_smooth_angles[leg_hip_ch[leg]]  = p_hip;
                g_smooth_angles[leg_knee_ch[leg]] = p_knee;
                servo_group_add(leg_hip_ch[leg],  p_hip);
                servo_group_add(leg_knee_ch[leg], p_knee);
                continue;
            } else if (is_stand_up) {
                float t = g_motion.stand_up_elapsed;
                float hold = 0.3f;
                float ramp = 3.0f;

                float sit_hip  = (leg_side[leg] == IK_SIDE_LEFT) ? 135.0f : 45.0f;
                float sit_knee = (leg_side[leg] == IK_SIDE_LEFT) ? 45.0f : 135.0f;
#if IK_KNEE_REAR_FORWARD
                if (leg_pair[leg] == IK_LEG_REAR) {
                    sit_hip  = 180.0f - sit_hip;
                    sit_knee = 180.0f - sit_knee;
                }
#endif

                ik_result_t ik = ik_solve_2dof(g_motion.center_offset, g_motion.height,
                                    g_motion.ik_L1, g_motion.ik_L2,
                                    leg_side[leg], leg_pair[leg]);

                if (t < hold) {
                    g_smooth_angles[leg_hip_ch[leg]]  = sit_hip;
                    g_smooth_angles[leg_knee_ch[leg]] = sit_knee;
                    servo_group_add(leg_hip_ch[leg],  sit_hip);
                    servo_group_add(leg_knee_ch[leg], sit_knee);
                } else {
                    float raw = (t - hold) / ramp;
                    if (raw > 1.0f) raw = 1.0f;
                    float ease = raw * raw * (3.0f - 2.0f * raw);
                    float out_hip  = sit_hip  + (ik.hip_deg  - sit_hip)  * ease;
                    float out_knee = sit_knee + (ik.knee_deg - sit_knee) * ease;
                    g_smooth_angles[leg_hip_ch[leg]]  = out_hip;
                    g_smooth_angles[leg_knee_ch[leg]] = out_knee;
                    servo_group_add(leg_hip_ch[leg],  out_hip);
                    servo_group_add(leg_knee_ch[leg], out_knee);
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
            } else {
                // 运动步态 (stride 正=前, 零=原地踏步, 负=后)

                // turn → per-side stride: 一侧不变, 另一侧 1→0→-1 连续缩放
                // 方向符号来自用户 stride (GO 只接管 magnitude, 不改方向)
                // 换向减速停期间保持原方向, 反向起步后才用新方向 (防止腿瞬间打架)
                float stride_sign;
                if (g_reverse && g_stop_decel)
                    stride_sign = (float)g_decel_sign;
                else
                    stride_sign = (g_motion.stride < 0.0f) ? -1.0f : 1.0f;
                float leg_stride;
                if (g_motion.turn >= 0.0f) {
                    float s = (leg_side[leg] == IK_SIDE_RIGHT)
                            ? (1.0f - g_motion.turn * 2.0f) : 1.0f;
                    leg_stride = eff_stride * s * stride_sign;
                } else {
                    float s = (leg_side[leg] == IK_SIDE_LEFT)
                            ? (1.0f + g_motion.turn * 2.0f) : 1.0f;
                    leg_stride = eff_stride * s * stride_sign;
                }
                float direction = (leg_stride < 0.0f) ? -1.0f : 1.0f;
                float abs_stride = fabsf(leg_stride);
                float max_stride = 2.0f * g_motion.body_half_l * 0.85f;
                if (abs_stride > max_stride) abs_stride = max_stride;

                // 每腿按自身方向选正向/反向相位
                // walk 族 (duty+gap<0.5) 方向反了需翻转相位; trot (0.5) 对称无需
                bool need_rev = (direction < 0) && (eff_duty + eff_gap < 0.50f);
                float offset = need_rev ? offsets_rev[leg] : offsets_fwd[leg];
                float leg_phase = g_phase + offset * TWO_PI;
                if (leg_phase > TWO_PI) leg_phase -= TWO_PI;
                float phase_norm = leg_phase / TWO_PI;

                foot_trajectory(phase_norm, abs_stride,
                                eff_height, g_motion.lift_height,
                                eff_duty, direction, &foot_x, &foot_z);

                // 起步过渡: 足端从站立 (0,height) 缓动到预备位轨迹
                if (g_motion.pose_trans == 1) {
                    float t = g_motion.pose_timer / TRANS_TIME;
                    if (t > 1.0f) t = 1.0f;
                    float ease = t * t * (3.0f - 2.0f * t);
                    foot_x = 0.0f + (foot_x - 0.0f) * ease;
                    foot_z = eff_height + (foot_z - eff_height) * ease;
                }
            }

            // 脚中位偏移
            foot_x += g_motion.center_offset;

            // 身体姿态补偿: roll/pitch → 四腿高度偏置
            float deg2rad = 0.0174533f;
            float z_roll  = g_motion.body_half_w * tanf(g_motion.body_roll  * deg2rad);
            float z_pitch = g_motion.body_half_l * tanf(g_motion.body_pitch * deg2rad);
            if (leg_side[leg] == IK_SIDE_LEFT)
                foot_z -= z_roll;   else foot_z += z_roll;
            if (leg == 0 || leg == 2)  // front legs
                foot_z += z_pitch;  else foot_z -= z_pitch;

            // 保存当前帧足端 (停过渡用)
            g_prev_fx[leg] = foot_x;
            g_prev_fz[leg] = foot_z;

            ik_result_t ik = ik_solve_2dof(foot_x, foot_z,
                                g_motion.ik_L1, g_motion.ik_L2,
                                leg_side[leg], leg_pair[leg]);
            // 静态姿态限速: 每帧最多走 SERVO_MAX_DEG_PER_FRAME 度
            if (is_stand || is_sit || is_play_bow) {
                ik.hip_deg  = servo_step_toward(leg_hip_ch[leg],  ik.hip_deg,  SERVO_MAX_DEG_PER_FRAME);
                ik.knee_deg = servo_step_toward(leg_knee_ch[leg], ik.knee_deg, SERVO_MAX_DEG_PER_FRAME);
            }
            g_smooth_angles[leg_hip_ch[leg]]  = ik.hip_deg;
            g_smooth_angles[leg_knee_ch[leg]] = ik.knee_deg;
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
    motion_load_geometry();  // 上电自动从 NVS 加载 L1/L2/髋距
    xTaskCreatePinnedToCore(motion_task_main, "motion", MOTION_STACK,
                            NULL, MOTION_PRIORITY, &g_task_handle, MOTION_CORE);
}

const motion_state_t *motion_get_state(void) { return &g_motion; }

void motion_set_gait(gait_type_t gait)
{
    if (gait < GAIT_COUNT) {
        g_motion.enabled = true;
        // 运动步态 → 静态步态: 先减速停, 延迟切换 (统一减速停, 减到 0 再收脚)
        if (is_static_gait(gait) && !is_static_gait(g_motion.gait)) {
            if (!g_stop_decel) {
                g_stop_decel = true;
                g_pending_gait = gait;
            }
            return;
        }
        // 静态→运动 或 同态切换: 立即生效, 并取消挂起的减速停
        if (!is_static_gait(gait)) {
            g_stop_decel = false;
            g_reverse = false;
        }
        g_motion.gait = gait;
        if (gait != GAIT_STAND && gait != GAIT_CROUCH && gait != GAIT_SIT && gait != GAIT_PLAY_BOW && gait != GAIT_STAND_UP)
            motion_apply_gait_params(gait);
    }
}

/* ---- 参数校验 ---- */
// 返回 true = 有问题, 参数不应写入
static bool motion_validate_params(float stride, float height)
{
    float L1 = g_motion.ik_L1;
    float L2 = g_motion.ik_L2;
    float x  = fabsf(stride) * 0.5f;
    float z  = height;
    bool  bad = false;

    // 机械步幅上限
    float max_stride = 2.0f * g_motion.body_half_l * 0.85f;
    if (fabsf(stride) > max_stride) {
        ESP_LOGW(TAG, "⚠ 步幅过大: |stride|=%.0f > max=%.0f (前后脚干涉), 参数未写入",
                 fabsf(stride), max_stride);
        bad = true;
    }

    // IK 膝角检查 (使用 ik.h 中的 ik_knee_min / ik_knee_max)
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

    if (knee_left < ik_knee_min || knee_left > ik_knee_max ||
        knee_right < ik_knee_min || knee_right > ik_knee_max) {

        float worst = knee_left;
        if (knee_right < ik_knee_min || knee_right > ik_knee_max)
            worst = (knee_left < knee_right) ? knee_left : knee_right;

        float cos_lim_min = cosf(ik_knee_min * (float)M_PI / 180.0f);
        float cos_lim_max = cosf(ik_knee_max * (float)M_PI / 180.0f);
        float d2_max = L1 * L1 + L2 * L2 - 2.0f * L1 * L2 * cos_lim_max;
        float d2_min = L1 * L1 + L2 * L2 - 2.0f * L1 * L2 * cos_lim_min;
        float d_max = sqrtf(d2_max);
        float d_min = sqrtf(d2_min);

        if (d > L_sum) {
            ESP_LOGW(TAG, "⚠ 足端不可达! d=%.1f > L1+L2=%.0f, 参数未写入", d, L1+L2);
        } else {
            ESP_LOGW(TAG, "⚠ 膝角触限! stride=%.0f height=%.0f → 膝≈%.0f° (限 %.0f°~%.0f°), 参数未写入",
                     stride, height, worst, ik_knee_min, ik_knee_max);
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
    // 速度范围 0~10
    if (speed < 0.0f || speed > 10.0f) {
        ESP_LOGW(TAG, "⚠ 速度超限: speed=%.1f (允许 0~10), 参数未写入", speed);
        ESP_LOGW(TAG, "→ 保持原值: speed=%.2f stride=%.0f height=%.0f",
                 g_motion.speed, g_motion.stride, g_motion.height);
        return;
    }

    // 校验步幅/高度
    if (stride != 0 && motion_validate_params(stride, height)) {
        ESP_LOGW(TAG, "→ 保持原值: speed=%.2f stride=%.0f height=%.0f",
                 g_motion.speed, g_motion.stride, g_motion.height);
        return;
    }

    g_motion.target_speed = speed;  // 纯频率, ≥0
    g_motion.stride = stride;       // 正=前, 零=原地踏步, 负=后
    g_motion.height = height;       // 站立高度
    ESP_LOGI(TAG, "Params: speed=%.2f stride=%.0f height=%.0f (omega=%.3f rad/s, cycle=%.1fs)",
             g_motion.speed, g_motion.stride, g_motion.height,
             g_motion.omega_base * g_motion.speed,
             TWO_PI / (g_motion.omega_base * g_motion.speed + 0.001f));
}

void motion_cal_ik(float L1, float L2)
{
    g_motion.ik_L1 = L1;
    g_motion.ik_L2 = L2;
    motion_save_geometry();
    ESP_LOGI(TAG, "IK cal: L1=%.1f L2=%.1f (saved)", L1, L2);
}

void motion_set_body_dims(float half_l, float half_w)
{
    g_motion.body_half_l = half_l;
    g_motion.body_half_w = half_w;
    motion_save_geometry();
    ESP_LOGI(TAG, "Body dims: half_l=%.1f half_w=%.1f (saved)", half_l, half_w);
}

void motion_set_joint_limits(float hip_min, float hip_max,
                              float knee_min, float knee_max)
{
    ik_hip_min  = hip_min;
    ik_hip_max  = hip_max;
    ik_knee_min = knee_min;
    ik_knee_max = knee_max;
    motion_save_geometry();
    ESP_LOGI(TAG, "Joint limits: hip[%.0f~%.0f] knee[%.0f~%.0f] (saved)",
             hip_min, hip_max, knee_min, knee_max);
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

    int32_t val;
    if (nvs_get_i32(handle, "l1", &val) == ESP_OK) g_motion.ik_L1 = val / 100.0f;
    if (nvs_get_i32(handle, "l2", &val) == ESP_OK) g_motion.ik_L2 = val / 100.0f;
    if (nvs_get_i32(handle, "bl", &val) == ESP_OK) g_motion.body_half_l = val / 100.0f;
    if (nvs_get_i32(handle, "bw", &val) == ESP_OK) g_motion.body_half_w = val / 100.0f;
    if (nvs_get_i32(handle, "hmin", &val) == ESP_OK) ik_hip_min  = val / 100.0f;
    if (nvs_get_i32(handle, "hmax", &val) == ESP_OK) ik_hip_max  = val / 100.0f;
    if (nvs_get_i32(handle, "kmin", &val) == ESP_OK) ik_knee_min = val / 100.0f;
    if (nvs_get_i32(handle, "kmax", &val) == ESP_OK) ik_knee_max = val / 100.0f;
    if (nvs_get_i32(handle, "co",   &val) == ESP_OK) g_motion.center_offset = val / 100.0f;
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
    motion_save_geometry();
    ESP_LOGI(TAG, "Center offset: %.0f mm (saved)", offset);
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
