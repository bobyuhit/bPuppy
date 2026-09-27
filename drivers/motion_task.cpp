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

#define MIN_HEIGHT        15.0f    // 最低站立高度 (mm), 过低腿折叠干涉

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
static gait_type_t g_pending_gait = GAIT_STOP;   // 减速停完成后切换的静态步态
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
    return g == GAIT_STOP;
}

/* ---- 全局状态 ---- */
static motion_mode_t g_mode = MODE_IDLE;   // 运行模式状态机

__attribute__((weak)) motion_state_t g_motion = {
    .gait           = GAIT_STOP,
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
    default: break; // stand/stand_up 不调参数
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

        bool is_stand    = (g_motion.gait == GAIT_STOP);

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
        bool now_static = is_stand;
        if (now_static && g_was_moving) {
            // 行走 → 静止: 启动停步过渡
            g_motion.pose_trans = 2;
            g_motion.pose_timer = 0.0f;
            g_motion.speed = 0.0f;
        } else if (!now_static && !g_was_moving) {
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

        // 换向检测: 运动方向变化 → 减速停 → 直接反向 (平滑换向, 不经过 GAIT_STOP)
        int cur_dir = (g_motion.stride < 0.0f) ? -1 : 1;
        if (cur_dir != g_dir_last && !now_static) {
            if (!g_stop_decel && g_motion.pose_trans == 0) {
                g_stop_decel = true;         // 先减速停 (每半步 -3 到 0)
                g_pending_gait = GAIT_STOP;  // 停稳后切静态步态 (真正停止才用)
                g_reverse = true;
                g_rev_gait = g_motion.gait;  // 记住运动步态 (反向起步用)
                g_decel_sign = g_dir_last;   // 记住原方向 (减速停期间保持, 防腿打架)
            }
        }
        g_dir_last = cur_dir;

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
        if (!is_stand) {
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
        // eff_stride 取绝对值 (方向由下方 stride_sign 决定, 与 GO 一致;
        // 否则 TROT/WALK 含符号 stride + 符号 stride_sign 双重符号 → 后退变前进)
        float eff_stride = fabsf(g_motion.stride);
        float eff_height = g_motion.height;
        // GO: speed≤4=walk  speed≥6=trot  → duty/gap/stride/height 插值
        // (body_pitch 不在此列: 它由 set_body_pose 手动设定, 三个步态一视同仁地生效,
        //  在这里清零会让 GO 下"设了没反应" —— 只有 walk/trot 生效)
        if (g_motion.gait == GAIT_GO) {
            float s = eff_speed;
            if (s <= 4.0f) {
                eff_duty=0.20f; eff_gap=0.04f;
                eff_stride=70.0f; eff_height=70.0f;
            } else if (s >= 6.0f) {
                eff_duty=0.40f; eff_gap=0.10f;
                eff_stride=50.0f; eff_height=70.0f;
            } else {
                float t = (s - 4.0f) / 2.0f;
                eff_duty   = 0.20f + t * 0.20f;
                eff_gap    = 0.04f + t * 0.06f;
                eff_stride = 70.0f - t * 20.0f;
                eff_height = 70.0f;
            }
        }
        // GO 起步首值注入: stride=0 且非停步 → 立刻给第1档 (不等半周期边界)
        if (g_motion.gait == GAIT_GO && g_stride_smooth < 0.05f
            && eff_speed > 0.15f && !g_stop_decel
            && g_motion.target_speed > 0.1f) {
            g_stride_smooth = eff_stride / 5.0f;
        }
        // GO 步长平滑: 起步 step=±目标/5 (5半周期到位)
        // 停步/换向停: 立刻切到目标/3 → 1个半周期后归零 → 切静态
        if (g_motion.gait == GAIT_GO) {
            bool stopping = g_stop_decel || g_motion.target_speed <= 0.1f;
            if (stopping) {
                // 立刻切 1/3 (不在半周期边界), 1个半周期后归零
                float one_third = fabsf(eff_stride) / 3.0f;
                if (g_stride_smooth > one_third + 0.1f)
                    g_stride_smooth = one_third;
                if (g_half_pulse)
                    g_stride_smooth = 0.0f;
            } else if (g_half_pulse) {
                float diff = eff_stride - g_stride_smooth;
                if (fabsf(diff) > 0.1f) {
                    float step = fabsf(eff_stride) / 5.0f;
                    if (step < 1.0f) step = 1.0f;
                    if (diff >  step) diff =  step;
                    if (diff < -step) diff = -step;
                    g_stride_smooth += diff;
                }
            }
            eff_stride = g_stride_smooth;
        }
        g_half_pulse = false;

        // 减速停完成: 步长渐收到 0
        // 换向: 直接反方向走 (步长=0 脚已在 stand, 不经过 stand 收脚+起步过渡)
        // 按停/滑块到0: 切静态步态, 收脚站定
        // ★ 非 GO 步态 (TROT/WALK) 无 g_stride_smooth, 减速停/换向立即完成
        float stop_smooth = (g_motion.gait == GAIT_GO) ? g_stride_smooth : 0.0f;
        if ((g_stop_decel || g_motion.target_speed <= 0.1f) && stop_smooth <= 0.1f) {
            g_stop_decel = false;
            if (g_reverse) {
                g_reverse = false;              // 直接反向 (跳过 stand 中间态)
            } else {
                g_motion.gait = g_pending_gait;
                g_motion.speed = 0.0f;
            }
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

        servo_group_begin();

        for (int leg = 0; leg < 4; leg++) {
            float foot_x, foot_z;

            if (is_stand) {
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

            // 保存当前帧足端 (停过渡用)
            // ★ 必须存在加偏移/加补偿**之前** —— 存的是轨迹原始值。
            //   停过渡 (上面 is_stand + pose_trans==2 那段) 拿它插值, 插值完下面还会
            //   再加一次偏移和补偿; 要是这里存的是加过的值, 那一帧就会偏移+补偿各算
            //   两次 → 切 stop 时脚瞬跳 co (默认 0 看不出来) + z_pitch (pitch=-5°
            //   时 5.5mm ≈ 2~3° 的抖)。
            g_prev_fx[leg] = foot_x;
            g_prev_fz[leg] = foot_z;

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

            ik_result_t ik = ik_solve_2dof(foot_x, foot_z,
                                g_motion.ik_L1, g_motion.ik_L2,
                                leg_side[leg], leg_pair[leg]);
            // 静态姿态限速: 每帧最多走 SERVO_MAX_DEG_PER_FRAME 度
            if (is_stand) {
                ik.hip_deg  = servo_step_toward(leg_hip_ch[leg],  ik.hip_deg,  SERVO_MAX_DEG_PER_FRAME);
                ik.knee_deg = servo_step_toward(leg_knee_ch[leg], ik.knee_deg, SERVO_MAX_DEG_PER_FRAME);
            }
            g_smooth_angles[leg_hip_ch[leg]]  = ik.hip_deg;
            g_smooth_angles[leg_knee_ch[leg]] = ik.knee_deg;
            servo_group_add(leg_hip_ch[leg],  ik.hip_deg);
            servo_group_add(leg_knee_ch[leg], ik.knee_deg);
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
    xTaskCreatePinnedToCore(motion_task_main, "motion", MOTION_STACK,
                            NULL, MOTION_PRIORITY, &g_task_handle, MOTION_CORE);
}

const motion_state_t *motion_get_state(void) { return &g_motion; }

void motion_set_gait(gait_type_t gait)
{
    if (gait < GAIT_COUNT) {
        // 任何步态指令 → 自动进入运动模式 (Python 无需显式切换)
        motion_set_mode(MODE_MOTION);
        g_motion.enabled = true;
        // 运动步态 → 静态步态:
        //   GO: 先减速停 (g_stride_smooth 平滑步长归零再切)
        //   TROT/WALK: 无平滑步长, 立即切 GAIT_STOP (限速逼近站姿)
        if (is_static_gait(gait) && !is_static_gait(g_motion.gait)) {
            if (g_motion.gait == GAIT_GO) {
                if (!g_stop_decel) {
                    g_stop_decel = true;
                    g_pending_gait = gait;
                }
                return;
            }
            g_motion.gait = gait;   // TROT/WALK → STOP 立即切
            return;
        }
        // 静态→运动 或 同态切换: 立即生效, 并取消挂起的减速停
        if (!is_static_gait(gait)) {
            g_stop_decel = false;
            g_reverse = false;
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

// 返回 true = 有问题, 参数不应写入
static bool motion_validate_params(float stride, float height, float lift)
{
    float L1 = g_motion.ik_L1;
    float L2 = g_motion.ik_L2;
    float x  = fabsf(stride) * 0.5f;
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

    // 机体姿态补偿量 —— 与运行时 (motion_task_main 里加 foot_z 的那段) 同一套符号,
    //   逐腿算: 左腿 -z_roll / 右腿 +z_roll, 前腿 +z_pitch / 后腿 -z_pitch。
    // ★ 必须算进来: 漏了它时校验看的是"没补偿的 z", 而狗实际走的是"补偿后的 z"。
    //   实测在用的 pitch=-5° → z_pitch=5.5mm, 后腿实际比校验以为的低 5.5mm。
    float deg2rad = 0.0174533f;
    float z_roll  = g_motion.body_half_w * tanf(g_motion.body_roll  * deg2rad);
    float z_pitch = g_motion.body_half_l * tanf(g_motion.body_pitch * deg2rad);

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

    for (int i = 0; i < 5; i++) {
        float ease = ease_pts[i];
        float xv = -x + 2.0f * x * ease;
        xv += g_motion.center_offset;
        float zv = height - lift * sinf(ease * (float)M_PI);
        for (int k = 0; k < 4; k++) {
            float zf = zv + ((leg_side[k] == IK_SIDE_LEFT) ? -z_roll : z_roll)
                          + ((k == 0 || k == 2) ? z_pitch : -z_pitch);
            if (zf < min_zf) { min_zf = zf; min_leg = k; }
            if (zf < 0.0f) continue;   // 入地由循环后统一报一次, 这里跳过角度检查免得刷屏
            if (ik_pos_check(xv, zf, leg_side[k], leg_pair[k], L1, L2)) {
                ESP_LOGW(TAG, "⚠ 足端 (x=%.0f z=%.0f 腿%d) 髋/膝超限, 参数未写入 "
                         "(stride=%.0f height=%.0f lift=%.0f roll=%.1f pitch=%.1f)",
                         xv, zf, k, stride, height, lift,
                         g_motion.body_roll, g_motion.body_pitch);
                bad = true;
            }
        }
    }

    // 补偿后的 z 必须有下限: 上面只查了 ik_pos_check (可达距离 + 髋/膝角度),
    //   它拦不住"略低于地面"的点 —— x 大、z 略负时 d=√(x²+z²) 仍在可达范围内,
    //   角度也可能合规, 于是脚被要求插到地面以下。机体姿态压得狠时会碰到。
    if (min_zf < 0.0f) {
        ESP_LOGW(TAG, "⚠ 足端入地 (抬腿过度): 最深 z=%.1fmm (腿%d, 叠加姿态补偿后) —— "
                 "lift=%.0f 超过 height=%.0f, 或姿态压太狠 (roll=%.1f pitch=%.1f), 参数未写入",
                 min_zf, min_leg, lift, height,
                 g_motion.body_roll, g_motion.body_pitch);
        bad = true;
    }
    return bad;
}

int motion_check_params(float stride, float height)
{
    return motion_validate_params(stride, height, g_motion.lift_height) ? 1 : 0;
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

    // 校验步幅/高度 (含抬腿, 髋/膝角, 姿态补偿)
    // ★ 不加任何前置条件 —— MicroPython 绑定层判"要不要打被拒日志"用的是同一句
    //   motion_check_params()。两边条件不一致时日志会撒谎:
    //   以前这里带 `stride != 0 &&`, 而那边带 `(stride > 0 || height > 0) &&`,
    //   于是 set_params(2, 0, 20) 这组"抬腿超过站高"的参数**跳过校验直接写入**。
    if (motion_validate_params(stride, height, g_motion.lift_height)) {
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

/* ---- 几何 setter: 非法输入一律拒写并保持原值 ----
 *
 * 复用运动参数那套现成模式 (见 motion_set_lift): 返回 bool 表示是否写入成功,
 * 由 MicroPython 绑定层 (motion_task_mpy.c) 把"被拒"打到用户看得见的地方。
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
// ★ 偏移超出可达范围时运行时**不报错** —— ik_solve_2dof 只把 d 钳到可达值,
//   表现为腿静默失去伸展 (狗歪着走)。而 ik_pos_check 只在 motion_validate_params
//   里被调用, 报的还是"stride/height/lift 超限", 骂错了参数。所以必须在这里拦。
// NaN 也走到 false (NaN 参与比较恒为假) → 视为非法, 无需额外判断。
static bool geom_center_offset_valid(float off)
{
    return fabsf(off) <= g_motion.ik_L1 * 0.5f;
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
    if (g_task_handle == NULL) motion_load_geometry();
}

void motion_set_omega(float omega)
{
    g_motion.omega_base = omega;
    ESP_LOGI(TAG, "Omega base: %.2f rad/s", omega);
}

bool motion_set_lift(float lift)
{
    // 抬腿校验: 用当前 stride/height 检查是否超限/干涉
    if (motion_validate_params(g_motion.stride, g_motion.height, lift)) {
        ESP_LOGW(TAG, "⚠ 抬腿超限: lift=%.0f (stride=%.0f height=%.0f), 保持原值 %.0f",
                 lift, g_motion.stride, g_motion.height, g_motion.lift_height);
        return false;
    }
    g_motion.lift_height = lift;
    ESP_LOGI(TAG, "Lift height: %.0f mm", lift);
    return true;
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

bool motion_set_center(float offset)
{
    if (!geom_center_offset_valid(offset)) {
        ESP_LOGW(TAG, "Center offset rejected: %.1fmm 超出 ±%.1fmm (大腿 %.1f 的一半), "
                 "保持原值 %.1fmm", offset, g_motion.ik_L1 * 0.5f, g_motion.ik_L1,
                 g_motion.center_offset);
        return false;
    }
    g_motion.center_offset = offset;
    motion_save_geometry();
    ESP_LOGI(TAG, "Center offset: %.0f mm (saved)", offset);
    return true;
}

bool motion_is_running(void)
{
    return g_motion.enabled && !g_motion.emergency_stop;
}

void motion_set_mode(motion_mode_t mode)
{
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
    if (g_mode != MODE_POSE) {
        motion_set_mode(MODE_POSE);
    }
}
