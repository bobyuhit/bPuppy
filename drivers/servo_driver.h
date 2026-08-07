/*
 * bPuppy 舵机驱动 — 头文件
 *
 * 使用 ESP32-S3 LEDC PWM 控制器，支持 8 路舵机。
 * S3 LEDC: 8 通道, 全部 LOW_SPEED_MODE, 4 个定时器。
 * 舵机: 50Hz, 500-2500μs 脉宽, 角度范围由 SERVO_MODEL 宏编译期选择:
 *       SERVO_MODEL_180: 0°~180°   (180° 舵机)
 *       SERVO_MODEL_270: -35°~215° (270° 舵机, 90° 对准中位 1500us)
 */

#pragma once

#include "driver/ledc.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 硬件配置 ---- */
#define SERVO_MAX_CHANNELS  8       // S3 LEDC 共 8 通道

/* ---- 舵机编号 ---- */
#define SERVO_LF_HIP     0   // 左前大腿
#define SERVO_LF_KNEE    1   // 左前小腿
#define SERVO_LH_HIP     2   // 左后大腿
#define SERVO_LH_KNEE    3   // 左后小腿
#define SERVO_RF_HIP     4   // 右前大腿
#define SERVO_RF_KNEE    5   // 右前小腿
#define SERVO_RH_HIP     6   // 右后大腿
#define SERVO_RH_KNEE    7   // 右后小腿

/* ---- 舵机参数 ---- */
#define SERVO_FREQ          50      // 舵机标准 50Hz (20ms 周期)
#define SERVO_RESOLUTION    LEDC_TIMER_14_BIT  // 14-bit (0-16383)

// ★ 舵机型号选择 (编译期二选一, 切换后重新编译烧录)
//   角度→脉宽映射 (见 servo_driver.c angle_to_duty): 线性, 范围下限→500us, 上限→2500us。
//   SERVO_MODEL_180  0°~180°:   0°→500us  90°→1500us  180°→2500us
//   SERVO_MODEL_270  -35°~215°: -35°→500us 90°→1500us  215°→2500us  (90°对准舵机中位)
//   其余角度语义 (IK/poses/校准) 两种型号下均不变:
//   set_angle(90) 始终驱动舵机转到 90° 物理位置 (腿垂直)。
//   ⚠ 若舵机脉宽规格不是 500~2500μs 全程, 需同步调整 PULSE_MAX_US / PULSE_MIN_US。
#define SERVO_MODEL_180     0
#define SERVO_MODEL_270     1

// ★ 当前使用型号 (默认 270° 舵机测试中; 换 180° 舵机时改这里)
#define SERVO_MODEL         SERVO_MODEL_270

#if SERVO_MODEL == SERVO_MODEL_180
    #define SERVO_ANGLE_MIN     0       // 角度下限 → 500us
    #define SERVO_RANGE_DEG     180     // 行程跨度 (度)
#else
    #define SERVO_ANGLE_MIN     (-35)   // 角度下限 → 500us
    #define SERVO_RANGE_DEG     250     // 行程跨度 (度)
#endif
#define SERVO_ANGLE_MAX     (SERVO_ANGLE_MIN + SERVO_RANGE_DEG)  // 角度上限 → 2500us

#define PULSE_MIN_US        500     // 角度下限对应脉宽
#define PULSE_MAX_US        2500    // 角度上限对应脉宽 (500-2500μs)
#define PWM_PERIOD_US       20000   // 20ms = 50Hz

/* ---- 舵机配置结构体 ---- */
typedef struct {
    uint8_t  gpio;          // GPIO 引脚号
    uint8_t  channel;       // LEDC 通道号 (0-7)
    bool     initialized;   // 是否已初始化
    float    current_angle; // 当前角度
    float    target_angle;  // 目标角度
} servo_config_t;

/* ---- API ---- */

// 初始化单个舵机
void servo_init(uint8_t channel, uint8_t gpio);

// 设置单个舵机角度 (0°-180°)
void servo_set_angle(uint8_t channel, float angle_deg);

// 获取当前角度
float servo_get_angle(uint8_t channel);

// 批量同步更新多个舵机（先设置角度，最后统一更新）
void servo_group_begin(void);
void servo_group_add(uint8_t channel, float angle_deg);
void servo_group_commit(void);

// 自动初始化所有 8 路舵机（GPIO 映射在 servo_init_all() 中配置）
void servo_init_all(void);

// ★ 三点校准 (point: 0=0°点, 1=90°点, 2=180°点)
//   每个点存"命令该角度时舵机实际应转的角度", 中间分段线性插值补偿。
//   标定流程: set_angle(ch, A) 试角度看位置 → 调到对 → cal_point(ch, p, A) 存 NVS。
//   设置单点校准参考角 (90° 点, 兼容旧接口)
void servo_set_cal(uint8_t channel, float ref_angle_deg);

// 读取 90° 点校准参考角 (兼容旧接口)
float servo_get_cal(uint8_t channel);

// 设置指定校准点参考角 (point: 0/1/2 = 0°/90°/180°)
void servo_set_cal_point(uint8_t channel, uint8_t point, float ref_angle_deg);

// 读取指定校准点参考角
float servo_get_cal_point(uint8_t channel, uint8_t point);

// 从 NVS 加载校准数据（开机时调用）
void servo_load_cal(void);

// 紧急停止所有舵机（关闭 PWM）
void servo_emergency_stop(void);

#ifdef __cplusplus
}
#endif
