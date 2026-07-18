/*
 * bPuppy 舵机驱动 — 头文件
 *
 * 使用 ESP32-S3 LEDC PWM 控制器，支持 8 路舵机。
 * S3 LEDC: 8 通道, 全部 LOW_SPEED_MODE, 4 个定时器。
 * 标准舵机: 50Hz, 500-2500μs 脉宽, 对应 0°-180°
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
#define PULSE_MIN_US        500     // 0° 脉宽
#define PULSE_MAX_US        2500    // 180° 脉宽
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

// 设置舵机校准参考角（90° 时腿真正垂直的那个舵机值）
void servo_set_cal(uint8_t channel, float ref_angle_deg);

// 读取校准参考角（offset = ref - 90）
float servo_get_cal(uint8_t channel);

// 从 NVS 加载校准数据（开机时调用）
void servo_load_cal(void);

// 紧急停止所有舵机（关闭 PWM）
void servo_emergency_stop(void);

#ifdef __cplusplus
}
#endif
