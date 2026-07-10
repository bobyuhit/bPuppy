/*
 * bPuppy 舵机驱动 — 实现
 *
 * 使用 ESP32-S3 LEDC PWM 控制器驱动多路舵机。
 * S3 仅有 8 个通道 (LOW_SPEED_MODE)，无高低速之分。
 *
 * 提供:
 *  - 单舵机独立控制 (servo_set_angle)
 *  - 批量同步更新 (servo_group_begin/add/commit)
 *  - MicroPython 导出接口
 */

#include "servo_driver.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "servo";

/* ---- 全局舵机数组 (weak 共享) ---- */
__attribute__((weak)) servo_config_t g_servos[SERVO_MAX_CHANNELS];
__attribute__((weak)) bool g_servo_initialized = false;
__attribute__((weak)) float g_servo_cal[SERVO_MAX_CHANNELS];  // 每通道校准参考角

/* ---- 定时器 + 速度模式分配 (S3 统一 LS) ---- */
// ESP32-S3 LEDC: 8 通道, 全部 LOW_SPEED_MODE, 4 个定时器
// 舵机 1~8 (通道 0~7): LOW_SPEED_MODE + TIMER_0

#define SERVO_TIMER       LEDC_TIMER_0
#define SERVO_SPEED_MODE  LEDC_LOW_SPEED_MODE

static inline ledc_mode_t servo_speed_mode(uint8_t channel) {
    (void)channel;
    return LEDC_LOW_SPEED_MODE;
}
static inline ledc_timer_t servo_timer(uint8_t channel) {
    (void)channel;
    return SERVO_TIMER;
}
static inline ledc_channel_t servo_ledc_ch(uint8_t channel) {
    return (ledc_channel_t)(channel % SOC_LEDC_CHANNEL_NUM);
}

/* ---- 初始化所有舵机硬件 ---- */
static void servo_hardware_init(void)
{
    if (g_servo_initialized) return;

    ledc_timer_config_t timer_cfg = {
        .duty_resolution = SERVO_RESOLUTION,
        .freq_hz         = SERVO_FREQ,
        .speed_mode      = LEDC_LOW_SPEED_MODE,
        .timer_num       = SERVO_TIMER,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_cfg));

    g_servo_initialized = true;
    ESP_LOGI(TAG, "Servo timer initialized (50Hz, 14-bit, S3 LS mode)");
}

/* ---- 初始化单个舵机通道 ---- */
void servo_init(uint8_t channel, uint8_t gpio)
{
    if (channel >= SERVO_MAX_CHANNELS) {
        ESP_LOGE(TAG, "Channel %d out of range (max %d)", channel, SERVO_MAX_CHANNELS);
        return;
    }

    servo_hardware_init();

    servo_config_t *s = &g_servos[channel];
    s->gpio          = gpio;
    s->channel       = channel;
    s->initialized   = false;
    s->current_angle = 90.0f;
    s->target_angle  = 90.0f;

    // 配置 LEDC 通道 (S3: 统一 LOW_SPEED_MODE)
    ledc_channel_config_t ch_cfg = {
        .gpio_num   = gpio,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = servo_ledc_ch(channel),
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = servo_timer(channel),
        .duty       = 0,
        .hpoint     = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));

    s->initialized = true;
    ESP_LOGI(TAG, "Servo channel %d initialized on GPIO %d", channel, gpio);
}

/* ---- 角度 → PWM 占空比 ---- */
static uint32_t angle_to_duty(float angle_deg)
{
    // 钳位到 [0, 180]
    if (angle_deg < 0.0f) angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;

    // 线性映射: 0°→500us, 180°→2500us
    float pulse_us = PULSE_MIN_US + (angle_deg / 180.0f) * (PULSE_MAX_US - PULSE_MIN_US);

    // 脉宽 → LEDC duty (14-bit: 0-16383)
    uint32_t max_duty = (1 << 14) - 1;
    return (uint32_t)(pulse_us / (float)PWM_PERIOD_US * max_duty);
}

/* ---- 更新单个舵机 PWM 占空比 ---- */
static void servo_update_duty(uint8_t channel)
{
    servo_config_t *s = &g_servos[channel];
    if (!s->initialized) return;

    ledc_mode_t mode = servo_speed_mode(channel);
    ledc_channel_t ch = servo_ledc_ch(channel);
    uint32_t duty = angle_to_duty(s->current_angle);
    ledc_set_duty(mode, ch, duty);
    ledc_update_duty(mode, ch);
}

/* ---- 校准 (NVS 持久化) ---- */
#include "nvs_flash.h"
#include "nvs.h"
#define CAL_NVS_NAMESPACE "servo_cal"

static bool g_nvs_ready = false;

static void cal_init_nvs(void)
{
    if (g_nvs_ready) return;
    g_nvs_ready = true;
}

static void cal_save_to_nvs(uint8_t channel)
{
    nvs_handle_t handle;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) return;

    char key[8];
    snprintf(key, sizeof(key), "ch%d", channel);
    nvs_set_i32(handle, key, (int32_t)(g_servo_cal[channel] * 100.0f));
    nvs_commit(handle);
    nvs_close(handle);
}

static void cal_load_from_nvs(void)
{
    nvs_handle_t handle;
    if (nvs_open(CAL_NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) return;

    for (int ch = 0; ch < SERVO_MAX_CHANNELS; ch++) {
        char key[8];
        snprintf(key, sizeof(key), "ch%d", ch);
        int32_t val = 0;
        if (nvs_get_i32(handle, key, &val) == ESP_OK) {
            g_servo_cal[ch] = val / 100.0f;
        }
    }
    nvs_close(handle);
    ESP_LOGI(TAG, "Calibration loaded from NVS");
}

void servo_load_cal(void)
{
    cal_init_nvs();
    cal_load_from_nvs();
}

void servo_set_cal(uint8_t channel, float ref_angle_deg)
{
    if (channel >= SERVO_MAX_CHANNELS) return;
    cal_init_nvs();
    // offset = ref - 90: cal(ch,95) → offset=+5 → set_angle(90) 发 95° → 腿垂直
    g_servo_cal[channel] = ref_angle_deg - 90.0f;
    cal_save_to_nvs(channel);
    ESP_LOGI(TAG, "ch%d cal: %.1f (offset=%+.1f)", channel, ref_angle_deg, g_servo_cal[channel]);

    // 立即驱动舵机到参考角 (不经过校准), 让用户当场验证
    if (g_servos[channel].initialized) {
        g_servos[channel].current_angle = ref_angle_deg;
        servo_update_duty(channel);
    }
}

// 应用校准：把"理论角度"映射到"实际舵机值"
static float servo_apply_cal(uint8_t channel, float angle_deg)
{
    return angle_deg + g_servo_cal[channel];
}

/* ---- 设置舵机角度（立即生效）---- */
void servo_set_angle(uint8_t channel, float angle_deg)
{
    if (channel >= SERVO_MAX_CHANNELS || !g_servos[channel].initialized) {
        return;
    }

    float calibrated = servo_apply_cal(channel, angle_deg);
    g_servos[channel].current_angle = calibrated;
    g_servos[channel].target_angle  = calibrated;
    servo_update_duty(channel);
}

/* ---- 获取当前角度 ---- */
float servo_get_angle(uint8_t channel)
{
    if (channel >= SERVO_MAX_CHANNELS || !g_servos[channel].initialized) {
        return -1.0f;
    }
    return g_servos[channel].current_angle;
}

/* ---- 批量同步更新 ---- */
__attribute__((weak)) uint8_t  g_group_channels[SERVO_MAX_CHANNELS];
__attribute__((weak)) float    g_group_angles[SERVO_MAX_CHANNELS];
__attribute__((weak)) int      g_group_count = 0;

void servo_group_begin(void)
{
    g_group_count = 0;
}

void servo_group_add(uint8_t channel, float angle_deg)
{
    if (channel >= SERVO_MAX_CHANNELS || !g_servos[channel].initialized) {
        return;
    }
    if (g_group_count >= SERVO_MAX_CHANNELS) {
        return;
    }

    g_group_channels[g_group_count] = channel;
    g_group_angles[g_group_count]   = servo_apply_cal(channel, angle_deg);
    g_group_count++;
}

void servo_group_commit(void)
{
    // 先设置所有 duty
    for (int i = 0; i < g_group_count; i++) {
        uint8_t ch = g_group_channels[i];
        g_servos[ch].current_angle = g_group_angles[i];
        g_servos[ch].target_angle  = g_group_angles[i];

        uint32_t duty = angle_to_duty(g_group_angles[i]);
        ledc_set_duty(servo_speed_mode(ch), servo_ledc_ch(ch), duty);
    }

    // 统一更新
    for (int i = 0; i < g_group_count; i++) {
        uint8_t ch = g_group_channels[i];
        ledc_update_duty(servo_speed_mode(ch), servo_ledc_ch(ch));
    }
}

/* ---- 自动初始化所有舵机（GPIO 待用户指定）---- */
void servo_init_all(void)
{
    // TODO: 用户根据 PCB 设计指定 GPIO 引脚
    servo_init(0, 1);    // LF_HIP
    servo_init(1, 2);    // LF_KNEE
    servo_init(2, 47);   // LH_HIP
    servo_init(3, 21);   // LH_KNEE
    servo_init(4, 42);   // RF_HIP
    servo_init(5, 41);   // RF_KNEE
    servo_init(6, 45);   // RH_HIP
    servo_init(7, 48);   // RH_KNEE
    ESP_LOGI(TAG, "All 8 servos initialized");
}

/* ---- 紧急停止 ---- */
void servo_emergency_stop(void)
{
    for (int i = 0; i < SERVO_MAX_CHANNELS; i++) {
        if (g_servos[i].initialized) {
            ledc_stop(servo_speed_mode(i), servo_ledc_ch(i), 0);
        }
    }
    ESP_LOGW(TAG, "All servos stopped (emergency)");
}

/* ================================================================
 * MicroPython 导出接口
 *
 * 在 Python 中使用:
 *   import bpuppy_servo
 *   bpuppy_servo.init(0, 13)         # 通道0, GPIO13
 *   bpuppy_servo.set_angle(0, 90)    # 通道0 → 90°
 *   bpuppy_servo.get_angle(0)        # 读取角度
 *   bpuppy_servo.stop()              # 紧急停止
 * ================================================================ */

// ---- servo_init(channel, gpio) ----
STATIC mp_obj_t mp_servo_init(mp_obj_t ch_obj, mp_obj_t gpio_obj) {
    int ch = mp_obj_get_int(ch_obj);
    int gpio = mp_obj_get_int(gpio_obj);
    servo_init((uint8_t)ch, (uint8_t)gpio);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_servo_init_obj, mp_servo_init);

// ---- servo_set_angle(channel, angle_deg) ----
STATIC mp_obj_t mp_servo_set_angle(mp_obj_t ch_obj, mp_obj_t angle_obj) {
    int ch = mp_obj_get_int(ch_obj);
    float angle = mp_obj_get_float(angle_obj);
    servo_set_angle((uint8_t)ch, angle);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_servo_set_angle_obj, mp_servo_set_angle);

// ---- servo_get_angle(channel) → float ----
STATIC mp_obj_t mp_servo_get_angle(mp_obj_t ch_obj) {
    int ch = mp_obj_get_int(ch_obj);
    float angle = servo_get_angle((uint8_t)ch);
    return mp_obj_new_float(angle);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_servo_get_angle_obj, mp_servo_get_angle);

// ---- servo_init_all() ----
STATIC mp_obj_t mp_servo_init_all(void) {
    servo_init_all();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_servo_init_all_obj, mp_servo_init_all);

// ---- servo_stop() ----
STATIC mp_obj_t mp_servo_stop(void) {
    servo_emergency_stop();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_servo_stop_obj, mp_servo_stop);

// ---- 校准函数 MPY 包装 ----
#define MAKE_SERVO_CAL(ch, name) \
    STATIC mp_obj_t mp_cal_##name(mp_obj_t deg_obj) { \
        servo_set_cal((ch), mp_obj_get_float(deg_obj)); \
        return mp_const_none; \
    } \
    STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_cal_##name##_obj, mp_cal_##name);

MAKE_SERVO_CAL(SERVO_LF_HIP,  LF_HIP)
MAKE_SERVO_CAL(SERVO_LF_KNEE, LF_KNEE)
MAKE_SERVO_CAL(SERVO_LH_HIP,  LH_HIP)
MAKE_SERVO_CAL(SERVO_LH_KNEE, LH_KNEE)
MAKE_SERVO_CAL(SERVO_RF_HIP,  RF_HIP)
MAKE_SERVO_CAL(SERVO_RF_KNEE, RF_KNEE)
MAKE_SERVO_CAL(SERVO_RH_HIP,  RH_HIP)
MAKE_SERVO_CAL(SERVO_RH_KNEE, RH_KNEE)

// ---- load_cal ----
STATIC mp_obj_t mp_servo_load_cal(void) {
    servo_load_cal();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_servo_load_cal_obj, mp_servo_load_cal);

// ---- cal(channel, ref_deg) 统一接口 ----
STATIC mp_obj_t mp_servo_cal(mp_obj_t ch_obj, mp_obj_t deg_obj) {
    int ch = mp_obj_get_int(ch_obj);
    float deg = mp_obj_get_float(deg_obj);
    servo_set_cal((uint8_t)ch, deg);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_servo_cal_obj, mp_servo_cal);

// ---- 模块定义 ----
STATIC const mp_rom_map_elem_t bpuppy_servo_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_bpuppy_servo) },
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&mp_servo_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_init_all),    MP_ROM_PTR(&mp_servo_init_all_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_angle),   MP_ROM_PTR(&mp_servo_set_angle_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_angle),   MP_ROM_PTR(&mp_servo_get_angle_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),        MP_ROM_PTR(&mp_servo_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_load_cal),    MP_ROM_PTR(&mp_servo_load_cal_obj) },
    { MP_ROM_QSTR(MP_QSTR_cal),        MP_ROM_PTR(&mp_servo_cal_obj) },
    // 校准 (兼容旧接口)
    { MP_ROM_QSTR(MP_QSTR_cal_LF_HIP),  MP_ROM_PTR(&mp_cal_LF_HIP_obj) },
    { MP_ROM_QSTR(MP_QSTR_cal_LF_KNEE), MP_ROM_PTR(&mp_cal_LF_KNEE_obj) },
    { MP_ROM_QSTR(MP_QSTR_cal_LH_HIP),  MP_ROM_PTR(&mp_cal_LH_HIP_obj) },
    { MP_ROM_QSTR(MP_QSTR_cal_LH_KNEE), MP_ROM_PTR(&mp_cal_LH_KNEE_obj) },
    { MP_ROM_QSTR(MP_QSTR_cal_RF_HIP),  MP_ROM_PTR(&mp_cal_RF_HIP_obj) },
    { MP_ROM_QSTR(MP_QSTR_cal_RF_KNEE), MP_ROM_PTR(&mp_cal_RF_KNEE_obj) },
    { MP_ROM_QSTR(MP_QSTR_cal_RH_HIP),  MP_ROM_PTR(&mp_cal_RH_HIP_obj) },
    { MP_ROM_QSTR(MP_QSTR_cal_RH_KNEE), MP_ROM_PTR(&mp_cal_RH_KNEE_obj) },
    // 舵机命名常量
    { MP_ROM_QSTR(MP_QSTR_LF_HIP),      MP_ROM_INT(SERVO_LF_HIP) },
    { MP_ROM_QSTR(MP_QSTR_LF_KNEE),     MP_ROM_INT(SERVO_LF_KNEE) },
    { MP_ROM_QSTR(MP_QSTR_LH_HIP),      MP_ROM_INT(SERVO_LH_HIP) },
    { MP_ROM_QSTR(MP_QSTR_LH_KNEE),     MP_ROM_INT(SERVO_LH_KNEE) },
    { MP_ROM_QSTR(MP_QSTR_RF_HIP),      MP_ROM_INT(SERVO_RF_HIP) },
    { MP_ROM_QSTR(MP_QSTR_RF_KNEE),     MP_ROM_INT(SERVO_RF_KNEE) },
    { MP_ROM_QSTR(MP_QSTR_RH_HIP),      MP_ROM_INT(SERVO_RH_HIP) },
    { MP_ROM_QSTR(MP_QSTR_RH_KNEE),     MP_ROM_INT(SERVO_RH_KNEE) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_servo_globals, bpuppy_servo_globals_table);

const mp_obj_module_t bpuppy_servo_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_servo_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy_servo, bpuppy_servo_module);
