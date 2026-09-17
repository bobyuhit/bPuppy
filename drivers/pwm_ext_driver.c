/*
 * bPuppy PWM_EXT 扩展舵机驱动 — MCPWM
 *
 * 用途: 驱动 PWM_EXT1/2/3 扩展脚上的舵机 (主舵机之外的额外舵机)。
 *
 * ── 为什么用 MCPWM 而不是 LEDC ──────────────────────────────────
 * ESP32-S3 的 LEDC **总共只有 8 个通道**且不支持高速模式
 * (soc_caps.h: SOC_LEDC_CHANNEL_NUM=8, 无 SOC_LEDC_SUPPORT_HS_MODE),
 * 已被 8 个主舵机全部占用 (servo_driver.c, LOW_SPEED + TIMER_0, ch 0~7)。
 * 所以第 9 个舵机在 LEDC 上**无路可走**。
 * MCPWM 完全空闲: 2 组 × 3 运算符 = 6 路独立输出。
 *
 * ⚠ 切勿用 MicroPython 的 machine.PWM 驱动扩展舵机 —— 它走 LEDC,
 *   且账本是 LEDC_SPEED_MODE_MAX × LEDC_CHANNEL_MAX = 16 (硬件实际只有 8),
 *   又不知道 servo_driver.c 已直接占用 ch 0~7 → **会静默顶掉某个主舵机**。
 *
 * ── 输出 ────────────────────────────────────────────────────────
 * 标准舵机信号: 50Hz (周期 20ms), 脉宽 500~2500us。
 * 定时器 1MHz 分辨率 → 1 tick = 1us, comparator 值直接就是脉宽微秒数。
 *
 * MicroPython 接口:
 *   import bpuppy_pwm_ext
 *   bpuppy_pwm_ext.init(ch, gpio)        # ch 0~5 (PWM_EXT1=0 / EXT2=1 / EXT3=2 由接线决定)
 *   bpuppy_pwm_ext.set_pulse_us(ch, us)  # 直接给脉宽 500~2500
 *   bpuppy_pwm_ext.set_angle(ch, deg)    # 角度, 映射与主舵机一致 (270°: -35~215)
 *   bpuppy_pwm_ext.deinit(ch)            # 停脉冲并释放
 *   bpuppy_pwm_ext.status(ch)            # → (in_use, gpio, pulse_us)
 */

#include "py/runtime.h"
#include "py/obj.h"
#include "esp_err.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"

static const char *TAG = "pwm_ext";

/* 2 组 × 每组 3 个运算符 = 6 路 (每组也有 3 个定时器, 够每路一个) */
#define PWM_EXT_CH_MAX        6
#define PWM_EXT_OPS_PER_GROUP 3

/* 1MHz → 1 tick = 1us;  20000 tick = 20ms = 50Hz (舵机标准) */
#define PWM_EXT_RESOLUTION_HZ 1000000
#define PWM_EXT_PERIOD_TICKS  20000

#define PWM_EXT_PULSE_MIN     500
#define PWM_EXT_PULSE_MAX     2500
#define PWM_EXT_PULSE_MID     1500

/* 角度映射 — 与 servo_driver.c 保持一致 (270° 舵机: -35°→500us, 215°→2500us) */
#define PWM_EXT_ANGLE_MIN     (-35.0f)
#define PWM_EXT_ANGLE_RANGE   (250.0f)

typedef struct {
    bool used;
    int  gpio;
    uint32_t pulse_us;
    mcpwm_timer_handle_t timer;
    mcpwm_oper_handle_t  oper;
    mcpwm_cmpr_handle_t  cmpr;
    mcpwm_gen_handle_t   gen;
} pwm_ext_slot_t;

static pwm_ext_slot_t s_slots[PWM_EXT_CH_MAX];

/* pwm_ext_init 的 fail 路径要调它, 而它定义在后面 → 前向声明 */
esp_err_t pwm_ext_deinit(int ch);

/* ---- 内部: 参数检查 ---- */
static bool pwm_ext_ch_valid(int ch)
{
    return (ch >= 0 && ch < PWM_EXT_CH_MAX);
}

/* 角度 → 脉宽 (us), 与 servo_driver.c 的 angle_to_duty 同一套线性映射 */
static int pwm_ext_angle_to_us(float deg)
{
    int us = (int)(PWM_EXT_PULSE_MIN +
                   (deg - PWM_EXT_ANGLE_MIN) / PWM_EXT_ANGLE_RANGE *
                   (PWM_EXT_PULSE_MAX - PWM_EXT_PULSE_MIN) + 0.5f);
    if (us < PWM_EXT_PULSE_MIN) us = PWM_EXT_PULSE_MIN;
    if (us > PWM_EXT_PULSE_MAX) us = PWM_EXT_PULSE_MAX;
    return us;
}

/* 脉宽 (us) → 角度 (deg), 上面那个映射的逆。
 * 只此一份 —— Python 层要读角度就调 get_angle(), 别自己再写一遍换算。 */
static float pwm_ext_us_to_angle(int us)
{
    return (us - PWM_EXT_PULSE_MIN) * PWM_EXT_ANGLE_RANGE /
           (PWM_EXT_PULSE_MAX - PWM_EXT_PULSE_MIN) + PWM_EXT_ANGLE_MIN;
}

/* ================================================================
 * 底层
 * ================================================================ */

esp_err_t pwm_ext_init(int ch, int gpio)
{
    if (!pwm_ext_ch_valid(ch)) {
        ESP_LOGE(TAG, "通道号越界: %d (合法 0~%d)", ch, PWM_EXT_CH_MAX - 1);
        return ESP_ERR_INVALID_ARG;
    }
    if (s_slots[ch].used) return ESP_OK;   /* 幂等: 重复 init 直接返回 */

    pwm_ext_slot_t *s = &s_slots[ch];
    const int group = ch / PWM_EXT_OPS_PER_GROUP;

    mcpwm_timer_config_t tcfg = {
        .group_id      = group,
        .clk_src       = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = PWM_EXT_RESOLUTION_HZ,
        .count_mode    = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks  = PWM_EXT_PERIOD_TICKS,
    };
    esp_err_t e = mcpwm_new_timer(&tcfg, &s->timer);
    if (e != ESP_OK) { ESP_LOGE(TAG, "ch%d new_timer: %d", ch, e); return e; }

    mcpwm_operator_config_t ocfg = { .group_id = group };
    e = mcpwm_new_operator(&ocfg, &s->oper);
    if (e != ESP_OK) { ESP_LOGE(TAG, "ch%d new_operator: %d", ch, e); goto fail; }
    e = mcpwm_operator_connect_timer(s->oper, s->timer);
    if (e != ESP_OK) { ESP_LOGE(TAG, "ch%d connect_timer: %d", ch, e); goto fail; }

    mcpwm_comparator_config_t ccfg = { 0 };
    e = mcpwm_new_comparator(s->oper, &ccfg, &s->cmpr);
    if (e != ESP_OK) { ESP_LOGE(TAG, "ch%d new_comparator: %d", ch, e); goto fail; }

    mcpwm_generator_config_t gcfg = { .gen_gpio_num = gpio };
    e = mcpwm_new_generator(s->oper, &gcfg, &s->gen);
    if (e != ESP_OK) { ESP_LOGE(TAG, "ch%d new_generator(GPIO%d): %d", ch, gpio, e); goto fail; }

    /* 定时器归零 → 拉高 (脉冲起点)。
     * 注意 EMPTY 事件在上电时会先触发一次, 所以初始电平由这里决定, 不需要额外设。 */
    e = mcpwm_generator_set_action_on_timer_event(s->gen,
            (mcpwm_gen_timer_event_action_t) {
                .direction = MCPWM_TIMER_DIRECTION_UP,
                .event     = MCPWM_TIMER_EVENT_EMPTY,
                .action    = MCPWM_GEN_ACTION_HIGH,
            });
    if (e != ESP_OK) { ESP_LOGE(TAG, "ch%d set_action(timer): %d", ch, e); goto fail; }

    /* 比较器命中 → 拉低 (脉冲终点) */
    e = mcpwm_generator_set_action_on_compare_event(s->gen,
            (mcpwm_gen_compare_event_action_t) {
                .direction  = MCPWM_TIMER_DIRECTION_UP,
                .comparator = s->cmpr,
                .action     = MCPWM_GEN_ACTION_LOW,
            });
    if (e != ESP_OK) { ESP_LOGE(TAG, "ch%d set_action(compare): %d", ch, e); goto fail; }

    e = mcpwm_comparator_set_compare_value(s->cmpr, PWM_EXT_PULSE_MID);   /* 先给中位, 别让舵机乱冲 */
    if (e != ESP_OK) { ESP_LOGE(TAG, "ch%d set_compare: %d", ch, e); goto fail; }

    e = mcpwm_timer_enable(s->timer);
    if (e != ESP_OK) { ESP_LOGE(TAG, "ch%d timer_enable: %d", ch, e); goto fail; }

    e = mcpwm_timer_start_stop(s->timer, MCPWM_TIMER_START_NO_STOP);
    if (e != ESP_OK) { ESP_LOGE(TAG, "ch%d timer_start: %d", ch, e); goto fail; }

    s->used     = true;
    s->gpio     = gpio;
    s->pulse_us = PWM_EXT_PULSE_MID;
    ESP_LOGI(TAG, "PWM_EXT ch%d @GPIO%d 就绪 (50Hz, %d~%dus)",
             ch, gpio, PWM_EXT_PULSE_MIN, PWM_EXT_PULSE_MAX);
    return ESP_OK;

fail:
    /* 半初始化状态比不初始化更糟 (定时器可能已在跑而引脚悬空), 就地拆掉 */
    pwm_ext_deinit(ch);
    return e;
}

esp_err_t pwm_ext_set_pulse_us(int ch, int us)
{
    if (!pwm_ext_ch_valid(ch)) return ESP_ERR_INVALID_ARG;
    if (!s_slots[ch].used)     return ESP_ERR_INVALID_STATE;

    if (us < PWM_EXT_PULSE_MIN) us = PWM_EXT_PULSE_MIN;
    if (us > PWM_EXT_PULSE_MAX) us = PWM_EXT_PULSE_MAX;

    esp_err_t e = mcpwm_comparator_set_compare_value(s_slots[ch].cmpr, (uint32_t)us);
    if (e == ESP_OK) s_slots[ch].pulse_us = (uint32_t)us;
    return e;
}

esp_err_t pwm_ext_get_pulse_us(int ch, int *out_us)
{
    if (!pwm_ext_ch_valid(ch) || !out_us) return ESP_ERR_INVALID_ARG;
    if (!s_slots[ch].used)                return ESP_ERR_INVALID_STATE;
    *out_us = (int)s_slots[ch].pulse_us;
    return ESP_OK;
}

esp_err_t pwm_ext_get_angle(int ch, float *out_deg)
{
    if (!out_deg) return ESP_ERR_INVALID_ARG;
    int us;
    esp_err_t e = pwm_ext_get_pulse_us(ch, &us);
    if (e != ESP_OK) return e;
    *out_deg = pwm_ext_us_to_angle(us);
    return ESP_OK;
}

esp_err_t pwm_ext_deinit(int ch)
{
    if (!pwm_ext_ch_valid(ch)) return ESP_ERR_INVALID_ARG;

    pwm_ext_slot_t *s = &s_slots[ch];
    /* 逆序拆: 先停定时器, 再删生成器/比较器/运算符/定时器。
     * 每步都判空 —— init 的 fail 路径会在只建了一半时调进来。 */
    if (s->timer) {
        mcpwm_timer_start_stop(s->timer, MCPWM_TIMER_STOP_EMPTY);
        mcpwm_timer_disable(s->timer);
    }
    if (s->gen)   { mcpwm_del_generator(s->gen);   s->gen   = NULL; }
    if (s->cmpr)  { mcpwm_del_comparator(s->cmpr); s->cmpr  = NULL; }
    if (s->oper)  { mcpwm_del_operator(s->oper);   s->oper  = NULL; }
    if (s->timer) { mcpwm_del_timer(s->timer);     s->timer = NULL; }

    if (s->used) ESP_LOGI(TAG, "PWM_EXT ch%d @GPIO%d 已释放", ch, s->gpio);
    s->used     = false;
    s->gpio     = -1;
    s->pulse_us = 0;
    return ESP_OK;
}

/* ================================================================
 * MicroPython 导出
 * ================================================================ */

STATIC mp_obj_t mp_pwm_ext_init(mp_obj_t ch_obj, mp_obj_t gpio_obj) {
    int ch = mp_obj_get_int(ch_obj);
    int gpio = mp_obj_get_int(gpio_obj);
    if (!pwm_ext_ch_valid(ch)) {
        mp_raise_ValueError(MP_ERROR_TEXT("通道号越界 (合法 0~5)"));
    }
    esp_err_t e = pwm_ext_init(ch, gpio);
    if (e != ESP_OK) {
        /* 不能 mp_raise_OSError(e): 它要的是 POSIX errno, esp_err_t 不是。
         * 具体错误码已由 pwm_ext_init 里的 ESP_LOGE 打进串口日志。 */
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("PWM_EXT init 失败 (错误码见串口日志)"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_pwm_ext_init_obj, mp_pwm_ext_init);

STATIC mp_obj_t mp_pwm_ext_set_pulse_us(mp_obj_t ch_obj, mp_obj_t us_obj) {
    int ch = mp_obj_get_int(ch_obj);
    int us = mp_obj_get_int(us_obj);
    if (!pwm_ext_ch_valid(ch)) {
        mp_raise_ValueError(MP_ERROR_TEXT("通道号越界 (合法 0~5)"));
    }
    if (!s_slots[ch].used) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("通道未 init"));
    }
    pwm_ext_set_pulse_us(ch, us);   /* 超范围静默钳位到 500~2500 */
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_pwm_ext_set_pulse_us_obj, mp_pwm_ext_set_pulse_us);

STATIC mp_obj_t mp_pwm_ext_set_angle(mp_obj_t ch_obj, mp_obj_t deg_obj) {
    int ch = mp_obj_get_int(ch_obj);
    float deg = mp_obj_get_float(deg_obj);
    if (!pwm_ext_ch_valid(ch)) {
        mp_raise_ValueError(MP_ERROR_TEXT("通道号越界 (合法 0~5)"));
    }
    if (!s_slots[ch].used) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("通道未 init"));
    }
    pwm_ext_set_pulse_us(ch, pwm_ext_angle_to_us(deg));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_pwm_ext_set_angle_obj, mp_pwm_ext_set_angle);

STATIC mp_obj_t mp_pwm_ext_deinit(mp_obj_t ch_obj) {
    int ch = mp_obj_get_int(ch_obj);
    if (!pwm_ext_ch_valid(ch)) {
        mp_raise_ValueError(MP_ERROR_TEXT("通道号越界 (合法 0~5)"));
    }
    pwm_ext_deinit(ch);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_pwm_ext_deinit_obj, mp_pwm_ext_deinit);

STATIC mp_obj_t mp_pwm_ext_status(mp_obj_t ch_obj) {
    int ch = mp_obj_get_int(ch_obj);
    if (!pwm_ext_ch_valid(ch)) {
        mp_raise_ValueError(MP_ERROR_TEXT("通道号越界 (合法 0~5)"));
    }
    pwm_ext_slot_t *s = &s_slots[ch];
    mp_obj_t items[3] = {
        mp_obj_new_bool(s->used),
        mp_obj_new_int(s->used ? s->gpio : -1),
        mp_obj_new_int(s->used ? (int)s->pulse_us : 0),
    };
    return mp_obj_new_tuple(3, items);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_pwm_ext_status_obj, mp_pwm_ext_status);

STATIC mp_obj_t mp_pwm_ext_get_pulse_us(mp_obj_t ch_obj) {
    int ch = mp_obj_get_int(ch_obj);
    if (!pwm_ext_ch_valid(ch)) {
        mp_raise_ValueError(MP_ERROR_TEXT("通道号越界 (合法 0~5)"));
    }
    int us;
    if (pwm_ext_get_pulse_us(ch, &us) != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("通道未 init"));
    }
    return mp_obj_new_int(us);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_pwm_ext_get_pulse_us_obj, mp_pwm_ext_get_pulse_us);

STATIC mp_obj_t mp_pwm_ext_get_angle(mp_obj_t ch_obj) {
    int ch = mp_obj_get_int(ch_obj);
    if (!pwm_ext_ch_valid(ch)) {
        mp_raise_ValueError(MP_ERROR_TEXT("通道号越界 (合法 0~5)"));
    }
    float deg;
    if (pwm_ext_get_angle(ch, &deg) != ESP_OK) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("通道未 init"));
    }
    return mp_obj_new_float(deg);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_pwm_ext_get_angle_obj, mp_pwm_ext_get_angle);

// ---- 模块定义 ----
STATIC const mp_rom_map_elem_t bpuppy_pwm_ext_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),     MP_ROM_QSTR(MP_QSTR_bpuppy_pwm_ext) },
    { MP_ROM_QSTR(MP_QSTR_init),         MP_ROM_PTR(&mp_pwm_ext_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_pulse_us), MP_ROM_PTR(&mp_pwm_ext_set_pulse_us_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_angle),    MP_ROM_PTR(&mp_pwm_ext_set_angle_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),       MP_ROM_PTR(&mp_pwm_ext_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_status),       MP_ROM_PTR(&mp_pwm_ext_status_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_pulse_us), MP_ROM_PTR(&mp_pwm_ext_get_pulse_us_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_angle),    MP_ROM_PTR(&mp_pwm_ext_get_angle_obj) },
    // 常量
    { MP_ROM_QSTR(MP_QSTR_CH_MAX),       MP_ROM_INT(PWM_EXT_CH_MAX) },
    { MP_ROM_QSTR(MP_QSTR_PULSE_MIN),    MP_ROM_INT(PWM_EXT_PULSE_MIN) },
    { MP_ROM_QSTR(MP_QSTR_PULSE_MAX),    MP_ROM_INT(PWM_EXT_PULSE_MAX) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_pwm_ext_globals, bpuppy_pwm_ext_globals_table);

const mp_obj_module_t bpuppy_pwm_ext_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_pwm_ext_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy_pwm_ext, bpuppy_pwm_ext_module);
