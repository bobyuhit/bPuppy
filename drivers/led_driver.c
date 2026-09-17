/*
 * bPuppy WS2812 电池电压指示灯 (V3.0 硬件: GPIO48)
 *
 * GPIO48 接一颗 WS2812 RGB LED, 用作电池电量指示:
 *   - 标定电压 ≥7.4V          蓝色   (2S 满电)
 *   - 标定电压 6.6~7.4V       蓝→紫→红 连续渐变
 *   - 标定电压 ≤6.6V          红色   (低压)
 *   - 标定电压 <6.4V          红色闪烁 (危险告警, ~0.5s 周期)
 *
 * 由 bpuppy_adc.init() 激活 (adc_driver.c 挂钩 led_batt_start()):
 * 初始化 ADC 后自动启动电池监控任务 (core 1, 100ms 轮询), 颜色随电压变化。
 *
 * 驱动: ESP-IDF **legacy RMT API** (driver/rmt.h) — 与 MicroPython 的
 *   machine_bitstream.c 完全同款 (clk_div=2 → 40MHz)。
 *   ⚠ 不能混用新 RMT 驱动 (rmt_tx.h): legacy 与 driver_ng 互斥,
 *     混用会触发 "CONFLICT! driver_ng is not allowed..." 断言重启。
 *   RMT TX 通道 0 — 避开 machine.bitstream 默认占用的 TX 通道 3, 两者共存。
 *   WS2812 数据序为 GRB, 单帧 24bit。
 *
 * 电压标定 (最小二乘拟合, 由 mpy_modules/batt.py 用实测点算出):
 *   显示值 v_disp = read_mv() × 6.1 / 1000
 *   标定值 v     = a × v_disp + b
 *   默认 a=1.0379, b=0.4660 (4 点实测)
 *
 * ⚠ 本文件是**唯一**的标定实现 —— Python 层不再各存一份常量:
 *   voltage.read_v()      → 直接返回 bpuppy_led.batt_v() (下面缓存的同一个值)
 *   batt.read_batt_v()    → 标定工具自己的检查, 不参与运行时读数
 *   所以 LED 颜色和 Python 读到的电压**不可能不一致** (同一条代码路径)。
 *
 * 标定值存 NVS (namespace "bpuppy_batt"), 掉电保留、固件升级不丢。
 * 改标定: batt.py 采集 + 拟合 → apply_to_board() → 写进 NVS, 不用重编译。
 *
 * MicroPython 接口:
 *   import bpuppy_led
 *   bpuppy_led.init()              # 初始化 RMT (幂等)
 *   bpuppy_led.set_color(r,g,b)    # 手动设色 0-255
 *   bpuppy_led.off()               # 熄灭
 *   bpuppy_led.batt(on=True)       # 手动启停电池监控任务
 *   bpuppy_led.batt_v()            # 最近一次已标定电池电压 (V); 监控未跑时返回 -1.0
 *   bpuppy_led.get_cal()           # → (a, b) 当前生效的标定系数
 *   bpuppy_led.set_cal(a, b)       # 设置并写入 NVS (掉电保留)
 *   bpuppy_led.reset_cal()         # 清除 NVS 标定, 恢复默认常量
 */

#include "py/runtime.h"
#include "py/obj.h"
#include "esp_err.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt.h"

static const char *TAG = "led";

/* ---- WS2812 / RMT 配置 ---- */
#define LED_GPIO          48
#define LED_BRIGHTNESS    0.10f  /* 全局亮度: 固定满亮的 10% (LED 太亮夜间刺眼) */

/* WS2812 时序 (40MHz tick, clk_div=2 → 25ns/bit): T0H=0.35us/T0L=0.80us/T1H=0.80us/T1L=0.45us */
#define T0H_TICKS         14
#define T0L_TICKS         32
#define T1H_TICKS         32
#define T1L_TICKS         18

/* ---- 电池电压阈值 (V, 标定后) ---- */
#define BATT_HIGH_V       7.4f   /* ≥ 此值: 蓝色 (满电) */
#define BATT_LOW_V        6.6f   /* ≤ 此值: 红色 (低压) */
#define BATT_BLINK_V      6.4f   /* < 此值: 红色闪烁 (危险) */

/* 标定默认值 (来自 batt.py 最小二乘拟合): 标定值 = a × 显示值 + b
 * 运行时实际用的是 s_cal_a / s_cal_b — 会被 NVS 里的值覆盖 (见 led_batt_load_cal) */
#define BATT_CAL_A        1.0379f
#define BATT_CAL_B        0.4660f
#define BATT_DIVIDER      6.1f   /* 51k/10k 分压换算 */

/* 标定值持久化 — namespace 与舵机几何 (GEOM_NVS_NS) 分开, 互不影响 */
#define BATT_CAL_NVS_NS   "bpuppy_batt"
#define BATT_CAL_SCALE    10000.0f  /* NVS 只存 i32: a/b ×10000 存入, 读回再除 → 保留 4 位小数 */

/* ---- 监控任务 ---- */
#define MONITOR_TICK_MS   100    /* 10Hz 轮询 */
#define BLINK_HALF_MS     250    /* 闪烁半周期 (全周期 ~0.5s) */
#define MONITOR_STACK     2048
#define MONITOR_PRIO      1
#define MONITOR_CORE      1      /* core 1: 与 VM/IMU 同核, 不占 core 0 步态实时性 */

/* 复用 adc_driver.c 的读数 (外部符号) */
extern int adc_read_mv(void);

static rmt_channel_t s_channel = RMT_CHANNEL_0;
static bool s_led_ready = false;
static TaskHandle_t s_monitor_task = NULL;

/* 运行时标定系数 — 初值是上面两个默认常量, led_batt_start() 时被 NVS 覆盖 */
static float s_cal_a = BATT_CAL_A;
static float s_cal_b = BATT_CAL_B;

/* 最近一次算出的已标定电压 (V); <0 = 无有效读数 (ADC 未就绪 / 监控未跑)
 * 由 batt_monitor_task 每 100ms 更新, bpuppy_led.batt_v() 直接读它 —— 只有这一条路径 */
static volatile float s_batt_v = -1.0f;

static void led_set_rgb(uint8_t r, uint8_t g, uint8_t b);   /* 前向声明 */

/* ================================================================
 * RMT 底层
 * ================================================================ */

void led_init(void)
{
    if (s_led_ready) return;

    /* legacy RMT (driver/rmt.h) — 与 MicroPython machine_bitstream.c 同款驱动 */
    rmt_config_t config = RMT_DEFAULT_CONFIG_TX(LED_GPIO, RMT_CHANNEL_0);
    config.clk_div = 2;              /* APB 80MHz/2 = 40MHz → 25ns/bit */
    ESP_ERROR_CHECK(rmt_config(&config));
    ESP_ERROR_CHECK(rmt_driver_install(config.channel, 0, 0));

    s_channel = config.channel;
    s_led_ready = true;
    led_set_rgb(0, 0, 0);
    ESP_LOGI(TAG, "WS2812 ready  GPIO=%d (RMT legacy chan 0)", LED_GPIO);
}

static void led_set_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (!s_led_ready) return;
    /* 全局亮度缩放 (LED_BRIGHTNESS, 固定 10%) — 调用方仍按 0-255 传值 */
    r = (uint8_t)(r * LED_BRIGHTNESS + 0.5f);
    g = (uint8_t)(g * LED_BRIGHTNESS + 0.5f);
    b = (uint8_t)(b * LED_BRIGHTNESS + 0.5f);
    uint8_t grb[3] = { g, r, b };        /* WS2812 数据序 GRB */

    rmt_item32_t items[24];
    for (int byte = 0; byte < 3; byte++) {
        for (int bit = 0; bit < 8; bit++) {
            rmt_item32_t *it = &items[byte * 8 + bit];
            if ((grb[byte] >> (7 - bit)) & 1) {   /* MSB first */
                it->level0 = 1; it->duration0 = T1H_TICKS;   /* bit 1: 长高短低 */
                it->level1 = 0; it->duration1 = T1L_TICKS;
            } else {
                it->level0 = 1; it->duration0 = T0H_TICKS;   /* bit 0: 短高长低 */
                it->level1 = 0; it->duration1 = T0L_TICKS;
            }
        }
    }
    ESP_ERROR_CHECK(rmt_write_items(s_channel, items, 24, true));   /* 阻塞等发完 */
    /* 帧间 >50us 复位由轮询间隔 (100ms) 保证 */
}

/* ================================================================
 * 电池监控任务
 * ================================================================ */

static void batt_monitor_task(void *arg)
{
    int blink_on = 1;
    uint32_t blink_ms = 0;

    for (;;) {
        int mv = adc_read_mv();

        if (mv < 0) {
            led_set_rgb(0, 0, 0);        /* ADC 未就绪 → 熄灭 */
            s_batt_v = -1.0f;
        } else {
            /* mv 已是 ADC 引脚电压 (mV) — adc_read_mv() 内部已 ×3100/4096 量程换算 */
            float v_disp = (float)mv * BATT_DIVIDER / 1000.0f;   /* ×6.1 分压比 → 电池显示电压 V */
            float v = s_cal_a * v_disp + s_cal_b;                /* 标定后电压 V (系数可被 NVS 覆盖) */
            s_batt_v = v;                                        /* 缓存: bpuppy_led.batt_v() 读这个 */

            if (v >= BATT_HIGH_V) {
                led_set_rgb(0, 0, 255);          /* 蓝: 满电 */
            } else if (v < BATT_BLINK_V) {
                blink_ms += MONITOR_TICK_MS;     /* 危险: 闪烁红 */
                if (blink_ms >= BLINK_HALF_MS) {
                    blink_ms = 0;
                    blink_on = !blink_on;
                }
                led_set_rgb(blink_on ? 255 : 0, 0, 0);
            } else {
                /* 渐变: 蓝(7.4V) → 紫 → 红(6.6V) */
                float t = (v - BATT_LOW_V) / (BATT_HIGH_V - BATT_LOW_V);
                if (t < 0.0f) t = 0.0f;
                if (t > 1.0f) t = 1.0f;
                led_set_rgb((uint8_t)(255 * (1.0f - t)), 0, (uint8_t)(255 * t));
            }
        }

        vTaskDelay(pdMS_TO_TICKS(MONITOR_TICK_MS));
    }
}

/* ================================================================
 * 标定值持久化 (NVS)
 * ================================================================ */

/* float → NVS 存的 i32 (×10000)。必须区分正负再取整: 负值 +0.5 会朝零截断 */
static int32_t cal_to_i32(float v)
{
    return (int32_t)(v * BATT_CAL_SCALE + (v >= 0.0f ? 0.5f : -0.5f));
}

/* 从 NVS 载入标定系数。没有记录 / 记录不完整 → 保持默认常量, 不算错误 */
static void led_batt_load_cal(void)
{
    nvs_handle_t h;
    if (nvs_open(BATT_CAL_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        ESP_LOGI(TAG, "batt cal: NVS 无记录, 用默认 a=%.4f b=%.4f", s_cal_a, s_cal_b);
        return;
    }
    int32_t ia = 0, ib = 0;
    bool ok_a = (nvs_get_i32(h, "a", &ia) == ESP_OK);
    bool ok_b = (nvs_get_i32(h, "b", &ib) == ESP_OK);
    nvs_close(h);

    if (ok_a && ok_b) {
        s_cal_a = (float)ia / BATT_CAL_SCALE;
        s_cal_b = (float)ib / BATT_CAL_SCALE;
        ESP_LOGI(TAG, "batt cal: NVS 载入 a=%.4f b=%.4f", s_cal_a, s_cal_b);
    } else {
        ESP_LOGW(TAG, "batt cal: NVS 记录不完整, 用默认 a=%.4f b=%.4f", s_cal_a, s_cal_b);
    }
}

/* 把 s_cal_a/s_cal_b 写进 NVS; 返回 true = 已持久化 */
static bool led_batt_save_cal(void)
{
    nvs_handle_t h;
    if (nvs_open(BATT_CAL_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ESP_LOGE(TAG, "batt cal: NVS 打开失败, 标定未持久化");
        return false;
    }
    esp_err_t e1 = nvs_set_i32(h, "a", cal_to_i32(s_cal_a));
    esp_err_t e2 = nvs_set_i32(h, "b", cal_to_i32(s_cal_b));
    esp_err_t e3 = nvs_commit(h);
    nvs_close(h);

    if (e1 != ESP_OK || e2 != ESP_OK || e3 != ESP_OK) {
        ESP_LOGE(TAG, "batt cal: NVS 写入失败 (set a=%d set b=%d commit=%d)", e1, e2, e3);
        return false;
    }
    ESP_LOGI(TAG, "batt cal: 已写入 NVS a=%.4f b=%.4f", s_cal_a, s_cal_b);
    return true;
}

void led_batt_start(void)
{
    if (s_monitor_task) return;
    led_init();
    led_batt_load_cal();   /* 起任务前载入, 首次读数就用对的系数 */
    xTaskCreatePinnedToCore(batt_monitor_task, "batt_led", MONITOR_STACK,
                            NULL, MONITOR_PRIO, &s_monitor_task, MONITOR_CORE);
    ESP_LOGI(TAG, "battery LED monitor started  (core %d, %dms)", MONITOR_CORE, MONITOR_TICK_MS);
}

void led_batt_stop(void)
{
    if (s_monitor_task) {
        vTaskDelete(s_monitor_task);
        s_monitor_task = NULL;
        s_batt_v = -1.0f;      /* 任务没了, 缓存值失效 — 否则 batt_v() 会返回陈旧读数 */
        led_set_rgb(0, 0, 0);
        ESP_LOGI(TAG, "battery LED monitor stopped");
    }
}

/* ================================================================
 * MicroPython 导出接口
 *
 *   import bpuppy_led
 *   bpuppy_led.init()              # 初始化 RMT
 *   bpuppy_led.set_color(r,g,b)    # 手动设色
 *   bpuppy_led.off()               # 熄灭
 *   bpuppy_led.batt(on=True)       # 启停电池监控
 *   bpuppy_led.batt_v()            # 已标定电池电压 (V); 监控未跑 → -1.0
 *   bpuppy_led.get_cal()           # → (a, b)
 *   bpuppy_led.set_cal(a, b)       # 设置 + 写 NVS (失败抛 OSError)
 *   bpuppy_led.reset_cal()         # 恢复默认标定并清 NVS
 * ================================================================ */

STATIC mp_obj_t mp_led_init(void) {
    led_init();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_led_init_obj, mp_led_init);

STATIC mp_obj_t mp_led_set_color(mp_obj_t r_obj, mp_obj_t g_obj, mp_obj_t b_obj) {
    led_init();
    uint8_t r = (uint8_t)mp_obj_get_int(r_obj);
    uint8_t g = (uint8_t)mp_obj_get_int(g_obj);
    uint8_t b = (uint8_t)mp_obj_get_int(b_obj);
    led_set_rgb(r, g, b);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_led_set_color_obj, mp_led_set_color);

STATIC mp_obj_t mp_led_off(void) {
    led_init();
    led_set_rgb(0, 0, 0);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_led_off_obj, mp_led_off);

STATIC mp_obj_t mp_led_batt(mp_obj_t on_obj) {
    if (mp_obj_is_true(on_obj)) {
        led_batt_start();
    } else {
        led_batt_stop();
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_led_batt_obj, mp_led_batt);

STATIC mp_obj_t mp_led_batt_v(void) {
    return mp_obj_new_float(s_batt_v);   /* 监控任务缓存的已标定电压; -1.0 = 无有效读数 */
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_led_batt_v_obj, mp_led_batt_v);

STATIC mp_obj_t mp_led_get_cal(void) {
    mp_obj_t items[2] = {
        mp_obj_new_float(s_cal_a),
        mp_obj_new_float(s_cal_b),
    };
    return mp_obj_new_tuple(2, items);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_led_get_cal_obj, mp_led_get_cal);

STATIC mp_obj_t mp_led_set_cal(mp_obj_t a_obj, mp_obj_t b_obj) {
    s_cal_a = mp_obj_get_float(a_obj);
    s_cal_b = mp_obj_get_float(b_obj);
    /* 系数已生效 (监控任务下一拍就用新的), 但没写进 NVS 就是没持久化 —— 必须让调用方知道 */
    if (!led_batt_save_cal()) {
        mp_raise_msg(&mp_type_OSError, MP_ERROR_TEXT("batt cal: NVS 写入失败 (系数本次生效, 重启后丢失)"));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_led_set_cal_obj, mp_led_set_cal);

STATIC mp_obj_t mp_led_reset_cal(void) {
    nvs_handle_t h;
    if (nvs_open(BATT_CAL_NVS_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_all(h);      /* 整个 namespace 都是本驱动的, 直接清空 */
        nvs_commit(h);
        nvs_close(h);
    }
    s_cal_a = BATT_CAL_A;
    s_cal_b = BATT_CAL_B;
    ESP_LOGI(TAG, "batt cal: 已恢复默认 a=%.4f b=%.4f", s_cal_a, s_cal_b);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_led_reset_cal_obj, mp_led_reset_cal);

// ---- 模块定义 ----
STATIC const mp_rom_map_elem_t bpuppy_led_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),  MP_ROM_QSTR(MP_QSTR_bpuppy_led) },
    { MP_ROM_QSTR(MP_QSTR_init),      MP_ROM_PTR(&mp_led_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_color), MP_ROM_PTR(&mp_led_set_color_obj) },
    { MP_ROM_QSTR(MP_QSTR_off),       MP_ROM_PTR(&mp_led_off_obj) },
    { MP_ROM_QSTR(MP_QSTR_batt),      MP_ROM_PTR(&mp_led_batt_obj) },
    { MP_ROM_QSTR(MP_QSTR_batt_v),    MP_ROM_PTR(&mp_led_batt_v_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_cal),   MP_ROM_PTR(&mp_led_get_cal_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_cal),   MP_ROM_PTR(&mp_led_set_cal_obj) },
    { MP_ROM_QSTR(MP_QSTR_reset_cal), MP_ROM_PTR(&mp_led_reset_cal_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_led_globals, bpuppy_led_globals_table);

const mp_obj_module_t bpuppy_led_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_led_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy_led, bpuppy_led_module);
