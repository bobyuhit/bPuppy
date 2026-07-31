/*
 * bPuppy ADC 电压测量 — GPIO 38 (ADC1_CH2)
 *
 * MicroPython 接口:
 *   import bpuppy_adc
 *   bpuppy_adc.init()            # GPIO 38, 11dB 衰减 (~0-3.1V)
 *   mv = bpuppy_adc.read_mv()    # 读取电压 (毫伏)
 *   raw = bpuppy_adc.read_raw()  # 读取原始值 (0-4095)
 *
 * 电池测量: 7.4V LiPo → 分压 (33k+10k) → ADC ≈ 1.72V@7.4V → read_mv()×4.3
 *
 * ⚠ 必须用 legacy ADC API (adc1_*): MicroPython 的 machine.ADC 使用 legacy
 *   driver, ESP-IDF 5.x 中 legacy 与 driver_ng (adc_oneshot) 互斥,
 *   混用会触发 "CONFLICT! driver_ng is not allowed..." 断言重启。
 */

#include "py/runtime.h"
#include "py/obj.h"
#include "driver/adc.h"
#include "esp_log.h"

static const char *TAG = "adc";

#define ADC_DEFAULT_CHANNEL  ADC1_CHANNEL_2   // ADC1_CH2 (GPIO 38)
#define ADC_DEFAULT_ATTEN    ADC_ATTEN_DB_11  // ~0-3.1V

static bool g_adc_ready = false;

void adc_init(void)
{
    if (g_adc_ready) return;

    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_DEFAULT_CHANNEL, ADC_DEFAULT_ATTEN));

    g_adc_ready = true;
    ESP_LOGI(TAG, "ADC ready  GPIO=38  ADC1_CH2  atten=11dB (legacy)");
}

int adc_read_raw(void)
{
    if (!g_adc_ready) return -1;
    return adc1_get_raw(ADC_DEFAULT_CHANNEL);   // 失败返回 -1
}

int adc_read_mv(void)
{
    if (!g_adc_ready) return -1;
    int raw = adc1_get_raw(ADC_DEFAULT_CHANNEL);
    if (raw < 0) return -1;
    // 11dB 衰减: ~0-3100mV → 0-4095 (12-bit)
    return (int)((int64_t)raw * 3100 / 4096);
}

void adc_stop(void)
{
    g_adc_ready = false;
    ESP_LOGI(TAG, "ADC stopped (read 返回 -1, 可重新 init)");
}


/* ================================================================
 * MicroPython 导出接口
 *
 *   import bpuppy_adc
 *   bpuppy_adc.init()        # GPIO 38, ADC1_CH2
 *   mv = bpuppy_adc.read_mv()
 *   raw = bpuppy_adc.read_raw()
 * ================================================================ */

STATIC mp_obj_t mp_adc_init(void) {
    adc_init();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_adc_init_obj, mp_adc_init);

STATIC mp_obj_t mp_adc_read_mv(void) {
    return mp_obj_new_int(adc_read_mv());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_adc_read_mv_obj, mp_adc_read_mv);

STATIC mp_obj_t mp_adc_read_raw(void) {
    return mp_obj_new_int(adc_read_raw());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_adc_read_raw_obj, mp_adc_read_raw);

STATIC mp_obj_t mp_adc_stop(void) {
    adc_stop();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_adc_stop_obj, mp_adc_stop);

// ---- 模块定义 ----
STATIC const mp_rom_map_elem_t bpuppy_adc_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),  MP_ROM_QSTR(MP_QSTR_bpuppy_adc) },
    { MP_ROM_QSTR(MP_QSTR_init),      MP_ROM_PTR(&mp_adc_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_mv),   MP_ROM_PTR(&mp_adc_read_mv_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_raw),  MP_ROM_PTR(&mp_adc_read_raw_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),      MP_ROM_PTR(&mp_adc_stop_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_adc_globals, bpuppy_adc_globals_table);

const mp_obj_module_t bpuppy_adc_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_adc_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy_adc, bpuppy_adc_module);
