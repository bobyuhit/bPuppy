/*
 * bPuppy ADC 电压测量 — 电池检测 (V3.0 硬件: 启用)
 *
 * V3.0 硬件 (当前在用): IMU SDA 改到 GPIO14, GPIO3=ADC1_CH2 腾出接电池分压。
 *   ADC1 在 BLE/WiFi 下可用 → 启用 (ENABLE=1)。
 *
 * V2.0 硬件 (历史): 电池分压接 GPIO14 = ADC2_CH3, ADC2 与 BLE/WiFi 射频共享,
 *   蓝牙一开读数即失败 (ESP_ERR_TIMEOUT) → 电池检测停用 (ENABLE=0)。
 *
 * 硬件分压 (见 docs/硬件连接.md, R4=47k/R5=10k):
 *   电池正 ──[R1 47kΩ]──┬── GPIO3 (V3.0)
 *                       │
 *   GND  ────[R2 10kΩ]──┤
 *   分压比 10/57 ≈ 0.175, 软件换算 ×5.7
 *   满电 8.4V → ADC ~1.47V; 标称 7.4V → ADC ~1.30V
 *
 * MicroPython 接口:
 *   import bpuppy_adc
 *   bpuppy_adc.init()            # 初始化 (11dB 衰减, ~0-3.1V)
 *   mv = bpuppy_adc.read_mv()    # ADC 引脚电压 (mV)
 *   volt = mv * 5.7 / 1000       # 电池电压 (V), 软件换算
 *
 * ⚠ 必须用 legacy ADC API (adc1_*): MicroPython 的 machine.ADC 使用 legacy
 *   driver, ESP-IDF 5.x 中 legacy 与 driver_ng (adc_oneshot) 互斥,
 *   混用会触发 "CONFLICT! driver_ng is not allowed..." 断言重启。
 */

#define BPUPPY_ADC_ENABLE  1   // V3.0: 电池=GPIO3/ADC1_CH2, BLE/WiFi 下可用 → 启用

#include "py/runtime.h"
#include "py/obj.h"
#include "driver/adc.h"
#include "esp_log.h"

static const char *TAG = "adc";

#if BPUPPY_ADC_ENABLE
#define ADC_DEFAULT_CHANNEL  ADC1_CHANNEL_2   // GPIO3 (V3.0: IMU SDA 改到 GPIO14 后腾出)
#define ADC_DEFAULT_ATTEN    ADC_ATTEN_DB_11  // ~0-3.1V
#endif

static bool g_adc_ready = false;

void adc_init(void)
{
#if BPUPPY_ADC_ENABLE
    if (g_adc_ready) return;

    ESP_ERROR_CHECK(adc1_config_width(ADC_WIDTH_BIT_12));
    ESP_ERROR_CHECK(adc1_config_channel_atten(ADC_DEFAULT_CHANNEL, ADC_DEFAULT_ATTEN));

    g_adc_ready = true;
    ESP_LOGI(TAG, "ADC ready  atten=11dB (legacy)");
#else
    ESP_LOGW(TAG, "电池检测已停用 (V2.0 硬件: 电池=GPIO14/ADC2, BLE 下不可用); V3.0 板需 BPUPPY_ADC_ENABLE=1");
    return;
#endif
}

int adc_read_raw(void)
{
#if BPUPPY_ADC_ENABLE
    if (!g_adc_ready) return -1;
    return adc1_get_raw(ADC_DEFAULT_CHANNEL);   // 失败返回 -1
#else
    return -1;   // 已停用
#endif
}

int adc_read_mv(void)
{
#if BPUPPY_ADC_ENABLE
    if (!g_adc_ready) return -1;
    int raw = adc1_get_raw(ADC_DEFAULT_CHANNEL);
    if (raw < 0) return -1;
    // 11dB 衰减: ~0-3100mV → 0-4095 (12-bit)
    return (int)((int64_t)raw * 3100 / 4096);
#else
    return -1;   // 已停用
#endif
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
 *   bpuppy_adc.init()        # GPIO 3, ADC1_CH2
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
