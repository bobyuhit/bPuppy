/*
 * bPuppy ADC 电压测量 — ⚠ 已停用 (2026-08)
 *
 * 原用 GPIO38 = ADC1_CH2 测电池电压, 但 GPIO38 已改作舵机(左前小腿)。
 * 且该描述有误: ESP32-S3 的 ADC1_CH2 实际对应 GPIO3 (IMU I2C0 SDA),
 *   GPIO38 本身不是 ADC 引脚 → 原电池测量本就不可靠。
 *
 * 可用 ADC 引脚只有 GPIO1~20 (ADC1: 1~10, ADC2: 11~20), 现全部被占用
 *   (舵机/IMU/摄像头/UART2), ADC2 在 BLE/WiFi 下还会失败。
 * 需要电池检测时: 复用摄像头 ADC1 脚 (如 GPIO4=ADC1_CH3), 不拍照时
 *   deinit 摄像头 → init ADC; 同时把下方 BPUPPY_ADC_ENABLE 改 1。
 *
 * MicroPython 接口 (init 已无效, 见下):
 *   import bpuppy_adc
 *   bpuppy_adc.init()            # ⚠ 停用: 打印警告, 不初始化
 *   mv = bpuppy_adc.read_mv()    # 返回 -1
 *
 * ⚠ 必须用 legacy ADC API (adc1_*): MicroPython 的 machine.ADC 使用 legacy
 *   driver, ESP-IDF 5.x 中 legacy 与 driver_ng (adc_oneshot) 互斥,
 *   混用会触发 "CONFLICT! driver_ng is not allowed..." 断言重启。
 */

#define BPUPPY_ADC_ENABLE  0   // 0=停用 (GPIO38 已给舵机); 1=启用(需改通道+复用引脚)

#include "py/runtime.h"
#include "py/obj.h"
#include "driver/adc.h"
#include "esp_log.h"

static const char *TAG = "adc";

#if BPUPPY_ADC_ENABLE
#define ADC_DEFAULT_CHANNEL  ADC1_CHANNEL_2   // 需按实际复用的引脚改通道
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
    ESP_LOGW(TAG, "电池检测已停用 (GPIO38 已给舵机); 需复用摄像头 ADC1 脚 + BPUPPY_ADC_ENABLE=1 才可用");
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
