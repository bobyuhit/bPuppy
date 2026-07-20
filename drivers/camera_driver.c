/*
 * bPuppy 摄像头驱动 — OV2640 + esp32-camera 组件
 *
 * ESP32-S3 DVP 并行接口, 8-bit Y2-Y9 数据线.
 * XCLK 由 LCD_CAM 外设内部时钟分频生成, 不占用 LEDC.
 * SCCB (I2C) 由 esp32-camera 组件内部管理, 不与舵机 I2C 冲突.
 *
 * MicroPython 接口:
 *   import bpuppy_camera
 *   bpuppy_camera.init()
 *   data, w, h, fmt = bpuppy_camera.capture()   # JPEG 或 RGB565 bytes
 *   bpuppy_camera.deinit()
 */

#include "camera_driver.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "esp_log.h"

static const char *TAG = "camera";
static bool g_camera_ready = false;

/* ---- 初始化 (默认参数) ---- */
esp_err_t camera_driver_init(void)
{
    return camera_driver_init_adv(CAM_DEFAULT_FRAME_SIZE,
                                   CAM_DEFAULT_JPEG_QUAL,
                                   CAM_DEFAULT_FB_COUNT,
                                   CAM_DEFAULT_XCLK,
                                   PIXFORMAT_JPEG);
}

/* ---- 初始化 (自定义参数) ---- */
esp_err_t camera_driver_init_adv(framesize_t frame_size, int jpeg_quality,
                                  int fb_count, int xclk_freq,
                                  pixformat_t format)
{
    if (g_camera_ready) {
        ESP_LOGW(TAG, "Already initialized, deinit first");
        camera_driver_deinit();
    }

    camera_config_t config = {
        .pin_pwdn       = CAM_PIN_PWDN,
        .pin_reset      = CAM_PIN_RESET,
        .pin_xclk       = CAM_PIN_XCLK,
        .pin_sccb_sda   = CAM_PIN_SIOD,
        .pin_sccb_scl   = CAM_PIN_SIOC,
        .pin_d7         = CAM_PIN_D7,
        .pin_d6         = CAM_PIN_D6,
        .pin_d5         = CAM_PIN_D5,
        .pin_d4         = CAM_PIN_D4,
        .pin_d3         = CAM_PIN_D3,
        .pin_d2         = CAM_PIN_D2,
        .pin_d1         = CAM_PIN_D1,
        .pin_d0         = CAM_PIN_D0,
        .pin_vsync      = CAM_PIN_VSYNC,
        .pin_href       = CAM_PIN_HREF,
        .pin_pclk       = CAM_PIN_PCLK,

        .xclk_freq_hz   = xclk_freq,
        .ledc_timer     = CAM_LEDC_TIMER,
        .ledc_channel   = CAM_LEDC_CHANNEL,
        .pixel_format   = format,
        .frame_size     = frame_size,
        .jpeg_quality   = jpeg_quality,
        .fb_count       = fb_count,
        .fb_location    = CAMERA_FB_IN_PSRAM,
        .grab_mode      = CAMERA_GRAB_LATEST,    // 始终返回最新帧
    };

    ESP_LOGI(TAG, "Initializing OV2640 (fmt=%d, size=%d, fb=%d, xclk=%dHz)...",
             format, frame_size, fb_count, xclk_freq);

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed: 0x%x", err);
        return err;
    }

    g_camera_ready = true;

    sensor_t *s = esp_camera_sensor_get();
    if (s) {
        s->set_brightness(s, 0);
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_vflip(s, 1);       // 上下翻转
        s->set_hmirror(s, 0);     // 不左右镜像
    }

    const char *fmt_name = (format == PIXFORMAT_JPEG) ? "JPEG" :
                            (format == PIXFORMAT_RGB565) ? "RGB565" : "other";
    ESP_LOGI(TAG, "OV2640 ready (fmt=%s, size=%d, fb=%d)", fmt_name, frame_size, fb_count);
    return ESP_OK;
}

/* ---- 反初始化 ---- */
esp_err_t camera_driver_deinit(void)
{
    if (!g_camera_ready) return ESP_OK;

    esp_err_t err = esp_camera_deinit();
    if (err == ESP_OK) {
        g_camera_ready = false;
        ESP_LOGI(TAG, "Camera deinitialized");
    } else {
        ESP_LOGW(TAG, "Camera deinit failed: 0x%x", err);
    }
    return err;
}

/* ---- 捕获一帧 ---- */
camera_fb_t* camera_driver_capture(void)
{
    if (!g_camera_ready) {
        ESP_LOGW(TAG, "Camera not ready");
        return NULL;
    }
    return esp_camera_fb_get();
}

/* ---- 归还帧缓冲 ---- */
void camera_driver_return(camera_fb_t *fb)
{
    if (fb) esp_camera_fb_return(fb);
}

/* ---- 状态查询 ---- */
bool camera_driver_is_ready(void)
{
    return g_camera_ready;
}

/* ---- 获取 sensor ---- */
sensor_t* camera_driver_get_sensor(void)
{
    if (!g_camera_ready) return NULL;
    return esp_camera_sensor_get();
}

/* ================================================================
 * MicroPython 导出接口
 *
 *   import bpuppy_camera
 *   # 默认 (JPEG SVGA)
 *   bpuppy_camera.init()
 *   # 自定义 (RGB565 QVGA)
 *   bpuppy_camera.init_adv(bpuppy_camera.QVGA, 0, 2, 20000000, bpuppy_camera.RGB565)
 *
 *   data, w, h, fmt = bpuppy_camera.capture()
 *   # JPEG:  data 是 JPEG bytes, fmt=4
 *   # RGB565: data 是原始像素 bytes, fmt=1
 *   bpuppy_camera.deinit()
 *
 * fmt 常量: bpuppy_camera.JPEG(4) / RGB565(1) / GRAYSCALE(5)
 * frame_size 常量: bpuppy_camera.QVGA / VGA / SVGA / XGA / UXGA
 * ================================================================ */

// ---- init() ----
STATIC mp_obj_t mp_camera_init(void) {
    esp_err_t err = camera_driver_init();
    if (err != ESP_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
                          "Camera init failed: 0x%x", err);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_camera_init_obj, mp_camera_init);

// ---- init_adv(framesize, quality, fb_count, xclk_freq, format) ----
STATIC mp_obj_t mp_camera_init_adv(size_t n_args, const mp_obj_t *args) {
    framesize_t fs = (framesize_t)mp_obj_get_int(args[0]);
    int quality    = mp_obj_get_int(args[1]);
    int fb_count   = mp_obj_get_int(args[2]);
    int xclk       = mp_obj_get_int(args[3]);
    pixformat_t fmt = (pixformat_t)mp_obj_get_int(args[4]);
    esp_err_t err = camera_driver_init_adv(fs, quality, fb_count, xclk, fmt);
    if (err != ESP_OK) {
        mp_raise_msg_varg(&mp_type_RuntimeError,
                          "Camera init_adv failed: 0x%x", err);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_camera_init_adv_obj, 5, 5, mp_camera_init_adv);

// ---- deinit() ----
STATIC mp_obj_t mp_camera_deinit(void) {
    camera_driver_deinit();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_camera_deinit_obj, mp_camera_deinit);

// ---- capture() → (bytes, width, height, format) 或 None ----
STATIC mp_obj_t mp_camera_capture(void) {
    camera_fb_t *fb = camera_driver_capture();
    if (!fb) {
        return mp_const_none;
    }

    mp_obj_t tuple[4] = {
        mp_obj_new_bytes(fb->buf, fb->len),
        mp_obj_new_int(fb->width),
        mp_obj_new_int(fb->height),
        mp_obj_new_int(fb->format),
    };

    camera_driver_return(fb);
    return mp_obj_new_tuple(4, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_camera_capture_obj, mp_camera_capture);

// ---- is_ready() → bool ----
STATIC mp_obj_t mp_camera_ready(void) {
    return mp_obj_new_bool(camera_driver_is_ready());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_camera_ready_obj, mp_camera_ready);

/* ---- 模块定义 ---- */
STATIC const mp_rom_map_elem_t bpuppy_camera_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_bpuppy_camera) },
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&mp_camera_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_init_adv),    MP_ROM_PTR(&mp_camera_init_adv_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit),      MP_ROM_PTR(&mp_camera_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_capture),     MP_ROM_PTR(&mp_camera_capture_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_ready),    MP_ROM_PTR(&mp_camera_ready_obj) },

    // ---- pixel_format 常量 ----
    { MP_ROM_QSTR(MP_QSTR_JPEG),        MP_ROM_INT(PIXFORMAT_JPEG) },
    { MP_ROM_QSTR(MP_QSTR_RGB565),      MP_ROM_INT(PIXFORMAT_RGB565) },
    { MP_ROM_QSTR(MP_QSTR_GRAYSCALE),   MP_ROM_INT(PIXFORMAT_GRAYSCALE) },

    // ---- frame_size 常量 ----
    { MP_ROM_QSTR(MP_QSTR_QQVGA),       MP_ROM_INT(FRAMESIZE_QQVGA) },    // 160x120
    { MP_ROM_QSTR(MP_QSTR_QVGA),        MP_ROM_INT(FRAMESIZE_QVGA) },     // 320x240
    { MP_ROM_QSTR(MP_QSTR_VGA),         MP_ROM_INT(FRAMESIZE_VGA) },      // 640x480
    { MP_ROM_QSTR(MP_QSTR_SVGA),        MP_ROM_INT(FRAMESIZE_SVGA) },     // 800x600
    { MP_ROM_QSTR(MP_QSTR_XGA),         MP_ROM_INT(FRAMESIZE_XGA) },      // 1024x768
    { MP_ROM_QSTR(MP_QSTR_UXGA),        MP_ROM_INT(FRAMESIZE_UXGA) },     // 1600x1200
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_camera_globals, bpuppy_camera_globals_table);

const mp_obj_module_t bpuppy_camera_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_camera_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy_camera, bpuppy_camera_module);
