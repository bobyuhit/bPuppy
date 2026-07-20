/*
 * bPuppy 摄像头驱动 — OV2640 + esp32-camera 组件
 *
 * DVP 引脚映射 (ESP32-S3 小智板载 OV2640):
 *   SIOD=4  SIOC=5  VSYNC=6  HREF=7  XCLK=15  PCLK=13
 *   D0=11(Y2) D1=9(Y3) D2=8(Y4) D3=10(Y5)
 *   D4=12(Y6) D5=18(Y7) D6=17(Y8) D7=16(Y9)
 *   PWDN/RESET 未接
 *
 * XCLK 由 LCD_CAM 外设内部时钟分频生成 (160MHz ÷ 分频数)，不走 LEDC
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "esp_camera.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---- OV2640 DVP 引脚 (ESP32-S3 小智板载) ---- */
#define CAM_PIN_SIOD  4
#define CAM_PIN_SIOC  5
#define CAM_PIN_VSYNC 6
#define CAM_PIN_HREF  7
#define CAM_PIN_XCLK  15
#define CAM_PIN_PCLK  13
#define CAM_PIN_D0    11   // Y2
#define CAM_PIN_D1    9    // Y3
#define CAM_PIN_D2    8    // Y4
#define CAM_PIN_D3    10   // Y5
#define CAM_PIN_D4    12   // Y6
#define CAM_PIN_D5    18   // Y7
#define CAM_PIN_D6    17   // Y8
#define CAM_PIN_D7    16   // Y9
#define CAM_PIN_PWDN  -1   // 未接
#define CAM_PIN_RESET -1   // 未接

/* ---- XCLK 配置 (ESP32-S3 上 LEDC 参数被忽略，仅填占位值) ---- */
#define CAM_LEDC_TIMER   LEDC_TIMER_1
#define CAM_LEDC_CHANNEL LEDC_CHANNEL_0

/* ---- 默认参数 ---- */
#define CAM_DEFAULT_XCLK       20000000    // 20MHz
#define CAM_DEFAULT_FRAME_SIZE FRAMESIZE_UXGA  // 1600x1200 (OV2640 最大)
#define CAM_DEFAULT_JPEG_QUAL  10          // 0(最优)-63
#define CAM_DEFAULT_FB_COUNT   2           // 双缓冲

/* ---- API ---- */

// 使用默认参数初始化摄像头
esp_err_t camera_driver_init(void);

// 使用自定义参数初始化
// format: PIXFORMAT_JPEG / PIXFORMAT_RGB565 / PIXFORMAT_GRAYSCALE
// quality 仅在 JPEG 格式时有效，RGB565 时传 0
esp_err_t camera_driver_init_adv(framesize_t frame_size, int jpeg_quality,
                                  int fb_count, int xclk_freq,
                                  pixformat_t format);

// 反初始化
esp_err_t camera_driver_deinit(void);

// 捕获一帧 (返回的 fb 用 camera_driver_return 归还)
camera_fb_t* camera_driver_capture(void);

// 归还帧缓冲
void camera_driver_return(camera_fb_t *fb);

// 是否已就绪
bool camera_driver_is_ready(void);

// 获取 sensor 对象 (用于高级调节)
sensor_t* camera_driver_get_sensor(void);

#ifdef __cplusplus
}
#endif
