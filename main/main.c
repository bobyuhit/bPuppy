/*
 * bPuppy 机器狗 — ESP32-S3 主入口
 *
 * 通过 micropython-helper 组件启动 MicroPython 运行时。
 * mpy_startup() 内部会:
 *   1. 初始化 NVS
 *   2. 创建 MicroPython FreeRTOS 任务
 *   3. MicroPython 任务依次执行:
 *      - _boot.py (frozen)
 *      - boot.py  (文件系统, 可选)
 *      - main.py  (frozen 或文件系统)
 *   4. 进入 REPL 交互循环
 */

#include <stdio.h>
#include "esp_system.h"
#include "esp_flash.h"
#include "esp_log.h"

// micropython-helper 提供的启动函数
extern void mpy_startup(void);

static const char *TAG = "bPuppy";

void app_main(void)
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  bPuppy Robot Dog - ESP32-S3 MicroPython");
    ESP_LOGI(TAG, "  Chip: ESP32-S3 WROOM-1 N16R8");

    uint32_t flash_size = 0;
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "  Flash: %lu MB", (unsigned long)(flash_size / (1024 * 1024)));
    }

    ESP_LOGI(TAG, "========================================");

    // 启动 MicroPython 运行时（内部处理 NVS、FreeRTOS 任务等）
    // 注意：mpy_startup() 永不返回
    ESP_LOGI(TAG, "Starting MicroPython...");
    mpy_startup();
}
