/*
 * bDog BLE 驱动 — NimBLE GATT Server
 *
 * 两种模式（编译互斥，见 micropython.cmake 编译宏）:
 *   BPUPPY_BLE_KEBLOCK  — KittenBlock 蓝牙 (Nordic UART + dupterm REPL)
 *       服务 6E400001: 6E400002 WRITE (PC→设备) / 6E400003 NOTIFY (设备→PC)
 *       广播含 0x6E40, 设备名 bPuppy_XXXX
 *   BPUPPY_BLE_HIWONDER — Hiwonder Wonderbot App (MechDog 协议)
 *       服务 0xFFE0: FFE1 WRITE (App→设备) / FFE2 NOTIFY (设备→App)
 *       广播含 0xFFE0, 设备名 mechdog_XX
 *
 * ⚠ 两个蓝牙功能不要同时编译，同一固件只能启用其一。
 */
#include "ble_driver.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "esp_nimble_hci.h"
#include <string.h>
#include <stdio.h>

static const char *TAG = "ble";

#ifdef BPUPPY_BLE_HIWONDER
static char g_device_name[32] = "mechdog_00";
#else
static char g_device_name[32] = "bPuppy_00";
#endif

// ---- 状态 ----
static bool      g_connected = false;
static uint16_t  g_conn_handle = 0;
static uint16_t  g_tx_handle = 0;
static uint16_t  g_rx_handle = 0;
static bool      g_ble_started = false;   // NimBLE 栈是否已初始化
static bool      g_adv_enabled = false;   // 广播是否开启 (手动 stop 后关闭)

// 接收缓冲
#define RX_BUF_SIZE 512
static char     g_rx_buf[RX_BUF_SIZE];
static volatile int g_rx_head = 0;
static volatile int g_rx_tail = 0;
static SemaphoreHandle_t g_rx_mutex = NULL;

// ---- GATT 读写回调 ----
// 所有 WRITE 特征的数据统一进 g_rx_buf（单一模式，无协议分流需求）
static int gatt_cb(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (ctxt->om) {
            int len = OS_MBUF_PKTLEN(ctxt->om);
            if (len > 0 && len < 128) {
                uint8_t buf[128];
                os_mbuf_copydata(ctxt->om, 0, len, buf);
                buf[len] = 0;
                xSemaphoreTake(g_rx_mutex, portMAX_DELAY);
                for (int i = 0; i < len; i++) {
                    g_rx_buf[g_rx_head] = buf[i];
                    g_rx_head = (g_rx_head + 1) % RX_BUF_SIZE;
                }
                xSemaphoreGive(g_rx_mutex);
                // 注意: 不打日志! 蓝牙 REPL 模式下每条命令都会打印, 刷屏并干扰 USB CDC 输出
            }
        }
    }
    return 0;
}

// ---- GATT 服务定义 (编译互斥) ----
#ifdef BPUPPY_BLE_HIWONDER
// Hiwonder 模式: FFE1=WRITE (App写命令), FFE2=NOTIFY (设备发回复)
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = BLE_UUID16_DECLARE(0xFFE0),
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = BLE_UUID16_DECLARE(0xFFE1),
                .access_cb = gatt_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &g_tx_handle,
            },
            {
                .uuid = BLE_UUID16_DECLARE(0xFFE2),
                .access_cb = gatt_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &g_rx_handle,
            },
            {0}
        },
    },
    {0}
};
#else
// KittenBlock 模式: Nordic UART 透传 (6E400001)
//   6E400002: WRITE (PC→设备, KittenBlock 写入)
//   6E400003: NOTIFY (设备→PC, KittenBlock 接收)
static const ble_uuid128_t nordic_svc_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x01, 0x00, 0x40, 0x6E);
static const ble_uuid128_t nordic_rx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x02, 0x00, 0x40, 0x6E);
static const ble_uuid128_t nordic_tx_uuid =
    BLE_UUID128_INIT(0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0,
                     0x93, 0xF3, 0xA3, 0xB5, 0x03, 0x00, 0x40, 0x6E);

static const struct ble_gatt_svc_def gatt_svcs[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &nordic_svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[]) {
            {
                .uuid = &nordic_rx_uuid.u,      // 6E400002: PC 写入
                .access_cb = gatt_cb,
                .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP,
                .val_handle = &g_tx_handle,
            },
            {
                .uuid = &nordic_tx_uuid.u,      // 6E400003: 设备通知
                .access_cb = gatt_cb,
                .flags = BLE_GATT_CHR_F_NOTIFY,
                .val_handle = &g_rx_handle,
            },
            {0}
        },
    },
    {0}
};
#endif

static void start_adv(void);

// ---- GAP 事件 ----
static int gap_cb(struct ble_gap_event *ev, void *arg)
{
    switch (ev->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (ev->connect.status == 0) {
            g_conn_handle = ev->connect.conn_handle;
            g_connected = true;
            ESP_LOGI(TAG, "连接");
        }
        break;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "断开");
        g_connected = false;
        if (g_adv_enabled) start_adv();   // 手动 stop 后不自动重启广播
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        if (g_adv_enabled) start_adv();
        break;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ESP_LOGI(TAG, "订阅 h%d notify=%d", ev->subscribe.attr_handle,
                 ev->subscribe.cur_notify);
        break;
    }
    return 0;
}

static void start_adv(void)
{
    if (!g_adv_enabled) return;
    struct ble_gap_adv_params p = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    struct ble_hs_adv_fields f = {0};
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
#ifdef BPUPPY_BLE_HIWONDER
    f.uuids16 = (ble_uuid16_t[]){BLE_UUID16_INIT(0xFFE0)};
#else
    f.uuids16 = (ble_uuid16_t[]){BLE_UUID16_INIT(0x6E40)};
#endif
    f.num_uuids16 = 1;
    f.uuids16_is_complete = 1;
    f.name = (uint8_t*)g_device_name;
    f.name_len = strlen(g_device_name);
    f.name_is_complete = 1;
    ble_gap_adv_set_fields(&f);
#ifndef BPUPPY_BLE_HIWONDER
    // KittenBlock 模式: 扫描响应加 128 位 Nordic 服务 UUID (iOS 对 16 位广播不友好,
    // 完整 128 位 UUID 让 iOS Web Bluetooth 更容易匹配到设备)
    struct ble_hs_adv_fields rsp = {0};
    rsp.uuids128 = &nordic_svc_uuid;
    rsp.num_uuids128 = 1;
    rsp.uuids128_is_complete = 1;
    ble_gap_adv_rsp_set_fields(&rsp);
#endif
    int rc = ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                               &p, gap_cb, NULL);
    if (rc != 0 && rc != BLE_HS_EALREADY) {
        ESP_LOGW(TAG, "adv_start rc=%d", rc);
    }
    ESP_LOGI(TAG, "广播: %s", g_device_name);
}

static void on_sync(void) {
    if (g_adv_enabled) start_adv();
}

static void on_reset(int reason) {
    ESP_LOGE(TAG, "NimBLE reset: %d", reason);
}

static void gatt_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg) {
    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGI(TAG, "GATT svc registered handle=%d", ctxt->svc.handle);
        break;
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGI(TAG, "GATT chr registered def_handle=%d val_handle=%d",
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;
    default:
        break;
    }
}

static void host_task(void *p) {
    nimble_port_run();
    vTaskDelete(NULL);
}

// ---- Public API ----
void ble_driver_start(void) {
    if (!g_ble_started) {
        int rc;
        // 读取 MAC 地址, 生成设备名
        uint8_t mac[6];
        esp_read_mac(mac, ESP_MAC_BT);
#ifdef BPUPPY_BLE_HIWONDER
        snprintf(g_device_name, sizeof(g_device_name), "mechdog_%02X", mac[5]);
#else
        snprintf(g_device_name, sizeof(g_device_name), "bPuppy_%02X%02X", mac[4], mac[5]);
#endif

        g_rx_mutex = xSemaphoreCreateMutex();
        esp_nimble_hci_init();
        nimble_port_init();

        // 配置 NimBLE host（必须在 nimble_port_freertos_init 之前）
        ble_hs_cfg.reset_cb = on_reset;
        ble_hs_cfg.sync_cb = on_sync;
        ble_hs_cfg.gatts_register_cb = gatt_register_cb;
        ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

        // 注册 GATT 服务（必须在 nimble_port_freertos_init 之前）
        ble_svc_gap_device_name_set(g_device_name);
        rc = ble_gatts_count_cfg(gatt_svcs);
        ESP_LOGI(TAG, "count_cfg: %d", rc);
        rc = ble_gatts_add_svcs(gatt_svcs);
        ESP_LOGI(TAG, "add_svcs: %d  tx_h=%d rx_h=%d", rc, g_tx_handle, g_rx_handle);

        nimble_port_freertos_init(host_task);
        g_ble_started = true;
    }
    // 栈已初始化: 重新开启广播 (首次或 stop 后再开)
    g_adv_enabled = true;
    start_adv();
}

void ble_driver_stop(void) {
    if (!g_ble_started) return;
    g_adv_enabled = false;                 // 先关, 防止回调自动重启广播
    ble_gap_adv_stop();
    if (g_connected) {
        ble_gap_terminate(g_conn_handle, BLE_ERR_REM_USER_CONN_TERM);
        g_connected = false;
    }
    ESP_LOGI(TAG, "BLE stopped (广播已停, 连接已断)");
}

bool ble_is_connected(void) { return g_connected; }

int ble_recv_command(char *buf, int max) {
    int n = 0;
    if (!g_rx_mutex) return 0;
    xSemaphoreTake(g_rx_mutex, pdMS_TO_TICKS(50));
    while (g_rx_tail != g_rx_head && n < max - 1) {
        buf[n++] = g_rx_buf[g_rx_tail];
        g_rx_tail = (g_rx_tail + 1) % RX_BUF_SIZE;
    }
    buf[n] = 0;
    xSemaphoreGive(g_rx_mutex);
    return n;
}

// 接收缓冲中可读字节数 (BLE REPL 流 poll 用)
int ble_available(void) {
    if (!g_rx_mutex) return 0;
    int n;
    xSemaphoreTake(g_rx_mutex, pdMS_TO_TICKS(50));
    n = (g_rx_head - g_rx_tail + RX_BUF_SIZE) % RX_BUF_SIZE;
    xSemaphoreGive(g_rx_mutex);
    return n;
}

// 设备通过 TX 特征 (Hiwonder: FFE2 / KittenBlock: 6E400003) 发通知给 PC
void ble_send(const char *data) {
    if (!data) return;
    ble_send_len(data, strlen(data));
}

void ble_send_len(const char *data, int len) {
    if (!g_connected || !g_rx_handle || !data || len <= 0) return;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, len);
    if (om) ble_gattc_notify_custom(g_conn_handle, g_rx_handle, om);
}

void ble_set_battery(uint8_t pct) { (void)pct; /* 暂无 Battery Service */ }
