/*
 * bDog BLE 驱动 — NimBLE GATT Server (FFE0 UART)
 *
 * 兼容 Hiwonder Wonderbot App (MechDog 协议):
 *   FFE1: WRITE  — App 向设备写 CMD 指令
 *   FFE2: NOTIFY — 设备向 App 发送回复
 *
 * 注意: FFE1/FFE2 的角色对 MechDog 是反直觉的 —
 *   "TX" (FFE1) 实际是 App→设备, "RX" (FFE2) 实际是 设备→App
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
static char g_device_name[32] = "mechdog_00";

// ---- 状态 ----
static bool      g_connected = false;
static uint16_t  g_conn_handle = 0;
static uint16_t  g_tx_handle = 0;
static uint16_t  g_rx_handle = 0;

// 接收缓冲
#define RX_BUF_SIZE 512
static char     g_rx_buf[RX_BUF_SIZE];
static volatile int g_rx_head = 0;
static volatile int g_rx_tail = 0;
static SemaphoreHandle_t g_rx_mutex = NULL;

// ---- GATT 读写回调 ----
// MechDog 协议: App 向 FFE1 (g_tx_handle) 写命令
static int gatt_cb(uint16_t conn, uint16_t attr, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        if (attr == g_tx_handle && ctxt->om) {
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
                ESP_LOGI(TAG, "CMD: %s", buf);
            }
        }
    }
    return 0;
}

// ---- GATT 服务定义 ----
// MechDog 协议: FFE1=WRITE (App写命令), FFE2=NOTIFY (设备发回复)
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
        ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, NULL,
                          gap_cb, NULL);
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER, NULL,
                          gap_cb, NULL);
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
    struct ble_gap_adv_params p = {
        .conn_mode = BLE_GAP_CONN_MODE_UND,
        .disc_mode = BLE_GAP_DISC_MODE_GEN,
    };
    struct ble_hs_adv_fields f = {0};
    f.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    f.uuids16 = (ble_uuid16_t[]){BLE_UUID16_INIT(0xFFE0)};
    f.num_uuids16 = 1;
    f.uuids16_is_complete = 1;
    f.name = (uint8_t*)g_device_name;
    f.name_len = strlen(g_device_name);
    f.name_is_complete = 1;
    ble_gap_adv_set_fields(&f);
    ble_gap_adv_start(BLE_OWN_ADDR_PUBLIC, NULL, BLE_HS_FOREVER,
                      &p, gap_cb, NULL);
    ESP_LOGI(TAG, "广播: %s", g_device_name);
}

static void on_sync(void) {
    start_adv();
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
    int rc;
    // 读取 MAC 地址, 生成 mechdog_{:02X} 设备名
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    snprintf(g_device_name, sizeof(g_device_name), "mechdog_%02X", mac[5]);

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

// MechDog 协议: 设备通过 FFE2 (g_rx_handle) 发通知给 App
void ble_send(const char *data) {
    if (!g_connected || !g_rx_handle || !data) return;
    struct os_mbuf *om = ble_hs_mbuf_from_flat(data, strlen(data));
    if (om) ble_gattc_notify_custom(g_conn_handle, g_rx_handle, om);
}

void ble_set_battery(uint8_t pct) { (void)pct; /* 暂无 Battery Service */ }
