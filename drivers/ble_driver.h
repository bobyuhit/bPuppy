/*
 * bDog BLE 驱动 — Hiwonder Wonderbot App 兼容
 * ESP-IDF NimBLE: FFE0 UART + 180F Battery
 */
#pragma once
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

void ble_driver_start(void);
bool ble_is_connected(void);

// 收指令: 返回长度, 0=无数据. buf 至少 128 字节
int  ble_recv_command(char *buf, int max_len);

// 发通知 (TX characteristic)
void ble_send(const char *data);

// 电量 0-100
void ble_set_battery(uint8_t pct);

#ifdef __cplusplus
}
#endif
