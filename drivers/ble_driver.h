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
void ble_driver_stop(void);
bool ble_is_connected(void);

// 收指令: 返回长度, 0=无数据. 最多取 max_len-1 字节
int  ble_recv_command(char *buf, int max_len);

// 接收缓冲中可读字节数 (BLE REPL 流 poll 用)
int  ble_available(void);

// 发通知 (TX characteristic, 字符串版, 兼容 ble_hiwonder.py)
void ble_send(const char *data);

// 发通知 (TX characteristic, 带长度, BLE REPL 流用)
void ble_send_len(const char *data, int len);

// 电量 0-100
void ble_set_battery(uint8_t pct);

#ifdef __cplusplus
}
#endif
