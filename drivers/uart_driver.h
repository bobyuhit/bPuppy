/*
 * bPuppy UART 通信驱动 — 头文件
 *
 * 与上位机/遥控器双向通信
 * ESP32-S3 有 3 个 UART: UART0, UART1, UART2
 * 控制台可通过 USB-JTAG CDC 使用，UART0 可腾出给外设
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 命令回调类型 ---- */
typedef void (*uart_cmd_callback_t)(uint8_t cmd, const uint8_t *data, int len);

/* ---- API ---- */

// 初始化 UART
// uart_num: UART_NUM_1 或 UART_NUM_2
void uart_comm_init(uint8_t uart_num, int tx_pin, int rx_pin, int baud);

// 发送数据
void uart_comm_send(const uint8_t *data, int len);

// 发送字符串
void uart_comm_send_str(const char *str);

// 读取数据（非阻塞）
// 返回实际读取的字节数
int uart_comm_read(uint8_t *buf, int max_len);

// 设置命令回调
void uart_comm_set_callback(uart_cmd_callback_t cb);

#ifdef __cplusplus
}
#endif
