/*
 * bPuppy UART 通信驱动 — 头文件
 *
 * UART2 (GPIO 20=RX, 19=TX) — 默认通信口 (CI-33T / micro:bit)  ⚠ 2026-08-19 起反转 TX=GPIO19/RX=GPIO20
 * UART1 (GPIO 4=TX, 5=RX) — 摄像头复用口 (SCCB SDA/SCL)
 * UART0 (GPIO 43/44)         — 烧录 + REPL 控制台
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- 引脚默认值 ---- */
#define UART2_DEFAULT_RX   GPIO_NUM_20   // 2026-08-19 起反转: TX=GPIO19/RX=GPIO20
#define UART2_DEFAULT_TX   GPIO_NUM_19
#define UART2_DEFAULT_BAUD 115200

#define UART1_DEFAULT_RX   GPIO_NUM_5    // CAM SCCB SCL
#define UART1_DEFAULT_TX   GPIO_NUM_4    // CAM SCCB SDA
#define UART1_DEFAULT_BAUD 115200

/* ---- 命令回调类型 ---- */
typedef void (*uart_cmd_callback_t)(uint8_t cmd, const uint8_t *data, int len);

/* ---- UART2 API ---- */

void uart_comm_init(uint8_t uart_num, int tx_pin, int rx_pin, int baud);
void uart_comm_send(const uint8_t *data, int len);
void uart_comm_send_str(const char *str);
int uart_comm_read(uint8_t *buf, int max_len);
void uart_comm_set_callback(uart_cmd_callback_t cb);
// 停止 UART2 (释放外设, 可重新 init)
void uart_comm_stop(void);

/* ---- UART1 API (摄像头复用) ---- */

void uart1_comm_init(int tx_pin, int rx_pin, int baud);
void uart1_comm_send(const uint8_t *data, int len);
int uart1_comm_read(uint8_t *buf, int max_len);
// 停止 UART1 (释放外设, 可重新 init)
void uart1_comm_stop(void);

#ifdef __cplusplus
}
#endif
