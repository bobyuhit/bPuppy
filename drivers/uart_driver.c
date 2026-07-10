/*
 * bPuppy UART 通信驱动 — 实现（骨架）
 *
 * TODO: 完善 UART 初始化 + 中断/轮询接收逻辑
 * 当前为占位实现
 */

#include "uart_driver.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "uart";

__attribute__((weak)) bool g_uart_ready = false;
__attribute__((weak)) uart_cmd_callback_t g_cmd_callback = NULL;

void uart_comm_init(uint8_t uart_num, int tx_pin, int rx_pin, int baud)
{
    // TODO: 配置 UART 参数 + 安装驱动
    ESP_LOGI(TAG, "UART init placeholder: UART%d, TX=%d, RX=%d, %d baud",
             uart_num, tx_pin, rx_pin, baud);
    g_uart_ready = true;
}

void uart_comm_send(const uint8_t *data, int len)
{
    if (!g_uart_ready || !data || len <= 0) return;
    // TODO: uart_write_bytes
}

void uart_comm_send_str(const char *str)
{
    uart_comm_send((const uint8_t *)str, strlen(str));
}

int uart_comm_read(uint8_t *buf, int max_len)
{
    if (!g_uart_ready || !buf || max_len <= 0) return 0;
    // TODO: uart_read_bytes (非阻塞)
    return 0;
}

void uart_comm_set_callback(uart_cmd_callback_t cb)
{
    g_cmd_callback = cb;
}

/* ================================================================
 * MicroPython 导出接口（骨架）
 *
 * 在 Python 中使用:
 *   import bpuppy_uart
 *   bpuppy_uart.init(1, 17, 16, 115200)  # UART1, TX=17, RX=16
 *   bpuppy_uart.send("hello")
 *   data = bpuppy_uart.read(64)
 * ================================================================ */

STATIC mp_obj_t mp_uart_init(size_t n_args, const mp_obj_t *args) {
    // uart.init(num, tx, rx, baud)
    int num  = mp_obj_get_int(args[0]);
    int tx   = mp_obj_get_int(args[1]);
    int rx   = mp_obj_get_int(args[2]);
    int baud = mp_obj_get_int(args[3]);
    uart_comm_init((uint8_t)num, tx, rx, baud);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_uart_init_obj, 4, 4, mp_uart_init);

STATIC mp_obj_t mp_uart_send(mp_obj_t str_obj) {
    const char *s = mp_obj_str_get_str(str_obj);
    uart_comm_send_str(s);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_uart_send_obj, mp_uart_send);

STATIC mp_obj_t mp_uart_read(mp_obj_t len_obj) {
    int max_len = mp_obj_get_int(len_obj);
    uint8_t buf[256];
    if (max_len > 256) max_len = 256;
    int n = uart_comm_read(buf, max_len);
    return mp_obj_new_bytes(buf, n);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_uart_read_obj, mp_uart_read);

// ---- 模块定义 ----
STATIC const mp_rom_map_elem_t bpuppy_uart_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_bpuppy_uart) },
    { MP_ROM_QSTR(MP_QSTR_init),     MP_ROM_PTR(&mp_uart_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_send),     MP_ROM_PTR(&mp_uart_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),     MP_ROM_PTR(&mp_uart_read_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_uart_globals, bpuppy_uart_globals_table);

const mp_obj_module_t bpuppy_uart_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_uart_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy_uart, bpuppy_uart_module);
