/*
 * bPuppy UART 通信驱动
 *
 * UART2 (GPIO 19=RX, GPIO 20=TX) — 默认通信口 (CI-33T / micro:bit)
 * UART1 (GPIO 4=TX, GPIO 5=RX) — 摄像头复用口 (SCCB SDA/SCL)
 * UART0 (GPIO 43/44)          — 烧录 + REPL 控制台
 */

#include "uart_driver.h"
#include "driver/uart.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "uart";

/* ================================================================
 * UART2 (默认通信口)
 * ================================================================ */

__attribute__((weak)) bool g_uart2_ready = false;
__attribute__((weak)) uart_port_t g_uart2_num = UART_NUM_2;

void uart_comm_init(uint8_t uart_num, int tx_pin, int rx_pin, int baud)
{
    if (g_uart2_ready) {
        ESP_LOGW(TAG, "UART2 already initialized");
        return;
    }
    g_uart2_num = (uart_port_t)uart_num;

    uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(g_uart2_num, 512, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(g_uart2_num, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(g_uart2_num, tx_pin, rx_pin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    g_uart2_ready = true;
    ESP_LOGI(TAG, "UART2 ready  RX=GPIO%d  TX=GPIO%d  %d baud", rx_pin, tx_pin, baud);
}

void uart_comm_send(const uint8_t *data, int len)
{
    if (!g_uart2_ready || !data || len <= 0) return;
    int sent = uart_write_bytes(g_uart2_num, data, len);
    if (sent < len) ESP_LOGW(TAG, "UART2 send truncated: %d/%d", sent, len);
}

void uart_comm_send_str(const char *str)
{
    uart_comm_send((const uint8_t *)str, strlen(str));
}

int uart_comm_read(uint8_t *buf, int max_len)
{
    if (!g_uart2_ready || !buf || max_len <= 0) return 0;
    size_t avail = 0;
    if (uart_get_buffered_data_len(g_uart2_num, &avail) != ESP_OK || avail == 0) return 0;
    if ((int)avail > max_len) avail = (size_t)max_len;
    int n = uart_read_bytes(g_uart2_num, buf, avail, 0);
    return (n > 0) ? n : 0;
}

__attribute__((weak)) uart_cmd_callback_t g_cmd_callback = NULL;

void uart_comm_set_callback(uart_cmd_callback_t cb)
{
    g_cmd_callback = cb;
}

void uart_comm_stop(void)
{
    if (!g_uart2_ready) return;
    uart_driver_delete(g_uart2_num);
    g_uart2_ready = false;
    ESP_LOGI(TAG, "UART2 stopped (可重新 init)");
}


/* ================================================================
 * UART1 (摄像头复用口, GPIO 4=TX, 5=RX)
 * ================================================================ */

__attribute__((weak)) bool g_uart1_ready = false;
__attribute__((weak)) uart_port_t g_uart1_num = UART_NUM_1;

void uart1_comm_init(int tx_pin, int rx_pin, int baud)
{
    if (g_uart1_ready) {
        ESP_LOGW(TAG, "UART1 already initialized");
        return;
    }

    uart_config_t cfg = {
        .baud_rate  = baud,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM_1, 512, 512, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM_1, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM_1, tx_pin, rx_pin,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    g_uart1_ready = true;
    ESP_LOGI(TAG, "UART1 ready  RX=GPIO%d  TX=GPIO%d  %d baud", rx_pin, tx_pin, baud);
}

void uart1_comm_send(const uint8_t *data, int len)
{
    if (!g_uart1_ready || !data || len <= 0) return;
    int sent = uart_write_bytes(UART_NUM_1, data, len);
    if (sent < len) ESP_LOGW(TAG, "UART1 send truncated: %d/%d", sent, len);
}

static void uart1_comm_send_str(const char *str)
{
    uart1_comm_send((const uint8_t *)str, strlen(str));
}

int uart1_comm_read(uint8_t *buf, int max_len)
{
    if (!g_uart1_ready || !buf || max_len <= 0) return 0;
    size_t avail = 0;
    if (uart_get_buffered_data_len(UART_NUM_1, &avail) != ESP_OK || avail == 0) return 0;
    if ((int)avail > max_len) avail = (size_t)max_len;
    int n = uart_read_bytes(UART_NUM_1, buf, avail, 0);
    return (n > 0) ? n : 0;
}

void uart1_comm_stop(void)
{
    if (!g_uart1_ready) return;
    uart_driver_delete(UART_NUM_1);
    g_uart1_ready = false;
    ESP_LOGI(TAG, "UART1 stopped (可重新 init)");
}


/* ================================================================
 * MicroPython 导出接口
 *
 *   import bpuppy_uart
 *
 *   # UART2 (默认)
 *   bpuppy_uart.init(2, 20, 19, 115200)
 *   bpuppy_uart.send(b"hello")
 *   data = bpuppy_uart.read(64)
 *
 *   # UART1 (摄像头复用)
 *   bpuppy_uart.u1_init(4, 5, 115200)     # TX=4, RX=5
 *   bpuppy_uart.u1_send(b"hello")
 *   data = bpuppy_uart.u1_read(64)
 * ================================================================ */

// ==================== UART2 MP ====================

STATIC mp_obj_t mp_uart_init(size_t n_args, const mp_obj_t *args) {
    int num  = mp_obj_get_int(args[0]);
    int tx   = mp_obj_get_int(args[1]);
    int rx   = mp_obj_get_int(args[2]);
    int baud = mp_obj_get_int(args[3]);
    uart_comm_init((uint8_t)num, tx, rx, baud);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_uart_init_obj, 4, 4, mp_uart_init);

STATIC mp_obj_t mp_uart_send(mp_obj_t obj) {
    mp_buffer_info_t bufinfo;
    if (mp_get_buffer(obj, &bufinfo, MP_BUFFER_READ)) {
        uart_comm_send(bufinfo.buf, bufinfo.len);
    } else {
        uart_comm_send_str(mp_obj_str_get_str(obj));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_uart_send_obj, mp_uart_send);

STATIC mp_obj_t mp_uart_read(mp_obj_t len_obj) {
    int max_len = mp_obj_get_int(len_obj);
    if (max_len > 1024) max_len = 1024;
    uint8_t buf[1024];
    int n = uart_comm_read(buf, max_len);
    return mp_obj_new_bytes(buf, n);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_uart_read_obj, mp_uart_read);

STATIC mp_obj_t mp_uart_any(void) {
    if (!g_uart2_ready) return mp_obj_new_int(0);
    size_t avail = 0;
    uart_get_buffered_data_len(g_uart2_num, &avail);
    return mp_obj_new_int((int)avail);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_uart_any_obj, mp_uart_any);

STATIC mp_obj_t mp_uart_stop(void) {
    uart_comm_stop();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_uart_stop_obj, mp_uart_stop);

STATIC mp_obj_t mp_uart_sendline(mp_obj_t str_obj) {
    uart_comm_send_str(mp_obj_str_get_str(str_obj));
    uart_comm_send((const uint8_t *)"\r\n", 2);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_uart_sendline_obj, mp_uart_sendline);

// ==================== UART1 MP ====================

STATIC mp_obj_t mp_u1_init(size_t n_args, const mp_obj_t *args) {
    int tx   = mp_obj_get_int(args[0]);
    int rx   = mp_obj_get_int(args[1]);
    int baud = mp_obj_get_int(args[2]);
    uart1_comm_init(tx, rx, baud);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_u1_init_obj, 3, 3, mp_u1_init);

STATIC mp_obj_t mp_u1_send(mp_obj_t obj) {
    mp_buffer_info_t bufinfo;
    if (mp_get_buffer(obj, &bufinfo, MP_BUFFER_READ)) {
        uart1_comm_send(bufinfo.buf, bufinfo.len);
    } else {
        uart1_comm_send_str(mp_obj_str_get_str(obj));
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_u1_send_obj, mp_u1_send);

STATIC mp_obj_t mp_u1_read(mp_obj_t len_obj) {
    int max_len = mp_obj_get_int(len_obj);
    if (max_len > 1024) max_len = 1024;
    uint8_t buf[1024];
    int n = uart1_comm_read(buf, max_len);
    return mp_obj_new_bytes(buf, n);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_u1_read_obj, mp_u1_read);

STATIC mp_obj_t mp_u1_any(void) {
    if (!g_uart1_ready) return mp_obj_new_int(0);
    size_t avail = 0;
    uart_get_buffered_data_len(UART_NUM_1, &avail);
    return mp_obj_new_int((int)avail);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_u1_any_obj, mp_u1_any);

STATIC mp_obj_t mp_u1_stop(void) {
    uart1_comm_stop();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_u1_stop_obj, mp_u1_stop);

STATIC mp_obj_t mp_u1_sendline(mp_obj_t str_obj) {
    uart1_comm_send_str(mp_obj_str_get_str(str_obj));
    uart1_comm_send((const uint8_t *)"\r\n", 2);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_u1_sendline_obj, mp_u1_sendline);

// ==================== 模块定义 ====================

STATIC const mp_rom_map_elem_t bpuppy_uart_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),  MP_ROM_QSTR(MP_QSTR_bpuppy_uart) },
    // UART2
    { MP_ROM_QSTR(MP_QSTR_init),      MP_ROM_PTR(&mp_uart_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_send),      MP_ROM_PTR(&mp_uart_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_read),      MP_ROM_PTR(&mp_uart_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_any),       MP_ROM_PTR(&mp_uart_any_obj) },
    { MP_ROM_QSTR(MP_QSTR_sendline),  MP_ROM_PTR(&mp_uart_sendline_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),      MP_ROM_PTR(&mp_uart_stop_obj) },
    // UART1 (摄像头复用)
    { MP_ROM_QSTR(MP_QSTR_u1_init),     MP_ROM_PTR(&mp_u1_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_u1_send),     MP_ROM_PTR(&mp_u1_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_u1_read),     MP_ROM_PTR(&mp_u1_read_obj) },
    { MP_ROM_QSTR(MP_QSTR_u1_any),      MP_ROM_PTR(&mp_u1_any_obj) },
    { MP_ROM_QSTR(MP_QSTR_u1_sendline), MP_ROM_PTR(&mp_u1_sendline_obj) },
    { MP_ROM_QSTR(MP_QSTR_u1_stop),     MP_ROM_PTR(&mp_u1_stop_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_uart_globals, bpuppy_uart_globals_table);

const mp_obj_module_t bpuppy_uart_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_uart_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy_uart, bpuppy_uart_module);
