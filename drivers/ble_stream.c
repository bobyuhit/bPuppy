/*
 * bPuppy BLE 流对象 — 桥接 MicroPython REPL 到 Nordic UART
 *
 * 仅 BPUPPY_BLE_KEBLOCK 模式编译（KittenBlock 蓝牙）。
 * 配合 os.dupterm() 使用: BLE 成为 MicroPython REPL 的第二输入输出通道。
 *
 *   read  — 从 Nordic 6E400002 (PC→设备) 接收缓冲取字节
 *   write — 通过 Nordic 6E400003 (设备→PC) 发通知
 */

#ifdef BPUPPY_BLE_KEBLOCK

#include "ble_driver.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "py/stream.h"
#include "esp_timer.h"
#include <string.h>

// ---- echo 批量打包 ----
// 逐字符发通知会瞬间吃光 NimBLE 小包池 (MSYS_1 仅 12 块), 饿死紧随的下一写入包 → 指令截断。
// 攒到一整行 (遇 \n) 或超时再发一个通知 (MTU 256 可装 253 字节), 一条命令的 echo 从几十个通知变 1 个。
#define TX_FLUSH_MAX        480     // 缓冲上限 (接近 MTU 256 载荷, 留余量)
#define TX_FLUSH_TIMEOUT_MS 30      // 无 \n 的输出 (如 ">>> " 提示符) 超时兜底

typedef struct {
    mp_obj_base_t base;
    char pending[64];       // read 拉取缓冲 (ble_recv_command 一次拉一块)
    mp_uint_t pending_len;
    mp_uint_t pending_pos;
    char tx_buf[TX_FLUSH_MAX];   // echo 批量打包缓冲
    mp_uint_t tx_len;
    int64_t  tx_last_ms;         // 最后一次写入时间 (超时兜底 flush)
} ble_stream_obj_t;

// 把攒下的 echo 一次发出去 (一个通知包)
STATIC void ble_stream_tx_flush(ble_stream_obj_t *self) {
    if (self->tx_len == 0) return;
    ble_send_len(self->tx_buf, self->tx_len);
    self->tx_len = 0;
}

// 超时兜底 flush: 缓冲非空且超过时间 → 发出去 (挂在 read/poll 高频路径上)
STATIC void ble_stream_tx_flush_timeout(ble_stream_obj_t *self) {
    if (self->tx_len == 0) return;
    int64_t now = esp_timer_get_time() / 1000;
    if (now - self->tx_last_ms >= TX_FLUSH_TIMEOUT_MS) {
        ble_stream_tx_flush(self);
    }
}

STATIC mp_uint_t ble_stream_read(mp_obj_t self_in, void *buf, mp_uint_t size, int *errcode) {
    ble_stream_obj_t *self = MP_OBJ_TO_PTR(self_in);
    ble_stream_tx_flush_timeout(self);
    mp_uint_t got = 0;
    while (got < size) {
        if (self->pending_pos >= self->pending_len) {
            // 拉下一块
            self->pending_len = ble_recv_command(self->pending, sizeof(self->pending));
            self->pending_pos = 0;
            if (self->pending_len == 0) {
                break;
            }
        }
        ((uint8_t *)buf)[got++] = (uint8_t)self->pending[self->pending_pos++];
    }
    if (got == 0) {
        // 非阻塞: 无数据 → EAGAIN (dupterm 识别为非阻塞错误, 继续轮询)
        *errcode = MP_EAGAIN;
        return MP_STREAM_ERROR;
    }
    return got;
}

STATIC mp_uint_t ble_stream_write(mp_obj_t self_in, const void *buf, mp_uint_t size, int *errcode) {
    (void)errcode;
    ble_stream_obj_t *self = MP_OBJ_TO_PTR(self_in);
    const char *p = (const char *)buf;
    mp_uint_t i = 0;
    // 追加进打包缓冲 (空间不足时截断丢弃尾部, 属异常情况)
    while (i < size && self->tx_len < TX_FLUSH_MAX) {
        self->tx_buf[self->tx_len++] = p[i++];
    }
    self->tx_last_ms = esp_timer_get_time() / 1000;
    // 攒够一整行 (\n 结束) 或缓冲满 → 立刻发
    if (self->tx_len > 0 && (self->tx_buf[self->tx_len - 1] == '\n'
                             || self->tx_len >= TX_FLUSH_MAX)) {
        ble_stream_tx_flush(self);
    }
    return size;   // 全部接收 (异步稍后发出), dupterm 认为已写成功
}

STATIC mp_uint_t ble_stream_ioctl(mp_obj_t self_in, mp_uint_t request, uintptr_t arg, int *errcode) {
    ble_stream_obj_t *self = MP_OBJ_TO_PTR(self_in);
    (void)errcode;
    ble_stream_tx_flush_timeout(self);
    switch (request) {
        case MP_STREAM_POLL: {
            mp_uint_t ret = 0;
            if (arg & MP_STREAM_POLL_RD) {
                if (self->pending_pos < self->pending_len || ble_available() > 0) {
                    ret |= MP_STREAM_POLL_RD;
                }
            }
            if (arg & MP_STREAM_POLL_WR) {
                ret |= MP_STREAM_POLL_WR;
            }
            return ret;
        }
        default:
            return MP_STREAM_ERROR;
    }
}

STATIC const mp_stream_p_t ble_stream_stream_p = {
    .read = ble_stream_read,
    .write = ble_stream_write,
    .ioctl = ble_stream_ioctl,
};

// v1.22 slot 机制 (手动初始化)。类型名用已有 qstr (bpuppy_ble) 避免新增 QSTR
STATIC const mp_obj_type_t ble_stream_type = {
    { &mp_type_type },
    .name = MP_QSTR_bpuppy_ble,
    .flags = MP_TYPE_FLAG_NONE,
    .slot_index_protocol = 1,
    .slots = { &ble_stream_stream_p },
};

mp_obj_t mp_ble_get_stream(void) {
    ble_stream_obj_t *self = m_new_obj(ble_stream_obj_t);
    self->base.type = &ble_stream_type;
    self->pending_len = 0;
    self->pending_pos = 0;
    self->tx_len = 0;
    self->tx_last_ms = 0;
    return MP_OBJ_FROM_PTR(self);
}

#endif // BPUPPY_BLE_KEBLOCK
