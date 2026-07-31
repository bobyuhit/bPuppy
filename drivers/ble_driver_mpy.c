/*
 * bPuppy BLE — MicroPython 绑定
 */
#include "ble_driver.h"
#include "py/runtime.h"
#include "py/obj.h"
#include <string.h>

STATIC mp_obj_t mp_ble_start(void) {
    ble_driver_start();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_ble_start_obj, mp_ble_start);

STATIC mp_obj_t mp_ble_stop(void) {
    ble_driver_stop();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_ble_stop_obj, mp_ble_stop);

STATIC mp_obj_t mp_ble_connected(void) {
    return mp_obj_new_bool(ble_is_connected());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_ble_connected_obj, mp_ble_connected);

STATIC mp_obj_t mp_ble_recv(void) {
    char buf[256];
    int n = ble_recv_command(buf, sizeof(buf));
    if (n > 0) return mp_obj_new_str(buf, n);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_ble_recv_obj, mp_ble_recv);

STATIC mp_obj_t mp_ble_send(mp_obj_t data_obj) {
    const char *s = mp_obj_str_get_str(data_obj);
    ble_send(s);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_ble_send_obj, mp_ble_send);

STATIC mp_obj_t mp_ble_set_battery(mp_obj_t pct_obj) {
    ble_set_battery((uint8_t)mp_obj_get_int(pct_obj));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_ble_set_battery_obj, mp_ble_set_battery);

STATIC const mp_rom_map_elem_t bpuppy_ble_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),      MP_ROM_QSTR(MP_QSTR_bpuppy_ble) },
    { MP_ROM_QSTR(MP_QSTR_start),         MP_ROM_PTR(&mp_ble_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_stop),          MP_ROM_PTR(&mp_ble_stop_obj) },
    { MP_ROM_QSTR(MP_QSTR_connected),     MP_ROM_PTR(&mp_ble_connected_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv),          MP_ROM_PTR(&mp_ble_recv_obj) },
    { MP_ROM_QSTR(MP_QSTR_send),          MP_ROM_PTR(&mp_ble_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_battery),   MP_ROM_PTR(&mp_ble_set_battery_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_ble_globals, bpuppy_ble_globals_table);

const mp_obj_module_t bpuppy_ble_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_ble_globals,
};
MP_REGISTER_MODULE(MP_QSTR_bpuppy_ble, bpuppy_ble_module);
