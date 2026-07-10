/*
 * bPuppy 版本信息模块
 *
 * 每次功能变更手动更新 BP_VERSION，编译时间自动更新。
 * 在 REPL 中:
 *   import bpuppy
 *   bpuppy.version()       → "v0.1  Jul  9 2026 14:30:00"
 */

#include "py/runtime.h"
#include "py/obj.h"
#include <string.h>

#define BP_VERSION "202607090001"

STATIC mp_obj_t bpuppy_version(void) {
    return mp_obj_new_str(BP_VERSION " " __DATE__ " " __TIME__,
                          strlen(BP_VERSION " " __DATE__ " " __TIME__));
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(bpuppy_version_obj, bpuppy_version);

STATIC const mp_rom_map_elem_t bpuppy_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),  MP_ROM_QSTR(MP_QSTR_bpuppy) },
    { MP_ROM_QSTR(MP_QSTR_version),   MP_ROM_PTR(&bpuppy_version_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_globals, bpuppy_globals_table);

const mp_obj_module_t bpuppy_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy, bpuppy_module);
