/*
 * bPuppy 运动控制 — MicroPython 绑定
 */

#include "motion_task.h"
#include "py/runtime.h"
#include "py/obj.h"
#include <string.h>

STATIC mp_obj_t mp_motion_start(void) {
    motion_task_start();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_start_obj, mp_motion_start);

STATIC mp_obj_t mp_motion_set_gait(mp_obj_t gait_obj) {
    const char *s = mp_obj_str_get_str(gait_obj);
    gait_type_t g = GAIT_STAND;
         if (strcmp(s, "stand") == 0)        g = GAIT_STAND;
    else if (strcmp(s, "walk") == 0)        g = GAIT_WALK;
    else if (strcmp(s, "walkfwd") == 0)     g = GAIT_WALK;
    else if (strcmp(s, "walkbck") == 0)     g = GAIT_WALK;
    else if (strcmp(s, "go") == 0)          g = GAIT_GO;
    else if (strcmp(s, "trot") == 0)        g = GAIT_TROT;
    else if (strcmp(s, "trotfwd") == 0)     g = GAIT_TROT;
    else if (strcmp(s, "trotbck") == 0)     g = GAIT_TROT;
    else if (strcmp(s, "sit") == 0)         g = GAIT_SIT;
    motion_set_gait(g);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_motion_set_gait_obj, mp_motion_set_gait);

STATIC mp_obj_t mp_motion_set_params(mp_obj_t speed_obj, mp_obj_t stride_obj,
                                      mp_obj_t height_obj) {
    float stride = mp_obj_get_float(stride_obj);
    float height = mp_obj_get_float(height_obj);
    if ((stride > 0 || height > 0) && motion_check_params(stride, height)) {
        mp_printf(&mp_plat_print, "⚠ 参数超限! stride=%.0f height=%.0f 被拒，保持原值\n",
                  stride, height);
    }
    motion_set_params(mp_obj_get_float(speed_obj), stride, height);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_motion_set_params_obj, mp_motion_set_params);

STATIC mp_obj_t mp_motion_cal_ik(mp_obj_t L1_obj, mp_obj_t L2_obj) {
    motion_cal_ik(mp_obj_get_float(L1_obj), mp_obj_get_float(L2_obj));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_motion_cal_ik_obj, mp_motion_cal_ik);

STATIC mp_obj_t mp_motion_set_omega(mp_obj_t omega_obj) {
    motion_set_omega(mp_obj_get_float(omega_obj));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_motion_set_omega_obj, mp_motion_set_omega);

STATIC mp_obj_t mp_motion_set_lift(mp_obj_t lift_obj) {
    motion_set_lift(mp_obj_get_float(lift_obj));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_motion_set_lift_obj, mp_motion_set_lift);

STATIC mp_obj_t mp_motion_set_body_pose(mp_obj_t roll_obj, mp_obj_t pitch_obj,
                                         mp_obj_t yaw_obj) {
    motion_set_body_pose(mp_obj_get_float(roll_obj),
                         mp_obj_get_float(pitch_obj),
                         mp_obj_get_float(yaw_obj));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_motion_set_body_pose_obj, mp_motion_set_body_pose);

STATIC mp_obj_t mp_motion_estop(void) {
    motion_emergency_stop();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_estop_obj, mp_motion_estop);

STATIC mp_obj_t mp_motion_resume(void) {
    motion_resume();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_resume_obj, mp_motion_resume);

STATIC mp_obj_t mp_motion_stand_up(void) {
    motion_stand_up();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_stand_up_obj, mp_motion_stand_up);

STATIC mp_obj_t mp_motion_set_turn(mp_obj_t turn_obj) {
    motion_set_turn(mp_obj_get_float(turn_obj));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_motion_set_turn_obj, mp_motion_set_turn);

STATIC mp_obj_t mp_motion_jump(void) {
    motion_jump();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_jump_obj, mp_motion_jump);

STATIC mp_obj_t mp_motion_set_center(mp_obj_t obj) {
    motion_set_center(mp_obj_get_float(obj));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_motion_set_center_obj, mp_motion_set_center);

STATIC const mp_rom_map_elem_t bpuppy_motion_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),       MP_ROM_QSTR(MP_QSTR_bpuppy_motion) },
    { MP_ROM_QSTR(MP_QSTR_start),          MP_ROM_PTR(&mp_motion_start_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_gait),       MP_ROM_PTR(&mp_motion_set_gait_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_params),     MP_ROM_PTR(&mp_motion_set_params_obj) },
    { MP_ROM_QSTR(MP_QSTR_cal_ik),         MP_ROM_PTR(&mp_motion_cal_ik_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_omega),      MP_ROM_PTR(&mp_motion_set_omega_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_lift),      MP_ROM_PTR(&mp_motion_set_lift_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_body_pose),  MP_ROM_PTR(&mp_motion_set_body_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_emergency_stop), MP_ROM_PTR(&mp_motion_estop_obj) },
    { MP_ROM_QSTR(MP_QSTR_resume),         MP_ROM_PTR(&mp_motion_resume_obj) },
    { MP_ROM_QSTR(MP_QSTR_stand_up),      MP_ROM_PTR(&mp_motion_stand_up_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_turn),     MP_ROM_PTR(&mp_motion_set_turn_obj) },
    { MP_ROM_QSTR(MP_QSTR_jump),         MP_ROM_PTR(&mp_motion_jump_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_center),  MP_ROM_PTR(&mp_motion_set_center_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_motion_globals, bpuppy_motion_globals_table);

const mp_obj_module_t bpuppy_motion_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_motion_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy_motion, bpuppy_motion_module);
