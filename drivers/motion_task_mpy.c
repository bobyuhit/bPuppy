/*
 * bPuppy 运动控制 — MicroPython 绑定
 */

#include "motion_task.h"
#include "ik.h"
#include "servo_driver.h"
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
    else if (strcmp(s, "crouch") == 0)         g = GAIT_CROUCH;
    else if (strcmp(s, "sit") == 0)            g = GAIT_SIT;
    else if (strcmp(s, "play") == 0)           g = GAIT_PLAY_BOW;
    else if (strcmp(s, "wave") == 0)          g = GAIT_WAVE;
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

STATIC mp_obj_t mp_motion_set_body_dims(mp_obj_t bl_obj, mp_obj_t bw_obj) {
    motion_set_body_dims(mp_obj_get_float(bl_obj), mp_obj_get_float(bw_obj));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_motion_set_body_dims_obj, mp_motion_set_body_dims);

STATIC mp_obj_t mp_motion_set_joint_limits(size_t n_args, const mp_obj_t *args) {
    motion_set_joint_limits(mp_obj_get_float(args[0]), mp_obj_get_float(args[1]),
                             mp_obj_get_float(args[2]), mp_obj_get_float(args[3]));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_motion_set_joint_limits_obj, 4, 4, mp_motion_set_joint_limits);

STATIC mp_obj_t mp_motion_load_geometry(void) {
    motion_load_geometry();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_load_geometry_obj, mp_motion_load_geometry);

// 读取当前运动参数 (speed, stride, height, lift, omega, turn, gait)
STATIC mp_obj_t mp_motion_get_params(void) {
    const motion_state_t *m = motion_get_state();
    mp_obj_t items[7] = {
        mp_obj_new_float(m->speed),
        mp_obj_new_float(m->stride),
        mp_obj_new_float(m->height),
        mp_obj_new_float(m->lift_height),
        mp_obj_new_float(m->omega_base),
        mp_obj_new_float(m->turn),
        mp_obj_new_int((int)m->gait),
    };
    return mp_obj_new_tuple(7, items);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_get_params_obj, mp_motion_get_params);

STATIC mp_obj_t mp_motion_show_geometry(void) {
    const motion_state_t *m = motion_get_state();
    const char *ch_names[8] = {
        "LF_HIP","LF_KNEE","LH_HIP","LH_KNEE",
        "RF_HIP","RF_KNEE","RH_HIP","RH_KNEE"};
    mp_printf(&mp_plat_print, "========== Geometry Parameters ==========\n");
    mp_printf(&mp_plat_print, "Leg:     L1=%.1f mm  L2=%.1f mm\n", m->ik_L1, m->ik_L2);
    mp_printf(&mp_plat_print, "Body:    half_l=%.1f  half_w=%.1f  (full=%.0f x %.0f mm)\n",
              m->body_half_l, m->body_half_w, m->body_half_l * 2, m->body_half_w * 2);
    mp_printf(&mp_plat_print, "Hip:     %.0f ~ %.0f deg\n", ik_hip_min, ik_hip_max);
    mp_printf(&mp_plat_print, "Knee:    %.0f ~ %.0f deg\n", ik_knee_min, ik_knee_max);
    mp_printf(&mp_plat_print, "Offset:  %.0f mm   Lift: %.0f mm\n",
              m->center_offset, m->lift_height);
    mp_printf(&mp_plat_print, "Omega:   %.2f rad/s\n", m->omega_base);
    mp_printf(&mp_plat_print, "--- Servo Calibration (ref_deg) ---\n");
    for (int i = 0; i < 8; i++) {
        mp_printf(&mp_plat_print, "  ch%d %-7s: %.1f deg  (offset=%+.1f)\n",
                  i, ch_names[i], servo_get_cal(i), servo_get_cal(i) - 90.0f);
    }
    mp_printf(&mp_plat_print, "==========================================\n");
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_show_geometry_obj, mp_motion_show_geometry);

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
    { MP_ROM_QSTR(MP_QSTR_set_body_dims), MP_ROM_PTR(&mp_motion_set_body_dims_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_joint_limits), MP_ROM_PTR(&mp_motion_set_joint_limits_obj) },
    { MP_ROM_QSTR(MP_QSTR_load_geometry), MP_ROM_PTR(&mp_motion_load_geometry_obj) },
    { MP_ROM_QSTR(MP_QSTR_show_geometry), MP_ROM_PTR(&mp_motion_show_geometry_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_params),    MP_ROM_PTR(&mp_motion_get_params_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_motion_globals, bpuppy_motion_globals_table);

const mp_obj_module_t bpuppy_motion_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_motion_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy_motion, bpuppy_motion_module);
