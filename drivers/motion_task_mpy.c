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
    gait_type_t g = GAIT_STOP;
         if (strcmp(s, "stop") == 0)        g = GAIT_STOP;
    else if (strcmp(s, "walk") == 0)        g = GAIT_WALK;
    else if (strcmp(s, "walkfwd") == 0)     g = GAIT_WALK;
    else if (strcmp(s, "walkbck") == 0)     g = GAIT_WALK;
    else if (strcmp(s, "go") == 0)          g = GAIT_GO;
    else if (strcmp(s, "trot") == 0)        g = GAIT_TROT;
    else if (strcmp(s, "trotfwd") == 0)     g = GAIT_TROT;
    else if (strcmp(s, "trotbck") == 0)     g = GAIT_TROT;
    else {
        // 未知步态名 → 停车。这是刻意的 fail-safe (指令可能被传坏, 如蓝牙丢字节
        // "trot"→"tr"), 不是缺陷。行为不变, 只把"名字打错"和"正常停车"区分开。
        // 详见 docs/操作指南.md §1.2 步态列表
        mp_printf(&mp_plat_print, "⚠ 未知步态 \"%s\" → 停车\n", s);
    }
    motion_set_gait(g);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_motion_set_gait_obj, mp_motion_set_gait);

STATIC mp_obj_t mp_motion_set_params(mp_obj_t speed_obj, mp_obj_t stride_obj,
                                      mp_obj_t height_obj) {
    float stride = mp_obj_get_float(stride_obj);
    float height = mp_obj_get_float(height_obj);
    // ★ 不再带 (stride > 0 || height > 0) 前置条件 —— 要和 C 侧 motion_set_params()
    //   的判据完全一致, 否则"打了被拒日志其实写进去了"。C 侧现在无条件校验。
    if (motion_check_params(stride, height)) {
        const motion_state_t *m = motion_get_state();
        mp_printf(&mp_plat_print, "⚠ 参数超限! stride=%.0f height=%.0f lift=%.0f 被拒, "
                  "保持原值 speed=%.1f stride=%.0f height=%.0f lift=%.0f\n",
                  stride, height, m->lift_height,
                  m->target_speed, m->stride, m->height, m->lift_height);
    }
    motion_set_params(mp_obj_get_float(speed_obj), stride, height);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_motion_set_params_obj, mp_motion_set_params);

STATIC mp_obj_t mp_motion_cal_ik(mp_obj_t L1_obj, mp_obj_t L2_obj) {
    float L1 = mp_obj_get_float(L1_obj);
    float L2 = mp_obj_get_float(L2_obj);
    // 写入被拒 (非法值) 时打出来 —— 否则改腿长"没反应"会被当成指令没生效
    if (!motion_cal_ik(L1, L2)) {
        motion_ensure_geometry_loaded();   // 保证下面报的"原值"是真实值 (任务未起时 NVS 还没载入)
        const motion_state_t *m = motion_get_state();
        mp_printf(&mp_plat_print, "⚠ 腿长非法! L1=%.1f L2=%.1f 被拒 "
                  "(需 %.0f~%.0f mm), 保持原值 L1=%.1f L2=%.1f\n",
                  L1, L2, IK_LEN_MIN, IK_LEN_MAX, m->ik_L1, m->ik_L2);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_motion_cal_ik_obj, mp_motion_cal_ik);

STATIC mp_obj_t mp_motion_set_omega(mp_obj_t omega_obj) {
    motion_set_omega(mp_obj_get_float(omega_obj));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_motion_set_omega_obj, mp_motion_set_omega);

STATIC mp_obj_t mp_motion_set_lift(mp_obj_t lift_obj) {
    float lift = mp_obj_get_float(lift_obj);
    if (!motion_set_lift(lift)) {
        const motion_state_t *m = motion_get_state();
        mp_printf(&mp_plat_print, "⚠ 抬腿超限! lift=%.0f (stride=%.0f height=%.0f) 被拒, "
                  "保持原值 lift=%.0f\n",
                  lift, m->stride, m->height, m->lift_height);
    }
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

STATIC mp_obj_t mp_motion_is_running(void) {
    return mp_obj_new_bool(motion_is_running());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_is_running_obj, mp_motion_is_running);

STATIC mp_obj_t mp_motion_get_mode(void) {
    return mp_obj_new_int((int)motion_get_mode());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_get_mode_obj, mp_motion_get_mode);

STATIC mp_obj_t mp_motion_set_turn(mp_obj_t turn_obj) {
    motion_set_turn(mp_obj_get_float(turn_obj));
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_motion_set_turn_obj, mp_motion_set_turn);

STATIC mp_obj_t mp_motion_set_center(mp_obj_t obj) {
    float off = mp_obj_get_float(obj);
    if (!motion_set_center(off)) {
        motion_ensure_geometry_loaded();   // 保证下面报的"原值"是真实值 (任务未起时 NVS 还没载入)
        const motion_state_t *m = motion_get_state();
        mp_printf(&mp_plat_print, "⚠ 重心偏移非法! %.1fmm 超出 ±%.1fmm (大腿 %.1f 的一半) 被拒, "
                  "保持原值 %.1fmm\n",
                  off, m->ik_L1 * 0.5f, m->ik_L1, m->center_offset);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_motion_set_center_obj, mp_motion_set_center);

STATIC mp_obj_t mp_motion_set_body_dims(mp_obj_t bl_obj, mp_obj_t bw_obj) {
    float bl = mp_obj_get_float(bl_obj);
    float bw = mp_obj_get_float(bw_obj);
    if (!motion_set_body_dims(bl, bw)) {
        motion_ensure_geometry_loaded();
        const motion_state_t *m = motion_get_state();
        mp_printf(&mp_plat_print, "⚠ 机身尺寸非法! half_l=%.1f half_w=%.1f 被拒 "
                  "(需 %.0f~%.0f mm), 保持原值 half_l=%.1f half_w=%.1f\n",
                  bl, bw, IK_LEN_MIN, IK_LEN_MAX, m->body_half_l, m->body_half_w);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_motion_set_body_dims_obj, mp_motion_set_body_dims);

STATIC mp_obj_t mp_motion_set_joint_limits(size_t n_args, const mp_obj_t *args) {
    float hmin = mp_obj_get_float(args[0]), hmax = mp_obj_get_float(args[1]);
    float kmin = mp_obj_get_float(args[2]), kmax = mp_obj_get_float(args[3]);
    // ★ 顺序反了 (min>max) 是最要命的一种: 它不会报错, 只会让狗彻底不动
    //   (四个角度钳位全被压成 max, 且 ik_pos_check 拒绝一切采样)
    if (!motion_set_joint_limits(hmin, hmax, kmin, kmax)) {
        mp_printf(&mp_plat_print, "⚠ 限位非法! hip[%.0f~%.0f] knee[%.0f~%.0f] 被拒 "
                  "(需 min<max, 且在舵机行程 [%d, %d] 内), 保持原值 "
                  "hip[%.0f~%.0f] knee[%.0f~%.0f]\n",
                  hmin, hmax, kmin, kmax,
                  (int)SERVO_ANGLE_MIN, (int)SERVO_ANGLE_MAX,
                  ik_hip_min, ik_hip_max, ik_knee_min, ik_knee_max);
    }
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_motion_set_joint_limits_obj, 4, 4, mp_motion_set_joint_limits);

STATIC mp_obj_t mp_motion_load_geometry(void) {
    motion_load_geometry();
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_load_geometry_obj, mp_motion_load_geometry);

// 读取当前运动参数 (speed=目标步频, stride, height, lift, omega, turn, gait)
// speed 返回 target_speed (用户设定值, 停止时不被清零), 供网页滑块显示
STATIC mp_obj_t mp_motion_get_params(void) {
    const motion_state_t *m = motion_get_state();
    mp_obj_t items[7] = {
        mp_obj_new_float(m->target_speed),
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

// 读取当前几何参数 (L1, L2, 前后半距, 左右半宽), 单位 mm
// ★ Python 侧 (poses.stand / balance) 必须用这个, 不能抄 ik.h 的默认值 ——
//   cal_ik() 改的是 g_motion, 与编译期常量 bpuppy_ik.L1/L2 是两回事。
STATIC mp_obj_t mp_motion_get_geometry(void) {
    motion_ensure_geometry_loaded();   // 开机时运动任务可能还没创建, g_motion 尚未载入
    const motion_state_t *m = motion_get_state();
    mp_obj_t items[4] = {
        mp_obj_new_float(m->ik_L1),
        mp_obj_new_float(m->ik_L2),
        mp_obj_new_float(m->body_half_l),
        mp_obj_new_float(m->body_half_w),
    };
    return mp_obj_new_tuple(4, items);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_get_geometry_obj, mp_motion_get_geometry);

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
    { MP_ROM_QSTR(MP_QSTR_is_running),    MP_ROM_PTR(&mp_motion_is_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_mode),      MP_ROM_PTR(&mp_motion_get_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_turn),     MP_ROM_PTR(&mp_motion_set_turn_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_center),  MP_ROM_PTR(&mp_motion_set_center_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_body_dims), MP_ROM_PTR(&mp_motion_set_body_dims_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_joint_limits), MP_ROM_PTR(&mp_motion_set_joint_limits_obj) },
    { MP_ROM_QSTR(MP_QSTR_load_geometry), MP_ROM_PTR(&mp_motion_load_geometry_obj) },
    { MP_ROM_QSTR(MP_QSTR_show_geometry), MP_ROM_PTR(&mp_motion_show_geometry_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_params),    MP_ROM_PTR(&mp_motion_get_params_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_geometry),  MP_ROM_PTR(&mp_motion_get_geometry_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_motion_globals, bpuppy_motion_globals_table);

const mp_obj_module_t bpuppy_motion_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_motion_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy_motion, bpuppy_motion_module);
