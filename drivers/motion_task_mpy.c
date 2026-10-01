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
    else if (strcmp(s, "go") == 0)          g = GAIT_GO;
    else if (strcmp(s, "trot") == 0)        g = GAIT_TROT;
    // ★ walkfwd / walkbck / trotfwd / trotbck 四个别名已删除 —— 它们是**被静默吞掉的方向输入**:
    //   四个名字映射到同两个枚举, 方向全靠当时的 stride 符号, 所以 set_gait('walkbck')
    //   在前进步长下照样往前走。方向已经独立成 set_direction(±1), 这四个名字没有再存在的理由;
    //   删掉后旧程序调它们会落到下面的"未知步态 → 停车", **明确失败**好过静默走反。
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

// 运动参数: (步长, 抬脚高度, 站立高度) —— 同一条足端轨迹的三个维度, 一起判一起写。
// ★ 以前这里先调 motion_check_params() 预检一遍来打中文日志, 然后 C 侧被拒时又打一遍
//   (同一件事报两次)。现在直接用 C 侧的 bool 返回值判 —— 判据只有一份, 不会漂。
// ★ 返回值透传给 MicroPython 调用方: True=已生效, False=整组被拒且保持原值。
STATIC mp_obj_t mp_motion_set_params(mp_obj_t stride_obj, mp_obj_t lift_obj,
                                      mp_obj_t height_obj) {
    float stride = mp_obj_get_float(stride_obj);
    float lift   = mp_obj_get_float(lift_obj);
    float height = mp_obj_get_float(height_obj);
    if (!motion_set_params(stride, lift, height)) {
        // ★ 措辞**不猜原因**: 被拒有两条路 (步长为负 / 组合够不着), 具体哪条由 C 侧
        //   ESP_LOGW 打 (friendly REPL 可见)。在 Python 侧重写一遍判据就是第二个会漂的判据。
        const motion_state_t *m = motion_get_state();
        mp_printf(&mp_plat_print, "⚠ 参数被拒! stride=%.0f lift=%.0f height=%.0f, "
                  "保持原值 stride=%.0f lift=%.0f height=%.0f (原因见上一行 C 日志)\n",
                  stride, lift, height, m->stride, m->lift_height, m->height);
        return mp_const_false;
    }
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_3(mp_motion_set_params_obj, mp_motion_set_params);

// 速度/步频 (0~10) —— 与轨迹无关, 独立于 stride/lift/height
STATIC mp_obj_t mp_motion_set_speed(mp_obj_t speed_obj) {
    float speed = mp_obj_get_float(speed_obj);
    if (!motion_set_speed(speed)) {
        mp_printf(&mp_plat_print, "⚠ 速度超限! speed=%.1f (允许 0~10) 被拒, 保持原值 %.2f\n",
                  speed, motion_get_state()->target_speed);
        return mp_const_false;
    }
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_motion_set_speed_obj, mp_motion_set_speed);

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

// ★ set_lift 已删除 —— 抬脚高度并入 set_params 第 2 参。板上残留的旧程序调它会得到
//   AttributeError, 这是预期行为 (见 docs/操作指南.md 「升固件后必须重新下载程序」)。

// 身体姿态: (俯仰, 横滚) —— 顺序跟积木文案「俯仰 [PITCH]…滚转 [ROLL]」一致。
// 返回 True=已采纳, False=被拒 (跟当前 stride/height/lift/重心组合后足端够不着或入地,
// 俯仰和横滚都保持原值)。2026-09-28 起 C 侧做组合校验, 不再是"没有'被拒'这回事"。
STATIC mp_obj_t mp_motion_set_body_pose(mp_obj_t pitch_obj, mp_obj_t roll_obj) {
    float pitch = mp_obj_get_float(pitch_obj);
    float roll  = mp_obj_get_float(roll_obj);
    bool ok = motion_set_body_pose(pitch, roll);
    if (!ok) {
        motion_ensure_geometry_loaded();   // 保证下面报的"原值"是真实值 (任务未起时 NVS 还没载入)
        const motion_state_t *m = motion_get_state();
        mp_printf(&mp_plat_print, "⚠ 机身姿态 (俯仰 %.1f 滚转 %.1f) 被拒, 保持原值 "
                  "(俯仰 %.1f 滚转 %.1f): 当前 stride=%.0f height=%.0f lift=%.0f center=%.1f "
                  "下这个姿态会把足端推出可达范围\n",
                  pitch, roll, m->body_pitch, m->body_roll,
                  m->stride, m->height, m->lift_height, m->center_offset);
    }
    return mp_obj_new_bool(ok);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_2(mp_motion_set_body_pose_obj, mp_motion_set_body_pose);

STATIC mp_obj_t mp_motion_is_running(void) {
    return mp_obj_new_bool(motion_is_running());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_is_running_obj, mp_motion_is_running);

STATIC mp_obj_t mp_motion_get_mode(void) {
    return mp_obj_new_int((int)motion_get_mode());
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_get_mode_obj, mp_motion_get_mode);

// 返回 True=原样采纳。False 有**两种, 含义不同**:
//   超限 → 已**钳位**到 ±1 且写入了钳位后的值 (不是拒绝)
//   NaN  → **拒绝**, 保持原值
// ★ 这两句以前合在一句里说 ("超出 ±1, 已钳位到 …"), 对 NaN 是**假话** ——
//   实测 REPL 打 `set_turn(0.0/0.0)` 会看到"已钳位到 0.0"而值其实没变。分开报。
STATIC mp_obj_t mp_motion_set_turn(mp_obj_t turn_obj) {
    float turn = mp_obj_get_float(turn_obj);
    float old  = motion_get_state()->turn;
    bool ok = motion_set_turn(turn);
    if (!ok) {
        // 比较式判 NaN —— 与 C 侧同一套写法 (NaN 与任何数比较恒假)
        if (!(turn >= -1.0f && turn <= 1.0f)) {
            mp_printf(&mp_plat_print, "⚠ 转弯率是 NaN, **拒绝**并保持原值 %.2f\n", old);
        } else {
            mp_printf(&mp_plat_print, "⚠ 转弯率 %.1f 超出 ±1, 已钳位到 %.1f\n",
                      turn, motion_get_state()->turn);
        }
    }
    return mp_obj_new_bool(ok);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_motion_set_turn_obj, mp_motion_set_turn);

// 方向 (+1=前, -1=后) —— 独立于步长/速度/步态, 上电默认 +1。
// 返回 True=已采纳, False=被拒 (0 或 NaN), 保持原值。
STATIC mp_obj_t mp_motion_set_direction(mp_obj_t dir_obj) {
    float dir = mp_obj_get_float(dir_obj);
    if (!motion_set_direction(dir)) {
        mp_printf(&mp_plat_print, "⚠ 方向只能是 ±1! 收到 %.1f 被拒, 保持原值 %.0f\n",
                  dir, motion_get_state()->direction);
        return mp_const_false;
    }
    return mp_const_true;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(mp_motion_set_direction_obj, mp_motion_set_direction);

// 返回 True=已采纳, False=被拒。被拒有两种原因, 这里**不猜是哪一种** ——
// 判据只有 C 侧一套 (traj_combo_bad), 在 Python 侧重写一遍就是第二个会漂移的判据
// (motion_check_params 当初正是因此被删掉)。具体原因看 C 侧 ESP_LOGW (friendly REPL 可见)。
STATIC mp_obj_t mp_motion_set_center(mp_obj_t obj) {
    float off = mp_obj_get_float(obj);
    bool ok = motion_set_center(off);
    if (!ok) {
        motion_ensure_geometry_loaded();   // 保证下面报的"原值"是真实值 (任务未起时 NVS 还没载入)
        const motion_state_t *m = motion_get_state();
        mp_printf(&mp_plat_print, "⚠ 重心偏移 %.1fmm 被拒, 保持原值 %.1fmm —— 要么超 "
                  "±%.1fmm (大腿 %.1f 的一半), 要么跟当前 stride=%.0f height=%.0f lift=%.0f "
                  "组合后足端够不着\n",
                  off, m->center_offset, m->ik_L1 * 0.5f, m->ik_L1,
                  m->stride, m->height, m->lift_height);
    }
    return mp_obj_new_bool(ok);
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

// 读取当前运动参数, 8 元组 (定序, 消费者按索引取, 不要重排):
//     [0] speed       目标步频 (target_speed, 停止时不被清零, 供网页滑块显示)
//     [1] stride      步长 mm —— **带符号 = 幅度 × 方向** (正=前 负=后)
//     [2] height      站立高度 mm
//     [3] lift        抬脚高度 mm
//     [4] omega       基准角频率 rad/s
//     [5] turn        转弯系数 (-1~+1)
//     [6] gait        步态枚举 (无人读, 保留)
//     [7] direction   ★ 新增: 方向 (±1, 上电默认 +1)
// ★ [1] 刻意返回**带符号**的 stride: 「步长」读数积木和网页都靠它显示前后,
//   幅度是 abs([1]), 方向是 [7]。两者都读得到, 老读者也不用改。
// ★ 前 7 项**定序不动** (camera_stream / 5 个读数积木按索引取), 新项只能往后追加。
// ★ 注意读写的**顺序不一样**, 这是本模块最容易踩的坑:
//     读 = (speed, stride, height, lift, omega, turn, gait, direction)
//     写 = set_params(stride, lift, height) / set_speed(speed) / set_direction(±1)
//   即 lift 在 get_params 里是第 4 个、在 set_params 里是第 2 个。重排元组能"看起来整齐",
//   但会让 camera_stream.py / 5 个读数积木 / 所有 REPL 片段静默错位, 所以定死不动。
STATIC mp_obj_t mp_motion_get_params(void) {
    const motion_state_t *m = motion_get_state();
    mp_obj_t items[8] = {
        mp_obj_new_float(m->target_speed),
        mp_obj_new_float(m->stride * m->direction),   // 带符号, 兼容旧读者
        mp_obj_new_float(m->height),
        mp_obj_new_float(m->lift_height),
        mp_obj_new_float(m->omega_base),
        mp_obj_new_float(m->turn),
        mp_obj_new_int((int)m->gait),
        mp_obj_new_float(m->direction),
    };
    return mp_obj_new_tuple(8, items);
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

// 读**本帧实际生效**的步态参数, 2 元组 (eff_stride, eff_duty) —— 只读, 纯上报。
//
// ★ 用途: 转弯归一化。`turn` 是个**几何系数**(左右步长比), 同样 turn=0.5 在 walk 下
//   实际转 0.59 rad/s、trot 下 1.35 —— 差 2.3 倍。要让"同一个 turn 转得一样快",
//   就得除以当前步态的换算系数 G:
//
//       G_c = eff_stride / ((1 − eff_duty) · half_w)     [rad / 周期, per unit turn]
//       G_s = G_c · (omega_base · speed) / (2π)          [rad / s,   per unit turn]
//
//   而 G 依赖 **实际生效的** eff_stride/eff_duty ⇒ 必须用这个函数, **不能用 get_params()**
//   —— 后者报的是用户设进去的值, 而 **GO 步态会无视用户设的 stride/duty**。
//   (half_w 找 get_geometry()[3], speed 找 get_params()[0], omega_base 找 get_params()[4])
//
// ⚠ 运动任务没跑时这两个数停在初值, 不会更新。
STATIC mp_obj_t mp_motion_get_effective(void) {
    float es = 0, ed = 0;
    motion_get_effective(&es, &ed);
    mp_obj_t items[2] = { mp_obj_new_float(es), mp_obj_new_float(ed) };
    return mp_obj_new_tuple(2, items);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_motion_get_effective_obj, mp_motion_get_effective);

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
    { MP_ROM_QSTR(MP_QSTR_set_speed),      MP_ROM_PTR(&mp_motion_set_speed_obj) },
    { MP_ROM_QSTR(MP_QSTR_cal_ik),         MP_ROM_PTR(&mp_motion_cal_ik_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_omega),      MP_ROM_PTR(&mp_motion_set_omega_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_body_pose),  MP_ROM_PTR(&mp_motion_set_body_pose_obj) },
    { MP_ROM_QSTR(MP_QSTR_is_running),    MP_ROM_PTR(&mp_motion_is_running_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_mode),      MP_ROM_PTR(&mp_motion_get_mode_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_turn),     MP_ROM_PTR(&mp_motion_set_turn_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_direction), MP_ROM_PTR(&mp_motion_set_direction_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_center),  MP_ROM_PTR(&mp_motion_set_center_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_body_dims), MP_ROM_PTR(&mp_motion_set_body_dims_obj) },
    { MP_ROM_QSTR(MP_QSTR_set_joint_limits), MP_ROM_PTR(&mp_motion_set_joint_limits_obj) },
    { MP_ROM_QSTR(MP_QSTR_load_geometry), MP_ROM_PTR(&mp_motion_load_geometry_obj) },
    { MP_ROM_QSTR(MP_QSTR_show_geometry), MP_ROM_PTR(&mp_motion_show_geometry_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_params),    MP_ROM_PTR(&mp_motion_get_params_obj) },
    { MP_ROM_QSTR(MP_QSTR_get_geometry),  MP_ROM_PTR(&mp_motion_get_geometry_obj) },
    // 本帧实际生效的 (eff_stride, eff_duty) —— 转弯归一化用, 见上面的长注释
    { MP_ROM_QSTR(MP_QSTR_get_effective), MP_ROM_PTR(&mp_motion_get_effective_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_motion_globals, bpuppy_motion_globals_table);

const mp_obj_module_t bpuppy_motion_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_motion_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy_motion, bpuppy_motion_module);
