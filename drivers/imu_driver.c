/*
 * bPuppy IMU 传感器驱动 — QMI8658 6轴 IMU
 *
 * I2C 接口引脚和地址由 Python 层初始化时指定。
 * 使用 ESP-IDF v5.x 新版 I2C Master API (driver/i2c_master.h)
 *
 * 数据格式:
 *   加速度: 16-bit signed, 量程 ±16g → 2048 LSB/g
 *   陀螺仪: 16-bit signed, 量程 ±1024 dps → 32 LSB/dps
 *   温度: 有符号, 需要校准
 */

#include "imu_driver.h"
#include "py/runtime.h"
#include "py/obj.h"
#include "driver/i2c.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "imu";

/* ---- QMI8658 寄存器 ---- */
#define QMI8658_WHO_AM_I     0x00
#define QMI8658_REVISION_ID   0x01
#define QMI8658_CTRL1         0x02
#define QMI8658_CTRL2         0x03
#define QMI8658_CTRL3         0x04
#define QMI8658_CTRL5         0x06
#define QMI8658_CTRL7         0x08
#define QMI8658_CTRL8         0x09
#define QMI8658_CTRL9         0x0A
#define QMI8658_AX_L          0x35   // 数据从 0x35 开始, 连续12字节
#define QMI8658_TEMP_L        0x33

#define QMI8658_I2C_ADDR      0x6A

/* ---- 全局 ---- */
__attribute__((weak)) bool g_imu_ready = false;
static i2c_port_t g_i2c_port;

/* ---- I2C 读写 ---- */
static esp_err_t imu_write_reg(uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = {reg, val};
    return i2c_master_write_to_device(g_i2c_port, QMI8658_I2C_ADDR,
                                      buf, 2, pdMS_TO_TICKS(10));
}

static esp_err_t imu_read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_write_read_device(g_i2c_port, QMI8658_I2C_ADDR,
                                        &reg, 1, data, len, pdMS_TO_TICKS(10));
}

/* ---- 初始化 ---- */
void imu_init(uint8_t port, uint8_t sda_pin, uint8_t scl_pin, uint8_t addr)
{
    ESP_LOGI(TAG, "QMI8658 init: I2C%d, SDA=%d, SCL=%d", port, sda_pin, scl_pin);

    g_i2c_port = (i2c_port_t)port;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .scl_io_num = scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 400000,
    };
    ESP_ERROR_CHECK(i2c_param_config(g_i2c_port, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(g_i2c_port, conf.mode, 0, 0, 0));

    // 检查 WHO_AM_I
    uint8_t whoami = 0;
    if (imu_read_regs(QMI8658_WHO_AM_I, &whoami, 1) == ESP_OK) {
        ESP_LOGI(TAG, "WHO_AM_I: 0x%02X", whoami);
    }

    // 软复位
    imu_write_reg(QMI8658_CTRL9, 0xB0);
    vTaskDelay(pdMS_TO_TICKS(10));

    // CTRL1: 地址自增 + Big Endian
    imu_write_reg(QMI8658_CTRL1, 0x60);

    // CTRL7: 使能加速度计 + 陀螺仪
    imu_write_reg(QMI8658_CTRL7, 0x03);

    // CTRL2: 加速度计 ±4g, ODR 250Hz
    imu_write_reg(QMI8658_CTRL2, 0x91);

    // CTRL3: 陀螺仪 ±512dps, ODR 250Hz
    imu_write_reg(QMI8658_CTRL3, 0xD1);

    // CTRL5: 低通滤波
    imu_write_reg(QMI8658_CTRL5, 0x11);
    vTaskDelay(pdMS_TO_TICKS(20));

    g_imu_ready = true;
    ESP_LOGI(TAG, "QMI8658 ready");
}

/* ---- 读取原始数据 ---- */
void imu_read_raw(imu_raw_data_t *data)
{
    if (!g_imu_ready || !data) return;

    uint8_t buf[12];

    // 从 0x35 连续读 12 字节: AX,AY,AZ,GX,GY,GZ (Big Endian)
    if (imu_read_regs(QMI8658_AX_L, buf, 12) != ESP_OK) {
        memset(data, 0, sizeof(*data));
        return;
    }
    int16_t ax = (int16_t)((buf[1] << 8) | buf[0]);
    int16_t ay = (int16_t)((buf[3] << 8) | buf[2]);
    int16_t az = (int16_t)((buf[5] << 8) | buf[4]);
    int16_t gx = (int16_t)((buf[7] << 8) | buf[6]);
    int16_t gy = (int16_t)((buf[9] << 8) | buf[8]);
    int16_t gz = (int16_t)((buf[11] << 8) | buf[10]);

    // 温度 (0x33-0x34)
    uint8_t tbuf[2];
    float temp = 0.0f;
    if (imu_read_regs(QMI8658_TEMP_L, tbuf, 2) == ESP_OK) {
        int16_t t = (int16_t)((tbuf[1] << 8) | tbuf[0]);
        temp = t / 256.0f + 25.0f;
    }

    // ±4g → 8192 LSB/g → 9.81 m/s²/g
    data->accel_x = (float)ax / 8192.0f * 9.81f;
    data->accel_y = (float)ay / 8192.0f * 9.81f;
    data->accel_z = (float)az / 8192.0f * 9.81f;

    // ±512dps → 64 LSB/dps → PI/180 rad/s/dps
    data->gyro_x = (float)gx / 64.0f * 0.0174533f;
    data->gyro_y = (float)gy / 64.0f * 0.0174533f;
    data->gyro_z = (float)gz / 64.0f * 0.0174533f;

    data->temp_c = temp;
}

/* ---- 互补滤波 ---- */
// 加速度计算重力方向 (roll/pitch), 陀螺积分高速响应, 两路加权融合
// 无磁力计 → yaw 不计算, 保持为 0

#define FILTER_A  0.98f   // 陀螺权重 (高速)
static float cf_roll  = 0.0f;
static float cf_pitch = 0.0f;
static int64_t cf_last_us = 0;

void imu_read_angles(imu_angles_t *angles)
{
    if (!g_imu_ready || !angles) return;

    imu_raw_data_t raw;
    imu_read_raw(&raw);

    int64_t now = esp_timer_get_time();
    float dt = (cf_last_us == 0) ? 0.0f : (now - cf_last_us) * 1e-6f;
    cf_last_us = now;

    // 加速度计角度 (重力分量)
    float acc_roll  = atan2f(raw.accel_y, raw.accel_z) * 57.29578f;
    float acc_pitch = atan2f(-raw.accel_x, sqrtf(raw.accel_y*raw.accel_y + raw.accel_z*raw.accel_z)) * 57.29578f;

    if (dt > 0.0f && dt < 0.5f) {
        // 陀螺积分 + 互补融合
        cf_roll  = FILTER_A * (cf_roll  + raw.gyro_x * dt * 57.29578f) + (1.0f - FILTER_A) * acc_roll;
        cf_pitch = FILTER_A * (cf_pitch + raw.gyro_y * dt * 57.29578f) + (1.0f - FILTER_A) * acc_pitch;
    } else {
        // 首帧: 直接用加速度计
        cf_roll  = acc_roll;
        cf_pitch = acc_pitch;
    }

    angles->roll  = cf_roll;
    angles->pitch = cf_pitch;
    angles->yaw   = 0.0f;
}

void imu_calibrate(int samples)
{
    if (!g_imu_ready) return;
    ESP_LOGI(TAG, "IMU calibrate: %d samples (placeholder)", samples);
}

void imu_task_start(int freq_hz)
{
    ESP_LOGI(TAG, "IMU task: %d Hz (placeholder)", freq_hz);
}

/* ================================================================
 * MicroPython 导出接口
 *
 *   import bpuppy_imu
 *   bpuppy_imu.init(0, 1, 2, 0x6A)  # I2C0, SDA=1, SCL=2
 *   accel, gyro, temp = bpuppy_imu.read_raw()
 * ================================================================ */

STATIC mp_obj_t mp_imu_init(size_t n_args, const mp_obj_t *args) {
    int port = mp_obj_get_int(args[0]);
    int sda  = mp_obj_get_int(args[1]);
    int scl  = mp_obj_get_int(args[2]);
    int addr = mp_obj_get_int(args[3]);
    imu_init((uint8_t)port, (uint8_t)sda, (uint8_t)scl, (uint8_t)addr);
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(mp_imu_init_obj, 4, 4, mp_imu_init);

STATIC mp_obj_t mp_imu_read_raw(void) {
    imu_raw_data_t data;
    memset(&data, 0, sizeof(data));
    imu_read_raw(&data);

    mp_obj_t accel[3] = {
        mp_obj_new_float(data.accel_x),
        mp_obj_new_float(data.accel_y),
        mp_obj_new_float(data.accel_z),
    };
    mp_obj_t gyro[3] = {
        mp_obj_new_float(data.gyro_x),
        mp_obj_new_float(data.gyro_y),
        mp_obj_new_float(data.gyro_z),
    };
    mp_obj_t tuple[3] = {
        mp_obj_new_tuple(3, accel),
        mp_obj_new_tuple(3, gyro),
        mp_obj_new_float(data.temp_c),
    };
    return mp_obj_new_tuple(3, tuple);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_imu_read_raw_obj, mp_imu_read_raw);

STATIC mp_obj_t mp_imu_read_angles(void) {
    imu_angles_t a;
    imu_read_angles(&a);
    mp_obj_t t[3] = {
        mp_obj_new_float(a.roll),
        mp_obj_new_float(a.pitch),
        mp_obj_new_float(a.yaw),
    };
    return mp_obj_new_tuple(3, t);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_0(mp_imu_read_angles_obj, mp_imu_read_angles);

STATIC const mp_rom_map_elem_t bpuppy_imu_globals_table[] = {
    { MP_ROM_QSTR(MP_QSTR___name__),    MP_ROM_QSTR(MP_QSTR_bpuppy_imu) },
    { MP_ROM_QSTR(MP_QSTR_init),        MP_ROM_PTR(&mp_imu_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_raw),    MP_ROM_PTR(&mp_imu_read_raw_obj) },
    { MP_ROM_QSTR(MP_QSTR_read_angles), MP_ROM_PTR(&mp_imu_read_angles_obj) },
};
STATIC MP_DEFINE_CONST_DICT(bpuppy_imu_globals, bpuppy_imu_globals_table);

const mp_obj_module_t bpuppy_imu_module = {
    .base = { &mp_type_module },
    .globals = (mp_obj_dict_t *)&bpuppy_imu_globals,
};

MP_REGISTER_MODULE(MP_QSTR_bpuppy_imu, bpuppy_imu_module);
