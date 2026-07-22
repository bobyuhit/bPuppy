
/*
 * bPuppy IMU 传感器驱动 — MPU9250 9轴 IMU + AK8963 磁力计
 *
 * I2C 接口引脚由 Python 层初始化时指定。
 * 使用 ESP-IDF v5.x I2C Master API (driver/i2c_master.h)
 *
 * 数据格式:
 *   加速度: 16-bit signed, 量程 ±8g → 4096 LSB/g
 *   陀螺仪: 16-bit signed, 量程 ±500dps → 65.5 LSB/dps
 *   磁力计: 16-bit signed (AK8963), 量程 ±4800μT → 0.15 μT/LSB
 *
 * 姿态解算: Mahony AHRS 滤波器 (9轴融合, 无漂移 yaw)
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---- IMU 数据结构 ---- */
typedef struct {
    float accel_x;      // 加速度 (m/s²)
    float accel_y;
    float accel_z;
    float gyro_x;       // 角速度 (rad/s)
    float gyro_y;
    float gyro_z;
    float mag_x;        // 磁力计 (μT)
    float mag_y;
    float mag_z;
    float temp_c;       // 温度 (°C)
} imu_raw_data_t;

typedef struct {
    float roll;         // 横滚角 (deg)
    float pitch;        // 俯仰角 (deg)
    float yaw;          // 偏航角 (deg) — 磁力计修正, 无漂移
} imu_angles_t;

/* ---- API ---- */

// 初始化 IMU (I2C 接口)
// port: I2C 端口号 (I2C_NUM_0 或 I2C_NUM_1)
// sda_pin, scl_pin: GPIO 引脚
// addr: I2C 地址 (MPU9250 默认 0x68)
void imu_init(uint8_t port, uint8_t sda_pin, uint8_t scl_pin, uint8_t addr);

// 读取原始数据 (9轴 + 温度)
void imu_read_raw(imu_raw_data_t *data);

// 读取姿态角 (需要 imu_read_raw() 持续调用 → 内部 AHRS 滤波器)
void imu_read_angles(imu_angles_t *angles);

// 校准: 采集静止状态下 N 次数据计算零偏
// 加速度计 + 陀螺仪: 均值归零
// 磁力计: 硬铁校准 (max+min/2)
void imu_calibrate(int samples);

#ifdef __cplusplus
}
#endif
