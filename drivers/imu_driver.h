/*
 * bDog IMU 传感器驱动 — 头文件
 *
 * 支持 I2C 接口的 6 轴 IMU（MPU6050 / MPU9250 / ICM-42688 等）
 * C 层负责原始数据读取 + 姿态解算（互补滤波）
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
    float temp_c;       // 温度 (°C)
} imu_raw_data_t;

typedef struct {
    float roll;         // 横滚角 (deg)
    float pitch;        // 俯仰角 (deg)
    float yaw;          // 偏航角 (deg) — 无磁力计时会漂移
} imu_angles_t;

/* ---- API ---- */

// 初始化 IMU (I2C 接口)
// port: I2C 端口号 (I2C_NUM_0 或 I2C_NUM_1)
// sda_pin, scl_pin: GPIO 引脚
// addr: I2C 地址 (MPU6050 默认 0x68)
void imu_init(uint8_t port, uint8_t sda_pin, uint8_t scl_pin, uint8_t addr);

// 读取原始数据
void imu_read_raw(imu_raw_data_t *data);

// 读取姿态角 (需要 imu_update() 持续运行)
void imu_read_angles(imu_angles_t *angles);

// 校准（采集静止状态下 N 次数据，计算零偏）
void imu_calibrate(int samples);

// FreeRTOS 任务 — 以固定频率读取 IMU 并更新姿态估计
void imu_task_start(int freq_hz);

#ifdef __cplusplus
}
#endif
