
/*
 * bPuppy IMU 传感器驱动 — MPU6050 / MPU9250 自适应
 *
 * I2C 接口引脚由 Python 层初始化时指定。
 * 使用 ESP-IDF legacy I2C API (driver/i2c.h) —— 与 MicroPython machine.I2C 同一套,
 * 不能换 driver_ng (两者互斥会断言重启)
 *
 * WHO_AM_I 自动识别: 0x68/0x69 = MPU6050, 0x70 = MPU6500, 0x71/0x73 = MPU9250
 * ⚠ **有没有磁力计不看型号** —— 一律问 imu_has_mag()。6500 / 9250 模块都可能带 AK8963。
 *   有磁力计 → mag_* 可用, Mahony 9轴融合, yaw 不漂
 *   没有     → mag_* 恒 0, 6轴姿态, yaw 有漂移
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

// 初始化 IMU (I2C 接口), 幂等: 已初始化则直接返回
// port: I2C 端口号 (I2C_NUM_0 或 I2C_NUM_1)
// sda_pin, scl_pin: GPIO 引脚
// addr: **当前被忽略** —— 驱动自己按 0x68/0x69 扫 (见 imu_driver.c)
void imu_init(uint8_t port, uint8_t sda_pin, uint8_t scl_pin, uint8_t addr);

// 是否已初始化 (供依赖模块按需启动)
bool imu_is_ready(void);

// 已识别的芯片型号: "mpu6050" / "mpu6500" / "mpu9250" / "unknown" (init 后有效)
//   ⚠ 只说核心是谁, **不代表有没有磁力计** —— 那由 imu_has_mag() 回答
const char *imu_chip_name(void);

// 是否有磁力计 (init 时真去问 AK8963 的 WHO_AM_I, 认出来才为 true)
//   ⚠ 别用 chip 名字推断: 实测有 0x70 (6500 核心) 的模块带真 AK8963,
//     真 6500 又没有。判错会让磁力计校准死循环。
bool imu_has_mag(void);

// 停止 AHRS 任务, 释放 IMU (可重新 init)
void imu_stop(void);

// 磁力计是否参与 roll/pitch 融合. balance 用 OFF(只修yaw), 避免磁力计残差拉偏 roll/pitch
void imu_set_mag_fusion(bool enable);

// 读取原始数据 (9轴 + 温度)
void imu_read_raw(imu_raw_data_t *data);

// 读取姿态角 (AHRS 由独立任务持续跑, 调用方不需要喂它)
void imu_read_angles(imu_angles_t *angles);

// 校准: 采集静止状态下 N 次数据计算零偏
// 加速度计 + 陀螺仪: 均值归零
void imu_calibrate(int samples);

// 磁力计 3D 椭球拟合校准 (引导式, SLV0 后台读持续运行)
void imu_start_mag_cal(void);
// 采集一次磁力计数据, 返回统计: (count, r_x, r_y, r_z, min_x, max_x, min_y, max_y, min_z, max_z)
// 调用前需先 start_mag_cal, 调用后通过指针获取结果
bool imu_mag_cal_collect(int *count,
                          float *r_x, float *r_y, float *r_z,
                          float *mn_x, float *mx_x,
                          float *mn_y, float *mx_y,
                          float *mn_z, float *mx_z);
// 椭球拟合 + 写 NVS + 恢复 IMU 任务. 返回残差 (-1=fail)
float imu_finish_mag_cal(void);

// 软铁 3×3 矩阵 (行优先), 默认单位阵
extern float g_mag_soft_iron[9];

#ifdef __cplusplus
}
#endif
