# IMU MPU9250 调试记录

## 硬件

- 芯片: MPU9250 (WHOAMI=0x71), AK8963 磁力计 (WHOAMI=0x48)
- 接口: I2C, SDA=GPIO3, SCL=GPIO14
- I2C 地址: MPU9250@0x68, AK8963@0x0C (I2C Master 模式)
- I2C 速率: **100kHz**（400kHz 不稳定，内部上拉偏弱）

## 已解决

| 问题 | 原因 | 修复 |
|------|------|------|
| 磁力计全零/不变 | `ak8963_init` 结尾没配 `slv0_bg()`（SLV0 后台 8 字节读） | 加 `slv0_bg()` 到 init 末尾 |
| Yaw 锁死不跟旋转 | Mahony KP=1.0 KI=1.0 太大，磁力计修正吃掉陀螺积分 | KP→0.5, KI→0.01 |
| BYPASS 模式不通 | 模块 EDA/ECL 引脚悬空，未桥接到 SDA/SCL | 必须用 I2C Master 模式 |
| I2C 400kHz 不稳定 | ESP32-S3 内部上拉不足 | 降到 100kHz |
| 校准后 mag 卡死 | `calibrate_mag()` 停/启 SLV0 未正确恢复 | 删除 calibate_mag，硬铁校准暂跳过 |
| managed_components 重下载 | 误删目录导致 500MB+ 重下 | 只删 build，不动 managed_components |

## 待解决

| 问题 | 现象 | 方向 |
|------|------|------|
| **Yaw 静止时跳动** | 不转也 ±3° 来回跳(90→89→75→58…) | KP/KI 需细调(当前 KP=0.5 KI=0.01) |
| **磁力计命中率** | AK8963@100Hz vs IMU 任务@100Hz, SLV0 后台读清 DRDY 太快 | 可能需用 I2C_MST_DELAY_CTRL 降低 SLV0 读速 |
| **硬铁校准** | 暂无可靠方法（SLV0 后台读干扰校准数据采集） | 需停 SLV0 再用按需触读做校准 |
| **ASA 校准** | AK8963 FUSE ROM 读失败(全 0) | 可能需 100ms 级延迟 |
| **轴重映射** | 芯片 Y=狗尾(后), X=狗左 | 已做映射(前=-Y, 右=-X, 上=Z)，需实际验证 |

## 当前驱动结构

```
imu_driver.c (~350行)
  ├── imu_init()          MPU9250 I2C Master 初始化 + AK8963 init
  ├── ak8963_init()       WHOAMI 检查 + 软复位 + 连续测量 + slv0_bg()
  ├── slv0_write/read/bg  SLV0 操作（先停再配）
  ├── read_all()          Accel+Gyro(14字节) + EXT_SENS_DATA(8字节) 分开读
  ├── mahony_update()     Mahony AHRS 9轴融合
  ├── imu_task_main()     100Hz FreeRTOS 后台任务
  ├── imu_calibrate()     Gyro/Accel 零偏校准（静止）
  └── MicroPython 绑定    init, read_raw, read_angles, calibrate
```

## Git 记录

- `0b570fa` - 当前工作版: mag 读数正常, KP=0.5 KI=0.01
- `c713c87` - 上一版: KP=1.0 KI=1.0(mag 正常但 yaw 锁死)

## 当前状态 (2026-07-22)

✅ mag 数据正常刷新，永磁铁测试灵敏
✅ gyro 校准正常（calibrate 300）
✅ 旋转时 yaw 有响应（90→58→45）
⚠️ **静止时 yaw 跳动**：不放也 ±3° 来回跳，偶发大幅跳变（90→75→58→45）

## 明天计划

1. **调 KP/KI**：先不动，直接用 `test_heading` 测航向锁定看 ±5° 死区能否稳住
2. **如不行**：KI 降到 0.001（减少磁力计修正累积），KP 保持 0.5
3. **磁力计命中率**：用 I2C_MST_DELAY_CTRL 降低 SLV0 读速，拉长 DRDY 窗口
4. **硬铁校准**：停 SLV0 → 按需触读采集 min/max → 恢复 SLV0
5. 每步 commit
