"""
bPuppy 传感器 — Python 层 API

封装 bpuppy_imu C 模块，提供:
- IMU 数据读取
- 姿态估计
- 碰撞检测（基于加速度突变）

TODO: 完善传感器数据处理
"""


class IMU:
    """IMU 传感器封装"""
    def __init__(self, port=0, sda=14, scl=21, addr=0x68):  # V3.0 硬件: SDA=14, SCL=21 (电池检测走 GPIO3=ADC1)
        self.port = port
        self.sda = sda
        self.scl = scl
        self.addr = addr
        self._initialized = False

    def init(self):
        """初始化 IMU"""
        import bpuppy_imu
        bpuppy_imu.init(self.port, self.sda, self.scl, self.addr)
        self._initialized = True

    def chip(self):
        """已识别芯片型号 → "mpu6050" / "mpu9250" / "unknown" """
        if not self._initialized:
            self.init()
        import bpuppy_imu
        return bpuppy_imu.get_chip()

    def has_mag(self):
        """是否有磁力计 → MPU9250=True, MPU6050=False"""
        if not self._initialized:
            self.init()
        import bpuppy_imu
        return bpuppy_imu.has_mag()

    def read_raw(self):
        """读取原始数据 → (accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z), temp"""
        if not self._initialized:
            self.init()
        import bpuppy_imu
        return bpuppy_imu.read_raw()

    def read_accel(self):
        """读取加速度 (x, y, z) m/s²"""
        accel, _, _ = self.read_raw()
        return accel

    def read_gyro(self):
        """读取角速度 (x, y, z) rad/s"""
        _, gyro, _ = self.read_raw()
        return gyro


class CollisionDetector:
    """碰撞检测 — 基于加速度阈值"""
    def __init__(self, threshold=15.0):
        self.threshold = threshold  # m/s²，超过此值视为碰撞
        self.imu = IMU()

    def check(self):
        """检查是否发生碰撞 → True/False"""
        ax, ay, az = self.imu.read_accel()
        magnitude = (ax*ax + ay*ay + az*az) ** 0.5
        return magnitude > self.threshold


# 全局传感器实例
imu = IMU()
collision = CollisionDetector()
