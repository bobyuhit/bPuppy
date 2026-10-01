"""方向锚定 (step1) —— 产出一个"固定目标航向"的 err 来源

**它自己不跑循环、不发指令** —— 只返回一个 `err_fn` 交给 heading_follow.run()。

    import heading_anchor as anc, heading_follow as flw

    flw.run(anc.anchor(30))     # 转 30° 后沿该方向一直走
    flw.run(anc.hold())         # 保持当前朝向 (= anchor(0))

★ 收的是**角度量**("要转多少"), 不是绝对航向 —— 调用方不用知道当前朝向是几度,
  ±180 回绕也由 heading_follow 那边统一处理。

⚠ 锚定需要 IMU。上电**不自动** init, 这里会顺手 init 一次(幂等)。
"""
import bpuppy_imu as imu

_I2C_PORT = 0
_SDA_PIN  = 14
_SCL_PIN  = 21
_ADDR     = 0x68


def yaw():
    """当前航向 (度, ±180)。"""
    return imu.read_angles()[2]


def anchor(offset_deg, verbose=False):
    """返回一个 err_fn: 目标 = **调用这一刻**的 yaw + offset_deg。

    目标在此刻**锁存**, 之后每拍由 err_fn 现算 `target − 当前 yaw` ——
    所以转到目标后会**一直保持**这个方向(被打滑/被推也会纠回来),
    不是"转一次就完"。想换目标就再调一次 anchor()。

    verbose=True 时每拍打印当前 yaw 和目标 —— yaw 只有这一层知道
    (heading_follow 只收 err, 不该去碰 IMU)。调参时一起开::

        flw.run(anc.anchor(30, verbose=True), verbose=True)
    """
    imu.init(_I2C_PORT, _SDA_PIN, _SCL_PIN, _ADDR)     # 幂等
    target = yaw() + offset_deg
    if verbose:
        print('>>>>> 锚定: yaw=%8.2f + %.1f  =>  目标 yaw=%8.2f'
              % (target - offset_deg, offset_deg, target))

    def err_fn():
        # ⚠ IMU 没就绪 ⇒ 返回 None("这一拍没数据"), heading_follow 会当 0 处理 ⇒ 直走。
        #   不检查的话:**未 init 时 read_angles() 恒返回 (0,0,0)**, 闭环会把这个假 0
        #   当成"当前朝向 0 度", 于是一直输出一个恒定的转角转到天荒地老 ——
        #   实测踩过(`i.init()` 漏调, err 恒 30 跑了整整 6 秒)。
        #   固件版有看门狗(yaw 无效 1s 自动解除), 这个是 Python 版的对应物。
        if not imu.is_ready():
            return None
        y = yaw()
        if verbose:
            print('   yaw=%8.2f   target=%8.2f' % (y, target))
        return target - y

    return err_fn


def hold():
    """保持当前朝向(= anchor(0))。"""
    return anchor(0.0)
