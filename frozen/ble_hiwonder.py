"""
bDog BLE — Hiwonder Wonderbot App 兼容 (C 驱动版)

使用 bpuppy_ble C 模块 (NimBLE), 实现 MechDog 协议:
  FFE0 Service
  FFE1: WRITE  — App 向设备发送 CMD 指令
  FFE2: NOTIFY — 设备向 App 回复

协议: CMD|<命令号>|<参数>|...|$
颜色滑块调速: 左=0, 右=20 (色相累积)
"""

import time
import bpuppy_ble
import bpuppy_motion
import bpuppy_imu


def _rgb_to_hue(r, g, b):
    """RGB(0-255) → 色相角度 0-360"""
    r, g, b = r / 255.0, g / 255.0, b / 255.0
    mx = max(r, g, b)
    mn = min(r, g, b)
    d = mx - mn
    if d == 0:
        return 0.0
    if mx == r:
        h = 60 * (((g - b) / d) % 6)
    elif mx == g:
        h = 60 * ((b - r) / d + 2)
    else:
        h = 60 * ((r - g) / d + 4)
    if h < 0:
        h += 360
    return h


class HiwonderBLE:
    """Hiwonder Wonderbot App BLE 遥控接口"""

    def __init__(self):
        self._on_cmd = None
        self._speed = 0.0       # 当前速度 0-12
        self._moving = False    # 当前是否运动中
        self._last_dir = 0      # 上一方向
        self._balance = False   # 自稳开关
        bpuppy_ble.start()
        print("[BLE] NimBLE 已启动")

    def connected(self):
        return bpuppy_ble.connected()

    def stop(self):
        """关闭 BLE: 停止广播 + 断开连接 (NimBLE 栈保留, 可再 start)"""
        bpuppy_ble.stop()
        self._balance = False
        print("[BLE] 已停止 (广播停, 连接断)")

    def send(self, data):
        if isinstance(data, str):
            bpuppy_ble.send(data)

    def on_command(self, cb):
        self._on_cmd = cb

    def check(self):
        # 自稳: 每轮询周期读 IMU → 推入 set_body_pose
        if self._balance:
            try:
                a = bpuppy_imu.read_angles()
                bpuppy_motion.set_body_pose(a[0], a[1], 0)
            except Exception:
                pass

        raw = bpuppy_ble.recv()
        if not raw:
            return

        for msg in raw.split("$"):
            msg = msg.strip()
            if not msg.startswith("CMD"):
                continue
            try:
                parts = msg.split("|")
                if len(parts) < 2:
                    continue
                cmd_id = int(parts[1])
                args = parts[2:] if len(parts) > 2 else []
            except (ValueError, IndexError):
                continue

            if self._on_cmd:
                try:
                    self._on_cmd(cmd_id, args)
                except Exception as e:
                    print("[BLE] 回调: %s" % e)
            else:
                self._handle(cmd_id, args)

    def _handle(self, cmd_id, args):
        if cmd_id == 0:
            self.send("CMD|0|mechdog|$")

        elif cmd_id == 1:
            # CMD|1|DATA|... → DATA=3 自稳开关
            try:
                data = int(args[0]) if args else 0
            except ValueError:
                return
            if data == 3:
                try:
                    val = int(args[1]) if len(args) > 1 else 0
                except ValueError:
                    return
                self._balance = (val == 1)
                print("[BLE] 自稳: %s" % ("开" if self._balance else "关"))
                if not self._balance:
                    bpuppy_motion.set_body_pose(0, 0, 0)

        elif cmd_id == 2:
            try:
                a = int(args[0]) if args else 0
            except ValueError:
                return
            if a == 3:
                print("[BLE] stand")
                bpuppy_motion.stand_up()
            elif a == 4:
                print("[BLE] crouch")
                bpuppy_motion.set_gait("crouch")

        elif cmd_id == 3:
            try:
                d = int(args[0]) if args else 0
            except ValueError:
                return

            s = self._speed
            if d == 0:
                s = 0

            self._last_dir = d
            turn = {1: 0.75, 2: 0.25, 4: -0.25, 5: -0.75,
                    6: -0.25, 8: 0.25}.get(d, 0)
            bpuppy_motion.set_turn(turn)

            # stride 符号决定方向: 正=前, 负=后  (GO 忽略 stride 值，但后续会用到)
            stride_dir = -70 if d in (6, 7, 8) else 70

            print("[BLE] 方向=%d speed=%.1f stride=%d turn=%.1f" % (d, s, stride_dir, turn))
            if d == 0:
                bpuppy_motion.set_params(0, 0, 70)  # speed=0, 保持站立高度
                bpuppy_motion.set_gait("stand")
                self._moving = False
            else:
                bpuppy_motion.set_params(s, stride_dir, 70)  # stride/height 由 GO 自适应
                bpuppy_motion.set_gait("go")
                self._moving = True

        elif cmd_id == 4:
            # 颜色滑块 → 调速: 红紫蓝绿黄红, 绝对位置
            try:
                r = int(args[1]) if len(args) > 1 else 0
                g = int(args[2]) if len(args) > 2 else 0
                b = int(args[3]) if len(args) > 3 else 0
            except ValueError:
                return
            h = _rgb_to_hue(r, g, b)
            # hue 两端都是 0°, 用 G 区分: 左端 G=0→speed=0, 右端 G>0→speed=12
            self._speed = 12.0 if (h < 5.0 and g > 0) else ((360.0 - h) % 360.0) / 360.0 * 12.0

            if self._moving:
                stride_dir = -70 if self._last_dir in (6, 7, 8) else 70
                print("[BLE] set speed=%.1f stride=%d (from slider)" % (self._speed, stride_dir))
                bpuppy_motion.set_params(self._speed, stride_dir, 60)

        elif cmd_id == 6:
            self.send("CMD|6|85|$")


def run_ble_task(ble, poll_ms=50):
    """后台 BLE 轮询 (在 _thread 中运行)"""
    while True:
        try:
            ble.check()
        except Exception as e:
            print("[BLE] 轮询: %s" % e)
        time.sleep(poll_ms / 1000.0)
