# bDog Frozen modules manifest
freeze(".", "main.py")
freeze(".", "voltage.py")
freeze(".", "voice.py")
freeze(".", "poses.py")
freeze(".", "pwm_ext.py")
freeze(".", "camera_serial.py")
freeze(".", "camera_stream.py")
freeze(".", "set_heading.py")
freeze(".", "calib_mag.py")
freeze(".", "balance.py")
# 航向锁定闭环 —— step1 产 err (heading_anchor) + step2 执行 (heading_follow)。
# KittenBlock 的「航向锁定 偏转 / 解除航向锁定」两个积木靠它们。**必须冻结**:
# KittenBlock 只下载 /main.py, 不会把模块带上板 —— 没冻结就是 ImportError。
# ⚠ 工作副本在 mpy_modules/, 改了那边记得 cp 过来再编 (两份内容要一致)。
freeze(".", "heading_anchor.py")
freeze(".", "heading_follow.py")
