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
# ⚠ **源就在 frozen/ 里, 不要放第二份到 mpy_modules/**: 两份会漂, 而且
#   **VFS 优先于 frozen** —— 传上去的那份会盖住固件版, 表现是"编了没生效"。
freeze(".", "heading_anchor.py")
freeze(".", "heading_follow.py")
