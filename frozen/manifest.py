# bDog Frozen modules manifest
freeze(".", "main.py")
freeze(".", "voltage.py")
freeze(".", "voice.py")
freeze(".", "poses.py")
freeze(".", "pwm_ext.py")
freeze(".", "camera_serial.py")
freeze(".", "camera_stream.py")
freeze(".", "calib_mag.py")
freeze(".", "balance.py")
# 航向锁定闭环 —— step1 产 err (heading_anchor) + step2 执行 (heading_follow)。
# KittenBlock 的「航向锁定 偏转 / 解除航向锁定」两个积木靠它们。
#
# ⚠ **临时移出 frozen (2026-10-02, debug 转向符号中)**: 源暂放 mpy_modules/,
#   用 bTool/_upload_heading.py 传到板子根目录 (VFS 优先, 板上那份生效)。
#   标定完成后: 移回 frozen/ 并取消下面两行的注释, 再重编译烧录。
# freeze(".", "heading_anchor.py")
# freeze(".", "heading_follow.py")
