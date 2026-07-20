"""
camera_serial — 串口拍照回传

用法:
    import camera_serial
    camera_serial.snap()
"""

import bpuppy_camera
import binascii
import time


def snap():
    if not bpuppy_camera.is_ready():
        bpuppy_camera.init()

    # 清掉缓冲区旧帧 (双缓冲可能存着几秒前的画面)
    for _ in range(3):
        bpuppy_camera.capture()
        time.sleep(0.03)

    # 现在取真正的新帧
    data, w, h, fmt = bpuppy_camera.capture()
    if not data:
        print("<<<IMG 0 0 0 0>>>IMG")
        return

    print(f"<<<IMG {w} {h} {fmt} {len(data)}")
    for i in range(0, len(data), 384):
        print(binascii.b2a_base64(data[i:i + 384]).decode().strip())
    print(">>>IMG")
