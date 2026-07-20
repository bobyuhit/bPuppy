"""
capture.py — PC 端拍照工具

打开串口后保持连接，按 Enter 拍照，不重启 ESP32。

用法:
    python capture.py COM14
    > (按 Enter)  → 拍照
    > (按 Enter)  → 再拍
    > q           → 退出
"""

import serial
import base64
import sys
import os
import time


def snap_one(ser):
    """发送拍照命令，返回 (data, w, h, fmt) 或 None"""
    ser.reset_input_buffer()
    ser.write(b"import camera_serial\r\n")
    time.sleep(0.15)
    ser.write(b"camera_serial.snap()\r\n")
    time.sleep(0.15)

    capturing = False
    b64_lines = []
    w, h, fmt, size = 0, 0, 0, 0

    deadline = time.time() + 15
    while time.time() < deadline:
        line = ser.readline()
        if not line:
            continue
        text = line.decode("utf-8", errors="ignore").strip()
        if not text:
            continue

        if text.startswith("<<<IMG "):
            parts = text[7:].split()
            if len(parts) >= 4:
                w, h, fmt, size = int(parts[0]), int(parts[1]), int(parts[2]), int(parts[3])
            if size == 0:
                return None
            capturing = True
            continue

        if capturing and text.startswith(">>>IMG"):
            break

        if capturing:
            b64_lines.append(text)

    if not b64_lines:
        return None

    b64_data = "".join(b64_lines)
    return base64.b64decode(b64_data), w, h, fmt


def main():
    if len(sys.argv) < 2:
        print("用法: python capture.py COM14")
        sys.exit(1)

    port = sys.argv[1]
    baud = 115200

    # 打开串口（尝试阻止复位）
    ser = serial.Serial(port, baud, timeout=1, dsrdtr=False)
    time.sleep(0.5)

    # 消化启动残留数据
    ser.reset_input_buffer()

    print(f"已连接 {port}")
    print("按 Enter 拍照, q 退出")
    print()

    count = 1
    while True:
        cmd = input(f"[{count}] > ").strip().lower()
        if cmd == "q":
            break

        print("  拍照中...", end=" ", flush=True)
        result = snap_one(ser)
        if result is None:
            print("失败!")
        else:
            data, w, h, fmt = result
            fmt_map = {4: "JPEG", 1: "RGB565", 5: "GRAYSCALE"}
            fmt_name = fmt_map.get(fmt, f"unk({fmt})")
            fn = f"capture_{count:03d}_{w}x{h}_{fmt_name}.jpg"
            with open(fn, "wb") as f:
                f.write(data)
            print(f"OK → {fn}  ({len(data)} bytes)")
            os.startfile(fn)
            count += 1

    ser.close()
    print("已断开")


if __name__ == "__main__":
    main()
