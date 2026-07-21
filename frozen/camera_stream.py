"""
camera_stream — WiFi 实时图传 (MJPEG over HTTP)

ESP32-S3 启动 WiFi AP，手机连上后在浏览器中查看摄像头实时画面。
底层使用 OV2640 硬件 JPEG 编码，CPU 零参与。

用法:
    import camera_stream
    camera_stream.start()
    # 手机连 WiFi "bPuppy" (密码 12345678)
    # 浏览器打开 http://192.168.4.1
    camera_stream.stop()
"""

import bpuppy_camera
import network
import socket
import time

try:
    import _thread
except ImportError:
    _thread = None


# ---- 状态 ----
_ap = None
_running = False
_ssid = "bPuppy"
_password = "12345678"

# ---- MJPEG 常量 ----
_BOUNDARY = "--bPuppyFrame"

_HTTP_MJPEG_HEADER = (
    "HTTP/1.0 200 OK\r\n"
    "Content-Type: multipart/x-mixed-replace; boundary=" + _BOUNDARY + "\r\n"
    "\r\n"
).encode()

_PART_TEMPLATE = (
    "\r\n--" + _BOUNDARY + "\r\n"
    "Content-Type: image/jpeg\r\n"
    "Content-Length: {}\r\n"
    "\r\n"
)

_HTML_PAGE = """\
HTTP/1.0 200 OK\r
Content-Type: text/html\r
\r
<!DOCTYPE html>
<html><head><meta name="viewport" content="width=device-width,initial-scale=1">
<title>bPuppy Camera</title></head>
<body style="margin:0;background:#000">
<img src="/stream" style="width:100%;display:block">
</body></html>"""


# ---- HTTP 响应 ----

def _send_html(client):
    """返回简单 HTML 页面，内嵌 <img src='/stream'> 触发浏览器请求 MJPEG 流"""
    try:
        client.send(_HTML_PAGE.encode())
    except OSError:
        pass
    client.close()


def _send_stream(client):
    """持续发送 MJPEG 流，直到客户端断开或 g_running 变 False"""
    global _running
    client.settimeout(3.0)

    try:
        client.send(_HTTP_MJPEG_HEADER)
    except OSError:
        client.close()
        return

    # 丢掉缓冲区旧帧（双缓冲可能存着几秒前的画面）
    for _ in range(3):
        bpuppy_camera.capture()
        time.sleep(0.03)

    while _running:
        result = bpuppy_camera.capture()
        if result is None:
            time.sleep(0.05)
            continue

        data = result[0]
        try:
            client.sendall(_PART_TEMPLATE.format(len(data)).encode())
            client.sendall(data)
        except OSError:
            # 客户端断开或超时
            break

        time.sleep(0.07)  # ~14fps

    client.close()


# ---- HTTP 服务器主循环 ----

def _accept_loop():
    """后台线程：accept HTTP 连接并分发到 / 或 /stream"""
    global _running

    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 80))
    s.listen(2)

    while _running:
        try:
            s.settimeout(1.0)
            client, addr = s.accept()
        except OSError:
            # timeout — 检查 _running 标志后继续
            continue

        try:
            client.settimeout(2.0)
            request = client.recv(256).decode("utf-8", "ignore")
        except OSError:
            client.close()
            continue

        if not request:
            client.close()
            continue

        # 只解析请求行: "GET /stream HTTP/1.1"
        lines = request.split("\r\n")
        if not lines:
            client.close()
            continue

        first_line = lines[0]
        if "/stream" in first_line:
            _send_stream(client)
        else:
            _send_html(client)

    s.close()


# ---- 公开 API ----

def start(ssid="bPuppy", password="12345678"):
    """启动 WiFi 图传

    参数:
        ssid:     WiFi 热点名称
        password: WiFi 密码 (至少 8 位)
    """
    global _ap, _running, _ssid, _password

    if _running:
        print("camera_stream: already running")
        return

    if _thread is None:
        print("camera_stream: _thread not available")
        return

    _ssid = ssid
    _password = password

    # 1. 摄像头 — JPEG SVGA 800x600
    if not bpuppy_camera.is_ready():
        bpuppy_camera.init_adv(bpuppy_camera.SVGA, 10, 2, 20000000, bpuppy_camera.JPEG)

    # 2. WiFi AP
    _ap = network.WLAN(network.AP_IF)
    _ap.active(True)
    _ap.config(
        essid=_ssid,
        password=_password,
        authmode=network.AUTH_WPA2_PSK,
        max_clients=4,
    )
    time.sleep(0.5)

    # 3. HTTP 后台线程
    _running = True
    _thread.start_new_thread(_accept_loop, ())
    print("camera_stream: started — http://192.168.4.1")


def stop():
    """停止图传 — 关闭 WiFi + 释放摄像头"""
    global _ap, _running

    if not _running:
        return

    _running = False
    time.sleep(0.4)  # 给 accept 循环时间退出

    if _ap:
        try:
            _ap.active(False)
        except Exception:
            pass
        _ap = None

    try:
        bpuppy_camera.deinit()
    except Exception:
        pass

    print("camera_stream: stopped")


def state():
    """返回当前状态: 'running' 或 'stopped'"""
    return "running" if _running else "stopped"
