"""
camera_stream — WiFi 实时图传 + 网页遥控器

用法:
    import camera_stream
    camera_stream.start()
    # http://192.168.4.1
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

_ap = None
_running = False
_lock = None
_g_speed = 0
_g_turn = 0.0
_g_stride = 70
_g_height = 70
_g_gait = "stand"
_g_stream_client = None  # 当前唯一的流客户端

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

# ---- 网页遥控器 ----
_HTML_PAGE = """\
HTTP/1.0 200 OK\r\n\
Content-Type: text/html; charset=utf-8\r\n\
\r\n\
<!DOCTYPE html>
<html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>bPuppy</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#1a1a1a;color:#ddd;font:14px/1.4 system-ui,sans-serif;margin:0}
img{width:100%;max-width:400px;display:block;transform:scaleX(-1)}
#pad{width:100%;max-width:400px;background:#222;padding:8px 10px}
.r{display:flex;justify-content:center;gap:6px;margin:4px 0}
a{text-decoration:none;color:#ddd;border-radius:10px;display:inline-flex;align-items:center;justify-content:center;font-size:14px;height:42px;min-width:52px;padding:0 8px}
a:active{opacity:0.6}
.d{width:64px;height:56px;line-height:56px;border-radius:14px;font-size:26px;background:#345}
.ds{background:#622;font-size:16px;font-weight:bold;height:56px;line-height:56px}
.t{font-size:11px;color:#aaa;margin-top:-2px}
.p{background:#3a3a3a;font-size:14px;height:38px;line-height:38px;border-radius:8px;min-width:58px}
.w{background:#543010;font-size:14px;height:38px;line-height:38px;border-radius:8px;min-width:58px}
.sl{display:flex;align-items:center;gap:8px}
.sl input{flex:1;height:30px;accent-color:#4a4}
.sl span{font-size:20px;font-weight:bold;color:#4f4;min-width:28px;text-align:right}
.sl label{font-size:12px;color:#888}
</style>
</head><body>

<img src="/stream">
<iframe name="f" style="display:none"></iframe>

<div id="pad">

  <form class="sl" action="/cmd" method="get" target="f">
    <label>速</label>
    <input type="hidden" name="set" value="1">
    <input type="range" name="speed" min="0" max="10" step="0.5" value="3"
      onchange="this.form.submit()"
      oninput="document.getElementById('sv').innerHTML=this.value">
    <span id="sv">3</span>
  </form>

  <div class="r">
    <a href="/cmd?stride=70&turn=-0.5"  class="d" target="f">&#x2196;</a>
    <a href="/cmd?stride=70&turn=0"     class="d" target="f">&#x25B2;</a>
    <a href="/cmd?stride=70&turn=0.5"   class="d" target="f">&#x2197;</a>
  </div>


  <div class="r">
    <a href="/cmd?stride=0&turn=-0.8"   class="d" target="f">&#x25C0;</a>
    <a href="/cmd?gait=stand" class="d ds" target="f">停</a>
    <a href="/cmd?stride=0&turn=0.8"    class="d" target="f">&#x25B6;</a>
  </div>

  <div class="r">
    <a href="/cmd?stride=-70&turn=-0.5" class="d" target="f">&#x2199;</a>
    <a href="/cmd?stride=-70&turn=0"    class="d" target="f">&#x25BC;</a>
    <a href="/cmd?stride=-70&turn=0.5"  class="d" target="f">&#x2198;</a>
  </div>

  <div class="r">
    <a href="/cmd?gait=stand"  class="p" target="f">站立</a>
    <a href="/cmd?gait=sit"    class="p" target="f">坐下</a>
    <a href="/cmd?gait=crouch" class="p" target="f">蹲下</a>
  </div>

  <div class="r">
    <a href="/cmd?gait=play" class="w" target="f">玩</a>
    <a href="/cmd?wave=1" class="w" target="f">挥手</a>
  </div>

</div>
</body></html>"""


def _parse_cmd(path):
    global _g_speed, _g_turn, _g_stride, _g_height, _g_gait

    qs = path[5:] if path.startswith("/cmd?") else path
    # print("CMD:", qs)

    params = {}
    for part in qs.split("&"):
        kv = part.split("=", 1)
        if len(kv) == 2:
            params[kv[0]] = kv[1]
    # print("PARAMS:", params)

    try:
        import bpuppy_motion
    except ImportError:
        return "motion N/A"

    with _lock:
        # === 挥手 ===
        if "wave" in params:
            bpuppy_motion.set_gait("wave")
            _g_gait = "wave"
            return "OK:wave"

        # === 急停 ===
        if "stop" in params:
            bpuppy_motion.emergency_stop()
            bpuppy_motion.set_gait("stand")
            _g_gait = "stand"
            return "OK:stop"

        # === 仅存值，不调运动（滑块用） ===
        if "set" in params:
            if "speed" in params:
                _g_speed = float(params["speed"])
            if "turn" in params:
                _g_turn = float(params["turn"])
            if "stride" in params:
                _g_stride = float(params["stride"])
            if "height" in params:
                _g_height = float(params["height"])
            return "OK:set"

        # === 步态切换 ===
        if "gait" in params:
            g = params["gait"]
            bpuppy_motion.set_gait(g)
            _g_gait = g
            return "OK:gait"

        # === 方向/移动指令：应用所有已存值 ===
        # 存新值
        if "speed" in params:
            _g_speed = float(params["speed"])
        if "turn" in params:
            _g_turn = float(params["turn"])
        if "stride" in params:
            _g_stride = float(params["stride"])
        if "height" in params:
            _g_height = float(params["height"])

        # 切 go
        if _g_gait not in ("go", "walk", "trot"):
            _g_gait = "go"
            bpuppy_motion.set_gait("go")

        # 应用
        bpuppy_motion.set_params(abs(_g_speed), _g_stride, _g_height)
        bpuppy_motion.set_turn(_g_turn)

    return "OK"


def _send_html(client):
    raw = _HTML_PAGE.encode("utf-8")
    total = len(raw)
    sent = 0
    data = raw
    while data:
        try:
            n = client.send(data)
            if n <= 0:
                break
            sent += n
            data = data[n:]
        except OSError:
            break
    print("HTML: total=%d sent=%d" % (total, sent))
    client.close()


def _send_stream(client):
    global _g_speed, _g_turn, _g_gait, _g_stride, _g_height, _g_stream_client

    # 关闭旧流连接
    old = _g_stream_client
    _g_stream_client = client
    if old:
        try:
            old.close()
        except Exception:
            pass

    client.settimeout(3.0)

    try:
        client.send(_HTTP_MJPEG_HEADER)
    except OSError:
        client.close()
        return

    for _ in range(3):
        bpuppy_camera.capture()
        time.sleep(0.03)

    last_gait = None
    last_speed = None
    last_turn = None

    while _running:
        try:
            import bpuppy_motion
            with _lock:
                if last_gait != _g_gait:
                    bpuppy_motion.set_gait(_g_gait)
                    last_gait = _g_gait
                    last_speed = None
                if last_speed != _g_speed or last_turn != _g_turn:
                    bpuppy_motion.set_params(abs(_g_speed), _g_stride, _g_height)
                    bpuppy_motion.set_turn(_g_turn)
                    last_speed = _g_speed
                    last_turn = _g_turn
        except ImportError:
            pass

        result = bpuppy_camera.capture()
        if result is None:
            time.sleep(0.05)
            continue

        data = result[0]
        try:
            client.sendall(_PART_TEMPLATE.format(len(data)).encode())
            client.sendall(data)
        except OSError:
            break
        time.sleep(0.07)

    client.close()


def _accept_loop():
    global _running

    s = socket.socket()
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", 80))
    s.listen(3)

    while _running:
        try:
            s.settimeout(1.0)
            client, addr = s.accept()
        except OSError:
            continue

        try:
            client.settimeout(2.0)
            request = client.recv(512).decode("utf-8", "ignore")
        except OSError:
            client.close()
            continue

        if not request:
            client.close()
            continue

        lines = request.split("\r\n")
        if not lines:
            client.close()
            continue

        first_line = lines[0]
        # print("HTTP:", first_line)

        if "/stream" in first_line:
            if _thread:
                try:
                    _thread.start_new_thread(_send_stream, (client,))
                except OSError:
                    client.close()
            else:
                _send_stream(client)

        elif "/cmd" in first_line:
            try:
                path = first_line.split(" ")[1]
                result = _parse_cmd(path)
                resp = "HTTP/1.0 200 OK\r\n\r\n" + result
                client.send(resp.encode("utf-8"))
            except Exception as e:
                print("CMD ERR:", e)  # DEBUG
            client.close()

        else:
            _send_html(client)

    s.close()


def start(ssid="bPuppy", password="12345678"):
    global _ap, _running, _lock

    if _running:
        print("camera_stream: already running")
        return

    if _thread is None:
        print("camera_stream: _thread not available")
        return

    _lock = _thread.allocate_lock()

    if not bpuppy_camera.is_ready():
        bpuppy_camera.init_adv(bpuppy_camera.SVGA, 10, 2, 20000000, bpuppy_camera.JPEG)

    _ap = network.WLAN(network.AP_IF)
    _ap.active(True)
    _ap.config(essid=ssid, password=password, authmode=network.AUTH_WPA2_PSK, max_clients=4)
    time.sleep(0.5)

    _running = True
    _thread.start_new_thread(_accept_loop, ())
    print("camera_stream: started — http://192.168.4.1")


def stop():
    global _ap, _running, _lock

    if not _running:
        return

    _running = False
    time.sleep(0.4)

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

    _lock = None
    print("camera_stream: stopped")


def state():
    return "running" if _running else "stopped"


def wave():
    """挥手: 坐下 → 右前膝摆动 3 次 → 回坐"""
    import bpuppy_motion
    bpuppy_motion.set_gait("wave")
    print("wave done")
