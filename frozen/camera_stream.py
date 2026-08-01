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
_stream_on = False       # 图传是否开启 (可运行时切换)

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

# ---- 网页遥控器 (图传可选, 运行时开关) ----
_HTML_TEMPLATE = """\
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
#vbox{position:relative;width:100%;max-width:400px;margin:0 auto;background:#111}
#dogph{height:225px;display:flex;align-items:center;justify-content:center}
.sb{background:rgba(0,0,0,0.55);color:#fff;border:1px solid rgba(255,255,255,0.35);font-size:12px;height:28px;line-height:28px;min-width:54px;padding:0 8px;border-radius:14px}
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

<div id="vbox">
  __VIDEO__
  <div style="position:absolute;top:6px;right:6px;z-index:10">__STREAM_BTN__</div>
</div>
<iframe name="f" style="display:none"></iframe>

<div id="pad">

  <form class="sl" action="/cmd" method="get" target="f">
    <label>速</label>
    <input type="hidden" name="set" value="1">
    <input type="range" name="speed" min="0" max="10" step="0.5" value="0"
      onchange="this.form.submit()"
      oninput="document.getElementById('sv').innerHTML=this.value">
    <span id="sv">0</span>
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


_DOG_SVG = (
    '<svg viewBox="0 0 400 300" style="width:auto;height:100%" preserveAspectRatio="xMidYMid meet">'
    '<g transform="translate(0,8)">'
    '<rect x="130" y="140" width="170" height="65" rx="28" fill="#c89a5e"/>'
    '<circle cx="312" cy="150" r="34" fill="#c89a5e"/>'
    '<ellipse cx="344" cy="162" rx="18" ry="14" fill="#d8ad75"/>'
    '<circle cx="340" cy="158" r="4" fill="#333"/>'
    '<polygon points="295,118 318,80 338,120" fill="#8a5f33"/>'
    '<path d="M130 155 Q 88 140 80 175" stroke="#8a5f33" stroke-width="13" fill="none" stroke-linecap="round"/>'
    '<rect x="158" y="205" width="20" height="75" rx="10" fill="#c89a5e"/>'
    '<rect x="210" y="205" width="20" height="75" rx="10" fill="#c89a5e"/>'
    '<rect x="265" y="205" width="20" height="75" rx="10" fill="#c89a5e"/>'
    '<rect x="302" y="205" width="20" height="75" rx="10" fill="#c89a5e"/>'
    '<circle cx="322" cy="144" r="4.5" fill="#222"/>'
    '<path d="M280 170 Q 300 180 318 172" stroke="#e35d4a" stroke-width="6" fill="none"/>'
    '</g></svg>'
)


def _html_page():
    """按图传状态动态生成遥控页面"""
    if _stream_on:
        video = '<img src="/stream">'
        btn = ('<a href="/cmd?stream=off" '
               'onclick="fetch(this.href);setTimeout(function(){location.reload()},200);return false;" '
               'class="sb">图传 关</a>')
    else:
        video = '<div id="dogph">' + _DOG_SVG + '</div>'
        btn = ('<a href="/cmd?stream=on" '
               'onclick="fetch(this.href);setTimeout(function(){location.reload()},200);return false;" '
               'class="sb">图传 开</a>')
    return _HTML_TEMPLATE.replace("__VIDEO__", video).replace("__STREAM_BTN__", btn)


def _open_stream():
    """开启图传: 初始化摄像头"""
    global _stream_on
    if _stream_on:
        return
    if not bpuppy_camera.is_ready():
        bpuppy_camera.init_adv(bpuppy_camera.SVGA, 10, 2, 20000000, bpuppy_camera.JPEG)
    _stream_on = True
    print("camera_stream: stream ON")


def _close_stream():
    """关闭图传: 停流线程 + 释放摄像头"""
    global _stream_on
    if not _stream_on:
        return
    _stream_on = False
    time.sleep_ms(100)          # 让流线程退出
    bpuppy_camera.deinit()
    print("camera_stream: stream OFF")


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
        # === 图传开关 (运行时) ===
        if "stream" in params:
            if params["stream"] == "on":
                _open_stream()
            else:
                _close_stream()
            return "OK:stream"

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
    raw = _html_page().encode("utf-8")
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
        time.sleep_ms(30)

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
            time.sleep_ms(50)
            continue

        data = result[0]
        try:
            client.sendall(_PART_TEMPLATE.format(len(data)).encode())
            client.sendall(data)
        except OSError:
            break
        time.sleep_ms(70)

    client.close()


def _dns_server():
    """Captive portal DNS: 把所有域名解析到 192.168.4.1, 触发手机自动弹窗"""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.bind(("0.0.0.0", 53))
        s.settimeout(1.0)
    except OSError as e:
        print("camera_stream: DNS bind fail: %s" % e)
        return
    print("camera_stream: captive portal DNS on :53")
    while _running:
        try:
            data, addr = s.recvfrom(512)
        except OSError:
            continue                    # 超时, 回到循环检查 _running
        if len(data) < 12:
            continue
        try:
            # 定位 question 结束 (header 12 + QNAME)
            pos = 12
            while pos < len(data) and data[pos] != 0:
                pos += data[pos] + 1
            if pos >= len(data) - 4:
                continue
            qend = pos + 5               # 0 + QTYPE(2) + QCLASS(2)
            # DNS 响应: 回显 ID/flags/question + A 记录 → 192.168.4.1
            resp = (data[0:2] + b"\x81\x80" + data[4:6] + b"\x00\x01"
                    + b"\x00\x00\x00\x00" + data[12:qend]
                    + b"\xc0\x0c" + b"\x00\x01" + b"\x00\x01"
                    + b"\x00\x00\x00\x3c" + b"\x00\x04"
                    + b"\xc0\xa8\x04\x01")
            s.sendto(resp, addr)
        except OSError:
            break
        except Exception:
            pass
    s.close()


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
            raw = client.recv(512)
        except OSError:
            client.close()
            continue

        try:
            request = raw.decode("utf-8", "ignore")   # MicroPython: 非法 UTF-8 可能仍抛 UnicodeError
        except UnicodeError:
            request = ""

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
            if _stream_on and _thread:
                try:
                    _thread.start_new_thread(_send_stream, (client,))
                except OSError:
                    client.close()
            else:
                try:
                    client.send(b"HTTP/1.0 404 Not Found\r\n\r\nstream off")
                except OSError:
                    pass
                client.close()

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


def start(ssid=None, password="12345678", stream=False, captive=True):
    global _ap, _running, _lock

    if _running:
        print("camera_stream: already running")
        return

    if _thread is None:
        print("camera_stream: _thread not available")
        return

    _lock = _thread.allocate_lock()

    if ssid is None:
        # 默认热点名 = bPuppy_<MAC后四位>, 避免多台设备重名
        import ubinascii
        mac = network.WLAN(network.AP_IF).config('mac')
        ssid = "bPuppy_" + ubinascii.hexlify(mac).decode().upper()[-4:]
        print("camera_stream: AP ssid=%s" % ssid)

    if stream:
        _open_stream()          # 启动时可选开启图传

    _ap = network.WLAN(network.AP_IF)
    _ap.active(True)
    _ap.config(essid=ssid, password=password, authmode=network.AUTH_WPA2_PSK, max_clients=4)
    time.sleep_ms(500)

    _running = True
    _thread.start_new_thread(_accept_loop, ())
    if captive:
        _thread.start_new_thread(_dns_server, ())   # captive portal 自动弹窗
    print("camera_stream: started — http://192.168.4.1")


def stop():
    global _ap, _running, _lock

    if not _running:
        return

    _running = False
    time.sleep_ms(400)

    _close_stream()          # 只释放已开启的图传

    if _ap:
        try:
            _ap.active(False)
        except Exception:
            pass
        _ap = None

    _lock = None
    print("camera_stream: stopped")


def state():
    return "running" if _running else "stopped"


def wave():
    """挥手: 坐下 → 右前膝摆动 3 次 → 回坐"""
    import bpuppy_motion
    bpuppy_motion.set_gait("wave")
    print("wave done")
