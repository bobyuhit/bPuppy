"""
camera_stream — WiFi 实时图传 + 网页遥控器

ESP32-S3 启动 WiFi AP，手机连上后在浏览器中查看实时画面并控制机器狗。

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


# ============================================================
# 全局状态
# ============================================================
_ap = None
_running = False
_lock = None           # _thread lock，保护 motion 调用
_active_streams = []   # 活跃的 stream client socket 列表

# 运动共享状态（命令端点写入，stream 线程可读）
_g_speed = 0
_g_turn = 0.0
_g_stride = 70
_g_height = 60
_g_gait = "stand"
_g_lift = 30

# ============================================================
# MJPEG 常量
# ============================================================
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

# ============================================================
# HTML 页面（自包含 CSS + JS）
# ============================================================

_HTML_PAGE = """\
HTTP/1.0 200 OK\r
Content-Type: text/html\r
\r
<!DOCTYPE html>
<html><head>
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<meta name="apple-mobile-web-app-capable" content="yes">
<title>bPuppy</title>
<style>
*{margin:0;padding:0;box-sizing:border-box}
body{background:#111;color:#fff;font:14px sans-serif;overflow:hidden;touch-action:none}
img{width:100%;display:block}
#ctls{position:fixed;bottom:0;left:0;right:0;background:rgba(0,0,0,0.85);padding:8px}
.row{display:flex;align-items:center;justify-content:center;gap:6px;margin:4px 0}
#spd_lbl{width:24px;text-align:center;font-weight:bold;color:#4f4}
#spd{flex:1;max-width:120px}
canvas{background:#1a1a2e;border-radius:50%;border:2px solid #333;display:block;margin:0 auto}
.btn{min-width:48px;height:44px;border:none;border-radius:8px;color:#fff;font-size:14px;cursor:pointer}
.btn-go{background:#2a2}
.btn-go:active{background:#0a0}
.btn-sit{background:#a82}
.btn-sit:active{background:#c94}
.btn-crouch{background:#84a}
.btn-crouch:active{background:#a6c}
.btn-play{background:#e80}
.btn-play:active{background:#fa2}
.btn-stop{background:#c22;width:100%;height:52px;font-size:18px;font-weight:bold}
.btn-stop:active{background:#f44}
#stat{font-size:11px;color:#888;text-align:center;margin-top:2px}
</style>
</head><body>

<img src="/stream" onerror="this.src='/stream'">

<div id="ctls">
  <div class="row">
    <button class="btn btn-go" onclick="cmd('gait=go')">GO</button>
    <button class="btn btn-sit" onclick="cmd('gait=sit')">坐</button>
    <button class="btn btn-crouch" onclick="cmd('gait=crouch')">蹲</button>
    <button class="btn btn-play" onclick="cmd('gait=play')">玩</button>
  </div>
  <div class="row">
    <span style="color:#888">速</span>
    <input type="range" id="spd" min="0" max="10" step="0.5" value="3"
      oninput="onSpd(this.value)">
    <span id="spd_lbl">3</span>
  </div>
  <canvas id="joy" width="150" height="150"></canvas>
  <div class="row" style="margin-top:6px">
    <button class="btn btn-stop" onclick="cmd('stop=1')">急停</button>
  </div>
  <div id="stat">&nbsp;</div>
</div>

<script>
var canvas = document.getElementById('joy');
var ctx = canvas.getContext('2d');
var cx = canvas.width/2, cy = canvas.height/2;
var maxR = 55;
var joyX = cx, joyY = cy;
var touching = false;
var lastSend = 0;

function drawJoy(x, y) {
    ctx.clearRect(0, 0, canvas.width, canvas.height);
    // 底圈
    ctx.beginPath();
    ctx.arc(cx, cy, maxR, 0, Math.PI*2);
    ctx.strokeStyle = '#444';
    ctx.lineWidth = 2;
    ctx.stroke();
    // 十字线
    ctx.beginPath();
    ctx.moveTo(cx - maxR, cy); ctx.lineTo(cx + maxR, cy);
    ctx.moveTo(cx, cy - maxR); ctx.lineTo(cx, cy + maxR);
    ctx.strokeStyle = '#222';
    ctx.lineWidth = 1;
    ctx.stroke();
    // 摇杆球
    ctx.beginPath();
    ctx.arc(x, y, 16, 0, Math.PI*2);
    var g = ctx.createRadialGradient(x-3, y-3, 2, x, y, 16);
    g.addColorStop(0, '#fff');
    g.addColorStop(1, '#666');
    ctx.fillStyle = g;
    ctx.fill();
}

function joyFrom(e) {
    var r = canvas.getBoundingClientRect();
    return {x: e.touches[0].clientX - r.left, y: e.touches[0].clientY - r.top};
}

function sendJoystick() {
    var dx = joyX - cx, dy = joyY - cy;
    var dist = Math.sqrt(dx*dx + dy*dy) / maxR;  // 0~1
    if (dist < 0.08) { dx = 0; dy = 0; }
    var turn = Math.max(-1, Math.min(1, dx / maxR));
    var speed = Math.max(-10, Math.min(10, -dy / maxR * 10));
    speed = Math.round(speed * 2) / 2;  // 0.5 步进
    cmd('speed=' + speed + '&turn=' + turn.toFixed(2));
}

canvas.addEventListener('touchstart', function(e) {
    e.preventDefault();
    touching = true;
    var p = joyFrom(e);
    joyX = p.x; joyY = p.y;
    drawJoy(joyX, joyY);
    sendJoystick();
});

canvas.addEventListener('touchmove', function(e) {
    e.preventDefault();
    if (!touching) return;
    var p = joyFrom(e);
    // 限制在圈内
    var dx = p.x - cx, dy = p.y - cy;
    var dist = Math.sqrt(dx*dx + dy*dy);
    if (dist > maxR) { dx = dx / dist * maxR; dy = dy / dist * maxR; }
    joyX = cx + dx; joyY = cy + dy;
    drawJoy(joyX, joyY);
    var now = Date.now();
    if (now - lastSend > 100) { lastSend = now; sendJoystick(); }
});

canvas.addEventListener('touchend', function(e) {
    e.preventDefault();
    touching = false;
    joyX = cx; joyY = cy;
    drawJoy(joyX, joyY);
    cmd('speed=0&turn=0');
    document.getElementById('spd').value = 0;
    document.getElementById('spd_lbl').textContent = '0';
});

function onSpd(v) {
    document.getElementById('spd_lbl').textContent = v;
    cmd('speed=' + v);
}

function cmd(q) {
    fetch('/cmd?' + q).catch(function(){});
    if (q.indexOf('speed=') >= 0) {
        var m = q.match(/speed=([\d.-]+)/);
        if (m) document.getElementById('stat').textContent = 'speed:' + m[1];
    }
    if (q.indexOf('gait=') >= 0) {
        var m = q.match(/gait=(\w+)/);
        if (m) document.getElementById('stat').textContent = 'gait:' + m[1];
    }
    if (q.indexOf('stop=1') >= 0) {
        document.getElementById('stat').textContent = 'STOP';
        document.getElementById('spd').value = 0;
        document.getElementById('spd_lbl').textContent = '0';
        joyX = cx; joyY = cy;
        drawJoy(joyX, joyY);
    }
}

drawJoy(joyX, joyY);
</script>
</body></html>"""


# ============================================================
# 命令处理
# ============================================================

def _parse_cmd(path):
    """解析 /cmd?speed=5&turn=-0.3&gait=stand&stride=70&height=60&lift=30"""
    global _g_speed, _g_turn, _g_stride, _g_height, _g_gait, _g_lift

    qs = path[5:] if path.startswith("/cmd?") else path
    params = {}
    for part in qs.split("&"):
        kv = part.split("=", 1)
        if len(kv) == 2:
            params[kv[0]] = kv[1]

    try:
        import bpuppy_motion
    except ImportError:
        return "motion N/A"

    with _lock:
        # 急停
        if "stop" in params:
            bpuppy_motion.emergency_stop()
            bpuppy_motion.set_gait("stand")
            _g_gait = "stand"
            _g_speed = 0
            _g_turn = 0.0
            return "OK:stop"

        # 步态
        if "gait" in params:
            g = params["gait"]
            bpuppy_motion.set_gait(g)
            _g_gait = g

        # 速度
        if "speed" in params:
            _g_speed = float(params["speed"])

        # 转向
        if "turn" in params:
            _g_turn = float(params["turn"])

        # 步幅
        if "stride" in params:
            _g_stride = float(params["stride"])

        # 高度
        if "height" in params:
            _g_height = float(params["height"])

        # 抬腿
        if "lift" in params:
            _g_lift = float(params["lift"])
            bpuppy_motion.set_lift(_g_lift)

        # 应用运动参数
        if "speed" in params or "turn" in params or "stride" in params or "height" in params:
            bpuppy_motion.set_params(abs(_g_speed), _g_stride, _g_height)
            bpuppy_motion.set_turn(_g_turn)

    return "OK"


# ============================================================
# HTTP 响应
# ============================================================

def _send_html(client):
    """返回 HTML 遥控页面"""
    try:
        client.send(_HTML_PAGE.encode())
    except OSError:
        pass
    client.close()


def _send_stream(client):
    """持续发送 MJPEG 流，直到客户端断开或 _running 变 False"""
    global _g_speed, _g_turn, _g_gait, _g_stride, _g_height, _g_lift
    client.settimeout(3.0)

    try:
        client.send(_HTTP_MJPEG_HEADER)
    except OSError:
        client.close()
        return

    # 丢掉缓冲区旧帧
    for _ in range(3):
        bpuppy_camera.capture()
        time.sleep(0.03)

    last_gait = None
    last_speed = None
    last_turn = None
    last_lift = None

    while _running:
        # 应用运动参数（有变化时）
        try:
            import bpuppy_motion
            with _lock:
                if last_gait != _g_gait:
                    bpuppy_motion.set_gait(_g_gait)
                    last_gait = _g_gait
                    last_speed = None  # 切换步态后强制重设参数
                if last_speed != _g_speed or last_turn != _g_turn or last_gait != _g_gait:
                    bpuppy_motion.set_params(abs(_g_speed), _g_stride, _g_height)
                    bpuppy_motion.set_turn(_g_turn)
                    last_speed = _g_speed
                    last_turn = _g_turn
                if last_lift != _g_lift:
                    bpuppy_motion.set_lift(_g_lift)
                    last_lift = _g_lift
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


# ============================================================
# HTTP 服务器主循环
# ============================================================

def _accept_loop():
    """后台线程：accept HTTP 连接并分发"""
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
            request = client.recv(256).decode("utf-8", "ignore")
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

        if "/stream" in first_line:
            # MJPEG 流 — 新线程处理
            if _thread:
                _thread.start_new_thread(_send_stream, (client,))
            else:
                _send_stream(client)

        elif "/cmd" in first_line:
            # 命令 — 本线程处理
            try:
                path = first_line.split(" ")[1]
                result = _parse_cmd(path)
                resp = "HTTP/1.0 200 OK\r\nAccess-Control-Allow-Origin: *\r\n\r\n" + result
                client.send(resp.encode())
            except Exception:
                pass
            client.close()

        else:
            # HTML 页面
            _send_html(client)

    s.close()


# ============================================================
# 公开 API
# ============================================================

def start(ssid="bPuppy", password="12345678"):
    """启动 WiFi 图传 + 遥控器

    参数:
        ssid:     WiFi 热点名称
        password: WiFi 密码 (至少 8 位)
    """
    global _ap, _running, _lock

    if _running:
        print("camera_stream: already running")
        return

    if _thread is None:
        print("camera_stream: _thread not available")
        return

    _lock = _thread.allocate_lock()

    # 1. 摄像头 — JPEG SVGA 800x600
    if not bpuppy_camera.is_ready():
        bpuppy_camera.init_adv(bpuppy_camera.SVGA, 10, 2, 20000000, bpuppy_camera.JPEG)

    # 2. WiFi AP
    _ap = network.WLAN(network.AP_IF)
    _ap.active(True)
    _ap.config(
        essid=ssid,
        password=password,
        authmode=network.AUTH_WPA2_PSK,
        max_clients=4,
    )
    time.sleep(0.5)

    # 3. HTTP 后台线程
    _running = True
    _thread.start_new_thread(_accept_loop, ())
    print("camera_stream: started — http://192.168.4.1")


def stop():
    """停止图传 + 遥控器 — 关闭 WiFi + 释放摄像头"""
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
    """返回当前状态: 'running' 或 'stopped'"""
    return "running" if _running else "stopped"
