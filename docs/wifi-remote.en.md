# bPuppy WiFi Remote Guide

> 🌐 [中文](wifi设备遥控指南.md)

The robot dog has a built-in WiFi hotspot and web remote: once your phone connects, a browser lets you view the live video and drive the robot. No router or external network is needed.

> **This document has two parts:** "How to Use" involves no programming; "How It Works and the API" is for developers.

---

# 1. How to Use

Factory-assembled units are **ready to use out of the box** — the hotspot turns on automatically at power-on; just connect to it.

## 1.1 Two Steps

| Step | Action |
|---|---|
| One | Connect your phone to the robot dog's WiFi → 1.2 |
| Two | Open the web page and drive → 1.3 |

## 1.2 Connect to WiFi

1. In your phone's WiFi settings, connect to **`bPuppy_XXXX`** (`XXXX` is a device identifier, different on each unit)
2. Password: **`12345678`**

> Your phone may warn that "this WiFi cannot access the internet"; that is normal (this hotspot does not connect to the internet) — just choose to keep the connection.

## 1.3 Open the Web Page

On most phones the remote page **pops up automatically** after connecting; if it does not, visit this address in a browser:

```
192.168.4.1
```

## 1.4 Interface Guide

| Control | Function |
|---|---|
| Speed slider | Adjusts the step frequency; the larger the value, the faster |
| Direction pad (8-way) | Controls the direction of travel |
| "停" (Stop) | Stops immediately and stands up |
| "站立" (Stand) / "坐下" (Sit) / "蹲下" (Crouch) | Switch poses |
| "玩" (Play) | Play-bow action (front low, rear high + hip shaking) |
| "挥手" (Wave) | Sits down and waves the right front paw 3 times |
| "图传 开" (Start stream) / "图传 关" (Stop stream) | Only toggles the video; does not affect remote control |

## 1.5 Common Situations

| Situation | What to do |
|---|---|
| Only want to turn off the video to save power | Click "图传 关" (Stop stream) on the web page |
| Clicking "图传 开" (Start stream) pops up "没检测到摄像头" | This is a normal notice: the camera is optional, and without one there is no picture; the hotspot and remote control are unaffected |
| No longer want it to start automatically | Delete `camera_on.py` from the board and power-cycle (see 2.3) |
| Want to shut it down completely right away | Delete the file and power-cycle (clicking only "图传 关" (Stop stream) does not stop the hotspot); or see 2.2 |
| `bPuppy_XXXX` cannot be found | Check in order: the device is powered on → whether airplane mode is still on / the phone is still connected to another WiFi → turn the phone's WiFi off and on, then search again. If it still cannot be found, see 2.3 (whether `camera_on.py` is still in the on-board storage) |
| Connected, but the web page will not open | Manually visit `192.168.4.1`; if that does not work, power-cycle |

---

# 2. How It Works and the API

> For developers who need to **write code or modify this feature**. If you only want to use it, the part above is enough.

## 2.1 Quick Reference (Ready to Run)

```python
import camera_stream
import bpuppy_motion
bpuppy_motion.set_gait("stop")     # stop and stand first (start does not stop automatically, so the dog does not keep moving)
camera_stream.start(stream=True)   # hotspot + video stream + web remote → http://192.168.4.1
# camera_stream.start()            # remote control only, no video streaming (default)
camera_stream.stop()               # turn off: hotspot closed, camera released
camera_stream.state()              # check state: "running" / "stopped"
```

## 2.2 API

| Call | Description |
|---|---|
| `camera_stream.start(stream=False, ssid=…, password=…)` | Starts the hotspot and the HTTP service. Only `stream=True` initializes the camera (SVGA 800×600 JPEG); by default only the remote-control page is served |
| `camera_stream.stop()` | Closes the hotspot and releases the camera (calls `deinit()` automatically) |
| `camera_stream.state()` | Returns `"running"` / `"stopped"` |

> `start(stream=True)` calls `init_adv()` internally, and `stop()` calls `deinit()` automatically —
> you **do not need to manage the camera manually** when switching between video streaming and photo capture.
> For the camera's own API (resolution / format / photo capture), see [micropython-guide.en.md](micropython-guide.en.md).

## 2.3 Auto-Start at Power-On: `camera_on.py`

Factory-assembled units **already ship with** `camera_on.py` in the on-board storage, so the hotspot turns on automatically at power-on — this is why 1.1 says "ready to use out of the box".

```python
# The entire contents of /camera_on.py
import camera_stream
camera_stream.start(stream=True)     # want only the web page, not the camera → remove (stream=True)
```

On every boot, `frozen/main.py` scans fixed file names in the board's root directory: **if the file exists it is executed; if it does not exist it is skipped**.
Therefore **the file name itself is the switch** — no recompiling the firmware and no commands to type; it takes effect the next time the board is powered on.

**Uploading / deleting:**

> 📖 See **Section 2, "Upload files"** in the [Getting Started Guide](getting-started.en.md) —
> which tool to use (the file manager page of bTool), which cable to connect, which buttons to click, and the three restrictions during transfer.
>
> - **Delete** the file → it no longer starts automatically at power-on (use this to shut the hotspot down completely)
> - **Upload it again** → auto-start is restored
> - If you flashed the firmware onto a **bare board** yourself: the modules in `frozen/` are all inside the firmware, but the on-board storage is empty, and you need to follow that section to upload `camera_on.py` and the other power-on scripts

> For the general rules (which power-on scripts there are, where they go, why they **must be pure ASCII**,
> and why they do not interfere with the `/main.py` downloaded by KittenBlock), see
> the "**2.2 Power-on Scripts**" section of [micropython-guide.en.md](micropython-guide.en.md).

## 2.4 Custom Hotspot Name / Password

```python
import camera_stream
camera_stream.start(ssid="MyDog", password="mypassword", stream=True)
```

## 2.5 Switching Between Video Streaming and Photo Capture

Both exclusively occupy the camera, so only one of them can be used at a time:

```python
# video streaming → photo capture
camera_stream.stop()
import camera_serial
camera_serial.snap()

# photo capture → video streaming
import camera_stream
camera_stream.start(stream=True)
```

## 2.6 Implementation Notes

- **The hotspot IP is fixed at `192.168.4.1`**; the automatic pop-up after the phone connects relies on Captive Portal, so there is no need to type the address manually
- **Video streaming is SVGA 800×600 JPEG**, about 10~15 fps. **Turning it on only when needed** is a deliberate trade-off — both the hotspot and the camera consume power
- After connecting there is **no internet access** (the hotspot is provided by the robot dog itself); the system warning that it "cannot access the internet" can be ignored
