# bPuppy MicroPython Guide

> 🌐 [中文](micropython编程指南.md)

For **people who write code**: type in the REPL, or write a `.py` and upload it to the board to run. **Every MicroPython interface is covered here.**

> - Using blocks? → [kittenblock-guide.en.md](kittenblock-guide.en.md)
> - WiFi video streaming / web remote → [wifi-remote.en.md](wifi-remote.en.md)
> - Just got a board (flash firmware / calibrate / acceptance) → [getting-started.en.md](getting-started.en.md)
> - Hardware wiring → [硬件连接.md](../PCB/硬件连接.md)  Principles and full source → [AGENTS.md](../AGENTS.md)

**How to upload a `.py` to the board**: see the bTool "File Management" (文件管理) page in [getting-started.en.md](getting-started.en.md).
Serial / REPL connection parameters (**115200**, CH343, over UART0) are in the "Serial Connection" (串口连接) section of [AGENTS.md](../AGENTS.md).

---

## Contents

- [0. Quick Reference](#0-quick-reference)
  - [1. Common Motion Presets](#1-common-motion-presets)
  - [3. Stand-up Self-Balancing + IMU Readings](#3-stand-up-self-balancing--imu-readings)
  - [6. Persistent Parameters (NVS Geometry / Center of Mass)](#6-persistent-parameters-nvs-geometry--center-of-mass)
- [1. Motion](#1-motion)
  - [1.1 Common Motion Presets](#11-common-motion-presets)
  - [1.2 Motion Commands — bpuppy_motion](#12-motion-commands--bpuppy_motion)
    - [1.2.1 Run Modes (automatic switching)](#121-run-modes-automatic-switching)
    - [1.2.2 Parameter Validation (out-of-range rejected)](#122-parameter-validation-out-of-range-rejected)
  - [1.3 Stand-up Self-Balancing — balance](#13-stand-up-self-balancing--balance)
- [2. Communication](#2-communication)
  - [2.1 Serial Communication — bpuppy_uart](#21-serial-communication--bpuppy_uart)
  - [2.2 Power-On Scripts](#22-power-on-scripts)
- [3. Vision](#3-vision)
  - [3.1 Camera Operation — bpuppy_camera](#31-camera-operation--bpuppy_camera)
- [4. Settings & Calibration](#4-settings--calibration)
  - [4.1 Servo Calibration — bpuppy_servo](#41-servo-calibration--bpuppy_servo)
  - [4.2 Persistent Parameters (★NVS, survive power-off)](#42-persistent-parameters-nvs-survive-power-off)
- [5. Sensors & Attitude](#5-sensors--attitude)
  - [5.1 IMU Debugging — bpuppy_imu](#51-imu-debugging--bpuppy_imu)
  - [5.2 IMU Calibration (gyro + accelerometer)](#52-imu-calibration-gyro--accelerometer)
  - [5.3 Magnetometer Calibration — calib_mag](#53-magnetometer-calibration--calib_mag)
  - [5.4 Heading Lock — heading_anchor](#54-heading-lock--heading_anchor)
  - [5.5 Battery Voltage — bpuppy_adc](#55-battery-voltage--bpuppy_adc)
  - [5.6 Extension Servos — PWM_EXT](#56-extension-servos--pwm_ext)
- [6. PC Tools](#6-pc-tools)
  - [6.1 Serial Photo Capture — capture.py](#61-serial-photo-capture--capturepy)
- [Appendix · Build / Flash / Serial](#appendix--build--flash--serial)

## 0. Quick Reference

Every snippet can be **copied and run on its own** (all imports are included). At power-on the dog stands by in the standing pose (POSE mode).


### 1. Common Motion Presets


```python
import bpuppy_motion
# Fastest
bpuppy_motion.set_params(70, 10, 60)      # stride/lift/height (all three set in one call)
bpuppy_motion.set_speed(8.5)              # step frequency
bpuppy_motion.set_gait("trot")

# Also good
bpuppy_motion.set_params(50, 10, 70)
bpuppy_motion.set_speed(8.5)
bpuppy_motion.set_gait("trot")

# Most stable
bpuppy_motion.set_params(70, 30, 70)
bpuppy_motion.set_speed(2.5)
bpuppy_motion.set_gait("walk")
```

> **`set_params(stride, lift, height)` validates and writes all three values in one call**; the `True`/`False` return value tells you whether they were written.
> The lift used to live in a separate function `set_lift`, and the two functions each validated against the other's **old** value, which forced a call order (first `set_lift`, then `set_params`) — now they are merged into a single call and that trap is gone. `set_lift` has been removed.
>
> Speed is the **step frequency** and has nothing to do with the leg trajectory, so it has its own `set_speed(0~10)` you can change at any time without touching stride/lift/height. It also returns `True`/`False`.
>
> ⚠ **After upgrading the firmware you must download your program again**: `set_lift(...)` in an old program raises `AttributeError`, and the old-style `set_params(speed, stride, height)` call gets its arguments misaligned.


### 3. Stand-up Self-Balancing + IMU Readings

```python
import bpuppy_imu
bpuppy_imu.init(0, 14, 21, 0x68)      # 1. Initialize the IMU first (AHRS needs to converge; may need retrying right after power-on)

import time
time.sleep_ms(2000)                        # 2. Wait for the attitude to stabilize (AHRS convergence)

import balance
balance.start()                      # 3. Pick one of the three (auto emergency stop)
# balance.start(kp=0.06, ki=0.001, kd=0.5)       # Custom PID
balance.stop()                       # Stop; restores the motion task
```

**Helper · Loop-print inertial data:**

```python
import bpuppy_imu
bpuppy_imu.init(0, 14, 21, 0x68)      # Initialize (idempotent)
import time
for i in range(20):
    r, p, y = bpuppy_imu.read_angles()
    print('roll=%.2f pitch=%.2f yaw=%.2f' % (r, p, y))
    time.sleep_ms(500)
```

**Helper · Magnetometer data output:**

```python
import bpuppy_imu
bpuppy_imu.init(0, 14, 21, 0x68)      # Initialize (idempotent)
import time
while True:
    a, g, m, t = bpuppy_imu.read_raw()
    print('mag=%.3f %.3f %.3f uT' % (m[0], m[1], m[2]))
    time.sleep_ms(200)
```

### 6. Persistent Parameters (NVS Geometry / Center of Mass)


```python
import bpuppy_motion
bpuppy_motion.cal_ik(40, 45)                     # Leg lengths (L1, L2 mm)
bpuppy_motion.set_body_dims(62.5, 59)            # Hip spacing (front/rear half-spacing, left/right half-width mm)
bpuppy_motion.set_joint_limits(0, 180, 10, 170)  # Joint limits (hip min/max, knee min/max deg)
bpuppy_motion.set_center(0)                      # CoM offset (positive = feet forward); must also stay reachable combined with stride/height — only ±11.4mm at default parameters
bpuppy_motion.show_geometry()                    # Show everything
```


## 1. Motion

### 1.1 Common Motion Presets

Three configurations measured as optimal (full code in Quick Reference 1 above — copy and run):

| Preset | speed | stride | height | lift | gait |
|------|-------|--------|--------|------|------|
| Fastest | 8.5 | 70 | 60 | 20 | trot |
| Also good | 8.5 | 50 | 70 | 30 | trot |
| Most stable | 2.5 | 70 | 70 | 30 | walk |

> Before powering off, call `poses.crouch()` first to relax the servos.

### 1.2 Motion Commands — bpuppy_motion

**Functions (ready to call):**

| Call example | Description |
|------|------|
| `bpuppy_motion.start()` | Start the 50Hz motion task (loads geometry parameters automatically) |
| `bpuppy_motion.set_gait("walk")` | Switch gait (see the gait list below) |
| `bpuppy_motion.set_gait("stop")` | Stop and stand (GAIT_STOP; stays in motion mode) |
| `bpuppy_motion.get_mode()` | Query the run mode (0=IDLE, 1=POSE, 2=MOTION) |
| `bpuppy_motion.show_geometry()` | Show all persistent + runtime parameters |
| `bpuppy_motion.load_geometry()` | Reload persistent parameters from NVS |

**Runtime parameters (lost on power-off; set them each time):**

| Call example | Default | Description |
|------|--------|------|
| `bpuppy_motion.set_params(70, 30, 70)` | 70, 30, 70 | (stride, lift, height): **stride / lift height / standing height**. stride is **magnitude only** (≥0; 0 = marching in place); **direction is set separately** — negative values are rejected. All three are validated and written together; returns `True`/`False` |
| `bpuppy_motion.set_direction(1)` | 1 | **Direction**: `+1` = forward / `-1` = backward. **Fully decoupled** from stride/speed/gait — changing direction doesn't touch the stride magnitude, and changing stride won't turn the dog around. `0` or `NaN` is rejected. Returns `True`/`False` |
| `bpuppy_motion.set_speed(2.5)` | 2.5 | **Step frequency** 0~10, unrelated to the leg trajectory, can be changed on its own. Returns `True`/`False` |
| `bpuppy_motion.set_omega(2.0)` | 2.0 | Base angular frequency (rad/s); adjusts how fast the gait runs |
| `bpuppy_motion.set_body_pose(0, 0)` | 0, 0 | **(pitch, roll)** body tilt angles (deg). **Purely manual — no IMU feedback**; effective in go/walk/trot and held once set. **If unreachable, the whole call is rejected** (neither pitch nor roll is written; old values kept). Returns `True`/`False`. With default parameters \|pitch\| ≤ 3.7°, \|roll\| ≤ 6.2° — to lean further, first reduce the stride |
| `bpuppy_motion.set_turn(0)` | 0 | Turn rate -1~+1. Values beyond ±1 **are clamped to ±1** (still written); in that case it returns `False` |

> **Changing standing height / lift / pitch / roll / CoM while moving is smoothly interpolated** — it advances a small step each frame and typically takes 0.3s to arrive; it never jumps in a single frame. When stationary the change takes effect immediately (the "max 3° per frame" limiter covers that case).
>
> `set_lift()` has been removed (merged into `set_params` as the 2nd argument); the 3rd argument `yaw` of `set_body_pose` was a dead field and has been removed.
> Read values back with `get_params()`, whose order is `(speed, stride, height, lift, omega, turn, gait, direction)`
> — note this is **a different argument order** from `set_params(stride, lift, height)`.
> `[1]` is the **signed** stride (= magnitude × direction; a negative sign means backward), and `[7]` is the direction itself.
>
> ⚠ **The `go` gait takes over stride/lift/height.** Under go, stride, standing height and lift are computed from speed itself
> (speed≤4 → stride 70 / height 70 / lift 30; speed≥6 → stride 50 / height 70 / lift **5**; linear interpolation in between).
> The three values you set **are accepted and saved, but do not participate in go's motion** — switching to walk / trot afterwards immediately uses them.
> Consequently: the four direction blocks (forward/backward/turn left/turn right) all use go ⇒ the "Motion Params" block has **no effect** on them;
> to make "Motion Params" work, first pick walk or trot with the "Switch Gait" block.
> **Direction is not affected by go** — it is an independent parameter, treated the same in all three gaits.

**Measured optimal parameters:**

| Gait | speed | stride | height | lift | pose | Notes |
|------|-------|--------|--------|------|------|------|
| walk | 2.5 | 70 | 70 | 30 | pitch=-5 | walk; stable and quiet |
| trot | 8~8.5 | 50~70 | 70 | 30 | pitch=-3 | trot; fastest |

> When lowering the standing height (height<70) you must reduce the lift: height=60 → lift≤20, height=50 → lift≤10, otherwise the hip angle exceeds its limit and the parameter validation rejects it.

**Whichever gait you use, it's always the same four things before moving:**

```python
bpuppy_motion.set_gait("go")             # 1. Choose the gait
bpuppy_motion.set_direction(1)           # 2. Set the direction (+1 forward / -1 backward; defaults to +1 at power-on)
bpuppy_motion.set_speed(2.5)             # 3. Set the step frequency 0~10
bpuppy_motion.set_params(70, 30, 70)     # 4. The three advanced parameters: stride magnitude / lift height / standing height
```

The four are **mutually independent**: changing direction doesn't touch the stride magnitude, changing stride won't turn the dog around, and changing speed doesn't affect the gait.
`go` is the only exception — its stride/height/lift are derived from `speed`, and values set with `set_params`
**are accepted and saved but do not participate in go's motion** (they take effect as soon as you switch to walk / trot).
But **direction stays independent**: go obeys `set_direction` as well; forward/backward no longer depends on the stride's sign.

**Gait list:**

| Call example | Description |
|-----------|------|
| `bpuppy_motion.set_gait("stop")` | Stop and stand (GAIT_STOP; height follows parameters) |
| `bpuppy_motion.set_gait("walk")` | walk (direction set by `set_direction`; speed only controls step frequency) |
| `bpuppy_motion.set_gait("trot")` | trot (direction set by `set_direction`; speed only controls step frequency) |
| `bpuppy_motion.set_gait("go")` | adaptive (recommended): speed≤4→walk, speed≥6→trot, all parameters interpolated between 4 and 6. ★ Under go, **all three of stride/lift/height have no effect**; only speed matters (see the warning below) |
| `poses.crouch()` | crouch (pose mode, fixed angles) |
| `poses.sit()` | sit (pose mode) |
| `poses.play()` | play (front low, rear high + 8 hip wiggles at 4 Hz) |
| `poses.wave()` | wave (sit + right front knee swings 3 times, then back to sit) |
| `poses.stand()` | stand (pose mode POSE_STAND, fixed height) |

> **An unknown gait name → stop the robot; this is a deliberate fail-safe, not a defect.**
> Inside `set_gait()`, `gait_type_t g = GAIT_STOP;` is the initial value; when the name matches no entry no assignment happens, so it falls straight through to STOP. So a single typo (e.g. `"tr"`) = the dog stops in place.
> The reason is that **commands can genuinely get corrupted** — for example, Bluetooth transfers of Chinese text / long strings drop whole runs of bytes (code containing Chinese loses bytes over Bluetooth, so `"trot"` may arrive as `"tr"`). Here, stopping is the only safe outcome, better than guessing a gait from a corrupted string.
> Since v3.0-13 an unknown name prints `⚠ 未知步态 "xxx" → 停车` on the REPL; the behavior is unchanged, it's just no longer silent.
>
> *The device prints these messages in Chinese.*
>
> ⚠ The old aliases `"walkfwd"` / `"walkbck"` / `"trotfwd"` / `"trotbck"` **have been removed**; they now hit the fallback above (stop + warning).
> They never should have existed — four names mapped to the same two gaits, and direction relied entirely on **the stride's sign at the time**, so
> `set_gait("walkbck")` would still walk forward with a forward stride. Direction is now set explicitly with `set_direction(±1)`.

### 1.2.1 Run Modes (automatic switching)

The system has **three run modes**, switched automatically by the C-layer state machine; **no manual operation needed**:

| Mode | Value | Description |
|------|-----|------|
| IDLE | 0 | Not yet initialized after power-on (brief) |
| POSE | 1 | Pose mode: Python drives the servos (crouch/sit/stand/custom) |
| MOTION | 2 | Motion mode: C layer controls at 50Hz (go/walk/trot/stop) |

**Automatic switching rules:**
- **Entering MOTION**: `set_gait('go'/'walk'/'trot'/'stop')` → switches to motion mode automatically
- **Entering POSE**: Python writes to the servos (`set_angle`/`group_commit`/`cal`) → switches to pose mode automatically
- Query the current mode: `bpuppy_motion.get_mode()`

**Stop (GAIT_STOP) vs stand (POSE_STAND):**

| | `set_gait('stop')` | `poses.stand()` |
|---|---|---|
| Mode | MOTION | POSE |
| Controller | C-layer IK | Python IK |
| Height | Follows the `set_params` height | Fixed at 70mm |
| Purpose | Stop and stand | Standing in the pose domain |

### 1.2.2 Parameter Validation (out-of-range rejected)

`set_params` validates automatically when setting parameters: **if any value is out of range or the combination interferes, the whole group is rejected and existing values are kept** (a notice is printed on the REPL, return value `False`).
**Speed takes no part in the geometric check** (only a 0~10 range check); it is set by the separate `set_speed`.

| Parameter | Condition | Rejection reason |
|------|---------|---------|
| **Speed** | 0 ~ 10 | Out of range |
| **Stride** | \|stride\| ≤ 2×body_half_l×0.85 ≈ **106mm** | Front/rear foot interference |
| **Height** | Nominally `height ≥ 15mm`, but in practice **the hip angle binds first** (≈ **20.6mm** with default parameters) | Too low: leg-fold interference / hip angle exceeded |
| **Lift** | height - lift ≥ 0 | Excessive lift (foot end above the hip) |
| **Hip/knee angle** | Walks the actual swing-phase foot trajectory; at every point hip∈[0,180]°, knee∈[10,170]° | Out of limits (including the lift peak) |

**Foot trajectory walk-through** (the core of the hip/knee check), sampled at `ease = 0, 0.25, 0.5, 0.75, 1.0`:

```
x = -|stride|/2 + |stride|·ease + center_offset
z = height - lift·sin(ease·π)
```

IK is solved for all 4 legs (left/right × front/rear) and the hip/knee angles are checked against the limits; **a single point out of range rejects the whole set**.

> The check involves only the **three parameters stride / height / lift**; speed gets its own 0~10 range check and takes no part in the geometry.
> Lowering the height requires reducing the lift (e.g. height=60→lift≤20, height=50→lift≤10), otherwise the hip angle exceeds its limit and gets rejected. The validation log shows which leg and which angle.
> Defaults: speed=2.5, stride=70, height=70, lift=30.

### 1.3 Stand-up Self-Balancing — balance

Bypasses the motion task and drives the servos directly. Incremental PID, 50Hz closed loop. `start()` automatically switches magnetometer fusion to a "yaw-only" mode (`set_mag_fusion(False)`) so roll/pitch stay accurate when level; `stop()` restores it.

> ⚠ `start()` initializes the IMU automatically, but **if the zero point is recorded right after init while the AHRS has not yet converged, self-balancing can diverge**. It's recommended to **initialize the IMU manually first and wait for the attitude to stabilize** before starting:

```python
import bpuppy_imu
bpuppy_imu.init(0, 14, 21, 0x68)      # Initialize the IMU first (may need retries right after power-on)

import time
time.sleep_ms(2000)                        # Wait for the AHRS to converge

import balance
balance.start()                                  # ← pick one of the three (default parameters)
# balance.start(kp=0.06, kd=0.2)                 # Custom PD
# balance.start(kp=0.06, ki=0.001, kd=0.5)       # Custom PID
```

| Parameter | Default | Meaning |
|------|------|------|
| kp | 0.06 | Proportional gain |
| ki | 0.0 | Integral gain |
| kd | 0.43 | Derivative gain |
| deadband | 0.5 | Deadband (°) |
| max_body | 30.0 | Max body tilt (°) |
| height | 60.0 | Target center height (mm) |

**Stop:**

```python
import balance
balance.stop()
```

> `balance.start()` switches into pose mode (POSE) automatically by writing to the servos. **After interrupting with Ctrl-C** you must call `balance.stop()` manually to recover (it calls `set_gait("stop")` internally to restore motion).

**Serial output:**

```
IMU P+0.1 R-2.4 | Err Pe+0.0 Re+0.0 | Inc iP+0.00 iR+0.00 | Tar Pt+0.0 Rt+0.0
```

| Field | Meaning |
|------|------|
| IMU P/R | IMU attitude angles (Pitch/Roll) |
| Err Pe/Re | Error (initial − current) |
| Inc iP/iR | Per-frame compensation increment |
| Tar Pt/Rt | Target body tilt |

---


## 2. Communication

### 2.1 Serial Communication — bpuppy_uart

**UART2 (default communication port)** — GPIO 20=RX, 19=TX; connects to the CI-33T voice module / micro:bit. ⚠ Since 2026-08-19 the **pins are swapped** (TX=GPIO19/RX=GPIO20); for the reason see the UART2 section of [硬件连接.md](../PCB/硬件连接.md).

```python
import bpuppy_uart
bpuppy_uart.init(2, 19, 20, 115200)   # UART2, TX=19, RX=20
bpuppy_uart.send(b"hello")            # Send bytes
bpuppy_uart.sendline("AT")            # Send a string + newline
data = bpuppy_uart.read(64)           # Read bytes (non-blocking)
n = bpuppy_uart.any()                 # Bytes available in the buffer
```

**UART1 (camera-shared port)** — GPIO 4=TX, 5=RX; **shares pins with the camera SCCB SDA/SCL — usable when not taking photos**.

```python
import bpuppy_uart
bpuppy_uart.u1_init(4, 5, 115200)      # TX=4, RX=5
bpuppy_uart.u1_send(b"hello")
bpuppy_uart.u1_sendline("AT")
data = bpuppy_uart.u1_read(64)
n = bpuppy_uart.u1_any()
```

| Call example | Description |
|------|------|
| `bpuppy_uart.init(2, 19, 20, 115200)` | Initialize UART2 (num, tx, rx, baud) |
| `bpuppy_uart.send(b"hello")` | Send bytes or str |
| `bpuppy_uart.read(64)` | Read up to 64 bytes (non-blocking) |
| `bpuppy_uart.any()` | Number of readable bytes in the buffer |
| `bpuppy_uart.sendline("AT")` | Send a string + CRLF |
| `bpuppy_uart.stop()` | Stop UART2 (releases the peripheral; can init again) |
| `bpuppy_uart.u1_init(17, 18, 115200)` | Initialize UART1 (camera-shared pins) |
| `bpuppy_uart.u1_send(b"hello")` / `u1_read(n)` / `u1_any()` / `u1_sendline(s)` | The UART1 equivalents |
| `bpuppy_uart.u1_stop()` | Stop UART1 (releases the peripheral; can init again) |

> Both are **initialized manually** and do not start automatically at power-on (to avoid fighting the camera for the pins).

**Voice control (CI-33T module, on by default at power-on)** — implemented in `frozen/voice.py`; `import voice` automatically starts UART2 (**9600 baud**) and polls it in the background. CI-33T wiring: **PA2(TX)→GPIO20(UART2 RX), PA3(RX)←GPIO19(UART2 TX)**, 9600 baud (see [硬件连接.md](../PCB/硬件连接.md) for details).

> ⚠ **GPIO19/20 = USB_D-/USB_D+; TinyUSB must be disabled to use them as UART2** (root cause measured 2026-08-19):
> On the ESP32-S3, GPIO19=USB_D- and GPIO20=USB_D+ are **native USB pins**. The MicroPython component enables **TinyUSB CDC** by default (`usb_init()` in `mpy_startup.c` → `tinyusb_driver_install()`), which initializes the USB-OTG PHY at startup and **takes over GPIO19/20**. Symptoms:
> - `machine.Pin(19)` has no effect, and UART2 TX produces no waveform on GPIO19 (invisible on a scope)
> - Measured pin levels are the USB idle state: **D-(19) low, D+(20) high**
> - Downstream (GPIO20 receiving) occasionally works (the input path survives); upstream (GPIO19 transmitting) fails completely
>
> **Fix (firmware customization)**: comment out the `usb_init()` call in `components/mr9you__micropython-helper/mpy_startup.c` to skip TinyUSB initialization; GPIO19/20 return to plain GPIO for UART2. **USB-CDC virtual serial is therefore unavailable — this is not a feature that can be turned on**: the two pins belong to the voice module, and restoring USB serial means giving up UART2 (REPL/flashing go over UART0=COM14 and BLE goes over RF, both unaffected).

Protocol: **upstream** = `AA 55 <CMD> <PARAM> 55 AA`; **downstream** = **fixed-length 4-byte frames, two kinds**
(changed 2026-09-26; previously downstream was a bare 2-byte `<CMD> <PARAM>`).

**Downstream (CI-33T → this system)** — two frame kinds, distinguished by the **first byte's value range**:

| Frame | Format | Description |
|---|---|---|
| **Command frame** | `BB <CMD> <PARAM> EE` | Motion/pose commands; CMD in the table below; PARAM reserved (always `00` for now) |
| **Angle frame** | `<angle> 00 00 00` | **Sound-source angle (DOA)**, 0–180 degrees. ⚠ Fixed on the module side; this repository cannot change it |

The only criterion is the first byte: `0xBB` (=187) → command frame; `≤ 0xB4` (=180) → angle frame.
**The two ranges do not overlap** (the angle can never be 187), so they cannot be confused.

The CMD byte of a command frame (`0x30`–`0x3F`; `0x3D`/`0x3E` unused for now):

| CMD | Action | CMD | Action |
|--------|------|--------|------|
| `0x30` | Stop | `0x38` | Stand |
| `0x31` | Forward | `0x39` | Crouch |
| `0x32` | Backward | `0x3A` | Sit |
| `0x33` | Turn left | `0x3B` | Wave |
| `0x34` | Turn right | `0x3C` | Play |
| `0x35` | Speed up | `0x3D` | (unused) |
| `0x36` | Slow down | `0x3E` | (unused) |
| `0x37` | Nod | `0x3F` | Announce voltage ★firmware built-in action |

`voice.py` prints `VOICE RX: <hex>` as soon as bytes arrive, then splits by frame structure (`voice._parse`):
command frames dispatch events, angle frames are stored into `voice.SoundAngle`. **If fewer than 4 bytes have arrived it keeps them for the next round**,
so frames split apart or stuck together by the serial stream don't affect parsing.

> ⚠ **This change fixed a real bug (before 2026-09-26)**: the old version **scanned byte by byte** for the command range,
> and the first byte of an angle frame `<angle> 00 00 00` is the angle itself — **at 48–63 degrees the low byte falls right
> inside the scanned range and gets executed as a motion command** (53° → `0x35` = speed up; 48–63 degrees cover the whole `0x30`–`0x3F` range).
> Now the decision is structural, so the problem is gone at the root. The criterion is the non-overlapping first-byte ranges — no reliance on
> timing, length, or delays.

**Upstream (this system → CI-33T)** — sound/feedback; frame = `AA 55 <CMD> <PARAM> 55 AA`:

| Frame content | Meaning |
|--------|------|
| `AA 55 70 00 55 AA` | Bark sound #0 |
| `AA 55 70 01 55 AA` | Bark sound #1 = **"woof woof"** (confirmed by testing) |
| `AA 55 70 02 55 AA` | Bark sound #2 = **"whimper"** (confirmed by testing; changed on 2026-09-27 from `AA 55 71 02`) |
| `AA 55 71 00 55 AA` | Platform custom sound slot 0 |
| `AA 55 72 <number> 55 AA` | **Announce a number** (0–100; sent by the "Report Number [NUM]" block) |

⭐ **For bark-type sounds the first byte is uniformly `0x70`; the 4th number selects the actual sound** — to add a new sound, change the 4th number,
matching the `SND_xxx` constants in the firmware's `voice.py`.

```python
import voice
voice.play('汪汪')      # Play a preset sound (woof/whimper; mapping in voice.SND_WANG/SND_YING)
voice.say(0x70, 1)      # Send bark sound #1 = woof woof (AA 55 70 01 55 AA)
voice.say(0x70, 2)      # Send bark sound #2 = whimper (AA 55 70 02 55 AA)
voice.SoundAngle        # Last sound-source angle (deg, 0-180); -1 = none received since boot
voice.on_cmd(0x30, fn)  # Register a callback: fn runs when the stop command is received
voice.stop()            # Stop the background polling
```

> ⚠ **Events only — no built-in actions (since 2026-08-19)**: on receiving a voice command the firmware **only prints + fires event callbacks** (KittenBlock event blocks), with **no action built in** (forward/backward etc. were all removed); all actions are programmed by the user in KittenBlock.
>
> ⭐ **The only exception: `0x3F` announce voltage (since 2026-09-27)** — on receiving it the firmware **automatically** sends
> an "announce number" frame to the CI-33T with the battery percentage (`AA 55 72 <pct> 55 AA`, with pct from `voltage.read_pct()`).
> Reason for the exception: it is "reading out the firmware's own battery measurement" — an announcement of the firmware's own state, not motion/pose;
> and if left to user code, the sole source of truth for voltage is in the C layer, unreachable from Python. **No user code required.**
>
> - **Platform prerequisite**: the CI-33T "Serial Input" (串口输入) must have **entries for 0–100** matching the `AA 55 72 <data> 55 AA` frame,
>   otherwise the frame arrives but there is nothing to play (see §3.2.2 of the voice event system "语音事件系统" in [AGENTS.md](../AGENTS.md)).
> - **Unconditional**: the `voiceWhenVolt` event **still** registers as usual. So if the user program also contains a
>   "when receive [report voltage]" block, **the firmware announcement and the user action both fire** (it sounds like it reports twice).
> - If no battery reading is available it announces **0** and prints a one-line log explaining why.

**CI-33T side configuration (the 智能公元 platform)**: under "Offline Command Words & Responses Customization" (离线命令词与应答语自定义) configure a **serial send** for each command word ("forward", "stop", …) that emits the corresponding **command frame** `BB <CMD> 00 EE` (e.g. forward = `BB 31 00 EE`; see the CMD table above); **leave the sound-source angle output as `<angle> 00 00 00` — do not change it** (change it and parsing breaks). To make this system emit sound, configure a "Serial Input" (串口输入) entry matching the `AA 55 <data> 55 AA` frame to trigger the matching sound/announcement. Set both sides to 9600 baud.

> ⚠ **The module side and the firmware must both use the new format**: with command frames changed from 2 to 4 bytes, **new firmware does not recognize the old 2-byte commands**
> (2 bytes can't form a 4-byte frame, so they sit there as a fragment) and all 14 command words go dead.
> So before flashing the new firmware, update the platform side's 14 command outputs to `BB <CMD> 00 EE`;
> or accept "flash first, voice commands unresponsive in the meantime" — they recover once the platform side is updated.
> The reverse (platform updated first, firmware still old) **does not affect use** — old firmware byte-scans and still dispatches the CMD byte inside the frame.


### 2.2 Power-On Scripts

Every boot, `frozen/main.py` looks for these power-on scripts on the board: **if the file exists it runs; if you delete it, nothing happens**.
So the file itself is the switch — no need to rebuild the firmware or type commands; it takes effect at the next power-on.

| Power-on script | Effect | Source file |
|---------|------|--------|
| `/camera_on.py` | Automatically starts "web UI + camera" at power-on | `mpy_modules/camera_on.py` |
| `/pwm_ext_on.py` | Automatically enables extension servos (PWM_EXT) at power-on | `mpy_modules/pwm_ext_on.py` |

```python
# Full contents of /camera_on.py
import camera_stream
camera_stream.start(stream=True)     # Web UI only, no camera → remove (stream=True)
```

```python
# Full contents of /pwm_ext_on.py (PWM_EXT2 enabled in the current version)
EXT1 = 0        # GPIO 3  shared with the battery ADC → battery readings stop working
EXT2 = 1        # GPIO 47 free pin → no side effects
EXT3 = 0        # GPIO 48 shared with the WS2812 → battery LED goes dark
ANGLE = 90      # Initial angle for the enabled channel(s)

import pwm_ext
for _n, _want in {1: EXT1, 2: EXT2, 3: EXT3}.items():
    if _want:
        pwm_ext.on(_n)
        pwm_ext.set_angle(_n, ANGLE)
```

> **To switch which channel is enabled, just edit the 0/1 values above.** Which GPIO, which MCPWM channel, and whether to stop the battery
> ADC first — all of that lives in the firmware (`pwm_ext.on()` in `frozen/pwm_ext.py`); you don't need to touch it and you can't reach it.
> ⚠ Keep the comments **pure English**: uploads over Bluetooth drop non-ASCII bytes.

- **On**: copy `mpy_modules/<the matching file>` to the board (drag it in with ViperIDE, or `mpremote cp ...`) — **the file name must match the table above**, placed in the root directory
- **Off**: just delete that file from the board
- **No interaction with `/main.py`** — each KittenBlock download only rewrites `/main.py` (the user program it downloads) and never touches these files
- ⚠ These files must be **pure ASCII** (don't write Chinese comments): Bluetooth transfers drop bytes from content containing Chinese. The copies in the repository are all pure ASCII
- ⚠ If you enable **PWM_EXT1 (GPIO3)** or **PWM_EXT3 (GPIO48)**, they share pins with battery sensing, so **you must call `bpuppy_adc.stop()` first**;
  otherwise the firmware still drives/reads the same pin. The cost is that battery voltage readings and the WS2812 indicator stop until `bpuppy_adc.init()` restores them.
  **PWM_EXT2 (GPIO47) is a free pin and needs no such step** (the sample file uses it)

## 3. Vision

### 3.1 Camera Operation — bpuppy_camera

**Take a photo:**

```python
import bpuppy_camera

# Initialize (manual init required the first time after power-on)
bpuppy_camera.init()

# Capture → returns (data, width, height, format)
data, w, h, fmt = bpuppy_camera.capture()
# data: JPEG bytes (1600×1200 by default)
# fmt:  4=JPEG, 1=RGB565, 5=GRAYSCALE

# Release
bpuppy_camera.deinit()
```

**Format switching:**

The default `init()` outputs JPEG 1600×1200. For other formats or resolutions:

```python
import bpuppy_camera
# QVGA 320×240 RGB565 (for color-ball detection)
bpuppy_camera.init_adv(bpuppy_camera.QVGA, 0, 2, 20000000, bpuppy_camera.RGB565)

# SVGA 800×600 JPEG (for WiFi video streaming)
bpuppy_camera.init_adv(bpuppy_camera.SVGA, 10, 2, 20000000, bpuppy_camera.JPEG)
```

| Resolution | Constant | Pixels |
|--------|------|------|
| 160×120 | `QQVGA` | — |
| 320×240 | `QVGA` | — |
| 640×480 | `VGA` | — |
| 800×600 | `SVGA` | — |
| 1024×768 | `XGA` | — |
| 1600×1200 | `UXGA` | default |

| Format | Constant | Use |
|------|------|------|
| JPEG | `JPEG` (4) | Photos/video streaming; hardware-encoded |
| RGB565 | `RGB565` (1) | Image recognition; raw pixels |
| GRAYSCALE | `GRAYSCALE` (5) | Grayscale images |

**Known issues:**

- The first few frames after initialization may show the old scene (double-buffer residue); `camera_serial.snap()` already discards the first 3 frames
- The image was upside down: already fixed in the driver with `set_vflip(1)`

---


## 4. Settings & Calibration

### 4.1 Servo Calibration — bpuppy_servo

**Servo naming constants:**

| Constant | Value | Meaning |
|------|-----|------|
| `bpuppy_servo.LF_HIP` | 0 | Left front hip |
| `bpuppy_servo.LF_KNEE` | 1 | Left front knee |
| `bpuppy_servo.LH_HIP` | 2 | Left rear hip |
| `bpuppy_servo.LH_KNEE` | 3 | Left rear knee |
| `bpuppy_servo.RF_HIP` | 4 | Right front hip |
| `bpuppy_servo.RF_KNEE` | 5 | Right front knee |
| `bpuppy_servo.RH_HIP` | 6 | Right rear hip |
| `bpuppy_servo.RH_KNEE` | 7 | Right rear knee |

**Functions (ready to call):**

| Call example | Description |
|------|------|
| `bpuppy_servo.init_all()` | Initialize all 8 servos per the hardware mapping |
| `bpuppy_servo.set_angle(0, 90)` | (channel, angle) set a servo angle (current mapping -35~215) |
| `bpuppy_servo.get_angle(0)` | (channel) return the current angle (float) |
| `bpuppy_servo.stop()` | Emergency-stop all servos (PWM off) |
| `bpuppy_servo.cal_point(0, 0, 3)` ★ | (channel, point, reference angle) **three-point calibration**: point 0/1/2 = 0°/90°/180° |
| `bpuppy_servo.get_cal_point(0, 1)` | (channel, point) read the calibration reference angle of a point |
| `bpuppy_servo.cal(0, 90)` | (channel, reference angle) legacy-compatible = calibrates only the 90° point |
| `bpuppy_servo.get_cal(0)` | (channel) read the 90° point's calibration reference angle |
| `bpuppy_servo.load_cal()` ★ | Load calibration from NVS (called automatically at boot) |

**Three-point calibration (★recommended):**

Each channel stores, at the three points **0°/90°/180°**, "the angle the servo should actually be sent when this angle is commanded", with piecewise-linear interpolation in between — compensating **both zero offset and slope/non-linearity** (single-point `cal` compensates only the zero offset).

**Calibration procedure** (three points per servo; `cal_point` does it in one step: writes NVS + turns the servo to the raw angle on the spot):

```python
bpuppy_servo.cal_point(0, 0,   3)   # 1. try 3   → check whether the leg is at the ideal 0° position → if not, try another number and resend
bpuppy_servo.cal_point(0, 1,  95)   # 2. try 95  → leg vertical (90° position)
bpuppy_servo.cal_point(0, 2, 178)   # 3. try 178 → leg at the ideal 180° position
```

Every call writes NVS once, so **the last value you settle on is the final one** — there is no separate "save" step. `cal_point` sends the **raw angle** you give it (bypassing the calibration table); `set_angle` is the opposite — it sends the value from three-point interpolation — so **don't use `set_angle` while experimenting** (the table changes as you try things, so its output won't match your input). Once calibrated, verify each point with `set_angle(0/90/180)`. Values persist across power-off; intermediate angles (45°/135° etc.) are filled in by interpolation. `cal(ch, ref)` is kept for legacy compatibility (equivalent to `cal_point(ch, 1, ref)`, calibrating only the 90° point).

> For direction conventions (where the hip points at 0/90/180, left/right knee mirroring) see the "Servo Angle Calibration" ("舵机角度标定") table in [AGENTS.md](../AGENTS.md).

### 4.2 Persistent Parameters (★NVS, survive power-off)

| Group | Call example | Default |
|----|------|--------|
| Geometry | `bpuppy_motion.cal_ik(40, 45)` ★ | Upper leg 40, lower leg 45 (mm) |
| Geometry | `bpuppy_motion.set_body_dims(62.5, 59)` ★ | Front/rear half-spacing 62.5, left/right half-width 59 (mm) |
| Geometry | `bpuppy_motion.set_joint_limits(0, 180, 10, 170)` ★ | Hip 0~180, knee 10~170 (deg) |
| Servo | `bpuppy_servo.cal(ch, 90)` ★ | Calibration reference angles for all 8 channels (deg) |
| CoM | `bpuppy_motion.set_center(0)` ★ | Foot center offset (mm); positive = forward. **Two checks**: ① ≤ upper-leg length / 2 (default ±20); ② reachable in combination with the current stride/height — at default parameters ② is tighter and it actually only goes to **±11.4mm** |

> The parameters above are written to NVS automatically when set (9 geometry/CoM values + 3 points × 8 servos = 24 calibration values); `motion.start()` restores them automatically. With no NVS data it falls back to the `ik.h` macro defaults.
>
> `show_geometry()` prints the geometry parameters + each servo's **90° point** calibration value; **it does not show the 0°/180° points** — read those separately with `bpuppy_servo.get_cal_point(ch, 0/2)`.
>
> `get_geometry()` returns `(L1, L2, front/rear half-spacing, left/right half-width)` so programs can read the current geometry (`show_geometry()` only prints; you can't capture it in a variable).
> `poses.stand()` (including the power-on stand pose and the "Stand" block) and `balance` both read it live, so **after `cal_ik()` changes the leg lengths, the stand pose and self-balancing follow immediately**.
> ⚠ Don't compute the stand pose with `bpuppy_ik.L1/L2` — those are **compile-time defaults**; `cal_ik()` can't change them, and mixing them makes the stand pose and the gait solve with two different leg lengths.
>
> **Illegal values are rejected** (values stay unchanged, returns `False`, the REPL prints `⚠ ... 被拒`): `cal_ik` / `set_body_dims` require 1~500 mm; `set_joint_limits` requires `min < max` within the servo travel (currently 270° servos = −35~215); for `set_center` / `set_body_pose` see below.
> Because these values live in NVS and **survive firmware upgrades**, both the write path and the boot-time read path validate them — a board with corrupted values shows `⚠ NVS 里 ... 非法 -> 回退默认` in the boot log after flashing new firmware and recovers automatically, rather than staying broken forever.
>
> **`set_center` and `set_body_pose` both have two checks** (since 2026-09-28): ① each on its own — CoM ≤ half the upper-leg length (default ±20mm); ② **whether the foot ends can still reach**, combined with the current stride / height / lift / pose. When they can't reach, `ik_solve_2dof` just clamps the distance to the reachable maximum and **silently loses extension** (feet dragging on the ground, dog walking crooked — **no error**), so both checks are enforced. **Order no longer matters**: whether you lean the pose first and then set the parameters, or the other way around, both are caught.
>
> ② is much tighter than ①. At the default stride=70 / height=70 the foot already extends to 78.3mm with only 5.7mm left to the reachable maximum of 84 ⇒ **the CoM can actually shift at most ±11.4mm**, and **body pose \|pitch\| ≤ 3.7°, \|roll\| ≤ 6.2°**. To shift more / lean more, **reduce the stride first**: stride 60 → CoM 15mm passes; stride 40 → pitch up to 8° passes.
>
> ⚠ When `set_center` is rejected **nothing is written** to flash (both checks return before the NVS write). At boot, reading back NVS **rechecks only check ①'s magnitude** (stride/height are still at defaults then, so the combined check ② can't be computed) — so don't expect boot to catch combinations that were legal when stored but became unreachable with a later stride change.


## 5. Sensors & Attitude

### 5.1 IMU Debugging — bpuppy_imu


> Manual initialization: `bpuppy_imu.init(0, 14, 21, 0x68)`. The `start()` / `on()` of balance / heading_anchor / calib_mag start it automatically, so no manual step is needed. Stop: `bpuppy_imu.stop()` (stops the AHRS task; you can init again).

**Magnetometer fusion switch `set_mag_fusion`:**

```python
bpuppy_imu.set_mag_fusion(False)   # Magnetometer corrects yaw only, not involved in roll/pitch
bpuppy_imu.set_mag_fusion(True)    # Full 9-axis fusion (default)
```

> Magnetometer calibration residuals can pull roll/pitch off (attitude drifts when rotating about Z). `False` makes the magnetometer **handle heading (yaw) only**, with roll/pitch determined by the accelerometer (more accurate when level) while yaw stays stable. **balance uses this mode automatically**; switch back to `True` when you want full 9-axis (magnetometer participating in the horizontal plane).

**View raw data (accelerometer / gyro / magnetometer / temperature):**

```python
import bpuppy_imu
bpuppy_imu.init(0, 14, 21, 0x68)      # Initialize (idempotent)
import time
while True:
    a, g, m, t = bpuppy_imu.read_raw()
    print('acc=%.2f %.2f %.2f  gyro=%.2f %.2f %.2f  mag=%.1f %.1f %.1f  temp=%.1f' %
          (a[0],a[1],a[2], g[0],g[1],g[2], m[0],m[1],m[2], t))
    time.sleep_ms(500)
```

**View attitude angles (roll, pitch, yaw):**

```python
import bpuppy_imu
bpuppy_imu.init(0, 14, 21, 0x68)      # Initialize (idempotent)
import time
while True:
    r, p, y = bpuppy_imu.read_angles()
    print('roll=%.2f pitch=%.2f yaw=%.2f' % (r, p, y))
    time.sleep_ms(500)
```


### 5.2 IMU Calibration (gyro + accelerometer)

```python
bpuppy_imu.calibrate(300)     # Hold still for about 5 seconds; the result is saved to NVS automatically and persists across power-off
```

⚠ **The body must be level during calibration.** When you need it and what happens if you don't → the calibration section of [getting-started.en.md](getting-started.en.md).

### 5.3 Magnetometer Calibration — calib_mag

```python
import calib_mag
calib_mag.start()             # Guided 3D ellipsoid calibration; follow the serial prompts and rotate; result saved to NVS
```

Only needed for boards with a magnetometer (6-axis modules skip it automatically). How to judge the residual → the magnetometer section of [getting-started.en.md](getting-started.en.md).

### 5.4 Heading Lock — heading_anchor

The closed loop is split into two files, each handling half:

| File | Role |
|---|---|
| `heading_anchor.py` | **step1** — the err source from IMU heading. Public API is just four: `on` / `off` / `read` / `set` |
| `heading_follow.py` | **step2** — the closed-loop executor, `err → turn`. **It doesn't know where err comes from**, so vision / sound source can feed it too |

**Both are frozen into the firmware**, so just import and use.

> ⚠ **Don't manually copy these two `.py` files to the board anymore.** MicroPython's lookup order is
> `sys.modules` → **the board root directory** → firmware, so a leftover same-named `.py` on the board **shadows the firmware version** —
> the symptom is "the firmware was clearly upgraded, but the behavior is still old". If that happens, run `import os; os.remove('heading_anchor.py')`.
> (Uploading `pwm_ext_on.py` / `camera_on.py` and the like **doesn't conflict** — carry on.)

```python
import heading_anchor as anc

anc.on()        # On: lock the **current heading**
anc.on(30)      # On: turn **another 30°** from the current direction, then lock
anc.read()      # (on?, target angle, current yaw, error)
anc.off()       # Off: stop the loop + release steering
```

> ⚠ **`on()` takes "how much to turn", not "which angle to turn to"** — it's a **relative amount**.
> When the magnetometer is unreliable, absolute heading is untrustworthy anyway (measured indoors, it changes with **position**, not **orientation**),
> and only the relative amount holds up.

> ⭐ **Adjustable at any time while moving** — calling `anc.on(30)` again while already running just **changes the target**, and the loop
> picks it up on its next tick: no thread restart, `turn` never drops to 0, and the turn is pulled smoothly.

> ⚠ The **first** call spends 0.6 seconds doing a **quick check** of whether attitude is stable: if stable (e.g. the IMU was already
> initialized at the start of your program) it locks the target immediately; only if still in cold-start ramp-up does it do the full wait
> (about 5 seconds). **It will never lock a yaw that hasn't converged** — that would make the loop chase drift and spin the dog around.
> After boot, call it once while **standing** to "warm it up"; calls while moving then return instantly.

> ⚠ **`off()` only releases steering; it doesn't stop the dog** — what it stops is the "correct the heading" job. To stop the dog use "Stop" / `set_gait("stop")`.

Tuning — **everything has defaults; it runs without tuning**:

```python
anc.cfg                    # Show current parameters
anc.set(kp=2.5)            # Change one (takes effect immediately while locked; target unchanged)
anc.set()                  # No arguments = restore all defaults
```

**KittenBlock**: the two blocks live under "**Sensors**" (传感器功能) — "**Heading lock, turn [DEG]°**" (航向锁定 偏转 [ ] 度) and "**Release heading lock**" (解除航向锁定).
**Programs generated by KittenBlock initialize the IMU automatically at the top** (`bpuppy_imu.init(...)` was added to both the `libs`
injection header and `afterConnect`) — by the time the user clicks a block, attitude has long since converged, so the 0.6-second quick
check passes immediately and is **basically unnoticeable**.
(No need to click "Init IMU" manually anymore; doing so is just a no-op call, idempotent.)


### 5.5 Battery Voltage — bpuppy_adc

The battery divider is on **GPIO3 = ADC1_CH2** (ADC1 remains usable alongside BLE/WiFi) → enabled (`BPUPPY_ADC_ENABLE=1`). For hardware wiring see the battery voltage measurement section of `PCB/硬件连接.md` (51k/10k divider).

**Reading (recommended):**

```python
import bpuppy_led
v = bpuppy_led.batt_v()       # Calibrated battery voltage (V); -1.0 if monitoring isn't running
p = bpuppy_led.batt_pct()     # Battery percentage 0-100; 0 if monitoring isn't running
print("Battery: %.2f V  Charge: %d%%" % (v, p))
```

Or use the thin wrapper (the latter is recommended — it also makes sure the ADC + indicator LED are started):

```python
import voltage                # Imported automatically at power-on; no manual step needed
v = voltage.read_v()          # Same source as bpuppy_led.batt_v(); no second conversion
p = voltage.read_pct()        # Battery percentage 0-100 (integer)
```

**Battery percentage `read_pct()` (0-100):**

Scale: **7.4V = 100%, 6.6V = 0%** — the two endpoints are exactly the LED's blue/red thresholds, so
"percentage hits 0" and "the LED turns red" happen at **the same instant**; they can't disagree. Below 6.6V (including the red-flash zone) it's always 0;
above 7.4V (a real full charge is about 8.4V) it's clamped to 100.

```python
import voltage
print(voltage.read_pct())     # 0-100; 0 (not -1) if monitoring isn't running / the ADC isn't ready
```

> When no data is available it returns **0**, indistinguishable from "genuinely 0% charge" — to tell them apart also look at `read_v()`
> (`< 0` means no valid reading). The percentage is computed on demand from `s_batt_v` in the C layer, not cached separately.

> **There is only one calibration implementation.** The coefficients (a, b) live in the C layer, `drivers/led_driver.c`,
> persisted in NVS. `voltage.read_v()` / `read_pct()` return the very same values as `bpuppy_led.batt_v()` / `batt_pct()`
> — so **the LED color and the voltage/percentage you read can never disagree** (same code path).

**Under the hood (uncalibrated):**

```python
import bpuppy_adc
bpuppy_adc.init()                      # Initialize (V3.0: ADC1_CH2 GPIO3)
                                       #   ↳ also activates the GPIO48 WS2812 battery indicator
mv = bpuppy_adc.read_mv()              # ADC pin voltage (mV, after the divider)
batt_v = mv * 6.1 / 1000               # Uncalibrated battery voltage (V), converted for the 51k/10k divider
```


> **WS2812 battery indicator**: after `bpuppy_adc.init()`, the WS2812 on GPIO48 automatically lights according to voltage —
> ≥7.4V blue (full) / 6.6~7.4V blue→purple→red gradient / ≤6.6V red (low) / **<6.4V blinking red (danger)**.
> The two color thresholds **are** the 100% / 0% endpoints (both take the same set of C-layer constants).
> Manual control: `import bpuppy_led; bpuppy_led.set_color(r,g,b)` / `bpuppy_led.off()`. Thresholds are the constants at the top of `drivers/led_driver.c`.

---


### 5.6 Extension Servos — PWM_EXT

The main board reserves 3 extension servo outputs (for the 9th servo and beyond). For wiring, shared pins and the MCPWM principle, see "12. PWM_EXT Extension Servo Outputs" ("12. PWM_EXT 扩展舵机输出") in [硬件连接.md](../PCB/硬件连接.md).

**Smooth control (recommended — use `poses`, the same API as the main servos):**

Unified numbering: `0`~`7` = main servos, `8`/`9`/`10` = **PWM_EXT1/2/3**.

```python
import poses
poses.set_servo(9, 90)        # 9 = PWM_EXT2 (writes the buffer only, no movement)
poses.set_step(3)             # Transition speed in °/frame (default 3.0 = 150°/s)
poses.commit()                # All approach smoothly together; main and extension servos arrive at the same time
poses.get_servo_angle(9)      # Read current angle
poses.oscillate(9, 15, 1)     # Sine oscillation (ch, amp, hz, cycles)
```

> **There is only one easing implementation** (`poses._move_to`) — the easing algorithm is unrelated to "which servo is driven",
> only to "read angle / write angle". Main and extension servos approach together in the **same loop**.
> The `pwm_ext` module only does number mapping and pass-through read/write, **with no easing**.

**Raw control (no easing; takes effect immediately):**

```python
import pwm_ext
pwm_ext.on(2)                # Enable PWM_EXT2 — call it only in a power-on script (see 2.2)
pwm_ext.set_angle(2, 90)     # PWM_EXT2 jumps to 90° immediately (hard cut)
pwm_ext.get_angle(2)         # Read current angle
pwm_ext.get_pulse_us(2)      # Read the current pulse width 500~2500
pwm_ext.off(2)               # Release this channel
```

> In KittenBlock, the dropdowns of the three blocks "Set Servo [SERVO] to [ANGLE]°", "Servo [SERVO] Angle" and "Oscillate [SERVO2] …"
> already include **EXT1/EXT2/EXT3** — just use the blocks, no typing needed.

⚠ **Reading or writing a channel that isn't enabled raises an error** — it will not enable it for you. Pin functions are **fixed at power-on**.

**Auto-enable at power-on**: copy `mpy_modules/pwm_ext_on.py` to the board root (file present = on; delete it = off;
no rebuild needed). To choose which channels, edit the 0/1 values at the top of the file; which GPIO, which MCPWM channel, and whether
to stop the battery ADC first are all in the firmware — no need to worry about them.

⚠ PWM_EXT1 (GPIO3) shares a pin with the battery ADC and PWM_EXT3 (GPIO48) shares one with the WS2812 — enabling these two
**automatically stops battery monitoring first** (side effects: voltage reads -1.0, the battery LED goes dark). PWM_EXT2 (GPIO47) is a free pin with no side effects.

---


## 6. PC Tools

### 6.1 Serial Photo Capture — capture.py


With the ESP32 powered on, run this in a PC terminal:

```powershell
pip install pyserial
cd tools
python capture.py COM3   # replace with your actual port (check Device Manager)
```

Once connected, press **Enter** to take a photo (it saves and opens a picture preview automatically); type **q** to quit. The ESP32 is never restarted.

> How it works: the PC sends `import camera_serial; camera_serial.snap()` over serial → the ESP32 takes a photo → sends it back base64-encoded → the PC decodes it and saves it as JPEG.

---


## Appendix · Build / Flash / Serial

**Build**: Docker build, the version rule (MicroPython v1.22.1 **only** works with ESP-IDF v5.1.2) → "Build Environment" ("编译环境") in [AGENTS.md](../AGENTS.md).
**Flash**: for a new board's first flash use the bTool GUI → [getting-started.en.md](getting-started.en.md); command line / app-partition-only → "Windows 烧录" (Windows flashing) in [AGENTS.md](../AGENTS.md).
**Serial**: 115200, CH343, PuTTY / Tera Term / VS Code → "Serial Connection" ("串口连接") in [AGENTS.md](../AGENTS.md).
(The REPL must use **friendly REPL**; C-side `ESP_LOGx` output is discarded under raw REPL.)

Artifact: `build/micropython_bpuppy.bin`.
