# bPuppy Getting Started Guide

> 🌐 [中文](新板上电操作指南.md)

*Note: screenshots use bTool's English interface. Messages printed by the board's own REPL appear in Chinese, as shown where quoted.*

Once the PCB is soldered and the whole robot is assembled, some necessary flashing and calibration work still needs to be done. The exact order is:
flash firmware → upload files → calibrate


## 0. Preparation

**1) Download the bTool software**
[⬇ Download bTool.exe](https://github.com/bobyuhit/bTool/releases/latest/download/bTool.exe)

**2) Download the firmware and MicroPython program files**
**bPuppy system firmware**
| File | Purpose | Flash address |
|---|---|---|
| [micropython_bpuppy.bin](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/firmware/micropython_bpuppy.bin) | Application, **flash it every time** | `0x10000` |
| [bootloader.bin](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/firmware/bootloader.bin) | Bootloader, flash on first power-on / after changing the bootloader | `0x0` |
| [partition-table.bin](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/firmware/partition-table.bin) | Partition table, flash on first power-on / after changing the partition table | `0x8000` |

**3 .py files you can optionally upload to the MCU system's VFS** (upload them to the MCU system's root directory / VFS):

| File | Purpose |
|---|---|
| [pwm_ext_on.py](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/mpy_modules/pwm_ext_on.py) | Automatically enable the extension servo interface at power-on |
| [camera_on.py](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/mpy_modules/camera_on.py) | Automatically start the web video stream at power-on |
| [batt.py](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/mpy_modules/batt.py) | Battery sampling calibration tool |

**3) Hardware:**
Prepare one USB-A to Type-C data cable. Use it to connect your computer to the Type-C port used for flashing on the bPuppy control board. The control board has two Type-C ports; choose the one that the `◄CODE` marking on the board points to.
⚠ Do not turn on the robot dog's power switch at this point.

**4) Serial port connection**
Look for and select the serial port number that the control board is currently connected on.
![Example image](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/16.en.png)
Click the Connect button. If it succeeds, the icon at the top right of the serial port area changes from a cross to a check mark, and "Connected" appears in the status bar at the bottom of the window.

## 1. Flashing the Firmware
1. In the bTool window, check the top right corner to make sure the serial connection is OK (a check mark); in the page selector on the left side of the window, make sure you are on the download / flashing page (the downward arrow icon).
2. Click Add Firmware. On the first flash, import all of the files downloaded in the previous step: `micropython_bpuppy.bin`, `bootloader.bin`, `partition-table.bin`. For routine upgrades, select only **`micropython_bpuppy.bin`**.
![Example image](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/17.en.png)
3. Click the **Start Flashing** button.
- ⚠ Except for the first flash after powering on a new circuit board, never check "Erase entire Flash first" for any later flash, because it erases the system's calibration data.

✅ **Pass**: the flashing log shows the following at the end:

```
✓ Flash complete
[serial port reopened]
```

---

## 2. Uploading the Files
The bPuppy controller uses an ESP32-S3 module. The module's non-volatile storage is divided into three parts: **program space, VFS space, and NVS space**, used to store the low-level program, user programs (.py files), and system configuration data respectively.
The bPuppy control system is a mix of C and MicroPython programming; you can write your own MicroPython programs and upload them to the VFS space.
Recommended .py files to upload to the VFS (optional):

| File | Purpose | What happens if you don't upload it |
|---|---|---|
| `mpy_modules/pwm_ext_on.py` | Enable the extension servos at power-on (`EXT1/EXT2/EXT3` macros; only `EXT2` is enabled by default) | Reading or writing the extension servo ports reports "未启用" (not enabled) |
| `mpy_modules/camera_on.py` | Automatically start WiFi web remote control and video streaming at power-on | WiFi remote control and video streaming are turned off, but the robot uses less power |
| `mpy_modules/batt.py` | Dedicated battery calibration tool used on the first power-on | No impact |

Steps:

1. In the bTool window, check the top right corner to make sure the serial connection is OK (a check mark); in the page selector on the left side of the window, choose **File Manager** (the folder icon).
2. In the local file area, select the directory where the three .py files you just downloaded are.
3. Select the files in the left-hand file list and click **`Upload →`**.
4. Make sure the files have arrived in the device VFS file list on the right.
![Example image](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/18.en.png)
5. Unplug the USB data cable from the robot dog.

---

## 3. Servo Calibration

This robot uses extremely low-cost MG90S servos, whose load capacity and precision are both limited. To get better performance out of it, you must calibrate the servos as accurately as you can. The calibration data is stored in NVS storage and kept there long-term.
To make the servos run more precisely, every joint servo of the bPuppy gets a **0°/90°/180°** three-point calibration. In the servo control algorithm, the calibration results are used to interpolate the output angle so it is more accurate.

### 3.1 Servo Angle Conventions
First, the conventions for describing servo angles in this system are as follows:

| Joint | Channel | 0° | 90° | 180° |
|---|---|---|---|---|
| Left front hip `LF_HIP` | 0 | Points forward | Points down | Points backward |
| Left front knee `LF_KNEE` | 1 | Overlaps the hip | Points forward, at a right angle to the hip | Extended, in line with the hip |
| Left rear hip `LH_HIP` | 2 | Points forward | Points down | Points backward |
| Left rear knee `LH_KNEE` | 3 | Extended, in line with the hip | Points backward, at a right angle to the hip | Overlaps the hip |
| Right front hip `RF_HIP` | 4 | Points backward | Points down | Points forward |
| Right front knee `RF_KNEE` | 5 | Extended, in line with the hip | Points forward, at a right angle to the hip | Overlaps the hip |
| Right rear hip `RH_HIP` | 6 | Points backward | Points down | Points forward |
| Right rear knee `RH_KNEE` | 7 | Overlaps the hip | Points backward, at a right angle to the hip | Extended, in line with the hip |
> 1) **Left front hip** refers to the hip joint servo of the left front leg, and **right rear knee** refers to the knee joint servo of the right rear leg. The rest follow the same pattern.
> 2) **Points forward** means pointing toward the dog's head, parallel to the ground; **points backward** means pointing toward the dog's tail, parallel to the ground; **points down** means pointing toward the center of the earth, perpendicular to the ground.
> 3) When describing where a knee points, the hip is assumed to be in the 90° position, straight down.

The figure below shows the state when all joints are set to 90°:
![Example image](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/19.en.png)

### 3.2 Initial Alignment
1) Make sure the robot dog's power switch is **OFF**. Loosen all four screws on the robot dog that connect the servos to the hips, and take the legs off the hip servos; be careful not to unplug any wires, and make out which leg belongs to which hip servo.
2) Flip the robot dog's switch to ON to power it up. Note: the robot dog's legs will jerk around briefly the moment it powers on.
3) Plug in the USB data cable to connect the robot dog to your computer. Open the bTool software, find the robot dog's serial port, and click the plug icon button to connect to the robot dog.
4) In the page selector on the left side of the bTool window, find the **puppy icon** and click it to enter the robot dog settings page. Click **Servo Joint Calibration**, and on the page that pops up click **Turn All to 90°**. The robot dog's legs will jerk around briefly.
![Example image](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/20.en.png)
5) Do not turn the power off. Following the angles in the figure in section 3.1, remove and reinstall all the hip and knee servos.
- ⚠ The servos are holding their angles at this point, so do not force the servo shafts while installing, or you will damage the servos.
6) Because the servo shafts have very few spline teeth, no joint can actually reach exactly 90° just by reinstalling. Just get as close to 90° as you can; you will fine-tune it in the next step.

### 3.3 Servo Fine-Tuning
Once everything is installed, it is time to do the three-point calibration of the servos.
1) Start with the left front hip. You will see that the angle between the hip and the body is not a standard 90°, but slightly crooked.
![Example image](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/21.en.png)
2) On bTool's joint calibration page, select **Left front hip** with reference angle **90°**, try adding 5° to the **actual angle**, and click **Set**.
![Example image](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/22.en.png)
3) Watch how the left front hip angle changes — does it move **farther from** the vertical line, or **closer to** it? Depending on the result, keep adjusting the **actual angle** and clicking **Set** until the left front hip is, by eye, completely perpendicular to the body and pointing at the ground. Gently wiggle the left front hip; the play should be even front to back.
4) That completes the 90° calibration of the left front hip.
5) Next, do the 0° calibration of the left front hip. Select reference angle 0° in bTool, then click **Set**. The left front hip will rotate to point forward. There will still be an angle error.
![Example image](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/23.en.png)
6) Repeat the same steps — keep adjusting the **actual angle** and clicking **Set** — until the left front hip is completely parallel to the body and points forward.
7) Following the conventions in **3.1 Servo Angle Conventions**, continue adjusting the three calibration points of all the joints.
![Example image](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/24.en.png)
- ⚠ Everything must be set strictly according to the conventions in **3.1 Servo Angle Conventions**.
8) After all 8 servos are adjusted, close the **bTool Servo Joint Calibration** page.

## 4. IMU and Magnetometer Calibration
### 4.1 IMU
1) The bTool robot dog settings page has an IMU calibration button; click it and follow the prompts to calibrate.
2) Before calibrating, you must place the robot dog on a stable surface, and the flatter the body looks, the better.
3) Do not move the robot dog during calibration.

### 4.2 Magnetometer Calibration

1) The bTool robot dog settings page has a magnetometer calibration button; click it and follow the prompts to calibrate.
2) After calibration starts, pick up the robot dog and rotate it following the prompts (tilt left and right → tilt forward and backward → turn a full 360° horizontally). Once enough samples are collected, the software reports that collection succeeded.
✅ **Pass**: `✓ 校准完成! 残差: 0.0xxx`
> * Residual **<0.05**: good
> * Residual **>0.08** triggers a poor-quality warning → calibrate again

At this point, all the setup a new board needs is complete.
