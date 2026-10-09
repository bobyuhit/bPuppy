# bPuppy KittenBlock Guide

> 🌐 [中文](kittenblock图形化编程指南.md)

This document is written for users who program the bPuppy robot dog with **KittenBlock** graphical blocks.

> **Quick URL Reference**
>
> KittenBlock online programming URL:
> ```
> https://kblock.kittenbot.cc/
> ```
>
> KittenBlock extension import URL:
> ```
> https://raw.githubusercontent.com/bobyuhit/bPuppy/master/bpuppy-kittenblock.zip
> ```

## 1. KittenBlock Graphical Programming

**KittenBlock** is graphical programming software for open-source hardware and artificial intelligence (AIoT), created by Kittenbot for youth STEAM education, K-12 information technology teaching, and hobbyist DIY. It is built as a secondary development based on the Scratch 3.0 platform from the Massachusetts Institute of Technology (MIT).

With KittenBlock you can program the bPuppy: either **control it online**, or **upload a program to the robot dog**; after a program is uploaded, the robot dog runs it by itself on every power-on.

KittenBlock can be used either online or through a local desktop client; online programming is recommended (extension versions are updated more promptly there). The online programming URL is:

```
https://kblock.kittenbot.cc/
```

or

```
https://kblock.kittenbot.cn/
```

Both are official KittenBlock websites and both can be used, but they do not share user data. We recommend registering an account before use: a registered user's programs can be saved in the cloud and are less easily lost.

### ★ For iPad/iPhone users, the Bluefy browser is recommended

On iPad/iPhone, the Web Bluetooth feature of Safari/Chrome is restricted, so you cannot connect to the robot dog over Bluetooth directly. Please use **Bluefy** (a Web Bluetooth browser made specially for iOS): search for **Bluefy** (Web BLE Browser) in the App Store and install it.

## 2. Import the bPuppy Extension

### 2.1 How to Import

**Import by URL (recommended)**

1. Open the KittenBlock website and click the "**添加扩展**" (Add Extension) button in the bottom-left corner.
   ![The "添加扩展" (Add Extension) button in the bottom-left corner](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/1.png)

2. On the "添加扩展" (Add Extension) page, first click the "**用户扩展**" (User Extensions) tab at the top, then click "**URL 导入**" (Import by URL).
   ![用户扩展 (User Extensions) → URL 导入 (Import by URL)](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/2.png)

3. In the window that pops up, paste the URL below and click "**+ 导入扩展**" (+ Import Extension):
   ```
   https://raw.githubusercontent.com/bobyuhit/bPuppy/master/bpuppy-kittenblock.zip
   ```
   ![Paste the URL and click "+ 导入扩展" (+ Import Extension)](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/3.png)

4. After loading completes, click "**返回**" (Back) in the upper-left corner to return to the main page, then click "**选择硬件**" (Select Hardware) in the upper-left corner.
   ![After returning to the main page, click "选择硬件" (Select Hardware)](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/4.png)

5. In the hardware list, find "**bPuppy 机器狗**" (bPuppy Robot Dog) and click to select it.
   ![Select "bPuppy 机器狗" (bPuppy Robot Dog) in the hardware list](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/5.png)

6. A "**bPuppy 机器狗**" (bPuppy Robot Dog) category appears at the very bottom of the left category column (at the arrow in the figure below); all of the bPuppy blocks are collected in it.
   ![The "bPuppy 机器狗" (bPuppy Robot Dog) category at the bottom of the left column](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/6.png)

You can now start block programming. The block area on the left contains 7 categories of bPuppy blocks in total:

```
🟠 Pose Actions: Stand / Crouch / Sit / Play / Wave / Wait [1] s
🟢 Motion Control: Stop / Forward / Backward / Turn Left / Turn Right
🎙️ Voice: when receive [VOICE▼] (dropdown: stop / forward / backward / turn left / turn right / speed up / speed down / nod / stand / crouch / sit / wave / play / report voltage / sound direction) / (sound angle) / bark [wang wang / ying ying▼] / report number [0-100]
🟣 Servo Control: Set Servo [LF Hip▼] to [90]° / Transition Speed [3]°/frame / Execute Pose / Servo [LF Hip▼] Angle / Oscillate [SERVO▼] ±[10]° [4]Hz [8] (oscillation also belongs to this category)
🔵 Advanced Motion Control: Motion Params  stride [70] mm  height [70] mm  lift [30] mm / (Stride) (Height) (Lift) / Speed [2.5] dir [forward▼] / (Speed) (Direction) / Switch Gait [Auto▼] / Set Turn [0] / (Turn) / Body Pose  pitch [0] deg  roll [0] deg / <Succeeded?>
🟡 Sensors: Init IMU / (Roll) / (Pitch) / (Yaw) / Heading lock, turn [30]° / Release heading lock
⚙️ System Settings: Center Offset [0] mm   ⚠ Stored on the board (kept across power-off)
```

For detailed usage of each block category, see [Section 4, "Graphical Programming"](#4-graphical-programming).

## 3. Connect the Robot Dog

1. Turn on the robot dog's power.
2. Make sure Bluetooth is turned on on the programming device (computer / iPad / phone, etc.).
3. Click "**没有连接**" (Not connected) at the top of the KittenBlock page.
   ![Click "没有连接" (Not connected) at the top](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/7.png)
4. In the window that pops up, choose "**蓝牙连接**" (Bluetooth connection) (choose "**数据线连接**" (Cable connection) if you connect with a USB data cable).
   ![Choose "蓝牙连接" (Bluetooth connection)](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/8.png)
5. When the browser's pairing window pops up, select the device **bPuppy_XXXX** (XXXX differs for each unit) and click "**配对**" (Pair).
   ![Browser pairing window: select the device and click "配对" (Pair)](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/9.png)
6. Back in KittenBlock's connection window, click "**连接**" (Connect) on the right side of the device row.
   ![Click "连接" (Connect) on the right side of the device row](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/10.png)
7. Wait until the "**交互模式已打开**" (Interactive mode opened) message appears at the top of the page; the connection is now established.
   ![The "交互模式已打开" (Interactive mode opened) message](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/11.png)

At this point the bPuppy robot dog is connected, and you can start block programming.

## 4. Graphical Programming

The bPuppy extension's block area on the left is divided into **7 categories** by function; this chapter introduces them one by one in the same order.

| Category | Purpose |
|---|---|
| 🟠 Pose Actions | Fixed poses and performance actions |
| 🟢 Motion Control | Walking, turning, stopping |
| 🎙️ Voice | Voice-command events and sound output |
| 🟣 Servo Control | Fine control of individual servos |
| 🔵 Advanced Motion Control | Motion parameter settings and readouts |
| 🟡 Sensors | IMU-based attitude-angle readouts and heading lock |
| ⚙️ System Settings | System-level parameters that are written to the robot dog and kept across power-off |

### 4.0 Blocks — the Foundation of Graphical Programming

These blocks each have their own function, and all of them can be dragged out of the side palette and placed in the workspace. Once several blocks are magnetically snapped together, they form a program that can make the robot dog complete complex tasks — that is **graphical programming**.

This system has a very convenient feature: **online debugging**. While writing a program, you do not have to wait until the whole program is finished and then download it to the robot dog to run it. As long as the robot dog is connected, you can click with the mouse on any block or block group in the programming area or in the side palette, and the robot dog will immediately run that function or program segment — very convenient for debugging.

After the program is finished, you can click "**代码**" (Code) to open the **code window**, then click the "**上传**" (Upload) button on the code box. After a moment, the program designed in KittenBlock has been downloaded into the robot dog. The next time it is powered on, the robot dog runs this program automatically.

![Open the code window and click the "上传" (Upload) button](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/12.png)

There are some functional differences between **online debugging mode** and **download-code mode**:

1. A program that is to be downloaded to the robot dog and run there must start from the "**当绿旗被点击**" (when green flag clicked) event block, and the blocks connect head to tail from top to bottom. For example, this program:

   ![Sample program: green flag → Forward → Wait 2 s → Stop → Wait 1 s → Wave](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/13.png)

   After this program is downloaded to the robot dog, the robot dog will automatically do the following the next time it is powered on: **go forward for two seconds → stop → wait 1 second → wave**.
   **Online debugging** mode, in contrast, runs without depending on the "当绿旗被点击" (when green flag clicked) block.

2. Voice-event blocks and keyboard blocks have **exactly opposite** requirements for how they run: **voice events** (left in the figure below) only take effect after being **downloaded to the robot dog and reset**, and do not work in online debugging; **keyboard blocks** (right in the figure below) **can only run online** — key presses are detected in the computer's browser, so after downloading to the board, pressing any key has no effect.

   ![Left: voice-event blocks; right: keyboard blocks](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/14.png)

### 4.1 Pose Actions 🟠

Make the robot dog hold a fixed pose, or perform a short action.

| Block | Effect |
|---|---|
| Stand | The four legs extend; stands at a fixed height of 70mm |
| Crouch | The four legs tuck in; crouches low |
| Sit | Front legs extended, rear legs folded (cat sit) |
| Play | Front low, rear high; shakes its hips 8 times |
| Wave | Sits down first, then swings the right front leg 3 times |
| Wait [1] s | Delay; performs no action |

These blocks wait for the previous action to finish before executing the next one in sequence.

### 4.2 Motion Control 🟢

| Block | Effect |
|---|---|
| Forward / Backward | Keeps walking forward / backward |
| Turn Left / Turn Right | Turns while walking |
| Stop | Finishes the step and straightens up, all four legs standing (not an emergency power-cut stop) |

- The direction blocks are **commands**: they return immediately after execution, and the robot dog keeps walking until "Stop" or the next motion command.
- ⚠ All four direction blocks use the "Auto (go)" gait.
- ⚠ In this system, the go gait's stride / standing height / lift are **calculated automatically from the speed** — the "Motion Params" block in Advanced Motion Control has no effect on the go gait.
- ⚠ The turning actions in this section are not controlled by the **turn rate** in Advanced Motion Control. They **override the turn rate** instead: Forward/Backward reset the **turn rate** to zero, while Turn Left / Turn Right set the **turn rate** to -0.8 / +0.8 respectively.

### 4.3 Voice 🎙️

Used together with the CI-33T voice module: say a command word and the robot dog responds; you can also make the dog produce sounds. There are 4 blocks in total:

| Block | Description |
|---|---|
| when receive [VOICE▼] (hat-shaped block) | Triggers when the voice module recognizes that command, and the blocks attached below it are then executed. |
| (sound angle) | Direction of the most recent sound source (0~180°); `-1` = none received since this power-on |
| bark [wang wang / ying ying] | Plays one dog bark |
| report number [0~100] | Makes the module speak the specified number |


- The "sound angle" variable changes with the sound-source direction of valid commands: **0°~180° are valid directions**, and `-1` means "no valid voice command has been received yet".

  ![Sound-angle diagram: 0° is on the robot dog's right, 180° on its left; straight ahead and straight behind are both 90°](https://raw.githubusercontent.com/bobyuhit/bPuppy/master/docs/PIC/15.png)


- ⚠ **Voice events only take effect after "downloading to the board + reset"**; they do not work in online mode.
- The "report voltage" command has a built-in action: on receipt, the robot dog automatically announces the battery percentage — no programming needed.

### 4.4 Servo Control 🟣

Bypass the gait engine and directly control the 8 leg servos plus 3 channels of extension servos, for fine pose adjustments or custom actions.

| Block | Description |
|---|---|
| Set Servo [LF Hip▼] to [90]° | Sets the target angle (only records the target, **does not execute immediately**) |
| Transition Speed [3]°/frame | Speed of the smooth approach (1 frame = 20ms) |
| Execute Pose | Smoothly moves to the target angles |
| Servo [LF Hip▼] Angle | Reads the current angle |
| Oscillate [SERVO▼] ±[10]° [4]Hz [8] | A single servo oscillates back and forth in a sine wave; returns when finished |

- Typical use: set servo 1's angle → set servo 2's angle → … → set the transition speed → Execute Pose.
  If you only **set angles** without **Execute Pose**, the servos will not move.
- EXT1~EXT3 are extension servos; **this feature must first be enabled on the board** (see [micropython-guide.en.md](micropython-guide.en.md), Section 5.6).
- ⚠ If the robot dog is walking and you use commands from this section to force-drive the leg servos, the robot dog exits the motion gait.

### 4.5 Advanced Motion Control 🔵

Users can configure the robot dog's special gaits to complete more complex tasks. This category is arranged in "setting → readout" pairs:

| Setting block | Corresponding readout | Description |
|---|---|---|
| Motion Params: stride / height / lift | (Stride) (Height) (Lift) | The three are: how far each step reaches, how high the robot dog stands, and how high the foot lifts when taking a step; one block sets them and validates them together |
| Speed [2.5] dir [forward▼] | (Speed) (Direction) | Speed = step frequency (0~10) |
| Set Turn [0] | (Turn) | Controls the amount of turning by setting the stride ratio between the left and right legs: range -1~1. A positive value turns the robot dog right, a negative value turns it left; at ±1 the legs on the two sides move in opposite directions — the fastest turn; at ±0.5 one side keeps its normal stride while the other side's stride is 0; at 0 the robot dog walks straight without turning. |
| Switch Gait [Auto▼] | — | Auto (go) / Walk / Trot / Stop; switching resets the turn rate to zero |
| Body Pose pitch / roll | — | Sets the body's pitch angle and roll angle; once set they are held and do not reset automatically |
| — | <Succeeded?> | Whether the most recent setting block **really took effect** |

- All setting blocks **take effect immediately**; the next movement uses the new values.
- **Gait details**:
  **"Walk"** — at any moment, at most one leg is lifted. A stable gait; recommended for speeds below 5. The lift height can be set somewhat higher, making it easier to step over obstacles.
  **"Trot"** — the diagonal legs form two pairs (left front with right rear, right front with left rear), and the two pairs lift alternately. A light gait, suited to fast forward motion; recommended speed 5 or above. It is also recommended to reduce the lift height to below 10 so the servos do not fall behind the command rate.
  **"Auto (go)"** — at speed ≤ 4 it walks (one leg lifted at a time; stable), at ≥ 6 it trots (two diagonal legs lifted together; fast), with a continuous transition in between. In the go gait, the stride / standing height / lift are calculated automatically from the speed and are not constrained by the motion parameters.
  **"Stop"** — the machine stops and all four legs stand. Unlike the **Stand** pose, the body height in this state is the **standing height** from the motion parameters.

- **Out-of-range parameters**: unreasonable parameter settings can cause mechanical collisions or the mechanism moving past its limits; such settings are rejected by the system, which keeps the old parameters. When a parameter setting is rejected, the robot dog makes the "ying ying" sound. Try adjusting the parameters and setting them again.

> Commonly used limits with the default parameters (stride 70 / standing height 70): pitch ±3.7°, roll ±6.2°, center offset ±11.4mm. To push a larger pose or shift the center of gravity more, reduce the stride first.

### 4.6 Sensors 🟡

Based on the on-board IMU (attitude sensor):

| Block | Description |
|---|---|
| Init IMU | This system's IMU consists of an accelerometer and a gyroscope (models with a magnetometer are 9-axis); the chip model is recognized automatically by the firmware (MPU6050 / MPU9250). It is initialized automatically at the start of the program, so manual calls are usually unnecessary. |
| (Roll) / (Pitch) / (Yaw) | Reads the body's three-axis attitude angles (degrees) |
| Heading lock, turn [30]° | The robot dog records its current body direction, **turns another 30°**, then locks that heading |
| Release heading lock | Turns off the heading lock |

- **Heading lock**: when this command runs, it reads the robot dog's heading at that moment and adds the set angle, using the result as the target direction of motion.
- After heading lock is engaged, as soon as the robot dog switches to a walking mode, it automatically turns to the target direction and locks onto it while walking; even if the angle is knocked off course in the process, it turns itself back.
- You can use this command while walking to change the target angle; it takes effect immediately and does not interrupt the walking state.
- On first use, there may be a short wait for the attitude data to converge before it takes effect (several seconds after a cold power-on).

### 4.7 System Settings ⚙️

| Block | Description |
|---|---|
| Center Offset [0] mm | Fore-aft offset of the foot neutral position, used to adjust the center-of-gravity distribution |

- Physically, the robot dog's center of gravity may not fall exactly on the midpoint between the front and rear legs, and an offset center of gravity makes the gait unstable. Adjusting this parameter moves the toe position while standing (the midpoint of the stride) forward or backward, making the gait stable.
- Once written, the value **survives power-off** (each setting writes to flash once) — after tuning, there is no need to set it repeatedly.
- With the default parameters the maximum is ±11.4mm (it constrains and is constrained by stride / pitch / roll; out-of-range values are rejected, see 4.5).
