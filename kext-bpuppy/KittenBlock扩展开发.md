# KittenBlock 硬件扩展开发指南（bPuppy 实例）

> 本文档面向**开发者/AI**，完整记录如何在 KittenBlock 上为 bPuppy 机器狗开发硬件扩展。
> 所有内容均在本项目实测验证过。改代码前先通读本文，尤其是「踩坑记录」章节。
> 配套文件：`extension.json`、`kblock.json5`（本目录），可对照阅读。

---

## 1. KittenBlock 扩展体系总览

KittenBlock 有两类扩展，**能力完全不同，不可混用**：

| 维度 | 软件扩展（角色扩展） | 硬件扩展（主控板扩展） |
|------|---------------------|----------------------|
| 目录前缀 | `s3ext-` | `kext-`（约定，非强制） |
| 文件 | 单个 `index.js` | 文件夹：`extension.json` + `kblock.json5` + 图标 |
| `extension.json` 的 `type` | `scratch3` | `micropy`（MicroPython）/ `arduino` |
| 代码翻译（积木→硬件代码） | ❌ 无 | ✅ `kblock.json5` 里 `pycode` 字段 |
| 上传到硬件 | ❌ | ✅ 串口 raw REPL |
| 加载方式 | URL 导入 / zip 导入 | **放扩展目录** 或 **zip 导入** |
| 典型扩展 | music、charts | futureboard-lite-mpy、otto@kext、mpython |

**关键结论（实测）**：
- bPuppy 扩展必须用 **`micropy` 硬件扩展**格式，否则无法「积木→MicroPython→上传」。
- URL 导入**只支持单个 `index.js` 软件扩展**。硬件扩展多文件（json5+json+png），**无法用 URL 导入**。分发靠 zip 导入或直接放扩展目录。

---

## 2. 文件结构与放置位置

### 2.1 文件结构

```
kext-bpuppy/
├── extension.json      # 必填。扩展配置主入口
├── kblock.json5        # 必填。积木定义 + 代码生成（extension.json 的 "file" 字段指定）
└── bpuppy.png          # 可选。图标，extension.json 的 "image" 字段指定
```

### 2.2 放置位置

KittenBlock 读取两个扩展目录（见用户数据 `kittenblock189.json`）：

| 路径 | 字段 | 作用 |
|------|------|------|
| `C:\Users\<用户>\AppData\Local\Kittenblock1.89\resources\app\extensions\` | `extpath` | 系统内置扩展 |
| `C:\Users\<用户>\AppData\Roaming\Kittenblock\local-ext\` | `extraext` | **zip 导入的落点** |

KittenBlock 扫描 `extpath` 下**所有含 `extension.json` 的文件夹**，把每个文件夹当作一个扩展。
- **zip 导入** = 解压 zip → 落进 `local-ext\<zip名>\`
- **手动放置** = 把 `kext-bpuppy` 文件夹整个拷进 `extpath`

> ⚠ 同名扩展冲突：如果 `extpath` 和 `local-ext` 同时存在同 id 扩展，KittenBlock 加载哪个不确定。改扩展后必须**清干净旧的**再重新导入。

---

## 3. extension.json 字段详解

`micropy` 硬件扩展的完整字段（对照本项目的 `extension.json`）：

| 字段 | 值（本项目） | 含义 |
|------|------------|------|
| `id` | `"bpuppy"` | 扩展唯一 ID，全局不能重复，改名会损坏旧项目 |
| `name` | `{"zh-CN":..., "en":...}` | 用户可见名称（硬件选择界面） |
| `version` | `"1.0.0"` | 语义化版本 |
| `type` | `"micropy"` | **关键**。决定下载/上传机制，硬件扩展必须 micropy |
| `fwtype` | `"esptool"` | 固件类型：uf2/bin/kflash/pio/esptool/scratch3 |
| `replmode` | `true` | 启用 REPL 在线执行 |
| `execmode` | `true` | 启用执行模式 |
| `kittenext` | `true` | 用 KittenBlock 扩展机制解析（必须 true） |
| `mainboard` | `true` | 是主控板（出现在硬件选择列表） |
| `buildinpyb` | `true` | 固件内置 pyb 支持 |
| `color1/2/3` | `#2DD4BF` 等 | 积木主体/下拉框/边框颜色 |
| `filesystem` | `"pyboard"` | 文件系统类型：disk/microbit/pyboard |
| `masterExt` | `"bpuppy"` | 主扩展名，一般等于 id |
| `io` | `["serial"]` | 通信方式：serial/tcp/ble |
| `connect` | `{baudrate:115200, batch:48, wait:5000}` | 串口连接参数（见 4） |
| `file` | `"kblock.json5"` | **主积木文件路径** |
| `ending` | `"\r\n"` | **关键**。发送到 REPL 的每行结尾 |
| `micropython` | `{mainfile:"main.py", no_execfile:true}` | MicroPython 专用配置 |
| `micropython.mainfile` | `"main.py"` | 上传后写入 VFS 的文件名 |
| `micropython.no_execfile` | `true` | 上传后是否直接 exec（true=不 exec，靠重启运行） |
| `image` | `"bpuppy.png"` | 图标文件名 |
| `imageconvert` | `{type:"png"}` | 图标格式 |
| `weights` | `990` | 硬件列表排序权重 |

### 完整示例（本项目实际使用的）

```json
{
    "id": "bpuppy",
    "name": { "zh-CN": "bPuppy 机器狗", "en": "bPuppy Robot Dog" },
    "version": "1.0.0",
    "type": "micropy",
    "fwtype": "esptool",
    "replmode": true,
    "execmode": true,
    "kittenext": true,
    "mainboard": true,
    "buildinpyb": true,
    "color1": "#2DD4BF",
    "color2": "#14B8A6",
    "color3": "#0D9488",
    "filesystem": "pyboard",
    "masterExt": "bpuppy",
    "io": ["serial"],
    "connect": { "baudrate": 115200, "batch": 48, "wait": 5000 },
    "file": "kblock.json5",
    "ending": "\r\n",
    "micropython": { "mainfile": "main.py", "no_execfile": true },
    "image": "bpuppy.png",
    "imageconvert": { "type": "png" },
    "weights": 990
}
```

---

## 4. 串口连接配置（connect）

`connect` 决定 KittenBlock 如何与硬件通信：

| 字段 | 本项目值 | 说明 |
|------|---------|------|
| `baudrate` | `115200` | **必须 115200**。bPuppy USB CDC 实际是虚拟串口，速率无物理意义，但 KittenBlock 的 raw REPL 握手只认 115200。实测 921600 上传失败 |
| `batch` | `48` | 每次写入字节数 |
| `wait` | `5000` | 连接握手等待时间（ms），要足够覆盖 bPuppy 启动 banner 输出 |

---

## 5. kblock.json5 详解

### 5.1 顶层结构

```json5
{
  libs: { ... },    // 库导入 + 自动注入代码（关键）
  blocks: [ ... ],  // 积木定义数组
  menus: { ... }    // 下拉菜单定义
}
```

### 5.2 libs — 自动注入代码（★核心设计）

`libs` 里定义每个积木**自动附带**的代码。`"*"` 表示对**所有积木生效**。

```json5
libs: {
  "*": {
    import: 'import bpuppy_motion, bpuppy_servo, time\nfrom time import sleep\n\n# ...初始化...'
  }
}
```

**生成位置**：`libs.import` 的代码注入到生成文件**最靠前**（import 之后、用户积木代码之前），且**无论多少条绿旗链都只注入一次**。

**关键实践**：**初始化代码放 `libs`，不要放积木 pycode**。如果放积木 pycode，每条绿旗链都会重复生成初始化。

```json5
// ✅ 推荐：初始化放 libs（自动注入一次）
libs: {
  "*": { import: 'import bpuppy_motion, bpuppy_servo, time\nfrom time import sleep\n\n_speed = 2.5\n...\nbpuppy_motion.stand_up()\ntime.sleep(1)' }
}

// ❌ 不推荐：初始化放积木 pycode（多条链会重复）
// { opcode:'init', blockType:'command', text:'初始化', pycode:'...' }
```

**转义**：json5 字符串里换行用字面 `\n`（反斜杠 n），**不是** `\\n`。本文件是 `.json5`，单引号字符串里的 `\n` 会被解析为换行。注意不要误写成 `\\n`（那是字面反斜杠+n，KittenBlock 不会换行）。

### 5.3 blocks — 积木定义

每个积木是一个对象，核心字段：

| 字段 | 说明 |
|------|------|
| `opcode` | 积木唯一 ID，全文件唯一 |
| `blockType` | `command`（执行）/ `reporter`（返回值）/ `boolean`（布尔）/ `hat`（事件触发） |
| `text` | 积木显示文字，参数用 `[参数名]` 占位 |
| `arguments` | 参数定义，键名对应 text 里的 `[参数名]` |
| `pycode` | **生成的 MicroPython 代码**，参数用 `[参数名]` 引用 |

### 5.4 arguments 参数类型

| type | 含义 | 额外字段 |
|------|------|---------|
| `number` | 数字输入框 | `defaultValue`（**必须是字符串**，如 `'2.5'`） |
| `string` | 文本输入框 | `defaultValue` |
| `slider` | 滑块（数字） | `defaultValue`、`min`、`max` |
| `value` | 变量引用 | — |
| `colorrgb` | 颜色选择器 | 生成 RGB 元组 |
| （配合 `menu`） | 下拉菜单 | `menu: '菜单名'`（对应 `menus`） |

> ⚠ `defaultValue` 用字符串，即使值是数字：`defaultValue: '2.5'` 而不是 `2.5`。这是从工作正常扩展（otto@kext、futureboard）观察到的惯例。

### 5.5 积木分类标题与分隔线

blocks 数组里可以混入字符串作为排版元素：

```json5
"## 运动",   // 分类标题（以 "## " 开头）
"---",       // 分隔线
```

### 5.6 menus — 下拉菜单

```json5
menus: {
  gaitMenu: [
    { text: '自适应', value: 'go' },
    { text: '猫步',   value: 'walk' },
    { text: '小跑',   value: 'trot' }
  ]
}
```

积木里引用：
```json5
{
  opcode: 'setGait',
  blockType: 'command',
  text: '切换步态 [GAIT]',
  arguments: {
    GAIT: { type: 'string', menu: 'gaitMenu', defaultValue: 'go' }
  },
  pycode: "bpuppy_motion.set_gait('[GAIT]')"
}
```

### 5.7 pycode 参数替换规则

- `[参数名]` 会被实际值替换
- **普通 string 文本框参数**（无 `menu`）：**手动加引号**。`pycode: "bpuppy_motion.set_gait('[GAIT]')"` → 生成 `set_gait('go')`
- **menu 下拉参数**（有 `menu` 字段）：**不要手动加引号**！KittenBlock 自动加。`pycode: "bpuppy_motion.set_gait([GAIT])"` → 生成 `set_gait('go')`。若写了 `'[GAIT]'` 会生成 `set_gait(''go'')` 双层引号 → **SyntaxError**
- 数字参数直接替换：`pycode: 'bpuppy_motion.set_lift([LIFT])'`
- `value` 类型参数：直接替换变量引用，不加引号

### 5.8 完整积木示例（本项目 15 块）

```json5
blocks: [
  "## 运动",
  {
    opcode: 'forward',
    blockType: 'command',
    text: '前进',
    pycode: "bpuppy_motion.set_params(_speed, _stride, _height); bpuppy_motion.set_gait('go')"
  },
  {
    opcode: 'backward',
    blockType: 'command',
    text: '后退',
    pycode: "bpuppy_motion.set_params(_speed, -abs(_stride), _height); bpuppy_motion.set_gait('go')"
  },
  {
    opcode: 'turnLeft',
    blockType: 'command',
    text: '左转',
    pycode: "bpuppy_motion.set_turn(-0.8); bpuppy_motion.set_params(_speed, _stride, _height); bpuppy_motion.set_gait('go')"
  },
  {
    opcode: 'turnRight',
    blockType: 'command',
    text: '右转',
    pycode: "bpuppy_motion.set_turn(0.8); bpuppy_motion.set_params(_speed, _stride, _height); bpuppy_motion.set_gait('go')"
  },
  {
    opcode: 'stop',
    blockType: 'command',
    text: '停止',
    pycode: "bpuppy_motion.set_gait('stand')"
  },
  "---",
  "## 参数",
  {
    opcode: 'setSpeed',
    blockType: 'command',
    text: '速度设为 [SPEED]',
    arguments: {
      SPEED: { type: 'number', defaultValue: '2.5' }
    },
    pycode: '_speed = [SPEED]; bpuppy_motion.set_params(_speed, _stride, _height)'
  },
  {
    opcode: 'setStride',
    blockType: 'command',
    text: '步幅设为 [STRIDE]',
    arguments: {
      STRIDE: { type: 'number', defaultValue: '70' }
    },
    pycode: '_stride = [STRIDE]'
  },
  {
    opcode: 'setHeight',
    blockType: 'command',
    text: '高度设为 [HEIGHT] mm',
    arguments: {
      HEIGHT: { type: 'number', defaultValue: '70' }
    },
    pycode: '_height = [HEIGHT]; bpuppy_motion.set_params(_speed, _stride, _height)'
  },
  {
    opcode: 'setLift',
    blockType: 'command',
    text: '抬腿高度 [LIFT] mm',
    arguments: {
      LIFT: { type: 'number', defaultValue: '30' }
    },
    pycode: 'bpuppy_motion.set_lift([LIFT])'
  },
  "---",
  "## 步态",
  {
    opcode: 'setGait',
    blockType: 'command',
    text: '切换步态 [GAIT]',
    arguments: {
      GAIT: { type: 'string', menu: 'gaitMenu', defaultValue: 'go' }
    },
    pycode: "bpuppy_motion.set_gait('[GAIT]')"
  },
  "---",
  "## 姿态动作",
  {
    opcode: 'crouch', blockType: 'command', text: '蹲下',
    pycode: "bpuppy_motion.set_gait('crouch')"
  },
  {
    opcode: 'sit', blockType: 'command', text: '坐下',
    pycode: "bpuppy_motion.set_gait('sit')"
  },
  {
    opcode: 'stand', blockType: 'command', text: '站立',
    pycode: "bpuppy_motion.set_gait('stand')"
  },
  {
    opcode: 'playBow', blockType: 'command', text: '邀玩',
    pycode: "bpuppy_motion.set_gait('play')"
  },
  {
    opcode: 'waveBlock', blockType: 'command', text: '挥手',
    pycode: "bpuppy_motion.set_gait('wave')"
  }
],

menus: {
  gaitMenu: [
    { text: '自适应', value: 'go' },
    { text: '猫步',   value: 'walk' },
    { text: '小跑',   value: 'trot' },
    { text: '站立',   value: 'stand' },
    { text: '蹲下',   value: 'crouch' }
  ]
}
```

---

## 6. 代码生成机制（★理解核心）

KittenBlock 把图形化积木翻译成 MicroPython，规则：

1. **程序入口 = 内置「当绿旗被点击」hat 块**。积木必须接在绿旗下面，代码面板才会生成代码。
2. **自定义 hat 块 ≠ 程序入口**。自定义 hat 块（如 blynk 的 `whenConnected`）会被当成**末尾函数**生成（`@装饰器` + `def 函数():`），代码放在文件末尾，不是开头。**不要试图用自定义 hat 当入口**。
3. **libs.import 代码** 注入到文件**最靠前**，且**只一次**。初始化应放这里。
4. 积木 pycode 按积木连接顺序逐条生成。

### 生成代码结构示例

用户拖：`当绿旗被点击 → 前进`，生成：

```python
import bpuppy_motion, bpuppy_servo, time
from time import sleep

# ===== bPuppy 自动初始化 =====     ← libs 注入（最前，一次）
_speed = 2.5
_stride = 70
_height = 70
bpuppy_servo.init_all()
...
bpuppy_motion.stand_up()
time.sleep(1)

# ===== 用户积木 =====
bpuppy_motion.set_params(_speed, _stride, _height); bpuppy_motion.set_gait('go')
```

---

## 7. 上传与运行机制

### 7.1 上传通道

KittenBlock 通过串口 MicroPython **raw REPL paste 协议**上传（控制字节 `\x01`、`\x04` 等）。关键参数：

- `connect.baudrate = 115200`（必须）
- `ending = "\r\n"`（REPL 命令结尾）
- `micropython.mainfile = "main.py"`（上传写入 VFS 的文件名）

### 7.2 运行流程（bPuppy 固件侧）

bPuppy 固件的 `frozen/main.py` 启动逻辑（本项目已实现）：

```
上电 → frozen main.py
  → 挂载 VFS 分区 (esp32 Partition, label='vfs')
  → 检查 /main.py 是否存在
    → 存在 → exec(用户程序)，不跑原厂逻辑
    → 不存在 → 跑原厂逻辑（站立 + WiFi 遥控）
```

**上传后需要重启 bPuppy** 才会执行新程序（`no_execfile: true`）。

### 7.3 VFS 挂载（bPuppy 特有）

bPuppy 固件的 VFS 分区**不会自动挂载**，必须在 frozen main.py 里手动挂载：

```python
from esp32 import Partition
import os, uos
bdev = Partition.find(1, label='vfs')  # 1 = TYPE_DATA
if bdev:
    try:
        uos.mount(uos.VfsFat(bdev[0]), '/')   # 已格式化 → 直接挂载
    except:
        uos.VfsFat.mkfs(bdev[0])              # 首次 → 格式化
        uos.mount(uos.VfsFat(bdev[0]), '/')
```

> ⚠ `mkfs` 会**抹掉整个 VFS**。绝不能每次启动都执行，必须"先尝试挂载，失败才格式化"。

### 7.4 恢复原厂

删除 VFS 里的 `/main.py`，重启后 frozen main.py 会跑原厂逻辑：

```python
import os
os.remove('/main.py')   # 需先挂载 VFS
import machine
machine.reset()
```

---

## 8. 测试流程（改扩展后必须走）

1. 更新 `kext-bpuppy/` 下文件
2. **清理旧版本**：删掉 `extpath` 和 `local-ext` 里所有旧 bPuppy（`kittenblock189.json` 的 extpath/extraext 指向）
3. 重新打包 zip：`Compress-Archive -Path extension.json,kblock.json5,bpuppy.png -DestinationPath bpuppy-kittenblock.zip -Force`（**进入文件夹选文件压缩，不要压文件夹本身**）
4. 重启 KittenBlock
5. zip 导入 → 硬件列表出现 bPuppy
6. 拖 `当绿旗被点击 → 前进`，检查代码面板：
   - 初始化只出现一次且在开头
   - 积木代码正确
7. 串口连 bPuppy → 上传 → 重启 → 狗动

---

## 9. 踩坑记录（全部实测，务必读）

### 9.1 波特率 921600 上传失败 → 必须 115200

**现象**：KittenBlock 显示"上传成功"，但 VFS 里没有文件，狗不动。
**原因**：KittenBlock 的 raw REPL 握手在 921600 下不通，但 KittenBlock 不报错。
**解法**：`connect.baudrate` 设 `115200`。bPuppy 是 USB CDC 虚拟串口，速率无物理意义，115200 完全够。

### 9.2 缺 `ending: "\r\n"` → REPL 无响应

**现象**：连接后命令发出去没反应。
**原因**：MicroPython REPL 需要 `\r\n` 结尾。
**解法**：extension.json 加 `"ending": "\r\n"`。

### 9.3 `mkfs` 每次启动抹掉用户程序

**现象**：上传成功但重启后 `/main.py` 消失，VFS 空。
**原因**：frozen main.py 里 `uos.VfsFat.mkfs(bdev)` 每次启动都执行，格式化整个分区。
**解法**：先 `try: mount`，`except: mkfs + mount`。

### 9.4 zip 导入目录结构

**现象**：zip 导入失败。
**原因**：打包时把整个文件夹压进去了，zip 根目录是 `kext-bpuppy/` 而不是三个文件。
**解法**：**进入文件夹内部**，选中三个文件压缩。zip 根目录必须直接是 `extension.json` 等文件。

### 9.5 local-ext 旧版残留 → 扩展列表不刷新

**现象**：改了扩展但 KittenBlock 还是旧版；移走了 extpath 里的扩展，硬件列表里还有 bPuppy。
**原因**：zip 导入的旧版解压在 `local-ext`（`kittenblock189.json` 的 `extraext`），移 extpath 没用。
**解法**：改扩展前**先清空 `local-ext` 里对应文件夹**，再重新导入。

### 9.6 初始化放积木 pycode → 多条绿旗链重复初始化

**现象**：两条绿旗链时初始化代码出现两次。
**原因**：初始化写在 command 积木的 pycode 里，每个被拖到的位置都生成。
**解法**：初始化移到 `libs.import`，自动注入一次。

### 9.7 代码面板空 → 积木没接绿旗

**现象**：拖了积木但代码面板是空的。
**原因**：积木没有接在任何 hat 块下面，代码生成器找不到起点。
**解法**：必须接在「当绿旗被点击」下面。

### 9.8 menu 参数被双层加引号 → SyntaxError

**现象**：切换步态积木生成 `bpuppy_motion.set_gait(''go'')`，报语法错误。
**原因**：`menu` 下拉参数替换时 KittenBlock **自动加引号**，pycode 里又手动写了 `'[GAIT]'` → 双层引号。
**解法**：menu 参数 pycode 写 `[GAIT]`，**不加引号**。普通 string 文本框参数才需要手动加引号。详见 5.7。

### 9.9 积木加载了但 `pycode` 不翻译

**现象**：积木出现在面板，能拖，但生成的代码只有 import 没有积木代码。
**原因**：多为格式问题（如 `pycode` 用了 `\\n` 字面量、`defaultValue` 类型不对、`gen` 字段误用）。
**解法**：对照第 5 节格式；用最小版本（单个积木）排除法。

---

## 10. bPuppy 固件配合改动

本项目为了支持 KittenBlock 上传运行，改了固件 `frozen/main.py`：

- 上电挂载 VFS（见 7.3）
- 有 `/main.py` 则 exec 用户程序，否则跑原厂逻辑

改这部分后需重新编译烧录固件：

```bash
bash build.sh     # 项目根目录（Docker 编译）
esptool --chip esp32s3 --port COM3 --baud 921600 write-flash 0x10000 build/micropython_bpuppy.bin
```

---

## 11. bPuppy 现有扩展积木清单

| 分类 | 积木 | pycode 生成 |
|------|------|------------|
| 运动 | 前进 / 后退 | `set_params(_speed, ±_stride, _height); set_gait('go')` |
| 运动 | 左转 / 右转 | `set_turn(±0.8); set_params(...); set_gait('go')` |
| 运动 | 停止 | `set_gait('stand')` |
| 参数 | 速度设为 / 步幅设为 / 高度设为 / 抬腿高度 | 更新变量 + `set_params` / `set_lift` |
| 步态 | 切换步态 [下拉] | `set_gait('[GAIT]')` |
| 姿态 | 蹲下 / 坐下 / 站立 | `set_gait('crouch'/'sit'/'stand')` |
| 动作 | 邀玩 / 挥手 | `set_gait('play'/'wave')` |

底层 API（固件 C 模块，MicroPython 可调）：
- `bpuppy_motion.set_params(speed, stride, height)` — speed 0~10, stride 正前负后, height mm
- `bpuppy_motion.set_gait('go'/'walk'/'trot'/'stand'/'sit'/'crouch'/'play'/'wave')`
- `bpuppy_motion.set_turn(-0.8~0.8)`
- `bpuppy_motion.set_lift(mm)`
- `bpuppy_servo.init_all()` / `bpuppy_servo.load_cal()` / `bpuppy_servo.set_angle(ch, deg)`

---

## 12. 参考

- KittenBlock 插件开发指南 01/02：积木定义、串口通信基础
- 本目录 `extension.json`、`kblock.json5`：可直接复制修改
- KittenBlock 自带参考扩展：`extensions/otto@kext/`（micropy 硬件扩展）、`extensions/mpython/`、`extensions/futureboard-lite-mpy/`
