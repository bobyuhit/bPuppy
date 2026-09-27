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
| 加载方式 | 放扩展目录 | **URL 导入 / zip 导入 / 放扩展目录** |
| 典型扩展 | music、charts | futureboard-lite-mpy、otto@kext、mpython |

**关键结论（实测）**：
- bPuppy 扩展必须用 **`micropy` 硬件扩展**格式，否则无法「积木→MicroPython→上传」。
- **URL 导入支持 `micropy` 硬件扩展**（zip 顶层含 `extension.json` 即可）。单个 `index.js` 软件扩展**反而不能** URL 导入（无 extension.json，校验失败）。详见第 2.3 节。

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
- **URL 导入** = 下载 zip → 解压 → 落进 `local-ext\<zip名>\`（见 2.3）
- **zip 导入** = 解压 zip → 落进 `local-ext\<zip名>\`
- **手动放置** = 把 `kext-bpuppy` 文件夹整个拷进 `extpath`

> ⚠ 同名扩展冲突：如果 `extpath` 和 `local-ext` 同时存在同 id 扩展，KittenBlock 加载哪个不确定。改扩展后必须**清干净旧的**再重新导入。

### 2.3 URL 导入机制（实测源码确认）

KittenBlock 的「URL 导入」（扩展 → 用户扩展 → URL 导入）本质：

1. **接受两种 URL**：
   - `.zip` 地址（http/https，下载并解压）
   - `.git` 仓库地址（git clone 到 `local-ext\<仓库名>\`）
   - 其他形式 → 提示「无效链接」（`efficacy`）
2. **校验**：zip 顶层 entry 必须含 `extension.json`，否则「无效 Zip 文件」（`error`）。这就是为什么单 index.js 软件扩展不能 URL 导入。
3. **解压落点**：
   - zip 顶层是文件（散在根）→ 解到 `local-ext\<zip名去扩展名>\`
   - zip 顶层是目录 → 解到 `local-ext\` 保留目录名
4. **刷新**：解压后 `reboot-web` 刷新编辑器，重新枚举扩展。

**GitHub 发布方式**（本项目实测）：

| 方式 | URL | 说明 |
|------|-----|------|
| **raw 链接（推荐）** | `https://raw.githubusercontent.com/<user>/<repo>/master/bpuppy-kittenblock.zip` | 把 zip 提交进仓库根目录，push 后立即可用。已验证成功 |
| Release 附件 | `https://github.com/<user>/<repo>/releases/download/v1.0.0/bpuppy-kittenblock.zip` | URL 更短更稳定，需网页创建 Release |

**注意事项**：
- URL 必须以 `.zip` 结尾，短链服务会失败
- raw URL 国内访问可能慢（GitHub 通病）
- 不建议用整仓库 `.git` URL clone（大仓库慢，扩展在子目录非顶层）

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
| `io` | `["serial","ble"]` | 通信方式：serial/tcp/ble（本项目串口 + 蓝牙两条通道都启用） |
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
    "io": ["serial", "ble"],
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
  "*": { import: 'import poses, bpuppy_motion, bpuppy_servo, bpuppy_imu, time\nfrom time import sleep\n\n_speed = 2.5\n_stride = 70\n_height = 70\n\n# 原厂已 init_all + 站姿待命, 不重复初始化' }
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
| `text` | 积木显示文字，参数用 `[参数名]` 占位。**可以是数组** —— 会生成 `message0`/`message1`… 但**渲染多半还挤在一行**，见下 |
| `arguments` | 参数定义，键名对应 text 里的 `[参数名]` |
| `pycode` | **生成的 MicroPython 代码**，参数用 `[参数名]` 引用。**可以是数组 ⇒ 多行代码**，见下 |

#### `text` 写成数组 ≠ 竖排（2026-09-27 实测否掉了我的推论）

想知道怎么竖排，先看**数组到底做了什么**：它确实让每一行各自成为 `messageN` / `argsN`
（依据 `resources/app/web/lib.min.js` 生成块 JSON 的循环，`command` 块的 `branchCount`
是 `undefined`，子堆栈那支不执行），每行也照常走 `replace(/\[(.+?)]/g, convertPlaceholders)`
解析参数。**但生成的对 ≠ 画出来的样子。**

⚠ **KittenBlock 给每个扩展积木都硬写了 `inputsInline: true`**（同一个文件里
`blockJSON = { type: extendedOpcode, inputsInline: true, ... }`）。scratch-blocks 见到它
就把**值输入尽量挤在同一行** —— 于是 `message0`、`message1` 被并回一行。
2026-09-27 实测（`advMotion` 两行文案 + 7 个参数）：块上出来的是
`高级运动 步态 [▼] 速度 [2.5] 方向 [▼] 步长 [70] mm … 转弯率 [0]`，**一整行**，
两行文案首尾相接、参数全在、顺序也对 —— 就是没换行。

**能强制换行的只有语句输入槽（`SUBSTACK`）。** 这正是内置 `arduino` 的
`void setup(){}` / `void loop(){}` 能竖排的原因 —— 靠的是它俩中间那个 `{}`，不是数组。
所以 `conditional` + `branchCount` 是唯一能做出多行的路，**代价是行间会多出空的 C 形槽**
（`branchCount: 1` + 两行文案 → 渲染成「文案 / 空槽 / 文案」三行）。
要不要接受这个空槽，得看积木值不值得。

> 结论：**要让积木短，靠缩短文案，别指望换行。** [`advMotion`](#12-bpuppy-现有扩展积木清单)
> 就是横排单行的：7 槽那版实测约 1030px 宽（超出积木区，要拖动看），
> 2026-09-27 缩到 3 槽（步长/身体高度/抬脚高度）后就不显长了。

⚠ **一行只能有一个 `$$key`**：`maybeFormatMessage` 只在**整行**以 `$$` 开头时才查表，
并且把 `$$` 之后的**整串**当作一个 key（`.replace("$$", '')` 只去掉第一个）。
所以 `'$$a $$b'` 会去查 key `"a $$b"` —— 查不到。**几行就几个 key。**
（这条在数组写法下依然成立：每个数组元素是一个独立 key。）

#### `pycode` 写成数组 = 多行代码

```json5
pycode: ['if x > 1:', '    do_something()']
```

KittenBlock 生成代码时 `Array.isArray(pycode) ? pycode.join("\r\n") : pycode` ⇒ 数组元素用 `\r\n` 连接，
**元素里的缩进原样保留**，所以能写带缩进的语句块。

⚠ **别用分号把 `if` 挤在一行**：Python 里 `if c: a; b` 的 `b` 也会被算进 `if` 体
（`suite: simple_stmt (';' simple_stmt)*`），条件为假时 `b` 根本不执行。要多个语句就用数组写法。

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
  pycode: "bpuppy_motion.set_gait([GAIT])"   // ★ menu 参数不加引号, 见 5.7
}
```

### 5.7 pycode 参数替换规则

- `[参数名]` 会被实际值替换
- **普通 string 文本框参数**（无 `menu`）：**手动加引号**。`pycode: "bpuppy_motion.set_gait('[GAIT]')"` → 生成 `set_gait('go')`
- **menu 下拉参数**（有 `menu` 字段）：**不要手动加引号**！KittenBlock 自动加。`pycode: "bpuppy_motion.set_gait([GAIT])"` → 生成 `set_gait('go')`。若写了 `'[GAIT]'` 会生成 `set_gait(''go'')` 双层引号 → **SyntaxError**
- 数字参数直接替换：`pycode: 'bpuppy_motion.set_lift([LIFT])'`
- `value` 类型参数：直接替换变量引用，不加引号

### 5.8 积木定义示例

> 语法演示用。**当前实际生效的完整积木清单见 [§12](#12-bpuppy-现有扩展积木清单)** ——
> `kblock.json5` 里的 `text` 用的是 `$$xxx` 多语言 key（文案在 `bpuppy.l10n.json`），
> 下面为便于阅读直接写中文。

```json5
blocks: [
  "## 运动",
  {
    opcode: 'forward',
    blockType: 'command',
    text: '前进',
    pycode: "bpuppy_motion.set_turn(0); bpuppy_motion.set_params(_speed, _stride, _height); bpuppy_motion.set_gait('go')"
  },
  {
    opcode: 'backward',
    blockType: 'command',
    text: '后退',
    pycode: "bpuppy_motion.set_turn(0); bpuppy_motion.set_params(_speed, -abs(_stride), _height); bpuppy_motion.set_gait('go')"
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
    pycode: "bpuppy_motion.set_gait('stop')"
  },
  "---",
  "## 高级运动",
  {
    // ★ 2026-09-27 改版: 原 7 槽组合块拆开。这个块只管三个身体尺寸;
    //   步长/高度/抬脚的三个单参数块已删除。
    opcode: 'advMotion',
    blockType: 'command',
    text: '运动参数  步长 [STRIDE] mm  身体高度 [HEIGHT] mm  抬脚高度 [LIFT] mm',
    arguments: {
      STRIDE: { type: 'number', defaultValue: '70' },
      HEIGHT: { type: 'number', defaultValue: '70' },
      LIFT:   { type: 'number', defaultValue: '30' }
    },
    // pycode 用数组 (KittenBlock 用 join("\r\n")) —— 单行 `if c: a; b` 里 b 也会进 if 体
    pycode: [
      '_stride = abs([STRIDE])',
      '_height = [HEIGHT]',
      '_dir = 1 if bpuppy_motion.get_params()[1] >= 0 else -1',
      'bpuppy_motion.set_lift([LIFT])',
      'bpuppy_motion.set_params(_speed, _dir * _stride, _height)',
      'if abs(bpuppy_motion.get_params()[3] - [LIFT]) > 0.5:',
      '    bpuppy_motion.set_lift([LIFT])'
    ]
  },
  {
    opcode: 'setSpeed',
    blockType: 'command',
    text: '速度 [SPEED]  方向 [DIR]',
    arguments: {
      SPEED: { type: 'number', defaultValue: '2.5' },
      DIR:   { type: 'value', menu: 'dirMenu', defaultValue: '1' }
    },
    pycode: '_speed = [SPEED]; bpuppy_motion.set_params(_speed, ([DIR]) * abs(_stride), _height)'
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
    pycode: "bpuppy_motion.set_turn(0); bpuppy_motion.set_gait([GAIT])"   // ★ menu 参数不加引号, 见 5.7
  },
  "---",
  "## 姿态动作",
  // ⚠ 姿态是 poses.py 的**函数**, 不是步态 —— 绝不能写 bpuppy_motion.set_gait('stand')。
  //   set_gait 只认 8 个合法步态名 (stop/walk/walkfwd/walkbck/go/trot/trotfwd/trotbck),
  //   收到别的名字会 fail-safe 停车 + 打印 "⚠ 未知步态"。
  {
    opcode: 'stand', blockType: 'command', text: '站立',
    pycode: "poses.stand()"
  },
  {
    opcode: 'crouch', blockType: 'command', text: '蹲下',
    pycode: "poses.crouch()"
  },
  {
    opcode: 'sit', blockType: 'command', text: '坐下',
    pycode: "poses.sit()"
  },
  {
    opcode: 'playBow', blockType: 'command', text: '邀玩',
    pycode: "poses.play()"
  },
  {
    opcode: 'waveBlock', blockType: 'command', text: '挥手',
    pycode: "poses.wave()"
  }
],

menus: {
  gaitMenu: [
    { text: '自适应', value: 'go' },
    { text: '猫步',   value: 'walk' },
    { text: '小跑',   value: 'trot' },
    { text: '停止',   value: 'stop' }
    // ⚠ 不要放 'stand' / 'crouch' —— 它们不是合法步态名 (见上), 选中会 fail-safe 停车
  ]
}
```

---

## 6. 代码生成机制（★理解核心）

KittenBlock 把图形化积木翻译成 MicroPython，规则：

1. **程序入口 = 内置「当绿旗被点击」hat 块**。积木必须接在绿旗下面，代码面板才会生成代码。
2. **自定义 hat 块 ≠ 程序入口**。自定义 hat 块（如 blynk 的 `whenConnected`）会被当成**独立函数**生成（`@装饰器` + `def 函数():`）。**函数定义排在正文之前**，不在末尾（生成器 `finish()` = `definitions_.join()` + 正文，2026-09-16 复核 `lib.min.js`）—— 所以「重复执行」生成的顶格 `while True:` **挡不到它**。但它只被**定义**、不会被自动调用，所以**不要试图用自定义 hat 当入口**。
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
# 原厂已 init_all + 站姿待命 (不重复初始化)
# ===== 用户积木 =====
bpuppy_motion.set_turn(0); bpuppy_motion.set_params(_speed, _stride, _height); bpuppy_motion.set_gait('go')
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
3. 重新打包 zip（**4 个文件，一个都不能少**，尤其别漏 `bpuppy.l10n.json` —— 漏了三语文本会回退成 key）：
   ```powershell
   Compress-Archive -Path extension.json,kblock.json5,bpuppy.png,bpuppy.l10n.json `
                    -DestinationPath bpuppy-kittenblock.zip -Force
   ```
   （**进入文件夹选文件压缩，不要压文件夹本身**；zip 顶层必须是这 4 个文件平铺）
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

### 9.10 `pycode` 含中文 → 字符被抹掉（2026-09-26 实测）

**现象**：（当时是两个积木）「语音播放汪汪」和「语音播放嘤嘤」**点哪个都只汪汪**；看代码面板，
两个积木生成的**都是** `voice.play('')` —— 引号里是空的。

**原因**：这两个积木的 `pycode` 原本写的是中文字面量 `voice.play('汪汪')` / `voice.play('嘤嘤')`。
**KittenBlock 生成/下发代码时会把非 ASCII 字符抹掉**，到板上真的就是 `voice.play('')`；
而 `voice.play()` 对认不出的名字会**回退到汪汪**（`frozen/voice.py` 里 `m.get(name, SND_WANG)`
的兜底），于是两个积木都出汪汪。

**判据**（怎么确认是这个问题，而不是模块码值不对）：

1. 代码面板里字符串字面量变成 `''`（不是乱码，是**整个没了**）→ 就是被抹掉。
2. 板上 `import voice; voice.say(0x70, 2)` 能正常嘤嘤 → 排除"模块侧码值不对"。

**解法**：**积木的 `pycode` 一律写纯 ASCII**，不要把中文写进去。发声积木改用

```js
{ opcode: 'voiceSound', pycode: 'voice.say(*voice.SND_[SOUND])' }   // [SOUND] 下拉裸代入
```

用 `*voice.SND_xxx` 解包而不是写死 `voice.say(0x70, 0x01)`，是为了保住
「换声音只改 `frozen/voice.py` 的 `SND_*` 常量、积木不用动」这条性质。

> 2026-09-27 起两个发声积木**合并成一个 `voiceSound`（「狗叫 [SOUND]」）**，下拉 `soundMenu`
> 的值 `WANG`/`YING` 裸代入属性名 → `voice.say(*voice.SND_WANG)`。合并后这条教训照样适用：
> 只要 pycode 里出现中文，它就会被抹掉。

> ⚠ 显示文本不受影响 —— 积木上显示的中文来自 `bpuppy.l10n.json`，能正常渲染；
> 会被抹的只有**生成代码里的 `pycode`**。
> `libs` 头串（`kblock.json5` 顶部）里也有中文注释，同样会被抹，但那是注释，不影响执行。

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

## 11. 蓝牙（BLE）连接 — 编译互斥⚠

KittenBlock 可通过**蓝牙**连接 bPuppy，把 BLE 当作与串口等价的 MicroPython REPL 通道（在线执行、上传 main.py 都走 raw REPL）。

### 11.1 原理

- KittenBlock 用 **Web Bluetooth**（Electron `navigator.bluetooth`），要求硬件实现 **Nordic UART 透传服务**：
  - 服务 `6E400001-B5A3-F393-E0A9-E50E24DCCA9E`
  - `6E400002`（Nordic RX）= PC→设备 WRITE 通道
  - `6E400003`（Nordic TX）= 设备→PC NOTIFY 通道
- 广播须含服务 UUID `0x6E40`，否则 KittenBlock 扫不到
- 连接后走 MicroPython REPL：bPuppy 把 BLE 用 `os.dupterm()` 注册为第二 REPL 通道

### 11.2 ★编译互斥（必须遵守）

**两个蓝牙功能不要同时编译，编译其中一个时保证另一个不编译。**

用 **CMake 编译宏**二选一（`drivers/micropython.cmake`）：

| 宏 | 编译的蓝牙 | 用途 |
|------|-----------|------|
| `BPUPPY_BLE_KEBLOCK` | Nordic UART + dupterm REPL | KittenBlock 蓝牙编程 |
| `BPUPPY_BLE_HIWONDER` | FFE0（`ble_hiwonder.py` 已删除，无人解析报文） | Wonderbot App 遥控 —— **当前不可用** |

- 切换：注释/取消注释 `micropython.cmake` 里两行 `target_compile_definitions(usermod INTERFACE ...)`，重编译
- 同一固件**只能启用其一**，绝不共存编译
- 相关 C 代码：`ble_driver.c`（GATT 服务 + 广播）、`ble_stream.c`（流对象，仅 KEBLOCK 编译）、`ble_driver_mpy.c`（dupterm 注册）

### 11.3 固件改动点

| 文件 | 改动 |
|------|------|
| `drivers/micropython.cmake` | `BPUPPY_BLE_KEBLOCK` / `BPUPPY_BLE_HIWONDER` 编译宏（互斥） |
| `drivers/ble_driver.c` | 按宏隔离两套服务（Nordic / FFE0），广播 0x6E40 或 0xFFE0 + 扫描响应 128 位 Nordic UUID |
| `drivers/ble_stream.c` | BLE 流对象（read/write/ioctl），dupterm 桥接 |
| `drivers/ble_driver_mpy.c` | `start()` 时自动 `os.dupterm()`（C 层注册） |
| `drivers/ble_driver.h` | `ble_available()` / `ble_send_len()` 供流对象用 |
| `components/.../mphalport.c` | `mp_hal_stdin_rx_chr` 补 dupterm 输入分支（标准 MicroPython 行为，原移植漏了） |
| `frozen/main.py` | 上电 `bpuppy_ble.start()`（C 层自动注册 dupterm） |

> ⚠ `mphalport.c` 在 `components/mr9you__micropython-helper/`（MicroPython 移植层，因改它从 managed_components 移到 components 保留修改）。版本固定 1.22.1，改后 `build.sh` 重编译生效。蓝牙连接时建议不用 USB REPL 同时输入（会抢 REPL 输入）。

### 11.4 平台支持（实测）

| 平台 | 连接方式 | 实测 |
|------|---------|:--:|
| **安卓** | Chrome 打开 `kblock.kittenbot.cc` → Web Bluetooth | ✅ |
| **iPad** | **Bluefy 浏览器**打开 kblock.kittenbot.cc | ✅ |
| **PC 桌面版** | KittenBlock 桌面版 → 串口/蓝牙 | ✅ |
| **iPad Safari/Chrome** | 原生 Web Bluetooth | ❌ Apple 限制 |
| **iPad KittenBlock App** | App URL 导入自定义主板 | ❌ App 不支持 |

> **关键**：iOS 的 Safari/Chrome（WebKit）Web Bluetooth 被 Apple 限制，无法连接。**必须用 Bluefy**（App Store 的 Web BLE 浏览器）才能绕过。iPad KittenBlock App 也无法加载 URL 导入的自定义主板扩展。

### 11.5 WiFi 与蓝牙共存

- 上电 **WiFi 热点默认不开**（`main.py` 注释了 `camera_stream.start()`），把 RF 让给蓝牙
- 需要 WiFi 时手动：`import camera_stream; camera_stream.start()`
- ESP32-S3 硬件支持 WiFi+BLE 共存（时分），但上电全给 BLE 最稳

### 11.6 指令发送不全 — 根因与修复（2026-08 实测确认）⚠

**现象**：KittenBlock 长指令/初始化命令**截断在 20 字节处**（如 afterConnect 的 `_speed = 2.5; _strid`，`_stride`/`_height` 未赋值 → 后续引用 `NameError`）。

**排查链（为什么单修 MTU 不够）**：
1. 固件已主动 `ble_gattc_exchange_mtu()` → MTU 256，日志确认 `MTU 交换完成: 256`，**仍截断** → 排除 ATT 层 20 字节载荷限制。
2. 查 KittenBlock 源码：JS 库**硬编码 `MAX_CHUNK_SIZE=20`** 分包（Nordic UART 经典实现，**与链路 MTU 无关**）→ 长指令**仍是多个 20 字节独立 ATT 写**，MTU 修复管不到它。
3. 串口实测 `print(_stride)` → `NameError: name '_stride' isn't defined` → 确认**接收路径真丢包**（非仅 echo 回显丢失）。

**根因（真正的丢包点 = echo 洪峰饿死 mbuf 池）**：
- MicroPython REPL 普通模式逐字符 echo，`ble_stream.c` 旧实现**每 echo 一个字符就发一个 NOTIFY**，每字符分配一块 NimBLE **MSYS_1 mbuf（池仅 12 块）**。
- chunk1（20 字节）的 echo 洪峰瞬间吃光小包池 → **chunk2 到达时无 mbuf 可分配 → 在 NimBLE host 层被丢** → chunk3（`\n`）在池恢复后幸存。
- 现象恰为「chunk1 + `\n` 到了，chunk2 丢了」→ 命令在 20 字节处截断。

**修复（`ble_stream.c` echo 批量打包 = 核心）**：
- 把逐字符通知改为**攒到一整行（遇 `\n`）或 30ms 超时再发一个 NOTIFY**（MTU 256 单包可装 253 字节）。
- 一条命令的 echo 从几十个通知 → **1 个**；mbuf 分配从每字符 1 块 → 每行 1 块，洪峰消失，后续分包不再被饿死。
- 超时兜底挂 `ble_stream_read`/`ioctl`（REPL 空闲时高频轮询），覆盖无换行输出（如 `>>> ` 提示符）。KittenBlock 是**命令级流控**（等 `>>>` 再发下一条，非 chunk 级），批量 echo 不影响时序。

**验证（实测通过）**：
- 临时 `RXW len=...` 诊断日志（验证后已删）确认 `_speed = 2.5; _strid` 的 chunk1（20B）+ chunk2 `e = 70; _height = 70`（20B）+ 换行（2B）**全部到达 GATT 写回调**。
- 串口 `print(_stride)` → **`70`**（修复前 `NameError`）。

**配套改动**：

| 文件 | 改动 |
|------|------|
| `drivers/ble_stream.c` | echo 批量打包（`tx_buf` + 遇 `\n`/超时 flush）— **核心修复** |
| `drivers/ble_driver.c` | 连接成功主动 `ble_gattc_exchange_mtu()`（MTU→256，单包不再跨包）；接收缓冲 512→4096；写满丢新字节而非覆盖未读数据 |
| `drivers/ble_driver.h` | `ble_recv_command` 注释修正（最多取 max_len-1 字节） |

**为什么换 MCU 前能跑、现在不能**：与 MCU 无关。git 历史里 `_speed = 2.5; _stride = 70; _height = 70` 这条 40 字节 afterConnect 是 2026-08 才加进 `extension.json`；此前命令都 ≤21 字节，**从不触发 20 字节分包边界**。20 字节分包隐患自 BLE 引入（NimBLE）起就存在，只是这次命令长度 + echo 并发时序共同触发了它。

---

## 12. bPuppy 现有扩展积木清单

| 分类 | 积木 | pycode 生成 |
|------|------|------------|
| 运动 | 前进 / 后退 / 左转 / 右转 / 停止 | `set_turn(0); set_params(_speed, ±_stride, _height); set_gait('go')` / `set_gait('stop')` |
| 高级运动 | **运动参数**（步长 + 身体高度 + 抬脚高度，3 个槽，**横排单行**） | 见下方代码块 |
| 高级运动 | 速度 [SPEED] 方向 [DIR] | `_speed = [SPEED]; set_params(_speed, ([DIR]) * abs(_stride), _height)` ⚠ 2026-09-27 由原「速度设为」并入方向下拉 |
| 高级运动 | 切换步态 [GAIT] / 转弯率设为 [TURN] | `set_turn(0); set_gait([GAIT])` / `set_turn([TURN])` |
| 高级运动 | **身体姿态**（俯仰 + 滚转） | `set_body_pose([ROLL], [PITCH], 0)` ⚠ 函数签名是 `(roll, pitch, yaw)`，块上文字却是"先俯仰后滚转" ⇒ pycode 里 `[ROLL]` 必须在前，写反不报错、只是两者对调 |
| 高级运动 | **重心偏移 [OFFSET]** mm | `set_center([OFFSET])` ⚠ **会写 NVS**（`motion_task.cpp:983` 调 `motion_save_geometry`），值跨重启保留；上限 ±`L1/2`（默认 20mm） |
| 姿态 | 站立 / 蹲下 / 坐下 / 邀玩 / 挥手 | `poses.stand()/crouch()/sit()/play()/wave()` |
| 舵机编辑 | 舵机设为 / 过渡速度 / 执行姿态 / 舵机角度 | `poses.set_servo/set_step/commit` + `bpuppy_servo.get_angle` |
| 动作 | 摆动 / 等待 | `poses.oscillate(...)` / `sleep(...)` |
| 传感器 | 初始化 IMU / 横滚角 / 俯仰角 / 偏航角 | `bpuppy_imu.init` / `read_angles()[0/1/2]` |
| 语音 | 当收到 [指令]（1 个 hat，下拉选指令，15 选项 = 14 指令 + 声音角度） | `def voiceWhen<value>():` 独立函数（定义在正文前） + voice.py 按名注册回调 |
| 语音 | (声音角度)（reporter，读变量） | `voice.SoundAngle`（0–180 度；`-1` = 还没收到过） |
| 语音 | 狗叫 [汪汪/嘤嘤]（1 个积木，下拉选声音） | `voice.say(*voice.SND_[SOUND])`（码值真源 `frozen/voice.py` 的 `SND_WANG`/`SND_YING`；⚠ pycode 必须纯 ASCII，见 §9.10） |
| 语音 | 播报数字 [NUM]（滑块 0–100） | `voice.say_num([NUM])` → 帧 `AA 55 72 <数字> 55 AA`（越界钳位、非数字忽略，均在固件侧） |

「运动参数」生成的代码（`pycode` 是数组，KittenBlock 用 `join("\r\n")` 逐行拼）：

```python
_stride = abs([STRIDE])                    # 存正数; 「后退」块用 -abs(_stride)
_height = [HEIGHT]
_dir = 1 if bpuppy_motion.get_params()[1] >= 0 else -1   # 方向 = 沿用板上当前步长的符号
bpuppy_motion.set_lift([LIFT])
bpuppy_motion.set_params(_speed, _dir * _stride, _height)
if abs(bpuppy_motion.get_params()[3] - [LIFT]) > 0.5:
    bpuppy_motion.set_lift([LIFT])         # 顺序耦合重试, 见下
```

三个关键点：

1. **必须回写 `_stride` / `_height` 模块全局** —— 「前进」「后退」「速度 … 方向」等块全靠它们。
   不回写的话，用完「运动参数」再拖一个「前进」，参数会被旧值覆盖回去。
2. **方向不在这个块里**：`set_params` 的 stride 带符号，而 `_stride` 全局恒存正数 ⇒
   前后由 `get_params()[1]` 的符号决定（`>= 0` 即前进）。这样狗在「后退」中改步长**不会掉头**。
3. **顺序耦合**（`docs/error.md` §2.2 #7）：`set_params` 用**当前** `lift` 校验，`set_lift` 用**当前**
   `stride/height` 校验。一个块发多个 setter 会踩到"谁先谁后决定成败"。
   先 lift → params → **读回 lift 不对就补一次**，可覆盖全部情况，不需要固件侧原子 API。

> **2026-09-27 改版**：原 7 槽的「高级运动」拆成「运动参数」+ 单参数块。
> 「步长设为」「身体高度设为」「抬脚高度设为」**三个块已删除**（并入「运动参数」）；
> 「速度设为」并入方向下拉变成「速度 [SPEED] 方向 [DIR]」；
> 「切换步态」「转弯率设为」保持不变。分类键仍是 `cat_gait`，只是显示名改成了**高级运动**。
> 拆的理由：7 个槽挤一行约 1030px，且块内改步态会**打断**当前步态 —— 现在改尺寸不动步态。

底层 API（固件 C 模块，MicroPython 可调）：
- `bpuppy_motion.set_params(speed, stride, height)` — speed 0~10, stride 正前负后, height mm
- `bpuppy_motion.set_gait('stop'/'walk'/'walkfwd'/'walkbck'/'go'/'trot'/'trotfwd'/'trotbck')` — 只有这 8 个是合法步态名。⚠ 未知名字**不会报错**，会走 fail-safe 停车 + 打印 `⚠ 未知步态`
- ⚠ **`stand`/`sit`/`crouch`/`play`/`wave` 不是步态**，是 `poses.py` 里的**姿态函数**（`poses.stand()` 等），别混用
- `bpuppy_motion.set_turn(-0.8~0.8)`
- `bpuppy_motion.set_lift(mm)`
- `bpuppy_servo.init_all()` / `bpuppy_servo.load_cal()` / `bpuppy_servo.set_angle(ch, deg)`

---

## 13. 事件积木（hat）与 micropy 触发机制（2026-08-19 实测源码确认）

自定义 hat 积木在**离线 micropy 代码生成**里的行为（源码 `offlineCodeGen` + `provideFunction_`）：

1. **生成独立函数（定义在正文之前）**：`blockType:'hat'` 积木把 `pycode` 逐行拆分——以 `@`/`#`/`def` 开头的行进函数头区，其余行进函数体；整段经 `provideFunction_` 落进生成器的 `definitions_`。生成器的 `finish()` 是 `definitions_.join("\n\n") + "\n\n\n" + code`（2026-09-16 复核 `lib.min.js`）→ **函数定义排在正文前面，不是末尾**。`pycode:['def voiceWhen[VOICE]()']` 生成 `def voiceWhenFwd():` + 用户积木体。**函数名 = pycode 里 `[VOICE]` 占位符被参数值替换后的结果**（无占位符时 `provideFunction_` 不改名）。函数去重 key = `opcode_参数值`（如 `voiceWhen_Fwd`），所以多个实例选不同指令会生成各自的函数、互不 dedupe 冲突。
2. **没人自动调用它**：离线代码生成只**定义**函数，不生成调用。asr@kai 的 `kai_whenHeard` 靠在线 Scratch 事件，离线只是空函数。
3. **真正的事件范式 = meowbit/futureboard 的「注册 + 调度」**（官方 board 扩展）：
   ```json5
   {
     opcode: 'buttonEvt', blockType: 'hat',
     micropy: { instance: "sensor.startSchedule()" },   // 一次性启动调度线程 (definitions_ 顶部注入)
     pycode: ["sensor.btnTrig['[BTN]'] = on_[BTN]_pressed",  // 注册行 → 会被放进函数体末尾!
              "def on_[BTN]_pressed()"],                      // 函数头
   }
   ```
   `micropy.instance` 注入到生成文件**顶部**（`definitions_` 非 import/def 区，imports 之后），`pycode` 数组 join 后按行拆：注册行进函数体末尾、def 行进函数头。调度线程扫描 globals 按命名约定（`on_*_pressed`）调用一次 → 触发自注册 → 轮询分发。

**bPuppy 语音事件积木的实现**（避免「调用一次 = 用户体执行一次」的副作用）：
- hat 积木 `pycode` 只有 `['def voiceWhen[VOICE]()']`（VOICE 是 `type:'value'`+`voiceMenu` 下拉参数），**不写注册行**。下拉值 `Fwd` 裸代入 → `def voiceWhenFwd():`；**必须 `type:'value'` 不能 `type:'string'`**——string 会被 KittenBlock 自动加引号 → `def voiceWhen'Fwd'():` 语法错误（`replaceArgs` 加引号只发生在 `type === STRING`）。
- voice.py 后台线程 `_scan_events()` 扫**主脚本全局 dict**（frozen main.py `exec(_user_code)` / REPL 的作用域），按 `_EVT_FUNCS` 表（`voiceWhenX` → 命令码）把函数**直接注册**为回调，**不调用** → 用户体不在开机时误执行。
- ⚠ **全局 dict 来源**（实测 2026-08-19）：本固件 MicroPython **`sys.modules['__main__']` 为 `None`**，不能用它找函数。必须在 `frozen/main.py` 与 KittenBlock afterConnect / libs 里调 `voice.set_main_globals(globals())` 显式传入（主脚本与 REPL 共享同一 dict，`exec` 新定义的 `voiceWhen*` 会进入该 dict，后台扫描即可看到）。
- 收到下行指令 → `_dispatch` 调注册的回调。**固件无任何内置动作**（2026-08-19 起已移除 `_ACTIONS`/`auto_move`），命令来了要么触发用户事件函数、要么什么都不做，动作全由用户编程。

> ⚠ 下拉 **value 必须与 `_EVT_FUNCS` 后缀一致**（`Fwd` → `def voiceWhenFwd()`），`_EVT_FUNCS` 才能映射到命令码；value 拼错或与后缀不匹配会注册不上。加新指令不用新增积木，只需在 `voiceMenu` 加一项 + l10n 加显示文本。

📘 语音「事件」系统的全链路交接文档（含改法 Recipe 与踩坑清单）：[docs/README.md 语音「事件」系统节](../docs/README.md)。

## 14. 参考

- KittenBlock 插件开发指南 01/02：积木定义、串口通信基础
- 本目录 `extension.json`、`kblock.json5`：可直接复制修改
- KittenBlock 自带参考扩展：`extensions/otto@kext/`（micropy 硬件扩展）、`extensions/mpython/`、`extensions/futureboard-lite-mpy/`
