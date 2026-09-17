# bTool — ESP32 图形化烧写 / 文件管理 / REPL 工具

一个单文件的 Tkinter 小工具，把平常烧板子要敲的几条命令收进界面：选串口 → 选固件 → 点烧写。
另外带上设备文件管理（替代 ViperIDE 的拖文件）和 REPL 终端。

适用于 ESP32 / ESP32-S3 / ESP32-C3 / ESP32-C6。

---

## 安装

```bash
pip install pyserial esptool
```

> esptool **必须 5.x**（本工具按 5.x 的 API 写的：`esptool.cmds.detect_chip` /
> `erase_flash` / `write_flash`，以及 `esptool.logger.EsptoolLogger` 进度钩子）。
> 4.x 的 API 完全不同，会直接报错。

## 运行

```bash
python btool.py
```

---

## 功能说明

### 1. 烧写

| 操作 | 说明 |
|------|------|
| **添加固件…** | 可多选。地址会**按文件名自动猜**（见下表），双击某行可手改 |
| **双击地址列** | 手动改烧写地址（十六进制，如 `0x10000`）|
| **先擦除整片 Flash** | 默认**不勾**。勾上等同 `erase_flash`，见下方警告 |
| **擦除并烧写** | 开始。带进度条，下面是 esptool 的原始日志 |

**地址自动猜测规则：**

| 文件名包含 | 地址 |
|-----------|------|
| `bootloader` | `0x0` |
| `partition-table` / `partition_table` | `0x8000` |
| `boot_app0` | `0xE000` |
| 其它 | `0x10000` |

> ⚠ **擦除会清空整片 Flash —— 包括 NVS。**
> 对 bPuppy 来说就是**舵机三点标定、几何参数、电池标定全部丢失**。
> 只在改了分区表、或板子状态不明时才需要擦；平常更新程序**别勾**，直接烧。

**烧写时串口会被让出来**：esptool 要自己开串口，所以工具会先关掉 REPL 连接，
烧完自动重开（日志里会打 `[已释放串口供烧写使用]` / `[串口已重新打开]`）。

### 2. 文件管理

左边本地、右边设备 VFS。

| 按钮 | 作用 |
|------|------|
| **上传 →** | 本地选中 → 设备当前目录 |
| **← 下载** | 设备选中 → 本地当前目录 |
| **重命名 / 删除** | 对设备文件操作 |
| **刷新** | 重读设备目录 |

- 双击本地目录项进入，双击 `../` 回上级
- 设备文件走 **raw REPL**，传输用 base64 分块（避开引号转义和行缓冲限制）
- 可以传 `.py`：把脚本丢到板子上，复位即可运行

> 板上文件必须**纯 ASCII** 才保险 —— 蓝牙/串口传含中文的内容可能丢字节。
> 仓库里的 `mpy_modules/*.py` 就是按这个规则写的。

### 3. REPL 终端

- 输入命令回车发送，下面是返回结果
- **Ctrl+C** 按钮 = 发 `\x03`，中断正在跑的程序（例如 `while True` 卡住了）
- **切到 Raw / 切回 REPL** = `Ctrl-A` / `Ctrl-B`，文件操作用的是 raw 模式
- 连接时**自动**打断并拿提示符，不主动复位板子（不会打断正在跑的程序）

---

## 打包成单文件 exe

```bash
pip install pyinstaller

# Windows (Git Bash 里必须带 MSYS_NO_PATHCONV=1, 否则 --add-data 的路径会被改写)
MSYS_NO_PATHCONV=1 python -m PyInstaller --onefile --windowed --name bTool \
  --collect-submodules esptool \
  --add-data "$(python -c 'import esptool,os;print(os.path.dirname(os.path.dirname(esptool.__file__)))')/esptool/targets/stub_flasher;esptool/targets/stub_flasher" \
  --exclude-module IPython \
  --exclude-module matplotlib \
  --exclude-module PyQt5 \
  --exclude-module PyQt6 \
  --exclude-module PySide2 \
  --exclude-module PySide6 \
  --noconfirm --clean \
  btool.py
```

产物：`dist/bTool.exe`（单文件，双击即用，无需装 Python）。

**PowerShell 版本：**

```powershell
pyinstaller --onefile --windowed --name bTool `
  --collect-submodules esptool `
  --add-data "$(python -c "import esptool,os;print(os.path.dirname(os.path.dirname(esptool.__file__)))")\esptool\targets\stub_flasher;esptool\targets\stub_flasher" `
  --exclude-module IPython --exclude-module matplotlib `
  --exclude-module PyQt5 --exclude-module PyQt6 --exclude-module PySide2 --exclude-module PySide6 `
  --noconfirm --clean `
  btool.py
```

### ⚠ 每个参数为什么不能省

**`--add-data ... stub_flasher`**

esptool 的 stub flasher 是一批 **JSON 数据文件**（`esptool/targets/stub_flasher/{1,2}/*.json`，
共 27 个），代码里按 `__file__` 的相对路径找：

```python
STUB_DIR = os.path.join(os.path.dirname(__file__), "targets", "stub_flasher")
```

PyInstaller **不会自动收集非 .py 文件**。少了它，exe **能启动、能连芯片，但一开始烧写就失败** ——
典型的"本机跑得好好的，打包出来就废"。所以必须把它按原路径结构塞进去。

**`--collect-submodules esptool`**

esptool 的芯片定义（`esptool/targets/esp32s3.py` 等 15 个）是**动态导入**的，
静态分析扫不到，不加会漏。

**`--exclude-module IPython / matplotlib / PyQt5 …`**

这条最容易被忽略，而且不加的话 exe 会**从几十 MB 膨胀到几百 MB**，打包时间也翻好几倍。

原因是一条很隐蔽的依赖链：

```
esptool/__init__.py → import rich_click as click     ← 无条件, 排不掉
        ↓
rich → (可选集成) pygments → (可选集成) IPython → matplotlib → PyQt5
```

`rich` 是 esptool 的**硬依赖**（`__init__.py:46` 直接 import，没有 try/except），所以不能排。
但从 rich 往下的那串全是 PyInstaller 的 hook 顺着**可选集成**摸出来的，运行时根本用不到 ——
本工具只用 esptool 的 `cmds` 和 `logger`，从不碰 CLI，也就用不到 rich 的渲染，
更用不到 IPython/matplotlib/Qt。所以把它们排掉。

**`--windowed`**

不带控制台黑框。调试时**去掉这个参数**，esptool 的报错就能直接在终端看到。

### 打包后自检

```bash
# 1) 确认 stub 数据真的进去了 (应该数出 27 个 json)
find build/bTool -path "*stub_flasher*" -name "*.json" | wc -l

# 2) 直接跑起来, 用「添加固件 → 烧写」实烧一次
./dist/bTool.exe
```

第 2 步别省 —— stub 数据缺失是**只在真烧写时才暴露**的问题，光启动界面看不出来。

---

## 常见问题

| 现象 | 原因 / 处理 |
|------|------------|
| 串口列表为空 | 板子没插好、驱动没装（CH343/CP210x）、或被别的程序占着 |
| 打开串口报"拒绝访问" | 别的程序占着（ViperIDE / 串口助手 / 另一个 bTool）。关掉再试 |
| 烧写时 `Could not open port` | 同上 —— 烧写要独占串口 |
| 连上后没看到 `>>>` | 板子在跑 `while True` 程序，点 **Ctrl+C** 打断 |
| 进 raw REPL 失败 | bPuppy 的语音模块 UART 回显会插进握手，工具已自动重试 5 次；仍失败就多点几次「切到 Raw」 |
| 上传后文件内容不对 | 检查文件是不是纯 ASCII；中文经串口/BLE 传输可能丢字节 |
| 打包的 exe 烧写失败但 python 跑正常 | 十有八九是 `--add-data stub_flasher` 漏了 |

---

## 实现要点（给接手的人）

**串口互斥**：三种用途争同一个串口 —— REPL、raw REPL、烧写。
前两个其实**是同一条连接的两种模式**（`Ctrl-A`/`Ctrl-B` 切换，不用重开）；
只有烧写要真正独占，因为 esptool 自己开串口。所以：

```
烧写前: 关掉我们的连接 → esptool 独占 → 烧完重开
文件操作: 切到 raw REPL( Ctrl-A ), 不重开串口
```

**进度条怎么来的**：esptool 5.x 把日志收在一个单例 `EsptoolLogger` 里，
`set_logger()` 可以换掉它（实现上直接改单例的 `__class__`）。
子类化它、覆写 `progress_bar`，就把进度接进 Tkinter 了。

**raw REPL 协议**：
- 发 `\x01` 进 raw 模式 → 回 `raw REPL; CTRL-B to exit`
- 发 `代码 + \x04` → 回 `OK<stdout>\x04<stderr>\x04>`
- 解析时**从第一个 `OK` 开始**，避开回显和杂讯

**文件传输**：base64 分块。每块长度必须是 4 的倍数，否则单块无法独立解码。
读用 hex（2 倍体积但绝对无歧义），写用 base64。

**线程模型**：所有串口 I/O 在后台线程，UI 更新走 `queue` + `after(60, ...)` 轮询。
Tkinter 不是线程安全的，后台线程绝不直接碰控件。
