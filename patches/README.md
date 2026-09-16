# patches/ — 对第三方组件的本地改动

## 为什么需要这个目录

`components/` 和 `managed_components/` 里的东西**不是本仓库的代码**，是组件管理器
按 `main/idf_component.yml` 从网上**下载**的依赖（`components/` 里就裹着一整棵 57MB
的 MicroPython 源码树）。这两个目录都被 `.gitignore` 忽略——就像 `node_modules`。

问题是：**bPuppy 对其中一个依赖做了必须的改动**。改动只存在于当时那台电脑的硬盘上，
`git clone` 到新机器、或者依赖被重下/升版，改动就没了，而且**不会有任何报错**，
只是某个功能静默失效。

所以这些改动以补丁形式存进仓库，`build.sh` 每次编译前自动打上。

## 现有补丁

| 补丁 | 目标文件 | 作用 |
|------|---------|------|
| `0001-disable-tiniusb-release-gpio19-20.patch` | `components/mr9you__micropython-helper/mpy_startup.c` | 注释掉 `usb_init()`，把 GPIO19/20 让给语音模块的 UART2 |

丢掉 0001 的后果：`usb_init()` 恢复 → USB-OTG PHY 接管 GPIO19/20 → **CI-33T 语音模块的
UART2 TX 发不出波形**，`machine.Pin(19)` 驱动无效。现象在硬件层，很难反查到"少了个补丁"。
代价是 USB-CDC 虚拟串口不可用——这不是可选项，想恢复就得放弃 UART2。

## 怎么应用

不用手动做，`bash build.sh` 会自动处理，且是幂等的：

- 反向能打上 → 说明已在目标状态 → 打印"已应用, 跳过"
- 正向能打上 → 打印"已应用补丁"
- 两个都不行 → 报错退出（多半是依赖升版、上下文对不上）

## 怎么新增一个补丁

```bash
# 1. 先改文件本身
vim components/<组件>/<文件>

# 2. 取原始文件作对照 (上游仓库按组件的 idf_component.yml 里 repository 字段找)
curl -sfL -o /tmp/up.c https://raw.githubusercontent.com/<owner>/<repo>/master/<文件>

# 3. 生成补丁 (a/ b/ 前缀, 从仓库根目录算起的相对路径)
mkdir -p /tmp/pt/a/<目录> /tmp/pt/b/<目录>
cp /tmp/up.c            /tmp/pt/a/<目录>/<文件>
cp components/<目录>/<文件> /tmp/pt/b/<目录>/<文件>
(cd /tmp/pt && diff -u a/<目录>/<文件> b/<目录>/<文件>) > patches/000N-<描述>.patch
```

**验证补丁往返（必做）**：

```bash
# 从上游原文出发打一遍，结果必须与你手改的文件逐字节一致
cp /tmp/up.c components/<目录>/<文件>
git -c core.autocrlf=false apply patches/000N-<描述>.patch
cmp components/<目录>/<文件> <你手改的那份备份>
```

⚠ **必须带 `-c core.autocrlf=false`**。本仓库 `core.autocrlf=true`，直接 `git apply`
会把整个文件转成 CRLF——C 编译不受影响，但补丁之后就无法反向校验，幂等判断也会失效。

## 什么时候该警惕

- 升级 `main/idf_component.yml` 里的依赖版本 → 补丁上下文可能对不上 → `build.sh` 会报错退出
- 换电脑 / 重装 / `rm -rf components/` → 正常场景，`build.sh` 会自动补上
- 往 `components/` 里手改了别的文件 → 记得也做成补丁，否则同样会丢
