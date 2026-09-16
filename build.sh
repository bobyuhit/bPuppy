#!/bin/bash
# ============================================================
# bPuppy 固件编译脚本（在宿主机 Git Bash 中运行）
#
# 首次使用:
#   mkdir -p ~/.ccache_bpuppy     # 建 ccache 目录（只需一次）
#   bash build.sh                 # 编译
#
# 日常增量编译:
#   bash build.sh
#
# 改了 CMakeLists.txt / sdkconfig / idf_component.yml:
#   rm -rf build && bash build.sh
#
# 多台电脑:
#   每台电脑各自有 ~/.ccache_bpuppy，互不影响
#   build/ 目录不提交 git，每台电脑独立
# ============================================================
set -e

PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
CCACHE_DIR="$HOME/.ccache_bpuppy"

mkdir -p "$CCACHE_DIR"

echo "=== bPuppy 编译 ==="
echo "项目: $PROJECT_DIR"
echo "缓存: $CCACHE_DIR"
echo ""

# ---- 应用 patches/ 下的本地补丁 ----
# components/ 是组件管理器自动下载的第三方依赖 (见 main/idf_component.yml), 整个目录
# 被 .gitignore 忽略 —— 所以对它做的必要改动**不会进 git**, 换台电脑重下依赖就丢了。
# 这些改动以补丁形式存在 patches/ 里, 每次编译前自动打上。
#
# 必须带 -c core.autocrlf=false: 本仓库 core.autocrlf=true, 直接 git apply 会把
# 打补丁的文件整体转成 CRLF (C 编译不受影响, 但补丁之后就无法反向校验了)。
for p in "$PROJECT_DIR"/patches/*.patch; do
  [ -e "$p" ] || continue
  _name="$(basename "$p")"
  if git -C "$PROJECT_DIR" -c core.autocrlf=false apply --check --reverse "$p" 2>/dev/null; then
    echo "补丁已应用, 跳过: $_name"      # 反向能打 = 已在目标状态, 幂等
  elif git -C "$PROJECT_DIR" -c core.autocrlf=false apply "$p"; then
    echo "已应用补丁: $_name"
  else
    echo "!! 补丁应用失败: $_name" >&2
    echo "   多半是 components/ 里的文件版本变了 (依赖升版), 上下文对不上。" >&2
    echo "   处理: 手动改对应文件, 再按 patches/README.md 重新生成补丁。" >&2
    exit 1
  fi
done
echo ""

# MSYS_NO_PATHCONV=1 阻止 Git Bash 把 /d/xxx 转成 C:/Program Files/Git/xxx
MSYS_NO_PATHCONV=1 docker run --rm \
  -v "${PROJECT_DIR}:/workspace" \
  -v "${CCACHE_DIR}:/root/.ccache" \
  -e CCACHE_DIR=/root/.ccache \
  espressif/idf:v5.1.2 \
  bash -c "source /opt/esp/idf/export.sh && cd /workspace && idf.py build"

echo ""
echo "=== 编译完成 ==="
echo "固件: build/micropython_bpuppy.bin"
echo ""
echo "烧录 (Windows PowerShell):"
echo "  cd $(cygpath -w "$PROJECT_DIR" 2>/dev/null || echo "$PROJECT_DIR")\\build"
echo "  esptool --chip esp32s3 --port COM14 --baud 115200 write-flash 0x0 bootloader/bootloader.bin 0x8000 partition_table/partition-table.bin 0x10000 micropython_bpuppy.bin"
