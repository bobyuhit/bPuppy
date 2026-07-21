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
