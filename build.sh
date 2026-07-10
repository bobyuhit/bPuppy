#!/bin/bash
# =============================================================
# bPuppy ESP32-S3 MicroPython 构建脚本（容器内使用）
#
# 首次构建: ./build.sh first
# 日常构建: ./build.sh
# =============================================================
set -e

# 激活 ESP-IDF 环境（官方镜像 /opt/esp/idf, 自定义镜像 /opt/esp/idf-v5.1.2）
if [ -f /opt/esp/idf-v5.1.2/export.sh ]; then
    export IDF_PATH=/opt/esp/idf-v5.1.2
    source /opt/esp/idf-v5.1.2/export.sh > /dev/null 2>&1
elif [ -f /opt/esp/idf/export.sh ]; then
    export IDF_PATH=/opt/esp/idf
    source /opt/esp/idf/export.sh > /dev/null 2>&1
fi

echo "ESP-IDF: $(idf.py --version)"

MODE="${1:-build}"

if [ "$MODE" = "first" ]; then
    echo "============================================================"
    echo "  首次构建: reconfigure → 验证组件 → build"
    echo "============================================================"
    echo ""

    # ① 让组件管理器下载 micropython-helper
    echo "[1/3] 下载依赖组件..."
    rm -rf build managed_components dependencies.lock
    idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 \
           -D MICROPY_BOARD_VARIANT=SPIRAM_OCT \
           reconfigure

    # ② 确认组件已下载
    echo ""
    echo "[2/3] 验证组件..."
    if [ -d "managed_components/mr9you__micropython-helper" ]; then
        echo "  ✓ micropython-helper 已就绪"
        ls managed_components/mr9you__micropython-helper/mpy.cmake > /dev/null && \
            echo "  ✓ mpy.cmake 存在"
    else
        echo "  ✗ managed_components/mr9you__micropython-helper/ 未找到!"
        echo "  请检查 main/idf_component.yml 和网络连接"
        exit 1
    fi

    # ③ 构建
    echo ""
    echo "[3/3] 编译固件..."
fi

idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 \
       -D MICROPY_BOARD_VARIANT=SPIRAM_OCT \
       build

echo ""
echo "============================================================"
echo " BUILD SUCCESS!"
echo " Firmware: build/micropython_bpuppy.bin"
echo "============================================================"
