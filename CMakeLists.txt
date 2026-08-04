cmake_minimum_required(VERSION 3.16)

# =============================================================
# bPuppy 机器狗 — ESP32-S3 MicroPython + C 混合开发
# 遵循 micropython-helper 官方文档
#
# 1. include mpy.cmake 设置 MicroPython 构建变量
# 2. include project.cmake + project() 标准 IDF 流程
# =============================================================

# 定位 micropython-helper
# 原在 managed_components/, 因改了 mphalport.c (dupterm 输入) 移到 components/ 保留修改
if(NOT MICROPY_HELPER_DIR)
    set(MICROPY_HELPER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/components/mr9you__micropython-helper)
endif()

if(EXISTS ${MICROPY_HELPER_DIR})
    # 模式 A: micropython-helper 已就绪
    set(ENV{IDF_TARGET} esp32s3)
    set(MICROPY_BOARD ESP32_GENERIC_S3)
    set(MICROPY_BOARD_VARIANT SPIRAM_OCT)   # Octal PSRAM + 240MHz

    # 注册用户自定义 C 模块（MicroPython 要求变量名 USER_C_MODULES）
    set(USER_C_MODULES ${CMAKE_CURRENT_SOURCE_DIR}/drivers/micropython.cmake)

    # Frozen Python 模块
    set(MICROPY_FROZEN_MANIFEST ${CMAKE_CURRENT_SOURCE_DIR}/frozen/manifest.py)

    # 确保 QSTR 预处理能找到板级头文件 mpconfigboard.h
    set(MICROPY_CPP_INC_EXTRA ${MICROPY_HELPER_DIR}/micropython/ports/esp32/boards/${MICROPY_BOARD})

    # mpy.cmake 设置 MicroPython 构建变量
    include(${MICROPY_HELPER_DIR}/mpy.cmake)

    # bPuppy Flash/PSRAM 覆盖 — 必须在 board sdkconfig 之后加载
    list(APPEND SDKCONFIG_DEFAULTS ${CMAKE_CURRENT_SOURCE_DIR}/sdkconfig.bpuppy)

    # 标准 ESP-IDF 项目初始化
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(micropython_bpuppy)
    # esp_tinyusb v1.0.3 缺少 CFG_TUD_CDC_EP_BUFSIZE 定义
    idf_build_set_property(COMPILE_OPTIONS "-DCFG_TUD_CDC_EP_BUFSIZE=256" APPEND)

else()
    # 模式 B: 首次 set-target（组件尚未下载）
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(bPuppy_esp32s3)
endif()
