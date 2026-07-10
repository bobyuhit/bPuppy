@echo off
REM =============================================================
REM bPuppy ESP32-S3 MicroPython — 首次环境搭建 + 构建
REM 用法: 在 ESP-IDF 命令提示符中运行此脚本
REM =============================================================

echo ============================================================
echo   bPuppy ESP32-S3 MicroPython Build Setup
echo ============================================================
echo.

REM 设置目标芯片
echo [1/3] Setting target to ESP32-S3...
idf.py set-target esp32s3
if %errorlevel% neq 0 goto :error

REM 下载依赖组件 (micropython-helper, etc.)
echo.
echo [2/3] Fetching dependencies...
idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 -D MICROPY_BOARD_VARIANT=SPIRAM_OCT reconfigure
if %errorlevel% neq 0 goto :error

REM 构建固件
echo.
echo [3/3] Building MicroPython firmware...
idf.py -D MICROPY_BOARD=ESP32_GENERIC_S3 -D MICROPY_BOARD_VARIANT=SPIRAM_OCT build
if %errorlevel% neq 0 goto :error

echo.
echo ============================================================
echo   BUILD SUCCESS!
echo   Firmware: build\micropython_bpuppy.bin
echo ============================================================
echo.
echo Flash command (USB-JTAG):
echo   esptool --chip esp32s3 --port COMx write_flash 0x0 build\bootloader\bootloader.bin 0x10000 build\micropython_bpuppy.bin 0x8000 build\partition_table\partition-table.bin
echo.
goto :end

:error
echo.
echo ============================================================
echo   BUILD FAILED! Check the error messages above.
echo ============================================================
exit /b 1

:end
