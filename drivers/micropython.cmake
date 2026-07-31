# ============================================================
# bPuppy 用户自定义 C 模块构建文件
# 该文件由 USER_C_MODULES 变量指向
# micropython-helper 会在构建过程中 include 这个文件
# 用户模块应添加到 "usermod" 目标（MicroPython 标准）
# ============================================================

# 添加模块源文件到 usermod 目标（usermod 是 INTERFACE 库）
target_sources(usermod INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/bpuppy_version.c
    ${CMAKE_CURRENT_LIST_DIR}/servo_driver.c
    ${CMAKE_CURRENT_LIST_DIR}/imu_driver.c
    ${CMAKE_CURRENT_LIST_DIR}/uart_driver.c
    ${CMAKE_CURRENT_LIST_DIR}/adc_driver.c
    ${CMAKE_CURRENT_LIST_DIR}/ik.c
    ${CMAKE_CURRENT_LIST_DIR}/motion_task.cpp
    ${CMAKE_CURRENT_LIST_DIR}/motion_task_mpy.c
    ${CMAKE_CURRENT_LIST_DIR}/ble_driver.c
    ${CMAKE_CURRENT_LIST_DIR}/ble_driver_mpy.c
    ${CMAKE_CURRENT_LIST_DIR}/camera_driver.c
)

# 添加模块头文件搜索路径
target_include_directories(usermod INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}
    # esp32-camera 组件 (managed_component 头文件不会自动传播到 usermod)
    ${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__esp32-camera/driver/include
    ${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__esp32-camera/conversions/include
    # esp_jpeg (esp32-camera 的私有依赖, jpeg_decoder.h 被 img_converters.h 引用)
    ${CMAKE_CURRENT_LIST_DIR}/../managed_components/espressif__esp_jpeg/include
)
