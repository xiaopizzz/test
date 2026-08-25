# 项目 4 同时包含纯协议解析和 Linux 串口设备实现。
add_library(my_serial STATIC serial_protocol.cpp serial_port.cpp)
target_include_directories(my_serial PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
