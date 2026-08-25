#pragma once
#include "serial_protocol.h"
#include <string>
#include <vector>
#include <cstdint>

// ============================================================================
// Ubuntu 串口设备封装，底层使用 termios，适用于 USB 转 TTL 的 /dev/ttyUSB*。
// 本类负责打开/关闭串口与收发字节；帧协议交由 mySerialProtocol 处理。
// ============================================================================
class mySerialPort {
public:
    mySerialPort() = default;
    ~mySerialPort();
    // 串口句柄不宜复制，删除拷贝；如确实需要多份，应分别打开不同设备。
    mySerialPort(const mySerialPort&) = delete;
    mySerialPort& operator=(const mySerialPort&) = delete;

    // 打开设备并配置为 8 数据位、无校验、1 停止位（8N1）。
    // device 如 "/dev/ttyUSB0"，baud_rate 为波特率，默认 115200。
    bool open(const std::string& device, int baud_rate = 115200);
    // 关闭设备（幂等，可重复调用）。
    void close();
    // 设备当前是否处于打开状态。
    bool isOpen() const;
    // 把 payload 编成一帧（自动加帧头帧尾）并写入串口；返回是否写完整。
    bool writeFrame(const std::vector<uint8_t>& payload) const;
    // 等待并解析一帧，timeout_ms 防止主循环永久阻塞。
    bool readFrame(SerialFrame& frame, int timeout_ms = 100);
private:
    int fd_ = -1;                 // 文件描述符，-1 表示未打开。
    mySerialProtocol decoder_;    // 帧协议解码器，内部维护接收缓存。
};
