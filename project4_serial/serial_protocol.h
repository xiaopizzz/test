#pragma once
#include <cstdint>
#include <cstddef>
#include <vector>
// ============================================================================
// 串口帧协议：一帧固定为 0xAA（帧头）+ payload（任意字节）+ 0xBB（帧尾）。
// 本文件负责编解码，不关心底层物理串口，因此可跨平台复用。
// ============================================================================
// 一帧串口数据只保存帧头和帧尾之间的有效载荷。
struct SerialFrame { std::vector<uint8_t> payload; };
// 协议格式严格为：0xAA（帧头）+ payload（任意字节）+ 0xBB（帧尾）。
class mySerialProtocol {
public:
    // 编码函数不添加长度字段，适合题目要求的简单单帧通信格式。
    // 给任意字节 payload 套上帧头帧尾。
    static std::vector<uint8_t> encode(const std::vector<uint8_t>& payload);
    // 把一个 float 打包成 4 字节并套帧头帧尾（用于发送距离等浮点量）。
    static std::vector<uint8_t> encodeFloat(float value);
    // 把一个 32 位整数打包成 4 字节并套帧头帧尾。
    static std::vector<uint8_t> encodeInt32(int32_t value);
    // 把串口本次收到的字节追加到缓存，允许一帧被拆成多次 read。
    void push(const uint8_t* data, size_t size);
    // 从缓存中取出完整帧；遇到半帧时保留数据等待下一次 push。
    bool pop(SerialFrame& frame);
private:
    std::vector<uint8_t> buffer_; // 流式接收缓存，按字节累积待解析数据。
};
