#include "serial_protocol.h"
#include <algorithm>
#include <cstring>
// 本文件实现 0xAA + payload + 0xBB 帧协议的编解码，不依赖操作系统。
// 给有效载荷加上固定帧头和帧尾，原样保留中间数据。
std::vector<uint8_t> mySerialProtocol::encode(const std::vector<uint8_t>& p) { std::vector<uint8_t> f; f.reserve(p.size()+2); f.push_back(0xAA); f.insert(f.end(),p.begin(),p.end()); f.push_back(0xBB); return f; }
// 使用 memcpy 保留 float 的 IEEE-754 内存表示，收发两端需约定字节序。
// 先封成 4 字节，再套帧头帧尾。
std::vector<uint8_t> mySerialProtocol::encodeFloat(float v) { std::vector<uint8_t> p(4); std::memcpy(p.data(),&v,4); return encode(p); }
// 与 encodeFloat 同理，把 32 位整数原样拷贝到 4 字节再组帧。
std::vector<uint8_t> mySerialProtocol::encodeInt32(int32_t v) { std::vector<uint8_t> p(4); std::memcpy(p.data(),&v,4); return encode(p); }
// 把本次读到的字节追加到接收缓存，供后续 pop 解析；支持一帧分多次到达。
void mySerialProtocol::push(const uint8_t* d,size_t n) { buffer_.insert(buffer_.end(),d,d+n); }
// 从缓存中尝试弹出一帧：找 0xAA 作为起点，再找 0xBB 作为终点。
bool mySerialProtocol::pop(SerialFrame& out) { auto a=std::find(buffer_.begin(),buffer_.end(),0xAA); if(a==buffer_.end()){buffer_.clear();return false;} auto b=std::find(a+1,buffer_.end(),0xBB); if(b==buffer_.end()){buffer_.erase(buffer_.begin(),a);return false;} out.payload.assign(a+1,b); buffer_.erase(buffer_.begin(),b+1); return true; }
