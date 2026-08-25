#include "serial_port.h"
// 本文件只在 Linux 下实现真实串口；其他平台退化为空实现（open 返回 false）。

#ifdef __linux__
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/select.h>

namespace {
// 将常用整数波特率映射到 termios 常量。
speed_t baud(int value) {
    switch (value) {
    case 9600: return B9600; case 19200: return B19200; case 38400: return B38400;
    case 57600: return B57600; case 115200: return B115200;
    default: return B115200; // 未知波特率时兜底为 115200。
    }
}
}

// 析构：关闭设备，防止句柄泄漏。
mySerialPort::~mySerialPort() { close(); }
// 打开并配置串口设备。
bool mySerialPort::open(const std::string& device, int baud_rate) {
    // O_NOCTTY 防止串口成为当前进程控制终端，O_SYNC 确保写入及时发送。
    // 若上一次已打开，先 close 清理，失败则返回 false。
    close(); fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC); if (fd_ < 0) return false;
    // 读取当前终端配置，失败则清理并返回。
    termios tty{}; if (tcgetattr(fd_, &tty) != 0) { close(); return false; }
    // 设置收发波特率。
    cfsetispeed(&tty, baud(baud_rate)); cfsetospeed(&tty, baud(baud_rate));
    // 配置 8N1，并关闭硬件流控；这与常见 USB-TTL 模块默认设置一致。
    // c_cflag：清掉字符长度位，设为 8 位；CLOCAL 忽略调制解调器控制线，CREAD 开启接收。
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8 | CLOCAL | CREAD;
    // 清除奇偶/偶校验、两位停止位和硬件流控位；清空输入输出/本地标志(裸模式)。
    tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS); tty.c_iflag = 0; tty.c_oflag = 0; tty.c_lflag = 0;
    // VMIN=0：read 至少返回已到达的字节（非阻塞语义）；VTIME=1：读等待上限 0.1 秒。
    tty.c_cc[VMIN] = 0; tty.c_cc[VTIME] = 1;
    // 立即应用配置；失败则清理并返回。
    if (tcsetattr(fd_, TCSANOW, &tty) != 0) { close(); return false; } return true;
}
// 关闭设备：fd 有效时才 close，并复位为 -1。
void mySerialPort::close() { if (fd_ >= 0) { ::close(fd_); fd_ = -1; } }
// 是否打开：看 fd 是否非负。
bool mySerialPort::isOpen() const { return fd_ >= 0; }
// 写入一帧：先用协议编码（补帧头帧尾），再一次性写出；要求写满全部字节才算成功。
bool mySerialPort::writeFrame(const std::vector<uint8_t>& payload) const {
    if (!isOpen()) return false; const auto bytes = mySerialProtocol::encode(payload);
    return ::write(fd_, bytes.data(), bytes.size()) == static_cast<ssize_t>(bytes.size());
}
// 读取一帧：先看缓存里是否已有完整帧，否则阻塞等待可读后读取更多数据。
bool mySerialPort::readFrame(SerialFrame& frame, int timeout_ms) {
    // 先消费缓存中已经完整的帧，再等待底层文件描述符变为可读。
    if (decoder_.pop(frame)) return true; if (!isOpen()) return false;
    // 用 select 等待可读，超时由 timeout_ms 决定（换算成秒+微秒）。
    fd_set set; FD_ZERO(&set); FD_SET(fd_, &set); timeval tv{timeout_ms / 1000, (timeout_ms % 1000) * 1000};
    if (select(fd_ + 1, &set, nullptr, nullptr, &tv) <= 0) return false; // 超时/出错。
    // 读到字节后喂给协议解码器，再尝试弹出一帧。
    uint8_t bytes[256]; const auto count = ::read(fd_, bytes, sizeof(bytes)); if (count <= 0) return false;
    decoder_.push(bytes, static_cast<size_t>(count)); return decoder_.pop(frame);
}
// ---- 非 Linux 平台：最小化空实现，保证程序可编译链接运行。 ----
#else
mySerialPort::~mySerialPort() = default;
bool mySerialPort::open(const std::string&, int) { return false; }
void mySerialPort::close() {}
bool mySerialPort::isOpen() const { return false; }
bool mySerialPort::writeFrame(const std::vector<uint8_t>&) const { return false; }
bool mySerialPort::readFrame(SerialFrame&, int) { return false; }
#endif
