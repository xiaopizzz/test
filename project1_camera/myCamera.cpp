#include "myCamera.h"
#include <utility>
// 本文件实现 myCamera 声明在头文件里的全部方法，负责真正调用 OpenCV 的
// cv::VideoCapture，并管理底层资源生命周期。

// 默认构造：让 capture_ / source_ 走各自的默认初始化即可，什么都不做。
myCamera::myCamera() = default;
// 构造时打开编号设备，打开失败时对象仍保持可安全析构的关闭状态。
myCamera::myCamera(int index) { open(index); }
// 构造时打开字符串源（文件路径 / 视频流地址）。
myCamera::myCamera(const std::string& source) { open(source); }
// 析构：无论设备是否打开都安全调用 release()，防止句柄泄漏。
myCamera::~myCamera() { release(); }
// 移动构造：把另一个对象的底层句柄“搬”过来，而不是复制，保证同一资源只被一个对象拥有。
myCamera::myCamera(myCamera&& other) noexcept : capture_(std::move(other.capture_)), source_(std::move(other.source_)) {}
// 移动赋值：先释放自己手上旧设备，再接管对方资源；判断 this!=&other 防止自赋值。
myCamera& myCamera::operator=(myCamera&& other) noexcept {
    if (this != &other) { release(); capture_ = std::move(other.capture_); source_ = std::move(other.source_); }
    return *this;
}
// 先释放旧设备，防止重复 open 时泄漏句柄；V4L2 失败后让 OpenCV 自动选择后端。
bool myCamera::open(int index) { release(); source_ = std::to_string(index); return capture_.open(index, cv::CAP_V4L2) || capture_.open(index); }
// 打开字符串输入源（自动识别文件 / RTSP / HTTP 等协议），并记录源用于日志。
bool myCamera::open(const std::string& source) { release(); source_ = source; return capture_.open(source); }
// release 可重复调用，因此析构函数和重新打开都可以直接使用。
void myCamera::release() { if (capture_.isOpened()) capture_.release(); }
// 只有设备已打开时才返回 true。
bool myCamera::isOpened() const { return capture_.isOpened(); }
// 先确认设备打开，再读取一帧；同时要求读到的图像非空，避免把空帧当有效帧。
bool myCamera::read(cv::Mat& frame) { return capture_.isOpened() && capture_.read(frame) && !frame.empty(); }
// 透传给 OpenCV 的属性设置接口。
bool myCamera::set(int property, double value) { return capture_.set(property, value); }
// 透传给 OpenCV 的属性查询接口。
double myCamera::get(int property) const { return capture_.get(property); }
// 读取帧宽，用 get() 包装为 int 返回；设备未打开时为 0。
int myCamera::width() const { return static_cast<int>(get(cv::CAP_PROP_FRAME_WIDTH)); }
// 读取帧高，同样包装为 int。
int myCamera::height() const { return static_cast<int>(get(cv::CAP_PROP_FRAME_HEIGHT)); }
// 读取目标帧率，单位 fps。
double myCamera::fps() const { return get(cv::CAP_PROP_FPS); }
// 返回最初记录的输入源字符串。
const std::string& myCamera::source() const { return source_; }
