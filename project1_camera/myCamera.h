#pragma once
#include <opencv2/opencv.hpp>
#include <string>

// ============================================================================
// 类名：myCamera
// 作用：跨平台相机封装，统一处理三类输入源——
//       1) 本地摄像头编号（int index）
//       2) 本地视频文件路径（std::string，例如 "test.avi"）
//       3) 网络视频流地址（std::string，例如 RTSP / HTTP）
//           内部真正干活的是 OpenCV 的 cv::VideoCapture，本类只是把它包装成
//           易于使用的接口，并顺带记录输入源字符串以便日志与调试。
// ============================================================================
class myCamera {
public:
    // ---- 构造函数 ------------------------------------------------
    // 默认构造：不立即打开任何设备，后面可再用 open() 手动打开。
    myCamera();
    // 按摄像头编号构造，等价于先构造再调用 open(index)。
    explicit myCamera(int index);
    // 按字符串源构造（文件路径 / 视频流地址），等价于 open(source)。
    explicit myCamera(const std::string& source);
    // 析构：自动释放底层设备句柄，避免资源泄漏。
    ~myCamera();

    // ---- 拷贝 / 移动控制 ------------------------------------------
    // cv::VideoCapture 在内部持有系统资源句柄，若允许隐式拷贝会读到同一份
    // 资源并导致双重释放等未定义行为，因此显式删除拷贝操作，只保留移动语义，
    // 允许把“相机的所有权”从一个对象转移给另一个对象。
    myCamera(const myCamera&) = delete;
    myCamera& operator=(const myCamera&) = delete;
    myCamera(myCamera&& other) noexcept;
    myCamera& operator=(myCamera&& other) noexcept;

    // ---- 打开 / 关闭设备 ------------------------------------------
    // index：摄像头编号。Linux 下优先用 V4L2 后端，失败后再让 OpenCV 自动挑，
    //        这样可以绕过某些环境里默认后端打不开设备的问题。
    bool open(int index);
    // source：视频文件路径或 RTSP 等实时流地址，直接交给 OpenCV 识别协议。
    bool open(const std::string& source);
    // 释放底层视频流；重复调用是安全的（内部会先判断句柄是否有效）。
    void release();
    // 设备是否已成功打开且可读取。
    bool isOpened() const;

    // ---- 读取与参数 ----------------------------------------------
    // 读取一帧 BGR 图像写入 frame。返回 false 表示设备未打开或流已结束。
    bool read(cv::Mat& frame);
    // 设置 OpenCV 视频属性（如 CAP_PROP_FRAME_WIDTH），返回值表示是否成功。
    bool set(int property, double value);
    // 查询 OpenCV 视频属性，units 由 property 决定（像素 / fps 等）。
    double get(int property) const;

    // ---- 便捷的常用属性 ------------------------------------------
    // 下三个接口封装了最常见参数的查询，返回 0 表示设备未打开或属性无效。
    int width() const;    // 当前帧宽（像素）
    int height() const;   // 当前帧高（像素）
    double fps() const;   // 目标帧率（可能被设备真实帧率覆盖）

    // 返回最初传入的输入源字符串（摄像头编号或路径/地址）。
    const std::string& source() const;

private:
    cv::VideoCapture capture_; // OpenCV 底层输入对象，真正负责打开与读取。
    std::string source_;        // 保存输入源，便于日志和调试。
};
