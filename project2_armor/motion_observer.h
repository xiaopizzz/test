#pragma once
#include <opencv2/opencv.hpp>

// ============================================================================
// alpha-beta 观测器：用当前位置修正速度，再预测下一时刻位置，适合装甲板中心跟踪。
// 这是一种经典的一维/二维目标跟踪滤波器：用位置残差去同时修正“位置”和“速度”
// 两个状态，alpha 控制位置修正强度，beta 控制速度修正强度。
// ============================================================================
class myMotionObserver {
public:
    // position_gain(alpha)：位置修正比例，(0,1) 之间；越大越信任测量值。
    // velocity_gain(beta)：速度修正比例，(0,1) 之间；越大速度跟随越快。
    explicit myMotionObserver(float position_gain = 0.65F, float velocity_gain = 0.25F);
    // 清空所有状态，让滤波器重新开始（例如丢失目标后重置）。
    void reset();
    // 喂入一个最新测量位置(像素)与对应时间戳(秒)，返回滤波后的当前位置。
    cv::Point2f update(cv::Point2f measured, double timestamp_seconds);
    // 预测 ahead_seconds 秒之后的位置（用于提前量 / 画运动箭头）。
    cv::Point2f predicted(double ahead_seconds = 0.05) const;
    // 当前估计的速度向量（像素/秒）。
    cv::Point2f velocity() const { return velocity_; }
    // 当前速度大小（像素/秒），开方点乘结果。
    float speed() const;
    // 是否已经收到至少一次测量（决定 update 是否走初始化分支）。
    bool initialized() const { return initialized_; }
private:
    float alpha_, beta_;       // 位置增益与速度增益。
    bool initialized_ = false; // 是否已初始化（收到第一帧测量）。
    double last_time_ = 0.0;   // 上一次测量的时间戳(秒)，用于计算时间差 dt。
    cv::Point2f position_{}, velocity_{}; // 当前估计位置(像素)与速度(像素/秒)。
};
