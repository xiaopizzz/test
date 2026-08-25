#include "motion_observer.h"
#include <algorithm>
#include <cmath>
// 本文件实现 myMotionObserver（alpha-beta 观测器）的全部方法。

// 构造函数：保存两路修正增益。
myMotionObserver::myMotionObserver(float position_gain, float velocity_gain) : alpha_(position_gain), beta_(velocity_gain) {}
// 重置：清空初始化标志、位置、速度和时间戳。
void myMotionObserver::reset() { initialized_ = false; position_ = {}; velocity_ = {}; last_time_ = 0.0; }
// 核心更新步骤：用最新测量修正位置和速度，返回滤波后的位置。
cv::Point2f myMotionObserver::update(cv::Point2f measured, double timestamp_seconds) {
    // 第一次测量：无法算速度，直接把测量值当作当前位置并记录时间戳。
    if (!initialized_) { initialized_ = true; position_ = measured; last_time_ = timestamp_seconds; return position_; }
    // dt 为两帧之间的时间差，限制在 [0.001, 1.0] 秒，避免除零或异常大跳变。
    const float dt = static_cast<float>(std::clamp(timestamp_seconds - last_time_, 0.001, 1.0));
    // 用当前速度外推一步，得到“预测位置”。
    const cv::Point2f predicted_position = position_ + velocity_ * dt;
    // 残差 = 实测 - 预测，度量预测偏离实测的程度。
    const cv::Point2f residual = measured - predicted_position;
    // alpha-beta 核心公式：位置在预测基础上按 alpha 修正，速度按 beta/dt 修正。
    // 位置: position = 预测 + 残差*alpha
    position_ = predicted_position + residual * alpha_;
    // 速度: velocity += 残差 * (beta/dt)（除以 dt 是因为残差是位置差，要还原成速度率）。
    velocity_ += residual * (beta_ / dt);
    last_time_ = timestamp_seconds; return position_;
}
// 对外预测：用当前状态外推 ahead_seconds 秒后的位置。
cv::Point2f myMotionObserver::predicted(double ahead_seconds) const { return position_ + velocity_ * static_cast<float>(ahead_seconds); }
// 速度大小：速度向量与自身点积再开方，即模长（像素/秒）。
float myMotionObserver::speed() const { return std::sqrt(velocity_.dot(velocity_)); }
