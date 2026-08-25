#pragma once
#include <opencv2/opencv.hpp>
#include <vector>

// ============================================================================
// 本头文件定义“传统视觉”装甲板检测所需的类型与封装类。
// 思路：装甲板由左右两条竖直灯条构成，灯条发出红光或蓝光。
//       通过“颜色通道差分 + 二值化 + 轮廓过滤 + 两两配对”找到装甲板。
// ============================================================================

// 待识别的灯条颜色。这里比较 BGR 通道差值，而不是依赖 HSV 的固定范围。
// （红灯：R 通道显著大于 B 通道；蓝灯：B 通道显著大于 R 通道）
enum class ArmorColor { Red, Blue };
// 一个装甲板由左右两个灯条组成，box 和 center 用于后续瞄准或绘制。
struct ArmorTarget {
    cv::RotatedRect left, right;  // 左右两个灯条的最小外接旋转矩形。
    cv::Rect box;                 // 合并左右灯条后的外接矩形（目标框）。
    cv::Point2f center;           // 装甲板中心点（两灯条中心的中点）。
    float confidence = 0.0F;      // 简单置信度，值越大表示配对越合理。
};
// 传统视觉筛选参数，可根据相机分辨率和赛场光照调整。
struct ArmorConfig {
    ArmorColor color = ArmorColor::Red; // 目标灯条颜色，默认红色。
    int binary_threshold = 80;           // 颜色差分后的二值化阈值（越大越严格）。
    int min_light_height = 8;            // 灯条最小长度（像素），太小视为噪声。
    float max_light_angle = 35.0F;       // 灯条相对竖直方向的最大偏角（度）。
    float max_height_ratio = 1.8F;       // 两灯条长度比上限，超出认为不是一对。
    float max_y_diff_ratio = 0.5F;       // 两灯条纵向错位(相对平均长度)上限。
    float min_pair_ratio = 0.8F;         // 两灯条间距/平均长度的下限。
    float max_pair_ratio = 5.5F;         // 两灯条间距/平均长度的上限。
};

class myArmorDetector {
public:
    // 保存配置，不在构造阶段访问摄像头或分配大图像缓存。
    explicit myArmorDetector(ArmorConfig config = {});
    // 流程：颜色通道差分 -> 二值化 -> 形态学闭运算 -> 灯条轮廓筛选 -> 两两配对。
    // debug 非空时在传入图像上绘制检测框和中心点。
    std::vector<ArmorTarget> detect(const cv::Mat& frame, cv::Mat* debug = nullptr) const;
    // 返回当前使用的配置（只读），便于外部查询参数。
    const ArmorConfig& config() const { return config_; }
private:
    ArmorConfig config_; // 保存本实例使用的检测配置。
};
