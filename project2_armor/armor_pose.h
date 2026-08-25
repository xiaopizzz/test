#pragma once
#include "armor_detector.h"
#include <opencv2/opencv.hpp>

// ============================================================================
// 本文件负责“从 2D 检测结果推算 3D 信息”。
// 已知相机内参和装甲板的真实尺寸，通过 solvePnP 求解装甲板相对相机的位置(平移)
// 和朝向(旋转)，进而算出相机到装甲板的距离，并将坐标轴画回图像用作可视化。
// ============================================================================

// 装甲板的三维姿态和距离结果。坐标系约定为相机坐标系：X 向右，Y 向下，Z 向前。
struct ArmorPose {
    bool valid = false;        // 是否成功解算出一组有效姿态。
    double distance = 0.0;     // 相机到装甲板中心的直线距离，单位 mm。
    cv::Vec3d rvec, tvec;      // solvePnP 输出的旋转向量和位移向量。
    cv::Point2f center;        // 装甲板中心在图像上的像素坐标。
};

// 使用已标定的相机内参和装甲板实际尺寸进行距离、姿态解算。
class myArmorPoseEstimator {
public:
    // armor_width_mm / armor_height_mm 必须与实际装甲板尺寸一致。
    // camera_matrix: 3x3 内参矩阵；dist_coeffs: 畸变系数(可用 1x5 全 0 表示无畸变)。
    myArmorPoseEstimator(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                         cv::Size image_size, cv::Size2f armor_size_mm = {135.0F, 55.0F});
    // 输入一个装甲板目标，返回解算出的姿态（含距离）。
    ArmorPose estimate(const ArmorTarget& target) const;
    // 在图像上绘制 XYZ 三根坐标轴，axis_length_mm 控制坐标轴长度(mm)。
    void drawAxes(cv::Mat& image, const ArmorPose& pose, float axis_length_mm = 50.0F) const;
private:
    cv::Mat camera_matrix_, dist_coeffs_; // 内参矩阵与畸变系数的本地副本。
    cv::Size image_size_;                // 图像尺寸（备用，便于扩展）。
    cv::Size2f armor_size_mm_;           // 装甲板真实宽度与高度(mm)。
};
