#include "armor_pose.h"
#include <cmath>
// 本文件实现 myArmorPoseEstimator：用 2D 检测框反推 3D 姿态与距离，并绘制坐标轴。

// 构造函数：保存图像尺寸和装甲板真实尺寸，并把内参、畸变系数拷贝到成员变量。
// 用 copyTo 是为了脱离传入矩阵，避免外部之后修改影响内部使用。
myArmorPoseEstimator::myArmorPoseEstimator(const cv::Mat& camera_matrix, const cv::Mat& dist_coeffs,
                                           cv::Size image_size, cv::Size2f armor_size_mm)
    : image_size_(image_size), armor_size_mm_(armor_size_mm) {
    camera_matrix.copyTo(camera_matrix_); dist_coeffs.copyTo(dist_coeffs_);
}

// 核心解算函数：输入检测到的装甲板目标，输出姿态与距离。
ArmorPose myArmorPoseEstimator::estimate(const ArmorTarget& target) const {
    ArmorPose result; result.center = target.center;
    // 内参为空或不是 3x3 时无法解算，直接返回无效姿态。
    if (camera_matrix_.empty() || camera_matrix_.rows != 3 || camera_matrix_.cols != 3) return result;
    // 以装甲板中心为原点，四个角点位于同一平面 Z=0。
    // 按“左上、右上、右下、左下”顺序构造装甲板四个角点的 3D 坐标(mm)。
    const float w = armor_size_mm_.width / 2.0F, h = armor_size_mm_.height / 2.0F;
    std::vector<cv::Point3f> object = {{-w,-h,0}, {w,-h,0}, {w,h,0}, {-w,h,0}};
    std::vector<cv::Point2f> image;
    // 用检测框四角作为稳定的近似角点，按图像位置排序得到左上、右上、右下、左下。
    // 对于强旋转目标，灯条旋转角点可进一步替换这里的矩形角点。
    cv::Rect box = target.box; image = {{(float)box.x,(float)box.y}, {(float)(box.x+box.width),(float)box.y},
                                        {(float)(box.x+box.width),(float)(box.y+box.height)}, {(float)box.x,(float)(box.y+box.height)}};
    // solvePnP：用 4 对 3D-2D 点求外参。使用 IPPE 算法，该算法对共面点求解更稳定。
    if (!cv::solvePnP(object, image, camera_matrix_, dist_coeffs_, result.rvec, result.tvec, false, cv::SOLVEPNP_IPPE)) return result;
    // 距离取位移向量 tvec 的模长；距离必须 > 0 才认为结果有效。
    result.distance = cv::norm(result.tvec); result.valid = result.distance > 0.0; return result;
}

// 把 XYZ 三个坐标轴投影回图像，并绘制线段与标签。
void myArmorPoseEstimator::drawAxes(cv::Mat& image, const ArmorPose& pose, float axis_length_mm) const {
    if (!pose.valid) return; // 姿态无效时直接跳过绘制。
    // 定义原点(0,0,0)与 X/Y/Z 三个方向端点（长度 axis_length_mm，单位 mm）。
    std::vector<cv::Point3f> axes = {{0,0,0}, {axis_length_mm,0,0}, {0,axis_length_mm,0}, {0,0,axis_length_mm}};
    // projectPoints 把 3D 点投影为图像像素坐标。
    std::vector<cv::Point2f> pixels; cv::projectPoints(axes, pose.rvec, pose.tvec, camera_matrix_, dist_coeffs_, pixels);
    // 依次画 X(红)、Y(绿)、Z(蓝)三根轴；颜色约定为 BGR。
    cv::line(image, pixels[0], pixels[1], {0,0,255}, 2); cv::line(image, pixels[0], pixels[2], {0,255,0}, 2); cv::line(image, pixels[0], pixels[3], {255,0,0}, 2);
    // 在轴末端标注 X、Y、Z，便于人工核对坐标系方向。
    cv::putText(image, "X", pixels[1], cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,0,255}, 1);
    cv::putText(image, "Y", pixels[2], cv::FONT_HERSHEY_SIMPLEX, 0.5, {0,255,0}, 1);
    cv::putText(image, "Z", pixels[3], cv::FONT_HERSHEY_SIMPLEX, 0.5, {255,0,0}, 1);
}
