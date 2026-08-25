#include "armor_detector.h"
#include <opencv2/opencv.hpp>
#include <algorithm>
#include <cmath>

namespace {
// 返回矩形长轴相对竖直方向(Y 轴)的夹角（度）。
// 直接根据四个角点求最长边的方向，避免被 OpenCV minAreaRect 的 size/angle 表示不一致误导
// （宽高交换或符号不同都会让"角度从竖直方向"的换算出错）。
float angle(const cv::RotatedRect& r) {
    cv::Point2f p[4]; r.points(p);
    const float e1 = cv::norm(p[1] - p[0]);  // 边 0 长度
    const float e2 = cv::norm(p[2] - p[1]);  // 边 1 长度
    // 取较长的一条边作为"长轴"
    const cv::Point2f axis = (e1 >= e2) ? (p[1] - p[0]) : (p[2] - p[1]);
    // 长轴与竖直方向的夹角，只关心偏离竖直的程度：对两分量取绝对值。
    // 图像坐标系 y 轴向下，长轴可能朝上(负 y)也可能朝下，取绝对值即可统一。
    return std::abs(std::atan2(std::abs(axis.x), std::abs(axis.y)) * 180.0 / CV_PI);
}
}
// 构造函数：把外传的配置存下来，方便后续 detect 使用。
myArmorDetector::myArmorDetector(ArmorConfig config) : config_(config) {}
std::vector<ArmorTarget> myArmorDetector::detect(const cv::Mat& frame, cv::Mat* debug) const {
    std::vector<ArmorTarget> result; if (frame.empty()) return result;
    // 采用颜色差分可以压制白色环境光：红灯看 R-B，蓝灯看 B-R。
    // 先把图像拆成 B、G、R 三通道，再根据目标颜色做对应通道的差分。
    cv::Mat bgr[3]; cv::split(frame, bgr); cv::Mat mask;
    if (config_.color == ArmorColor::Red) { cv::Mat m; cv::subtract(bgr[2], bgr[0], m); cv::threshold(m, mask, config_.binary_threshold, 255, cv::THRESH_BINARY); }
    else {
        // 蓝色装甲板：光晕很强，单靠 B-R 差分会把灯条和周围光晕连成一大团，
        // 导致外接矩形倾角异常而被过滤。
        // 这里额外加两个约束：蓝通道足够亮(B>200) 且 红通道不太高(R<160)，
        // 只保留灯条本体、剔除白色环境光与弥散光晕，从而得到两根竖直灯条。
        cv::Mat m, gb, gr;
        cv::subtract(bgr[0], bgr[2], m);                       // B - R
        cv::threshold(m, mask, config_.binary_threshold, 255, cv::THRESH_BINARY);   // B-R > 阈值
        cv::threshold(bgr[0], gb, 200, 255, cv::THRESH_BINARY);                    // B > 200
        cv::threshold(bgr[2], gr, 160, 255, cv::THRESH_BINARY_INV);                // R < 160
        cv::bitwise_and(mask, gb, mask); cv::bitwise_and(mask, gr, mask);
    }
    // 闭运算连接灯条内部的断裂像素，同时填补小孔洞。
    // 使用 3x3 矩形结构元素做 MORPH_CLOSE，让灯条区域更完整。
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::getStructuringElement(cv::MORPH_RECT, {3, 3}));
    // 提取外部轮廓，作为候选灯条区域。
    std::vector<std::vector<cv::Point>> contours; cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::vector<cv::RotatedRect> lights;
    // 先过滤面积、长度和倾角，得到候选灯条，减少后续两两配对的计算量。
    // 条件依次为：面积 >=10；最小外接矩形的高应超过阈值；宽必须为正；倾角不能过大。
    for (const auto& c : contours) { if (cv::contourArea(c) < 10) continue; auto r = cv::minAreaRect(c); float h = std::max(r.size.width, r.size.height), w = std::min(r.size.width, r.size.height); if (h < config_.min_light_height || w <= 0 || angle(r) > config_.max_light_angle) continue; lights.push_back(r); }
    // 按中心 x 坐标排序，让灯条按“左→右”顺序排列，方便配对。
    std::sort(lights.begin(), lights.end(), [](auto& a, auto& b) { return a.center.x < b.center.x; });
    // 两两配对：i 与 i 之后的所有 j 组成候选对。
    for (size_t i = 0; i < lights.size(); ++i) for (size_t j = i + 1; j < lights.size(); ++j) {
        auto& l = lights[i]; auto& r = lights[j]; float lh = std::max(l.size.width,l.size.height), rh = std::max(r.size.width,r.size.height), avg=(lh+rh)/2.0F;
        float dy=std::abs(l.center.y-r.center.y), dx=std::abs(l.center.x-r.center.x), ratio=dx/avg;
        // 配对约束：两灯高度接近、纵向错位小、横向间距处于合理比例。
        // 不满足任意一条约束就跳过这一对。
        if (std::max(lh,rh)/std::min(lh,rh)>config_.max_height_ratio || dy/avg>config_.max_y_diff_ratio || ratio<config_.min_pair_ratio || ratio>config_.max_pair_ratio) continue;
        // 取左右两灯条各自的最小外接矩形，再合并成一个总的包围框。
        cv::Point2f pts[4]; l.points(pts); cv::Rect box = cv::boundingRect(std::vector<cv::Point2f>(pts, pts+4)); r.points(pts); box |= cv::boundingRect(std::vector<cv::Point2f>(pts, pts+4));
        // 组装结果：中心取两灯条中心的中点；置信度用纵向错位程度估算，错位越小越可信。
        ArmorTarget a{l,r,box,(l.center+r.center)*0.5F, 1.0F/(1.0F+dy/avg)}; result.push_back(a);
        // 若调用者提供了 debug 图像，就绘制绿色目标框和蓝色中心点。
        if (debug) { cv::rectangle(*debug, box, {0,255,0}, 2); cv::circle(*debug, a.center, 3, {255,0,0}, -1); }
    }
    return result;
}
