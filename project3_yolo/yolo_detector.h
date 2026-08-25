#pragma once
#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
// YOLO 输出经过解码后的统一目标结构。
struct YoloTarget { int class_id; float confidence; cv::Rect box; };
// 使用 OpenCV DNN 读取 ONNX 等模型，避免绑定某个训练框架的运行时。
// 当前工程使用 Ultralytics 标准导出的 YOLOv8 ONNX：输出单个张量 [1, 4+类别数, 锚点数]，
// 解码逻辑位于 cpp 文件。
class myYoloDetector {
public:
    // model 为权重文件，classes 为每行一个类别名的文本文件。
    // confidence: 置信度阈值；nms: 非极大值抑制阈值。二者都是可调参数。
    bool load(const std::string& model, const std::string& classes = "", float confidence = 0.25F, float nms = 0.45F);
    // 对单帧图像执行缩放、前向推理和置信度过滤。
    // 返回经过 NMS 去重后的目标列表。
    std::vector<YoloTarget> detect(const cv::Mat& frame) const;
private:
    cv::dnn::Net net_;                 // OpenCV DNN 网络对象。
    std::vector<std::string> classes_; // 类别名称，仅用于上层显示。
    float confidence_ = 0.25F;         // 置信度阈值。
    float nms_ = 0.45F;                // NMS 阈值，保留为配置项供后续扩展。
    int input_size_ = 640;             // YOLO 常用输入边长。
};
