#include "yolo_detector.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <cmath>
// 本文件实现 ONNX(YOLOv8) 模型的加载与解码推理。
// 当前模型使用 Ultralytics 标准导出格式：整个网络只输出一个张量，
// 形状为 [1, 4+类别数, 8400]，即“4 个框参数 + 各类别得分”的扁平输出，无需再按网格解码 DFL。

// 加载模型：model 为权重路径，classes 为可选类别文件路径。
bool myYoloDetector::load(const std::string& model, const std::string& classes, float confidence, float nms) {
    // 模型路径错误、格式不支持时 readNet 会抛异常，转换为 false 交给调用者处理。
    try { net_ = cv::dnn::readNet(model); } catch (...) { return false; }
    // 保存用户给的置信度与 NMS 阈值。
    confidence_ = confidence; nms_ = nms;
    // 类别文件不是推理必需项，因此不存在时仍允许模型加载成功。
    // 逐行读取类别名，跳过空行，存入 classes_ 供上层显示使用。
    if (!classes.empty()) { std::ifstream f(classes); std::string s; while (std::getline(f, s)) if (!s.empty()) classes_.push_back(s); }
    // 网络是否成功加载由内部 Net 是否为空决定。
    return !net_.empty();
}
// 对一帧图像做推理，并输出解码、去重后的目标框。
std::vector<YoloTarget> myYoloDetector::detect(const cv::Mat& frame) const {
    std::vector<YoloTarget> out; if (frame.empty() || net_.empty()) return out;
    // blobFromImage 将 BGR 图像缩放到网络输入，并把像素归一化到 0~1。
    // 参数说明：1/255 做归一化；尺寸为正方形 640；squeeze 不提升；减均值不启用。
    cv::Mat blob = cv::dnn::blobFromImage(frame, 1 / 255.0, {input_size_, input_size_}, {}, true, false);
    // 拷贝网络对象以便并行/复用，设置输入并执行一次前向，取出所有输出层。
    auto net = net_; net.setInput(blob);
    std::vector<cv::Mat> outputs; net.forward(outputs, net.getUnconnectedOutLayersNames());
    // 该模型只剩一个输出张量：output0，形状为 [1, 4+类别数, 锚点数]。
    // 布局为按“通道-锚点”平铺：对每个锚点 a，前 4 个数是框(cx,cy,w,h)，
    // 之后是各类别得分(已过 sigmoid，取值 0~1)。坐标均在网络输入 640 尺寸下。
    if (outputs.empty()) return out;
    const cv::Mat& o = outputs[0];
    const int channels = o.size[1];          // 4 + 类别数
    const int anchors = o.size[2];           // 锚点数（640 时为 80*80+40*40+20*20 = 8400）
    if (channels < 5 || anchors <= 0) return out;
    const int nclasses = channels - 4;       // 类别数 = 通道数 - 4
    const float* data = reinterpret_cast<const float*>(o.data);
    // 每通道在内存里连续存放 anchors 个值，因此跨通道偏移量就是 anchors。
    const float sx = static_cast<float>(frame.cols) / input_size_;
    const float sy = static_cast<float>(frame.rows) / input_size_;
    std::vector<cv::Rect> boxes; std::vector<float> scores; std::vector<int> ids;
    boxes.reserve(anchors); scores.reserve(anchors); ids.reserve(anchors);
    // 遍历每个锚点，找出得分最高的类别并换算成原图坐标系下的矩形框。
    for (int a = 0; a < anchors; ++a) {
        // 读取框参数：cx, cy, w, h（网络输入 640 尺寸下的像素值）。
        const float cx = data[0 * anchors + a], cy = data[1 * anchors + a];
        const float w = data[2 * anchors + a], h = data[3 * anchors + a];
        int best_class = -1; float best_score = 0.0F;
        for (int c = 0; c < nclasses; ++c) {
            float p = data[(4 + c) * anchors + a];
            if (p > best_score) { best_score = p; best_class = c; }
        }
        if (best_score < confidence_) continue; // 低于置信度阈值直接丢弃。
        // 中心点 ± 半宽半高得到左上角与宽高，并乘以缩放比换回原图坐标。
        float x1 = (cx - w * 0.5F) * sx, y1 = (cy - h * 0.5F) * sy;
        float x2 = (cx + w * 0.5F) * sx, y2 = (cy + h * 0.5F) * sy;
        boxes.emplace_back(static_cast<int>(x1), static_cast<int>(y1),
                           static_cast<int>(x2 - x1), static_cast<int>(y2 - y1));
        scores.push_back(best_score); ids.push_back(best_class);
    }
    // 对所有候选框做 NMS 去重，压制同一目标的重复检测。
    std::vector<int> keep; cv::dnn::NMSBoxes(boxes, scores, confidence_, nms_, keep);
    // 按保留的索引组装最终输出。
    for (int i : keep) out.push_back({ids[i], scores[i], boxes[i]});
    return out;
}
