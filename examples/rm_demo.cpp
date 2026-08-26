#include "project1_camera/myCamera.h"
#include "project2_armor/armor_detector.h"
#include "project2_armor/armor_pose.h"
#include "project2_armor/motion_observer.h"
#include "project3_yolo/yolo_detector.h"
#include "project4_serial/serial_port.h"
#include <opencv2/opencv.hpp>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

// ============================================================================
// 综合示例：把 4 个项目串起来，演示一整套“视觉→测距→跟踪→串口发送”的流程。
//   - 输入：摄像头编号（如 "0"）或图片路径。可用 --yolo 切换 YOLO 检测模式。
//   - 图片模式：只处理一帧并保存结果图。
//   - 摄像头模式：持续读取、处理、显示，并计算移动速度，按 Esc 退出。
//   - 每检测到有效装甲板，就把中心坐标(x,y)以两个 4 字节 float 通过串口发出。
// ============================================================================
int main(int argc, char** argv) {
    // 处理 --help：打印用法后退出。
    if (argc > 1 && std::string(argv[1]) == "--help") { std::cout << "Usage: rm_demo [image_or_camera_index]\n       rm_demo --yolo [image_or_camera_index]\n"; return 0; }
    // 解析参数：use_yolo 表示是否用 YOLO 模式；source_arg 指向实际输入源所在的 argv 下标。
    const bool use_yolo = argc > 1 && std::string(argv[1]) == "--yolo"; const int source_arg = use_yolo ? 2 : 1;
    // source 是输入参数；缺省为 "0"。若全是数字，则视为摄像头编号，否则视为文件/流路径。
    const std::string source = argc > source_arg ? argv[source_arg] : "0"; const bool camera_input = source.find_first_not_of("0123456789") == std::string::npos;
    // 创建相机对象和帧容器。
    myCamera camera; cv::Mat frame;
    // camera_input 为真：打开对应编号摄像头；否则读入图片文件。
    if (camera_input) { if (!camera.open(std::stoi(source))) { std::cerr << "Cannot open camera\n"; return 1; } }
    else { frame = cv::imread(source); if (frame.empty()) { std::cerr << "Cannot read input: " << source << "\n"; return 1; } }
    // 创建 YOLO 检测器与装甲板、运动观测器；仅在 YOLO 模式下加载模型。
    myYoloDetector yolo; if (use_yolo && !yolo.load("best.onnx")) { std::cerr << "Cannot load best.onnx\n"; return 1; }
    // 传统视觉：同时识别红色与蓝色装甲板，各自一套颜色参数。
    // 蓝色色差带光晕，加了 B>200 / R<160 门控(见 armor_detector.cpp)，阈值用 90。
    ArmorConfig red_cfg;   red_cfg.color = ArmorColor::Red;  red_cfg.binary_threshold = 100;
    ArmorConfig blue_cfg;  blue_cfg.color = ArmorColor::Blue; blue_cfg.binary_threshold = 90;
    myArmorDetector red_detector(red_cfg), blue_detector(blue_cfg);
    myMotionObserver observer;

    // 项目 4：默认打开 USB 转 TTL 的第一个常见设备。串口不存在时仍允许视觉功能运行。
    mySerialPort serial;
    constexpr const char* serial_device = "/dev/ttyUSB0";
    if (serial.open(serial_device, 115200)) std::cout << "Serial opened: " << serial_device << " @ 115200\n";
    else std::cerr << "Warning: cannot open " << serial_device << "; serial transmission is disabled\n";

    // 有效载荷为两个连续的 32 位 IEEE-754 float（x、y，单位为像素）；
    // writeFrame 会自动加 0xAA 帧头和 0xBB 帧尾。
    // send_center 负责把装甲板中心坐标打包成 8 字节并写串口。
    auto send_center = [&](cv::Point2f center) {
        if (!serial.isOpen()) return; // 未打开串口则跳过发送。
        const float x = center.x, y = center.y;
        std::vector<uint8_t> payload(sizeof(x) + sizeof(y));
        // 按 x、y 顺序写入，接收端按相同的 32 位浮点字节序解析。
        std::memcpy(payload.data(), &x, sizeof(x));
        std::memcpy(payload.data() + sizeof(x), &y, sizeof(y));
        if (!serial.writeFrame(payload)) std::cerr << "Serial write failed\n";
        else std::cout << "Serial TX: AA + float(center_x=" << x
                      << ", center_y=" << y << ") + BB\n";
    };

    // process 是对单帧图像的处理闭包：检测→测距→跟踪→绘图→发送→左上角信息叠加。
    auto process = [&](cv::Mat& image) {
        if (image.empty()) return;
        // 用图像尺寸构造一个近似内参矩阵（示例用，真实应使用标定结果）；畸变设 0。
        // 注意：这里的 fx=fy=cols 只是占位近似，实际距离会因此不准确。
        cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << image.cols, 0, image.cols / 2.0, 0, image.cols, image.rows / 2.0, 0, 0, 1);
        myArmorPoseEstimator pose(camera_matrix, cv::Mat::zeros(1, 5, CV_64F), image.size()); const double now = cv::getTickCount() / cv::getTickFrequency(); // 当前时间(秒)给观测器用。
        // 信息叠加：把每个识别结果画在画面左上角，逐行往下排。
        int line = 0;
        auto draw_info = [&](const char* color, cv::Point2f center, double dist, float speed) {
            char buf[192];
            // 格式：颜色  (中心x,中心y)  距离xx mm  速度xx px/s
            std::snprintf(buf, sizeof(buf), "%s  (%.0f,%.0f)  dist=%.0fmm  speed=%.0f",
                          color, center.x, center.y, dist, speed);
            cv::putText(image, buf, cv::Point(12, 22 + 28 * line),
                        cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 255), 2, cv::LINE_AA);
            ++line;
        };
        // 统一处理一个装甲板目标：测距→绘图→跟踪→发送→左上角信息。
        auto handle = [&](const ArmorTarget& t, const char* color) {
            auto p = pose.estimate(t); pose.drawAxes(image, p);
            observer.update(t.center, now);
            cv::arrowedLine(image, t.center, observer.predicted(), {0, 255, 255}, 2);
            // 中心坐标来自检测结果本身，不要求 solvePnP 姿态解算成功。
            send_center(t.center);
            draw_info(color, t.center, p.distance, observer.speed());
            std::cout << color << " center=" << t.center << " distance_mm=" << p.distance
                      << " speed_pixel_s=" << observer.speed() << "\n";
        };
        if (use_yolo) {
            // YOLO 模式：对每个检测框，构造一个 ArmorTarget，做测距、绘图、跟踪与发送。
            for (const auto& t : yolo.detect(image)) {
                cv::rectangle(image, t.box, {0,255,0}, 2);
                ArmorTarget a{}; a.box=t.box; a.center={(float)t.box.x+t.box.width/2.0F,(float)t.box.y+t.box.height/2.0F};
                // 颜色显示：类别 0=蓝色 1=红色，其余显示未知。
                const char* color = t.class_id == 0 ? "blue" : (t.class_id == 1 ? "red" : "?");
                handle(a, color);
            }
        } else {
            // 传统视觉：先红色后蓝色，把两类检测结果都处理。
            for (const auto& t : red_detector.detect(image, &image)) handle(t, "red");
            for (const auto& t : blue_detector.detect(image, &image)) handle(t, "blue");
        }
    };
    // 摄像头模式：循环读帧→处理→显示，Esc 退出；图片模式：处理一次并保存结果。
    if (camera_input) { while (camera.read(frame)) { process(frame); cv::imshow("RM Vision", frame); if (cv::waitKey(1)==27) break; } }
    else { process(frame); cv::imwrite("armor_result.jpg", frame); }
    return 0;
}
