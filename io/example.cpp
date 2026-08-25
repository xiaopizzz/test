#include "project1_camera/myCamera.h"
#include <opencv2/opencv.hpp>
#include <iostream>

// PDF 要求的项目 1 示例：打开摄像头并持续显示画面，按 ESC 退出。
// 演示 myCamera 的构造、isOpened 检查、read 循环读取以及窗口显示。
int main() {
    // 构造时直接打开 0 号摄像头。
    myCamera camera(0);
    // 打开失败则提示并返回错误码。
    if (!camera.isOpened()) { std::cerr << "Unable to open camera 0\n"; return 1; }
    // 循环读取帧并显示；Esc(ASCII 27)退出。
    cv::Mat frame;
    while (camera.read(frame)) {
        cv::imshow("myCamera", frame);
        if (cv::waitKey(1) == 27) break;
    }
    return 0;
}
