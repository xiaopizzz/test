# 潜云机器人实验室 DGE 战队视觉组招新任务

安装环境依赖
```bash
sudo apt update                    #更新软件源   
sudo apt install make cmake        #安装编译器
sudo apt install g++               #安装编译器
sudo apt install libopencv-dev     #安装opencv库
```

首次编译
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j$(nproc)
```

后续编译
```bash
cmake --build build -j$(nproc)
```
删除编译
```bash
rm -rf build
```

确认设备名
```bash
ls /dev/video*                     #确认摄像头
ls /dev/ttyUSB* /dev/ttyACM*       #确认串口
```

传统方法模式
```bash
./build/rm_demo 0                  # 使用 0 号摄像头
./build/rm_demo test.jpg           # 使用图片
```

YOLO 模式：
```bash
./build/rm_demo --yolo 0           # 使用 0 号摄像头
./build/rm_demo --yolo test.jpg    # 使用图片
```

# 单独运行项目
项目 1：摄像头
```bash
./build/camera_example
```

项目 2 + 发挥 2、3、4：传统视觉
```bash
./build/rm_demo 0
```

项目 3 + 发挥 3、4：YOLO
```bash
./build/rm_demo --yolo 0
```

项目 4 已经自动被 rm_demo 调用：识别到有效装甲板后会将距离通过 /dev/ttyUSB0 发给单片机。

工程注释
- `project1_camera`: 跨 Windows/Ubuntu 的 `myCamera` 类，支持构造、析构、`read`、参数设置。
- `project2_armor`: OpenCV 传统视觉装甲板检测，支持颜色选择、灯条配对、目标框和中心点输出。
- `project2_armor/armor_pose.*`: 使用 `solvePnP` 解算装甲板距离和 3D 姿态，并绘制 XYZ 坐标轴。
- `project2_armor/motion_observer.*`: alpha-beta 观测器和短时预测器，输出像素速度并绘制运动箭头。
- `project3_yolo`: OpenCV DNN YOLO 推理封装，支持 ONNX、类别文件和置信度/NMS 参数。
- `project4_serial`: `0xAA + payload + 0xBB` 帧协议、32 位整数/浮点打包、流式解码，以及 Ubuntu `termios` USB-TTL 串口收发类。

当前 `best.onnx` 是 2 类 YOLOv8 DFL 检测模型，程序已经按其 `80/40/20` 三个尺度输出进行解码。
注意：yolo训练的模型为`best.pt`文件需要导出为`best.onnx`文件，需要在配置好的yolo环境下进行

在环境下运行
```bash
python3 best.py
```
环境配置链接：https://www.bilibili.com/video/BV182bZzMEYD?t=1.2

程序会在结果图上绘制坐标轴和运动箭头，并在终端打印 `distance_mm`、`speed_pixel_s`。示例中的相机内参是根据图像尺寸生成的近似值；实际使用时应在 `examples/rm_demo.cpp` 中替换为相机标定得到的内参和畸变系数。距离结果还依赖 `armor_pose.h` 中的装甲板实际尺寸，默认是 135 mm x 55 mm。

摄像头模式会持续运行，按 `Esc` 退出；图片模式只处理一帧，因此速度为零或没有明显运动箭头是正常现象。

`rm_demo` 启动时会以 115200 波特率自动打开 `/dev/ttyUSB0`。每检测到一个有效装甲板，程序向串口发送 `0xAA + 4 字节 float 距离(mm) + 0xBB`；终端出现 `Serial TX` 即表示一帧已写入 USB-TTL。接收端必须按相同的 32 位浮点字节序解析。

串口使用示例：`mySerialPort port; port.open("/dev/ttyUSB0", 115200);`。
