cmake_minimum_required(VERSION 3.16)
project(rm_vision_tasks LANGUAGES CXX)

# 所有子项目使用 C++17，便于在 Ubuntu 20.04/22.04 上直接构建。

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# OpenCV 模块分别用于图像处理、相机读取、DNN 推理和示例输出。
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio dnn highgui calib3d)

add_subdirectory(project1_camera)
add_subdirectory(project2_armor)
add_subdirectory(project3_yolo)
add_subdirectory(project4_serial)

add_executable(rm_demo examples/rm_demo.cpp)
target_link_libraries(rm_demo PRIVATE my_camera my_armor my_yolo my_serial ${OpenCV_LIBS})
target_include_directories(rm_demo PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(camera_example io/example.cpp)
target_link_libraries(camera_example PRIVATE my_camera ${OpenCV_LIBS})
target_include_directories(camera_example PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})

install(TARGETS rm_demo camera_example RUNTIME DESTINATION bin)
