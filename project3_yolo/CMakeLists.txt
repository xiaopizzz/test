# 项目 3 的 YOLO 推理库；模型文件和数据集由运行时参数提供。
add_library(my_yolo STATIC yolo_detector.cpp)
target_include_directories(my_yolo PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(my_yolo PUBLIC ${OpenCV_LIBS})
