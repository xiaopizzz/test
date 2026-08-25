# 项目 2 的传统视觉算法库，不包含主函数，便于嵌入机器人程序。
add_library(my_armor STATIC armor_detector.cpp armor_pose.cpp motion_observer.cpp)
target_include_directories(my_armor PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(my_armor PUBLIC ${OpenCV_LIBS})
