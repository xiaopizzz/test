# 项目 1 独立生成静态库，其他项目通过 target_link_libraries 复用。
add_library(my_camera STATIC myCamera.cpp)
target_include_directories(my_camera PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(my_camera PUBLIC ${OpenCV_LIBS})
