cmake_minimum_required(VERSION 3.0)

project(Fisheye)
set(CMAKE_CXX_FLAGS "-std=c++11")

#寻找opencv库
find_package(OpenCV REQUIRED)
#添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS})

add_executable(fisheye fisheye.cpp)

#链接OpenCV库
target_link_libraries(fisheye ${OpenCV_LIBS})