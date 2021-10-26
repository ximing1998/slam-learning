cmake_minimum_required(VERSION 3.0)

project(Disparity)
set(CMAKE_CXX_FLAGS "-std=c++11")

#寻找Pangolin库
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)

#添加头文件
include_directories("/usr/include/eigen3")
include_directories( ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})

add_executable(disparity disparity.cpp)

#链接OpenCV库
target_link_libraries(disparity ${OpenCV_LIBS} ${Pangolin_LIBRARIES})