cmake_minimum_required(VERSION 3.0)
project(gaussNewton)
set(CMAKE_CXX_FLAGS "-std=c++11")

#添加库
include_directories("/usr/include/eigen3")

#寻找opencv库
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS} )

#添加可执行文件
#链接OpenCV库
add_executable(gaussNewton gaussnewton.cpp)
target_link_libraries(gaussNewton ${OpenCV_LIBS})