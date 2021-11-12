cmake_minimum_required(VERSION 3.0)

project(E2RT)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")

#添加头文件
include_directories( "/usr/include/eigen3")
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})


add_executable(gn_ba GN-BA.cpp)

#链接OpenCV库
target_link_libraries(gn_ba ${Sophus_LIBRARIES} fmt)

