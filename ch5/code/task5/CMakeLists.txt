cmake_minimum_required(VERSION 3.0)

project(E2RT)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE "Release")

#添加头文件
include_directories( "/usr/include/eigen3")
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)
#添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})


add_executable(useICP useICP.cpp)

#链接OpenCV库
target_link_libraries(useICP ${Sophus_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS} fmt)

