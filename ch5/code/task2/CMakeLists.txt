cmake_minimum_required(VERSION 3.0)
project(ComputeORB)

#IF(NOT DEFINED ${CMAKE_BUILD_TYPE})
    SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

MESSAGE(STATUS "CMAKE_BUILD_TYPE IS ${CMAKE_BUILD_TYPE}")

SET(CMAKE_CXX_STANDARD 17)

#寻找opencv库
find_package(OpenCV REQUIRED)

#添加头文件
include_directories( ${OpenCV_INCLUDE_DIRS} "/usr/local/include/eigen3" "/usr/local/include/sophus")

SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/OUTPUT)

add_executable(computeORB computeORB.cpp)

#链接OpenCV库 和tbb库
target_link_libraries(computeORB ${OpenCV_LIBS} tbb)
