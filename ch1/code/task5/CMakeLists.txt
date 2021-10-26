# 指明对cmake的最低版本要求
cmake_minimum_required(VERSION 3.0)
# 定义工程的名字
PROJECT(HELLO)

# 设置用Ｒelease模式编译这个工程
SET(CMAKE_BUILD_TYPE "Release")
# 添加头文件
INCLUDE_DIRECTORIES(./include)
# 构建动态链接库
ADD_LIBRARY(hello SHARED ./src/hello.cpp)

# INSTALL指令　安装动态链接库和头文件
INSTALL(TARGETS hello
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib)
INSTALL(FILES ./include/hello.h DESTINATION include)

# 让工程找到 FindHELLO.cmake模块
SET(CMAKE_MODULE_PATH /home/ximing/code/slam_code/ch1/task4/cmake)
MESSAGE(${CMAKE_MODULE_PATH})

# 使用find_package查找定义的动态链接库
FIND_PACKAGE(Hello REQUIRED)
FIND_PACKAGE(glog REQUIRED)
INCLUDE_DIRECTORIES(${glog_INCLUDE_DIR})
FIND_PACKAGE(gflags REQUIRED)
INCLUDE_DIRECTORIES(${gflags_INCLUDE_DIR})
#FIND_PACKAGE(gtest REQUIRED)
#INCLUDE_DIRECTORIES(${gtest_INCLUDE_DIR})

IF(HELLO_FOUND)
    MESSAGE(${CMAKE_MODULE_PATH})
    ADD_EXECUTABLE(sayHello useHello.cpp)
    # 为Target添加共享库
    TARGET_LINK_LIBRARIES(sayHello hello glog gflags gtest)
ENDIF(HELLO_FOUND)

#ADD_EXECUTABLE(sayHello useHello.cpp)
# 为Target添加共享库

