cmake_minimum_required(VERSION 3.0)

project(TrajectoryError)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(trajectoryError trajectoryError.cpp)
target_link_libraries(trajectoryError ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})
