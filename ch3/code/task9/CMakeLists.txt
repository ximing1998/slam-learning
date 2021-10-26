cmake_minimum_required(VERSION 3.0)

project(DrawTrajectory)

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${Pangolin_INCLUDE_DIRS})
include_directories(${Sophus_INCLUDE_DIRS})

add_executable(draw_trajectory draw_trajectory.cpp)
target_link_libraries(draw_trajectory ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})
