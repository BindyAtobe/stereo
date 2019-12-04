cmake_minimum_required(VERSION 3.15)
project(stereo_match_sgbm)
find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

set(CMAKE_CXX_STANDARD 14)

add_executable(stereo_match_sgbm main.cpp)
target_link_libraries(stereo_match_sgbm ${OpenCV_LIBS})