cmake_minimum_required(VERSION 3.14)
project(calibrate_undistort)
find_package(OpenCV)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 14)

add_executable(calibrate_undistort main.cpp)
target_link_libraries( calibrate_undistort ${OpenCV_LIBS} )