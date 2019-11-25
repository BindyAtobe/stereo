cmake_minimum_required(VERSION 3.15)
project(stereo_calib)
find_package(OpenCV)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 14)

add_executable(stereo_calib main.cpp)
target_link_libraries( stereo_calib ${OpenCV_LIBS} )