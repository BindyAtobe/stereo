cmake_minimum_required(VERSION 3.15)
project(myCalibrate_undistort)
find_package(OpenCV)
include_directories( ${OpenCV_INCLUDE_DIRS} )

set(CMAKE_CXX_STANDARD 14)

add_executable(myCalibrate_undistort main.cpp myCalibration.h myCalibration.cpp
               copyFromOpenCV/calib3d_c_api.h copyFromOpenCV/distortion_model.hpp copyFromOpenCV/calibration.cpp)
target_link_libraries( myCalibrate_undistort ${OpenCV_LIBS} )