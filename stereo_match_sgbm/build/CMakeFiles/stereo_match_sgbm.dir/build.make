# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.15

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/local/Cellar/cmake/3.15.4/bin/cmake

# The command to remove a file.
RM = /usr/local/Cellar/cmake/3.15.4/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /Users/wangconghao/Documents/github-stereo/stereo_match_sgbm

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /Users/wangconghao/Documents/github-stereo/stereo_match_sgbm/build

# Include any dependencies generated for this target.
include CMakeFiles/stereo_match_sgbm.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo_match_sgbm.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo_match_sgbm.dir/flags.make

CMakeFiles/stereo_match_sgbm.dir/main.cpp.o: CMakeFiles/stereo_match_sgbm.dir/flags.make
CMakeFiles/stereo_match_sgbm.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/Users/wangconghao/Documents/github-stereo/stereo_match_sgbm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo_match_sgbm.dir/main.cpp.o"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_match_sgbm.dir/main.cpp.o -c /Users/wangconghao/Documents/github-stereo/stereo_match_sgbm/main.cpp

CMakeFiles/stereo_match_sgbm.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_match_sgbm.dir/main.cpp.i"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /Users/wangconghao/Documents/github-stereo/stereo_match_sgbm/main.cpp > CMakeFiles/stereo_match_sgbm.dir/main.cpp.i

CMakeFiles/stereo_match_sgbm.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_match_sgbm.dir/main.cpp.s"
	/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /Users/wangconghao/Documents/github-stereo/stereo_match_sgbm/main.cpp -o CMakeFiles/stereo_match_sgbm.dir/main.cpp.s

# Object files for target stereo_match_sgbm
stereo_match_sgbm_OBJECTS = \
"CMakeFiles/stereo_match_sgbm.dir/main.cpp.o"

# External object files for target stereo_match_sgbm
stereo_match_sgbm_EXTERNAL_OBJECTS =

stereo_match_sgbm: CMakeFiles/stereo_match_sgbm.dir/main.cpp.o
stereo_match_sgbm: CMakeFiles/stereo_match_sgbm.dir/build.make
stereo_match_sgbm: /usr/local/lib/libopencv_gapi.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_stitching.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_aruco.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_bgsegm.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_bioinspired.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_ccalib.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_dnn_objdetect.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_dnn_superres.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_dpm.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_face.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_freetype.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_fuzzy.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_hdf.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_hfs.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_img_hash.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_line_descriptor.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_quality.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_reg.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_rgbd.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_saliency.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_sfm.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_stereo.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_structured_light.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_superres.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_surface_matching.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_tracking.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_videostab.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_xfeatures2d.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_xobjdetect.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_xphoto.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_highgui.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_shape.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_datasets.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_plot.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_text.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_dnn.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_ml.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_phase_unwrapping.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_optflow.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_ximgproc.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_video.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_videoio.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_imgcodecs.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_objdetect.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_calib3d.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_features2d.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_flann.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_photo.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_imgproc.4.1.2.dylib
stereo_match_sgbm: /usr/local/lib/libopencv_core.4.1.2.dylib
stereo_match_sgbm: CMakeFiles/stereo_match_sgbm.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/Users/wangconghao/Documents/github-stereo/stereo_match_sgbm/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable stereo_match_sgbm"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_match_sgbm.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo_match_sgbm.dir/build: stereo_match_sgbm

.PHONY : CMakeFiles/stereo_match_sgbm.dir/build

CMakeFiles/stereo_match_sgbm.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo_match_sgbm.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo_match_sgbm.dir/clean

CMakeFiles/stereo_match_sgbm.dir/depend:
	cd /Users/wangconghao/Documents/github-stereo/stereo_match_sgbm/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /Users/wangconghao/Documents/github-stereo/stereo_match_sgbm /Users/wangconghao/Documents/github-stereo/stereo_match_sgbm /Users/wangconghao/Documents/github-stereo/stereo_match_sgbm/build /Users/wangconghao/Documents/github-stereo/stereo_match_sgbm/build /Users/wangconghao/Documents/github-stereo/stereo_match_sgbm/build/CMakeFiles/stereo_match_sgbm.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereo_match_sgbm.dir/depend

