# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ximing/slam-code/ch5/task2

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ximing/slam-code/ch5/task2/build

# Include any dependencies generated for this target.
include CMakeFiles/computeORB.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/computeORB.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/computeORB.dir/flags.make

CMakeFiles/computeORB.dir/computeORB.cpp.o: CMakeFiles/computeORB.dir/flags.make
CMakeFiles/computeORB.dir/computeORB.cpp.o: ../computeORB.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ximing/slam-code/ch5/task2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/computeORB.dir/computeORB.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/computeORB.dir/computeORB.cpp.o -c /home/ximing/slam-code/ch5/task2/computeORB.cpp

CMakeFiles/computeORB.dir/computeORB.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/computeORB.dir/computeORB.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ximing/slam-code/ch5/task2/computeORB.cpp > CMakeFiles/computeORB.dir/computeORB.cpp.i

CMakeFiles/computeORB.dir/computeORB.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/computeORB.dir/computeORB.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ximing/slam-code/ch5/task2/computeORB.cpp -o CMakeFiles/computeORB.dir/computeORB.cpp.s

CMakeFiles/computeORB.dir/computeORB.cpp.o.requires:

.PHONY : CMakeFiles/computeORB.dir/computeORB.cpp.o.requires

CMakeFiles/computeORB.dir/computeORB.cpp.o.provides: CMakeFiles/computeORB.dir/computeORB.cpp.o.requires
	$(MAKE) -f CMakeFiles/computeORB.dir/build.make CMakeFiles/computeORB.dir/computeORB.cpp.o.provides.build
.PHONY : CMakeFiles/computeORB.dir/computeORB.cpp.o.provides

CMakeFiles/computeORB.dir/computeORB.cpp.o.provides.build: CMakeFiles/computeORB.dir/computeORB.cpp.o


# Object files for target computeORB
computeORB_OBJECTS = \
"CMakeFiles/computeORB.dir/computeORB.cpp.o"

# External object files for target computeORB
computeORB_EXTERNAL_OBJECTS =

computeORB: CMakeFiles/computeORB.dir/computeORB.cpp.o
computeORB: CMakeFiles/computeORB.dir/build.make
computeORB: /usr/local/lib/libopencv_dnn.so.3.4.16
computeORB: /usr/local/lib/libopencv_highgui.so.3.4.16
computeORB: /usr/local/lib/libopencv_ml.so.3.4.16
computeORB: /usr/local/lib/libopencv_objdetect.so.3.4.16
computeORB: /usr/local/lib/libopencv_shape.so.3.4.16
computeORB: /usr/local/lib/libopencv_stitching.so.3.4.16
computeORB: /usr/local/lib/libopencv_superres.so.3.4.16
computeORB: /usr/local/lib/libopencv_videostab.so.3.4.16
computeORB: /usr/local/lib/libopencv_calib3d.so.3.4.16
computeORB: /usr/local/lib/libopencv_features2d.so.3.4.16
computeORB: /usr/local/lib/libopencv_flann.so.3.4.16
computeORB: /usr/local/lib/libopencv_photo.so.3.4.16
computeORB: /usr/local/lib/libopencv_video.so.3.4.16
computeORB: /usr/local/lib/libopencv_videoio.so.3.4.16
computeORB: /usr/local/lib/libopencv_imgcodecs.so.3.4.16
computeORB: /usr/local/lib/libopencv_imgproc.so.3.4.16
computeORB: /usr/local/lib/libopencv_core.so.3.4.16
computeORB: CMakeFiles/computeORB.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ximing/slam-code/ch5/task2/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable computeORB"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/computeORB.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/computeORB.dir/build: computeORB

.PHONY : CMakeFiles/computeORB.dir/build

CMakeFiles/computeORB.dir/requires: CMakeFiles/computeORB.dir/computeORB.cpp.o.requires

.PHONY : CMakeFiles/computeORB.dir/requires

CMakeFiles/computeORB.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/computeORB.dir/cmake_clean.cmake
.PHONY : CMakeFiles/computeORB.dir/clean

CMakeFiles/computeORB.dir/depend:
	cd /home/ximing/slam-code/ch5/task2/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ximing/slam-code/ch5/task2 /home/ximing/slam-code/ch5/task2 /home/ximing/slam-code/ch5/task2/build /home/ximing/slam-code/ch5/task2/build /home/ximing/slam-code/ch5/task2/build/CMakeFiles/computeORB.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/computeORB.dir/depend

