# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.20

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/clion/164/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /snap/clion/164/bin/cmake/linux/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/ximing/code/slam_code/ch1/task4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ximing/code/slam_code/ch1/task4/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/sayHello.dir/depend.make
# Include the progress variables for this target.
include CMakeFiles/sayHello.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/sayHello.dir/flags.make

CMakeFiles/sayHello.dir/useHello.cpp.o: CMakeFiles/sayHello.dir/flags.make
CMakeFiles/sayHello.dir/useHello.cpp.o: ../useHello.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ximing/code/slam_code/ch1/task4/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/sayHello.dir/useHello.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/sayHello.dir/useHello.cpp.o -c /home/ximing/code/slam_code/ch1/task4/useHello.cpp

CMakeFiles/sayHello.dir/useHello.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/sayHello.dir/useHello.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ximing/code/slam_code/ch1/task4/useHello.cpp > CMakeFiles/sayHello.dir/useHello.cpp.i

CMakeFiles/sayHello.dir/useHello.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/sayHello.dir/useHello.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ximing/code/slam_code/ch1/task4/useHello.cpp -o CMakeFiles/sayHello.dir/useHello.cpp.s

# Object files for target sayHello
sayHello_OBJECTS = \
"CMakeFiles/sayHello.dir/useHello.cpp.o"

# External object files for target sayHello
sayHello_EXTERNAL_OBJECTS =

sayHello: CMakeFiles/sayHello.dir/useHello.cpp.o
sayHello: CMakeFiles/sayHello.dir/build.make
sayHello: libhello.so
sayHello: CMakeFiles/sayHello.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ximing/code/slam_code/ch1/task4/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable sayHello"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/sayHello.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/sayHello.dir/build: sayHello
.PHONY : CMakeFiles/sayHello.dir/build

CMakeFiles/sayHello.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/sayHello.dir/cmake_clean.cmake
.PHONY : CMakeFiles/sayHello.dir/clean

CMakeFiles/sayHello.dir/depend:
	cd /home/ximing/code/slam_code/ch1/task4/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ximing/code/slam_code/ch1/task4 /home/ximing/code/slam_code/ch1/task4 /home/ximing/code/slam_code/ch1/task4/cmake-build-debug /home/ximing/code/slam_code/ch1/task4/cmake-build-debug /home/ximing/code/slam_code/ch1/task4/cmake-build-debug/CMakeFiles/sayHello.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/sayHello.dir/depend
