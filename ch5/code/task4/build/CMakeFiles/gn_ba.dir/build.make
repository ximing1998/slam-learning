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
CMAKE_SOURCE_DIR = /home/ximing/slam-code/ch5/task4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/ximing/slam-code/ch5/task4/build

# Include any dependencies generated for this target.
include CMakeFiles/gn_ba.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gn_ba.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gn_ba.dir/flags.make

CMakeFiles/gn_ba.dir/GN-BA.cpp.o: CMakeFiles/gn_ba.dir/flags.make
CMakeFiles/gn_ba.dir/GN-BA.cpp.o: ../GN-BA.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/ximing/slam-code/ch5/task4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/gn_ba.dir/GN-BA.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/gn_ba.dir/GN-BA.cpp.o -c /home/ximing/slam-code/ch5/task4/GN-BA.cpp

CMakeFiles/gn_ba.dir/GN-BA.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gn_ba.dir/GN-BA.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/ximing/slam-code/ch5/task4/GN-BA.cpp > CMakeFiles/gn_ba.dir/GN-BA.cpp.i

CMakeFiles/gn_ba.dir/GN-BA.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gn_ba.dir/GN-BA.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/ximing/slam-code/ch5/task4/GN-BA.cpp -o CMakeFiles/gn_ba.dir/GN-BA.cpp.s

CMakeFiles/gn_ba.dir/GN-BA.cpp.o.requires:

.PHONY : CMakeFiles/gn_ba.dir/GN-BA.cpp.o.requires

CMakeFiles/gn_ba.dir/GN-BA.cpp.o.provides: CMakeFiles/gn_ba.dir/GN-BA.cpp.o.requires
	$(MAKE) -f CMakeFiles/gn_ba.dir/build.make CMakeFiles/gn_ba.dir/GN-BA.cpp.o.provides.build
.PHONY : CMakeFiles/gn_ba.dir/GN-BA.cpp.o.provides

CMakeFiles/gn_ba.dir/GN-BA.cpp.o.provides.build: CMakeFiles/gn_ba.dir/GN-BA.cpp.o


# Object files for target gn_ba
gn_ba_OBJECTS = \
"CMakeFiles/gn_ba.dir/GN-BA.cpp.o"

# External object files for target gn_ba
gn_ba_EXTERNAL_OBJECTS =

gn_ba: CMakeFiles/gn_ba.dir/GN-BA.cpp.o
gn_ba: CMakeFiles/gn_ba.dir/build.make
gn_ba: CMakeFiles/gn_ba.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/ximing/slam-code/ch5/task4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable gn_ba"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gn_ba.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gn_ba.dir/build: gn_ba

.PHONY : CMakeFiles/gn_ba.dir/build

CMakeFiles/gn_ba.dir/requires: CMakeFiles/gn_ba.dir/GN-BA.cpp.o.requires

.PHONY : CMakeFiles/gn_ba.dir/requires

CMakeFiles/gn_ba.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gn_ba.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gn_ba.dir/clean

CMakeFiles/gn_ba.dir/depend:
	cd /home/ximing/slam-code/ch5/task4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/ximing/slam-code/ch5/task4 /home/ximing/slam-code/ch5/task4 /home/ximing/slam-code/ch5/task4/build /home/ximing/slam-code/ch5/task4/build /home/ximing/slam-code/ch5/task4/build/CMakeFiles/gn_ba.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gn_ba.dir/depend

