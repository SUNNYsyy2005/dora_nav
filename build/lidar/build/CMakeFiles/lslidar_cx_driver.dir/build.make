# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/xiling/Desktop/1/dora_nav/build/lidar

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/xiling/Desktop/1/dora_nav/build/lidar/build

# Include any dependencies generated for this target.
include CMakeFiles/lslidar_cx_driver.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/lslidar_cx_driver.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/lslidar_cx_driver.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lslidar_cx_driver.dir/flags.make

CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.o: CMakeFiles/lslidar_cx_driver.dir/flags.make
CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.o: /home/xiling/Desktop/1/dora_nav/build/lidar/src/lslidar_driver.cpp
CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.o: CMakeFiles/lslidar_cx_driver.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --progress-dir=/home/xiling/Desktop/1/dora_nav/build/lidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.o -MF CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.o.d -o CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.o -c /home/xiling/Desktop/1/dora_nav/build/lidar/src/lslidar_driver.cpp

CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Preprocessing CXX source to CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/xiling/Desktop/1/dora_nav/build/lidar/src/lslidar_driver.cpp > CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.i

CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green "Compiling CXX source to assembly CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/xiling/Desktop/1/dora_nav/build/lidar/src/lslidar_driver.cpp -o CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.s

# Object files for target lslidar_cx_driver
lslidar_cx_driver_OBJECTS = \
"CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.o"

# External object files for target lslidar_cx_driver
lslidar_cx_driver_EXTERNAL_OBJECTS =

liblslidar_cx_driver.a: CMakeFiles/lslidar_cx_driver.dir/src/lslidar_driver.cpp.o
liblslidar_cx_driver.a: CMakeFiles/lslidar_cx_driver.dir/build.make
liblslidar_cx_driver.a: CMakeFiles/lslidar_cx_driver.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color "--switch=$(COLOR)" --green --bold --progress-dir=/home/xiling/Desktop/1/dora_nav/build/lidar/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library liblslidar_cx_driver.a"
	$(CMAKE_COMMAND) -P CMakeFiles/lslidar_cx_driver.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lslidar_cx_driver.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lslidar_cx_driver.dir/build: liblslidar_cx_driver.a
.PHONY : CMakeFiles/lslidar_cx_driver.dir/build

CMakeFiles/lslidar_cx_driver.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lslidar_cx_driver.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lslidar_cx_driver.dir/clean

CMakeFiles/lslidar_cx_driver.dir/depend:
	cd /home/xiling/Desktop/1/dora_nav/build/lidar/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/xiling/Desktop/1/dora_nav/build/lidar /home/xiling/Desktop/1/dora_nav/build/lidar /home/xiling/Desktop/1/dora_nav/build/lidar/build /home/xiling/Desktop/1/dora_nav/build/lidar/build /home/xiling/Desktop/1/dora_nav/build/lidar/build/CMakeFiles/lslidar_cx_driver.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/lslidar_cx_driver.dir/depend
