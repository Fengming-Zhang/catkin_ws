# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test

# Include any dependencies generated for this target.
include CMakeFiles/we_can_motor_test.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/we_can_motor_test.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/we_can_motor_test.dir/flags.make

CMakeFiles/we_can_motor_test.dir/arm.cpp.o: CMakeFiles/we_can_motor_test.dir/flags.make
CMakeFiles/we_can_motor_test.dir/arm.cpp.o: arm.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/we_can_motor_test.dir/arm.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/we_can_motor_test.dir/arm.cpp.o -c /home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test/arm.cpp

CMakeFiles/we_can_motor_test.dir/arm.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/we_can_motor_test.dir/arm.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test/arm.cpp > CMakeFiles/we_can_motor_test.dir/arm.cpp.i

CMakeFiles/we_can_motor_test.dir/arm.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/we_can_motor_test.dir/arm.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test/arm.cpp -o CMakeFiles/we_can_motor_test.dir/arm.cpp.s

CMakeFiles/we_can_motor_test.dir/arm.cpp.o.requires:

.PHONY : CMakeFiles/we_can_motor_test.dir/arm.cpp.o.requires

CMakeFiles/we_can_motor_test.dir/arm.cpp.o.provides: CMakeFiles/we_can_motor_test.dir/arm.cpp.o.requires
	$(MAKE) -f CMakeFiles/we_can_motor_test.dir/build.make CMakeFiles/we_can_motor_test.dir/arm.cpp.o.provides.build
.PHONY : CMakeFiles/we_can_motor_test.dir/arm.cpp.o.provides

CMakeFiles/we_can_motor_test.dir/arm.cpp.o.provides.build: CMakeFiles/we_can_motor_test.dir/arm.cpp.o


# Object files for target we_can_motor_test
we_can_motor_test_OBJECTS = \
"CMakeFiles/we_can_motor_test.dir/arm.cpp.o"

# External object files for target we_can_motor_test
we_can_motor_test_EXTERNAL_OBJECTS =

we_can_motor_test: CMakeFiles/we_can_motor_test.dir/arm.cpp.o
we_can_motor_test: CMakeFiles/we_can_motor_test.dir/build.make
we_can_motor_test: CMakeFiles/we_can_motor_test.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable we_can_motor_test"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/we_can_motor_test.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/we_can_motor_test.dir/build: we_can_motor_test

.PHONY : CMakeFiles/we_can_motor_test.dir/build

CMakeFiles/we_can_motor_test.dir/requires: CMakeFiles/we_can_motor_test.dir/arm.cpp.o.requires

.PHONY : CMakeFiles/we_can_motor_test.dir/requires

CMakeFiles/we_can_motor_test.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/we_can_motor_test.dir/cmake_clean.cmake
.PHONY : CMakeFiles/we_can_motor_test.dir/clean

CMakeFiles/we_can_motor_test.dir/depend:
	cd /home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test /home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test /home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test /home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test /home/hardware/catkin_ws/src/kejia_hw_control/we_finger_motor_manager/src/test/CMakeFiles/we_can_motor_test.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/we_can_motor_test.dir/depend

