# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build

# Include any dependencies generated for this target.
include CMakeFiles/gait.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/gait.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/gait.dir/flags.make

CMakeFiles/gait.dir/src/gait.cpp.o: CMakeFiles/gait.dir/flags.make
CMakeFiles/gait.dir/src/gait.cpp.o: ../src/gait.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/gait.dir/src/gait.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gait.dir/src/gait.cpp.o -c /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/gait.cpp

CMakeFiles/gait.dir/src/gait.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gait.dir/src/gait.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/gait.cpp > CMakeFiles/gait.dir/src/gait.cpp.i

CMakeFiles/gait.dir/src/gait.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gait.dir/src/gait.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/gait.cpp -o CMakeFiles/gait.dir/src/gait.cpp.s

CMakeFiles/gait.dir/src/gait.cpp.o.requires:
.PHONY : CMakeFiles/gait.dir/src/gait.cpp.o.requires

CMakeFiles/gait.dir/src/gait.cpp.o.provides: CMakeFiles/gait.dir/src/gait.cpp.o.requires
	$(MAKE) -f CMakeFiles/gait.dir/build.make CMakeFiles/gait.dir/src/gait.cpp.o.provides.build
.PHONY : CMakeFiles/gait.dir/src/gait.cpp.o.provides

CMakeFiles/gait.dir/src/gait.cpp.o.provides.build: CMakeFiles/gait.dir/src/gait.cpp.o

# Object files for target gait
gait_OBJECTS = \
"CMakeFiles/gait.dir/src/gait.cpp.o"

# External object files for target gait
gait_EXTERNAL_OBJECTS =

../devel/lib/libgait.so: CMakeFiles/gait.dir/src/gait.cpp.o
../devel/lib/libgait.so: CMakeFiles/gait.dir/build.make
../devel/lib/libgait.so: CMakeFiles/gait.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library ../devel/lib/libgait.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gait.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/gait.dir/build: ../devel/lib/libgait.so
.PHONY : CMakeFiles/gait.dir/build

CMakeFiles/gait.dir/requires: CMakeFiles/gait.dir/src/gait.cpp.o.requires
.PHONY : CMakeFiles/gait.dir/requires

CMakeFiles/gait.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/gait.dir/cmake_clean.cmake
.PHONY : CMakeFiles/gait.dir/clean

CMakeFiles/gait.dir/depend:
	cd /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build/CMakeFiles/gait.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/gait.dir/depend

