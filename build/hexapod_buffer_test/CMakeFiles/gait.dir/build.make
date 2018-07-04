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
CMAKE_SOURCE_DIR = /home/quan/hexapod_buffer_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/quan/hexapod_buffer_ws/build

# Include any dependencies generated for this target.
include hexapod_buffer_test/CMakeFiles/gait.dir/depend.make

# Include the progress variables for this target.
include hexapod_buffer_test/CMakeFiles/gait.dir/progress.make

# Include the compile flags for this target's objects.
include hexapod_buffer_test/CMakeFiles/gait.dir/flags.make

hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o: hexapod_buffer_test/CMakeFiles/gait.dir/flags.make
hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o: /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/gait.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/quan/hexapod_buffer_ws/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o"
	cd /home/quan/hexapod_buffer_ws/build/hexapod_buffer_test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/gait.dir/src/gait.cpp.o -c /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/gait.cpp

hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/gait.dir/src/gait.cpp.i"
	cd /home/quan/hexapod_buffer_ws/build/hexapod_buffer_test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/gait.cpp > CMakeFiles/gait.dir/src/gait.cpp.i

hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/gait.dir/src/gait.cpp.s"
	cd /home/quan/hexapod_buffer_ws/build/hexapod_buffer_test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/gait.cpp -o CMakeFiles/gait.dir/src/gait.cpp.s

hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o.requires:
.PHONY : hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o.requires

hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o.provides: hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o.requires
	$(MAKE) -f hexapod_buffer_test/CMakeFiles/gait.dir/build.make hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o.provides.build
.PHONY : hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o.provides

hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o.provides.build: hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o

# Object files for target gait
gait_OBJECTS = \
"CMakeFiles/gait.dir/src/gait.cpp.o"

# External object files for target gait
gait_EXTERNAL_OBJECTS =

/home/quan/hexapod_buffer_ws/devel/lib/libgait.so: hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o
/home/quan/hexapod_buffer_ws/devel/lib/libgait.so: hexapod_buffer_test/CMakeFiles/gait.dir/build.make
/home/quan/hexapod_buffer_ws/devel/lib/libgait.so: hexapod_buffer_test/CMakeFiles/gait.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX shared library /home/quan/hexapod_buffer_ws/devel/lib/libgait.so"
	cd /home/quan/hexapod_buffer_ws/build/hexapod_buffer_test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/gait.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
hexapod_buffer_test/CMakeFiles/gait.dir/build: /home/quan/hexapod_buffer_ws/devel/lib/libgait.so
.PHONY : hexapod_buffer_test/CMakeFiles/gait.dir/build

hexapod_buffer_test/CMakeFiles/gait.dir/requires: hexapod_buffer_test/CMakeFiles/gait.dir/src/gait.cpp.o.requires
.PHONY : hexapod_buffer_test/CMakeFiles/gait.dir/requires

hexapod_buffer_test/CMakeFiles/gait.dir/clean:
	cd /home/quan/hexapod_buffer_ws/build/hexapod_buffer_test && $(CMAKE_COMMAND) -P CMakeFiles/gait.dir/cmake_clean.cmake
.PHONY : hexapod_buffer_test/CMakeFiles/gait.dir/clean

hexapod_buffer_test/CMakeFiles/gait.dir/depend:
	cd /home/quan/hexapod_buffer_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/quan/hexapod_buffer_ws/src /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test /home/quan/hexapod_buffer_ws/build /home/quan/hexapod_buffer_ws/build/hexapod_buffer_test /home/quan/hexapod_buffer_ws/build/hexapod_buffer_test/CMakeFiles/gait.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : hexapod_buffer_test/CMakeFiles/gait.dir/depend
