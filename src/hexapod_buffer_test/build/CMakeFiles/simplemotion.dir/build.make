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
include CMakeFiles/simplemotion.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/simplemotion.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/simplemotion.dir/flags.make

CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o: CMakeFiles/simplemotion.dir/flags.make
CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o: ../src/simplemotion_library/bufferedmotion.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o   -c /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/bufferedmotion.c

CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/bufferedmotion.c > CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.i

CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/bufferedmotion.c -o CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.s

CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o.requires:
.PHONY : CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o.requires

CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o.provides: CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o.requires
	$(MAKE) -f CMakeFiles/simplemotion.dir/build.make CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o.provides.build
.PHONY : CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o.provides

CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o.provides.build: CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o

CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o: CMakeFiles/simplemotion.dir/flags.make
CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o: ../src/simplemotion_library/busdevice.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o   -c /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/busdevice.c

CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/busdevice.c > CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.i

CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/busdevice.c -o CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.s

CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o.requires:
.PHONY : CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o.requires

CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o.provides: CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o.requires
	$(MAKE) -f CMakeFiles/simplemotion.dir/build.make CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o.provides.build
.PHONY : CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o.provides

CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o.provides.build: CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o

CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o: CMakeFiles/simplemotion.dir/flags.make
CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o: ../src/simplemotion_library/rs232.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o   -c /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/rs232.c

CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/rs232.c > CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.i

CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/rs232.c -o CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.s

CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o.requires:
.PHONY : CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o.requires

CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o.provides: CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o.requires
	$(MAKE) -f CMakeFiles/simplemotion.dir/build.make CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o.provides.build
.PHONY : CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o.provides

CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o.provides.build: CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o

CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o: CMakeFiles/simplemotion.dir/flags.make
CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o: ../src/simplemotion_library/simplemotion.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o   -c /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/simplemotion.c

CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/simplemotion.c > CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.i

CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/simplemotion.c -o CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.s

CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o.requires:
.PHONY : CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o.requires

CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o.provides: CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o.requires
	$(MAKE) -f CMakeFiles/simplemotion.dir/build.make CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o.provides.build
.PHONY : CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o.provides

CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o.provides.build: CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o

CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o: CMakeFiles/simplemotion.dir/flags.make
CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o: ../src/simplemotion_library/smtest.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o   -c /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/smtest.c

CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/smtest.c > CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.i

CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/smtest.c -o CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.s

CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o.requires:
.PHONY : CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o.requires

CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o.provides: CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o.requires
	$(MAKE) -f CMakeFiles/simplemotion.dir/build.make CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o.provides.build
.PHONY : CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o.provides

CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o.provides.build: CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o

CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o: CMakeFiles/simplemotion.dir/flags.make
CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o: ../src/simplemotion_library/sm_consts.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o   -c /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/sm_consts.c

CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/sm_consts.c > CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.i

CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/src/simplemotion_library/sm_consts.c -o CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.s

CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o.requires:
.PHONY : CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o.requires

CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o.provides: CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o.requires
	$(MAKE) -f CMakeFiles/simplemotion.dir/build.make CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o.provides.build
.PHONY : CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o.provides

CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o.provides.build: CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o

# Object files for target simplemotion
simplemotion_OBJECTS = \
"CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o" \
"CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o" \
"CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o" \
"CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o" \
"CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o" \
"CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o"

# External object files for target simplemotion
simplemotion_EXTERNAL_OBJECTS =

../devel/lib/libsimplemotion.so: CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o
../devel/lib/libsimplemotion.so: CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o
../devel/lib/libsimplemotion.so: CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o
../devel/lib/libsimplemotion.so: CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o
../devel/lib/libsimplemotion.so: CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o
../devel/lib/libsimplemotion.so: CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o
../devel/lib/libsimplemotion.so: CMakeFiles/simplemotion.dir/build.make
../devel/lib/libsimplemotion.so: CMakeFiles/simplemotion.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C shared library ../devel/lib/libsimplemotion.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/simplemotion.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/simplemotion.dir/build: ../devel/lib/libsimplemotion.so
.PHONY : CMakeFiles/simplemotion.dir/build

CMakeFiles/simplemotion.dir/requires: CMakeFiles/simplemotion.dir/src/simplemotion_library/bufferedmotion.c.o.requires
CMakeFiles/simplemotion.dir/requires: CMakeFiles/simplemotion.dir/src/simplemotion_library/busdevice.c.o.requires
CMakeFiles/simplemotion.dir/requires: CMakeFiles/simplemotion.dir/src/simplemotion_library/rs232.c.o.requires
CMakeFiles/simplemotion.dir/requires: CMakeFiles/simplemotion.dir/src/simplemotion_library/simplemotion.c.o.requires
CMakeFiles/simplemotion.dir/requires: CMakeFiles/simplemotion.dir/src/simplemotion_library/smtest.c.o.requires
CMakeFiles/simplemotion.dir/requires: CMakeFiles/simplemotion.dir/src/simplemotion_library/sm_consts.c.o.requires
.PHONY : CMakeFiles/simplemotion.dir/requires

CMakeFiles/simplemotion.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/simplemotion.dir/cmake_clean.cmake
.PHONY : CMakeFiles/simplemotion.dir/clean

CMakeFiles/simplemotion.dir/depend:
	cd /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build /home/quan/hexapod_buffer_ws/src/hexapod_buffer_test/build/CMakeFiles/simplemotion.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/simplemotion.dir/depend

