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
CMAKE_SOURCE_DIR = /home/dejan/projects/einbein

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/dejan/projects/einbein/build

# Include any dependencies generated for this target.
include test/CMakeFiles/digIO.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/digIO.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/digIO.dir/flags.make

test/CMakeFiles/digIO.dir/digIO.cpp.o: test/CMakeFiles/digIO.dir/flags.make
test/CMakeFiles/digIO.dir/digIO.cpp.o: ../test/digIO.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dejan/projects/einbein/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object test/CMakeFiles/digIO.dir/digIO.cpp.o"
	cd /home/dejan/projects/einbein/build/test && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/digIO.dir/digIO.cpp.o -c /home/dejan/projects/einbein/test/digIO.cpp

test/CMakeFiles/digIO.dir/digIO.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/digIO.dir/digIO.cpp.i"
	cd /home/dejan/projects/einbein/build/test && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/dejan/projects/einbein/test/digIO.cpp > CMakeFiles/digIO.dir/digIO.cpp.i

test/CMakeFiles/digIO.dir/digIO.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/digIO.dir/digIO.cpp.s"
	cd /home/dejan/projects/einbein/build/test && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/dejan/projects/einbein/test/digIO.cpp -o CMakeFiles/digIO.dir/digIO.cpp.s

test/CMakeFiles/digIO.dir/digIO.cpp.o.requires:
.PHONY : test/CMakeFiles/digIO.dir/digIO.cpp.o.requires

test/CMakeFiles/digIO.dir/digIO.cpp.o.provides: test/CMakeFiles/digIO.dir/digIO.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/digIO.dir/build.make test/CMakeFiles/digIO.dir/digIO.cpp.o.provides.build
.PHONY : test/CMakeFiles/digIO.dir/digIO.cpp.o.provides

test/CMakeFiles/digIO.dir/digIO.cpp.o.provides.build: test/CMakeFiles/digIO.dir/digIO.cpp.o

# Object files for target digIO
digIO_OBJECTS = \
"CMakeFiles/digIO.dir/digIO.cpp.o"

# External object files for target digIO
digIO_EXTERNAL_OBJECTS =

test/digIO: test/CMakeFiles/digIO.dir/digIO.cpp.o
test/digIO: test/CMakeFiles/digIO.dir/build.make
test/digIO: test/CMakeFiles/digIO.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable digIO"
	cd /home/dejan/projects/einbein/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/digIO.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/digIO.dir/build: test/digIO
.PHONY : test/CMakeFiles/digIO.dir/build

test/CMakeFiles/digIO.dir/requires: test/CMakeFiles/digIO.dir/digIO.cpp.o.requires
.PHONY : test/CMakeFiles/digIO.dir/requires

test/CMakeFiles/digIO.dir/clean:
	cd /home/dejan/projects/einbein/build/test && $(CMAKE_COMMAND) -P CMakeFiles/digIO.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/digIO.dir/clean

test/CMakeFiles/digIO.dir/depend:
	cd /home/dejan/projects/einbein/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dejan/projects/einbein /home/dejan/projects/einbein/test /home/dejan/projects/einbein/build /home/dejan/projects/einbein/build/test /home/dejan/projects/einbein/build/test/CMakeFiles/digIO.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/digIO.dir/depend

