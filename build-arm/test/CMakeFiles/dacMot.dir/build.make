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
CMAKE_SOURCE_DIR = /home/reto/projects/einbein

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/reto/projects/einbein/build-arm

# Include any dependencies generated for this target.
include test/CMakeFiles/dacMot.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/dacMot.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/dacMot.dir/flags.make

test/CMakeFiles/dacMot.dir/dacMot.cpp.o: test/CMakeFiles/dacMot.dir/flags.make
test/CMakeFiles/dacMot.dir/dacMot.cpp.o: ../test/dacMot.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object test/CMakeFiles/dacMot.dir/dacMot.cpp.o"
	cd /home/reto/projects/einbein/build-arm/test && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/dacMot.dir/dacMot.cpp.o -c /home/reto/projects/einbein/test/dacMot.cpp

test/CMakeFiles/dacMot.dir/dacMot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/dacMot.dir/dacMot.cpp.i"
	cd /home/reto/projects/einbein/build-arm/test && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/test/dacMot.cpp > CMakeFiles/dacMot.dir/dacMot.cpp.i

test/CMakeFiles/dacMot.dir/dacMot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/dacMot.dir/dacMot.cpp.s"
	cd /home/reto/projects/einbein/build-arm/test && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/test/dacMot.cpp -o CMakeFiles/dacMot.dir/dacMot.cpp.s

test/CMakeFiles/dacMot.dir/dacMot.cpp.o.requires:
.PHONY : test/CMakeFiles/dacMot.dir/dacMot.cpp.o.requires

test/CMakeFiles/dacMot.dir/dacMot.cpp.o.provides: test/CMakeFiles/dacMot.dir/dacMot.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/dacMot.dir/build.make test/CMakeFiles/dacMot.dir/dacMot.cpp.o.provides.build
.PHONY : test/CMakeFiles/dacMot.dir/dacMot.cpp.o.provides

test/CMakeFiles/dacMot.dir/dacMot.cpp.o.provides.build: test/CMakeFiles/dacMot.dir/dacMot.cpp.o

# Object files for target dacMot
dacMot_OBJECTS = \
"CMakeFiles/dacMot.dir/dacMot.cpp.o"

# External object files for target dacMot
dacMot_EXTERNAL_OBJECTS =

test/dacMot: test/CMakeFiles/dacMot.dir/dacMot.cpp.o
test/dacMot: test/CMakeFiles/dacMot.dir/build.make
test/dacMot: test/CMakeFiles/dacMot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable dacMot"
	cd /home/reto/projects/einbein/build-arm/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/dacMot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/dacMot.dir/build: test/dacMot
.PHONY : test/CMakeFiles/dacMot.dir/build

test/CMakeFiles/dacMot.dir/requires: test/CMakeFiles/dacMot.dir/dacMot.cpp.o.requires
.PHONY : test/CMakeFiles/dacMot.dir/requires

test/CMakeFiles/dacMot.dir/clean:
	cd /home/reto/projects/einbein/build-arm/test && $(CMAKE_COMMAND) -P CMakeFiles/dacMot.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/dacMot.dir/clean

test/CMakeFiles/dacMot.dir/depend:
	cd /home/reto/projects/einbein/build-arm && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/reto/projects/einbein /home/reto/projects/einbein/test /home/reto/projects/einbein/build-arm /home/reto/projects/einbein/build-arm/test /home/reto/projects/einbein/build-arm/test/CMakeFiles/dacMot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/dacMot.dir/depend

