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
include test/CMakeFiles/fqdMot.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/fqdMot.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/fqdMot.dir/flags.make

test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o: test/CMakeFiles/fqdMot.dir/flags.make
test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o: ../test/fqdMot.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dejan/projects/einbein/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o"
	cd /home/dejan/projects/einbein/build/test && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/fqdMot.dir/fqdMot.cpp.o -c /home/dejan/projects/einbein/test/fqdMot.cpp

test/CMakeFiles/fqdMot.dir/fqdMot.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/fqdMot.dir/fqdMot.cpp.i"
	cd /home/dejan/projects/einbein/build/test && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/dejan/projects/einbein/test/fqdMot.cpp > CMakeFiles/fqdMot.dir/fqdMot.cpp.i

test/CMakeFiles/fqdMot.dir/fqdMot.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/fqdMot.dir/fqdMot.cpp.s"
	cd /home/dejan/projects/einbein/build/test && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/dejan/projects/einbein/test/fqdMot.cpp -o CMakeFiles/fqdMot.dir/fqdMot.cpp.s

test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o.requires:
.PHONY : test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o.requires

test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o.provides: test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/fqdMot.dir/build.make test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o.provides.build
.PHONY : test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o.provides

test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o.provides.build: test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o

# Object files for target fqdMot
fqdMot_OBJECTS = \
"CMakeFiles/fqdMot.dir/fqdMot.cpp.o"

# External object files for target fqdMot
fqdMot_EXTERNAL_OBJECTS =

test/fqdMot: test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o
test/fqdMot: test/CMakeFiles/fqdMot.dir/build.make
test/fqdMot: test/CMakeFiles/fqdMot.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable fqdMot"
	cd /home/dejan/projects/einbein/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/fqdMot.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/fqdMot.dir/build: test/fqdMot
.PHONY : test/CMakeFiles/fqdMot.dir/build

test/CMakeFiles/fqdMot.dir/requires: test/CMakeFiles/fqdMot.dir/fqdMot.cpp.o.requires
.PHONY : test/CMakeFiles/fqdMot.dir/requires

test/CMakeFiles/fqdMot.dir/clean:
	cd /home/dejan/projects/einbein/build/test && $(CMAKE_COMMAND) -P CMakeFiles/fqdMot.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/fqdMot.dir/clean

test/CMakeFiles/fqdMot.dir/depend:
	cd /home/dejan/projects/einbein/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dejan/projects/einbein /home/dejan/projects/einbein/test /home/dejan/projects/einbein/build /home/dejan/projects/einbein/build/test /home/dejan/projects/einbein/build/test/CMakeFiles/fqdMot.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/fqdMot.dir/depend

