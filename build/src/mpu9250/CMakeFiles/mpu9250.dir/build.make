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
include src/mpu9250/CMakeFiles/mpu9250.dir/depend.make

# Include the progress variables for this target.
include src/mpu9250/CMakeFiles/mpu9250.dir/progress.make

# Include the compile flags for this target's objects.
include src/mpu9250/CMakeFiles/mpu9250.dir/flags.make

src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o: src/mpu9250/CMakeFiles/mpu9250.dir/flags.make
src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o: ../src/mpu9250/eMPL/inv_mpu.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dejan/projects/einbein/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o   -c /home/dejan/projects/einbein/src/mpu9250/eMPL/inv_mpu.c

src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.i"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/dejan/projects/einbein/src/mpu9250/eMPL/inv_mpu.c > CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.i

src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.s"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/dejan/projects/einbein/src/mpu9250/eMPL/inv_mpu.c -o CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.s

src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o.requires:
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o.requires

src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o.provides: src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o.requires
	$(MAKE) -f src/mpu9250/CMakeFiles/mpu9250.dir/build.make src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o.provides.build
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o.provides

src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o.provides.build: src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o

src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o: src/mpu9250/CMakeFiles/mpu9250.dir/flags.make
src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o: ../src/mpu9250/eMPL/inv_mpu_dmp_motion_driver.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dejan/projects/einbein/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o   -c /home/dejan/projects/einbein/src/mpu9250/eMPL/inv_mpu_dmp_motion_driver.c

src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.i"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/dejan/projects/einbein/src/mpu9250/eMPL/inv_mpu_dmp_motion_driver.c > CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.i

src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.s"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/dejan/projects/einbein/src/mpu9250/eMPL/inv_mpu_dmp_motion_driver.c -o CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.s

src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o.requires:
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o.requires

src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o.provides: src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o.requires
	$(MAKE) -f src/mpu9250/CMakeFiles/mpu9250.dir/build.make src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o.provides.build
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o.provides

src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o.provides.build: src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o

src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o: src/mpu9250/CMakeFiles/mpu9250.dir/flags.make
src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o: ../src/mpu9250/glue/linux_glue.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dejan/projects/einbein/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mpu9250.dir/glue/linux_glue.c.o   -c /home/dejan/projects/einbein/src/mpu9250/glue/linux_glue.c

src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mpu9250.dir/glue/linux_glue.c.i"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/dejan/projects/einbein/src/mpu9250/glue/linux_glue.c > CMakeFiles/mpu9250.dir/glue/linux_glue.c.i

src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mpu9250.dir/glue/linux_glue.c.s"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/dejan/projects/einbein/src/mpu9250/glue/linux_glue.c -o CMakeFiles/mpu9250.dir/glue/linux_glue.c.s

src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o.requires:
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o.requires

src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o.provides: src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o.requires
	$(MAKE) -f src/mpu9250/CMakeFiles/mpu9250.dir/build.make src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o.provides.build
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o.provides

src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o.provides.build: src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o: src/mpu9250/CMakeFiles/mpu9250.dir/flags.make
src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o: ../src/mpu9250/mpu9250/mpu9250.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dejan/projects/einbein/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o   -c /home/dejan/projects/einbein/src/mpu9250/mpu9250/mpu9250.c

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.i"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/dejan/projects/einbein/src/mpu9250/mpu9250/mpu9250.c > CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.i

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.s"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/dejan/projects/einbein/src/mpu9250/mpu9250/mpu9250.c -o CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.s

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o.requires:
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o.requires

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o.provides: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o.requires
	$(MAKE) -f src/mpu9250/CMakeFiles/mpu9250.dir/build.make src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o.provides.build
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o.provides

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o.provides.build: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o: src/mpu9250/CMakeFiles/mpu9250.dir/flags.make
src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o: ../src/mpu9250/mpu9250/quaternion.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dejan/projects/einbein/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o   -c /home/dejan/projects/einbein/src/mpu9250/mpu9250/quaternion.c

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.i"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/dejan/projects/einbein/src/mpu9250/mpu9250/quaternion.c > CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.i

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.s"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/dejan/projects/einbein/src/mpu9250/mpu9250/quaternion.c -o CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.s

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o.requires:
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o.requires

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o.provides: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o.requires
	$(MAKE) -f src/mpu9250/CMakeFiles/mpu9250.dir/build.make src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o.provides.build
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o.provides

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o.provides.build: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o: src/mpu9250/CMakeFiles/mpu9250.dir/flags.make
src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o: ../src/mpu9250/mpu9250/vector3d.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dejan/projects/einbein/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o   -c /home/dejan/projects/einbein/src/mpu9250/mpu9250/vector3d.c

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.i"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/dejan/projects/einbein/src/mpu9250/mpu9250/vector3d.c > CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.i

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.s"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/dejan/projects/einbein/src/mpu9250/mpu9250/vector3d.c -o CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.s

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o.requires:
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o.requires

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o.provides: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o.requires
	$(MAKE) -f src/mpu9250/CMakeFiles/mpu9250.dir/build.make src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o.provides.build
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o.provides

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o.provides.build: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o: src/mpu9250/CMakeFiles/mpu9250.dir/flags.make
src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o: ../src/mpu9250/mpu9250/MahonyAHRS.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/dejan/projects/einbein/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o   -c /home/dejan/projects/einbein/src/mpu9250/mpu9250/MahonyAHRS.c

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.i"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -E /home/dejan/projects/einbein/src/mpu9250/mpu9250/MahonyAHRS.c > CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.i

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.s"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && /usr/bin/arm-linux-gnueabihf-gcc  $(C_DEFINES) $(C_FLAGS) -S /home/dejan/projects/einbein/src/mpu9250/mpu9250/MahonyAHRS.c -o CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.s

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o.requires:
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o.requires

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o.provides: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o.requires
	$(MAKE) -f src/mpu9250/CMakeFiles/mpu9250.dir/build.make src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o.provides.build
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o.provides

src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o.provides.build: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o

# Object files for target mpu9250
mpu9250_OBJECTS = \
"CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o" \
"CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o" \
"CMakeFiles/mpu9250.dir/glue/linux_glue.c.o" \
"CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o" \
"CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o" \
"CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o" \
"CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o"

# External object files for target mpu9250
mpu9250_EXTERNAL_OBJECTS =

src/mpu9250/libmpu9250.a: src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o
src/mpu9250/libmpu9250.a: src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o
src/mpu9250/libmpu9250.a: src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o
src/mpu9250/libmpu9250.a: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o
src/mpu9250/libmpu9250.a: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o
src/mpu9250/libmpu9250.a: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o
src/mpu9250/libmpu9250.a: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o
src/mpu9250/libmpu9250.a: src/mpu9250/CMakeFiles/mpu9250.dir/build.make
src/mpu9250/libmpu9250.a: src/mpu9250/CMakeFiles/mpu9250.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking C static library libmpu9250.a"
	cd /home/dejan/projects/einbein/build/src/mpu9250 && $(CMAKE_COMMAND) -P CMakeFiles/mpu9250.dir/cmake_clean_target.cmake
	cd /home/dejan/projects/einbein/build/src/mpu9250 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/mpu9250.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/mpu9250/CMakeFiles/mpu9250.dir/build: src/mpu9250/libmpu9250.a
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/build

src/mpu9250/CMakeFiles/mpu9250.dir/requires: src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu.c.o.requires
src/mpu9250/CMakeFiles/mpu9250.dir/requires: src/mpu9250/CMakeFiles/mpu9250.dir/eMPL/inv_mpu_dmp_motion_driver.c.o.requires
src/mpu9250/CMakeFiles/mpu9250.dir/requires: src/mpu9250/CMakeFiles/mpu9250.dir/glue/linux_glue.c.o.requires
src/mpu9250/CMakeFiles/mpu9250.dir/requires: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/mpu9250.c.o.requires
src/mpu9250/CMakeFiles/mpu9250.dir/requires: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/quaternion.c.o.requires
src/mpu9250/CMakeFiles/mpu9250.dir/requires: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/vector3d.c.o.requires
src/mpu9250/CMakeFiles/mpu9250.dir/requires: src/mpu9250/CMakeFiles/mpu9250.dir/mpu9250/MahonyAHRS.c.o.requires
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/requires

src/mpu9250/CMakeFiles/mpu9250.dir/clean:
	cd /home/dejan/projects/einbein/build/src/mpu9250 && $(CMAKE_COMMAND) -P CMakeFiles/mpu9250.dir/cmake_clean.cmake
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/clean

src/mpu9250/CMakeFiles/mpu9250.dir/depend:
	cd /home/dejan/projects/einbein/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/dejan/projects/einbein /home/dejan/projects/einbein/src/mpu9250 /home/dejan/projects/einbein/build /home/dejan/projects/einbein/build/src/mpu9250 /home/dejan/projects/einbein/build/src/mpu9250/CMakeFiles/mpu9250.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/mpu9250/CMakeFiles/mpu9250.dir/depend

