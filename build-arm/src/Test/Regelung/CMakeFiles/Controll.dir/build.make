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
include src/Test/Regelung/CMakeFiles/Controll.dir/depend.make

# Include the progress variables for this target.
include src/Test/Regelung/CMakeFiles/Controll.dir/progress.make

# Include the compile flags for this target's objects.
include src/Test/Regelung/CMakeFiles/Controll.dir/flags.make

src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o: src/Test/Regelung/CMakeFiles/Controll.dir/flags.make
src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o: ../src/Test/Regelung/mainControll.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controll.dir/mainControll.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/mainControll.cpp

src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controll.dir/mainControll.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/mainControll.cpp > CMakeFiles/Controll.dir/mainControll.cpp.i

src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controll.dir/mainControll.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/mainControll.cpp -o CMakeFiles/Controll.dir/mainControll.cpp.s

src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o.requires:
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o.requires

src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o.provides: src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/CMakeFiles/Controll.dir/build.make src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o.provides.build
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o.provides

src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o.provides.build: src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o

src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o: src/Test/Regelung/CMakeFiles/Controll.dir/flags.make
src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o: ../src/Test/Regelung/CSControll.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controll.dir/CSControll.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/CSControll.cpp

src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controll.dir/CSControll.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/CSControll.cpp > CMakeFiles/Controll.dir/CSControll.cpp.i

src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controll.dir/CSControll.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/CSControll.cpp -o CMakeFiles/Controll.dir/CSControll.cpp.s

src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o.requires:
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o.requires

src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o.provides: src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/CMakeFiles/Controll.dir/build.make src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o.provides.build
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o.provides

src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o.provides.build: src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o

src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o: src/Test/Regelung/CMakeFiles/Controll.dir/flags.make
src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o: ../src/Test/Regelung/Encoder/Encoder.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/Encoder/Encoder.cpp

src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controll.dir/Encoder/Encoder.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/Encoder/Encoder.cpp > CMakeFiles/Controll.dir/Encoder/Encoder.cpp.i

src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controll.dir/Encoder/Encoder.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/Encoder/Encoder.cpp -o CMakeFiles/Controll.dir/Encoder/Encoder.cpp.s

src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o.requires:
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o.requires

src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o.provides: src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/CMakeFiles/Controll.dir/build.make src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o.provides.build
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o.provides

src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o.provides.build: src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o

src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o: src/Test/Regelung/CMakeFiles/Controll.dir/flags.make
src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o: ../src/Test/Regelung/Encoder/TransitionBlockEncoder.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/Encoder/TransitionBlockEncoder.cpp

src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/Encoder/TransitionBlockEncoder.cpp > CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.i

src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/Encoder/TransitionBlockEncoder.cpp -o CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.s

src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o.requires:
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o.requires

src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o.provides: src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/CMakeFiles/Controll.dir/build.make src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o.provides.build
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o.provides

src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o.provides.build: src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o

src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o: src/Test/Regelung/CMakeFiles/Controll.dir/flags.make
src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o: ../src/Test/Regelung/Base2Tool/VorKin.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/Base2Tool/VorKin.cpp

src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/Base2Tool/VorKin.cpp > CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.i

src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/Base2Tool/VorKin.cpp -o CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.s

src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o.requires:
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o.requires

src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o.provides: src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/CMakeFiles/Controll.dir/build.make src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o.provides.build
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o.provides

src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o.provides.build: src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o

src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o: src/Test/Regelung/CMakeFiles/Controll.dir/flags.make
src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o: ../src/Test/Regelung/MotorModell/MotorModell.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/MotorModell/MotorModell.cpp

src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/MotorModell/MotorModell.cpp > CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.i

src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/MotorModell/MotorModell.cpp -o CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.s

src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o.requires:
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o.requires

src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o.provides: src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/CMakeFiles/Controll.dir/build.make src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o.provides.build
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o.provides

src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o.provides.build: src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o

src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o: src/Test/Regelung/CMakeFiles/Controll.dir/flags.make
src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o: ../src/Test/Regelung/MotorModell/I2DAC.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/MotorModell/I2DAC.cpp

src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/MotorModell/I2DAC.cpp > CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.i

src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/MotorModell/I2DAC.cpp -o CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.s

src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o.requires:
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o.requires

src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o.provides: src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/CMakeFiles/Controll.dir/build.make src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o.provides.build
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o.provides

src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o.provides.build: src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o

src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o: src/Test/Regelung/CMakeFiles/Controll.dir/flags.make
src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o: ../src/Test/Regelung/PMotor/PMotor.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/PMotor/PMotor.cpp

src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controll.dir/PMotor/PMotor.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/PMotor/PMotor.cpp > CMakeFiles/Controll.dir/PMotor/PMotor.cpp.i

src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controll.dir/PMotor/PMotor.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/PMotor/PMotor.cpp -o CMakeFiles/Controll.dir/PMotor/PMotor.cpp.s

src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o.requires:
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o.requires

src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o.provides: src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/CMakeFiles/Controll.dir/build.make src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o.provides.build
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o.provides

src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o.provides.build: src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o

src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o: src/Test/Regelung/CMakeFiles/Controll.dir/flags.make
src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o: ../src/Test/Regelung/Trajektorie/Trajektorie.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_9)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/Trajektorie/Trajektorie.cpp

src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/Trajektorie/Trajektorie.cpp > CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.i

src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/Trajektorie/Trajektorie.cpp -o CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.s

src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o.requires:
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o.requires

src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o.provides: src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/CMakeFiles/Controll.dir/build.make src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o.provides.build
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o.provides

src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o.provides.build: src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o

src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o: src/Test/Regelung/CMakeFiles/Controll.dir/flags.make
src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o: ../src/Test/Regelung/PDV/PDV.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_10)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controll.dir/PDV/PDV.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/PDV/PDV.cpp

src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controll.dir/PDV/PDV.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/PDV/PDV.cpp > CMakeFiles/Controll.dir/PDV/PDV.cpp.i

src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controll.dir/PDV/PDV.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/PDV/PDV.cpp -o CMakeFiles/Controll.dir/PDV/PDV.cpp.s

src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o.requires:
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o.requires

src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o.provides: src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/CMakeFiles/Controll.dir/build.make src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o.provides.build
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o.provides

src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o.provides.build: src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o

src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o: src/Test/Regelung/CMakeFiles/Controll.dir/flags.make
src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o: ../src/Test/Regelung/Trajektorie/constInput.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/reto/projects/einbein/build-arm/CMakeFiles $(CMAKE_PROGRESS_11)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o -c /home/reto/projects/einbein/src/Test/Regelung/Trajektorie/constInput.cpp

src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.i"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/reto/projects/einbein/src/Test/Regelung/Trajektorie/constInput.cpp > CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.i

src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.s"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && /usr/bin/arm-linux-gnueabihf-g++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/reto/projects/einbein/src/Test/Regelung/Trajektorie/constInput.cpp -o CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.s

src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o.requires:
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o.requires

src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o.provides: src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o.requires
	$(MAKE) -f src/Test/Regelung/CMakeFiles/Controll.dir/build.make src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o.provides.build
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o.provides

src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o.provides.build: src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o

# Object files for target Controll
Controll_OBJECTS = \
"CMakeFiles/Controll.dir/mainControll.cpp.o" \
"CMakeFiles/Controll.dir/CSControll.cpp.o" \
"CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o" \
"CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o" \
"CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o" \
"CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o" \
"CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o" \
"CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o" \
"CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o" \
"CMakeFiles/Controll.dir/PDV/PDV.cpp.o" \
"CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o"

# External object files for target Controll
Controll_EXTERNAL_OBJECTS =

src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o
src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o
src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o
src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o
src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o
src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o
src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o
src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o
src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o
src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o
src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o
src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/build.make
src/Test/Regelung/Controll: src/Test/Regelung/CMakeFiles/Controll.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable Controll"
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Controll.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/Test/Regelung/CMakeFiles/Controll.dir/build: src/Test/Regelung/Controll
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/build

src/Test/Regelung/CMakeFiles/Controll.dir/requires: src/Test/Regelung/CMakeFiles/Controll.dir/mainControll.cpp.o.requires
src/Test/Regelung/CMakeFiles/Controll.dir/requires: src/Test/Regelung/CMakeFiles/Controll.dir/CSControll.cpp.o.requires
src/Test/Regelung/CMakeFiles/Controll.dir/requires: src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/Encoder.cpp.o.requires
src/Test/Regelung/CMakeFiles/Controll.dir/requires: src/Test/Regelung/CMakeFiles/Controll.dir/Encoder/TransitionBlockEncoder.cpp.o.requires
src/Test/Regelung/CMakeFiles/Controll.dir/requires: src/Test/Regelung/CMakeFiles/Controll.dir/Base2Tool/VorKin.cpp.o.requires
src/Test/Regelung/CMakeFiles/Controll.dir/requires: src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/MotorModell.cpp.o.requires
src/Test/Regelung/CMakeFiles/Controll.dir/requires: src/Test/Regelung/CMakeFiles/Controll.dir/MotorModell/I2DAC.cpp.o.requires
src/Test/Regelung/CMakeFiles/Controll.dir/requires: src/Test/Regelung/CMakeFiles/Controll.dir/PMotor/PMotor.cpp.o.requires
src/Test/Regelung/CMakeFiles/Controll.dir/requires: src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/Trajektorie.cpp.o.requires
src/Test/Regelung/CMakeFiles/Controll.dir/requires: src/Test/Regelung/CMakeFiles/Controll.dir/PDV/PDV.cpp.o.requires
src/Test/Regelung/CMakeFiles/Controll.dir/requires: src/Test/Regelung/CMakeFiles/Controll.dir/Trajektorie/constInput.cpp.o.requires
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/requires

src/Test/Regelung/CMakeFiles/Controll.dir/clean:
	cd /home/reto/projects/einbein/build-arm/src/Test/Regelung && $(CMAKE_COMMAND) -P CMakeFiles/Controll.dir/cmake_clean.cmake
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/clean

src/Test/Regelung/CMakeFiles/Controll.dir/depend:
	cd /home/reto/projects/einbein/build-arm && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/reto/projects/einbein /home/reto/projects/einbein/src/Test/Regelung /home/reto/projects/einbein/build-arm /home/reto/projects/einbein/build-arm/src/Test/Regelung /home/reto/projects/einbein/build-arm/src/Test/Regelung/CMakeFiles/Controll.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/Test/Regelung/CMakeFiles/Controll.dir/depend

