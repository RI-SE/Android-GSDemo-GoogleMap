# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.16

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Produce verbose output by default.
VERBOSE = 1

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
CMAKE_SOURCE_DIR = /home/konglobemeralt/Documents/gitz/util/C/isoObject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/konglobemeralt/Documents/gitz/util/C/isoObject/build

# Include any dependencies generated for this target.
include sockets/CMakeFiles/TEST_CAN.dir/depend.make

# Include the progress variables for this target.
include sockets/CMakeFiles/TEST_CAN.dir/progress.make

# Include the compile flags for this target's objects.
include sockets/CMakeFiles/TEST_CAN.dir/flags.make

sockets/CMakeFiles/TEST_CAN.dir/test_can.cpp.o: sockets/CMakeFiles/TEST_CAN.dir/flags.make
sockets/CMakeFiles/TEST_CAN.dir/test_can.cpp.o: /home/konglobemeralt/Documents/gitz/util/C/sockets/test_can.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/konglobemeralt/Documents/gitz/util/C/isoObject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object sockets/CMakeFiles/TEST_CAN.dir/test_can.cpp.o"
	cd /home/konglobemeralt/Documents/gitz/util/C/isoObject/build/sockets && /home/konglobemeralt/Android/Sdk/ndk/android-ndk-r21e/toolchains/llvm/prebuilt/linux-x86_64/bin/clang++ --target=aarch64-none-linux-android21  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/TEST_CAN.dir/test_can.cpp.o -c /home/konglobemeralt/Documents/gitz/util/C/sockets/test_can.cpp

sockets/CMakeFiles/TEST_CAN.dir/test_can.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/TEST_CAN.dir/test_can.cpp.i"
	cd /home/konglobemeralt/Documents/gitz/util/C/isoObject/build/sockets && /home/konglobemeralt/Android/Sdk/ndk/android-ndk-r21e/toolchains/llvm/prebuilt/linux-x86_64/bin/clang++ --target=aarch64-none-linux-android21 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/konglobemeralt/Documents/gitz/util/C/sockets/test_can.cpp > CMakeFiles/TEST_CAN.dir/test_can.cpp.i

sockets/CMakeFiles/TEST_CAN.dir/test_can.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/TEST_CAN.dir/test_can.cpp.s"
	cd /home/konglobemeralt/Documents/gitz/util/C/isoObject/build/sockets && /home/konglobemeralt/Android/Sdk/ndk/android-ndk-r21e/toolchains/llvm/prebuilt/linux-x86_64/bin/clang++ --target=aarch64-none-linux-android21 $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/konglobemeralt/Documents/gitz/util/C/sockets/test_can.cpp -o CMakeFiles/TEST_CAN.dir/test_can.cpp.s

# Object files for target TEST_CAN
TEST_CAN_OBJECTS = \
"CMakeFiles/TEST_CAN.dir/test_can.cpp.o"

# External object files for target TEST_CAN
TEST_CAN_EXTERNAL_OBJECTS =

sockets/TEST_CAN: sockets/CMakeFiles/TEST_CAN.dir/test_can.cpp.o
sockets/TEST_CAN: sockets/CMakeFiles/TEST_CAN.dir/build.make
sockets/TEST_CAN: sockets/libTCPUDPSocket.so
sockets/TEST_CAN: sockets/CMakeFiles/TEST_CAN.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/konglobemeralt/Documents/gitz/util/C/isoObject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable TEST_CAN"
	cd /home/konglobemeralt/Documents/gitz/util/C/isoObject/build/sockets && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/TEST_CAN.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
sockets/CMakeFiles/TEST_CAN.dir/build: sockets/TEST_CAN

.PHONY : sockets/CMakeFiles/TEST_CAN.dir/build

sockets/CMakeFiles/TEST_CAN.dir/clean:
	cd /home/konglobemeralt/Documents/gitz/util/C/isoObject/build/sockets && $(CMAKE_COMMAND) -P CMakeFiles/TEST_CAN.dir/cmake_clean.cmake
.PHONY : sockets/CMakeFiles/TEST_CAN.dir/clean

sockets/CMakeFiles/TEST_CAN.dir/depend:
	cd /home/konglobemeralt/Documents/gitz/util/C/isoObject/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/konglobemeralt/Documents/gitz/util/C/isoObject /home/konglobemeralt/Documents/gitz/util/C/sockets /home/konglobemeralt/Documents/gitz/util/C/isoObject/build /home/konglobemeralt/Documents/gitz/util/C/isoObject/build/sockets /home/konglobemeralt/Documents/gitz/util/C/isoObject/build/sockets/CMakeFiles/TEST_CAN.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : sockets/CMakeFiles/TEST_CAN.dir/depend
