# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.22

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:

#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:

# Disable VCS-based implicit rules.
% : %,v

# Disable VCS-based implicit rules.
% : RCS/%

# Disable VCS-based implicit rules.
% : RCS/%,v

# Disable VCS-based implicit rules.
% : SCCS/s.%

# Disable VCS-based implicit rules.
% : s.%

.SUFFIXES: .hpux_make_needs_suffix_list

# Produce verbose output by default.
VERBOSE = 1

# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
$(VERBOSE).SILENT:

# A target that is always out of date.
cmake_force:
.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /snap/cmake/1035/bin/cmake

# The command to remove a file.
RM = /snap/cmake/1035/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jesper/Documents/gitz/util/C/isoObject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jesper/Documents/gitz/util/C/isoObject/build

# Include any dependencies generated for this target.
include CMakeFiles/ISO_objectTest.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/ISO_objectTest.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/ISO_objectTest.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/ISO_objectTest.dir/flags.make

CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.o: CMakeFiles/ISO_objectTest.dir/flags.make
CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.o: ../testIsoObject.cpp
CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.o: CMakeFiles/ISO_objectTest.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.o -MF CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.o.d -o CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.o -c /home/jesper/Documents/gitz/util/C/isoObject/testIsoObject.cpp

CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jesper/Documents/gitz/util/C/isoObject/testIsoObject.cpp > CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.i

CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jesper/Documents/gitz/util/C/isoObject/testIsoObject.cpp -o CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.s

# Object files for target ISO_objectTest
ISO_objectTest_OBJECTS = \
"CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.o"

# External object files for target ISO_objectTest
ISO_objectTest_EXTERNAL_OBJECTS =

ISO_objectTest: CMakeFiles/ISO_objectTest.dir/testIsoObject.cpp.o
ISO_objectTest: CMakeFiles/ISO_objectTest.dir/build.make
ISO_objectTest: libISO_object.so
ISO_objectTest: /usr/local/lib/libTCPUDPSocket.so
ISO_objectTest: /usr/local/lib/libISO22133.so
ISO_objectTest: CMakeFiles/ISO_objectTest.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ISO_objectTest"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/ISO_objectTest.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/ISO_objectTest.dir/build: ISO_objectTest
.PHONY : CMakeFiles/ISO_objectTest.dir/build

CMakeFiles/ISO_objectTest.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/ISO_objectTest.dir/cmake_clean.cmake
.PHONY : CMakeFiles/ISO_objectTest.dir/clean

CMakeFiles/ISO_objectTest.dir/depend:
	cd /home/jesper/Documents/gitz/util/C/isoObject/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jesper/Documents/gitz/util/C/isoObject /home/jesper/Documents/gitz/util/C/isoObject /home/jesper/Documents/gitz/util/C/isoObject/build /home/jesper/Documents/gitz/util/C/isoObject/build /home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles/ISO_objectTest.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/ISO_objectTest.dir/depend

