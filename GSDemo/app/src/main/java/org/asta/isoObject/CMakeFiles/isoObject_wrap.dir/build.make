# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.25

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
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/jesper/Documents/gitz/util/C/isoObject

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/jesper/Documents/gitz/util/C/isoObject/build

# Include any dependencies generated for this target.
include CMakeFiles/isoObject_wrap.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/isoObject_wrap.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/isoObject_wrap.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/isoObject_wrap.dir/flags.make

CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.o: CMakeFiles/isoObject_wrap.dir/flags.make
CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.o: CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx
CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.o: CMakeFiles/isoObject_wrap.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.o -MF CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.o.d -o CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.o -c /home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx

CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx > CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.i

CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx -o CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.s

# Object files for target isoObject_wrap
isoObject_wrap_OBJECTS = \
"CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.o"

# External object files for target isoObject_wrap
isoObject_wrap_EXTERNAL_OBJECTS =

libisoObject_wrap.so: CMakeFiles/isoObject_wrap.dir/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx.o
libisoObject_wrap.so: CMakeFiles/isoObject_wrap.dir/build.make
libisoObject_wrap.so: libISO_object.so
libisoObject_wrap.so: iso22133/libISO22133.so
libisoObject_wrap.so: /usr/lib/jvm/default-java/lib/libjawt.so
libisoObject_wrap.so: /usr/lib/jvm/default-java/lib/server/libjvm.so
libisoObject_wrap.so: CMakeFiles/isoObject_wrap.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libisoObject_wrap.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/isoObject_wrap.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/isoObject_wrap.dir/build: libisoObject_wrap.so
.PHONY : CMakeFiles/isoObject_wrap.dir/build

CMakeFiles/isoObject_wrap.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/isoObject_wrap.dir/cmake_clean.cmake
.PHONY : CMakeFiles/isoObject_wrap.dir/clean

CMakeFiles/isoObject_wrap.dir/depend:
	cd /home/jesper/Documents/gitz/util/C/isoObject/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jesper/Documents/gitz/util/C/isoObject /home/jesper/Documents/gitz/util/C/isoObject /home/jesper/Documents/gitz/util/C/isoObject/build /home/jesper/Documents/gitz/util/C/isoObject/build /home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles/isoObject_wrap.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/isoObject_wrap.dir/depend

