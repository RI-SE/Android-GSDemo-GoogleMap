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

# Utility rule file for isoObject_wrap_swig_compilation.

# Include any custom commands dependencies for this target.
include CMakeFiles/isoObject_wrap_swig_compilation.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/isoObject_wrap_swig_compilation.dir/progress.make

CMakeFiles/isoObject_wrap_swig_compilation: CMakeFiles/isoObject_wrap.dir/isoObjectJAVA.stamp

CMakeFiles/isoObject_wrap.dir/isoObjectJAVA.stamp: ../isoObject.i
CMakeFiles/isoObject_wrap.dir/isoObjectJAVA.stamp: ../isoObject.i
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Swig compile isoObject.i for java"
	/snap/cmake/1035/bin/cmake -E make_directory /home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles/isoObject_wrap.dir /home/jesper/Documents/gitz/util/C/isoObject/build /home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles/isoObject_wrap.dir
	/snap/cmake/1035/bin/cmake -E touch /home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA.stamp
	/snap/cmake/1035/bin/cmake -E env SWIG_LIB=/usr/share/swig4.0 /usr/bin/swig4.0 -java -I/home/jesper/Documents/gitz/util/C/isoObject/inc -I/home/jesper/Documents/gitz/util/C/isoObject/sigslot/include/sigslot -I/usr/local/include -I/usr/local/include -I/usr/lib/jvm/default-java/include -I/usr/lib/jvm/default-java/include/linux -I/usr/lib/jvm/default-java/include -outdir /home/jesper/Documents/gitz/util/C/isoObject/build -c++ -o /home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles/isoObject_wrap.dir/isoObjectJAVA_wrap.cxx /home/jesper/Documents/gitz/util/C/isoObject/isoObject.i

isoObject_wrap_swig_compilation: CMakeFiles/isoObject_wrap.dir/isoObjectJAVA.stamp
isoObject_wrap_swig_compilation: CMakeFiles/isoObject_wrap_swig_compilation
isoObject_wrap_swig_compilation: CMakeFiles/isoObject_wrap_swig_compilation.dir/build.make
.PHONY : isoObject_wrap_swig_compilation

# Rule to build all files generated by this target.
CMakeFiles/isoObject_wrap_swig_compilation.dir/build: isoObject_wrap_swig_compilation
.PHONY : CMakeFiles/isoObject_wrap_swig_compilation.dir/build

CMakeFiles/isoObject_wrap_swig_compilation.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/isoObject_wrap_swig_compilation.dir/cmake_clean.cmake
.PHONY : CMakeFiles/isoObject_wrap_swig_compilation.dir/clean

CMakeFiles/isoObject_wrap_swig_compilation.dir/depend:
	cd /home/jesper/Documents/gitz/util/C/isoObject/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/jesper/Documents/gitz/util/C/isoObject /home/jesper/Documents/gitz/util/C/isoObject /home/jesper/Documents/gitz/util/C/isoObject/build /home/jesper/Documents/gitz/util/C/isoObject/build /home/jesper/Documents/gitz/util/C/isoObject/build/CMakeFiles/isoObject_wrap_swig_compilation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/isoObject_wrap_swig_compilation.dir/depend

