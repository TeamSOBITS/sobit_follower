# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.7

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/iwami/eigen-eigen-b3f3d4950030

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/iwami/eigen-eigen-b3f3d4950030/build

# Include any dependencies generated for this target.
include test/CMakeFiles/nomalloc_6.dir/depend.make

# Include the progress variables for this target.
include test/CMakeFiles/nomalloc_6.dir/progress.make

# Include the compile flags for this target's objects.
include test/CMakeFiles/nomalloc_6.dir/flags.make

test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o: test/CMakeFiles/nomalloc_6.dir/flags.make
test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o: ../test/nomalloc.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/iwami/eigen-eigen-b3f3d4950030/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o"
	cd /home/iwami/eigen-eigen-b3f3d4950030/build/test && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o -c /home/iwami/eigen-eigen-b3f3d4950030/test/nomalloc.cpp

test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/nomalloc_6.dir/nomalloc.cpp.i"
	cd /home/iwami/eigen-eigen-b3f3d4950030/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/iwami/eigen-eigen-b3f3d4950030/test/nomalloc.cpp > CMakeFiles/nomalloc_6.dir/nomalloc.cpp.i

test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/nomalloc_6.dir/nomalloc.cpp.s"
	cd /home/iwami/eigen-eigen-b3f3d4950030/build/test && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/iwami/eigen-eigen-b3f3d4950030/test/nomalloc.cpp -o CMakeFiles/nomalloc_6.dir/nomalloc.cpp.s

test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o.requires:

.PHONY : test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o.requires

test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o.provides: test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o.requires
	$(MAKE) -f test/CMakeFiles/nomalloc_6.dir/build.make test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o.provides.build
.PHONY : test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o.provides

test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o.provides.build: test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o


# Object files for target nomalloc_6
nomalloc_6_OBJECTS = \
"CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o"

# External object files for target nomalloc_6
nomalloc_6_EXTERNAL_OBJECTS =

test/nomalloc_6: test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o
test/nomalloc_6: test/CMakeFiles/nomalloc_6.dir/build.make
test/nomalloc_6: test/CMakeFiles/nomalloc_6.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/iwami/eigen-eigen-b3f3d4950030/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable nomalloc_6"
	cd /home/iwami/eigen-eigen-b3f3d4950030/build/test && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/nomalloc_6.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
test/CMakeFiles/nomalloc_6.dir/build: test/nomalloc_6

.PHONY : test/CMakeFiles/nomalloc_6.dir/build

test/CMakeFiles/nomalloc_6.dir/requires: test/CMakeFiles/nomalloc_6.dir/nomalloc.cpp.o.requires

.PHONY : test/CMakeFiles/nomalloc_6.dir/requires

test/CMakeFiles/nomalloc_6.dir/clean:
	cd /home/iwami/eigen-eigen-b3f3d4950030/build/test && $(CMAKE_COMMAND) -P CMakeFiles/nomalloc_6.dir/cmake_clean.cmake
.PHONY : test/CMakeFiles/nomalloc_6.dir/clean

test/CMakeFiles/nomalloc_6.dir/depend:
	cd /home/iwami/eigen-eigen-b3f3d4950030/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/iwami/eigen-eigen-b3f3d4950030 /home/iwami/eigen-eigen-b3f3d4950030/test /home/iwami/eigen-eigen-b3f3d4950030/build /home/iwami/eigen-eigen-b3f3d4950030/build/test /home/iwami/eigen-eigen-b3f3d4950030/build/test/CMakeFiles/nomalloc_6.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : test/CMakeFiles/nomalloc_6.dir/depend

