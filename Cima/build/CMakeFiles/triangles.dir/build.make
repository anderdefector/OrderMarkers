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
CMAKE_SOURCE_DIR = /home/andres/GIT/OrderMarkers/Cima

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/andres/GIT/OrderMarkers/Cima/build

# Include any dependencies generated for this target.
include CMakeFiles/triangles.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/triangles.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/triangles.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/triangles.dir/flags.make

CMakeFiles/triangles.dir/lib/marker/triangles.cc.o: CMakeFiles/triangles.dir/flags.make
CMakeFiles/triangles.dir/lib/marker/triangles.cc.o: ../lib/marker/triangles.cc
CMakeFiles/triangles.dir/lib/marker/triangles.cc.o: CMakeFiles/triangles.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andres/GIT/OrderMarkers/Cima/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/triangles.dir/lib/marker/triangles.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/triangles.dir/lib/marker/triangles.cc.o -MF CMakeFiles/triangles.dir/lib/marker/triangles.cc.o.d -o CMakeFiles/triangles.dir/lib/marker/triangles.cc.o -c /home/andres/GIT/OrderMarkers/Cima/lib/marker/triangles.cc

CMakeFiles/triangles.dir/lib/marker/triangles.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/triangles.dir/lib/marker/triangles.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andres/GIT/OrderMarkers/Cima/lib/marker/triangles.cc > CMakeFiles/triangles.dir/lib/marker/triangles.cc.i

CMakeFiles/triangles.dir/lib/marker/triangles.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/triangles.dir/lib/marker/triangles.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andres/GIT/OrderMarkers/Cima/lib/marker/triangles.cc -o CMakeFiles/triangles.dir/lib/marker/triangles.cc.s

# Object files for target triangles
triangles_OBJECTS = \
"CMakeFiles/triangles.dir/lib/marker/triangles.cc.o"

# External object files for target triangles
triangles_EXTERNAL_OBJECTS =

libtriangles.so: CMakeFiles/triangles.dir/lib/marker/triangles.cc.o
libtriangles.so: CMakeFiles/triangles.dir/build.make
libtriangles.so: CMakeFiles/triangles.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andres/GIT/OrderMarkers/Cima/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library libtriangles.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/triangles.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/triangles.dir/build: libtriangles.so
.PHONY : CMakeFiles/triangles.dir/build

CMakeFiles/triangles.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/triangles.dir/cmake_clean.cmake
.PHONY : CMakeFiles/triangles.dir/clean

CMakeFiles/triangles.dir/depend:
	cd /home/andres/GIT/OrderMarkers/Cima/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andres/GIT/OrderMarkers/Cima /home/andres/GIT/OrderMarkers/Cima /home/andres/GIT/OrderMarkers/Cima/build /home/andres/GIT/OrderMarkers/Cima/build /home/andres/GIT/OrderMarkers/Cima/build/CMakeFiles/triangles.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/triangles.dir/depend

