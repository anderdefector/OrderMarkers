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
include CMakeFiles/lambda.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/lambda.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/lambda.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/lambda.dir/flags.make

CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.o: CMakeFiles/lambda.dir/flags.make
CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.o: ../lib/marker/lambdaMatrix.cc
CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.o: CMakeFiles/lambda.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/andres/GIT/OrderMarkers/Cima/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.o -MF CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.o.d -o CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.o -c /home/andres/GIT/OrderMarkers/Cima/lib/marker/lambdaMatrix.cc

CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/andres/GIT/OrderMarkers/Cima/lib/marker/lambdaMatrix.cc > CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.i

CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/andres/GIT/OrderMarkers/Cima/lib/marker/lambdaMatrix.cc -o CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.s

# Object files for target lambda
lambda_OBJECTS = \
"CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.o"

# External object files for target lambda
lambda_EXTERNAL_OBJECTS =

liblambda.so: CMakeFiles/lambda.dir/lib/marker/lambdaMatrix.cc.o
liblambda.so: CMakeFiles/lambda.dir/build.make
liblambda.so: CMakeFiles/lambda.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/andres/GIT/OrderMarkers/Cima/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX shared library liblambda.so"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/lambda.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/lambda.dir/build: liblambda.so
.PHONY : CMakeFiles/lambda.dir/build

CMakeFiles/lambda.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/lambda.dir/cmake_clean.cmake
.PHONY : CMakeFiles/lambda.dir/clean

CMakeFiles/lambda.dir/depend:
	cd /home/andres/GIT/OrderMarkers/Cima/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/andres/GIT/OrderMarkers/Cima /home/andres/GIT/OrderMarkers/Cima /home/andres/GIT/OrderMarkers/Cima/build /home/andres/GIT/OrderMarkers/Cima/build /home/andres/GIT/OrderMarkers/Cima/build/CMakeFiles/lambda.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/lambda.dir/depend

