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
CMAKE_SOURCE_DIR = /home/p3zz/Documents/uni/robot-planning-project/map

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/p3zz/Documents/uni/robot-planning-project/build/map

# Include any dependencies generated for this target.
include CMakeFiles/test_map.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/test_map.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/test_map.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/test_map.dir/flags.make

CMakeFiles/test_map.dir/test/performance.cpp.o: CMakeFiles/test_map.dir/flags.make
CMakeFiles/test_map.dir/test/performance.cpp.o: /home/p3zz/Documents/uni/robot-planning-project/map/test/performance.cpp
CMakeFiles/test_map.dir/test/performance.cpp.o: CMakeFiles/test_map.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/p3zz/Documents/uni/robot-planning-project/build/map/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/test_map.dir/test/performance.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_map.dir/test/performance.cpp.o -MF CMakeFiles/test_map.dir/test/performance.cpp.o.d -o CMakeFiles/test_map.dir/test/performance.cpp.o -c /home/p3zz/Documents/uni/robot-planning-project/map/test/performance.cpp

CMakeFiles/test_map.dir/test/performance.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_map.dir/test/performance.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/p3zz/Documents/uni/robot-planning-project/map/test/performance.cpp > CMakeFiles/test_map.dir/test/performance.cpp.i

CMakeFiles/test_map.dir/test/performance.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_map.dir/test/performance.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/p3zz/Documents/uni/robot-planning-project/map/test/performance.cpp -o CMakeFiles/test_map.dir/test/performance.cpp.s

CMakeFiles/test_map.dir/test/main.cpp.o: CMakeFiles/test_map.dir/flags.make
CMakeFiles/test_map.dir/test/main.cpp.o: /home/p3zz/Documents/uni/robot-planning-project/map/test/main.cpp
CMakeFiles/test_map.dir/test/main.cpp.o: CMakeFiles/test_map.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/p3zz/Documents/uni/robot-planning-project/build/map/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/test_map.dir/test/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/test_map.dir/test/main.cpp.o -MF CMakeFiles/test_map.dir/test/main.cpp.o.d -o CMakeFiles/test_map.dir/test/main.cpp.o -c /home/p3zz/Documents/uni/robot-planning-project/map/test/main.cpp

CMakeFiles/test_map.dir/test/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/test_map.dir/test/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/p3zz/Documents/uni/robot-planning-project/map/test/main.cpp > CMakeFiles/test_map.dir/test/main.cpp.i

CMakeFiles/test_map.dir/test/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/test_map.dir/test/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/p3zz/Documents/uni/robot-planning-project/map/test/main.cpp -o CMakeFiles/test_map.dir/test/main.cpp.s

# Object files for target test_map
test_map_OBJECTS = \
"CMakeFiles/test_map.dir/test/performance.cpp.o" \
"CMakeFiles/test_map.dir/test/main.cpp.o"

# External object files for target test_map
test_map_EXTERNAL_OBJECTS =

test_map: CMakeFiles/test_map.dir/test/performance.cpp.o
test_map: CMakeFiles/test_map.dir/test/main.cpp.o
test_map: CMakeFiles/test_map.dir/build.make
test_map: gtest/libgtest_main.a
test_map: gtest/libgtest.a
test_map: libmap.a
test_map: /home/p3zz/Documents/uni/robot-planning-project/install/dubins/lib/libdubins.a
test_map: /home/p3zz/Documents/uni/robot-planning-project/install/shapes/lib/libshapes.a
test_map: /home/p3zz/Documents/uni/robot-planning-project/install/utils/lib/libutils.a
test_map: CMakeFiles/test_map.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/p3zz/Documents/uni/robot-planning-project/build/map/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX executable test_map"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/test_map.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/test_map.dir/build: test_map
.PHONY : CMakeFiles/test_map.dir/build

CMakeFiles/test_map.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/test_map.dir/cmake_clean.cmake
.PHONY : CMakeFiles/test_map.dir/clean

CMakeFiles/test_map.dir/depend:
	cd /home/p3zz/Documents/uni/robot-planning-project/build/map && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/p3zz/Documents/uni/robot-planning-project/map /home/p3zz/Documents/uni/robot-planning-project/map /home/p3zz/Documents/uni/robot-planning-project/build/map /home/p3zz/Documents/uni/robot-planning-project/build/map /home/p3zz/Documents/uni/robot-planning-project/build/map/CMakeFiles/test_map.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/test_map.dir/depend

