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
CMAKE_SOURCE_DIR = /home/mpheng/workspace/cfr_ws/src/cfr_manager

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mpheng/workspace/cfr_ws/src/build

# Include any dependencies generated for this target.
include CMakeFiles/cfr_manager.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/cfr_manager.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/cfr_manager.dir/flags.make

CMakeFiles/cfr_manager.dir/src/cfr_manager.cpp.o: CMakeFiles/cfr_manager.dir/flags.make
CMakeFiles/cfr_manager.dir/src/cfr_manager.cpp.o: /home/mpheng/workspace/cfr_ws/src/cfr_manager/src/cfr_manager.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mpheng/workspace/cfr_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/cfr_manager.dir/src/cfr_manager.cpp.o"
	/usr/bin/clang++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/cfr_manager.dir/src/cfr_manager.cpp.o -c /home/mpheng/workspace/cfr_ws/src/cfr_manager/src/cfr_manager.cpp

CMakeFiles/cfr_manager.dir/src/cfr_manager.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/cfr_manager.dir/src/cfr_manager.cpp.i"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mpheng/workspace/cfr_ws/src/cfr_manager/src/cfr_manager.cpp > CMakeFiles/cfr_manager.dir/src/cfr_manager.cpp.i

CMakeFiles/cfr_manager.dir/src/cfr_manager.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/cfr_manager.dir/src/cfr_manager.cpp.s"
	/usr/bin/clang++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mpheng/workspace/cfr_ws/src/cfr_manager/src/cfr_manager.cpp -o CMakeFiles/cfr_manager.dir/src/cfr_manager.cpp.s

# Object files for target cfr_manager
cfr_manager_OBJECTS = \
"CMakeFiles/cfr_manager.dir/src/cfr_manager.cpp.o"

# External object files for target cfr_manager
cfr_manager_EXTERNAL_OBJECTS =

libcfr_manager.a: CMakeFiles/cfr_manager.dir/src/cfr_manager.cpp.o
libcfr_manager.a: CMakeFiles/cfr_manager.dir/build.make
libcfr_manager.a: CMakeFiles/cfr_manager.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mpheng/workspace/cfr_ws/src/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libcfr_manager.a"
	$(CMAKE_COMMAND) -P CMakeFiles/cfr_manager.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/cfr_manager.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/cfr_manager.dir/build: libcfr_manager.a

.PHONY : CMakeFiles/cfr_manager.dir/build

CMakeFiles/cfr_manager.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/cfr_manager.dir/cmake_clean.cmake
.PHONY : CMakeFiles/cfr_manager.dir/clean

CMakeFiles/cfr_manager.dir/depend:
	cd /home/mpheng/workspace/cfr_ws/src/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mpheng/workspace/cfr_ws/src/cfr_manager /home/mpheng/workspace/cfr_ws/src/cfr_manager /home/mpheng/workspace/cfr_ws/src/build /home/mpheng/workspace/cfr_ws/src/build /home/mpheng/workspace/cfr_ws/src/build/CMakeFiles/cfr_manager.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/cfr_manager.dir/depend

