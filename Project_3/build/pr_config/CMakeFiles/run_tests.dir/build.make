# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.19

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
CMAKE_COMMAND = /home/yasin/cmake-install/bin/cmake

# The command to remove a file.
RM = /home/yasin/cmake-install/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/yasin/EE451/Project_3/src/pr_config

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yasin/EE451/Project_3/build/pr_config

# Utility rule file for run_tests.

# Include the progress variables for this target.
include CMakeFiles/run_tests.dir/progress.make

run_tests: CMakeFiles/run_tests.dir/build.make

.PHONY : run_tests

# Rule to build all files generated by this target.
CMakeFiles/run_tests.dir/build: run_tests

.PHONY : CMakeFiles/run_tests.dir/build

CMakeFiles/run_tests.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/run_tests.dir/cmake_clean.cmake
.PHONY : CMakeFiles/run_tests.dir/clean

CMakeFiles/run_tests.dir/depend:
	cd /home/yasin/EE451/Project_3/build/pr_config && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yasin/EE451/Project_3/src/pr_config /home/yasin/EE451/Project_3/src/pr_config /home/yasin/EE451/Project_3/build/pr_config /home/yasin/EE451/Project_3/build/pr_config /home/yasin/EE451/Project_3/build/pr_config/CMakeFiles/run_tests.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/run_tests.dir/depend

