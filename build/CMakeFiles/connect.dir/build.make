# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.30

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
CMAKE_COMMAND = /home/budali11/cmake-3.30.0/bin/cmake

# The command to remove a file.
RM = /home/budali11/cmake-3.30.0/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/budali11/workbench/atmel/sam4l-reb215

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/budali11/workbench/atmel/sam4l-reb215/build

# Utility rule file for connect.

# Include any custom commands dependencies for this target.
include CMakeFiles/connect.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/connect.dir/progress.make

CMakeFiles/connect:
	sudo openocd -s /usr/local/share/openocd/scripts -f /usr/local/share/openocd/scripts/board/atmel_sam4l8_xplained_pro.cfg

connect: CMakeFiles/connect
connect: CMakeFiles/connect.dir/build.make
.PHONY : connect

# Rule to build all files generated by this target.
CMakeFiles/connect.dir/build: connect
.PHONY : CMakeFiles/connect.dir/build

CMakeFiles/connect.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/connect.dir/cmake_clean.cmake
.PHONY : CMakeFiles/connect.dir/clean

CMakeFiles/connect.dir/depend:
	cd /home/budali11/workbench/atmel/sam4l-reb215/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/budali11/workbench/atmel/sam4l-reb215 /home/budali11/workbench/atmel/sam4l-reb215 /home/budali11/workbench/atmel/sam4l-reb215/build /home/budali11/workbench/atmel/sam4l-reb215/build /home/budali11/workbench/atmel/sam4l-reb215/build/CMakeFiles/connect.dir/DependInfo.cmake "--color=$(COLOR)"
.PHONY : CMakeFiles/connect.dir/depend
