# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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
CMAKE_SOURCE_DIR = /home/rafael/CSE180/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rafael/CSE180/build

# Include any dependencies generated for this target.
include lab2/CMakeFiles/poselogger.dir/depend.make

# Include the progress variables for this target.
include lab2/CMakeFiles/poselogger.dir/progress.make

# Include the compile flags for this target's objects.
include lab2/CMakeFiles/poselogger.dir/flags.make

lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o: lab2/CMakeFiles/poselogger.dir/flags.make
lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o: /home/rafael/CSE180/src/lab2/src/poselogger.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/rafael/CSE180/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o"
	cd /home/rafael/CSE180/build/lab2 && /usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/poselogger.dir/src/poselogger.cpp.o -c /home/rafael/CSE180/src/lab2/src/poselogger.cpp

lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/poselogger.dir/src/poselogger.cpp.i"
	cd /home/rafael/CSE180/build/lab2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/rafael/CSE180/src/lab2/src/poselogger.cpp > CMakeFiles/poselogger.dir/src/poselogger.cpp.i

lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/poselogger.dir/src/poselogger.cpp.s"
	cd /home/rafael/CSE180/build/lab2 && /usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/rafael/CSE180/src/lab2/src/poselogger.cpp -o CMakeFiles/poselogger.dir/src/poselogger.cpp.s

lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o.requires:
.PHONY : lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o.requires

lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o.provides: lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o.requires
	$(MAKE) -f lab2/CMakeFiles/poselogger.dir/build.make lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o.provides.build
.PHONY : lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o.provides

lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o.provides.build: lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o

# Object files for target poselogger
poselogger_OBJECTS = \
"CMakeFiles/poselogger.dir/src/poselogger.cpp.o"

# External object files for target poselogger
poselogger_EXTERNAL_OBJECTS =

/home/rafael/CSE180/devel/lib/lab2/poselogger: lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o
/home/rafael/CSE180/devel/lib/lab2/poselogger: lab2/CMakeFiles/poselogger.dir/build.make
/home/rafael/CSE180/devel/lib/lab2/poselogger: /opt/ros/indigo/lib/libroscpp.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /usr/lib/x86_64-linux-gnu/libboost_signals.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /opt/ros/indigo/lib/librosconsole.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /opt/ros/indigo/lib/librosconsole_log4cxx.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /opt/ros/indigo/lib/librosconsole_backend_interface.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /usr/lib/liblog4cxx.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /usr/lib/x86_64-linux-gnu/libboost_regex.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /opt/ros/indigo/lib/libroscpp_serialization.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /opt/ros/indigo/lib/librostime.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /opt/ros/indigo/lib/libxmlrpcpp.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /opt/ros/indigo/lib/libcpp_common.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /usr/lib/x86_64-linux-gnu/libboost_system.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /usr/lib/x86_64-linux-gnu/libboost_thread.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /usr/lib/x86_64-linux-gnu/libpthread.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: /usr/lib/x86_64-linux-gnu/libconsole_bridge.so
/home/rafael/CSE180/devel/lib/lab2/poselogger: lab2/CMakeFiles/poselogger.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable /home/rafael/CSE180/devel/lib/lab2/poselogger"
	cd /home/rafael/CSE180/build/lab2 && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/poselogger.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
lab2/CMakeFiles/poselogger.dir/build: /home/rafael/CSE180/devel/lib/lab2/poselogger
.PHONY : lab2/CMakeFiles/poselogger.dir/build

lab2/CMakeFiles/poselogger.dir/requires: lab2/CMakeFiles/poselogger.dir/src/poselogger.cpp.o.requires
.PHONY : lab2/CMakeFiles/poselogger.dir/requires

lab2/CMakeFiles/poselogger.dir/clean:
	cd /home/rafael/CSE180/build/lab2 && $(CMAKE_COMMAND) -P CMakeFiles/poselogger.dir/cmake_clean.cmake
.PHONY : lab2/CMakeFiles/poselogger.dir/clean

lab2/CMakeFiles/poselogger.dir/depend:
	cd /home/rafael/CSE180/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rafael/CSE180/src /home/rafael/CSE180/src/lab2 /home/rafael/CSE180/build /home/rafael/CSE180/build/lab2 /home/rafael/CSE180/build/lab2/CMakeFiles/poselogger.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : lab2/CMakeFiles/poselogger.dir/depend

