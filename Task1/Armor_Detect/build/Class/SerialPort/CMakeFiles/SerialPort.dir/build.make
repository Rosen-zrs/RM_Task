# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.10

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
CMAKE_SOURCE_DIR = /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build

# Include any dependencies generated for this target.
include Class/SerialPort/CMakeFiles/SerialPort.dir/depend.make

# Include the progress variables for this target.
include Class/SerialPort/CMakeFiles/SerialPort.dir/progress.make

# Include the compile flags for this target's objects.
include Class/SerialPort/CMakeFiles/SerialPort.dir/flags.make

Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o: Class/SerialPort/CMakeFiles/SerialPort.dir/flags.make
Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o: ../Class/SerialPort/SerialPort.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o"
	cd /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build/Class/SerialPort && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/SerialPort.dir/SerialPort.cpp.o -c /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/Class/SerialPort/SerialPort.cpp

Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/SerialPort.dir/SerialPort.cpp.i"
	cd /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build/Class/SerialPort && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/Class/SerialPort/SerialPort.cpp > CMakeFiles/SerialPort.dir/SerialPort.cpp.i

Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/SerialPort.dir/SerialPort.cpp.s"
	cd /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build/Class/SerialPort && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/Class/SerialPort/SerialPort.cpp -o CMakeFiles/SerialPort.dir/SerialPort.cpp.s

Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o.requires:

.PHONY : Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o.requires

Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o.provides: Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o.requires
	$(MAKE) -f Class/SerialPort/CMakeFiles/SerialPort.dir/build.make Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o.provides.build
.PHONY : Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o.provides

Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o.provides.build: Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o


# Object files for target SerialPort
SerialPort_OBJECTS = \
"CMakeFiles/SerialPort.dir/SerialPort.cpp.o"

# External object files for target SerialPort
SerialPort_EXTERNAL_OBJECTS =

Class/SerialPort/libSerialPort.a: Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o
Class/SerialPort/libSerialPort.a: Class/SerialPort/CMakeFiles/SerialPort.dir/build.make
Class/SerialPort/libSerialPort.a: Class/SerialPort/CMakeFiles/SerialPort.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX static library libSerialPort.a"
	cd /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build/Class/SerialPort && $(CMAKE_COMMAND) -P CMakeFiles/SerialPort.dir/cmake_clean_target.cmake
	cd /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build/Class/SerialPort && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/SerialPort.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
Class/SerialPort/CMakeFiles/SerialPort.dir/build: Class/SerialPort/libSerialPort.a

.PHONY : Class/SerialPort/CMakeFiles/SerialPort.dir/build

Class/SerialPort/CMakeFiles/SerialPort.dir/requires: Class/SerialPort/CMakeFiles/SerialPort.dir/SerialPort.cpp.o.requires

.PHONY : Class/SerialPort/CMakeFiles/SerialPort.dir/requires

Class/SerialPort/CMakeFiles/SerialPort.dir/clean:
	cd /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build/Class/SerialPort && $(CMAKE_COMMAND) -P CMakeFiles/SerialPort.dir/cmake_clean.cmake
.PHONY : Class/SerialPort/CMakeFiles/SerialPort.dir/clean

Class/SerialPort/CMakeFiles/SerialPort.dir/depend:
	cd /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/Class/SerialPort /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build/Class/SerialPort /home/rosen/桌面/Rosen/RM实习任务/Task1/Armor_Detect/build/Class/SerialPort/CMakeFiles/SerialPort.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : Class/SerialPort/CMakeFiles/SerialPort.dir/depend

