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
CMAKE_SOURCE_DIR = /home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/build

# Include any dependencies generated for this target.
include CMakeFiles/Detect.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/Detect.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Detect.dir/flags.make

CMakeFiles/Detect.dir/main.cpp.o: CMakeFiles/Detect.dir/flags.make
CMakeFiles/Detect.dir/main.cpp.o: ../main.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Detect.dir/main.cpp.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/Detect.dir/main.cpp.o -c /home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/main.cpp

CMakeFiles/Detect.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Detect.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/main.cpp > CMakeFiles/Detect.dir/main.cpp.i

CMakeFiles/Detect.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Detect.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/main.cpp -o CMakeFiles/Detect.dir/main.cpp.s

CMakeFiles/Detect.dir/main.cpp.o.requires:

.PHONY : CMakeFiles/Detect.dir/main.cpp.o.requires

CMakeFiles/Detect.dir/main.cpp.o.provides: CMakeFiles/Detect.dir/main.cpp.o.requires
	$(MAKE) -f CMakeFiles/Detect.dir/build.make CMakeFiles/Detect.dir/main.cpp.o.provides.build
.PHONY : CMakeFiles/Detect.dir/main.cpp.o.provides

CMakeFiles/Detect.dir/main.cpp.o.provides.build: CMakeFiles/Detect.dir/main.cpp.o


# Object files for target Detect
Detect_OBJECTS = \
"CMakeFiles/Detect.dir/main.cpp.o"

# External object files for target Detect
Detect_EXTERNAL_OBJECTS =

Detect: CMakeFiles/Detect.dir/main.cpp.o
Detect: CMakeFiles/Detect.dir/build.make
Detect: /usr/local/lib/libopencv_dnn.so.4.4.0
Detect: /usr/local/lib/libopencv_gapi.so.4.4.0
Detect: /usr/local/lib/libopencv_highgui.so.4.4.0
Detect: /usr/local/lib/libopencv_ml.so.4.4.0
Detect: /usr/local/lib/libopencv_objdetect.so.4.4.0
Detect: /usr/local/lib/libopencv_photo.so.4.4.0
Detect: /usr/local/lib/libopencv_stitching.so.4.4.0
Detect: /usr/local/lib/libopencv_video.so.4.4.0
Detect: /usr/local/lib/libopencv_videoio.so.4.4.0
Detect: Class/Energy_Switch/libEnergy_Switch.a
Detect: /usr/local/lib/libopencv_imgcodecs.so.4.4.0
Detect: /usr/local/lib/libopencv_calib3d.so.4.4.0
Detect: /usr/local/lib/libopencv_features2d.so.4.4.0
Detect: /usr/local/lib/libopencv_flann.so.4.4.0
Detect: /usr/local/lib/libopencv_imgproc.so.4.4.0
Detect: /usr/local/lib/libopencv_core.so.4.4.0
Detect: CMakeFiles/Detect.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable Detect"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Detect.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Detect.dir/build: Detect

.PHONY : CMakeFiles/Detect.dir/build

CMakeFiles/Detect.dir/requires: CMakeFiles/Detect.dir/main.cpp.o.requires

.PHONY : CMakeFiles/Detect.dir/requires

CMakeFiles/Detect.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Detect.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Detect.dir/clean

CMakeFiles/Detect.dir/depend:
	cd /home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect /home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect /home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/build /home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/build /home/rosen/桌面/Rosen/RM实习任务/Task1/ES_Detect/build/CMakeFiles/Detect.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Detect.dir/depend

