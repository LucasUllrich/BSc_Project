# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.11

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
CMAKE_SOURCE_DIR = /home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display

# Include any dependencies generated for this target.
include CMakeFiles/LoadAndDisplay.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/LoadAndDisplay.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/LoadAndDisplay.dir/flags.make

CMakeFiles/LoadAndDisplay.dir/LoadAndDisplay.cpp.o: CMakeFiles/LoadAndDisplay.dir/flags.make
CMakeFiles/LoadAndDisplay.dir/LoadAndDisplay.cpp.o: LoadAndDisplay.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/LoadAndDisplay.dir/LoadAndDisplay.cpp.o"
	/usr/lib64/ccache/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/LoadAndDisplay.dir/LoadAndDisplay.cpp.o -c /home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display/LoadAndDisplay.cpp

CMakeFiles/LoadAndDisplay.dir/LoadAndDisplay.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/LoadAndDisplay.dir/LoadAndDisplay.cpp.i"
	/usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display/LoadAndDisplay.cpp > CMakeFiles/LoadAndDisplay.dir/LoadAndDisplay.cpp.i

CMakeFiles/LoadAndDisplay.dir/LoadAndDisplay.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/LoadAndDisplay.dir/LoadAndDisplay.cpp.s"
	/usr/lib64/ccache/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display/LoadAndDisplay.cpp -o CMakeFiles/LoadAndDisplay.dir/LoadAndDisplay.cpp.s

# Object files for target LoadAndDisplay
LoadAndDisplay_OBJECTS = \
"CMakeFiles/LoadAndDisplay.dir/LoadAndDisplay.cpp.o"

# External object files for target LoadAndDisplay
LoadAndDisplay_EXTERNAL_OBJECTS =

LoadAndDisplay: CMakeFiles/LoadAndDisplay.dir/LoadAndDisplay.cpp.o
LoadAndDisplay: CMakeFiles/LoadAndDisplay.dir/build.make
LoadAndDisplay: /usr/local/lib64/libopencv_dnn.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_ml.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_objdetect.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_shape.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_stitching.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_superres.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_videostab.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_calib3d.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_features2d.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_flann.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_highgui.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_photo.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_video.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_videoio.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_imgcodecs.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_imgproc.so.3.4.3
LoadAndDisplay: /usr/local/lib64/libopencv_core.so.3.4.3
LoadAndDisplay: CMakeFiles/LoadAndDisplay.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable LoadAndDisplay"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/LoadAndDisplay.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/LoadAndDisplay.dir/build: LoadAndDisplay

.PHONY : CMakeFiles/LoadAndDisplay.dir/build

CMakeFiles/LoadAndDisplay.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/LoadAndDisplay.dir/cmake_clean.cmake
.PHONY : CMakeFiles/LoadAndDisplay.dir/clean

CMakeFiles/LoadAndDisplay.dir/depend:
	cd /home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display /home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display /home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display /home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display /home/lucasullrich/Documents/Studium/5_Semester/BSc_Arbeit/openCV_demo/01_load-and-display/CMakeFiles/LoadAndDisplay.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/LoadAndDisplay.dir/depend
