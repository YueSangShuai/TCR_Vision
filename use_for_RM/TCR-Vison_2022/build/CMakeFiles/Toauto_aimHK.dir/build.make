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
CMAKE_SOURCE_DIR = /home/yuesang/Project/CLionProjects/Toauto_aimHK

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/yuesang/Project/CLionProjects/Toauto_aimHK/build

# Include any dependencies generated for this target.
include CMakeFiles/Toauto_aimHK.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include CMakeFiles/Toauto_aimHK.dir/compiler_depend.make

# Include the progress variables for this target.
include CMakeFiles/Toauto_aimHK.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/Toauto_aimHK.dir/flags.make

CMakeFiles/Toauto_aimHK.dir/main.cpp.o: CMakeFiles/Toauto_aimHK.dir/flags.make
CMakeFiles/Toauto_aimHK.dir/main.cpp.o: ../main.cpp
CMakeFiles/Toauto_aimHK.dir/main.cpp.o: CMakeFiles/Toauto_aimHK.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuesang/Project/CLionProjects/Toauto_aimHK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/Toauto_aimHK.dir/main.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Toauto_aimHK.dir/main.cpp.o -MF CMakeFiles/Toauto_aimHK.dir/main.cpp.o.d -o CMakeFiles/Toauto_aimHK.dir/main.cpp.o -c /home/yuesang/Project/CLionProjects/Toauto_aimHK/main.cpp

CMakeFiles/Toauto_aimHK.dir/main.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Toauto_aimHK.dir/main.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuesang/Project/CLionProjects/Toauto_aimHK/main.cpp > CMakeFiles/Toauto_aimHK.dir/main.cpp.i

CMakeFiles/Toauto_aimHK.dir/main.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Toauto_aimHK.dir/main.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuesang/Project/CLionProjects/Toauto_aimHK/main.cpp -o CMakeFiles/Toauto_aimHK.dir/main.cpp.s

CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.o: CMakeFiles/Toauto_aimHK.dir/flags.make
CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.o: ../Thread/MyThread.cpp
CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.o: CMakeFiles/Toauto_aimHK.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuesang/Project/CLionProjects/Toauto_aimHK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.o -MF CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.o.d -o CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.o -c /home/yuesang/Project/CLionProjects/Toauto_aimHK/Thread/MyThread.cpp

CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuesang/Project/CLionProjects/Toauto_aimHK/Thread/MyThread.cpp > CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.i

CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuesang/Project/CLionProjects/Toauto_aimHK/Thread/MyThread.cpp -o CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.s

CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.o: CMakeFiles/Toauto_aimHK.dir/flags.make
CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.o: ../SerialPort/SerialPort.cpp
CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.o: CMakeFiles/Toauto_aimHK.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuesang/Project/CLionProjects/Toauto_aimHK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.o -MF CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.o.d -o CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.o -c /home/yuesang/Project/CLionProjects/Toauto_aimHK/SerialPort/SerialPort.cpp

CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuesang/Project/CLionProjects/Toauto_aimHK/SerialPort/SerialPort.cpp > CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.i

CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuesang/Project/CLionProjects/Toauto_aimHK/SerialPort/SerialPort.cpp -o CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.s

CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.o: CMakeFiles/Toauto_aimHK.dir/flags.make
CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.o: ../HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp
CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.o: CMakeFiles/Toauto_aimHK.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuesang/Project/CLionProjects/Toauto_aimHK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.o -MF CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.o.d -o CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.o -c /home/yuesang/Project/CLionProjects/Toauto_aimHK/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp

CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuesang/Project/CLionProjects/Toauto_aimHK/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp > CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.i

CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuesang/Project/CLionProjects/Toauto_aimHK/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp -o CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.s

CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.o: CMakeFiles/Toauto_aimHK.dir/flags.make
CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.o: ../HaikangCamera/CameraGray/HaiKangCameraGray.cpp
CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.o: CMakeFiles/Toauto_aimHK.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuesang/Project/CLionProjects/Toauto_aimHK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.o -MF CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.o.d -o CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.o -c /home/yuesang/Project/CLionProjects/Toauto_aimHK/HaikangCamera/CameraGray/HaiKangCameraGray.cpp

CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuesang/Project/CLionProjects/Toauto_aimHK/HaikangCamera/CameraGray/HaiKangCameraGray.cpp > CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.i

CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuesang/Project/CLionProjects/Toauto_aimHK/HaikangCamera/CameraGray/HaiKangCameraGray.cpp -o CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.s

CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.o: CMakeFiles/Toauto_aimHK.dir/flags.make
CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.o: ../AutoShoot/TRT/TRTModule.cpp
CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.o: CMakeFiles/Toauto_aimHK.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuesang/Project/CLionProjects/Toauto_aimHK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.o -MF CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.o.d -o CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.o -c /home/yuesang/Project/CLionProjects/Toauto_aimHK/AutoShoot/TRT/TRTModule.cpp

CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuesang/Project/CLionProjects/Toauto_aimHK/AutoShoot/TRT/TRTModule.cpp > CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.i

CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuesang/Project/CLionProjects/Toauto_aimHK/AutoShoot/TRT/TRTModule.cpp -o CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.s

CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.o: CMakeFiles/Toauto_aimHK.dir/flags.make
CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.o: ../Analyze_data/Analyze_Data.cpp
CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.o: CMakeFiles/Toauto_aimHK.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuesang/Project/CLionProjects/Toauto_aimHK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.o -MF CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.o.d -o CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.o -c /home/yuesang/Project/CLionProjects/Toauto_aimHK/Analyze_data/Analyze_Data.cpp

CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuesang/Project/CLionProjects/Toauto_aimHK/Analyze_data/Analyze_Data.cpp > CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.i

CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuesang/Project/CLionProjects/Toauto_aimHK/Analyze_data/Analyze_Data.cpp -o CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.s

CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.o: CMakeFiles/Toauto_aimHK.dir/flags.make
CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.o: ../PNP_Distance/PNP_Distance.cpp
CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.o: CMakeFiles/Toauto_aimHK.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuesang/Project/CLionProjects/Toauto_aimHK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.o -MF CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.o.d -o CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.o -c /home/yuesang/Project/CLionProjects/Toauto_aimHK/PNP_Distance/PNP_Distance.cpp

CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuesang/Project/CLionProjects/Toauto_aimHK/PNP_Distance/PNP_Distance.cpp > CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.i

CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuesang/Project/CLionProjects/Toauto_aimHK/PNP_Distance/PNP_Distance.cpp -o CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.s

CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.o: CMakeFiles/Toauto_aimHK.dir/flags.make
CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.o: ../Kalman/Kalman.cpp
CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.o: CMakeFiles/Toauto_aimHK.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/yuesang/Project/CLionProjects/Toauto_aimHK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Building CXX object CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.o"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.o -MF CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.o.d -o CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.o -c /home/yuesang/Project/CLionProjects/Toauto_aimHK/Kalman/Kalman.cpp

CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/yuesang/Project/CLionProjects/Toauto_aimHK/Kalman/Kalman.cpp > CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.i

CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/yuesang/Project/CLionProjects/Toauto_aimHK/Kalman/Kalman.cpp -o CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.s

# Object files for target Toauto_aimHK
Toauto_aimHK_OBJECTS = \
"CMakeFiles/Toauto_aimHK.dir/main.cpp.o" \
"CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.o" \
"CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.o" \
"CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.o" \
"CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.o" \
"CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.o" \
"CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.o" \
"CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.o" \
"CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.o"

# External object files for target Toauto_aimHK
Toauto_aimHK_EXTERNAL_OBJECTS =

Toauto_aimHK: CMakeFiles/Toauto_aimHK.dir/main.cpp.o
Toauto_aimHK: CMakeFiles/Toauto_aimHK.dir/Thread/MyThread.cpp.o
Toauto_aimHK: CMakeFiles/Toauto_aimHK.dir/SerialPort/SerialPort.cpp.o
Toauto_aimHK: CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraRGB/HaiKangCameraRGB.cpp.o
Toauto_aimHK: CMakeFiles/Toauto_aimHK.dir/HaikangCamera/CameraGray/HaiKangCameraGray.cpp.o
Toauto_aimHK: CMakeFiles/Toauto_aimHK.dir/AutoShoot/TRT/TRTModule.cpp.o
Toauto_aimHK: CMakeFiles/Toauto_aimHK.dir/Analyze_data/Analyze_Data.cpp.o
Toauto_aimHK: CMakeFiles/Toauto_aimHK.dir/PNP_Distance/PNP_Distance.cpp.o
Toauto_aimHK: CMakeFiles/Toauto_aimHK.dir/Kalman/Kalman.cpp.o
Toauto_aimHK: CMakeFiles/Toauto_aimHK.dir/build.make
Toauto_aimHK: /usr/local/cuda/lib64/libcudart_static.a
Toauto_aimHK: /usr/lib/x86_64-linux-gnu/librt.a
Toauto_aimHK: /opt/MVS/lib/64/libMvCameraControl.so
Toauto_aimHK: /usr/local/lib/libopencv_gapi.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_stitching.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_alphamat.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_aruco.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_barcode.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_bgsegm.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_bioinspired.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_ccalib.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_cudabgsegm.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_cudafeatures2d.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_cudaobjdetect.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_cudastereo.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_dnn_objdetect.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_dnn_superres.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_dpm.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_face.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_freetype.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_fuzzy.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_hdf.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_hfs.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_img_hash.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_intensity_transform.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_line_descriptor.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_mcc.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_quality.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_rapid.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_reg.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_rgbd.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_saliency.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_stereo.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_structured_light.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_superres.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_surface_matching.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_tracking.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_videostab.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_viz.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_wechat_qrcode.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_xfeatures2d.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_xobjdetect.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_xphoto.so.4.5.5
Toauto_aimHK: /usr/local/lib/libfmt.a
Toauto_aimHK: /usr/local/lib/libyaml-cpp.a
Toauto_aimHK: /usr/local/cuda/lib64/libcudart_static.a
Toauto_aimHK: /usr/lib/x86_64-linux-gnu/librt.a
Toauto_aimHK: /opt/MVS/lib/64/libMvCameraControl.so
Toauto_aimHK: /usr/local/lib/libopencv_shape.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_cudalegacy.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_highgui.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_datasets.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_plot.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_text.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_ml.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_phase_unwrapping.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_cudacodec.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_optflow.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_ximgproc.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_video.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_videoio.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_cudawarping.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_imgcodecs.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_objdetect.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_calib3d.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_dnn.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_features2d.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_flann.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_photo.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_cudaimgproc.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_cudafilters.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_imgproc.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_cudaarithm.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_core.so.4.5.5
Toauto_aimHK: /usr/local/lib/libopencv_cudev.so.4.5.5
Toauto_aimHK: /home/yuesang/miniconda3/lib/libpython3.9.so
Toauto_aimHK: /usr/local/lib/libfmt.a
Toauto_aimHK: CMakeFiles/Toauto_aimHK.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/yuesang/Project/CLionProjects/Toauto_aimHK/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_10) "Linking CXX executable Toauto_aimHK"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/Toauto_aimHK.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/Toauto_aimHK.dir/build: Toauto_aimHK
.PHONY : CMakeFiles/Toauto_aimHK.dir/build

CMakeFiles/Toauto_aimHK.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/Toauto_aimHK.dir/cmake_clean.cmake
.PHONY : CMakeFiles/Toauto_aimHK.dir/clean

CMakeFiles/Toauto_aimHK.dir/depend:
	cd /home/yuesang/Project/CLionProjects/Toauto_aimHK/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/yuesang/Project/CLionProjects/Toauto_aimHK /home/yuesang/Project/CLionProjects/Toauto_aimHK /home/yuesang/Project/CLionProjects/Toauto_aimHK/build /home/yuesang/Project/CLionProjects/Toauto_aimHK/build /home/yuesang/Project/CLionProjects/Toauto_aimHK/build/CMakeFiles/Toauto_aimHK.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/Toauto_aimHK.dir/depend

