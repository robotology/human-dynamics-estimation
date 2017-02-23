# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

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
CMAKE_SOURCE_DIR = /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build

# Include any dependencies generated for this target.
include CMakeFiles/yarpModuleLib.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/yarpModuleLib.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/yarpModuleLib.dir/flags.make

autogenerated/data/thrift_XsensFrame_thrift.cmake: ../thrift/XsensFrame.thrift
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Generating autogenerated/data/thrift_XsensFrame_thrift.cmake, autogenerated/data/include/thrift/Vector3.h, autogenerated/data/include/thrift/Vector4.h, autogenerated/data/include/thrift/XsensSegmentData.h, autogenerated/data/include/thrift/XsensSensorData.h, autogenerated/data/include/thrift/XsensFrame.h, autogenerated/data/src/Vector3.cpp, autogenerated/data/src/Vector4.cpp, autogenerated/data/src/XsensSegmentData.cpp, autogenerated/data/src/XsensSensorData.cpp, autogenerated/data/src/XsensFrame.cpp"
	cd /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge && /home/mlorenzini/gitHub/install/bin/yarpidl_thrift --out /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/_yarp_idl_/thrift/xsensframe --gen yarp:include_prefix --I /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge thrift/XsensFrame.thrift
	cd /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge && /usr/bin/cmake -P /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/_yarp_idl_/thrift/xsensframe/placeXsensFrame.cmake

autogenerated/data/include/thrift/Vector3.h: autogenerated/data/thrift_XsensFrame_thrift.cmake
	@$(CMAKE_COMMAND) -E touch_nocreate autogenerated/data/include/thrift/Vector3.h

autogenerated/data/include/thrift/Vector4.h: autogenerated/data/thrift_XsensFrame_thrift.cmake
	@$(CMAKE_COMMAND) -E touch_nocreate autogenerated/data/include/thrift/Vector4.h

autogenerated/data/include/thrift/XsensSegmentData.h: autogenerated/data/thrift_XsensFrame_thrift.cmake
	@$(CMAKE_COMMAND) -E touch_nocreate autogenerated/data/include/thrift/XsensSegmentData.h

autogenerated/data/include/thrift/XsensSensorData.h: autogenerated/data/thrift_XsensFrame_thrift.cmake
	@$(CMAKE_COMMAND) -E touch_nocreate autogenerated/data/include/thrift/XsensSensorData.h

autogenerated/data/include/thrift/XsensFrame.h: autogenerated/data/thrift_XsensFrame_thrift.cmake
	@$(CMAKE_COMMAND) -E touch_nocreate autogenerated/data/include/thrift/XsensFrame.h

autogenerated/data/src/Vector3.cpp: autogenerated/data/thrift_XsensFrame_thrift.cmake
	@$(CMAKE_COMMAND) -E touch_nocreate autogenerated/data/src/Vector3.cpp

autogenerated/data/src/Vector4.cpp: autogenerated/data/thrift_XsensFrame_thrift.cmake
	@$(CMAKE_COMMAND) -E touch_nocreate autogenerated/data/src/Vector4.cpp

autogenerated/data/src/XsensSegmentData.cpp: autogenerated/data/thrift_XsensFrame_thrift.cmake
	@$(CMAKE_COMMAND) -E touch_nocreate autogenerated/data/src/XsensSegmentData.cpp

autogenerated/data/src/XsensSensorData.cpp: autogenerated/data/thrift_XsensFrame_thrift.cmake
	@$(CMAKE_COMMAND) -E touch_nocreate autogenerated/data/src/XsensSensorData.cpp

autogenerated/data/src/XsensFrame.cpp: autogenerated/data/thrift_XsensFrame_thrift.cmake
	@$(CMAKE_COMMAND) -E touch_nocreate autogenerated/data/src/XsensFrame.cpp

autogenerated/services/thrift_XsensDriverService_thrift.cmake: ../thrift/XsensDriverService.thrift
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --blue --bold --progress-dir=/home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Generating autogenerated/services/thrift_XsensDriverService_thrift.cmake, autogenerated/services/include/thrift/XsensDriverService.h, autogenerated/services/src/XsensDriverService.cpp"
	cd /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge && /home/mlorenzini/gitHub/install/bin/yarpidl_thrift --out /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/_yarp_idl_/thrift/xsensdriverservice --gen yarp:include_prefix --I /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge thrift/XsensDriverService.thrift
	cd /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge && /usr/bin/cmake -P /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/_yarp_idl_/thrift/xsensdriverservice/placeXsensDriverService.cmake

autogenerated/services/include/thrift/XsensDriverService.h: autogenerated/services/thrift_XsensDriverService_thrift.cmake
	@$(CMAKE_COMMAND) -E touch_nocreate autogenerated/services/include/thrift/XsensDriverService.h

autogenerated/services/src/XsensDriverService.cpp: autogenerated/services/thrift_XsensDriverService_thrift.cmake
	@$(CMAKE_COMMAND) -E touch_nocreate autogenerated/services/src/XsensDriverService.cpp

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o: CMakeFiles/yarpModuleLib.dir/flags.make
CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o: autogenerated/data/src/Vector3.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o -c /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/Vector3.cpp

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/Vector3.cpp > CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.i

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/Vector3.cpp -o CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.s

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o.requires:

.PHONY : CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o.requires

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o.provides: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o.requires
	$(MAKE) -f CMakeFiles/yarpModuleLib.dir/build.make CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o.provides.build
.PHONY : CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o.provides

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o.provides.build: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o


CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o: CMakeFiles/yarpModuleLib.dir/flags.make
CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o: autogenerated/data/src/Vector4.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o -c /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/Vector4.cpp

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/Vector4.cpp > CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.i

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/Vector4.cpp -o CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.s

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o.requires:

.PHONY : CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o.requires

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o.provides: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o.requires
	$(MAKE) -f CMakeFiles/yarpModuleLib.dir/build.make CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o.provides.build
.PHONY : CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o.provides

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o.provides.build: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o


CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o: CMakeFiles/yarpModuleLib.dir/flags.make
CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o: autogenerated/data/src/XsensSegmentData.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o -c /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/XsensSegmentData.cpp

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/XsensSegmentData.cpp > CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.i

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/XsensSegmentData.cpp -o CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.s

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o.requires:

.PHONY : CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o.requires

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o.provides: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o.requires
	$(MAKE) -f CMakeFiles/yarpModuleLib.dir/build.make CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o.provides.build
.PHONY : CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o.provides

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o.provides.build: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o


CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o: CMakeFiles/yarpModuleLib.dir/flags.make
CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o: autogenerated/data/src/XsensSensorData.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o -c /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/XsensSensorData.cpp

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/XsensSensorData.cpp > CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.i

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/XsensSensorData.cpp -o CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.s

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o.requires:

.PHONY : CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o.requires

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o.provides: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o.requires
	$(MAKE) -f CMakeFiles/yarpModuleLib.dir/build.make CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o.provides.build
.PHONY : CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o.provides

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o.provides.build: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o


CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o: CMakeFiles/yarpModuleLib.dir/flags.make
CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o: autogenerated/data/src/XsensFrame.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o -c /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/XsensFrame.cpp

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/XsensFrame.cpp > CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.i

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/data/src/XsensFrame.cpp -o CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.s

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o.requires:

.PHONY : CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o.requires

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o.provides: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o.requires
	$(MAKE) -f CMakeFiles/yarpModuleLib.dir/build.make CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o.provides.build
.PHONY : CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o.provides

CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o.provides.build: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o


CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o: CMakeFiles/yarpModuleLib.dir/flags.make
CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o: autogenerated/services/src/XsensDriverService.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Building CXX object CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o -c /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/services/src/XsensDriverService.cpp

CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/services/src/XsensDriverService.cpp > CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.i

CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/autogenerated/services/src/XsensDriverService.cpp -o CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.s

CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o.requires:

.PHONY : CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o.requires

CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o.provides: CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o.requires
	$(MAKE) -f CMakeFiles/yarpModuleLib.dir/build.make CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o.provides.build
.PHONY : CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o.provides

CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o.provides.build: CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o


# Object files for target yarpModuleLib
yarpModuleLib_OBJECTS = \
"CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o" \
"CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o" \
"CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o" \
"CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o" \
"CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o" \
"CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o"

# External object files for target yarpModuleLib
yarpModuleLib_EXTERNAL_OBJECTS =

libyarpModuleLib.a: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o
libyarpModuleLib.a: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o
libyarpModuleLib.a: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o
libyarpModuleLib.a: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o
libyarpModuleLib.a: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o
libyarpModuleLib.a: CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o
libyarpModuleLib.a: CMakeFiles/yarpModuleLib.dir/build.make
libyarpModuleLib.a: CMakeFiles/yarpModuleLib.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_9) "Linking CXX static library libyarpModuleLib.a"
	$(CMAKE_COMMAND) -P CMakeFiles/yarpModuleLib.dir/cmake_clean_target.cmake
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/yarpModuleLib.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/yarpModuleLib.dir/build: libyarpModuleLib.a

.PHONY : CMakeFiles/yarpModuleLib.dir/build

CMakeFiles/yarpModuleLib.dir/requires: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector3.cpp.o.requires
CMakeFiles/yarpModuleLib.dir/requires: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/Vector4.cpp.o.requires
CMakeFiles/yarpModuleLib.dir/requires: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSegmentData.cpp.o.requires
CMakeFiles/yarpModuleLib.dir/requires: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensSensorData.cpp.o.requires
CMakeFiles/yarpModuleLib.dir/requires: CMakeFiles/yarpModuleLib.dir/autogenerated/data/src/XsensFrame.cpp.o.requires
CMakeFiles/yarpModuleLib.dir/requires: CMakeFiles/yarpModuleLib.dir/autogenerated/services/src/XsensDriverService.cpp.o.requires

.PHONY : CMakeFiles/yarpModuleLib.dir/requires

CMakeFiles/yarpModuleLib.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/yarpModuleLib.dir/cmake_clean.cmake
.PHONY : CMakeFiles/yarpModuleLib.dir/clean

CMakeFiles/yarpModuleLib.dir/depend: autogenerated/data/thrift_XsensFrame_thrift.cmake
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/data/include/thrift/Vector3.h
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/data/include/thrift/Vector4.h
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/data/include/thrift/XsensSegmentData.h
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/data/include/thrift/XsensSensorData.h
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/data/include/thrift/XsensFrame.h
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/data/src/Vector3.cpp
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/data/src/Vector4.cpp
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/data/src/XsensSegmentData.cpp
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/data/src/XsensSensorData.cpp
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/data/src/XsensFrame.cpp
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/services/thrift_XsensDriverService_thrift.cmake
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/services/include/thrift/XsensDriverService.h
CMakeFiles/yarpModuleLib.dir/depend: autogenerated/services/src/XsensDriverService.cpp
	cd /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build /home/mlorenzini/gitHub/human-dynamics-estimation/xsens-tf-bridge/build/CMakeFiles/yarpModuleLib.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/yarpModuleLib.dir/depend

