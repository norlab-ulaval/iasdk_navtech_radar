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
CMAKE_SOURCE_DIR = /home/william/iasdk/cpp_17/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/william/iasdk/cpp_17/build/Release

# Include any dependencies generated for this target.
include navigation/CMakeFiles/navigation.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include navigation/CMakeFiles/navigation.dir/compiler_depend.make

# Include the progress variables for this target.
include navigation/CMakeFiles/navigation.dir/progress.make

# Include the compile flags for this target's objects.
include navigation/CMakeFiles/navigation.dir/flags.make

navigation/CMakeFiles/navigation.dir/Peak_finder.cpp.o: navigation/CMakeFiles/navigation.dir/flags.make
navigation/CMakeFiles/navigation.dir/Peak_finder.cpp.o: /home/william/iasdk/cpp_17/src/navigation/Peak_finder.cpp
navigation/CMakeFiles/navigation.dir/Peak_finder.cpp.o: navigation/CMakeFiles/navigation.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/william/iasdk/cpp_17/build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object navigation/CMakeFiles/navigation.dir/Peak_finder.cpp.o"
	cd /home/william/iasdk/cpp_17/build/Release/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT navigation/CMakeFiles/navigation.dir/Peak_finder.cpp.o -MF CMakeFiles/navigation.dir/Peak_finder.cpp.o.d -o CMakeFiles/navigation.dir/Peak_finder.cpp.o -c /home/william/iasdk/cpp_17/src/navigation/Peak_finder.cpp

navigation/CMakeFiles/navigation.dir/Peak_finder.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/navigation.dir/Peak_finder.cpp.i"
	cd /home/william/iasdk/cpp_17/build/Release/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/william/iasdk/cpp_17/src/navigation/Peak_finder.cpp > CMakeFiles/navigation.dir/Peak_finder.cpp.i

navigation/CMakeFiles/navigation.dir/Peak_finder.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/navigation.dir/Peak_finder.cpp.s"
	cd /home/william/iasdk/cpp_17/build/Release/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/william/iasdk/cpp_17/src/navigation/Peak_finder.cpp -o CMakeFiles/navigation.dir/Peak_finder.cpp.s

navigation/CMakeFiles/navigation.dir/Sector_blanking.cpp.o: navigation/CMakeFiles/navigation.dir/flags.make
navigation/CMakeFiles/navigation.dir/Sector_blanking.cpp.o: /home/william/iasdk/cpp_17/src/navigation/Sector_blanking.cpp
navigation/CMakeFiles/navigation.dir/Sector_blanking.cpp.o: navigation/CMakeFiles/navigation.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/william/iasdk/cpp_17/build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object navigation/CMakeFiles/navigation.dir/Sector_blanking.cpp.o"
	cd /home/william/iasdk/cpp_17/build/Release/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT navigation/CMakeFiles/navigation.dir/Sector_blanking.cpp.o -MF CMakeFiles/navigation.dir/Sector_blanking.cpp.o.d -o CMakeFiles/navigation.dir/Sector_blanking.cpp.o -c /home/william/iasdk/cpp_17/src/navigation/Sector_blanking.cpp

navigation/CMakeFiles/navigation.dir/Sector_blanking.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/navigation.dir/Sector_blanking.cpp.i"
	cd /home/william/iasdk/cpp_17/build/Release/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/william/iasdk/cpp_17/src/navigation/Sector_blanking.cpp > CMakeFiles/navigation.dir/Sector_blanking.cpp.i

navigation/CMakeFiles/navigation.dir/Sector_blanking.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/navigation.dir/Sector_blanking.cpp.s"
	cd /home/william/iasdk/cpp_17/build/Release/navigation && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/william/iasdk/cpp_17/src/navigation/Sector_blanking.cpp -o CMakeFiles/navigation.dir/Sector_blanking.cpp.s

# Object files for target navigation
navigation_OBJECTS = \
"CMakeFiles/navigation.dir/Peak_finder.cpp.o" \
"CMakeFiles/navigation.dir/Sector_blanking.cpp.o"

# External object files for target navigation
navigation_EXTERNAL_OBJECTS =

navigation/libnavigation.a: navigation/CMakeFiles/navigation.dir/Peak_finder.cpp.o
navigation/libnavigation.a: navigation/CMakeFiles/navigation.dir/Sector_blanking.cpp.o
navigation/libnavigation.a: navigation/CMakeFiles/navigation.dir/build.make
navigation/libnavigation.a: navigation/CMakeFiles/navigation.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/william/iasdk/cpp_17/build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Linking CXX static library libnavigation.a"
	cd /home/william/iasdk/cpp_17/build/Release/navigation && $(CMAKE_COMMAND) -P CMakeFiles/navigation.dir/cmake_clean_target.cmake
	cd /home/william/iasdk/cpp_17/build/Release/navigation && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/navigation.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
navigation/CMakeFiles/navigation.dir/build: navigation/libnavigation.a
.PHONY : navigation/CMakeFiles/navigation.dir/build

navigation/CMakeFiles/navigation.dir/clean:
	cd /home/william/iasdk/cpp_17/build/Release/navigation && $(CMAKE_COMMAND) -P CMakeFiles/navigation.dir/cmake_clean.cmake
.PHONY : navigation/CMakeFiles/navigation.dir/clean

navigation/CMakeFiles/navigation.dir/depend:
	cd /home/william/iasdk/cpp_17/build/Release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/william/iasdk/cpp_17/src /home/william/iasdk/cpp_17/src/navigation /home/william/iasdk/cpp_17/build/Release /home/william/iasdk/cpp_17/build/Release/navigation /home/william/iasdk/cpp_17/build/Release/navigation/CMakeFiles/navigation.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : navigation/CMakeFiles/navigation.dir/depend

