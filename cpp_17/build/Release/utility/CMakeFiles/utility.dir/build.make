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
include utility/CMakeFiles/utility.dir/depend.make
# Include any dependencies generated by the compiler for this target.
include utility/CMakeFiles/utility.dir/compiler_depend.make

# Include the progress variables for this target.
include utility/CMakeFiles/utility.dir/progress.make

# Include the compile flags for this target's objects.
include utility/CMakeFiles/utility.dir/flags.make

utility/CMakeFiles/utility.dir/Active.cpp.o: utility/CMakeFiles/utility.dir/flags.make
utility/CMakeFiles/utility.dir/Active.cpp.o: /home/william/iasdk/cpp_17/src/utility/Active.cpp
utility/CMakeFiles/utility.dir/Active.cpp.o: utility/CMakeFiles/utility.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/william/iasdk/cpp_17/build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object utility/CMakeFiles/utility.dir/Active.cpp.o"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utility/CMakeFiles/utility.dir/Active.cpp.o -MF CMakeFiles/utility.dir/Active.cpp.o.d -o CMakeFiles/utility.dir/Active.cpp.o -c /home/william/iasdk/cpp_17/src/utility/Active.cpp

utility/CMakeFiles/utility.dir/Active.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utility.dir/Active.cpp.i"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/william/iasdk/cpp_17/src/utility/Active.cpp > CMakeFiles/utility.dir/Active.cpp.i

utility/CMakeFiles/utility.dir/Active.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utility.dir/Active.cpp.s"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/william/iasdk/cpp_17/src/utility/Active.cpp -o CMakeFiles/utility.dir/Active.cpp.s

utility/CMakeFiles/utility.dir/Log.cpp.o: utility/CMakeFiles/utility.dir/flags.make
utility/CMakeFiles/utility.dir/Log.cpp.o: /home/william/iasdk/cpp_17/src/utility/Log.cpp
utility/CMakeFiles/utility.dir/Log.cpp.o: utility/CMakeFiles/utility.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/william/iasdk/cpp_17/build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object utility/CMakeFiles/utility.dir/Log.cpp.o"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utility/CMakeFiles/utility.dir/Log.cpp.o -MF CMakeFiles/utility.dir/Log.cpp.o.d -o CMakeFiles/utility.dir/Log.cpp.o -c /home/william/iasdk/cpp_17/src/utility/Log.cpp

utility/CMakeFiles/utility.dir/Log.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utility.dir/Log.cpp.i"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/william/iasdk/cpp_17/src/utility/Log.cpp > CMakeFiles/utility.dir/Log.cpp.i

utility/CMakeFiles/utility.dir/Log.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utility.dir/Log.cpp.s"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/william/iasdk/cpp_17/src/utility/Log.cpp -o CMakeFiles/utility.dir/Log.cpp.s

utility/CMakeFiles/utility.dir/Option_parser.cpp.o: utility/CMakeFiles/utility.dir/flags.make
utility/CMakeFiles/utility.dir/Option_parser.cpp.o: /home/william/iasdk/cpp_17/src/utility/Option_parser.cpp
utility/CMakeFiles/utility.dir/Option_parser.cpp.o: utility/CMakeFiles/utility.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/william/iasdk/cpp_17/build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object utility/CMakeFiles/utility.dir/Option_parser.cpp.o"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utility/CMakeFiles/utility.dir/Option_parser.cpp.o -MF CMakeFiles/utility.dir/Option_parser.cpp.o.d -o CMakeFiles/utility.dir/Option_parser.cpp.o -c /home/william/iasdk/cpp_17/src/utility/Option_parser.cpp

utility/CMakeFiles/utility.dir/Option_parser.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utility.dir/Option_parser.cpp.i"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/william/iasdk/cpp_17/src/utility/Option_parser.cpp > CMakeFiles/utility.dir/Option_parser.cpp.i

utility/CMakeFiles/utility.dir/Option_parser.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utility.dir/Option_parser.cpp.s"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/william/iasdk/cpp_17/src/utility/Option_parser.cpp -o CMakeFiles/utility.dir/Option_parser.cpp.s

utility/CMakeFiles/utility.dir/Signal_handler.cpp.o: utility/CMakeFiles/utility.dir/flags.make
utility/CMakeFiles/utility.dir/Signal_handler.cpp.o: /home/william/iasdk/cpp_17/src/utility/Signal_handler.cpp
utility/CMakeFiles/utility.dir/Signal_handler.cpp.o: utility/CMakeFiles/utility.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/william/iasdk/cpp_17/build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object utility/CMakeFiles/utility.dir/Signal_handler.cpp.o"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utility/CMakeFiles/utility.dir/Signal_handler.cpp.o -MF CMakeFiles/utility.dir/Signal_handler.cpp.o.d -o CMakeFiles/utility.dir/Signal_handler.cpp.o -c /home/william/iasdk/cpp_17/src/utility/Signal_handler.cpp

utility/CMakeFiles/utility.dir/Signal_handler.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utility.dir/Signal_handler.cpp.i"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/william/iasdk/cpp_17/src/utility/Signal_handler.cpp > CMakeFiles/utility.dir/Signal_handler.cpp.i

utility/CMakeFiles/utility.dir/Signal_handler.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utility.dir/Signal_handler.cpp.s"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/william/iasdk/cpp_17/src/utility/Signal_handler.cpp -o CMakeFiles/utility.dir/Signal_handler.cpp.s

utility/CMakeFiles/utility.dir/Time_utils.cpp.o: utility/CMakeFiles/utility.dir/flags.make
utility/CMakeFiles/utility.dir/Time_utils.cpp.o: /home/william/iasdk/cpp_17/src/utility/Time_utils.cpp
utility/CMakeFiles/utility.dir/Time_utils.cpp.o: utility/CMakeFiles/utility.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/william/iasdk/cpp_17/build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object utility/CMakeFiles/utility.dir/Time_utils.cpp.o"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utility/CMakeFiles/utility.dir/Time_utils.cpp.o -MF CMakeFiles/utility.dir/Time_utils.cpp.o.d -o CMakeFiles/utility.dir/Time_utils.cpp.o -c /home/william/iasdk/cpp_17/src/utility/Time_utils.cpp

utility/CMakeFiles/utility.dir/Time_utils.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utility.dir/Time_utils.cpp.i"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/william/iasdk/cpp_17/src/utility/Time_utils.cpp > CMakeFiles/utility.dir/Time_utils.cpp.i

utility/CMakeFiles/utility.dir/Time_utils.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utility.dir/Time_utils.cpp.s"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/william/iasdk/cpp_17/src/utility/Time_utils.cpp -o CMakeFiles/utility.dir/Time_utils.cpp.s

utility/CMakeFiles/utility.dir/Timer.cpp.o: utility/CMakeFiles/utility.dir/flags.make
utility/CMakeFiles/utility.dir/Timer.cpp.o: /home/william/iasdk/cpp_17/src/utility/Timer.cpp
utility/CMakeFiles/utility.dir/Timer.cpp.o: utility/CMakeFiles/utility.dir/compiler_depend.ts
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/william/iasdk/cpp_17/build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object utility/CMakeFiles/utility.dir/Timer.cpp.o"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -MD -MT utility/CMakeFiles/utility.dir/Timer.cpp.o -MF CMakeFiles/utility.dir/Timer.cpp.o.d -o CMakeFiles/utility.dir/Timer.cpp.o -c /home/william/iasdk/cpp_17/src/utility/Timer.cpp

utility/CMakeFiles/utility.dir/Timer.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/utility.dir/Timer.cpp.i"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/william/iasdk/cpp_17/src/utility/Timer.cpp > CMakeFiles/utility.dir/Timer.cpp.i

utility/CMakeFiles/utility.dir/Timer.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/utility.dir/Timer.cpp.s"
	cd /home/william/iasdk/cpp_17/build/Release/utility && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/william/iasdk/cpp_17/src/utility/Timer.cpp -o CMakeFiles/utility.dir/Timer.cpp.s

# Object files for target utility
utility_OBJECTS = \
"CMakeFiles/utility.dir/Active.cpp.o" \
"CMakeFiles/utility.dir/Log.cpp.o" \
"CMakeFiles/utility.dir/Option_parser.cpp.o" \
"CMakeFiles/utility.dir/Signal_handler.cpp.o" \
"CMakeFiles/utility.dir/Time_utils.cpp.o" \
"CMakeFiles/utility.dir/Timer.cpp.o"

# External object files for target utility
utility_EXTERNAL_OBJECTS =

utility/libutility.a: utility/CMakeFiles/utility.dir/Active.cpp.o
utility/libutility.a: utility/CMakeFiles/utility.dir/Log.cpp.o
utility/libutility.a: utility/CMakeFiles/utility.dir/Option_parser.cpp.o
utility/libutility.a: utility/CMakeFiles/utility.dir/Signal_handler.cpp.o
utility/libutility.a: utility/CMakeFiles/utility.dir/Time_utils.cpp.o
utility/libutility.a: utility/CMakeFiles/utility.dir/Timer.cpp.o
utility/libutility.a: utility/CMakeFiles/utility.dir/build.make
utility/libutility.a: utility/CMakeFiles/utility.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/william/iasdk/cpp_17/build/Release/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Linking CXX static library libutility.a"
	cd /home/william/iasdk/cpp_17/build/Release/utility && $(CMAKE_COMMAND) -P CMakeFiles/utility.dir/cmake_clean_target.cmake
	cd /home/william/iasdk/cpp_17/build/Release/utility && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/utility.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
utility/CMakeFiles/utility.dir/build: utility/libutility.a
.PHONY : utility/CMakeFiles/utility.dir/build

utility/CMakeFiles/utility.dir/clean:
	cd /home/william/iasdk/cpp_17/build/Release/utility && $(CMAKE_COMMAND) -P CMakeFiles/utility.dir/cmake_clean.cmake
.PHONY : utility/CMakeFiles/utility.dir/clean

utility/CMakeFiles/utility.dir/depend:
	cd /home/william/iasdk/cpp_17/build/Release && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/william/iasdk/cpp_17/src /home/william/iasdk/cpp_17/src/utility /home/william/iasdk/cpp_17/build/Release /home/william/iasdk/cpp_17/build/Release/utility /home/william/iasdk/cpp_17/build/Release/utility/CMakeFiles/utility.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : utility/CMakeFiles/utility.dir/depend
