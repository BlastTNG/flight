# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.0

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
CMAKE_SOURCE_DIR = /home/seth/git/blastpol/external_libs/soem

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/seth/git/blastpol/external_libs/build

# Include any dependencies generated for this target.
include CMakeFiles/soem.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/soem.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/soem.dir/flags.make

CMakeFiles/soem.dir/soem/ethercatbase.c.o: CMakeFiles/soem.dir/flags.make
CMakeFiles/soem.dir/soem/ethercatbase.c.o: /home/seth/git/blastpol/external_libs/soem/soem/ethercatbase.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/seth/git/blastpol/external_libs/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/soem.dir/soem/ethercatbase.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/soem.dir/soem/ethercatbase.c.o   -c /home/seth/git/blastpol/external_libs/soem/soem/ethercatbase.c

CMakeFiles/soem.dir/soem/ethercatbase.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/soem.dir/soem/ethercatbase.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/seth/git/blastpol/external_libs/soem/soem/ethercatbase.c > CMakeFiles/soem.dir/soem/ethercatbase.c.i

CMakeFiles/soem.dir/soem/ethercatbase.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/soem.dir/soem/ethercatbase.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/seth/git/blastpol/external_libs/soem/soem/ethercatbase.c -o CMakeFiles/soem.dir/soem/ethercatbase.c.s

CMakeFiles/soem.dir/soem/ethercatbase.c.o.requires:
.PHONY : CMakeFiles/soem.dir/soem/ethercatbase.c.o.requires

CMakeFiles/soem.dir/soem/ethercatbase.c.o.provides: CMakeFiles/soem.dir/soem/ethercatbase.c.o.requires
	$(MAKE) -f CMakeFiles/soem.dir/build.make CMakeFiles/soem.dir/soem/ethercatbase.c.o.provides.build
.PHONY : CMakeFiles/soem.dir/soem/ethercatbase.c.o.provides

CMakeFiles/soem.dir/soem/ethercatbase.c.o.provides.build: CMakeFiles/soem.dir/soem/ethercatbase.c.o

CMakeFiles/soem.dir/soem/ethercatcoe.c.o: CMakeFiles/soem.dir/flags.make
CMakeFiles/soem.dir/soem/ethercatcoe.c.o: /home/seth/git/blastpol/external_libs/soem/soem/ethercatcoe.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/seth/git/blastpol/external_libs/build/CMakeFiles $(CMAKE_PROGRESS_2)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/soem.dir/soem/ethercatcoe.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/soem.dir/soem/ethercatcoe.c.o   -c /home/seth/git/blastpol/external_libs/soem/soem/ethercatcoe.c

CMakeFiles/soem.dir/soem/ethercatcoe.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/soem.dir/soem/ethercatcoe.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/seth/git/blastpol/external_libs/soem/soem/ethercatcoe.c > CMakeFiles/soem.dir/soem/ethercatcoe.c.i

CMakeFiles/soem.dir/soem/ethercatcoe.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/soem.dir/soem/ethercatcoe.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/seth/git/blastpol/external_libs/soem/soem/ethercatcoe.c -o CMakeFiles/soem.dir/soem/ethercatcoe.c.s

CMakeFiles/soem.dir/soem/ethercatcoe.c.o.requires:
.PHONY : CMakeFiles/soem.dir/soem/ethercatcoe.c.o.requires

CMakeFiles/soem.dir/soem/ethercatcoe.c.o.provides: CMakeFiles/soem.dir/soem/ethercatcoe.c.o.requires
	$(MAKE) -f CMakeFiles/soem.dir/build.make CMakeFiles/soem.dir/soem/ethercatcoe.c.o.provides.build
.PHONY : CMakeFiles/soem.dir/soem/ethercatcoe.c.o.provides

CMakeFiles/soem.dir/soem/ethercatcoe.c.o.provides.build: CMakeFiles/soem.dir/soem/ethercatcoe.c.o

CMakeFiles/soem.dir/soem/ethercatconfig.c.o: CMakeFiles/soem.dir/flags.make
CMakeFiles/soem.dir/soem/ethercatconfig.c.o: /home/seth/git/blastpol/external_libs/soem/soem/ethercatconfig.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/seth/git/blastpol/external_libs/build/CMakeFiles $(CMAKE_PROGRESS_3)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/soem.dir/soem/ethercatconfig.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/soem.dir/soem/ethercatconfig.c.o   -c /home/seth/git/blastpol/external_libs/soem/soem/ethercatconfig.c

CMakeFiles/soem.dir/soem/ethercatconfig.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/soem.dir/soem/ethercatconfig.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/seth/git/blastpol/external_libs/soem/soem/ethercatconfig.c > CMakeFiles/soem.dir/soem/ethercatconfig.c.i

CMakeFiles/soem.dir/soem/ethercatconfig.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/soem.dir/soem/ethercatconfig.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/seth/git/blastpol/external_libs/soem/soem/ethercatconfig.c -o CMakeFiles/soem.dir/soem/ethercatconfig.c.s

CMakeFiles/soem.dir/soem/ethercatconfig.c.o.requires:
.PHONY : CMakeFiles/soem.dir/soem/ethercatconfig.c.o.requires

CMakeFiles/soem.dir/soem/ethercatconfig.c.o.provides: CMakeFiles/soem.dir/soem/ethercatconfig.c.o.requires
	$(MAKE) -f CMakeFiles/soem.dir/build.make CMakeFiles/soem.dir/soem/ethercatconfig.c.o.provides.build
.PHONY : CMakeFiles/soem.dir/soem/ethercatconfig.c.o.provides

CMakeFiles/soem.dir/soem/ethercatconfig.c.o.provides.build: CMakeFiles/soem.dir/soem/ethercatconfig.c.o

CMakeFiles/soem.dir/soem/ethercatdc.c.o: CMakeFiles/soem.dir/flags.make
CMakeFiles/soem.dir/soem/ethercatdc.c.o: /home/seth/git/blastpol/external_libs/soem/soem/ethercatdc.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/seth/git/blastpol/external_libs/build/CMakeFiles $(CMAKE_PROGRESS_4)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/soem.dir/soem/ethercatdc.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/soem.dir/soem/ethercatdc.c.o   -c /home/seth/git/blastpol/external_libs/soem/soem/ethercatdc.c

CMakeFiles/soem.dir/soem/ethercatdc.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/soem.dir/soem/ethercatdc.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/seth/git/blastpol/external_libs/soem/soem/ethercatdc.c > CMakeFiles/soem.dir/soem/ethercatdc.c.i

CMakeFiles/soem.dir/soem/ethercatdc.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/soem.dir/soem/ethercatdc.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/seth/git/blastpol/external_libs/soem/soem/ethercatdc.c -o CMakeFiles/soem.dir/soem/ethercatdc.c.s

CMakeFiles/soem.dir/soem/ethercatdc.c.o.requires:
.PHONY : CMakeFiles/soem.dir/soem/ethercatdc.c.o.requires

CMakeFiles/soem.dir/soem/ethercatdc.c.o.provides: CMakeFiles/soem.dir/soem/ethercatdc.c.o.requires
	$(MAKE) -f CMakeFiles/soem.dir/build.make CMakeFiles/soem.dir/soem/ethercatdc.c.o.provides.build
.PHONY : CMakeFiles/soem.dir/soem/ethercatdc.c.o.provides

CMakeFiles/soem.dir/soem/ethercatdc.c.o.provides.build: CMakeFiles/soem.dir/soem/ethercatdc.c.o

CMakeFiles/soem.dir/soem/ethercatfoe.c.o: CMakeFiles/soem.dir/flags.make
CMakeFiles/soem.dir/soem/ethercatfoe.c.o: /home/seth/git/blastpol/external_libs/soem/soem/ethercatfoe.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/seth/git/blastpol/external_libs/build/CMakeFiles $(CMAKE_PROGRESS_5)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/soem.dir/soem/ethercatfoe.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/soem.dir/soem/ethercatfoe.c.o   -c /home/seth/git/blastpol/external_libs/soem/soem/ethercatfoe.c

CMakeFiles/soem.dir/soem/ethercatfoe.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/soem.dir/soem/ethercatfoe.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/seth/git/blastpol/external_libs/soem/soem/ethercatfoe.c > CMakeFiles/soem.dir/soem/ethercatfoe.c.i

CMakeFiles/soem.dir/soem/ethercatfoe.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/soem.dir/soem/ethercatfoe.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/seth/git/blastpol/external_libs/soem/soem/ethercatfoe.c -o CMakeFiles/soem.dir/soem/ethercatfoe.c.s

CMakeFiles/soem.dir/soem/ethercatfoe.c.o.requires:
.PHONY : CMakeFiles/soem.dir/soem/ethercatfoe.c.o.requires

CMakeFiles/soem.dir/soem/ethercatfoe.c.o.provides: CMakeFiles/soem.dir/soem/ethercatfoe.c.o.requires
	$(MAKE) -f CMakeFiles/soem.dir/build.make CMakeFiles/soem.dir/soem/ethercatfoe.c.o.provides.build
.PHONY : CMakeFiles/soem.dir/soem/ethercatfoe.c.o.provides

CMakeFiles/soem.dir/soem/ethercatfoe.c.o.provides.build: CMakeFiles/soem.dir/soem/ethercatfoe.c.o

CMakeFiles/soem.dir/soem/ethercatmain.c.o: CMakeFiles/soem.dir/flags.make
CMakeFiles/soem.dir/soem/ethercatmain.c.o: /home/seth/git/blastpol/external_libs/soem/soem/ethercatmain.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/seth/git/blastpol/external_libs/build/CMakeFiles $(CMAKE_PROGRESS_6)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/soem.dir/soem/ethercatmain.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/soem.dir/soem/ethercatmain.c.o   -c /home/seth/git/blastpol/external_libs/soem/soem/ethercatmain.c

CMakeFiles/soem.dir/soem/ethercatmain.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/soem.dir/soem/ethercatmain.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/seth/git/blastpol/external_libs/soem/soem/ethercatmain.c > CMakeFiles/soem.dir/soem/ethercatmain.c.i

CMakeFiles/soem.dir/soem/ethercatmain.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/soem.dir/soem/ethercatmain.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/seth/git/blastpol/external_libs/soem/soem/ethercatmain.c -o CMakeFiles/soem.dir/soem/ethercatmain.c.s

CMakeFiles/soem.dir/soem/ethercatmain.c.o.requires:
.PHONY : CMakeFiles/soem.dir/soem/ethercatmain.c.o.requires

CMakeFiles/soem.dir/soem/ethercatmain.c.o.provides: CMakeFiles/soem.dir/soem/ethercatmain.c.o.requires
	$(MAKE) -f CMakeFiles/soem.dir/build.make CMakeFiles/soem.dir/soem/ethercatmain.c.o.provides.build
.PHONY : CMakeFiles/soem.dir/soem/ethercatmain.c.o.provides

CMakeFiles/soem.dir/soem/ethercatmain.c.o.provides.build: CMakeFiles/soem.dir/soem/ethercatmain.c.o

CMakeFiles/soem.dir/soem/ethercatprint.c.o: CMakeFiles/soem.dir/flags.make
CMakeFiles/soem.dir/soem/ethercatprint.c.o: /home/seth/git/blastpol/external_libs/soem/soem/ethercatprint.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/seth/git/blastpol/external_libs/build/CMakeFiles $(CMAKE_PROGRESS_7)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/soem.dir/soem/ethercatprint.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/soem.dir/soem/ethercatprint.c.o   -c /home/seth/git/blastpol/external_libs/soem/soem/ethercatprint.c

CMakeFiles/soem.dir/soem/ethercatprint.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/soem.dir/soem/ethercatprint.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/seth/git/blastpol/external_libs/soem/soem/ethercatprint.c > CMakeFiles/soem.dir/soem/ethercatprint.c.i

CMakeFiles/soem.dir/soem/ethercatprint.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/soem.dir/soem/ethercatprint.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/seth/git/blastpol/external_libs/soem/soem/ethercatprint.c -o CMakeFiles/soem.dir/soem/ethercatprint.c.s

CMakeFiles/soem.dir/soem/ethercatprint.c.o.requires:
.PHONY : CMakeFiles/soem.dir/soem/ethercatprint.c.o.requires

CMakeFiles/soem.dir/soem/ethercatprint.c.o.provides: CMakeFiles/soem.dir/soem/ethercatprint.c.o.requires
	$(MAKE) -f CMakeFiles/soem.dir/build.make CMakeFiles/soem.dir/soem/ethercatprint.c.o.provides.build
.PHONY : CMakeFiles/soem.dir/soem/ethercatprint.c.o.provides

CMakeFiles/soem.dir/soem/ethercatprint.c.o.provides.build: CMakeFiles/soem.dir/soem/ethercatprint.c.o

CMakeFiles/soem.dir/soem/ethercatsoe.c.o: CMakeFiles/soem.dir/flags.make
CMakeFiles/soem.dir/soem/ethercatsoe.c.o: /home/seth/git/blastpol/external_libs/soem/soem/ethercatsoe.c
	$(CMAKE_COMMAND) -E cmake_progress_report /home/seth/git/blastpol/external_libs/build/CMakeFiles $(CMAKE_PROGRESS_8)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building C object CMakeFiles/soem.dir/soem/ethercatsoe.c.o"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -o CMakeFiles/soem.dir/soem/ethercatsoe.c.o   -c /home/seth/git/blastpol/external_libs/soem/soem/ethercatsoe.c

CMakeFiles/soem.dir/soem/ethercatsoe.c.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing C source to CMakeFiles/soem.dir/soem/ethercatsoe.c.i"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -E /home/seth/git/blastpol/external_libs/soem/soem/ethercatsoe.c > CMakeFiles/soem.dir/soem/ethercatsoe.c.i

CMakeFiles/soem.dir/soem/ethercatsoe.c.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling C source to assembly CMakeFiles/soem.dir/soem/ethercatsoe.c.s"
	/usr/bin/cc  $(C_DEFINES) $(C_FLAGS) -S /home/seth/git/blastpol/external_libs/soem/soem/ethercatsoe.c -o CMakeFiles/soem.dir/soem/ethercatsoe.c.s

CMakeFiles/soem.dir/soem/ethercatsoe.c.o.requires:
.PHONY : CMakeFiles/soem.dir/soem/ethercatsoe.c.o.requires

CMakeFiles/soem.dir/soem/ethercatsoe.c.o.provides: CMakeFiles/soem.dir/soem/ethercatsoe.c.o.requires
	$(MAKE) -f CMakeFiles/soem.dir/build.make CMakeFiles/soem.dir/soem/ethercatsoe.c.o.provides.build
.PHONY : CMakeFiles/soem.dir/soem/ethercatsoe.c.o.provides

CMakeFiles/soem.dir/soem/ethercatsoe.c.o.provides.build: CMakeFiles/soem.dir/soem/ethercatsoe.c.o

soem: CMakeFiles/soem.dir/soem/ethercatbase.c.o
soem: CMakeFiles/soem.dir/soem/ethercatcoe.c.o
soem: CMakeFiles/soem.dir/soem/ethercatconfig.c.o
soem: CMakeFiles/soem.dir/soem/ethercatdc.c.o
soem: CMakeFiles/soem.dir/soem/ethercatfoe.c.o
soem: CMakeFiles/soem.dir/soem/ethercatmain.c.o
soem: CMakeFiles/soem.dir/soem/ethercatprint.c.o
soem: CMakeFiles/soem.dir/soem/ethercatsoe.c.o
soem: CMakeFiles/soem.dir/build.make
.PHONY : soem

# Rule to build all files generated by this target.
CMakeFiles/soem.dir/build: soem
.PHONY : CMakeFiles/soem.dir/build

CMakeFiles/soem.dir/requires: CMakeFiles/soem.dir/soem/ethercatbase.c.o.requires
CMakeFiles/soem.dir/requires: CMakeFiles/soem.dir/soem/ethercatcoe.c.o.requires
CMakeFiles/soem.dir/requires: CMakeFiles/soem.dir/soem/ethercatconfig.c.o.requires
CMakeFiles/soem.dir/requires: CMakeFiles/soem.dir/soem/ethercatdc.c.o.requires
CMakeFiles/soem.dir/requires: CMakeFiles/soem.dir/soem/ethercatfoe.c.o.requires
CMakeFiles/soem.dir/requires: CMakeFiles/soem.dir/soem/ethercatmain.c.o.requires
CMakeFiles/soem.dir/requires: CMakeFiles/soem.dir/soem/ethercatprint.c.o.requires
CMakeFiles/soem.dir/requires: CMakeFiles/soem.dir/soem/ethercatsoe.c.o.requires
.PHONY : CMakeFiles/soem.dir/requires

CMakeFiles/soem.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/soem.dir/cmake_clean.cmake
.PHONY : CMakeFiles/soem.dir/clean

CMakeFiles/soem.dir/depend:
	cd /home/seth/git/blastpol/external_libs/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/seth/git/blastpol/external_libs/soem /home/seth/git/blastpol/external_libs/soem /home/seth/git/blastpol/external_libs/build /home/seth/git/blastpol/external_libs/build /home/seth/git/blastpol/external_libs/build/CMakeFiles/soem.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/soem.dir/depend

