# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

# Default target executed when no arguments are given to make.
default_target: all
.PHONY : default_target

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
CMAKE_SOURCE_DIR = ~/git/ibvs_formation_bearing

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = ~/git/ibvs_formation_bearing

#=============================================================================
# Targets provided globally by CMake.

# Special rule for the target edit_cache
edit_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running interactive CMake command-line interface..."
	/usr/bin/cmake -i .
.PHONY : edit_cache

# Special rule for the target edit_cache
edit_cache/fast: edit_cache
.PHONY : edit_cache/fast

# Special rule for the target install
install: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install

# Special rule for the target install
install/fast: preinstall/fast
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Install the project..."
	/usr/bin/cmake -P cmake_install.cmake
.PHONY : install/fast

# Special rule for the target install/local
install/local: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing only the local directory..."
	/usr/bin/cmake -DCMAKE_INSTALL_LOCAL_ONLY=1 -P cmake_install.cmake
.PHONY : install/local

# Special rule for the target install/local
install/local/fast: install/local
.PHONY : install/local/fast

# Special rule for the target install/strip
install/strip: preinstall
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Installing the project stripped..."
	/usr/bin/cmake -DCMAKE_INSTALL_DO_STRIP=1 -P cmake_install.cmake
.PHONY : install/strip

# Special rule for the target install/strip
install/strip/fast: install/strip
.PHONY : install/strip/fast

# Special rule for the target list_install_components
list_install_components:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Available install components are: \"Unspecified\""
.PHONY : list_install_components

# Special rule for the target list_install_components
list_install_components/fast: list_install_components
.PHONY : list_install_components/fast

# Special rule for the target rebuild_cache
rebuild_cache:
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --cyan "Running CMake to regenerate build system..."
	/usr/bin/cmake -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
.PHONY : rebuild_cache

# Special rule for the target rebuild_cache
rebuild_cache/fast: rebuild_cache
.PHONY : rebuild_cache/fast

# The main all target
all: cmake_check_build_system
	$(CMAKE_COMMAND) -E cmake_progress_start ~/git/ibvs_formation_bearing/CMakeFiles ~/git/ibvs_formation_bearing/CMakeFiles/progress.marks
	$(MAKE) -f CMakeFiles/Makefile2 all
	$(CMAKE_COMMAND) -E cmake_progress_start ~/git/ibvs_formation_bearing/CMakeFiles 0
.PHONY : all

# The main clean target
clean:
	$(MAKE) -f CMakeFiles/Makefile2 clean
.PHONY : clean

# The main clean target
clean/fast: clean
.PHONY : clean/fast

# Prepare targets for installation.
preinstall: all
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall

# Prepare targets for installation.
preinstall/fast:
	$(MAKE) -f CMakeFiles/Makefile2 preinstall
.PHONY : preinstall/fast

# clear depends
depend:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 1
.PHONY : depend

#=============================================================================
# Target rules for targets named ROSBUILD_genmsg_cpp

# Build rule for target.
ROSBUILD_genmsg_cpp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_genmsg_cpp
.PHONY : ROSBUILD_genmsg_cpp

# fast build rule for target.
ROSBUILD_genmsg_cpp/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_genmsg_cpp.dir/build.make CMakeFiles/ROSBUILD_genmsg_cpp.dir/build
.PHONY : ROSBUILD_genmsg_cpp/fast

#=============================================================================
# Target rules for targets named ROSBUILD_genmsg_py

# Build rule for target.
ROSBUILD_genmsg_py: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_genmsg_py
.PHONY : ROSBUILD_genmsg_py

# fast build rule for target.
ROSBUILD_genmsg_py/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_genmsg_py.dir/build.make CMakeFiles/ROSBUILD_genmsg_py.dir/build
.PHONY : ROSBUILD_genmsg_py/fast

#=============================================================================
# Target rules for targets named ROSBUILD_gensrv_cpp

# Build rule for target.
ROSBUILD_gensrv_cpp: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 ROSBUILD_gensrv_cpp
.PHONY : ROSBUILD_gensrv_cpp

# fast build rule for target.
ROSBUILD_gensrv_cpp/fast:
	$(MAKE) -f CMakeFiles/ROSBUILD_gensrv_cpp.dir/build.make CMakeFiles/ROSBUILD_gensrv_cpp.dir/build
.PHONY : ROSBUILD_gensrv_cpp/fast

#=============================================================================
# Target rules for targets named _catkin_empty_exported_target

# Build rule for target.
_catkin_empty_exported_target: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 _catkin_empty_exported_target
.PHONY : _catkin_empty_exported_target

# fast build rule for target.
_catkin_empty_exported_target/fast:
	$(MAKE) -f CMakeFiles/_catkin_empty_exported_target.dir/build.make CMakeFiles/_catkin_empty_exported_target.dir/build
.PHONY : _catkin_empty_exported_target/fast

#=============================================================================
# Target rules for targets named circle_detection

# Build rule for target.
circle_detection: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 circle_detection
.PHONY : circle_detection

# fast build rule for target.
circle_detection/fast:
	$(MAKE) -f CMakeFiles/circle_detection.dir/build.make CMakeFiles/circle_detection.dir/build
.PHONY : circle_detection/fast

#=============================================================================
# Target rules for targets named clean_test_results

# Build rule for target.
clean_test_results: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 clean_test_results
.PHONY : clean_test_results

# fast build rule for target.
clean_test_results/fast:
	$(MAKE) -f CMakeFiles/clean_test_results.dir/build.make CMakeFiles/clean_test_results.dir/build
.PHONY : clean_test_results/fast

#=============================================================================
# Target rules for targets named doxygen

# Build rule for target.
doxygen: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 doxygen
.PHONY : doxygen

# fast build rule for target.
doxygen/fast:
	$(MAKE) -f CMakeFiles/doxygen.dir/build.make CMakeFiles/doxygen.dir/build
.PHONY : doxygen/fast

#=============================================================================
# Target rules for targets named lib

# Build rule for target.
lib: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 lib
.PHONY : lib

# fast build rule for target.
lib/fast:
	$(MAKE) -f CMakeFiles/lib.dir/build.make CMakeFiles/lib.dir/build
.PHONY : lib/fast

#=============================================================================
# Target rules for targets named rosbuild_clean-test-results

# Build rule for target.
rosbuild_clean-test-results: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rosbuild_clean-test-results
.PHONY : rosbuild_clean-test-results

# fast build rule for target.
rosbuild_clean-test-results/fast:
	$(MAKE) -f CMakeFiles/rosbuild_clean-test-results.dir/build.make CMakeFiles/rosbuild_clean-test-results.dir/build
.PHONY : rosbuild_clean-test-results/fast

#=============================================================================
# Target rules for targets named rosbuild_precompile

# Build rule for target.
rosbuild_precompile: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rosbuild_precompile
.PHONY : rosbuild_precompile

# fast build rule for target.
rosbuild_precompile/fast:
	$(MAKE) -f CMakeFiles/rosbuild_precompile.dir/build.make CMakeFiles/rosbuild_precompile.dir/build
.PHONY : rosbuild_precompile/fast

#=============================================================================
# Target rules for targets named rosbuild_premsgsrvgen

# Build rule for target.
rosbuild_premsgsrvgen: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rosbuild_premsgsrvgen
.PHONY : rosbuild_premsgsrvgen

# fast build rule for target.
rosbuild_premsgsrvgen/fast:
	$(MAKE) -f CMakeFiles/rosbuild_premsgsrvgen.dir/build.make CMakeFiles/rosbuild_premsgsrvgen.dir/build
.PHONY : rosbuild_premsgsrvgen/fast

#=============================================================================
# Target rules for targets named rospack_genmsg

# Build rule for target.
rospack_genmsg: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_genmsg
.PHONY : rospack_genmsg

# fast build rule for target.
rospack_genmsg/fast:
	$(MAKE) -f CMakeFiles/rospack_genmsg.dir/build.make CMakeFiles/rospack_genmsg.dir/build
.PHONY : rospack_genmsg/fast

#=============================================================================
# Target rules for targets named rospack_genmsg_all

# Build rule for target.
rospack_genmsg_all: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_genmsg_all
.PHONY : rospack_genmsg_all

# fast build rule for target.
rospack_genmsg_all/fast:
	$(MAKE) -f CMakeFiles/rospack_genmsg_all.dir/build.make CMakeFiles/rospack_genmsg_all.dir/build
.PHONY : rospack_genmsg_all/fast

#=============================================================================
# Target rules for targets named rospack_genmsg_libexe

# Build rule for target.
rospack_genmsg_libexe: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_genmsg_libexe
.PHONY : rospack_genmsg_libexe

# fast build rule for target.
rospack_genmsg_libexe/fast:
	$(MAKE) -f CMakeFiles/rospack_genmsg_libexe.dir/build.make CMakeFiles/rospack_genmsg_libexe.dir/build
.PHONY : rospack_genmsg_libexe/fast

#=============================================================================
# Target rules for targets named rospack_gensrv

# Build rule for target.
rospack_gensrv: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 rospack_gensrv
.PHONY : rospack_gensrv

# fast build rule for target.
rospack_gensrv/fast:
	$(MAKE) -f CMakeFiles/rospack_gensrv.dir/build.make CMakeFiles/rospack_gensrv.dir/build
.PHONY : rospack_gensrv/fast

#=============================================================================
# Target rules for targets named run_tests

# Build rule for target.
run_tests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 run_tests
.PHONY : run_tests

# fast build rule for target.
run_tests/fast:
	$(MAKE) -f CMakeFiles/run_tests.dir/build.make CMakeFiles/run_tests.dir/build
.PHONY : run_tests/fast

#=============================================================================
# Target rules for targets named test

# Build rule for target.
test: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test
.PHONY : test

# fast build rule for target.
test/fast:
	$(MAKE) -f CMakeFiles/test.dir/build.make CMakeFiles/test.dir/build
.PHONY : test/fast

#=============================================================================
# Target rules for targets named test-future

# Build rule for target.
test-future: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test-future
.PHONY : test-future

# fast build rule for target.
test-future/fast:
	$(MAKE) -f CMakeFiles/test-future.dir/build.make CMakeFiles/test-future.dir/build
.PHONY : test-future/fast

#=============================================================================
# Target rules for targets named test-results

# Build rule for target.
test-results: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test-results
.PHONY : test-results

# fast build rule for target.
test-results/fast:
	$(MAKE) -f CMakeFiles/test-results.dir/build.make CMakeFiles/test-results.dir/build
.PHONY : test-results/fast

#=============================================================================
# Target rules for targets named test-results-run

# Build rule for target.
test-results-run: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 test-results-run
.PHONY : test-results-run

# fast build rule for target.
test-results-run/fast:
	$(MAKE) -f CMakeFiles/test-results-run.dir/build.make CMakeFiles/test-results-run.dir/build
.PHONY : test-results-run/fast

#=============================================================================
# Target rules for targets named tests

# Build rule for target.
tests: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 tests
.PHONY : tests

# fast build rule for target.
tests/fast:
	$(MAKE) -f CMakeFiles/tests.dir/build.make CMakeFiles/tests.dir/build
.PHONY : tests/fast

#=============================================================================
# Target rules for targets named gtest

# Build rule for target.
gtest: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 gtest
.PHONY : gtest

# fast build rule for target.
gtest/fast:
	$(MAKE) -f gtest/CMakeFiles/gtest.dir/build.make gtest/CMakeFiles/gtest.dir/build
.PHONY : gtest/fast

#=============================================================================
# Target rules for targets named gtest_main

# Build rule for target.
gtest_main: cmake_check_build_system
	$(MAKE) -f CMakeFiles/Makefile2 gtest_main
.PHONY : gtest_main

# fast build rule for target.
gtest_main/fast:
	$(MAKE) -f gtest/CMakeFiles/gtest_main.dir/build.make gtest/CMakeFiles/gtest_main.dir/build
.PHONY : gtest_main/fast

src/get_circle.o: src/get_circle.cpp.o
.PHONY : src/get_circle.o

# target to build an object file
src/get_circle.cpp.o:
	$(MAKE) -f CMakeFiles/circle_detection.dir/build.make CMakeFiles/circle_detection.dir/src/get_circle.cpp.o
.PHONY : src/get_circle.cpp.o

src/get_circle.i: src/get_circle.cpp.i
.PHONY : src/get_circle.i

# target to preprocess a source file
src/get_circle.cpp.i:
	$(MAKE) -f CMakeFiles/circle_detection.dir/build.make CMakeFiles/circle_detection.dir/src/get_circle.cpp.i
.PHONY : src/get_circle.cpp.i

src/get_circle.s: src/get_circle.cpp.s
.PHONY : src/get_circle.s

# target to generate assembly for a file
src/get_circle.cpp.s:
	$(MAKE) -f CMakeFiles/circle_detection.dir/build.make CMakeFiles/circle_detection.dir/src/get_circle.cpp.s
.PHONY : src/get_circle.cpp.s

src/imgproc.o: src/imgproc.cpp.o
.PHONY : src/imgproc.o

# target to build an object file
src/imgproc.cpp.o:
	$(MAKE) -f CMakeFiles/circle_detection.dir/build.make CMakeFiles/circle_detection.dir/src/imgproc.cpp.o
	$(MAKE) -f CMakeFiles/lib.dir/build.make CMakeFiles/lib.dir/src/imgproc.cpp.o
.PHONY : src/imgproc.cpp.o

src/imgproc.i: src/imgproc.cpp.i
.PHONY : src/imgproc.i

# target to preprocess a source file
src/imgproc.cpp.i:
	$(MAKE) -f CMakeFiles/circle_detection.dir/build.make CMakeFiles/circle_detection.dir/src/imgproc.cpp.i
	$(MAKE) -f CMakeFiles/lib.dir/build.make CMakeFiles/lib.dir/src/imgproc.cpp.i
.PHONY : src/imgproc.cpp.i

src/imgproc.s: src/imgproc.cpp.s
.PHONY : src/imgproc.s

# target to generate assembly for a file
src/imgproc.cpp.s:
	$(MAKE) -f CMakeFiles/circle_detection.dir/build.make CMakeFiles/circle_detection.dir/src/imgproc.cpp.s
	$(MAKE) -f CMakeFiles/lib.dir/build.make CMakeFiles/lib.dir/src/imgproc.cpp.s
.PHONY : src/imgproc.cpp.s

# Help Target
help:
	@echo "The following are some of the valid targets for this Makefile:"
	@echo "... all (the default if no target is provided)"
	@echo "... clean"
	@echo "... depend"
	@echo "... ROSBUILD_genmsg_cpp"
	@echo "... ROSBUILD_genmsg_py"
	@echo "... ROSBUILD_gensrv_cpp"
	@echo "... _catkin_empty_exported_target"
	@echo "... circle_detection"
	@echo "... clean_test_results"
	@echo "... doxygen"
	@echo "... edit_cache"
	@echo "... install"
	@echo "... install/local"
	@echo "... install/strip"
	@echo "... lib"
	@echo "... list_install_components"
	@echo "... rebuild_cache"
	@echo "... rosbuild_clean-test-results"
	@echo "... rosbuild_precompile"
	@echo "... rosbuild_premsgsrvgen"
	@echo "... rospack_genmsg"
	@echo "... rospack_genmsg_all"
	@echo "... rospack_genmsg_libexe"
	@echo "... rospack_gensrv"
	@echo "... run_tests"
	@echo "... test"
	@echo "... test-future"
	@echo "... test-results"
	@echo "... test-results-run"
	@echo "... tests"
	@echo "... gtest"
	@echo "... gtest_main"
	@echo "... src/get_circle.o"
	@echo "... src/get_circle.i"
	@echo "... src/get_circle.s"
	@echo "... src/imgproc.o"
	@echo "... src/imgproc.i"
	@echo "... src/imgproc.s"
.PHONY : help



#=============================================================================
# Special targets to cleanup operation of make.

# Special rule to run CMake to check the build system integrity.
# No rule that depends on this can have commands that come from listfiles
# because they might be regenerated.
cmake_check_build_system:
	$(CMAKE_COMMAND) -H$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR) --check-build-system CMakeFiles/Makefile.cmake 0
.PHONY : cmake_check_build_system

