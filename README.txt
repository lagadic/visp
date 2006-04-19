                            ViSP-2
                     Visual Servoing Platform

         Copyright Projet Lagadic / IRISA-INRIA Rennes, 2006
                  www: http://www.irisa.fr/lagadic



This project is using the CMake build system.

CMake is a complete stand-alone platform-independant build-system 
replacing autotools (autoconf/autoheader/automake/libtools) completely. 
It depends just on installed cmake (tested with version 2.2.x)
See http://www.cmake.org for details.

USAGE: 
=====

1. Install the newest cmake from www.cmake.org
2. Check out/install ViSP source code

--- Linux ---
cd <visp source dir>;  
ccmake .  
  or for defaults:  cmake .
make
make install

--- WIN32 ---
Use cmake to set environment variables and proceed as described for Linux.



Important Files:
===============

- CMakeLists.txt:
    Overall configuration file. Detect third party libraries (X11, GTK, ...).

- CMakeHeaderFileList.cmake:
    The list of all the headers (*.h) or (*.t.cpp for templates) to copy
    in <visp source dir>/include/visp directory or to install in 
    <install prefix>/include/visp

- CMakeSourceFileList.cmake:
    The list of all the sources (*.cpp) used to build the ViSP library.

- src/CMakeLists.txt:
    Build the library in <visp source dir>/lib.

- include/vpConfig.h.in versus include/visp/vpConfig.h:
    vpConfig.h.in is the source for vpConfig.h.
    vpConfig.h is generated in CMakeList.txt by the command
      CONFIGURE_FILE(${VISP_SOURCE_DIR}/include/vpConfig.h.cmake 
                     ${VISP_INCLUDE_DIR}/vpConfig.h
    vpConfig.h contains preprocessor defines like VISP_HAVE_X11 and is .
    included in most header files (*.h) to allow system dependent code usage 

- CMakeModule/visp-config.in versus bin/visp-config:
    visp-config is a shell script for third party projects not using cmake to
    get hold of prefix, cflags, libs and version. 
    It is generated from visp-config.in by the module 
    CMakeModules/GenerateConfigScript.cmake in CMakeList.txt.

- lib/VISPBuildSettings.cmake:
    SET commands for most important variables, i.e.
      PROJECT_NAME
      CMAKE_VERSION
      C(XX)_COMPILER
      C(XX)_FLAGS(_*)
      BUILD_TYPE
      BUILD_TOOL
    It can be included from third cmake projects.

- CMakeModules/VISPConfig.cmake.in / lib/VISPConfig.cmake:
    VISPConfig.cmake contains SET command for some basic variables like
      VISP_INCLUDE_DIR, VISP_LINK_DIRECTORIES, VISP_BUILD_SETTINGS_FILE,
      VISP_SOURCE_DIR, VISP_LIBRARIES and VISP_USE_FILE
    VISPConfig.cmake can be used by by third cmake projects to make these
      variables available.
    VISPConfig.cmake.in is the source for VISPConfig.cmake


HOWTO:
======

1. Change configuration:
   ---------------------

   You can change configuration easily with ccmake GUI.   
   ccmake <source dir> 
   type 't' to toggle display of advanced variables
   or 
   cmake -LA <source dir>
   to have the list of all configuration variables.


2. Do a optimized build:
   --------------------
   cmake -DCMAKE_BUILD_TYPE:STRING=Release <visp source dir>

   or use ccmake <visp source dir> to set CMAKE_BUILD_TYPE.

   Available build types are: Release, Debug, RelWithDebInfo, MinSizeRel.
   The build process uses specific build variable CMAKE_CXX_FLAGS_*.
   For example Release build uses CMAKE_CXX_FLAGS_RELEASE, while Debug 
   build uses CMAKE_CXX_FLAGS_DEBUG.

   The flags according to the different build types can be seen with
      cmake -LA <source_dir>

3. Build shared libraries:
   -----------------------
   cmake -DBUILD_SHARED_LIBS:STRING=ON <visp source dir>

   or use ccmake <visp source dir> to set BUILD_SHARED_LIBS.

   Produce shared libraries (.so).

4. Build static libraries:
   -----------------------
   cmake -DBUILD_SHARED_LIBS:STRING=OFF <visp source dir>

   or use ccmake <visp source dir> to set BUILD_SHARED_LIBS.

   Produce static libraries (.a).

5. Add more files to clean target:
   ------------------------------
   SET_DIRECTORY_PROPERTIES(PROPERTIES ADDITIONAL_MAKE_CLEAN_FILES 
                            "file_to_remove")


6. Use ViSP with another cmake project:
   -----------------------------------
   Include the following into your CMakeList.txt:

   FIND_PACKAGE(VISP REQUIRED)
   IF(VISP_FOUND)
     INCLUDE(${VISP_USE_FILE})
   ENDIF(VISP_FOUND)
   
   Set the environment variable VISP_DIR or
   issue `cmake -DVISP_DIR=<path to visp library> <example source dir>`


7. Use ViSP with a common Makefile:
   -------------------------------
   a: Install ViSP: 
      - configure ViSP using cmake, particularly set the install prefix
        (cmake -DCMAKE_INSTALL_PREFIX:PATH=<install prefix>)
      - build ViSP: make
      - install ViSP by:
           make install
   b: Make sure, the visp-config shell script is found by the makefile:
	   export PATH=$PATH:<install prefix>/bin
   c: Use `visp-config --prefix/--cflags/--libs/--version]`
           to get the compiler/linker flags needed to use ViSP
         
  
8. View compiler and linker options used:
   -------------------------------------
   cmake -DCMAKE_VERBOSE_MAKEFILE:BOOL=ON <visp source dir>

   Generate more 'verbose' makefile including compiler calls instead of 
   default "SILENT" .

9. View all available options/variables:
   ------------------------------------
   ccmake <source dir> 
   type 't' to toggle display of advanced variables
   or 
   cmake -LA <source dir>

10. Remove Cache:
    -------------
   cd <build dir>
   rm CMakeCache.txt


11. Where to found doc about CMake:
    ------------------------------
   Almost complete documentation of cmake commands: 
   cmake --help-full  and  http://www.cmake.org

------------------------

Fabien Spindler


