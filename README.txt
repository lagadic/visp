                            ViSP-2.10.0
                     Visual Servoing Platform

      Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
                  www: http://www.irisa.fr/lagadic



This project is using the CMake build system.

CMake is a complete stand-alone platform-independant build-system 
replacing autotools (autoconf/autoheader/automake/libtools) completely. 
It depends just on installed cmake (tested with cmake cvs version). It
needs a cmake 2.6.x or more recent version of cmake.
See http://www.cmake.org for details.

USAGE: 
=====

1. Install the newest cmake from www.cmake.org

   See INSTALL file for further information.

2. Check out/install ViSP source code

   Checkout, copy or unzip the ViSP tar ball in <visp source dir>.

   For building you have got the option of "in-source" or "out-of-source"
   builds. CMake developpers strongly recommend using an "out-of-source" build
   which never writes any files to the source tree. Using a separate source and
   build tree greatly reduces the need for "make clean" and "make distclean"
   targets.

2.1 In-source build under Unix platforms:

   cd <visp source dir>;  
   ccmake .  
     or for defaults:  cmake .
   make
   make install

   The make install step is needed if you want to use ViSP in your own
   projects. To do the installation, you don't need to be root.

2.2 Out-of-source build under Unix platforms:
    
   The out-of-source build is very useful for different parallel build
   configurations:

   mkdir <visp build dir>
   cd <visp build dir>
   ccmake <visp source dir>   
     or for defaults:  cmake <source dir>
   make
   make install

   The make install step is needed if you want to use ViSP in your own
   projects. To do the installation, you don't need to be root.


2.3 Build under Win32

   Use cmake or CMakeSetup Gui for in-source or out-of-source build.
   Set environment variables and proceed as described for Unix.



Important Files:
===============

- CMakeLists.txt:
    Overall configuration file. Detect third party libraries (X11, GTK, ...).

- CMakeHeaderFileList.cmake:
    The list of all the headers (*.h) or (*.t.cpp for templates) to copy
    in <visp source dir>/include/visp directory or to install in 
    <visp install prefix>/include/visp

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
   cmake -DCMAKE_BUILD_TYPE=Release <visp source dir>

   or use ccmake <visp source dir> to set CMAKE_BUILD_TYPE.

   Available build types are: Release, Debug, RelWithDebInfo, MinSizeRel.
   The build process uses specific build variable CMAKE_CXX_FLAGS_*.
   For example Release build uses CMAKE_CXX_FLAGS_RELEASE, while Debug 
   build uses CMAKE_CXX_FLAGS_DEBUG.

   The flags according to the different build types can be seen with
      cmake -LA <source_dir>


3. Build shared libraries:
   -----------------------
   cmake -DBUILD_SHARED_LIBS=ON <visp source dir>

   or use ccmake <visp source dir> to set BUILD_SHARED_LIBS.

   Produce shared libraries (.so).


4. Build static libraries:
   -----------------------
   cmake -DBUILD_SHARED_LIBS=OFF <visp source dir>

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
   
   With CMake GUI set the environment variable VISP_DIR to 
   <visp install prefix>/lib
   or using a command line run:
   `cmake -DVISP_DIR=<visp install prefix>/lib <project source dir>`


7. Use ViSP with autoconf (autotools):
   -----------------------------------
   Copy the M4 macro file `macro/have_visp.m4` in your project. 

   cp macro/have_visp.m4 <your project home dir>/macro
   
   This macro check if <visp install prefix>/bin/visp-config shell script
   is available (see below how to use ViSP with a common Makefile).

   In your configure.ac project file add lines like:

   AC_HAVE_VISP_IFELSE(have_visp=yes,have_visp=no)
   if test "x$have_visp" = "xyes"; then
     CXXFLAGS="$CXXFLAGS $ac_visp_cflags "
     LIBS="$LIBS $ac_visp_libs "
   fi

   cd <your project home dir>
   aclocal -I macro # updates the aclocal.m4 file
   autoconf
   configure --with-visp-install-bin=<visp install prefix>/bin
      

8. Use ViSP with a common Makefile:
   -------------------------------
   a: Install ViSP: 
      - configure ViSP using cmake, particularly set the ViSP install prefix
        (cmake -DCMAKE_INSTALL_PREFIX:PATH=<visp install prefix>)
      - build ViSP: 
	  make
      - install ViSP by:
          make install
   b: Make sure, the visp-config shell script is found by the makefile:
	   export PATH=$PATH:<visp install prefix>/bin
   c: Use `visp-config --prefix/--cflags/--libs/--version`
           to get the compiler/linker flags needed to use ViSP
         
  
9. View compiler and linker options used:
   -------------------------------------
   cmake -DCMAKE_VERBOSE_MAKEFILE=ON <visp source dir>

   Generate more 'verbose' makefile including compiler calls instead of 
   default "SILENT" .


10. View all available options/variables:
   ------------------------------------
   ccmake <source dir> 
   type 't' to toggle display of advanced variables
   or 
   cmake -LA <source dir>


11. Remove Cache:
    -------------
   cd <visp build dir>
   rm CMakeCache.txt


12. Make a distclean:
    -----------------
  CMake does not generate a "make distclean" target (see the cmake FAQ).  CMake
  generates many files related to the build system, but since CMakeLists.txt
  files can run scripts and other arbitrary commands, there is no way it can
  keep track of exactly which files are generated as part of running CMake.


  So, for in-source build under Unix platforms, we have developped a distclean
  shell script (working only under Unix platforms) that removes these files
  related to the build system.  To remove some intermediate files related to
  the build system:  sh ./distclean.sh

  For out-of-source build, just remove the <visp build dir> tree.

13. Where to found documentation about CMake:
    ----------------------------------------
   Almost complete documentation of cmake commands: 
   cmake --help-full  and  http://www.cmake.org

   The 250-page book Mastering CMake by Ken Martin and Bill Hoffman, ISBN
   1-930934-16-5, published by Kitware, Inc.

------------------------

Fabien Spindler


