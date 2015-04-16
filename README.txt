                            ViSP-2.11.0
                     Visual Servoing Platform

      Copyright (C) 2005 - 2015 by INRIA. All rights reserved.
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

   The make install step is optional. You can use ViSP from build tree 
   or from install tree in your own projects. To do the installation, 
   you need to be root to install in the default /usr/local folder.

2.2 Out-of-source build under Unix platforms:
    
   The out-of-source build is very useful for different parallel build
   configurations:

   mkdir <visp build dir>
   cd <visp build dir>
   ccmake <visp source dir>   
     or for defaults:  cmake <source dir>
   make
   make install

   The make install step is optional. You can use ViSP from build tree 
   or from install tree in your own projects. To do the installation, 
   you need to be root to install in the default /usr/local folder.


2.3 Build under Win32

   Use cmake or CMakeSetup Gui for in-source or out-of-source build.
   Set environment variables and proceed as described for Unix.


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


5. Use ViSP with another cmake project:
   -----------------------------------
   Include the following into your CMakeList.txt:

   find_package(VISP REQUIRED)
   if(VISP_FOUND)
     include(${VISP_USE_FILE})
   endif()
   
   With CMake GUI set the environment variable VISP_DIR to the location 
   of the VISPConfig.cmake file. This file could be found either in 
   ViSP build tree or in ViSP install tree. 
   
   For example, on a Unix 64 bits, it will be found in 
   <visp install prefix>/lib/x86_64-linux-gnu/cmake/visp
   
   or using a command line run:
   `cmake -DVISP_DIR=<visp install prefix>/lib/x86_64-linux-gnu/cmake/visp <project source dir>`


6. Use ViSP with autoconf (autotools):
   -----------------------------------
   Copy the M4 macro file `macros/have_visp.m4` in your project. 

   cp macros/have_visp.m4 <your project home dir>/macro
   
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
      

7. Use ViSP with a common Makefile:
   -------------------------------
   a: Install ViSP: 
      - configure ViSP using cmake, particularly set the ViSP install prefix
        (cmake -DCMAKE_INSTALL_PREFIX=<visp install prefix>)
      - build ViSP: 
          make
      - install ViSP by:
          make install
   b: Make sure, the visp-config shell script is found by the makefile:
	        export PATH=$PATH:<visp install prefix>/bin
   c: Use `visp-config --prefix/--cflags/--libs/--version`
      to get the compiler/linker flags needed to use ViSP
         
  
8. View compiler and linker options used:
   -------------------------------------
   cmake -DCMAKE_VERBOSE_MAKEFILE=ON <visp source dir>

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
   cd <visp build dir>
   rm CMakeCache.txt

11. Where to found documentation about CMake:
    ----------------------------------------
   Almost complete documentation of cmake commands: 
   cmake --help-full  and  http://www.cmake.org

   The 250-page book Mastering CMake by Ken Martin and Bill Hoffman, ISBN
   1-930934-16-5, published by Kitware, Inc.

------------------------

Fabien Spindler


