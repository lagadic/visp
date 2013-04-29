#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
# 
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact INRIA about acquiring a ViSP Professional 
# Edition License.
#
# See http://www.irisa.fr/lagadic/visp/visp.html for more information.
# 
# This software was developed at:
# INRIA Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
# http://www.irisa.fr/lagadic
#
# If you have questions regarding the use of this file, please contact
# INRIA at visp@inria.fr
# 
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# ViSP overall configuration file. Detect third party libraries (X11, GTK, ...)
#
# Authors:
# Fabien Spindler
#
#############################################################################

PROJECT(VISP C CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

#-----------------------------------------------------------------------------
# VISP version number.  An even minor number corresponds to releases.
SET(VISP_VERSION_MAJOR "2")
SET(VISP_VERSION_MINOR "7")
SET(VISP_VERSION_PATCH "1")
SET(VISP_VERSION "${VISP_VERSION_MAJOR}.${VISP_VERSION_MINOR}.${VISP_VERSION_PATCH}")
# Package revision number
SET(VISP_REVISION "1")

# where are user-specific cmake modules
set(VISP_CMAKE_MODULE_PATH ${VISP_SOURCE_DIR}/CMakeModules)

find_file(GNU_INSTALL_DIRS_FROM_CMAKE ${CMAKE_ROOT}/Modules/GNUInstallDirs.cmake)
mark_as_advanced(GNU_INSTALL_DIRS_FROM_CMAKE)
if(GNU_INSTALL_DIRS_FROM_CMAKE)
  include(${CMAKE_ROOT}/Modules/GNUInstallDirs.cmake)
else()
  include(${VISP_CMAKE_MODULE_PATH}/GNUInstallDirs.cmake)
endif()

list(APPEND CMAKE_MODULE_PATH ${VISP_CMAKE_MODULE_PATH})

# the include directory we depend on
SET(VISP_INTERN_INCLUDE_DIR ${VISP_BINARY_DIR}/${CMAKE_INSTALL_INCLUDEDIR})
SET(VISP_INCLUDE_DIR "${VISP_INTERN_INCLUDE_DIR}/visp")
SET(VISP_DOC_DIR "${VISP_BINARY_DIR}/doc")

# The location in which to install VISP libraries.
SET(LIBRARY_OUTPUT_PATH ${VISP_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
# The location in which to install some VISP binaries.
SET(BINARY_OUTPUT_PATH ${VISP_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})

# the third party include directories we depend on
SET(VISP_EXTERN_INCLUDE_DIRS "")
# the internal visp libs we depend on
SET(VISP_INTERN_LIBRARY visp)
# the third party libraries we depend on
SET(VISP_EXTERN_LIBRARIES "")
# the compiler definitions
SET(VISP_DEFS "")

if(WIN32)
    # Postfix of .lib and .dll
    set(CMAKE_DEBUG_POSTFIX "d")
endif()

# Get the OS
SET(OS ${CMAKE_SYSTEM_NAME})

SET(OGRE_HOME $ENV{OGRE_HOME})
if(OGRE_HOME)
  # replace \ with / especially for windows
  STRING(REGEX REPLACE "\\\\" "/" OGRE_HOME ${OGRE_HOME}) 
endif()

# add the path to detect Ogre3D
if(WIN32)
  list(APPEND CMAKE_MODULE_PATH "${OGRE_HOME}/CMake")
endif(WIN32)
 
if(UNIX)
  list(APPEND CMAKE_MODULE_PATH "${OGRE_HOME}/cmake")
  list(APPEND CMAKE_MODULE_PATH "${OGRE_HOME}/CMake")
  list(APPEND CMAKE_MODULE_PATH "/usr/local/lib/OGRE/cmake")
  list(APPEND CMAKE_MODULE_PATH "/usr/lib/OGRE/cmake")
  list(APPEND CMAKE_MODULE_PATH "/usr/local/lib64/OGRE/cmake")
  list(APPEND CMAKE_MODULE_PATH "/usr/lib64/OGRE/cmake")
  list(APPEND CMAKE_MODULE_PATH "/usr/share/OGRE/cmake/modules")
endif(UNIX)

# Create include directory which will contain all the headers
MAKE_DIRECTORY(${VISP_INCLUDE_DIR})

#-----------------------------------------------------------------------------
# Add extra compilation flags under UNIX
#-----------------------------------------------------------------------------
include(${VISP_CMAKE_MODULE_PATH}/AddExtraCompilationFlags.cmake)
ADD_EXTRA_COMPILATION_FLAGS()

#--------------------------------------------------------------------
# By default set release configuration
#--------------------------------------------------------------------
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE String "Choose the type of build, options are: None Debug Release" FORCE)
endif()

#--------------------------------------------------------------------
# Option management
#--------------------------------------------------------------------

# Choose static or shared libraries.
OPTION(BUILD_SHARED_LIBS "Build ViSP shared libraries (.dll/.so) instead of static ones (.lib/.a)." OFF)
# Build examples as an option.
OPTION(BUILD_EXAMPLES "Build ViSP examples." ON)
# Build demos as an option.
OPTION(BUILD_DEMOS "Build ViSP demos." ON)
# Build demos as an option.
OPTION(BUILD_TUTORIAL "Build ViSP tutorial." ON)
# Build deprecated functions as an option.
OPTION(BUILD_DEPRECATED_FUNCTIONS "Build deprecated functionalities." ON)

# Note that it is better to set MOMENTS_COMBINE_MATRICES to OFF
OPTION(MOMENTS_COMBINE_MATRICES "Use linear combination of matrices instead of linear combination of moments to compute interaction matrices." OFF)
MARK_AS_ADVANCED(MOMENTS_COMBINE_MATRICES)

OPTION(ENABLE_TEST_WITHOUT_DISPLAY "Don't use display features when testing" ON)
MARK_AS_ADVANCED(ENABLE_TEST_WITHOUT_DISPLAY)

IF(ENABLE_TEST_WITHOUT_DISPLAY)
  SET(OPTION_TO_DESACTIVE_DISPLAY "-d")
ENDIF()

IF(BUILD_DEPRECATED_FUNCTIONS)
  SET(VISP_BUILD_DEPRECATED_FUNCTIONS TRUE) # for header vpConfig.h
ENDIF()

IF(MOMENTS_COMBINE_MATRICES)
  SET(VISP_MOMENTS_COMBINE_MATRICES TRUE) # for header vpConfig.h
ENDIF()


IF (UNIX)
  FIND_PACKAGE(DC1394)
  IF(DC1394_2_FOUND)
    OPTION(USE_DC1394_2    "Compile ViSP with the libdc1394.2 library" ON)
  ELSE(DC1394_2_FOUND)
    SET(USE_DC1394_2 OFF)
  ENDIF(DC1394_2_FOUND)
  IF(DC1394_1_FOUND)
    OPTION(USE_DC1394_1    "Compile ViSP with the libdc1394.1 library" ON)
  ELSE(DC1394_1_FOUND)
    SET(USE_DC1394_1 OFF)
  ENDIF(DC1394_1_FOUND)

  FIND_PACKAGE(V4L2)
  IF(V4L2_FOUND)
    OPTION(USE_V4L2    "Compile ViSP with the v4l2 (video for linux 2) library" ON)
  ELSE(V4L2_FOUND)
    SET(USE_V4L2 OFF)
  ENDIF(V4L2_FOUND)

  IF(NOT RAW1394_FOUND)
    FIND_PACKAGE(RAW1394)
  ENDIF(NOT RAW1394_FOUND)
  FIND_PACKAGE(RT)
  FIND_PACKAGE(CALINUX)
  FIND_PACKAGE(IRISA)
  IF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    OPTION(USE_AFMA4   "Compile ViSP for Afma4 robot usage at Irisa" ON)
    OPTION(USE_AFMA6   "Compile ViSP for Afma6 robot usage at Irisa" ON)
    OPTION(USE_VIPER650 "Compile ViSP for Viper S650 robot usage at Irisa" ON)
    OPTION(USE_VIPER850 "Compile ViSP for Viper S850 robot usage at Irisa" ON)
  ELSE(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    SET(USE_AFMA4 OFF)
    SET(USE_AFMA6 OFF)
    SET(USE_VIPER650 OFF)
    SET(USE_VIPER850 OFF)
  ENDIF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)

  FIND_PACKAGE(BICLOPS)
  IF(BICLOPS_FOUND)
    OPTION(USE_BICLOPS "Compile ViSP for Biclops head usage at Irisa" ON)
  ELSE(BICLOPS_FOUND)
    SET(USE_BICLOPS OFF)
  ENDIF(BICLOPS_FOUND)

  FIND_PACKAGE(PTU46)
  IF(PTU46_FOUND)
    OPTION(USE_PTU46 "Compile ViSP for Ptu-46 head usage at Irisa" ON)
  ELSE(PTU46_FOUND)
    SET(USE_PTU46 OFF)
  ENDIF(PTU46_FOUND)

  FIND_PACKAGE(CycabTk)
  IF(CycabTk_FOUND)
    OPTION(USE_CYCAB "Compile ViSP for Cycab car-like mobile robot at Irisa" OFF)
  ELSE(CycabTk_FOUND)
    SET(USE_CYCAB OFF)
  ENDIF(CycabTk_FOUND)
 
ENDIF(UNIX)

IF(WIN32)
  FIND_PACKAGE(CMU1394)
  IF(CMU1394_FOUND)
    OPTION(USE_CMU1394    "Compile ViSP with the CMU 1394 Digital Camera SDK" ON)
  ELSE(CMU1394_FOUND)
    SET(USE_CMU1394 OFF)
  ENDIF(CMU1394_FOUND)

  FIND_PACKAGE(GDI)
  IF(GDI_FOUND)
    OPTION(USE_GDI    "Manages the display on windows" ON)
  ELSE(GDI_FOUND)
    SET(USE_GDI OFF)
  ENDIF(GDI_FOUND)

  FIND_PACKAGE(DIRECT3D)
  IF(DIRECT3D_FOUND)
    OPTION(USE_DIRECT3D    "Manages the display on windows" ON)
  ELSE(DIRECT3D_FOUND)
    SET(USE_DIRECT3D OFF)
  ENDIF(DIRECT3D_FOUND)

  FIND_PACKAGE(DIRECTSHOW)
  IF(DIRECTSHOW_FOUND)
    OPTION(USE_DIRECTSHOW    "Manages the frame grabbing on windows" ON)
  ELSE(DIRECTSHOW_FOUND)
    SET(USE_DIRECTSHOW OFF)
  ENDIF(DIRECTSHOW_FOUND)
  
ENDIF(WIN32)

include(FindOpenMP)
if(OPENMP_FOUND)
    OPTION(USE_OPENMP    "Add C++ compiler flags for OpenMP parallization" ON)
else(OPENMP_FOUND)
    SET(USE_OPENMP OFF)
endif(OPENMP_FOUND)

include(FindCPP11)
if(CPP11_FOUND)
    OPTION(USE_CPP11    "Add C++ compiler flags for C++11 support" OFF)
else()
    SET(USE_CPP11 OFF)
endif()

# Since the FindLAPACK.cmake provided with CMake is for Fortran language,
# in CMakeModules we have added FindLAPACK_C.cmake for C language
FIND_PACKAGE(LAPACK_C)
IF(LAPACK_FOUND)
    OPTION(USE_LAPACK    "Compile ViSP with the lapack/blas libraries" ON)
ELSE(LAPACK_FOUND)
    SET(USE_LAPACK OFF)
ENDIF(LAPACK_FOUND)

IF(NOT USE_LAPACK)
  FIND_PACKAGE(GSL)
  IF(GSL_FOUND)
    OPTION(USE_GSL    "Compile ViSP with the GSL library" ON)
  ELSE(GSL_FOUND)
    SET(USE_GSL OFF)
  ENDIF(GSL_FOUND)
ENDIF()

INCLUDE (${CMAKE_ROOT}/Modules/FindOpenGL.cmake)
FIND_PACKAGE(OpenGL)
FIND_PACKAGE(Coin3D)
if (NOT COIN3D_FOUND)
  FIND_PACKAGE(MyCoin3D)
endif()
IF (COIN3D_FOUND AND OPENGL_FOUND)
  OPTION(USE_COIN    "Compile ViSP with Coin3D" ON)
ELSE (COIN3D_FOUND AND OPENGL_FOUND)
  SET(USE_COIN OFF)
ENDIF (COIN3D_FOUND AND OPENGL_FOUND)

FIND_PACKAGE(YARP QUIET)
MARK_AS_ADVANCED(YARP_DIR)
IF (YARP_FOUND)
  OPTION(USE_YARP "Compile ViSP with YARP" ON)
ELSE (YARP_FOUND)
  SET(USE_YARP OFF)
ENDIF ()

FIND_PACKAGE(OGRE QUIET)
MARK_AS_ADVANCED(OGRE_DIR)
IF (OGRE_FOUND)
  OPTION(USE_OGRE    "Compile ViSP with Ogre3D engine and renderer" ON)
ELSE ()
  SET(USE_OGRE OFF)
ENDIF ()
 
FIND_PACKAGE(OIS QUIET)
MARK_AS_ADVANCED(OIS_DIR)
IF (OGRE_FOUND AND OIS_FOUND)
  OPTION(USE_OIS    "Compile ViSP with OIS (Object Oriented Input System library)" ON)
ELSE ()
  SET(USE_OIS OFF)
ENDIF ()

FIND_PACKAGE(LIBFREENECT)
IF(LIBFREENECT_FOUND)
  OPTION(USE_LIBFREENECT    "Compile ViSP with the libfreenect library" ON)
ELSE(LIBFREENECT_FOUND)
  SET(USE_LIBFREENECT OFF)
ENDIF(LIBFREENECT_FOUND)

FIND_PACKAGE(LIBUSB-1)
IF(LIBUSB_1_FOUND)
  OPTION(USE_LIBUSB_1    "Compile ViSP with the libusb-1.0 library" ON)
ELSE(LIBUSB_1_FOUND)
  SET(USE_LIBUSB_1 OFF)
ENDIF(LIBUSB_1_FOUND)

IF(WIN32 AND USE_COIN)
  FIND_PACKAGE(SOWIN)
  IF (SOWIN_FOUND)
    OPTION(USE_SOWIN    "Compile ViSP with SoWin" OFF)
  ELSE (SOWIN_FOUND)
    SET(USE_SOWIN OFF)
  ENDIF (SOWIN_FOUND)
ENDIF(WIN32 AND USE_COIN)

IF(USE_COIN)
  FIND_PACKAGE(SOQT)
  IF (SOQT_FOUND)
    OPTION(USE_SOQT    "Compile ViSP with SoQt" OFF)
  ELSE (SOQT_FOUND)
    SET(USE_SOQT OFF)
  ENDIF (SOQT_FOUND)
  IF(USE_SOQT)
    INCLUDE (${CMAKE_ROOT}/Modules/FindQt.cmake) 
    IF (QT_FOUND)
      OPTION(USE_QT    "Compile ViSP with Qt" ON)
    ELSE (QT_FOUND)
	  SET(USE_QT OFF)
    ENDIF (QT_FOUND)
  ENDIF (USE_SOQT)
ENDIF(USE_COIN)

IF(USE_COIN)
  FIND_PACKAGE(SOXT)
  IF (SOXT_FOUND)
    OPTION(USE_SOXT    "Compile ViSP with SoXt" OFF)
  ELSE (SOXT_FOUND)
    SET(USE_SOXT OFF)
  ENDIF (SOXT_FOUND)
ENDIF(USE_COIN)

FIND_PACKAGE(PTHREAD)
IF (PTHREAD_FOUND)
  OPTION(USE_PTHREAD    "Compile ViSP with pthread" ON)
ELSE ()
  SET(USE_PTHREAD OFF)
ENDIF ()

FIND_PACKAGE(XML2)
IF(XML2_FOUND)
  OPTION(USE_XML2    "Compile ViSP with the xml2 library" ON)
ELSE(XML2_FOUND)
  SET(USE_XML2 OFF)
ENDIF(XML2_FOUND)

FIND_PACKAGE(OpenCV QUIET)
MARK_AS_ADVANCED(OpenCV_DIR)
IF(OpenCV_FOUND)
  OPTION(USE_OPENCV    "Compile ViSP with OpenCV >= 2.1 library" ON)
ELSE(OpenCV_FOUND)
  FIND_PACKAGE(OpenCV2 QUIET)
  IF(OpenCV_FOUND)
    OPTION(USE_OPENCV    "Compile ViSP with OpenCV <= 2.0 library" ON)
  ELSE(OpenCV_FOUND)
    SET(USE_OPENCV OFF)
  ENDIF(OpenCV_FOUND)
ENDIF(OpenCV_FOUND)

FIND_PACKAGE(ZLIB)
if(ZLIB_FOUND)
  OPTION(USE_ZLIB    "Compile ViSP with zlib library" ON)
else()
  FIND_PACKAGE(MyZLIB)
  if(ZLIB_FOUND)
    option(USE_ZLIB    "Compile ViSP with zlib library" ON)
  else()
    set(USE_ZLIB OFF)
  endif()
endif()

FIND_PACKAGE(X11)
IF(X11_FOUND)
  OPTION(USE_X11    "Compile ViSP with the X11 library" ON)
ELSE(X11_FOUND)
  SET(USE_X11 OFF)
ENDIF(X11_FOUND)

# The native FindGTK2.cmake doesn't consider libgobject-2.0 that is 
# requested by ViSP. That's why we use our FindMyGTK2.cmake 
#FIND_PACKAGE(GTK2)
#mark_as_advanced(GTK2_ATK_INCLUDE_DIR)
#mark_as_advanced(GTK2_CAIRO_INCLUDE_DIR)
#mark_as_advanced(GTK2_FONTCONFIG_INCLUDE_DIR)
#mark_as_advanced(GTK2_GDK_INCLUDE_DIR)
#mark_as_advanced(GTK2_GDK_PIXBUF_INCLUDE_DIR)
#mark_as_advanced(GTK2_GDKCONFIG_INCLUDE_DIR)
#mark_as_advanced(GTK2_GLIB_INCLUDE_DIR)
#mark_as_advanced(GTK2_GLIBCONFIG_INCLUDE_DIR)
#mark_as_advanced(GTK2_GOBJECT_INCLUDE_DIR)
#mark_as_advanced(GTK2_GTK_INCLUDE_DIR)
#mark_as_advanced(GTK2_PANGO_INCLUDE_DIR)
#IF(GTK2_FOUND)
#  OPTION(USE_GTK2    "Compile ViSP with the gtk2 library" ON)
#ELSE(GTK2_FOUND)
  FIND_PACKAGE(MyGTK2)
  if(GTK2_FOUND)
    OPTION(USE_GTK2    "Compile ViSP with the gtk2 library" OFF)
  else(GTK2_FOUND)
    SET(USE_GTK2 OFF)
  endif(GTK2_FOUND)
#ENDIF(GTK2_FOUND)

FIND_PACKAGE(JPEG)
if(JPEG_FOUND)
  OPTION(USE_LIBJPEG    "Compile ViSP with the libjpeg library" ON)
else(JPEG_FOUND)
  FIND_PACKAGE(MyJPEG)
  if(JPEG_FOUND)
    OPTION(USE_LIBJPEG    "Compile ViSP with the libjpeg library" ON)
  else(JPEG_FOUND)
    SET(USE_LIBJPEG OFF)
  endif()
endif()

FIND_PACKAGE(PNG)
if(PNG_FOUND)
  option(USE_LIBPNG    "Compile ViSP with the libpng library" ON)
else(PNG_FOUND)
  FIND_PACKAGE(MyPNG)
  if(PNG_FOUND)
    OPTION(USE_LIBPNG    "Compile ViSP with the libpng library" ON)
  else(PNG_FOUND)
    SET(USE_LIBPNG OFF)
  endif()
endif()

FIND_PACKAGE(FFMPEG)
IF(FFMPEG_FOUND)
  OPTION(USE_FFMPEG    "Compile ViSP with the ffmpeg library" ON)
ELSE(FFMPEG_FOUND)
  SET(USE_FFMPEG OFF)
ENDIF(FFMPEG_FOUND)

# To control Pioneer mobile robots, under UNIX we need Aria, pthread, rt and dl 3rd party libraries
find_package(ARIA)
if(ARIA_FOUND)
  OPTION(USE_ARIA    "Compile ViSP with Aria library to control Pioneer mobile robots" ON)
else()
  SET(USE_ARIA OFF)
endif()

find_package(RT)
if(RT_FOUND)
  OPTION(USE_RT    "Compile ViSP with rt library" ON)
else()
  SET(USE_RT OFF)
endif()
find_package(DL)
if(DL_FOUND)
  OPTION(USE_DL    "Compile ViSP with dl library" ON)
else()
  SET(USE_DL OFF)
endif()


# Set other options to default value
#SET(USE_X11        ON) # For Linux/OSX display
#SET(USE_XML2       ON) # For the Xml Parser (vpCameraParameters)
#SET(USE_COIN       ON) # For the simulator
#SET(USE_SOWIN      ON) # For the simulator viewer
#SET(USE_SOQT       ON) # For the simulator viewer
#SET(USE_SOXT       ON) # For the simulator viewer
#SET(USE_PTHREAD    ON) # For Biclops/Simulator
#SET(USE_OPENCV     ON) # For Intel opencv
#SET(USE_GSL        ON) # For matrix manipulations
#SET(USE_GTK2       ON) # For Linux/OSX/Windows display with gtk-2.x
#SET(USE_DIRECT3D   ON) # For Windows display
#SET(USE_GDI        ON) # For Windows display
#SET(USE_DC1394_1   ON) # For firewire grabber under Linux and OSX ?
#SET(USE_DC1394_2   ON) # For firewire grabber under Linux and OSX ?
#SET(USE_V4L2       ON) # For Video 4 Linux 2 grabber under Linux
#SET(USE_DIRECTSHOW ON) # For Windows direct show grabber
#SET(USE_LIBJPEG    ON) # For reading jpeg files
#SET(USE_LIBPNG     ON) # For reading png files
#SET(USE_ZLIB       ON) # For data compression
#SET(USE_YARP       ON) # For YARP
#SET(USE_FFMPEG     ON) # For video read and write

IF(BUILD_SHARED_LIBS)
  SET(VISP_BUILD_SHARED_LIBS TRUE)  # for header vpConfig.h
ENDIF(BUILD_SHARED_LIBS)


INCLUDE(CheckLibraryExists)

#--------------------------------------------------------------------
# yarp lib
#--------------------------------------------------------------------
#--------------------------------------------------------------------
# yarp lib
#--------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_YARP_FOUND "no")  # for ViSP-third-party.txt

IF(USE_YARP)
  IF(YARP_FOUND)
    MESSAGE(STATUS "yarp found")
    SET(VISP_HAVE_YARP TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_YARP_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${YARP_INCLUDE_DIRS})
    
	# Work arround to add Yarp libraries and also third party libraries requested by Yarp 
    LIST(REVERSE YARP_LIBRARIES) # to start with YARP_init, that depends on YARP_dev, YARP_sig and YARP_OS
    FOREACH(lib ${YARP_LIBRARIES})
 	  #MESSAGE("lib: ${lib}")
      GET_TARGET_PROPERTY(CONFIGURATIONS ${lib} IMPORTED_CONFIGURATIONS)
	  FOREACH(CONFIGURATION ${CONFIGURATIONS})		
		GET_TARGET_PROPERTY(YARP_LIB ${lib} "IMPORTED_LOCATION_${CONFIGURATION}") # Get Yarp full absolute library path and name
		
	    IF(WIN32)
		  #Work around when YARP is build as shared libraries
		  STRING(REGEX REPLACE ".dll$" ".lib" YARP_LIB ${YARP_LIB})
		  
          IF(${CONFIGURATION} STREQUAL "RELEASE")
            LIST(APPEND VISP_EXTERN_LIBRARIES optimized ${YARP_LIB}) # Append full absolute library path and name
	      ELSEIF(${CONFIGURATION} STREQUAL "DEBUG")
            LIST(APPEND VISP_EXTERN_LIBRARIES debug ${YARP_LIB}) # Append full absolute library path and name
	      ENDIF()
        ELSE()
 	      LIST(APPEND VISP_EXTERN_LIBRARIES ${YARP_LIB}) # Append full absolute library path and name
	    ENDIF()
        # Get 3rd party libraries requested by Yarp 
        GET_TARGET_PROPERTY(YARP_LINK_LIBS_ ${lib} "IMPORTED_LINK_INTERFACE_LIBRARIES_${CONFIGURATION}") 
        LIST(APPEND YARP_LINK_LIBS ${YARP_LINK_LIBS_})
      ENDFOREACH()
    ENDFOREACH()
    
	# Remove Yarp libraries since they were added previously with full absolute library path and name
    FOREACH(lib ${YARP_LIBRARIES})
      LIST(REMOVE_ITEM YARP_LINK_LIBS ${lib})
    ENDFOREACH()
    # Add 3rd party libraries requested by Yarp  	
    LIST(APPEND VISP_EXTERN_LIBRARIES ${YARP_LINK_LIBS})
    
    LIST(APPEND VISP_DEFS ${YARP_DEFINES})
  ELSE()
    MESSAGE(STATUS "yarp not found")
    SET(VISP_HAVE_YARP_FOUND "no")  # for ViSP-third-party.txt
  ENDIF()
ENDIF(USE_YARP)

#---------------------------------------------------------------------
# Try to determine if Inria's NAS server hosting /udd/ is available 
#----------------------------------------------------------------------
FIND_PACKAGE(NAS)
IF(NAS_FOUND)
  SET(VISP_HAVE_ACCESS_TO_NAS TRUE) # for header vpConfig.h
ENDIF(NAS_FOUND)

#---------------------------------------------------------------------
# Platform dependent
#----------------------------------------------------------------------
IF(WIN32 AND NOT CYGWIN)
  LIST(APPEND VISP_DEFS "-DWIN32")
  LIST(APPEND VISP_DEFS "-DGX") # To support exceptions
  CHECK_LIBRARY_EXISTS("winmm.lib" getch "" HAVE_LIBWINMM) # for timeGetTime()
  IF(HAVE_LIBWINMM)
    #MESSAGE("have winmm.lib")
    LIST(APPEND VISP_EXTERN_LIBRARIES "winmm.lib")
  ENDIF(HAVE_LIBWINMM)
  if(MSVC) #check if MSVC is the compiler and set VISP_USE_MSVC accordingly
    SET(VISP_USE_MSVC TRUE)
  endif()
  # Add library ws2_32.a or ws2_32.lib for vpNetwork class
  if(MINGW)
    CHECK_LIBRARY_EXISTS("ws2_32.a" getch "" HAVE_LIBWS2_32) # for inet_ntoa() and socket functionalities
    if(HAVE_LIBWS2_32)
      #message("have ws2_32.a")
      list(APPEND VISP_EXTERN_LIBRARIES "ws2_32.a")
    endif()
  else() # pure WIN32
    CHECK_LIBRARY_EXISTS("ws2_32.lib" getch "" HAVE_LIBWS2_32) # for inet_ntoa() and socket functionalities
    if(HAVE_LIBWS2_32)
      #message("have ws2_32.lib")
      list(APPEND VISP_EXTERN_LIBRARIES "ws2_32.lib")
    endif() 
  endif()
ENDIF(WIN32 AND NOT CYGWIN)

IF(UNIX)
  LIST(APPEND VISP_DEFS "-DUNIX")
  IF(APPLE)
    LIST(APPEND VISP_DEFS "-DAPPLE")
  ENDIF(APPLE)
ENDIF(UNIX)

IF(UNIX)
  INCLUDE(TestBigEndian)
  TEST_BIG_ENDIAN(BIGENDIAN)
  #SET(BIGENDIAN FALSE)
ENDIF()

IF(BIGENDIAN)
  LIST(APPEND VISP_DEFS "-DBIGENDIAN")
ENDIF(BIGENDIAN)

#--------------------------------------------------------------------
# OpenMP
#--------------------------------------------------------------------
if(USE_OPENMP)
  set(VISP_OPENMP_FLAGS "${OpenMP_CXX_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
else()
  set(VISP_OPENMP_FLAGS "")
  if(OpenMP_CXX_FLAGS)
    string(REPLACE ${OpenMP_CXX_FLAGS} "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  endif()
endif()

#--------------------------------------------------------------------
# C++11
#--------------------------------------------------------------------
if(USE_CPP11)
  set(VISP_CPP11_FLAGS "${CPP11_CXX_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CPP11_CXX_FLAGS}")
else()
  set(VISP_CPP11_FLAGS "")
  if(CPP11_CXX_FLAGS)
    string(REPLACE ${CPP11_CXX_FLAGS} "" CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
  endif()
endif()

# Remove duplicates compilation flags
separate_arguments(CMAKE_CXX_FLAGS)
list(REMOVE_DUPLICATES CMAKE_CXX_FLAGS)
string(REPLACE ";" " " CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}" CACHE STRING "common C++ build flags" FORCE)

#--------------------------------------------------------------------
# LAPACK lib
#--------------------------------------------------------------------
SET(VISP_HAVE_LAPACK_FOUND "no")      # for ViSP-third-party.txt
IF(USE_LAPACK)
  MESSAGE(STATUS "lapack/blas found")
  SET(VISP_HAVE_LAPACK TRUE)          # for header vpConfig.h
  SET(VISP_HAVE_LAPACK_FOUND "yes")   # for ViSP-third-party.txt
  
  #MESSAGE("LAPACK_LIBRARIES = ${LAPACK_LIBRARIES}")
  LIST(APPEND VISP_EXTERN_LIBRARIES ${LAPACK_LIBRARIES})
ENDIF(USE_LAPACK)

#--------------------------------------------------------------------
# X11 lib
#--------------------------------------------------------------------
SET(VISP_HAVE_X11_FOUND "no")  # for ViSP-third-party.txt
IF(USE_X11)
  MESSAGE(STATUS "X11 found")
  SET(VISP_HAVE_X11 TRUE)  # for header vpConfig.h
  SET(VISP_HAVE_X11_FOUND "yes")  # for ViSP-third-party.txt
  LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${X11_INCLUDE_DIR})
  LIST(APPEND VISP_EXTERN_LIBRARIES ${X11_LIBRARIES} -lm)
  #MESSAGE("X11: ${X11_LIBRARIES}")
ENDIF(USE_X11)


#--------------------------------------------------------------------
# OpenGL, Qt, SoQt and Coin libs
#--------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_SIMULATOR_FOUND "no") # for ViSP-third-party.txt
SET(VISP_HAVE_COIN_FOUND "no")      # for ViSP-third-party.txt
SET(VISP_HAVE_OGRE_FOUND "no")      # for ViSP-third-party.txt
SET(VISP_HAVE_OIS_FOUND "no")       # for ViSP-third-party.txt
SET(VISP_HAVE_SOWIN_FOUND "no")     # for ViSP-third-party.txt
SET(VISP_HAVE_SOXT_FOUND "no")      # for ViSP-third-party.txt
SET(VISP_HAVE_SOQT_FOUND "no")      # for ViSP-third-party.txt
SET(VISP_HAVE_QT4_FOUND "no")       # for ViSP-third-party.txt
SET(VISP_HAVE_QT3_FOUND "no")       # for ViSP-third-party.txt
SET(VISP_HAVE_OPENGL_FOUND "no")    # for ViSP-third-party.txt

IF(USE_OGRE)
  # Find Boost
  if (NOT OGRE_BUILD_PLATFORM_IPHONE)
    if (WIN32 OR APPLE)
      set(Boost_USE_STATIC_LIBS TRUE)
    else ()
      # Statically linking boost to a dynamic Ogre build doesn't work on Linux 64bit
      set(Boost_USE_STATIC_LIBS ${OGRE_STATIC})
    endif ()
    if (MINGW)
	# this is probably a bug in CMake: the boost find module tries to look for
	# boost libraries with name libboost_*, but CMake already prefixes library
	# search names with "lib". This is the workaround.
	set(CMAKE_FIND_LIBRARY_PREFIXES ${CMAKE_FIND_LIBRARY_PREFIXES} "")
    endif ()
    set(Boost_ADDITIONAL_VERSIONS "1.53.0" "1.52.0" "1.51.0" "1.50.0" "1.49.0" "1.48.0" "1.47.0" "1.46.0" "1.45.0" "1.44.0" "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
    # Components that need linking (NB does not include header-only components like bind)
    set(OGRE_BOOST_COMPONENTS thread date_time)
    find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
    mark_as_advanced(Boost_LIB_DIAGNOSTIC_DEFINITIONS Boost_DIR)
    if (NOT Boost_FOUND)
      # Try again with the other type of libs
      set(Boost_USE_STATIC_LIBS NOT ${Boost_USE_STATIC_LIBS})
      find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
    endif()
    find_package(Boost QUIET)
 
    if (Boost_FOUND)
      # Set up referencing of Boost
      LIST(APPEND VISP_DEFS "-DBOOST_ALL_NO_LIB")  
      LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${Boost_INCLUDE_DIR})  
      LIST(APPEND VISP_EXTERN_LIBRARIES ${Boost_LIBRARIES})
    endif()
  endif()

  MARK_AS_ADVANCED(OGRE_SAMPLES_INCLUDEPATH)
  #message("OGRE_SAMPLES_INCLUDEPATH: ${OGRE_SAMPLES_INCLUDEPATH}")
  if(OGRE_SAMPLES_INCLUDEPATH)
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${OGRE_SAMPLES_INCLUDEPATH})
  endif()

  # if OGRE_MEDIA_DIR is empty, try to find the path by searching for plugins.cfg
  # Introduced since Ubuntu 12.04
  if(NOT OGRE_MEDIA_DIR)
    FIND_PATH(OGRE_MEDIA_DIR ../plugins.cfg
      $ENV{OGRE_MEDIA_DIR}
      /usr/share/OGRE-1.7.4/media	
      /usr/share/OGRE-1.8.0/media	
      /usr/share/OGRE-1.8.1/media	
    )
  endif()

  #message("OGRE_PLUGIN_DIR_REL: ${OGRE_PLUGIN_DIR_REL}")
  #message("OGRE_PLUGIN_DIR_DBG: ${OGRE_PLUGIN_DIR_DBG}")
  SET(VISP_HAVE_OGRE_FOUND "yes")   # for ViSP-third-party.txt
  SET(VISP_HAVE_OGRE TRUE)          # for header vpConfig.h

  # Try to search for an existing plugins.cfg file
  # Here we cannot use OGRE_PLUGIN_DIR_REL or OGRE_PLUGIN_DIR_DBG where
  # we may find an existing plugins.cfg file, since under Windows in these 
  # files the PluginFolder is set to a relative path. We need an absolute 
  # path to avoid recopy of the plugins in ViSP.
  # Under Linux or OSX, we may find plugins.cfg with a PluginFolder set 
  # to an absolute path in OGRE_MEDIA_DIR/..
  FIND_PATH(VISP_HAVE_OGRE_PLUGINS_PATH 
    NAMES plugins.cfg
    PATHS ${OGRE_MEDIA_DIR}/..
    NO_SYSTEM_ENVIRONMENT_PATH
  )
  
  # If no plugins.cfg file is found, we create one with absolute path
  IF(NOT VISP_HAVE_OGRE_PLUGINS_PATH)
    include(${VISP_CMAKE_MODULE_PATH}/OgreTools.cmake)
    # case 1: normal case
    #--------------
    CREATE_OGRE_PLUGIN_CONFIG_FILE()

    # case 2: install or packaging case
    #--------------
    IF(UNIX)
      set(VISP_INSTALL_DIR_OGRE_PLUGINS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/visp/data/ogre-simulator")
    ELSE()
      set(VISP_INSTALL_DIR_OGRE_PLUGINS "${CMAKE_INSTALL_PREFIX}/data/ogre-simulator")
    ENDIF()

    # install rule for plugins.cfg:
    IF(UNIX)
      if(OGRE_PLUGIN_DIR_REL)
        INSTALL(FILES
          ${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins.cfg
          DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/visp/data/ogre-simulator
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT libraries
          )
      endif()
      if(OGRE_PLUGIN_DIR_DBG)
        INSTALL(FILES
          ${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins_d.cfg
          DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/visp/data/ogre-simulator
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT libraries
          )
      endif()

    ELSE()
      if(OGRE_PLUGIN_DIR_REL)
        INSTALL(FILES
          ${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins.cfg
          DESTINATION data/ogre-simulator
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT libraries
        )
      endif()
      if(OGRE_PLUGIN_DIR_DBG)
        INSTALL(FILES
          ${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins_d.cfg
          DESTINATION data/ogre-simulator
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT libraries
        )
      endif()
    ENDIF()
  ENDIF()  

  # Try to search for an existing resources.cfg file
  FIND_PATH(VISP_HAVE_OGRE_RESOURCES_PATH 
    NAMES resources.cfg
    PATHS ${OGRE_MEDIA_DIR}/..
    NO_SYSTEM_ENVIRONMENT_PATH
  )

  IF(NOT VISP_HAVE_OGRE_RESOURCES_PATH)
    # we create a resources.cfg file for vpAROgre.cpp
    # case 1: normal case
    #--------------
    SET(VISP_HAVE_OGRE_RESOURCES_PATH ${VISP_BINARY_DIR}/data/ogre-simulator)
    CONFIGURE_FILE(
      ${VISP_CMAKE_MODULE_PATH}/resources.cfg.in
      ${VISP_HAVE_OGRE_RESOURCES_PATH}/resources.cfg
      IMMEDIATE @ONLY)

    # case 2: install or packaging case
    #--------------
    IF(UNIX)
      set(VISP_INSTALL_DIR_OGRE_RESOURCES "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/visp/data/ogre-simulator")
    ELSE()
      set(VISP_INSTALL_DIR_OGRE_RESOURCES "${CMAKE_INSTALL_PREFIX}/data/ogre-simulator")
    ENDIF()

    # install rule for resources.cfg:
    IF(UNIX)
      INSTALL(FILES
        ${VISP_HAVE_OGRE_RESOURCES_PATH}/resources.cfg
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/visp/data/ogre-simulator
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
        COMPONENT libraries
      )
    ELSE()
      INSTALL(FILES
        ${VISP_HAVE_OGRE_RESOURCES_PATH}/resources.cfg
        DESTINATION data/ogre-simulator
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
        COMPONENT libraries
      )
    ENDIF()
  ENDIF()
  
  MARK_AS_ADVANCED(VISP_HAVE_OGRE_PLUGINS_PATH)
  MARK_AS_ADVANCED(VISP_HAVE_OGRE_RESOURCES_PATH)
  #message("VISP_HAVE_OGRE_PLUGINS_PATH: ${VISP_HAVE_OGRE_PLUGINS_PATH}")
  #message("VISP_HAVE_OGRE_RESOURCES_PATH: ${VISP_HAVE_OGRE_RESOURCES_PATH}")

  LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${OGRE_INCLUDE_DIRS})  
  LIST(APPEND VISP_EXTERN_LIBRARIES ${OGRE_LIBRARIES})
ENDIF(USE_OGRE)

IF(USE_OIS AND USE_OGRE)
  SET(VISP_HAVE_OIS_FOUND "yes")   # for ViSP-third-party.txt
  SET(VISP_HAVE_OIS TRUE)           # for header vpConfig.h
  LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${OIS_INCLUDE_DIR})
  LIST(APPEND VISP_EXTERN_LIBRARIES ${OIS_LIBRARIES})   
  IF(APPLE)
    # With Ogre 1.7.4 and 1.8.1 to be able to link with libOIS.a, Cocoa framework is requested.
    # This is a work arround since it should come with FindOGRE.cmake
    LIST(APPEND VISP_EXTERN_LIBRARIES "-framework Cocoa")
  ENDIF(APPLE)

ENDIF(USE_OIS AND USE_OGRE)

IF(USE_COIN)
  IF (COIN3D_FOUND)
    MESSAGE(STATUS "Coin3D found")
    SET(VISP_HAVE_COIN_FOUND "yes")   # for ViSP-third-party.txt
    SET(VISP_HAVE_COIN TRUE)          # for header vpConfig.h
    SET(VISP_HAVE_OPENGL TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_OPENGL_FOUND "yes")  # for ViSP-third-party.txt
    IF(WIN32)
      LIST(APPEND VISP_DEFS "-DCOIN_DLL")
    ENDIF(WIN32)
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${OPENGL_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${COIN3D_INCLUDE_DIRS})  
    LIST(APPEND VISP_EXTERN_LIBRARIES ${OPENGL_LIBRARIES})

    #MESSAGE("COIN3D_INCLUDE_DIRS = ${COIN3D_INCLUDE_DIRS}")
    #MESSAGE("COIN3D_LIBRARIES = ${COIN3D_LIBRARIES}")
    LIST(APPEND VISP_EXTERN_LIBRARIES ${COIN3D_LIBRARIES})
 
    IF (USE_SOWIN AND SOWIN_FOUND)
      # We manage SoWin and Coin
      SET(VISP_HAVE_SOWIN TRUE)         # for header vpConfig.h
      SET(VISP_HAVE_SOWIN_FOUND "yes")  # for ViSP-third-party.txt
      SET(VISP_HAVE_COIN_AND_GUI TRUE)  # for header vpConfig.h    
      
      #MESSAGE("SOWIN_INCLUDE_DIR = ${SOWIN_INCLUDE_DIRS}")
      #MESSAGE("SOWIN_LIBRARIES = ${SOWIN_LIBRARIES}")
 
      LIST(APPEND VISP_DEFS "-DSOWIN_DLL")
      LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${SOWIN_INCLUDE_DIRS})
      LIST(APPEND VISP_EXTERN_LIBRARIES ${SOWIN_LIBRARIES})
    ENDIF(USE_SOWIN AND SOWIN_FOUND)

    IF(USE_SOQT AND SOQT_FOUND)
      #MESSAGE("SoQt found")

      IF (USE_QT AND QT_FOUND)
	#MESSAGE("Qt found")
	# OpenGL, QT and SOQT are found
	# We first manage OpenGL
	SET(VISP_HAVE_SOQT   TRUE) # for header vpConfig.h 
	SET(VISP_HAVE_SOQT_FOUND "yes")      # for ViSP-third-party.txt
	SET(VISP_HAVE_QT     TRUE) # for header vpConfig.h
      	SET(VISP_HAVE_COIN_AND_GUI TRUE)  # for header vpConfig.h    

        #MESSAGE(STATUS "SOQT_INCLUDE_DIRS = ${SOQT_INCLUDE_DIRS}")
	LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${SOQT_INCLUDE_DIRS})
	LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${QT_INCLUDE_DIR})
        LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${QT_INCLUDES})

	# Add SoQt libraries
	#MESSAGE("SOQT_LIBRARIES = ${SOQT_LIBRARIES}")
	LIST(APPEND VISP_EXTERN_LIBRARIES ${SOQT_LIBRARIES})
	IF(WIN32)
	  LIST(APPEND VISP_DEFS "-DSOQT_DLL")
	ENDIF(WIN32)

	# We manage QT libraries
	IF (DESIRED_QT_VERSION MATCHES 3)
	  #Add Qt3 libraries
	  SET(VISP_HAVE_QT3_FOUND "yes")       # for ViSP-third-party.txt
	  #MESSAGE("QT_QT_LIBRARY ${QT_QT_LIBRARY}")
	  LIST(APPEND VISP_EXTERN_LIBRARIES ${QT_QT_LIBRARY})
	ELSEIF(DESIRED_QT_VERSION MATCHES 4)
	  #Add Qt4 libraries
	  SET(VISP_HAVE_QT4_FOUND "yes")       # for ViSP-third-party.txt
	  #MESSAGE("QT_QTGUI_LIBRARY ${QT_QTGUI_LIBRARY}")
	  #MESSAGE("QT_QTGUI_LIBRARY_RELEASE ${QT_QTGUI_LIBRARY_RELEASE}")
	  #MESSAGE("QT_QTGUI_LIBRARY_DEBUG ${QT_QTGUI_LIBRARY_DEBUG}")
	  IF (QT_QTGUI_LIBRARY_RELEASE AND QT_QTCORE_LIBRARY_RELEASE AND QT_QTGUI_LIBRARY_DEBUG AND QT_QTCORE_LIBRARY_DEBUG)
	    LIST(APPEND VISP_EXTERN_LIBRARIES optimized ${QT_QTGUI_LIBRARY_RELEASE})
            LIST(APPEND VISP_EXTERN_LIBRARIES optimized ${QT_QTCORE_LIBRARY_RELEASE})
	    LIST(APPEND VISP_EXTERN_LIBRARIES debug ${QT_QTGUI_LIBRARY_DEBUG})
            LIST(APPEND VISP_EXTERN_LIBRARIES debug ${QT_QTCORE_LIBRARY_DEBUG})
	  ELSEIF (QT_QTGUI_LIBRARY_RELEASE AND QT_QTCORE_LIBRARY_RELEASE)
	    LIST(APPEND VISP_EXTERN_LIBRARIES ${QT_QTGUI_LIBRARY_RELEASE})
            LIST(APPEND VISP_EXTERN_LIBRARIES ${QT_QTCORE_LIBRARY_RELEASE})
	  ELSEIF (QT_QTGUI_LIBRARY_DEBUG AND QT_QTCORE_LIBRARY_DEBUG)
	    LIST(APPEND VISP_EXTERN_LIBRARIES ${QT_QTGUI_LIBRARY_DEBUG})
            LIST(APPEND VISP_EXTERN_LIBRARIES ${QT_QTCORE_LIBRARY_DEBUG})
	  ENDIF()
	ENDIF (DESIRED_QT_VERSION MATCHES 3)

	# Because in QT_DEFINITIONS defs are separated by ";", parse the
	# QT_DEFINITIONS in order to build a space separated string
	#SET(VISP_DEFS "${VISP_DEFS} ${QT_DEFINITIONS}") did not work
	# in third party projects which use visp-config shell script 
	# (due to ";" # separate char)
	FOREACH(DEF ${QT_DEFINITIONS})
	  #MESSAGE("DEF: ${DEF}")
	  LIST(APPEND VISP_DEFS ${DEF})
	ENDFOREACH(DEF)
					  
	LIST(APPEND VISP_DEFS "-DQT_DLL")
      ENDIF (USE_QT AND QT_FOUND)
    ENDIF(USE_SOQT AND SOQT_FOUND)
        
    IF (USE_SOXT AND SOXT_FOUND)
      # OPenGL and SoXt are found
      # We first manage OpenGL
      SET(VISP_HAVE_SOXT TRUE)  # for header vpConfig.h    
      SET(VISP_HAVE_COIN_AND_GUI TRUE)  # for header vpConfig.h    
      SET(VISP_HAVE_SOXT_FOUND "yes")       # for ViSP-third-party.txt
      LIST(APPEND VISP_EXTERN_LIBRARIES ${SOXT_LIBRARIES})
    ENDIF(USE_SOXT AND SOXT_FOUND)

    IF(VISP_HAVE_COIN_AND_GUI)
      SET(VISP_HAVE_COIN_AND_GUI_FOUND "yes")  # for ViSP-third-party.txt
      MESSAGE(STATUS "Coin3D + GUI found")
    ELSE(VISP_HAVE_COIN_AND_GUI)
      MESSAGE(STATUS "Coin3D + GUI not found")
      SET(VISP_HAVE_COIN_AND_GUI_FOUND "no")  # for ViSP-third-party.txt
    ENDIF(VISP_HAVE_COIN_AND_GUI)
  ENDIF (COIN3D_FOUND)
ENDIF(USE_COIN)

#---------------------------------------------------------------------
# XML2
#---------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_XML2_FOUND "no")  # for ViSP-third-party.txt

IF(USE_XML2)
  #FIND_PACKAGE(XML2)
  IF(XML2_FOUND)
    SET(VISP_HAVE_XML2 TRUE)
    SET(VISP_HAVE_XML2_FOUND "yes")  # for ViSP-third-party.txt
    MESSAGE(STATUS "XML2 found")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${XML2_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${XML2_LIBRARIES})
  ELSE(XML2_FOUND)
    MESSAGE(STATUS "XML2 not found")
    SET(VISP_HAVE_XML2_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(XML2_FOUND)
ENDIF(USE_XML2)


#---------------------------------------------------------------------
# libfreenect
# Note that this library needs libusb-1.0 and libpthread
#---------------------------------------------------------------------
# default initialisation for ViSP-third-party.txt
SET(VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES_FOUND "no")
SET(VISP_HAVE_LIBFREENECT_FOUND "no") 
SET(VISP_HAVE_LIBUSB_1_FOUND "no") 

IF(USE_LIBFREENECT)
  #FIND_PACKAGE(LIBFREENECT)
  IF(LIBFREENECT_FOUND)
    SET(VISP_HAVE_LIBFREENECT TRUE)
    SET(VISP_HAVE_LIBFREENECT_FOUND "yes")  # for ViSP-third-party.txt
 
    MESSAGE(STATUS "libfreenect found")

    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${LIBFREENECT_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${LIBFREENECT_LIBRARIES})
  ELSE()
    MESSAGE(STATUS "libfreenect not found")
    SET(VISP_HAVE_LIBFREENECT_FOUND "no")  # for ViSP-third-party.txt
  ENDIF()
ENDIF()

IF(USE_LIBUSB_1)
  #FIND_PACKAGE(LIBUSB-1)
  IF(LIBUSB_1_FOUND)
    SET(VISP_HAVE_LIBUSB_1 TRUE)
    SET(VISP_HAVE_LIBUSB_1_FOUND "yes")  # for ViSP-third-party.txt
 
    MESSAGE(STATUS "libusb-1.0 found")
 
    LIST(APPEND VISP_EXTERN_LIBRARIES ${LIBUSB_1_LIBRARIES})
  ELSE()
    MESSAGE(STATUS "libusb-1.0 not found")
    SET(VISP_HAVE_LIBUSB_1_FOUND "no")  # for ViSP-third-party.txt
  ENDIF()
ENDIF()

# Check if libfreenect dependencies (ie libusb-1.0 and libpthread) are available
IF(USE_LIBFREENECT AND USE_LIBUSB_1 AND USE_PTHREAD)
  IF(LIBFREENECT_FOUND AND LIBUSB_1_FOUND AND PTHREAD_FOUND)
    SET(VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES TRUE)
    SET(VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES_FOUND "yes")  # for ViSP-third-party.txt

    # The material is found. Check if libfreenect is an old version
    include(CheckCXXSourceCompiles)
    SET(CMAKE_REQUIRED_LIBRARIES ${LIBFREENECT_LIBRARIES} ${PTHREAD_LIBRARIES} ${LIBUSB_1_LIBRARIES})
    SET(CMAKE_REQUIRED_INCLUDES ${LIBFREENECT_INCLUDE_DIRS} ${PTHREAD_INCLUDE_DIRS})
    CHECK_CXX_SOURCE_COMPILES("
      #include <libfreenect.hpp>

      class vpMyKinect : public Freenect::FreenectDevice
      {
      };

      int main()
      {
        Freenect::Freenect<vpMyKinect> freenect;
      }
      " LIBFREENECT_IS_OLD_VERSION)
    #MESSAGE("LIBFREENECT_IS_OLD_VERSION: ${LIBFREENECT_IS_OLD_VERSION}")
    IF(LIBFREENECT_IS_OLD_VERSION)
      SET(VISP_HAVE_LIBFREENECT_OLD TRUE)
    ELSE()
      SET(VISP_HAVE_LIBFREENECT_OLD  FALSE)
    ENDIF()

  ENDIF()
ENDIF()


#--------------------------------------------------------------------
# pthread lib
#--------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_PTHREAD_FOUND "no")  # for ViSP-third-party.txt

IF(USE_PTHREAD)
  #FIND_PACKAGE(PTHREAD)
  IF(PTHREAD_FOUND)
    MESSAGE(STATUS "pthread found")
    SET(VISP_HAVE_PTHREAD TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_PTHREAD_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${PTHREAD_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${PTHREAD_LIBRARIES})
    #MESSAGE("PTHREAD_LIBRARIES = ${PTHREAD_LIBRARIES}")
  ELSE(PTHREAD_FOUND)
    MESSAGE(STATUS "pthread not found")
    SET(VISP_HAVE_PTHREAD_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(PTHREAD_FOUND)
ENDIF(USE_PTHREAD)

#--------------------------------------------------------------------
# parallel port usage
#--------------------------------------------------------------------
FIND_PACKAGE(PARPORT)
IF(PARPORT_FOUND) 
  SET(VISP_HAVE_PARPORT TRUE)  # for header vpConfig.h
ENDIF(PARPORT_FOUND)  

#----------------------------------------------------------------------
# Third party software :
#----------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_OPENCV_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_GSL_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_GTK_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_D3D9_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_GDI_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_LIBJPEG_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_LIBPNG_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_ZLIB_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_FFMPEG_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_CMU1394_FOUND "no")  # for ViSP-third-party.txt

# open CV
IF(USE_OPENCV)
  #SET(OpenCV_FIND_QUIETLY TRUE)
  #FIND_PACKAGE(OpenCV)
  IF(OpenCV_FOUND)
    MESSAGE(STATUS "OpenCV found")
    IF(VISP_BUILD_SHARED_LIBS AND VISP_USE_MSVC)
      ADD_DEFINITIONS("-DCVAPI_EXPORTS")
    ENDIF(VISP_BUILD_SHARED_LIBS AND VISP_USE_MSVC)
    SET(VISP_HAVE_OPENCV  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_OPENCV_FOUND "yes")  # for ViSP-third-party.txt

    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LINK_DIR ${OpenCV_LIB_DIR})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${OpenCV_LIBS})
    #MESSAGE("OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
    #MESSAGE("OpenCV_LIBS = ${OpenCV_LIBS}")
    #MESSAGE("OpenCV_LIB_DIR = ${OpenCV_LIB_DIR}")
    SET(VISP_HAVE_OPENCV_VERSION "(${OpenCV_VERSION_MAJOR}*65536 + ${OpenCV_VERSION_MINOR}*256 + ${OpenCV_VERSION_PATCH})") # for ViSP-third-party.txt
    #MESSAGE("VISP_HAVE_OPENCV_VERSION = ${VISP_HAVE_OPENCV_VERSION}")

    # additional check to see if opencv_nonfree module introduced in OpenCV 2.4 is available
    #message("OpenCV_NONFREE_FOUND: ${OPENCV_NONFREE_FOUND}")
    if(${OpenCV_VERSION} VERSION_LESS "2.4.2")
        set(VISP_HAVE_OPENCV_NONFREE TRUE)  # for header vpConfig.h
    else() # Version is greater or equal to 2.4.0
      if(OPENCV_NONFREE_FOUND)
        message(STATUS "OpenCV nonfree found")
        set(VISP_HAVE_OPENCV_NONFREE TRUE)  # for header vpConfig.h
      else()
        message(STATUS "OpenCV nonfree not found")
      endif()
    endif()
  ELSE(OpenCV_FOUND)
    MESSAGE(STATUS "OpenCV not found")
    SET(VISP_HAVE_OPENCV_FOUND "no")    # for ViSP-third-party.txt
    SET(VISP_HAVE_OPENCV_VERSION "(0)") # for ViSP-third-party.txt
  ENDIF(OpenCV_FOUND)
ELSE(USE_OPENCV)
  SET(OpenCV_VERSION_MAJOR 0)
  SET(OpenCV_VERSION_MINOR 0)
  SET(OpenCV_VERSION_PATCH 0)
  SET(VISP_HAVE_OPENCV_VERSION "(0)") # for ViSP-third-party.txt
ENDIF(USE_OPENCV)


# gnu scientific library
IF(USE_GSL)
  #FIND_PACKAGE(GSL)
  IF(GSL_FOUND)
    MESSAGE(STATUS "GSL found")
    SET(VISP_HAVE_GSL  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_GSL_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${GSL_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LINK_DIR ${GSL_EXE_LINKER_FLAGS})
    LIST(APPEND VISP_EXTERN_LINK_DIR ${GSL_LINK_DIRECTORIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${GSL_LIBRARIES})
    #LINK_DIRECTORIES(${GSL_LINK_DIRECTORIES})
    #MESSAGE("GSL_INCLUDE_DIRS = ${GSL_INCLUDE_DIRS}")
    #MESSAGE("GSL_LINK_DIRECTORIES = ${GSL_LINK_DIRECTORIES}")
    #MESSAGE("GSL_LIBRARIES = ${GSL_LIBRARIES}")
  ELSE(GSL_FOUND)
    MESSAGE(STATUS "GSL not found")
    SET(VISP_HAVE_GSL_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(GSL_FOUND)
ENDIF(USE_GSL)

# strategy for gtk: search first gtk2. If not found search gtk1
# gtk-2 library:
IF(USE_GTK2)
    #FIND_PACKAGE(GTK2)
    IF(GTK2_FOUND)
      MESSAGE(STATUS "GTK2 found")
      SET(VISP_HAVE_GTK TRUE)  # for header vpConfig.h
      SET(VISP_HAVE_GTK_FOUND "yes")  # for ViSP-third-party.txt
      LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${GTK2_INCLUDE_DIRS})
      
      IF(UNIX)
	## ignore gtk pedantic warnings by gcc specific -isystem :
	SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -isystem ${GTK2_gtk_INCLUDE_PATH}")
      ENDIF(UNIX)

      LIST(APPEND VISP_EXTERN_LIBRARIES ${GTK2_LIBRARIES})
      # MESSAGE(STATUS "DBG CMAKE_CXX_FLAGS=${CMAKE_CXX_FLAGS}")
      #MESSAGE("GTK2 includes: ${GTK2_INCLUDE_DIRS}")
      #MESSAGE("GTK2 Libs: ${GTK2_LIBRARIES}")
      SET(USE_GTK1 OFF) # no need to found gtk-1
    ELSE(GTK2_FOUND)
      MESSAGE(STATUS "GTK2 not found")
      SET(VISP_HAVE_GTK_FOUND "no")  # for ViSP-third-party.txt
      #MESSAGE("GTK2 requested but not found.")
    ENDIF(GTK2_FOUND)
ENDIF(USE_GTK2)

# Under Windows, usage of Direct3D9
IF(USE_DIRECT3D)
  #FIND_PACKAGE(DIRECT3D)
  IF(DIRECT3D_FOUND)
    MESSAGE(STATUS "Direct3D found")
    SET(VISP_HAVE_D3D9 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_D3D9_FOUND "yes")  # for ViSP-third-party.txt
    #MESSAGE("DBG VISP_HAVE_D3D9= ${VISP_HAVE_D3D9}")
    #MESSAGE("DBG DIRECT3D_INCLUDE_DIRS=${DIRECT3D_INCLUDE_DIRS}")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${DIRECT3D_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${DIRECT3D_LIBRARIES})
    #MESSAGE("DIRECT3D_LIBRARIES = ${DIRECT3D_LIBRARIES}")
  ELSE(DIRECT3D_FOUND)
    MESSAGE(STATUS "Direct3D not found")
    SET(VISP_HAVE_D3D9_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(DIRECT3D_FOUND)
ENDIF(USE_DIRECT3D)

# Under Windows, usage of GDI (Graphics Device Interface)
IF(USE_GDI)
  #FIND_PACKAGE(GDI)
  IF(GDI_FOUND)
    MESSAGE(STATUS "GDI found")
    SET(VISP_HAVE_GDI TRUE) # for header vpConfig.h
    SET(VISP_HAVE_GDI_FOUND "yes")  # for ViSP-third-party.txt
    #MESSAGE("DBG VISP_HAVE_GDI= ${VISP_HAVE_GDI}")
    LIST(APPEND VISP_EXTERN_LIBRARIES ${GDI_LIBRARIES})
    #MESSAGE("GDI_LIBRARIES = ${GDI_LIBRARIES}")
  ELSE(GDI_FOUND)
    MESSAGE(STATUS "GDI not found")
    SET(VISP_HAVE_GDI_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(GDI_FOUND)
ENDIF(USE_GDI)

# zlib
IF(USE_ZLIB)
  #FIND_PACKAGE(ZLIB)
  IF(ZLIB_FOUND)
    MESSAGE(STATUS "zlib found")
    SET(VISP_HAVE_ZLIB  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_ZLIB_FOUND "yes")  # for ViSP-third-party.txt

    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${ZLIB_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${ZLIB_LIBRARIES})
    #MESSAGE(ZLIB_LIBRARIES: ${ZLIB_LIBRARIES})
  ELSE()
    MESSAGE(STATUS "zlib not found")
    SET(VISP_HAVE_FOUND "no")  # for ViSP-third-party.txt
  ENDIF()
ENDIF(USE_ZLIB)

# Libjpeg
IF(USE_LIBJPEG)
  #FIND_PACKAGE(JPEG)
  #MESSAGE("test ${JPEG_FOUND}")
  IF(JPEG_FOUND)
    MESSAGE(STATUS "libjpeg found")
    SET(VISP_HAVE_LIBJPEG  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_LIBJPEG_FOUND "yes")  # for ViSP-third-party.txt

	#message("JPEG_INCLUDE_DIR: ${JPEG_INCLUDE_DIR}")
 	# Here we use JPEG_INCLUDE_DIR that is the var name defined in CMake/Modules/FindJPEG.cmake
	# In FindMyJPEG.cmake, we define both JPEG_INCLUDE_DIR and JPEG_INCLUDE_DIRS
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${JPEG_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${JPEG_LIBRARIES})

    # If Qt3 and libjpeg are used, we add here QT_CLEAN_NAMESPACE define to
    # handle a compilation error "conflicting declaration 'typedef long int 
    # INT32'"
    IF(VISP_HAVE_QT3_FOUND)
      LIST(APPEND VISP_DEFS "-DQT_CLEAN_NAMESPACE")
      # MESSAGE("add -DQT_CLEAN_NAMESPACE")
    ENDIF(VISP_HAVE_QT3_FOUND)

  ELSE(JPEG_FOUND)
    MESSAGE(STATUS "libjpeg not found")
    SET(VISP_HAVE_LIBJPEG_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(JPEG_FOUND)
ENDIF(USE_LIBJPEG)

# Libpng
IF(USE_LIBPNG)
  #FIND_PACKAGE(PNG)
  IF(PNG_FOUND)
    MESSAGE(STATUS "libpng found")
    SET(VISP_HAVE_LIBPNG  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_LIBPNG_FOUND "yes")  # for ViSP-third-party.txt

    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${PNG_INCLUDE_DIR})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${PNG_LIBRARIES})
    LIST(APPEND VISP_DEFS ${PNG_DEFINITIONS})
  ELSE()
    MESSAGE(STATUS "libpng not found")
    SET(VISP_HAVE_LIBPNG_FOUND "no")  # for ViSP-third-party.txt
  ENDIF()
ENDIF(USE_LIBPNG)

# FFMPEG
IF(USE_FFMPEG)
  #FIND_PACKAGE(FFMPEG)
  IF(FFMPEG_FOUND)
    MESSAGE(STATUS "FFMPEG found")
    SET(VISP_HAVE_FFMPEG  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_FFMPEG_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${FFMPEG_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${FFMPEG_LIBRARIES})
  ELSE(LIBFFMPEG_FOUND)
    MESSAGE(STATUS "FFMPEG not found")
    SET(VISP_HAVE_FFMPEG_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(FFMPEG_FOUND)
ENDIF(USE_FFMPEG)

#----------------------------------------------------------------------
# Specific hardware : framegrabbers and cameras
#----------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_DC1394_1_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_DC1394_2_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_V4L2_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_DIRECTSHOW_FOUND "no")  # for ViSP-third-party.txt

# library for firewire cameras
IF(USE_DC1394_1 OR USE_DC1394_2)
  #FIND_PACKAGE(DC1394)
  IF(DC1394_FOUND)
    SET(VISP_HAVE_DC1394 TRUE) # for header vpConfig.h
    IF(DC1394_2_FOUND AND USE_DC1394_2)
      MESSAGE(STATUS "libdc1394-2.x found")
      SET(VISP_HAVE_DC1394_2 TRUE) # for header vpConfig.h
      SET(VISP_HAVE_DC1394_2_FOUND "yes")  # for ViSP-third-party.txt
      IF(DC1394_CAMERA_ENUMERATE_FOUND)
        SET(VISP_HAVE_DC1394_2_CAMERA_ENUMERATE TRUE) # for header vpConfig.h
      ENDIF(DC1394_CAMERA_ENUMERATE_FOUND)
      IF(DC1394_FIND_CAMERAS_FOUND)
        SET(VISP_HAVE_DC1394_2_FIND_CAMERAS TRUE) # for header vpConfig.h
      ENDIF(DC1394_FIND_CAMERAS_FOUND)
      LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${DC1394_2_INCLUDE_DIRS})
      LIST(APPEND VISP_EXTERN_LIBRARIES ${DC1394_2_LIBRARY})

      #MESSAGE("DBG VISP_HAVE_DC1394_2= ${VISP_HAVE_DC1394_2}")
    ELSE(DC1394_2_FOUND AND USE_DC1394_2)
      MESSAGE(STATUS "libdc1394-2.x not found")
      SET(VISP_HAVE_DC1394_2_FOUND "no")  # for ViSP-third-party.txt
    ENDIF(DC1394_2_FOUND AND USE_DC1394_2)
    
    IF(DC1394_1_FOUND AND USE_DC1394_1)
      MESSAGE(STATUS "libdc1394-1.x found")
      SET(VISP_HAVE_DC1394_1 TRUE) # for header vpConfig.h
      SET(VISP_HAVE_DC1394_1_FOUND "yes")  # for ViSP-third-party.txt
      #MESSAGE("DBG VISP_HAVE_DC1394_1= ${VISP_HAVE_DC1394_1}")
      LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${DC1394_1_INCLUDE_DIRS})
      LIST(APPEND VISP_EXTERN_LIBRARIES ${DC1394_1_LIBRARY})
    ELSE(DC1394_1_FOUND AND USE_DC1394_1)
      MESSAGE(STATUS "libdc1394-1.x not found")
      SET(VISP_HAVE_DC1394_1_FOUND "no")  # for ViSP-third-party.txt
    ENDIF(DC1394_1_FOUND AND USE_DC1394_1)
    #MESSAGE("DBG DC1394_INCLUDE_DIRS=${DC1394_INCLUDE_DIRS}")
    

    IF(APPLE)
      #SET(VISP_EXTERN_LIBRARIES ${VISP_EXTERN_LIBRARIES} IOkit "-framework Carbon")
      LIST(APPEND VISP_EXTERN_LIBRARIES "-framework IOkit -framework Carbon")
    ENDIF(APPLE)
     #MESSAGE("DC1394_LIBRARIES = ${DC1394_LIBRARIES}")
  ELSE(DC1394_FOUND)
    MESSAGE(STATUS "libdc1394-1.x not found")
    MESSAGE(STATUS "libdc1394-2.x not found")
    SET(VISP_HAVE_DC1394_1_FOUND "no")  # for ViSP-third-party.txt
    SET(VISP_HAVE_DC1394_2_FOUND "no")  # for ViSP-third-party.txt
    #MESSAGE("Firewire dc1394 requested but not found. Turn off USE_DC1394!")
    # SET(USE_DC1394 OFF)
  ENDIF(DC1394_FOUND)
ENDIF(USE_DC1394_1 OR USE_DC1394_2)

# library for CMU1394 pan-tilt head
IF(USE_CMU1394)
#   FIND_PACKAGE(CMU1394)
  IF(CMU1394_FOUND)
    MESSAGE(STATUS "CMU1394 found")
    SET(VISP_HAVE_CMU1394 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_CMU1394_FOUND "yes")  # for ViSP-third-party.txt
     #MESSAGE("DBG VISP_HAVE_CMU1394= ${VISP_HAVE_CMU1394}")
     #MESSAGE("DBG CMU1394_INCLUDE_DIRS=${CMU1394_INCLUDE_DIRS}")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${CMU1394_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${CMU1394_LIBRARIES})
     #MESSAGE("CMU1394_LIBRARIES = ${CMU1394_LIBRARIES}")
  ELSE(CMU1394_FOUND)
    MESSAGE(STATUS "CMU1394 not found")
    SET(VISP_HAVE_CMU1394_FOUND "no")  # for ViSP-third-party.txt
    # SET(CMU1394 OFF)
  ENDIF(CMU1394_FOUND)
ENDIF(USE_CMU1394)

# library for Video For Linux Two framegrabber devices
IF(USE_V4L2)
  #FIND_PACKAGE(V4L2)
  IF(V4L2_FOUND)
    MESSAGE(STATUS "V4l2 found")
    SET(VISP_HAVE_V4L2 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_V4L2_FOUND "yes")  # for ViSP-third-party.txt
     #MESSAGE("DBG VISP_HAVE_V4L2= ${VISP_HAVE_V4L2}")
     #MESSAGE("DBG V4L2_INCLUDE_DIRS=${V4L2_INCLUDE_DIRS}")
     #MESSAGE("DBG V4L2_LIBRARIES=${V4L2_LIBRARIES}")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${V4L2_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${V4L2_LIBRARIES})
  ELSE(V4L2_FOUND)
    MESSAGE(STATUS "V4l2 not found")
    SET(VISP_HAVE_V4L2_FOUND "no")  # for ViSP-third-party.txt
    #MESSAGE("Video For Linux Two requested but not found.")
    # SET(USE_V4L2 OFF)
  ENDIF(V4L2_FOUND)
ENDIF(USE_V4L2)

# Under Windows, usage of Direct Show
IF(USE_DIRECTSHOW)
  #FIND_PACKAGE(DIRECTSHOW)
  IF(DIRECTSHOW_FOUND)
    MESSAGE(STATUS "DirectShow found")
    SET(VISP_HAVE_DIRECTSHOW TRUE) # for header vpConfig.h
    SET(VISP_HAVE_DIRECTSHOW_FOUND "yes")  # for ViSP-third-party.txt
    #MESSAGE("DBG VISP_HAVE_DIRECTSHOW= ${VISP_HAVE_PTU46}")
    #MESSAGE("DBG DIRECTSHOW_INCLUDE_DIRS=${DIRECTSHOW_INCLUDE_DIRS}")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${DIRECTSHOW_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${DIRECTSHOW_LIBRARIES})
    #MESSAGE("DIRECTSHOW_LIBRARIES = ${DIRECTSHOW_LIBRARIES}")
  ELSE(DIRECTSHOW_FOUND)
    MESSAGE(STATUS "DirectShow not found")
    SET(VISP_HAVE_DIRECTSHOW_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(DIRECTSHOW_FOUND)
ENDIF(USE_DIRECTSHOW)

#----------------------------------------------------------------------
# Specific compiler options
#----------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_OPENMP_FOUND "no")
SET(VISP_HAVE_CPP11_COMPATIBILITY_FOUND "no")

IF(USE_OPENMP)
  IF(OPENMP_FOUND)
    MESSAGE(STATUS "OpenMP found")
    SET(VISP_HAVE_OPENMP TRUE) # for header vpConfig.h
    SET(VISP_HAVE_OPENMP_FOUND "yes")  # for ViSP-third-party.txt
  ELSE(OPENMP_FOUND)
    MESSAGE(STATUS "OpenMP not found")
    SET(VISP_HAVE_OPENMP_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(OPENMP_FOUND)
ENDIF(USE_OPENMP)

IF(USE_CPP11)
  IF(CPP11_FOUND)
    MESSAGE(STATUS "C++11 found")
    SET(VISP_HAVE_CPP11_COMPATIBILITY TRUE) # for header vpConfig.h
    SET(VISP_HAVE_CPP11_COMPATIBILITY_FOUND "yes")  # for ViSP-third-party.txt
  ELSE(CPP11_FOUND)
    MESSAGE(STATUS "C++11 not found")
    SET(VISP_HAVE_CPP11_COMPATIBILITY_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(CPP11_FOUND)
ENDIF()


#----------------------------------------------------------------------
# The wireframe simulator
#----------------------------------------------------------------------
# include all the wireframe scene files
INCLUDE(${VISP_SOURCE_DIR}/src/simulator/wireframe-simulator/scene/CMakeWireframeScenesList.cmake)

SET(WIREFRAME_SCENES_LIST "")
foreach(SCENE ${WIREFRAME_SCENES})
  set(WIREFRAME_SCENES_SRC "${VISP_SOURCE_DIR}/src/simulator/wireframe-simulator/scene/${SCENE}")
  set(WIREFRAME_SCENES_DST "${VISP_BINARY_DIR}/data/wireframe-simulator/${SCENE}")
  IF(CMAKE_MINOR_VERSION LESS 8 
    AND CMAKE_MAJOR_VERSION GREATER 1)
  ADD_CUSTOM_COMMAND(
    OUTPUT ${WIREFRAME_SCENES_DST}
    COMMAND ${CMAKE_COMMAND} -E copy ${WIREFRAME_SCENES_SRC} ${WIREFRAME_SCENES_DST}
    DEPENDS ${WIREFRAME_SCENES_SRC}
    )
  SET(WIREFRAME_SCENES_LIST ${WIREFRAME_SCENES_LIST} ${WIREFRAME_SCENES_DST})
  ELSE(CMAKE_MINOR_VERSION LESS 8 
    AND CMAKE_MAJOR_VERSION GREATER 1)
    file(COPY ${WIREFRAME_SCENES_SRC}
      DESTINATION data/wireframe-simulator
      FILE_PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
      OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
      OWNER_WRITE
      )
  ENDIF(CMAKE_MINOR_VERSION LESS 8 
    AND CMAKE_MAJOR_VERSION GREATER 1)
endforeach(SCENE)

IF(CMAKE_MINOR_VERSION LESS 8 
  AND CMAKE_MAJOR_VERSION GREATER 1)
  ADD_CUSTOM_TARGET(SCENE ALL
    DEPENDS ${WIREFRAME_SCENES_LIST}
    )
ENDIF(CMAKE_MINOR_VERSION LESS 8 
  AND CMAKE_MAJOR_VERSION GREATER 1)


#----------------------------------------------------------------------
# Robot simulators
#----------------------------------------------------------------------
INCLUDE(${VISP_SOURCE_DIR}/src/robot/simulator-robot/arms/CMakeRobotArmsList.cmake)

SET(ROBOT_ARMS_LIST "")
foreach(SCENE_ROBOT ${ROBOT_ARMS_SCENES})
  set(ROBOT_ARMS_SRC "${VISP_SOURCE_DIR}/src/robot/simulator-robot/arms/${SCENE_ROBOT}")
  set(ROBOT_ARMS_DST "${VISP_BINARY_DIR}/data/robot-simulator/${SCENE_ROBOT}")
  IF(CMAKE_MINOR_VERSION LESS 8 
    AND CMAKE_MAJOR_VERSION GREATER 1)
  ADD_CUSTOM_COMMAND(
    OUTPUT ${ROBOT_ARMS_DST}
    COMMAND ${CMAKE_COMMAND} -E copy ${ROBOT_ARMS_SRC} ${ROBOT_ARMS_DST}
    DEPENDS ${ROBOT_ARMS_SRC}
    )
  SET(ROBOT_ARMS_LIST ${ROBOT_ARMS_LIST} ${ROBOT_ARMS_DST})
  ELSE(CMAKE_MINOR_VERSION LESS 8 
    AND CMAKE_MAJOR_VERSION GREATER 1)
    file(COPY ${ROBOT_ARMS_SRC}
      DESTINATION data/robot-simulator
      FILE_PERMISSIONS OWNER_READ GROUP_READ WORLD_READ
      OWNER_EXECUTE GROUP_EXECUTE WORLD_EXECUTE
      OWNER_WRITE
      )
  ENDIF(CMAKE_MINOR_VERSION LESS 8 
    AND CMAKE_MAJOR_VERSION GREATER 1)
endforeach(SCENE_ROBOT)

IF(CMAKE_MINOR_VERSION LESS 8 
  AND CMAKE_MAJOR_VERSION GREATER 1)
  ADD_CUSTOM_TARGET(SCENE_ROBOT ALL
    DEPENDS ${ROBOT_ARMS_LIST}
    )
ENDIF(CMAKE_MINOR_VERSION LESS 8 
  AND CMAKE_MAJOR_VERSION GREATER 1)

#----------------------------------------------------------------------
# Specific hardware : robots
#----------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_AFMA4_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_AFMA6_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_BICLOPS_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_PTU46_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_PIONEER_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_VIPER650_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_VIPER850_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_CYCAB_FOUND "no")  # for ViSP-third-party.txt

# library for Irisa's Afma4 cylindrical robot
IF(USE_AFMA4)
  # Low level API to control the robot: 
  # libcalinux - librt - libraw1394 - libirisa
#   IF(NOT RAW1394_FOUND)
#     FIND_PACKAGE(RAW1394)
#   ENDIF(NOT RAW1394_FOUND)
#   FIND_PACKAGE(RT)
#   FIND_PACKAGE(CALINUX)
#   FIND_PACKAGE(IRISA)
  IF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Afma4 found")
    SET(VISP_HAVE_AFMA4 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_AFMA4_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${RAW1394_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${CALINUX_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${IRISA_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${IRISA_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${CALINUX_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${RAW1394_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${RT_LIBRARIES})
  ELSE(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Afma4 not found")
    MESSAGE("Afma4 robot requested but not found. Turn off USE_AFMA4 option!")
    SET(VISP_HAVE_AFMA4_FOUND "no")  # for ViSP-third-party.txt
    # SET(USE_AFMA4 OFF)
  ENDIF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
ENDIF(USE_AFMA4)

# library for Irisa's Afma6 cartesian robot
IF(USE_AFMA6)
  # Low level API to control the robot: 
  # libcalinux - librt - libraw1394 - libirisa
#   IF(NOT RAW1394_FOUND)
#     FIND_PACKAGE(RAW1394)
#   ENDIF(NOT RAW1394_FOUND)
#   FIND_PACKAGE(RT)
#   FIND_PACKAGE(CALINUX)
#   FIND_PACKAGE(IRISA)
  IF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Afma6 found")
    SET(VISP_HAVE_AFMA6 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_AFMA6_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${RAW1394_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${CALINUX_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${IRISA_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${IRISA_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${CALINUX_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${RAW1394_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${RT_LIBRARIES})
  ELSE(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Afma6 not found")
    MESSAGE("Afma6 robot requested but not found. Turn off USE_AFMA6 option!")
    SET(VISP_HAVE_AFMA6_FOUND "no")  # for ViSP-third-party.txt
    # SET(USE_AFMA6 OFF)
  ENDIF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
ENDIF(USE_AFMA6)

# library for Irisa's Viper850 arm robot
IF(USE_VIPER850)
  # Low level API to control the robot: 
  # libcalinux - librt - libraw1394 - libirisa
#   IF(NOT RAW1394_FOUND)
#     FIND_PACKAGE(RAW1394)
#   ENDIF(NOT RAW1394_FOUND)
#   FIND_PACKAGE(RT)
#   FIND_PACKAGE(CALINUX)
#   FIND_PACKAGE(IRISA)
  IF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Viper850 found")
    SET(VISP_HAVE_VIPER850 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_VIPER850_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${RAW1394_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${CALINUX_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${IRISA_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${IRISA_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${CALINUX_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${RAW1394_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${RT_LIBRARIES})
  ELSE(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Viper S850 not found")
    MESSAGE("Viper S850 robot requested but not found. Turn off USE_VIPER850 option!")
    SET(VISP_HAVE_VIPER850_FOUND "no")  # for ViSP-third-party.txt
    # SET(USE_VIPER850 OFF)
  ENDIF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
ENDIF(USE_VIPER850)

# library for Irisa's Viper650 arm robot
IF(USE_VIPER650)
  # Low level API to control the robot:
  # libcalinux - librt - libraw1394 - libirisa
#   IF(NOT RAW1394_FOUND)
#     FIND_PACKAGE(RAW1394)
#   ENDIF(NOT RAW1394_FOUND)
#   FIND_PACKAGE(RT)
#   FIND_PACKAGE(CALINUX)
#   FIND_PACKAGE(IRISA)
  IF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Viper650 found")
    SET(VISP_HAVE_VIPER650 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_VIPER650_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${RAW1394_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${CALINUX_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${IRISA_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${IRISA_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${CALINUX_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${RAW1394_LIBRARIES})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${RT_LIBRARIES})
  ELSE(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
    MESSAGE(STATUS "Viper S650 not found")
    MESSAGE("Viper S650 robot requested but not found. Turn off USE_VIPER650 option!")
    SET(VISP_HAVE_VIPER650_FOUND "no")  # for ViSP-third-party.txt
    # SET(USE_VIPER650 OFF)
  ENDIF(RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND)
ENDIF(USE_VIPER650)

# library for Biclops pan-tilt head
IF(USE_BICLOPS)
#   FIND_PACKAGE(BICLOPS)
  IF(BICLOPS_FOUND)
    MESSAGE(STATUS "Biclops found")
    SET(VISP_HAVE_BICLOPS TRUE) # for header vpConfig.h
    #message("BICLOPS_HAVE_GET_HOMED_STATE_FUNCTION: ${BICLOPS_HAVE_GET_HOMED_STATE_FUNCTION}")
    IF(BICLOPS_HAVE_GET_HOMED_STATE_FUNCTION)
      SET(VISP_HAVE_BICLOPS_AND_GET_HOMED_STATE_FUNCTION TRUE) # for header vpConfig.h
    ENDIF()
    SET(VISP_HAVE_BICLOPS_FOUND "yes")  # for ViSP-third-party.txt
     #MESSAGE("DBG VISP_HAVE_BICLOPS= ${VISP_HAVE_BICLOPS}")
     #MESSAGE("DBG BICLOPS_INCLUDE_DIRS=${BICLOPS_INCLUDE_DIRS}")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${BICLOPS_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${BICLOPS_LIBRARIES})
     #MESSAGE("BICLOPS_LIBRARIES = ${BICLOPS_LIBRARIES}")
  ELSE(BICLOPS_FOUND)
    MESSAGE(STATUS "Biclops not found")
    MESSAGE("Biclops requested but not found. Turn off USE_BICLOPS option!")
    SET(VISP_HAVE_BICLOPS_FOUND "no")  # for ViSP-third-party.txt
    # SET(USE_BICLOPS OFF)
  ENDIF(BICLOPS_FOUND)
ENDIF(USE_BICLOPS)

# library for Ptu-46 pan-tilt head
IF(USE_PTU46)
#   FIND_PACKAGE(PTU46)
  IF(PTU46_FOUND)
    MESSAGE(STATUS "Ptu46 found")
    SET(VISP_HAVE_PTU46 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_PTU46_FOUND "yes")  # for ViSP-third-party.txt
     #MESSAGE("DBG VISP_HAVE_PTU46= ${VISP_HAVE_PTU46}")
     #MESSAGE("DBG PTU46_INCLUDE_DIRS=${PTU46_INCLUDE_DIRS}")
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${PTU46_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${PTU46_LIBRARIES})
     #MESSAGE("PTU46_LIBRARIES = ${PTU46_LIBRARIES}")
  ELSE(PTU46_FOUND)
    MESSAGE(STATUS "Ptu46 not found")
    MESSAGE("Ptu-46 requested but not found. Turn off USE_PTU46 option!")
    SET(VISP_HAVE_PTU46_FOUND "no")  # for ViSP-third-party.txt
    # SET(USE_PTU46 OFF)
  ENDIF(PTU46_FOUND)
ENDIF(USE_PTU46)

# library for Cycab car-like mobile robot
IF(USE_CYCAB)
  IF(CycabTk_FOUND)
    IF(CycabTk_NEW_FOUND)
      SET(VISP_HAVE_CYCABTK TRUE) # for header vpConfig.h
    ENDIF(CycabTk_NEW_FOUND)
    IF(CycabTk_OLD_FOUND)
      SET(VISP_HAVE_CYCABTK_OLD TRUE) # for header vpConfig.h
    ENDIF(CycabTk_OLD_FOUND)
    SET(VISP_HAVE_CYCAB TRUE) # for header vpConfig.h
    SET(VISP_HAVE_CYCAB_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${CycabTk_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${CycabTk_LIBRARIES})
  ELSE(CycabTk_FOUND)
    MESSAGE(STATUS "Cycab interface not found")
    MESSAGE("Cycab requested but not found. Turn off USE_CYCAB option!")
    SET(VISP_HAVE_CYCAB_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(CycabTk_FOUND)
ENDIF(USE_CYCAB)

# libraries for Pioneer mobile robots
if(USE_ARIA)
  if(UNIX)
    # Under Unix we need Aria, pthread, dl and rt libraries
    if(ARIA_FOUND AND PTHREAD_FOUND AND RT_FOUND AND DL_FOUND)
      message(STATUS "Aria found")
      SET(VISP_HAVE_PIONEER TRUE) # for header vpConfig.h
      SET(VISP_HAVE_PIONEER_FOUND "yes")  # for ViSP-third-party.txt
      LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${ARIA_INCLUDE_DIRS})
      LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${PTHREAD_INCLUDE_DIRS})
      LIST(APPEND VISP_EXTERN_LIBRARIES ${ARIA_LIBRARIES})
      LIST(APPEND VISP_EXTERN_LIBRARIES ${PTHREAD_LIBRARIES})
      LIST(APPEND VISP_EXTERN_LIBRARIES ${RT_LIBRARIES})
      LIST(APPEND VISP_EXTERN_LIBRARIES ${DL_LIBRARIES})
    endif()
  else(UNIX)
    if(ARIA_FOUND)
      message(STATUS "Aria found")
      SET(VISP_HAVE_PIONEER TRUE) # for header vpConfig.h
      SET(VISP_HAVE_PIONEER_FOUND "yes")  # for ViSP-third-party.txt
      LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${ARIA_INCLUDE_DIRS})
      LIST(APPEND VISP_EXTERN_LIBRARIES ${ARIA_LIBRARIES})
    else()
      message(STATUS "Aria not found")
    endif()
  endif(UNIX)
endif()


#----------------------------------------------------------------------
# add html-doc target for doxygen documentation building
#----------------------------------------------------------------------
FIND_PACKAGE(Doxygen)
IF (DOXYGEN_FOUND)
  SET(VISP_HAVE_DOXYGEN "yes")        # for header vpConfig.h
  SET(VISP_HAVE_DOXYGEN_FOUND "yes")  # for ViSP-third-party.txt
  IF(DOXYGEN_DOT_EXECUTABLE)
    SET(VISP_HAVE_DOT "yes")        # for header vpConfig.h
    SET(VISP_HAVE_DOT_FOUND "yes")  # for ViSP-third-party.txt
  ELSE(DOXYGEN_DOT_EXECUTABLE)
    SET(VISP_HAVE_DOT "no")        # for header vpConfig.h
    SET(VISP_HAVE_DOT_FOUND "no")  # for ViSP-third-party.txt
  ENDIF(DOXYGEN_DOT_EXECUTABLE)
  ## we need latex for doxygen because of the formulas
  FIND_PACKAGE(LATEX)
  IF    (NOT LATEX_COMPILER)
    MESSAGE(STATUS "latex command LATEX_COMPILER not found but usually required. You will probably get warnings and user interaction on doxy run.")
  ENDIF (NOT LATEX_COMPILER)
  IF    (NOT MAKEINDEX_COMPILER)
    MESSAGE(STATUS "makeindex command MAKEINDEX_COMPILER not found but usually required.")
  ENDIF (NOT MAKEINDEX_COMPILER)
  IF    (NOT DVIPS_CONVERTER)
    MESSAGE(STATUS "dvips command DVIPS_CONVERTER not found but usually required.")
  ENDIF (NOT DVIPS_CONVERTER)
   
  CONFIGURE_FILE(${VISP_SOURCE_DIR}/doc/config-doxygen.in
    ${VISP_DOC_DIR}/config-doxygen
    @ONLY )

  CONFIGURE_FILE(${VISP_SOURCE_DIR}/doc/mainpage.doc.in
    ${VISP_SOURCE_DIR}/doc/mainpage.doc
    @ONLY )
  ADD_CUSTOM_TARGET(html-doc ${DOXYGEN_EXECUTABLE} ${VISP_DOC_DIR}/config-doxygen) 
ELSE(DOXYGEN_FOUND)
  SET(VISP_HAVE_DOXYGEN "no")        # for header vpConfig.h
  SET(VISP_HAVE_DOXYGEN_FOUND "no")  # for ViSP-third-party.txt
  SET(VISP_HAVE_DOT_FOUND "no")      # for ViSP-third-party.txt
ENDIF(DOXYGEN_FOUND)

#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
# With Visual Studio 2005, Microsoft deprecates the standard C library, for
# example fopen() and sprintf(), to non-portable functions fopen_s() and
# sprintf_s(). These functions are considered by Microsoft more secure. This is
# a worthwhile exercise ! The use of these deprecated functions causes a lot of
# warnings. To suppress it, we add the _CRT_SECURE_NO_DEPRECATE preprocessor
# definition
IF(WIN32)
  IF(MSVC80 OR MSVC90 OR MSVC10)
    #MESSAGE("Add -D_CRT_SECURE_NO_DEPRECATE ")
    list(APPEND VISP_DEFS "-D_CRT_SECURE_NO_DEPRECATE")
  ENDIF(MSVC80 OR MSVC90 OR MSVC10)
ENDIF(WIN32)

#MESSAGE("Definitions: ${VISP_DEFS}")
IF(VISP_DEFS)
  LIST(REMOVE_DUPLICATES VISP_DEFS)
ENDIF(VISP_DEFS)
#MESSAGE("Definitions without duplicates: ${VISP_DEFS}")
ADD_DEFINITIONS(${VISP_DEFS})

#----------------------------------------------------------------------
# Add intern and third party include dirs
#----------------------------------------------------------------------
#MESSAGE("VISP_EXTERN_INCLUDE_DIRS: ${VISP_EXTERN_INCLUDE_DIRS}")
IF(VISP_EXTERN_INCLUDE_DIRS)
  LIST(REMOVE_DUPLICATES VISP_EXTERN_INCLUDE_DIRS)
ENDIF(VISP_EXTERN_INCLUDE_DIRS)
#MESSAGE("VISP_EXTERN_INCLUDE_DIRS without duplicates: ${VISP_EXTERN_INCLUDE_DIRS}")
INCLUDE_DIRECTORIES(${VISP_INTERN_INCLUDE_DIR} ${VISP_EXTERN_INCLUDE_DIRS})

#----------------------------------------------------------------------
# Add link directories
#----------------------------------------------------------------------
#MESSAGE("VISP_EXTERN_LINK_DIR: ${VISP_EXTERN_LINK_DIR}")
IF(VISP_EXTERN_LINK_DIR)
  LIST(REMOVE_DUPLICATES VISP_EXTERN_LINK_DIR)
ENDIF(VISP_EXTERN_LINK_DIR)
#MESSAGE("VISP_EXTERN_LINK_DIR without duplicates: ${VISP_EXTERN_LINK_DIR}")
LINK_DIRECTORIES(${VISP_EXTERN_LINK_DIR})

#----------------------------------------------------------------------
# Cleanify the libraries list
#----------------------------------------------------------------------
IF(APPLE)
  LIST(REMOVE_DUPLICATES VISP_EXTERN_LIBRARIES)
ENDIF()

#----------------------------------------------------------------------
# Generate the package dependent visp-config shell script for projects which
# are not using CMake:
# Usage:
#    visp-config --cflags ...
#----------------------------------------------------------------------
INCLUDE(${VISP_CMAKE_MODULE_PATH}/GenerateConfigScript.cmake)

#----------------------------------------------------------------------
# add distclean target
#----------------------------------------------------------------------
INCLUDE(${VISP_CMAKE_MODULE_PATH}/TargetDistclean.cmake)

#----------------------------------------------------------------------
# customize clean target
#----------------------------------------------------------------------
INCLUDE(${VISP_SOURCE_DIR}/CMakeHeaderFileList.cmake)

SET(FILE_TO_CLEAN "core*;*~;gmon.out;DartTestfile.txt")
FOREACH(header ${HEADER_ALL})
  GET_FILENAME_COMPONENT(headerName ${header} NAME)
  LIST(APPEND FILE_TO_CLEAN ${CMAKE_SOURCE_DIR}/include/visp/${headerName})
ENDFOREACH(header)

SET_DIRECTORY_PROPERTIES(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES "${FILE_TO_CLEAN}"
)

#----------------------------------------------------------------------
# customize uninstall target
#----------------------------------------------------------------------
CONFIGURE_FILE(
  "${VISP_CMAKE_MODULE_PATH}/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)

ADD_CUSTOM_TARGET(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake")

#----------------------------------------------------------------------
# Configure the file describing how to use ViSP. VISPConfig.cmake
# is the main file configuring a CMake package.
# . Exports build settings and dependencies for projects using ViSP as a
#   third party project.
# . Create and install files for simple use of FIND_PACKAGE(VISP)
#   by other cmakified "user" projects and libraries depending on ViSP.
#   (see "Mastering CMake", pp.72)
# . To use ViSP in a third party project based on CMake:
#   FIND_PACKAGE(VISP REQUIRED)
#   IF(VISP_FOUND)
#     INCLUDE(${VISP_USE_FILE})
#   ENDIF(VISP_FOUND)
#----------------------------------------------------------------------

# Installation for ViSPConfig.cmake
# case 1: <binary dir>/ViSPConfig.cmake for use with make
# case 2: <install dir>/ViSPConfig.cmake for use with make install

# case 1
#--------------

set(VISP_ROOT_DIR_CONFIGCMAKE "${VISP_BINARY_DIR}")
set(VISP_ROOT_DIR_CMAKE_CONFIGCMAKE "${VISP_BINARY_DIR}")

SET(VISP_SCENES_DIR ${VISP_ROOT_DIR_CMAKE_CONFIGCMAKE}/data/wireframe-simulator)
SET(VISP_ROBOT_ARMS_DIR ${VISP_ROOT_DIR_CMAKE_CONFIGCMAKE}/data/robot-simulator)
    
# Generate the package dependent file include/visp/vpConfig.h
CONFIGURE_FILE(${VISP_SOURCE_DIR}/include/vpConfig.h.cmake
  ${VISP_INCLUDE_DIR}/vpConfig.h
)

CONFIGURE_FILE(
  ${VISP_CMAKE_MODULE_PATH}/VISPConfig.cmake.in
  ${VISP_BINARY_DIR}/VISPConfig.cmake
  IMMEDIATE @ONLY)

# Just to copy ./CMakeModule/VISPUse.cmake.in <binary dir>/UseVISP.cmake
CONFIGURE_FILE(
  ${VISP_CMAKE_MODULE_PATH}/VISPUse.cmake.in
  ${VISP_BINARY_DIR}/VISPUse.cmake
  IMMEDIATE @ONLY)

# Generate the package dependent file include/visp/vpConfig.h
CONFIGURE_FILE(${VISP_SOURCE_DIR}/ViSP-third-party.txt.cmake
  ${VISP_BINARY_DIR}/ViSP-third-party.txt
)


# case 2
#--------------

set(VISP_ROOT_DIR_CONFIGCMAKE "${CMAKE_INSTALL_PREFIX}")
IF(UNIX)
  set(VISP_ROOT_DIR_CMAKE_CONFIGCMAKE "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/visp")
ELSE()
  set(VISP_ROOT_DIR_CMAKE_CONFIGCMAKE "${CMAKE_INSTALL_PREFIX}")
ENDIF()

SET(VISP_SCENES_DIR ${VISP_ROOT_DIR_CMAKE_CONFIGCMAKE}/data/wireframe-simulator)
SET(VISP_ROBOT_ARMS_DIR ${VISP_ROOT_DIR_CMAKE_CONFIGCMAKE}/data/robot-simulator)
# Only if resources.cfg created by hand, we change the path to resources.cfg in install/vpConfig.h
if(VISP_INSTALL_DIR_OGRE_RESOURCES)
  SET(VISP_HAVE_OGRE_RESOURCES_PATH ${VISP_ROOT_DIR_CMAKE_CONFIGCMAKE}/data/ogre-simulator)
endif()
# Only if plugins.cfg created by hand, we change the path to plugins.cfg in install/vpConfig.h
if(VISP_INSTALL_DIR_OGRE_RESOURCES)
  SET(VISP_HAVE_OGRE_PLUGINS_PATH ${VISP_ROOT_DIR_CMAKE_CONFIGCMAKE}/data/ogre-simulator)
endif()

# Generate the package dependent file include/visp/vpConfig.h
CONFIGURE_FILE(${VISP_SOURCE_DIR}/include/vpConfig.h.cmake
  ${VISP_BINARY_DIR}/install/vpConfig.h
)

CONFIGURE_FILE(
  ${VISP_CMAKE_MODULE_PATH}/VISPConfig.cmake.in
  ${VISP_BINARY_DIR}/install/VISPConfig.cmake
  IMMEDIATE @ONLY)

# Just to copy ./CMakeModule/VISPUse.cmake.in in <install dir>/UseVISP.cmake
CONFIGURE_FILE(
  ${VISP_CMAKE_MODULE_PATH}/VISPUse.cmake.in
  ${VISP_BINARY_DIR}/install/VISPUse.cmake
  IMMEDIATE @ONLY)

# Export our build settings and library dependencies for use by the third party
# user projects
INCLUDE(${CMAKE_ROOT}/Modules/CMakeExportBuildSettings.cmake)
CMAKE_EXPORT_BUILD_SETTINGS(
  ${VISP_BINARY_DIR}/install/VISPBuildSettings.cmake
  )

# Install vpConfig.h that may differ from the one in include/visp
INSTALL(FILES ${VISP_BINARY_DIR}/install/vpConfig.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/visp
  PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
  COMPONENT headers
  )

# Create and install visp-config.1.gz man page
if(UNIX)
  FIND_PROGRAM(GZIP gzip)
  file(MAKE_DIRECTORY ${VISP_BINARY_DIR}/doc/man/man1)
  ADD_CUSTOM_COMMAND(
    OUTPUT ${VISP_BINARY_DIR}/doc/man/man1/visp-config.1.gz
    COMMAND ${GZIP} --best -c ${CMAKE_CURRENT_SOURCE_DIR}/doc/man/man1/visp-config.1 > ${VISP_BINARY_DIR}/doc/man/man1/visp-config.1.gz
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/doc/man/man1/visp-config.1
  )
  ADD_CUSTOM_TARGET(man ALL
    DEPENDS ${VISP_BINARY_DIR}/doc/man/man1/visp-config.1.gz
  )
  INSTALL(FILES
    ${VISP_BINARY_DIR}/doc/man/man1/visp-config.1.gz
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/man/man1
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
    COMPONENT libraries
  )
  MARK_AS_ADVANCED(GZIP)
endif()

# Install the packaging files for use by FIND_PACKAGE(VISP) in user projects
# install rule for vpConfig.h:
# we need to adapt VISP_SCENES_DIR and VISP_ROBOT_ARMS_DIR
# here set to /usr/share/visp/data
IF(UNIX)
  INSTALL(FILES
    ${VISP_BINARY_DIR}/install/VISPUse.cmake
    ${VISP_BINARY_DIR}/install/VISPConfig.cmake
    ${VISP_BINARY_DIR}/install/VISPBuildSettings.cmake
    ${VISP_BINARY_DIR}/ViSP-third-party.txt
    DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/visp
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
    COMPONENT libraries
  )
  foreach(SCENE ${WIREFRAME_SCENES})
    INSTALL(FILES 
      "${VISP_SOURCE_DIR}/src/simulator/wireframe-simulator/scene/${SCENE}"
      DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/visp/data/wireframe-simulator
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      COMPONENT libraries
    )
  endforeach()
  foreach(SCENE ${ROBOT_ARMS_SCENES})
    INSTALL(FILES 
      "${VISP_SOURCE_DIR}/src/robot/simulator-robot/arms/${SCENE}"
      DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/visp/data/robot-simulator
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      COMPONENT libraries
    )
  endforeach()
ELSE()
  INSTALL(FILES
    ${VISP_BINARY_DIR}/install/VISPUse.cmake
    ${VISP_BINARY_DIR}/install/VISPConfig.cmake
    ${VISP_BINARY_DIR}/install/VISPBuildSettings.cmake
    DESTINATION .
    PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
    COMPONENT libraries
  )
  foreach(SCENE ${WIREFRAME_SCENES})
    INSTALL(FILES 
      "${VISP_SOURCE_DIR}/src/simulator/wireframe-simulator/scene/${SCENE}"
      DESTINATION data/wireframe-simulator
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      COMPONENT libraries
    )
  endforeach()
  foreach(SCENE ${ROBOT_ARMS_SCENES})
    INSTALL(FILES 
      "${VISP_SOURCE_DIR}/src/robot/simulator-robot/arms/${SCENE}"
      DESTINATION data/robot-simulator
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      COMPONENT libraries
    )
  endforeach()
ENDIF()

#----------------------------------------------------------------------
# For Dart server and tests 
# We use CDash set through CTestConfig.cmake file
# Dashboards are sent to http://cdash.irisa.fr/CDash/index.php?project=ViSP
#----------------------------------------------------------------------
ENABLE_TESTING()
INCLUDE(Dart)
MARK_AS_ADVANCED(DART_ROOT)

#----------------------------------------------------------------------
# For CPack packaging tool
#----------------------------------------------------------------------

option(BUILD_PACKAGE "Configure ViSP packaging" OFF)
if (BUILD_PACKAGE)
  if(UNIX AND NOT APPLE AND NOT WIN32) # =linux
    option(BUILD_PACKAGE_DEBIAN "Build debian package" ON)
    option(BUILD_PACKAGE_RPM "Build rpm package" ON)
  endif()

  include (${VISP_SOURCE_DIR}/CMakeModules/CPackConfig.cmake)
endif(BUILD_PACKAGE)


#----------------------------------------------------------------------
# Propagation in sub dirs
#----------------------------------------------------------------------
SUBDIRS(src)
if(BUILD_DEMOS)
  subdirs(demo)
endif()
if(BUILD_EXAMPLES)
  subdirs(example)
endif()
if(BUILD_TUTORIAL)
  set(VISP_DIR ${PROJECT_BINARY_DIR})
  mark_as_advanced(VISP_DIR)
  mark_as_advanced(VISP_INCLUDE_DIRS)
  mark_as_advanced(VISP_LIBRARY_DIRS)
  subdirs(tutorial)
endif()

IF(BUILD_TESTING)
  #
  # Test coverage specific code
  #
  IF(CMAKE_COMPILER_IS_GNUCXX AND NOT BUILD_SHARED_LIBS AND CMAKE_BUILD_TYPE MATCHES "Debug")

    OPTION(BUILD_TEST_COVERAGE "Enable test coverage." OFF)
  ENDIF()

  IF(BUILD_TEST_COVERAGE)
    # Add build options for test coverage. Currently coverage is only supported
    # on gcc compiler
    # Because using -fprofile-arcs with shared lib can cause problems like:
    # hidden symbol `__bb_init_func', we add this option only for static
    # library build
    MESSAGE(STATUS "Add -ftest-coverage -fprofile-arcs compiler options")
    SET(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ftest-coverage -fprofile-arcs")
  ENDIF(BUILD_TEST_COVERAGE)

  SUBDIRS(test)
ENDIF(BUILD_TESTING)
