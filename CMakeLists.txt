#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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

# Need to be befor project(VISP) to work
if(WIN32)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")
endif()

project(VISP C CXX)

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

#-----------------------------------------------------------------------------
# VISP version number.  An even minor number corresponds to releases.
SET(VISP_VERSION_MAJOR "2")
SET(VISP_VERSION_MINOR "11")
SET(VISP_VERSION_PATCH "0")
SET(VISP_VERSION "${VISP_VERSION_MAJOR}.${VISP_VERSION_MINOR}.${VISP_VERSION_PATCH}")
# Package revision number
SET(VISP_REVISION "1")

# where are user-specific cmake modules
set(VISP_CMAKE_MODULE_PATH ${VISP_SOURCE_DIR}/cmake)

find_file(GNU_INSTALL_DIRS_FROM_CMAKE NAMES GNUInstallDirs.cmake PATHS ${CMAKE_ROOT}/Modules)
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
  set(VISP_DEBUG_POSTFIX "d")
  set(VISP_DLLVERSION "${VISP_VERSION_MAJOR}${VISP_VERSION_MINOR}${VISP_VERSION_PATCH}")
else()
  set(VISP_DEBUG_POSTFIX "")
  set(VISP_DLLVERSION "")
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
OPTION(BUILD_SHARED_LIBS "Build ViSP shared libraries (.dll/.so) instead of static ones (.lib/.a)." ON)
# Build examples as an option.
OPTION(BUILD_EXAMPLES "Build ViSP examples." ON)
# Build examples as an option.
OPTION(BUILD_TESTS "Build ViSP tests." ON)
# Build demos as an option.
OPTION(BUILD_DEMOS "Build ViSP demos." ON)
# Build demos as an option.
OPTION(BUILD_TUTORIALS "Build ViSP tutorials." ON)
# Build deprecated functions as an option.
OPTION(BUILD_DEPRECATED_FUNCTIONS "Build deprecated functionalities." ON)
# Debug and trace cflags
option(ACTIVATE_DEBUG_TRACE "Enable debug and trace printings" ON)

if(ACTIVATE_DEBUG_TRACE)
  list(APPEND VISP_DEFS "-DVP_TRACE")
  list(APPEND VISP_DEFS "-DVP_DEBUG")
else()
  string(REPLACE "-DVP_TRACE" "" " " VISP_DEFS "${VISP_DEFS}")
  string(REPLACE "-DVP_DEBUG" "" " " VISP_DEFS "${VISP_DEFS}")
endif()

if(MSVC)
  option(BUILD_WITH_STATIC_CRT    "Enables use of staticaly linked CRT for staticaly linked ViSP" ON)
endif()

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
  find_package(DC1394)
  if(DC1394_FOUND)
    option(USE_DC1394    "Compile ViSP with the libdc1394.2 library" ON)
  else()
    set(USE_DC1394 OFF)
  endif()

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
IF(LAPACK_C_FOUND)
    OPTION(USE_LAPACK    "Compile ViSP with the lapack/blas libraries" ON)
ELSE()
    SET(USE_LAPACK OFF)
ENDIF()

#IF(NOT USE_LAPACK) # line removed since the template tracker needs gsl
  find_package(GSL)
  if(GSL_FOUND)
    option(USE_GSL    "Compile ViSP with the GSL library" ON)
  else()
    set(USE_GSL OFF)
  endif()
#ENDIF()

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

FIND_PACKAGE(LIBUSB_1)
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
MARK_AS_ADVANCED(OpenCV_DIR OpenCV_FOUND OPENCV_FOUND)
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

find_package(FFMPEG)
if(FFMPEG_FOUND)
  if(USE_OPENCV)
    option(USE_FFMPEG    "Compile ViSP with the ffmpeg library" OFF)
  else()
    option(USE_FFMPEG    "Compile ViSP with the ffmpeg library" ON)
  endif()
else()
  set(USE_FFMPEG OFF)
endif()

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

find_package(ZBAR)
if(ZBAR_FOUND)
  OPTION(USE_ZBAR    "Compile ViSP with zbar library" ON)
else()
  SET(USE_ZBAR OFF)
endif()

find_package(DMTX)
if(DMTX_FOUND)
  OPTION(USE_DMTX    "Compile ViSP with zbar library" ON)
else()
  SET(USE_DMTX OFF)
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
#SET(USE_DC1394     ON) # For firewire grabber under Linux and OSX ?
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

#----------------------------------------------------------------------
# Try to find doxygen for documentation generation
# Use "make visp_doc" target to generate the documentation
#----------------------------------------------------------------------
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(VISP_HAVE_DOXYGEN "yes")        # for header vpConfig.h
  set(VISP_HAVE_DOXYGEN_FOUND "yes")  # for ViSP-third-party.txt
  if(DOXYGEN_DOT_EXECUTABLE)
    set(VISP_HAVE_DOT "yes")        # for header vpConfig.h
    set(VISP_HAVE_DOT_FOUND "yes")  # for ViSP-third-party.txt
  else()
    set(VISP_HAVE_DOT "no")        # for header vpConfig.h
    set(VISP_HAVE_DOT_FOUND "no")  # for ViSP-third-party.txt
  endif()
  ## we need latex for doxygen because of the formulas
  find_package(LATEX)
  if(NOT LATEX_COMPILER)
    message(STATUS "latex command LATEX_COMPILER not found but usually required. You will probably get warnings and user interaction on doxy run.")
  endif()
  if(NOT MAKEINDEX_COMPILER)
    message(STATUS "makeindex command MAKEINDEX_COMPILER not found but usually required.")
  endif()
  if(NOT DVIPS_CONVERTER)
    message(STATUS "dvips command DVIPS_CONVERTER not found but usually required.")
  endif()

  configure_file(${VISP_SOURCE_DIR}/doc/config-doxygen.in
    ${VISP_DOC_DIR}/config-doxygen
    @ONLY )

  configure_file(${VISP_SOURCE_DIR}/doc/mainpage.doc.in
    ${VISP_DOC_DIR}/mainpage.doc
    @ONLY )
else()
  set(VISP_HAVE_DOXYGEN "no")        # for header vpConfig.h
  set(VISP_HAVE_DOXYGEN_FOUND "no")  # for ViSP-third-party.txt
  set(VISP_HAVE_DOT_FOUND "no")      # for ViSP-third-party.txt
endif()

# ----------------------------------------------------------------------------
# Solution folders:
# ----------------------------------------------------------------------------
if(MSVC_IDE OR CMAKE_GENERATOR MATCHES Xcode)
  option(ENABLE_SOLUTION_FOLDERS "Solution folder in Visual Studio or in other IDEs" ON)
endif()

if(ENABLE_SOLUTION_FOLDERS)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
  set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif()

# Extra ViSP targets: uninstall, etc.
include(${VISP_CMAKE_MODULE_PATH}/VISPExtraTargets.cmake)


INCLUDE(CheckLibraryExists)

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
    
  # Work around to add Yarp libraries and also third party libraries requested by Yarp
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
    if(YARP_LINK_LIBS)
      foreach(lib ${YARP_LIBRARIES})
        list(REMOVE_ITEM YARP_LINK_LIBS ${lib})
      endforeach()
    endif()
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
  if(BUILD_DEPRECATED_FUNCTIONS)
    list(APPEND VISP_DEFS "-DWIN32") # only for compat with previous versions
  endif()
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
      list(APPEND VISP_EXTERN_LIBRARIES "ws2_32.a")
	else()
	  find_library(WS2_32_LIBRARY "libws2_32.a"
        "$ENV{MINGW_DIR}/lib"
        "$ENV{MINGW_DIR}/mingw/lib"
        C:/mingw/mingw/lib)
	  mark_as_advanced(WS2_32_LIBRARY)
	  if(WS2_32_LIBRARY)
	    list(APPEND VISP_EXTERN_LIBRARIES "${WS2_32_LIBRARY}")
	  endif()
    endif()
  else() # pure WIN32
    CHECK_LIBRARY_EXISTS("ws2_32.lib" getch "" HAVE_LIBWS2_32) # for inet_ntoa() and socket functionalities
    if(HAVE_LIBWS2_32)
      #message("have ws2_32.lib")
      list(APPEND VISP_EXTERN_LIBRARIES "ws2_32.lib")
    endif()
  endif()
ENDIF(WIN32 AND NOT CYGWIN)
if(UNIX)
  if(BUILD_DEPRECATED_FUNCTIONS)
    list(APPEND VISP_DEFS "-DUNIX")
    if(APPLE)
      list(APPEND VISP_DEFS "-DAPPLE")
    endif()
    if(${CMAKE_SYSTEM_NAME} MATCHES "SunOS")
      list(APPEND VISP_DEFS "-DSOLARIS")
    endif()
  endif()
endif()

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
SET(VISP_HAVE_LAPACK_C_FOUND "no")      # for ViSP-third-party.txt
IF(USE_LAPACK)
  MESSAGE(STATUS "lapack/blas found")
  SET(VISP_HAVE_LAPACK_C TRUE)          # for header vpConfig.h
  SET(VISP_HAVE_LAPACK_C_FOUND "yes")   # for ViSP-third-party.txt
  
  #MESSAGE("LAPACK_C_LIBRARIES = ${LAPACK_C_LIBRARIES}")
  LIST(APPEND VISP_EXTERN_LIBRARIES ${LAPACK_C_LIBRARIES})
ENDIF()

#--------------------------------------------------------------------
# X11 lib
#--------------------------------------------------------------------
set(VISP_HAVE_X11_FOUND "no")  # for ViSP-third-party.txt
if(USE_X11)
  message(STATUS "X11 found")
  set(VISP_HAVE_X11 TRUE)  # for header vpConfig.h
  set(VISP_HAVE_X11_FOUND "yes")  # for ViSP-third-party.txt
  # try to found -lm requested on some platforms to link with X11
  list(APPEND VISP_EXTERN_INCLUDE_DIRS ${X11_INCLUDE_DIR})
  list(APPEND VISP_EXTERN_LIBRARIES ${X11_LIBRARIES})
  find_library(M_LIBRARY NAMES m)
  mark_as_advanced(M_LIBRARY)
  if(M_LIBRARY)
    list(APPEND VISP_EXTERN_LIBRARIES ${M_LIBRARY})
  endif()
  #MESSAGE("X11: ${X11_LIBRARIES}")
else()
  # try to found -lsocket -lnsl requested for vpNetwork and vpSickLDMRS
  find_library(SOCKET_LIBRARY NAMES socket)
  find_library(NSL_LIBRARY NAMES nsl)
  if (SOCKET_LIBRARY)
    list(APPEND VISP_EXTERN_LIBRARIES ${SOCKET_LIBRARY})
  endif()
  if (NSL_LIBRARY)
    list(APPEND VISP_EXTERN_LIBRARIES ${NSL_LIBRARY})
  endif()
  mark_as_advanced(SOCKET_LIBRARY NSL_LIBRARY)
endif()


#--------------------------------------------------------------------
# OpenGL, Qt, SoQt and Coin libs
#--------------------------------------------------------------------
# default initialisation
SET(VISP_HAVE_SIMULATOR_FOUND "no") # for ViSP-third-party.txt
SET(VISP_HAVE_COIN3D_FOUND "no")      # for ViSP-third-party.txt
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
    #set(Boost_ADDITIONAL_VERSIONS "1.53.0" "1.52.0" "1.51.0" "1.50.0" "1.49.0" "1.48.0" "1.47.0" "1.46.0" "1.45.0" "1.44.0" "1.44" "1.44.0" "1.42" "1.42.0" "1.41.0" "1.41" "1.40.0" "1.40" "1.39.0" "1.39" "1.38.0" "1.38" "1.37.0" "1.37" )
    # Components that need to be linked with. Since Ogre 1.9 we need not only boost_thread and boost_date_time, but also boost_system 
    set(OGRE_BOOST_COMPONENTS thread system date_time)
    if(WIN32)
      list(APPEND OGRE_BOOST_COMPONENTS chrono)
    endif()
    find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
    mark_as_advanced(Boost_LIB_DIAGNOSTIC_DEFINITIONS Boost_DIR BOOST_THREAD_LIBRARY)
    if (NOT Boost_FOUND)
      set(OGRE_BOOST_COMPONENTS thread date_time)
      if(WIN32)
        list(APPEND OGRE_BOOST_COMPONENTS chrono)
      endif()
      find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
    endif()
    if (NOT Boost_FOUND)
      # Try again with the other type of libs
      set(Boost_USE_STATIC_LIBS NOT ${Boost_USE_STATIC_LIBS})
      find_package(Boost COMPONENTS ${OGRE_BOOST_COMPONENTS} QUIET)
    endif()
    
    if (Boost_FOUND)
      # Set up referencing of Boost
      #LIST(APPEND VISP_DEFS "-DBOOST_ALL_NO_LIB")
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
      /usr/share/OGRE-1.9.0/media	
    )
  endif()

  #message("OGRE_PLUGIN_DIR_REL: ${OGRE_PLUGIN_DIR_REL}")
  #message("OGRE_PLUGIN_DIR_DBG: ${OGRE_PLUGIN_DIR_DBG}")
  SET(VISP_HAVE_OGRE_FOUND "yes")   # for ViSP-third-party.txt
  SET(VISP_HAVE_OGRE TRUE)          # for header vpConfig.h

  # If Ogre media are not available we provide the minimal material to run the examples:
  # - resources.cfg
  # - plugins.cfg
  # - media/materials/...
  # - media/models/...
  #
  # We need to introduce OGRE_MEDIA_NOT_AVAILABLE to memorize when OGRE_MEDIA_DIR is not set.
  # Because in that case, OGRE_MEDIA_DIR should be set first to VISP_HAVE_OGRE_RESOURCES_PATH
  #  (for the "make all" case) then to VISP_INSTALL_DIR_OGRE_RESOURCES (for the "make install" case)
  if(NOT OGRE_MEDIA_DIR)
    set(OGRE_MEDIA_NOT_AVAILABLE "TRUE")
  endif()

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
    # install rule for plugins.cfg:
    IF(UNIX)
      if(OGRE_PLUGIN_DIR_REL)
        INSTALL(FILES
          ${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins.cfg
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/visp/data/ogre-simulator
          PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT libraries
          )
      endif()
      if(OGRE_PLUGIN_DIR_DBG)
        INSTALL(FILES
          ${VISP_HAVE_OGRE_PLUGINS_PATH}/plugins_d.cfg
          DESTINATION ${CMAKE_INSTALL_LIBDIR}/visp/data/ogre-simulator
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

  # Here we copy all the minimal media files
  # - media/materials/...
  # - media/models/...
  if(OGRE_MEDIA_NOT_AVAILABLE)
    file(COPY data/ogre-simulator/media DESTINATION ${VISP_BINARY_DIR}/data/ogre-simulator)
  endif()


  # Here we create a resources.cfg if it was not found
  IF(NOT VISP_HAVE_OGRE_RESOURCES_PATH)
    # we create a resources.cfg file for vpAROgre.cpp
    # case 1: normal case
    #         If OGRE_MEDIA_DIR is not found, we set it to VISP_HAVE_OGRE_RESOURCES_PATH in order to use
    #         the minimal requested media to run the examples
    #--------------
    SET(VISP_HAVE_OGRE_RESOURCES_PATH ${VISP_BINARY_DIR}/data/ogre-simulator)
    if(OGRE_MEDIA_NOT_AVAILABLE)
      set(OGRE_MEDIA_DIR ${VISP_HAVE_OGRE_RESOURCES_PATH}/media)
    endif()

    # On Fedora 20 when ogre-devel and ogre-samples packages are installed, AROgre and
    # AROgreBasic examples produce a segfault when the folowing line in resource.cf.in
    # is commented:
    # '#FileSystem=@OGRE_MEDIA_DIR@/materials/programs'
    # Here we check if @OGRE_MEDIA_DIR@/materials/programs forder is empty.
    # If empty, we comment the line to avoid lintian warnings.
    # If not empty we remove the comment to use the available programs.
    file(GLOB OGRE_PROGRAM_CONTENT ${OGRE_MEDIA_DIR}/materials/programs/*.*)
    if(OGRE_PROGRAM_CONTENT)
      set(OGRE_COMMENT_LINE "")
    else()
      set(OGRE_COMMENT_LINE "#")
    endif()
    CONFIGURE_FILE(
      ${VISP_CMAKE_MODULE_PATH}/resources.cfg.in
      ${VISP_HAVE_OGRE_RESOURCES_PATH}/resources.cfg
      IMMEDIATE @ONLY)

    # case 2: install or packaging case
    #         If OGRE_MEDIA_DIR is not found, we set it to VISP_INSTALL_DIR_OGRE_RESOURCES in order to use
    #         the minimal requested media to run the examples
    #--------------
    IF(UNIX)
      set(VISP_INSTALL_DIR_OGRE_RESOURCES "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/visp-${VISP_VERSION}/data/ogre-simulator")
    ELSE()
      set(VISP_INSTALL_DIR_OGRE_RESOURCES "${CMAKE_INSTALL_PREFIX}/data/ogre-simulator")
    ENDIF()

    if(OGRE_MEDIA_NOT_AVAILABLE)
      set(OGRE_MEDIA_DIR ${VISP_INSTALL_DIR_OGRE_RESOURCES}/media)
    endif()

    # install rule for resources.cfg and Ogre media if they are not available:
    if(UNIX)
      configure_file(
        ${VISP_CMAKE_MODULE_PATH}/resources.cfg.in
        ${VISP_BINARY_DIR}/unix-install/resources.cfg
        IMMEDIATE @ONLY)
      install(FILES
        ${VISP_BINARY_DIR}/unix-install/resources.cfg
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/visp-${VISP_VERSION}/data/ogre-simulator
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
        COMPONENT libraries
      )
      if(OGRE_MEDIA_NOT_AVAILABLE)
        install(DIRECTORY
          data/ogre-simulator/media
          DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/visp-${VISP_VERSION}/data/ogre-simulator
          FILE_PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT libraries
        )
      endif()
    else()
      configure_file(
        ${VISP_CMAKE_MODULE_PATH}/resources.cfg.in
        ${VISP_BINARY_DIR}/win-install/resources.cfg
        IMMEDIATE @ONLY)
      install(FILES
        ${VISP_BINARY_DIR}/win-install/resources.cfg
        DESTINATION data/ogre-simulator
        PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
        COMPONENT libraries
      )
      if(OGRE_MEDIA_NOT_AVAILABLE)
        install(DIRECTORY
          data/ogre-simulator/media
          DESTINATION data/ogre-simulator
          FILE_PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
          COMPONENT libraries
        )
      endif()
    endif()
  endif()
  
  MARK_AS_ADVANCED(VISP_HAVE_OGRE_PLUGINS_PATH)
  MARK_AS_ADVANCED(VISP_HAVE_OGRE_RESOURCES_PATH)
  #message("VISP_HAVE_OGRE_PLUGINS_PATH: ${VISP_HAVE_OGRE_PLUGINS_PATH}")
  #message("VISP_HAVE_OGRE_RESOURCES_PATH: ${VISP_HAVE_OGRE_RESOURCES_PATH}")
  
  # hack to fix possible presence of NOTFOUND in OGRE_INCLUDE_DIRS
  #message("OGRE_INCLUDE_DIRS: ${OGRE_INCLUDE_DIRS}")
  #LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${OGRE_INCLUDE_DIRS})  
  foreach(inc_ ${OGRE_INCLUDE_DIRS})
    if(NOT ${inc_} MATCHES "NOTFOUND")
	    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${inc_})  
	endif()
  endforeach()
  LIST(APPEND VISP_EXTERN_LIBRARIES ${OGRE_LIBRARIES})
ENDIF(USE_OGRE)

IF(USE_OIS AND USE_OGRE)
  SET(VISP_HAVE_OIS_FOUND "yes")   # for ViSP-third-party.txt
  SET(VISP_HAVE_OIS TRUE)           # for header vpConfig.h
  LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${OIS_INCLUDE_DIR})
  LIST(APPEND VISP_EXTERN_LIBRARIES ${OIS_LIBRARIES})   
  IF(APPLE)
    # With Ogre 1.7.4 and 1.8.1 to be able to link with libOIS.a, Cocoa framework is requested.
    # This is a work around since it should come with FindOGRE.cmake
    LIST(APPEND VISP_EXTERN_LIBRARIES "-framework Cocoa")
  ENDIF(APPLE)

ENDIF(USE_OIS AND USE_OGRE)

IF(USE_COIN)
  IF (COIN3D_FOUND)
    MESSAGE(STATUS "Coin3D found")
    SET(VISP_HAVE_COIN3D_FOUND "yes")   # for ViSP-third-party.txt
    SET(VISP_HAVE_COIN3D TRUE)          # for header vpConfig.h
    SET(VISP_HAVE_OPENGL TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_OPENGL_FOUND "yes")  # for ViSP-third-party.txt
    IF(WIN32)
      LIST(APPEND VISP_DEFS "-DCOIN_DLL")
    ENDIF(WIN32)
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${COIN3D_INCLUDE_DIRS})
    # On OSX cmake 2.8 found OpenGL but OPENGL_INCLUDE_DIR was set to NOT_FOUND
    # We add a test to be sure that the OPENGL vars exist.
    if(OPENGL_INCLUDE_DIR)
      list(APPEND VISP_EXTERN_INCLUDE_DIRS ${OPENGL_INCLUDE_DIR})
    endif()
    if(OPENGL_LIBRARIES)
      list(APPEND VISP_EXTERN_LIBRARIES ${OPENGL_LIBRARIES})
    endif()

    #MESSAGE("COIN3D_INCLUDE_DIRS = ${COIN3D_INCLUDE_DIRS}")
    #MESSAGE("COIN3D_LIBRARIES = ${COIN3D_LIBRARIES}")
    LIST(APPEND VISP_EXTERN_LIBRARIES ${COIN3D_LIBRARIES})
 
    IF (USE_SOWIN AND SOWIN_FOUND)
      # We manage SoWin and Coin
      SET(VISP_HAVE_SOWIN TRUE)         # for header vpConfig.h
      SET(VISP_HAVE_SOWIN_FOUND "yes")  # for ViSP-third-party.txt
      SET(VISP_HAVE_COIN3D_AND_GUI TRUE)  # for header vpConfig.h    
      
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
      	SET(VISP_HAVE_COIN3D_AND_GUI TRUE)  # for header vpConfig.h    

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
      SET(VISP_HAVE_COIN3D_AND_GUI TRUE)  # for header vpConfig.h    
      SET(VISP_HAVE_SOXT_FOUND "yes")       # for ViSP-third-party.txt
      LIST(APPEND VISP_EXTERN_LIBRARIES ${SOXT_LIBRARIES})
    ENDIF(USE_SOXT AND SOXT_FOUND)

    IF(VISP_HAVE_COIN3D_AND_GUI)
      SET(VISP_HAVE_COIN3D_AND_GUI_FOUND "yes")  # for ViSP-third-party.txt
      MESSAGE(STATUS "Coin3D + GUI found")
    ELSE(VISP_HAVE_COIN3D_AND_GUI)
      MESSAGE(STATUS "Coin3D + GUI not found")
      SET(VISP_HAVE_COIN3D_AND_GUI_FOUND "no")  # for ViSP-third-party.txt
    ENDIF(VISP_HAVE_COIN3D_AND_GUI)
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
  #FIND_PACKAGE(LIBUSB_1)
  IF(LIBUSB_1_FOUND)
    SET(VISP_HAVE_LIBUSB_1 TRUE)
    SET(VISP_HAVE_LIBUSB_1_FOUND "yes")  # for ViSP-third-party.txt
 
    MESSAGE(STATUS "libusb-1.0 found")
 
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${LIBUSB_1_INCLUDE_DIRS})
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
    SET(CMAKE_REQUIRED_INCLUDES ${LIBFREENECT_INCLUDE_DIRS} ${PTHREAD_INCLUDE_DIRS} ${LIBUSB_1_INCLUDE_DIRS})
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

# OpenCV
if(USE_OPENCV)
  #SET(OpenCV_FIND_QUIETLY TRUE)
  #FIND_PACKAGE(OpenCV)
  if(OpenCV_FOUND)
    message(STATUS "OpenCV found")
    set(VISP_HAVE_OPENCV  TRUE)  # for header vpConfig.h
    set(VISP_HAVE_OPENCV_FOUND "yes")  # for ViSP-third-party.txt
    # On win32 since OpenCV 2.4.7 and on OSX with OpenCV 2.4.10 we cannot use OpenCV_LIBS to set ViSP 3rd party libraries.
    # Using OpenCV_LIBS works to build visp library, examples, demos and test thanks to the components, 
    # but not tutorials that are stand alone Cmake project that use ViSP as a 3rd party.
    # To be clear OpenCV_LIBS contains opencv_core and not c:\...\opencv_core248.lib full path as requested
    # to use ViSP. This was not the case with OpenCV 2.4.6.
    # For the build of ViSP it works with OpenCV_LIBS: in that case thanks to opencv_core properties, CMake
    # is able to find the real name and location of the libraries.
    # But when ViSP is used as a 3rd party where it should import OpenCV libraries, it doesn't work with 
    # OpenCV_LIBS.
    # The solution here is to get the real name of OpenCV libraries thanks to the properties and link 
    # with these names.
    # An other way could be to include OpenCVConfig.cmake, but in that case, visp-config and visp.pc
    # will be not able to give the names of OpenCV libraries when used without CMake.
    #message("OpenCV_LIB_COMPONENTS: ${OpenCV_LIB_COMPONENTS}")
    #message("OpenCV_LIBS: ${OpenCV_LIBS}")
    #if(WIN32 AND OpenCV_LIB_COMPONENTS AND OpenCV_VERSION AND OpenCV_VERSION VERSION_GREATER 2.4.6.1)
    if(OpenCV_LIB_COMPONENTS AND OpenCV_VERSION AND OpenCV_VERSION VERSION_GREATER 2.4.6.1) 
      # ReleaseWithDebugInfo was requested to detect libopencv-devel under Fedora 20
      # RelWithDebugInfo was requested to detect ros-hydro-opencv2 2.4.9 under Ubuntu 12.04 LTS with ROS hydro   
      set(config_ "NONE" "RELEASE" "DEBUG" "RELEASEWITHDEBINFO" "RELWITHDEBINFO") 
      if(POLICY CMP0045)
        # Fix Error on non-existent target in get_target_property for 3rd party location extraction
        cmake_policy(PUSH)
        cmake_policy(SET CMP0045 OLD)
      endif()
      
      foreach(component_ ${OpenCV_LIB_COMPONENTS})
        foreach(imp_config_ ${config_})
          if(OpenCV_SHARED)
            get_target_property(component_property_${imp_config_}_ ${component_} IMPORTED_IMPLIB_${imp_config_})
            # particular case of opencv_ts that doesn't have an implib
            if(NOT EXISTS "${component_property_${imp_config_}_}")
              get_target_property(component_property_${imp_config_}_ ${component_} IMPORTED_LOCATION_${imp_config_})
            endif()
          else()
              get_target_property(component_property_${imp_config_}_ ${component_} IMPORTED_LOCATION_${imp_config_})
          endif()
          get_target_property(component_property_3rdparty_${imp_config_}_ ${component_} IMPORTED_LINK_INTERFACE_LIBRARIES_${imp_config_})
          #message("component_property_${imp_config_}_: ${component_property_${imp_config_}_}")
          #message("component_property_3rdparty_${imp_config_}_: ${component_property_3rdparty_${imp_config_}_}")

          # Under Unix, there is no specific suffix for OpenCV libraries. If one is found we add it
          # Under Windows, we add the "optimized", "debug" specific keywords
          if(WIN32 AND EXISTS "${component_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "RELEASE") # also valid for RELEASEWITHDEBINFO
            list(APPEND VISP_EXTERN_LIBRARIES optimized "${component_property_${imp_config_}_}")
          elseif(WIN32 AND EXISTS "${component_property_${imp_config_}_}" AND "${imp_config_}" MATCHES "DEBUG")
            list(APPEND VISP_EXTERN_LIBRARIES debug     "${component_property_${imp_config_}_}")
          elseif(EXISTS "${component_property_${imp_config_}_}")
            list(APPEND VISP_EXTERN_LIBRARIES "${component_property_${imp_config_}_}")
          endif()

          if(component_property_3rdparty_${imp_config_}_)
            foreach(3rdparty_ ${component_property_3rdparty_${imp_config_}_})
              #message("3rdparty_ ${3rdparty_}")
              list(FIND OpenCV_LIB_COMPONENTS ${3rdparty_} 3rdparty_is_opencv_component_)
              if(3rdparty_is_opencv_component_ LESS 0)
                #message("${3rdparty_} is not an opencv component")
                get_target_property(3rdparty_opt_location_ ${3rdparty_} IMPORTED_LOCATION_${imp_config_})
                if(NOT EXISTS "${3rdparty_opt_location_}")
                  #message("3rdparty_: ${3rdparty_} location doesn't exist in ${imp_config_}")
                  get_target_property(3rdparty_opt_location_ ${3rdparty_} IMPORTED_LOCATION)
                  #message("3rdparty_: ${3rdparty_} location : ${3rdparty_opt_location_}")
                endif()
                if(EXISTS "${3rdparty_opt_location_}")
                  #message("3rdparty_opt_location_: ${3rdparty_opt_location_} with config ${imp_config_}")
                  if(WIN32 AND "${imp_config_}" MATCHES "RELEASE")
                    #message("is release")
                    list(APPEND VISP_EXTERN_LIBRARIES optimized ${3rdparty_opt_location_})
                  elseif(WIN32 AND "${imp_config_}" MATCHES "DEBUG")
                    list(APPEND VISP_EXTERN_LIBRARIES debug ${3rdparty_opt_location_})
                  else()
                    list(APPEND VISP_EXTERN_LIBRARIES ${3rdparty_opt_location_})
                  endif()
                else()
                  find_library(3rdparty_location_ NAMES ${3rdparty_})
                  mark_as_advanced(3rdparty_location_)
                  if(3rdparty_location_)
	                  #message(${3rdparty_location_})
    	              list(APPEND VISP_EXTERN_LIBRARIES ${3rdparty_location_}) # should be a system dependency
    	          else()
    	              list(APPEND VISP_EXTERN_LIBRARIES ${3rdparty_}) # should be a system dependency
    	          endif()
                endif()
              endif()
            endforeach()
          endif()
        endforeach()
      endforeach()
      
      if(POLICY CMP0045)
        # Fix Error on non-existent target in get_target_property for 3rd party location extraction
        cmake_policy(POP)
      endif()
      
    else()
      # this should be an old OpenCV version that doesn't have the previous behavior
      list(APPEND VISP_EXTERN_LIBRARIES ${OpenCV_LIBS})
    endif()
    list(APPEND VISP_EXTERN_INCLUDE_DIRS ${OpenCV_INCLUDE_DIRS})
    #LIST(APPEND VISP_EXTERN_LIBRARIES ${OpenCV_LIBS})
    #MESSAGE("OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
    #MESSAGE("OpenCV_LIBS = ${OpenCV_LIBS}")
    #MESSAGE("OpenCV_LIB_DIR = ${OpenCV_LIB_DIR}")
    set(VISP_HAVE_OPENCV_VERSION "(${OpenCV_VERSION_MAJOR}*65536 + ${OpenCV_VERSION_MINOR}*256 + ${OpenCV_VERSION_PATCH})") # for ViSP-third-party.txt
    #MESSAGE("VISP_HAVE_OPENCV_VERSION = ${VISP_HAVE_OPENCV_VERSION}")
    #message("VISP_EXTERN_LIBRARIES = ${VISP_EXTERN_LIBRARIES}")

    # additional check to see if opencv_nonfree module introduced in OpenCV 2.4 is available
    #message("OpenCV_VERSION = ${OpenCV_VERSION}")
    #message("OpenCV_NONFREE_FOUND: ${OPENCV_NONFREE_FOUND}")
    if(OpenCV_VERSION)
      if(OpenCV_VERSION VERSION_LESS "2.4.0")
        message(STATUS "OpenCV nonfree module found")
        set(VISP_HAVE_OPENCV_NONFREE TRUE)  # for header vpConfig.h
      else() # Version is greater or equal to 2.4.0
        if(OPENCV_NONFREE_FOUND) # OpenCV < 3.0.0
          message(STATUS "OpenCV xfeatures2d module found")
          set(VISP_HAVE_OPENCV_NONFREE TRUE)  # for header vpConfig.h
        elseif(OPENCV_XFEATURES2D_FOUND) # OpenCV >= 3.0.0
          set(VISP_HAVE_OPENCV_XFEATURES2D TRUE)  # for header vpConfig.h
        else()
          message(STATUS "OpenCV nonfree or xfeature2d module not found")
        endif()
      endif()
    else()
      message("OpenCV_VERSION is false")
      message(STATUS "OpenCV nonfree not found")
    endif()
  else(OpenCV_FOUND)
    message(STATUS "OpenCV not found")
    set(VISP_HAVE_OPENCV_FOUND "no")    # for ViSP-third-party.txt
    set(VISP_HAVE_OPENCV_VERSION "(0)") # for ViSP-third-party.txt
  endif(OpenCV_FOUND)
else(USE_OPENCV)
  set(OpenCV_VERSION_MAJOR 0)
  set(OpenCV_VERSION_MINOR 0)
  SET(OpenCV_VERSION_PATCH 0)
  set(VISP_HAVE_OPENCV_VERSION "(0)") # for ViSP-third-party.txt
endif(USE_OPENCV)

# gnu scientific library
IF(USE_GSL)
  #FIND_PACKAGE(GSL)
  IF(GSL_FOUND)
    MESSAGE(STATUS "GSL found")
    SET(VISP_HAVE_GSL  TRUE)  # for header vpConfig.h
    SET(VISP_HAVE_GSL_FOUND "yes")  # for ViSP-third-party.txt
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${GSL_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${GSL_LIBRARIES})
    #MESSAGE("GSL_INCLUDE_DIRS = ${GSL_INCLUDE_DIRS}")
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
SET(VISP_HAVE_DC1394_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_V4L2_FOUND "no")  # for ViSP-third-party.txt
SET(VISP_HAVE_DIRECTSHOW_FOUND "no")  # for ViSP-third-party.txt

# library for firewire cameras
IF(USE_DC1394)
  #FIND_PACKAGE(DC1394)
  if(DC1394_FOUND)
    set(VISP_HAVE_DC1394 TRUE) # for header vpConfig.h

    MESSAGE(STATUS "libdc1394-2.x found")
    SET(VISP_HAVE_DC1394 TRUE) # for header vpConfig.h
    SET(VISP_HAVE_DC1394_FOUND "yes")  # for ViSP-third-party.txt
    IF(DC1394_CAMERA_ENUMERATE_FOUND)
      SET(VISP_HAVE_DC1394_CAMERA_ENUMERATE TRUE) # for header vpConfig.h
    ENDIF(DC1394_CAMERA_ENUMERATE_FOUND)
    IF(DC1394_FIND_CAMERAS_FOUND)
      SET(VISP_HAVE_DC1394_FIND_CAMERAS TRUE) # for header vpConfig.h
    ENDIF(DC1394_FIND_CAMERAS_FOUND)
    LIST(APPEND VISP_EXTERN_INCLUDE_DIRS ${DC1394_INCLUDE_DIRS})
    LIST(APPEND VISP_EXTERN_LIBRARIES ${DC1394_LIBRARY})

    #MESSAGE("DBG VISP_HAVE_DC1394= ${VISP_HAVE_DC1394}")
    #MESSAGE("DBG DC1394_INCLUDE_DIRS=${DC1394_INCLUDE_DIRS}")
    
    IF(APPLE)
      #SET(VISP_EXTERN_LIBRARIES ${VISP_EXTERN_LIBRARIES} IOkit "-framework Carbon")
      LIST(APPEND VISP_EXTERN_LIBRARIES "-framework IOkit -framework Carbon")
    ENDIF(APPLE)
    #MESSAGE("DC1394_LIBRARIES = ${DC1394_LIBRARIES}")
  ELSE(DC1394_FOUND)
    MESSAGE(STATUS "libdc1394-2.x not found")
    SET(VISP_HAVE_DC1394_FOUND "no")  # for ViSP-third-party.txt
    #MESSAGE("Firewire dc1394 requested but not found. Turn off USE_DC1394!")
    # SET(USE_DC1394 OFF)
  ENDIF(DC1394_FOUND)
ENDIF(USE_DC1394)

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

if(USE_ZBAR)
  set(VISP_HAVE_ZBAR TRUE) # for header vpConfig.h
  set(VISP_HAVE_ZBAR_FOUND "yes")  # for ViSP-third-party.txt
  list(APPEND VISP_EXTERN_INCLUDE_DIRS ${ZBAR_INCLUDE_DIRS})
  list(APPEND VISP_EXTERN_LIBRARIES ${ZBAR_LIBRARIES})
else()
  set(VISP_HAVE_ZBAR_FOUND "no")  # for ViSP-third-party.txt
endif()

if(USE_DMTX)
  set(VISP_HAVE_DMTX TRUE) # for header vpConfig.h
  set(VISP_HAVE_DMTX_FOUND "yes")  # for ViSP-third-party.txt
  list(APPEND VISP_EXTERN_INCLUDE_DIRS ${DMTX_INCLUDE_DIRS})
  list(APPEND VISP_EXTERN_LIBRARIES ${DMTX_LIBRARIES})
else()
  set(VISP_HAVE_DMTX_FOUND "no")  # for ViSP-third-party.txt
endif()

# Find IsNaN
find_package(IsNaN)
# Find isnan macro (C-style)
if(HAVE_FUNC_ISNAN)
  SET(VISP_HAVE_FUNC_ISNAN TRUE)
endif()

# Find std::isnan function (cmath)
if(HAVE_FUNC_STD_ISNAN)
  SET(VISP_HAVE_FUNC_STD_ISNAN TRUE)
endif()

# Find _isnan function for MSVC
if(HAVE_FUNC__ISNAN)
  SET(VISP_HAVE_FUNC__ISNAN TRUE)
endif()

#----------------------------------------------------------------------
# Add definitions
#----------------------------------------------------------------------
# With Visual Studio 2005, Microsoft deprecates the standard C library, for
# example fopen() and sprintf(), to non-portable functions fopen_s() and
# sprintf_s(). These functions are considered by Microsoft more secure. This is
# a worthwhile exercise ! The use of these deprecated functions causes a lot of
# warnings. To suppress it, we add the _CRT_SECURE_NO_DEPRECATE preprocessor
# definition
if(WIN32 AND MSVC)
  add_definitions("-D_CRT_SECURE_NO_DEPRECATE")
endif()

#MESSAGE("Definitions: ${VISP_DEFS}")
IF(VISP_DEFS)
  LIST(REMOVE_DUPLICATES VISP_DEFS)
ENDIF(VISP_DEFS)
#MESSAGE("Definitions without duplicates: ${VISP_DEFS}")
add_definitions(${VISP_DEFS})

#----------------------------------------------------------------------
# Add intern and third party include dirs
#----------------------------------------------------------------------
#MESSAGE("VISP_EXTERN_INCLUDE_DIRS: ${VISP_EXTERN_INCLUDE_DIRS}")
IF(VISP_EXTERN_INCLUDE_DIRS)
  LIST(REMOVE_DUPLICATES VISP_EXTERN_INCLUDE_DIRS)
ENDIF(VISP_EXTERN_INCLUDE_DIRS)
#MESSAGE("VISP_INTERN_INCLUDE_DIR: ${VISP_INTERN_INCLUDE_DIR}")
#MESSAGE("VISP_EXTERN_INCLUDE_DIRS: ${VISP_EXTERN_INCLUDE_DIRS}")
include_directories(BEFORE ${VISP_INTERN_INCLUDE_DIR})
include_directories(${VISP_EXTERN_INCLUDE_DIRS})

#----------------------------------------------------------------------
# Use statically or dynamically linked CRT?
# Default: dynamic
#----------------------------------------------------------------------
if(MSVC)
   include(${VISP_CMAKE_MODULE_PATH}/VISPCRTLinkage.cmake)
endif(MSVC)

#----------------------------------------------------------------------
# Platform specific
#----------------------------------------------------------------------
include(${VISP_CMAKE_MODULE_PATH}/VISPDetectPlatform.cmake)

# Set the path where to install the lib
if(WIN32)
  if(DEFINED VISP_RUNTIME AND DEFINED VISP_ARCH)
    set(VISP_INSTALL_BINARIES_PREFIX "${VISP_ARCH}/${VISP_RUNTIME}/")
  else()
    message(STATUS "Can't detect runtime and/or arch")
    set(VISP_INSTALL_BINARIES_PREFIX "")
  endif()
else()
  set(VISP_INSTALL_BINARIES_PREFIX "")
endif()

# where to install the library
if(WIN32)
  if(VISP_STATIC)
    set(VISP_LIB_INSTALL_PATH   "${VISP_INSTALL_BINARIES_PREFIX}static${CMAKE_INSTALL_LIBDIR}")
  else()
    set(VISP_LIB_INSTALL_PATH   "${VISP_INSTALL_BINARIES_PREFIX}${CMAKE_INSTALL_LIBDIR}")
  endif()
  set(VISP_BIN_INSTALL_PATH   "${VISP_INSTALL_BINARIES_PREFIX}${CMAKE_INSTALL_BINDIR}")
else()
  set(VISP_LIB_INSTALL_PATH     ${CMAKE_INSTALL_LIBDIR})
  set(VISP_BIN_INSTALL_PATH     ${CMAKE_INSTALL_BINDIR})
endif()

#----------------------------------------------------------------------
# Cleanify the libraries list
#----------------------------------------------------------------------
if(APPLE)
  list(REMOVE_DUPLICATES VISP_EXTERN_LIBRARIES)
endif()

#----------------------------------------------------------------------
# customize clean target
#----------------------------------------------------------------------
include(${VISP_CMAKE_MODULE_PATH}/CMakeHeaderFileList.cmake)

set(FILE_TO_CLEAN "core*;*~;gmon.out;DartTestfile.txt")
foreach(header ${HEADER_ALL})
  get_filename_component(headerName ${header} NAME)
  list(APPEND FILE_TO_CLEAN ${CMAKE_SOURCE_DIR}/include/visp/${headerName})
endforeach()

SET_DIRECTORY_PROPERTIES(PROPERTIES
  ADDITIONAL_MAKE_CLEAN_FILES "${FILE_TO_CLEAN}"
)

#----------------------------------------------------------------------
# Propagation in src to build the library
#----------------------------------------------------------------------
add_subdirectory(src)

#----------------------------------------------------------------------
# Configure the files that depend on the build <binary dir> or
# installation <install dir> usage. This is the case of:
# vpConfig.h
#----------------------------------------------------------------------

# case 1: when ViSP is build with make; files are used in <binary dir>
#--------------
set(VISP_ROOT_DIR_DATA_CONFIGCMAKE "${VISP_BINARY_DIR}")

SET(VISP_SCENES_DIR ${VISP_ROOT_DIR_DATA_CONFIGCMAKE}/data/wireframe-simulator)
SET(VISP_ROBOT_ARMS_DIR ${VISP_ROOT_DIR_DATA_CONFIGCMAKE}/data/robot-simulator)
    
# case 2: when ViSP is build with make install; files are used in <install dir>
#--------------
IF(UNIX)
  set(VISP_ROOT_DIR_DATA_CONFIGCMAKE "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/visp-${VISP_VERSION}")
ELSE()
  set(VISP_ROOT_DIR_DATA_CONFIGCMAKE "${CMAKE_INSTALL_PREFIX}")
ENDIF()

list(APPEND VISP_SCENES_DIR ${VISP_ROOT_DIR_DATA_CONFIGCMAKE}/data/wireframe-simulator)
list(APPEND VISP_ROBOT_ARMS_DIR ${VISP_ROOT_DIR_DATA_CONFIGCMAKE}/data/robot-simulator)
# Only if resources.cfg created by hand, we change the path to resources.cfg in install/vpConfig.h
if(VISP_INSTALL_DIR_OGRE_RESOURCES)
  list(APPEND VISP_HAVE_OGRE_RESOURCES_PATH "${VISP_ROOT_DIR_DATA_CONFIGCMAKE}/data/ogre-simulator")
endif()
# append to VISP_HAVE_OGRE_PLUGINS_PATH the path of the installed plugins.cfg file
# to be able to use ViSP from build tree or install tree
if(VISP_INSTALL_DIR_OGRE_RESOURCES)
  list(APPEND VISP_HAVE_OGRE_PLUGINS_PATH "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/visp/data/ogre-simulator")
endif()

# Generate the package dependent file include/visp/vpConfig.h
CONFIGURE_FILE(${VISP_SOURCE_DIR}/include/vpConfig.h.cmake
  ${VISP_INCLUDE_DIR}/vpConfig.h
)

#----------------------------------------------------------------------
# Generate the ViSP-third-party.txt information file
#----------------------------------------------------------------------
CONFIGURE_FILE(${VISP_CMAKE_MODULE_PATH}/ViSP-third-party.txt.cmake
  ${VISP_BINARY_DIR}/ViSP-third-party.txt
)

#----------------------------------------------------------------------
# Create and install visp-config.1.gz man page
#----------------------------------------------------------------------
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

#----------------------------------------------------------------------
# Export the library
#----------------------------------------------------------------------
# we need to adapt VISP_SCENES_DIR and VISP_ROBOT_ARMS_DIR
# here set to /usr/share/visp/data
if(UNIX)
  foreach(SCENE ${WIREFRAME_SCENES})
    install(FILES
      "${VISP_SOURCE_DIR}/src/simulator/wireframe-simulator/scene/${SCENE}"
      DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/visp-${VISP_VERSION}/data/wireframe-simulator
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      COMPONENT libraries
    )
  endforeach()
  foreach(SCENE ${ROBOT_ARMS_SCENES})
    install(FILES
      "${VISP_SOURCE_DIR}/src/robot/simulator-robot/arms/${SCENE}"
      DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/visp-${VISP_VERSION}/data/robot-simulator
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      COMPONENT libraries
    )
  endforeach()
else()
  foreach(SCENE ${WIREFRAME_SCENES})
    install(FILES
      "${VISP_SOURCE_DIR}/src/simulator/wireframe-simulator/scene/${SCENE}"
      DESTINATION data/wireframe-simulator
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      COMPONENT libraries
    )
  endforeach()
  foreach(SCENE ${ROBOT_ARMS_SCENES})
    install(FILES
      "${VISP_SOURCE_DIR}/src/robot/simulator-robot/arms/${SCENE}"
      DESTINATION data/robot-simulator
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      COMPONENT libraries
    )
  endforeach()
endif()

#----------------------------------------------------------------------
# For CPack packaging tool
#----------------------------------------------------------------------

option(BUILD_PACKAGE "Configure ViSP packaging" OFF)
if (BUILD_PACKAGE)
  if(UNIX AND NOT APPLE AND NOT WIN32) # =linux
    option(BUILD_PACKAGE_DEBIAN "Build debian package" ON)
    option(BUILD_PACKAGE_RPM "Build rpm package" ON)
  endif()

  include (${VISP_CMAKE_MODULE_PATH}/CPackConfig.cmake)
endif(BUILD_PACKAGE)

#----------------------------------------------------------------------
# Configure the file describing how to use ViSP. VISPConfig.cmake
# is the main file configuring a CMake package.
# . Exports build settings and dependencies for projects using ViSP as a
#   third party project.
# . Create and install files for simple use of find_package(VISP)
#   by other cmakified "user" projects and libraries depending on ViSP.
#   (see "Mastering CMake", pp.72)
# . To use ViSP in a third party project based on CMake:
#   find_package(VISP REQUIRED)
#   include_directories(${VISP_INCLUDE_DIRS})
#   target_link_libraries(<target> ${VISP_LIBRARIES})
#----------------------------------------------------------------------
include(${VISP_CMAKE_MODULE_PATH}/VISPGenerateConfig.cmake)

#----------------------------------------------------------------------
# Generate the package dependent visp-config shell script for projects which
# are not using CMake:
# Usage:
#    visp-config --cflags ...
#----------------------------------------------------------------------
include(${VISP_CMAKE_MODULE_PATH}/VISPGeneratePkgConfigScript.cmake)

#----------------------------------------------------------------------
# Propagation in sub dirs to build demo, example, test, tutorial
#----------------------------------------------------------------------
if(BUILD_TESTS)
  #----------------------------------------------------------------------
  # For Dart server and tests
  # We use CDash set through CTestConfig.cmake file
  # Dashboards are sent to http://cdash.irisa.fr/CDash/index.php?project=ViSP
  #----------------------------------------------------------------------
  enable_testing()
  include(CTest)
  mark_as_advanced(DART_ROOT)
  mark_as_advanced(BUILD_TESTING)
  #
  # Test coverage specific code
  #
  if(CMAKE_COMPILER_IS_GNUCXX AND NOT BUILD_SHARED_LIBS AND CMAKE_BUILD_TYPE MATCHES "Debug")
    option(BUILD_TEST_COVERAGE "Enable test coverage." OFF)
  endif()

  if(BUILD_TEST_COVERAGE)
    # Add build options for test coverage. Currently coverage is only supported
    # on gcc compiler
    # Because using -fprofile-arcs with shared lib can cause problems like:
    # hidden symbol `__bb_init_func', we add this option only for static
    # library build
    message(STATUS "Add -ftest-coverage -fprofile-arcs compiler options")
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -ftest-coverage -fprofile-arcs")
  endif()

  add_subdirectory(test)
endif()

if(BUILD_DEMOS)
  add_subdirectory(demo)
endif()
if(BUILD_EXAMPLES)
  add_subdirectory(example)
endif()
if(BUILD_TUTORIALS)
  set(VISP_DIR ${PROJECT_BINARY_DIR})
  mark_as_advanced(VISP_DIR)
  mark_as_advanced(VISP_INCLUDE_DIRS)
  add_subdirectory(tutorial)
endif()
