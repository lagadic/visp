#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2017 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See http://visp.inria.fr for more information.
#
# This software was developed at:
# Inria Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
#
# If you have questions regarding the use of this file, please contact
# Inria at visp@inria.fr
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

# Detect crosscompiling; need to be before project(VISP) to work
if(NOT CMAKE_TOOLCHAIN_FILE)
  # Modify default install prefix
  if(WIN32)
    set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")
  endif()
else()
  # Crosscompiling
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE PATH "Installation Directory")
endif()

if(CMAKE_SYSTEM_NAME MATCHES WindowsPhone OR CMAKE_SYSTEM_NAME MATCHES WindowsStore)
  set(WINRT TRUE)
endif(CMAKE_SYSTEM_NAME MATCHES WindowsPhone OR CMAKE_SYSTEM_NAME MATCHES WindowsStore)

if(WINRT)
  add_definitions(-DWINRT)

  if(CMAKE_SYSTEM_NAME MATCHES WindowsPhone)
    set(WINRT_PHONE TRUE)
    add_definitions(-DWINRT_PHONE)
  elseif(CMAKE_SYSTEM_NAME MATCHES WindowsStore)
    set(WINRT_STORE TRUE)
    add_definitions(-DWINRT_STORE)
  endif()

  if(CMAKE_SYSTEM_VERSION MATCHES 10 OR CMAKE_SYSTEM_VERSION MATCHES 10.0)
    set(WINRT_10 TRUE)
    add_definitions(-DWINRT_10)
  elseif(CMAKE_SYSTEM_VERSION MATCHES 8.1)
    set(WINRT_8_1 TRUE)
    add_definitions(-DWINRT_8_1)
  elseif(CMAKE_SYSTEM_VERSION MATCHES 8.0)
    set(WINRT_8_0 TRUE)
    add_definitions(-DWINRT_8_0)
  endif()
endif()

# By default set release configuration
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE String "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

project(VISP C CXX)

# Detect if the toolchain is for Aldebaran naoqi
if(CMAKE_TOOLCHAIN_FILE AND I_AM_A_ROBOT)
  include(platforms/naoqi/cmake/extra.cmake)
endif()

cmake_minimum_required(VERSION 2.8.3)

if(POLICY CMP0022)
  cmake_policy(SET CMP0022 NEW) # Due to sensor ATIDAQ_LIBRARIES
endif()

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

include(cmake/VISPUtils.cmake)

vp_clear_vars(VISPModules_TARGETS)

#-----------------------------------------------------------------------------
# VISP version number.  An even minor number corresponds to releases.
set(VISP_VERSION_MAJOR "3")
set(VISP_VERSION_MINOR "0")
set(VISP_VERSION_PATCH "2")
set(VISP_VERSION "${VISP_VERSION_MAJOR}.${VISP_VERSION_MINOR}.${VISP_VERSION_PATCH}")
# Package revision number
set(VISP_REVISION "1")

find_file(GNU_INSTALL_DIRS_FROM_CMAKE NAMES GNUInstallDirs.cmake PATHS ${CMAKE_ROOT}/Modules)
mark_as_advanced(GNU_INSTALL_DIRS_FROM_CMAKE)
if(GNU_INSTALL_DIRS_FROM_CMAKE)
  include(${CMAKE_ROOT}/Modules/GNUInstallDirs.cmake)
else()
  include(cmake/GNUInstallDirs.cmake)
endif()

#----------------------------------------------------------------------
# Platform specific
#----------------------------------------------------------------------
include(cmake/VISPDetectPlatform.cmake)

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

# where to install the library and headers
if(WIN32)
  set(VISP_INC_INSTALL_PATH     ${CMAKE_INSTALL_INCLUDEDIR})
  if(VISP_STATIC)
    set(VISP_LIB_INSTALL_PATH  "${VISP_INSTALL_BINARIES_PREFIX}static${CMAKE_INSTALL_LIBDIR}")
  else()
    set(VISP_LIB_INSTALL_PATH  "${VISP_INSTALL_BINARIES_PREFIX}${CMAKE_INSTALL_LIBDIR}")
  endif()
  set(VISP_BIN_INSTALL_PATH    "${VISP_INSTALL_BINARIES_PREFIX}${CMAKE_INSTALL_BINDIR}")
  set(VISP_3P_LIB_INSTALL_PATH "${VISP_INSTALL_BINARIES_PREFIX}static${CMAKE_INSTALL_LIBDIR}")
else()
  # The location where includes and libraries will be installed

  # The 2 following commented lines should be used to enable multi-arch support.
  # Since catkin doesn't support multi-arch we keep these 2 lines commented for the moment.
  # set(VISP_INC_INSTALL_PATH     ${CMAKE_INSTALL_INCLUDEDIR}/${CMAKE_LIBRARY_ARCHITECTURE})
  # set(VISP_LIB_INSTALL_PATH     ${CMAKE_INSTALL_LIBDIR}/${CMAKE_LIBRARY_ARCHITECTURE})
  set(VISP_INC_INSTALL_PATH     ${CMAKE_INSTALL_INCLUDEDIR})
  set(VISP_LIB_INSTALL_PATH     ${CMAKE_INSTALL_LIBDIR})
  set(VISP_BIN_INSTALL_PATH     ${CMAKE_INSTALL_BINDIR})
  set(VISP_3P_LIB_INSTALL_PATH  share/ViSP/3rdparty/${VISP_LIB_INSTALL_PATH})
endif()
set(VISP_3P_LIBRARY_OUTPUT_PATH "${VISP_BINARY_DIR}/3rdparty/lib")

# the include directory we depend on for the build
set(VISP_INCLUDE_DIR ${VISP_BINARY_DIR}/${VISP_INC_INSTALL_PATH})
set(VISP_DOC_DIR "${VISP_BINARY_DIR}/doc")

# The location where includes and libraries will be build
set(LIBRARY_OUTPUT_PATH ${VISP_BINARY_DIR}/${VISP_LIB_INSTALL_PATH})
set(BINARY_OUTPUT_PATH ${VISP_BINARY_DIR}/${VISP_BIN_INSTALL_PATH})

if(WIN32)
  # Postfix of .lib and .dll
  set(VISP_DEBUG_POSTFIX "d")
  set(VISP_DLLVERSION "${VISP_VERSION_MAJOR}${VISP_VERSION_MINOR}${VISP_VERSION_PATCH}")
else()
  set(VISP_DEBUG_POSTFIX "")
  set(VISP_DLLVERSION "")
endif()

include_directories(${VISP_INCLUDE_DIR})

#----------------------------------------------------------------------
# SSE optimization
#----------------------------------------------------------------------
VP_OPTION(ENABLE_SSE2  "" "" "Enable SSE2 instructions"  "" ON IF ((MSVC OR CMAKE_COMPILER_IS_GNUCXX) AND (X86 OR X86_64)) )
VP_OPTION(ENABLE_SSE3  "" "" "Enable SSE3 instructions"  "" ON IF ((MSVC OR CMAKE_COMPILER_IS_GNUCXX) AND (X86 OR X86_64)) )
VP_OPTION(ENABLE_SSSE3 "" "" "Enable SSSE3 instructions" "" ON IF ((MSVC OR CMAKE_COMPILER_IS_GNUCXX) AND (X86_64)) ) # X86 disabled since it produces an issue on Debian i386

# ----------------------------------------------------------------------------
# Handle always full RPATH
# http://www.cmake.org/Wiki/CMake_RPATH_handling
# ----------------------------------------------------------------------------

# avoid CMP0042 warning
if(APPLE)
  set(CMAKE_MACOSX_RPATH ON)
endif()

# use, i.e. don't skip the full RPATH for the build tree
set(CMAKE_SKIP_BUILD_RPATH  FALSE)

# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)

set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${VISP_LIB_INSTALL_PATH}")

# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# the RPATH to be used when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${VISP_LIB_INSTALL_PATH}" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${VISP_LIB_INSTALL_PATH}")
endif("${isSystemDir}" STREQUAL "-1")

# ----------------------------------------------------------------------------
#  Path for additional contrib modules
# ----------------------------------------------------------------------------
set(VISP_CONTRIB_MODULES_PATH "" CACHE PATH "Where to look for additional contrib ViSP modules")

# Get the OS
set(OS ${CMAKE_SYSTEM_NAME})

set(OGRE_HOME $ENV{OGRE_HOME})
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

#--------------------------------------------------------------------
# Option management
#--------------------------------------------------------------------

# Choose static or shared libraries.
VP_OPTION(BUILD_SHARED_LIBS  "" "" "Build ViSP shared libraries (.dll/.so) instead of static ones (.lib/.a)" "" NOT APPLE_FRAMEWORK)
# Build examples as an option.
VP_OPTION(BUILD_EXAMPLES  "" "" "Build ViSP examples" "" ON)
# Build examples as an option.
VP_OPTION(BUILD_TESTS  "" "" "Build ViSP tests" "" ON)
VP_OPTION(BUILD_COVERAGE "" "" "Enables test coverage" "" OFF IF (BUILD_TESTS AND CMAKE_COMPILER_IS_GNUCXX AND CMAKE_BUILD_TYPE MATCHES "Debug"))

# Build demos as an option.
VP_OPTION(BUILD_DEMOS  "" "" "Build ViSP demos" "" ON)
# Build demos as an option.
VP_OPTION(BUILD_TUTORIALS  "" "" "Build ViSP tutorials" "" ON)
# Build deprecated functions as an option.
VP_OPTION(BUILD_DEPRECATED_FUNCTIONS  "" "" "Build deprecated functionalities" "" ON)
# Debug and trace cflags
VP_OPTION(ACTIVATE_DEBUG_TRACE  "" "" "Enable debug and trace printings" "" ON)

VP_OPTION(BUILD_WITH_STATIC_CRT  "" "" "Enables use of staticaly linked CRT for staticaly linked ViSP" "" ON IF MSVC)

# ----------------------------------------------------------------------------
# Build options
# ----------------------------------------------------------------------------
VP_OPTION(ENABLE_SOLUTION_FOLDERS "" "" "Solution folder in Visual Studio or in other IDEs" "" (MSVC_IDE OR CMAKE_GENERATOR MATCHES Xcode))
# Note that it is better to set ENABLE_MOMENTS_COMBINE_MATRICES to OFF
VP_OPTION(ENABLE_MOMENTS_COMBINE_MATRICES  "" "" "Use linear combination of matrices instead of linear combination of moments to compute interaction matrices." "ENABLE_MOMENTS_COMBINE_MATRICES" OFF)
VP_OPTION(ENABLE_TEST_WITHOUT_DISPLAY      "" "" "Don't use display feature when testing" "" ON)
VP_OPTION(ENABLE_FULL_DOC      "" "" "Build doc with internal classes that are by default not part of the doc" "" OFF)

if(ENABLE_SOLUTION_FOLDERS)
  set_property(GLOBAL PROPERTY USE_FOLDERS ON)
  set_property(GLOBAL PROPERTY PREDEFINED_TARGETS_FOLDER "CMakeTargets")
endif()

if(ENABLE_TEST_WITHOUT_DISPLAY)
  set(OPTION_TO_DESACTIVE_DISPLAY "-d")
endif()

# Check for Inria's robot drivers
VP_CHECK_PACKAGE(RAW1394)
VP_CHECK_PACKAGE(RT)
VP_CHECK_PACKAGE(CALINUX)
VP_CHECK_PACKAGE(IRISA)
# check for linux/parport.h
VP_CHECK_PACKAGE(PARPORT)
# OpenGL
VP_CHECK_PACKAGE(OpenGL)
# for pioneer
VP_CHECK_PACKAGE(RT)
VP_CHECK_PACKAGE(DL)
# for Afma6 calibration data
VP_CHECK_PACKAGE(Afma6_data)
if(AFMA6_DATA_FOUND)
  set(VISP_AFMA6_DATA_PATH ${AFMA6_DATA_PATH})
else()
  set(VISP_AFMA6_DATA_PATH "")
endif()
# for Viper850 calibration data
VP_CHECK_PACKAGE(Viper850_data)
if(VIPER850_DATA_FOUND)
  set(VISP_VIPER850_DATA_PATH ${VIPER850_DATA_PATH})
else()
  set(VISP_VIPER850_DATA_PATH "")
endif()
# for Viper650 calibration data
VP_CHECK_PACKAGE(Viper650_data)
if(VIPER650_DATA_FOUND)
  set(VISP_VIPER650_DATA_PATH ${VIPER650_DATA_PATH})
else()
  set(VISP_VIPER650_DATA_PATH "")
endif()

find_host_program(XRANDR xrandr)

VP_OPTION(USE_AFMA4    "" "" "Include Afma4 robot support"      "" ON IF (RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND AND NOT WINRT))
VP_OPTION(USE_AFMA6    "" "" "Include Afma6 robot support"      "" ON IF (RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND AND NOT WINRT))
VP_OPTION(USE_VIPER650 "" "" "Include Viper s650 robot support" "" ON IF (RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND AND NOT WINRT))
VP_OPTION(USE_VIPER850 "" "" "Include Viper s850 robot support" "" ON IF (RAW1394_FOUND AND RT_FOUND AND CALINUX_FOUND AND IRISA_FOUND AND NOT WINRT))
VP_OPTION(USE_VIRTUOSE    Virtuose    "" "Include Virtuose SDK support for Haption devices" "" ON IF NOT WINRT)
VP_OPTION(USE_DC1394      DC1394      "" "Include dc1394 support"          "" ON IF UNIX AND NOT WINRT)
VP_OPTION(USE_V4L2        V4L2        "" "Include v4l2 support"            "" ON IF UNIX AND NOT WINRT)
VP_OPTION(USE_FLYCAPTURE  FlyCapture  "" "Include FlyCapture SDK support for PointGrey cameras" "" ON IF NOT WINRT)
VP_OPTION(USE_COMEDI      Comedi      "" "Include comedi (linux control and measurement device interface) support" "" ON IF NOT WINRT)
VP_OPTION(USE_BICLOPS     BICLOPS     "" "Include biclops support"         "" ON IF NOT WINRT)
VP_OPTION(USE_PTU46       PTU46       "" "Include ptu-46 support"          "" ON IF UNIX AND NOT WINRT)
VP_OPTION(USE_CMU1394     CMU1394     "" "Include cmu1494 support"         "" ON IF WIN32 AND NOT WINRT)
VP_OPTION(USE_GDI         GDI         "" "Include gdi support"             "" ON IF WIN32 AND NOT WINRT)
VP_OPTION(USE_DIRECT3D    DIRECT3D    "" "Include d3d support"             "" ON IF WIN32 AND NOT WINRT)
VP_OPTION(USE_DIRECTSHOW  DIRECTSHOW  "" "Include dshow support"           "" ON IF WIN32 AND NOT WINRT)
VP_OPTION(USE_OPENMP      OpenMP      "" "Include openmp support"          "" ON)
VP_OPTION(USE_EIGEN3      Eigen3      QUIET "Include eigen3 support"       "" ON IF NOT WINRT)
# Since the FindLAPACK.cmake provided with CMake is for Fortran language,
# in CMakeModules we have added FindLAPACK_C.cmake for C language
VP_OPTION(USE_LAPACK      LAPACK_C    "" "Include lapack support"          "" ON IF NOT WINRT)
VP_OPTION(USE_GSL         GSL         "" "Include gsl support"             "" ON IF NOT WINRT)
VP_OPTION(USE_COIN3D      "Coin3D;MyCoin3D" "" "Include coin3d support"    "" ON IF OPENGL_FOUND AND NOT WINRT)
VP_OPTION(USE_YARP        YARP        QUIET "Include yarp support"         "YARP_DIR" ON IF NOT WINRT)
VP_OPTION(USE_OGRE        OGRE        QUIET "Include Ogre support"         "OGRE_DIR" ON IF NOT WINRT)
VP_OPTION(USE_OIS         OIS         QUIET "Include Ogre/ois support"     "OIS_DIR"  ON IF USE_OGRE AND NOT WINRT)
VP_OPTION(USE_LIBFREENECT LIBFREENECT ""    "Include libfreenect support"  "" ON IF NOT WINRT)
VP_OPTION(USE_LIBUSB_1    LIBUSB_1    ""    "Include libusb-1 support"     "" ON IF NOT WINRT)
VP_OPTION(USE_REALSENSE   RealSense   ""    "Include RealSense SDK support" "" ON IF NOT WINRT)
if(USE_REALSENSE)
  VP_OPTION(USE_CPP11       CPP11     ""    "Include c++11 support"         "" ON)
  # if option is OFF, force to ON
  if(NOT USE_CPP11 AND CPP11_FOUND)
    unset(USE_REALSENSE)
    set(USE_REALSENSE OFF CACHE BOOL "Include RealSense SDK support" FORCE)
    message(WARNING "Realsense 3rd party was detected but turned OFF since USE_CPP11 is OFF. Realsense 3rd party needs cxx11 compiler flags. Thus to enable Realsense usage you have to turn USE_REALSENSE=ON and USE_CPP11=ON.")
  endif()
else()
  VP_OPTION(USE_CPP11       CPP11     ""    "Include c++11 support"        "" OFF)
endif()
VP_OPTION(USE_SOWIN       SOWIN       ""    "Include Coin/SoWin support"   "" OFF IF (WIN32 AND USE_COIN3D) AND NOT WINRT)
VP_OPTION(USE_SOQT        SOQT        ""    "Include Coin/SoQt support"    "" OFF IF USE_COIN3D AND NOT WINRT)
VP_OPTION(USE_QT          Qt          ""    "Include Coin/SoQt/Qt support" "" ON  IF USE_SOQT AND NOT WINRT)
VP_OPTION(USE_SOXT        SOXT        ""    "Include Coin/SoXt support"    "" OFF IF USE_COIN3D AND NOT WINRT)
VP_OPTION(USE_PTHREAD     PTHREAD     ""    "Include pthread support"      "" ON)
VP_OPTION(USE_XML2        XML2        ""    "Include xml support"          "" ON IF NOT WINRT)
if(CMAKE_TOOLCHAIN_FILE AND I_AM_A_ROBOT)
  VP_OPTION(USE_OPENCV      "MyOpenCV" QUIET "Include OpenCV support" "OpenCV_DIR;OpenCV_FOUND;OPENCV_FOUND" ON)
else()
  VP_OPTION(USE_OPENCV      "OpenCV;MyOpenCV" QUIET "Include OpenCV support" "OpenCV_DIR;OpenCV_FOUND;OPENCV_FOUND" ON)
endif()
VP_OPTION(USE_ZLIB        "ZLIB;MyZLIB" ""  "Include zlib support"         "" ON IF NOT WINRT)
VP_OPTION(USE_X11         X11         ""    "Include X11 support"          "" ON IF NOT WINRT)
# The native FindGTK2.cmake doesn't consider libgobject-2.0 that is
# requested by ViSP. That's why we use our FindMyGTK2.cmake
VP_OPTION(USE_GTK2        MyGTK2       ""    "Include gtk2 support"         "" OFF IF NOT WINRT)
VP_OPTION(USE_JPEG       "JPEG;MyJPEG" ""    "Include jpeg support"         "" ON)
VP_OPTION(USE_PNG        "PNG;MyPNG"   ""    "Include png support"          "" ON)
VP_OPTION(USE_FFMPEG      FFMPEG       ""    "Include ffmpeg support"       "" OFF)
# To control Pioneer mobile robots, under UNIX we need Aria, pthread, rt and dl 3rd party libraries
VP_OPTION(USE_ARIA        ARIA         ""    "Include aria support"         "" ON IF NOT WINRT)
#VP_OPTION(USE_RT          RT           ""    "Include rt support"           "" ON)
#VP_OPTION(USE_DL          DL           ""    "Include dl support"           "" ON)
# bar codes
VP_OPTION(USE_ZBAR        ZBAR         ""    "Include zbar support"         "" ON IF NOT WINRT)
VP_OPTION(USE_DMTX        DMTX         ""    "Include dmtx support"         "" ON IF NOT WINRT)
VP_OPTION(USE_PCL         PCL          QUIET "Include Point Cloud Library support" "" ON IF USE_REALSENSE AND NOT WINRT)
VP_OPTION(WITH_ATIDAQ     ""           ""    "Build atidaq-c as built-in library"  "" ON IF USE_COMEDI AND NOT WINRT)
VP_OPTION(WITH_CLIPPER    ""           ""    "Build clipper as built-in library"   "" ON IF USE_OPENCV)
VP_OPTION(WITH_LAPACK     ""           ""    "Build lapack as built-in library"    "" ON IF NOT USE_LAPACK)

# Find IsNaN (should be after USE_CPP11)
VP_CHECK_PACKAGE(IsNaN)
# Find IsInf (should be after USE_CPP11)
VP_CHECK_PACKAGE(IsInf)
# Find Round (should be after USE_CPP11)
VP_CHECK_PACKAGE(Round)
# Find Erfc
VP_CHECK_PACKAGE(Erfc)
# Find Strtof
VP_CHECK_PACKAGE(Strtof)
# Find Log1p
VP_CHECK_PACKAGE(Log1p)

#----------------------------------------------------------------------
# For Dart server and tests
# We use CDash set through CTestConfig.cmake file
# Dashboards are sent to http://cdash.irisa.fr/CDash/index.php?project=ViSP
#----------------------------------------------------------------------
if(BUILD_TESTS)
  enable_testing()
  mark_as_advanced(DART_ROOT)
  mark_as_advanced(BUILD_TESTING)
endif()

#----------------------------------------------------------------------
# Try to find doxygen for documentation generation
# Use "make visp_doc" target to generate the documentation
#----------------------------------------------------------------------
find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(VISP_HAVE_DOXYGEN "yes")        # for header vpConfig.h
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

  # set vars used in doxygen config file
  # - DOXYGEN_STRIP_FROM_INC_PATH corresponding to STRIP_FROM_INC_PATH in the doxy file
  set(DOXYGEN_STRIP_FROM_INC_PATH "")
  foreach(m ${VISP_MODULES_BUILD} ${VISP_MODULES_DISABLED_USER} ${VISP_MODULES_DISABLED_AUTO} ${VISP_MODULES_DISABLED_FORCE})
    if(m MATCHES "^visp_")
      set(DOXYGEN_STRIP_FROM_INC_PATH "${DOXYGEN_STRIP_FROM_INC_PATH} \\ \n\t\t\t \"${VISP_MODULE_${m}_LOCATION}/include\"")
    endif()
  endforeach()
  # - DOXYGEN_IMAGE_PATH corresponding to IMAGE_PATH in the doxy file
  set(DOXYGEN_IMAGE_PATH "\"${VISP_SOURCE_DIR}/doc/image\"")
  if(VISP_CONTRIB_MODULES_PATH)
    foreach(contrib ${VISP_CONTRIB_MODULES_PATH})
      set(image_path_ "${VISP_CONTRIB_MODULES_PATH}/doc/image")
      if(EXISTS ${image_path_})
        set(DOXYGEN_IMAGE_PATH "${DOXYGEN_IMAGE_PATH} \\ \n\t\t\t \"${image_path_}\"")
      endif()
    endforeach()
  endif()
  # - DOXYGEN_SHOULD_SKIP_THIS var
  if (ENABLE_FULL_DOC)
    set(DOXYGEN_SHOULD_SKIP_THIS "")
  else()
    set(DOXYGEN_SHOULD_SKIP_THIS "DOXYGEN_SHOULD_SKIP_THIS")
  endif()

  configure_file(${VISP_SOURCE_DIR}/doc/config-doxygen.in
    ${VISP_DOC_DIR}/config-doxygen
    @ONLY )

  # set vars used in mainpage.doc.in
  # - VISP_MAINPAGE_EXTENSION
  set(VISP_MAINPAGE_EXTENSION "")
  if(VISP_CONTRIB_MODULES_PATH)
    foreach(contrib ${VISP_CONTRIB_MODULES_PATH})
      set(mainpage_ext_file_ "${contrib}/doc/mainpage_extension.doc")
      if(EXISTS ${mainpage_ext_file_})
        file(READ ${mainpage_ext_file_} mainpage_ext_content_)
        set(VISP_MAINPAGE_EXTENSION "${VISP_MAINPAGE_EXTENSION}\n${mainpage_ext_content_}")
      endif()
    endforeach()
  endif()

  configure_file(${VISP_SOURCE_DIR}/doc/mainpage.doc.in
    ${VISP_DOC_DIR}/mainpage.doc
    @ONLY )
else()
  set(VISP_HAVE_DOXYGEN "no")        # for header vpConfig.h
endif()

# ----------------------------------------------------------------------------
# Extra ViSP targets: uninstall, etc.
# ----------------------------------------------------------------------------
include(cmake/VISPExtraTargets.cmake)

# Ogre plugins and resources
include(cmake/OgreTools.cmake)
if(USE_OGRE)
  vp_set_ogre_media()
elseif(CMAKE_TOOLCHAIN_FILE)
  vp_set_ogre_advanced_var()
endif()

# ----------------------------------------------------------------------------
# Check for system libs
# ----------------------------------------------------------------------------
include(CheckLibraryExists)

if(UNIX)
  # try to found -lm requested on some platforms to link with X11
  find_library(M_LIBRARY NAMES m)
  mark_as_advanced(M_LIBRARY)
  if(M_LIBRARY)
    list(APPEND VISP_LINKER_LIBS ${M_LIBRARY})
  endif()
  # try to found -lsocket -lnsl requested for vpNetwork and vpSickLDMRS
  find_library(SOCKET_LIBRARY NAMES socket)
  find_library(NSL_LIBRARY NAMES nsl)
  mark_as_advanced(SOCKET_LIBRARY NSL_LIBRARY)
  if (SOCKET_LIBRARY)
    list(APPEND VISP_LINKER_LIBS ${SOCKET_LIBRARY})
  endif()
  if (NSL_LIBRARY)
    list(APPEND VISP_LINKER_LIBS ${NSL_LIBRARY})
  endif()
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

#----------------------------------------------------------------------
# Use statically or dynamically linked CRT?
# Default: dynamic
#----------------------------------------------------------------------
if(MSVC)
   include(cmake/VISPCRTLinkage.cmake)
endif(MSVC)

#----------------------------------------------------------------------
# Create and install visp-config.1.gz man page
#----------------------------------------------------------------------
if(UNIX)
  find_host_program(GZIP gzip)
  if(GZIP)
    file(MAKE_DIRECTORY ${VISP_BINARY_DIR}/doc/man/man1)
    add_custom_command(
      OUTPUT ${VISP_BINARY_DIR}/doc/man/man1/visp-config.1.gz
      COMMAND ${GZIP} --best -c ${CMAKE_CURRENT_SOURCE_DIR}/doc/man/man1/visp-config.1 > ${VISP_BINARY_DIR}/doc/man/man1/visp-config.1.gz
      DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/doc/man/man1/visp-config.1
    )
    add_custom_target(man ALL
      DEPENDS ${VISP_BINARY_DIR}/doc/man/man1/visp-config.1.gz
    )
    install(FILES
      ${VISP_BINARY_DIR}/doc/man/man1/visp-config.1.gz
      DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/man/man1
      PERMISSIONS OWNER_READ GROUP_READ WORLD_READ OWNER_WRITE
      COMPONENT dev
    )
  endif()
endif()

#----------------------------------------------------------------------
# Add 3rd-party libraries
#----------------------------------------------------------------------
include(cmake/VISP3rdParty.cmake)

#-----------------------------------------------------------------------------
# Add extra compilation flags
#-----------------------------------------------------------------------------
include(cmake/AddExtraCompilationFlags.cmake) # should be called after FindOpenMP

#----------------------------------------------------------------------
# Modules
#----------------------------------------------------------------------
include(cmake/VISPModule.cmake)

# process subdirectories
add_subdirectory(modules)

#-------------------------------------------------------------------------------
# specific things that need to be updated in vpConfig.h
#-------------------------------------------------------------------------------
VP_SET(VISP_HAVE_OPENMP      TRUE IF USE_OPENMP)
VP_SET(VISP_HAVE_OPENCV      TRUE IF (BUILD_MODULE_visp_core AND USE_OPENCV))
VP_SET(VISP_HAVE_X11         TRUE IF (BUILD_MODULE_visp_core AND USE_X11))
VP_SET(VISP_HAVE_GTK         TRUE IF (BUILD_MODULE_visp_core AND USE_GTK2))
VP_SET(VISP_HAVE_GDI         TRUE IF (BUILD_MODULE_visp_core AND USE_GDI))
VP_SET(VISP_HAVE_D3D9        TRUE IF (BUILD_MODULE_visp_core AND USE_DIRECT3D))
VP_SET(VISP_HAVE_JPEG        TRUE IF (BUILD_MODULE_visp_core AND USE_JPEG))
VP_SET(VISP_HAVE_PNG         TRUE IF (BUILD_MODULE_visp_core AND USE_PNG))
VP_SET(VISP_HAVE_YARP        TRUE IF (BUILD_MODULE_visp_core AND USE_YARP))
VP_SET(VISP_HAVE_EIGEN3      TRUE IF (BUILD_MODULE_visp_core AND USE_EIGEN3))
VP_SET(VISP_HAVE_GSL         TRUE IF (BUILD_MODULE_visp_core AND USE_GSL))
VP_SET(VISP_HAVE_LAPACK      TRUE IF (BUILD_MODULE_visp_core AND (USE_LAPACK OR WITH_LAPACK)))
VP_SET(VISP_HAVE_LAPACK_BUILT_IN TRUE IF (BUILD_MODULE_visp_core AND WITH_LAPACK))
VP_SET(VISP_HAVE_PTHREAD     TRUE IF (BUILD_MODULE_visp_core AND USE_PTHREAD))
VP_SET(VISP_HAVE_XML2        TRUE IF (BUILD_MODULE_visp_core AND USE_XML2))
VP_SET(VISP_HAVE_FFMPEG      TRUE IF (BUILD_MODULE_visp_core AND USE_FFMPEG))
VP_SET(VISP_HAVE_PCL         TRUE IF (BUILD_MODULE_visp_core AND USE_PCL))

VP_SET(VISP_HAVE_OGRE        TRUE IF (BUILD_MODULE_visp_ar AND USE_OGRE))
VP_SET(VISP_HAVE_OIS         TRUE IF (BUILD_MODULE_visp_ar AND USE_OIS))
VP_SET(VISP_HAVE_COIN3D      TRUE IF (BUILD_MODULE_visp_ar AND USE_COIN3D))
VP_SET(VISP_HAVE_SOWIN       TRUE IF (BUILD_MODULE_visp_ar AND USE_SOWIN))
VP_SET(VISP_HAVE_SOXT        TRUE IF (BUILD_MODULE_visp_ar AND USE_SOXT))
VP_SET(VISP_HAVE_SOQT        TRUE IF (BUILD_MODULE_visp_ar AND USE_SOQT))
VP_SET(VISP_HAVE_QT          TRUE IF (BUILD_MODULE_visp_ar AND USE_QT))

VP_SET(VISP_HAVE_ZBAR        TRUE IF (BUILD_MODULE_visp_detection AND USE_ZBAR))
VP_SET(VISP_HAVE_DMTX        TRUE IF (BUILD_MODULE_visp_detection AND USE_DMTX))

VP_SET(VISP_HAVE_AFMA4       TRUE IF (BUILD_MODULE_visp_robot AND USE_AFMA4))
VP_SET(VISP_HAVE_AFMA6       TRUE IF (BUILD_MODULE_visp_robot AND USE_AFMA6))
VP_SET(VISP_HAVE_VIPER650    TRUE IF (BUILD_MODULE_visp_robot AND USE_VIPER650))
VP_SET(VISP_HAVE_VIPER850    TRUE IF (BUILD_MODULE_visp_robot AND USE_VIPER850))
VP_SET(VISP_HAVE_BICLOPS     TRUE IF (BUILD_MODULE_visp_robot AND USE_BICLOPS))
VP_SET(VISP_HAVE_PTU46       TRUE IF (BUILD_MODULE_visp_robot AND USE_PTU46))
#VP_SET(VISP_HAVE_PIONEER     TRUE IF (BUILD_MODULE_visp_robot AND USE_ARIA))
if(BUILD_MODULE_visp_robot AND USE_ARIA)
  if(UNIX AND USE_PTHREAD AND RT_FOUND AND DL_FOUND)
    set(VISP_HAVE_PIONEER TRUE)
  elseif(NOT UNIX)
    set(VISP_HAVE_PIONEER TRUE)
  endif()
endif()
#VP_SET(VISP_HAVE_VIRTUOSE    TRUE IF (BUILD_MODULE_visp_robot AND USE_VIRTUOSE))
if(BUILD_MODULE_visp_robot AND USE_VIRTUOSE)
  if(UNIX AND USE_PTHREAD AND RT_FOUND AND DL_FOUND)
    set(VISP_HAVE_VIRTUOSE TRUE)
  elseif(NOT UNIX)
    set(VISP_HAVE_VIRTUOSE TRUE)
  endif()
endif()
VP_SET(VISP_HAVE_COIN3D      TRUE IF (BUILD_MODULE_visp_robot AND USE_COIN3D))

VP_SET(VISP_HAVE_V4L2        TRUE IF (BUILD_MODULE_visp_sensor AND USE_V4L2))
VP_SET(VISP_HAVE_DC1394      TRUE IF (BUILD_MODULE_visp_sensor AND USE_DC1394))
VP_SET(VISP_HAVE_CMU1394     TRUE IF (BUILD_MODULE_visp_sensor AND USE_CMU1394))
VP_SET(VISP_HAVE_DIRECTSHOW  TRUE IF (BUILD_MODULE_visp_sensor AND USE_DIRECTSHOW))
VP_SET(VISP_HAVE_LIBFREENECT TRUE IF (BUILD_MODULE_visp_sensor AND USE_LIBFREENECT))
VP_SET(VISP_HAVE_LIBUSB_1    TRUE IF (BUILD_MODULE_visp_sensor AND USE_LIBUSB_1))
VP_SET(VISP_HAVE_REALSENSE   TRUE IF (BUILD_MODULE_visp_sensor AND USE_REALSENSE))
VP_SET(VISP_HAVE_FLYCAPTURE  TRUE IF (BUILD_MODULE_visp_sensor AND USE_FLYCAPTURE))
VP_SET(VISP_HAVE_COMEDI      TRUE IF (BUILD_MODULE_visp_sensor AND USE_COMEDI))
VP_SET(VISP_HAVE_ATIDAQ      TRUE IF (BUILD_MODULE_visp_sensor AND WITH_ATIDAQ))
VP_SET(VISP_HAVE_CLIPPER     TRUE IF (BUILD_MODULE_visp_mbt AND BUILD_MODULE_visp_klt AND WITH_CLIPPER))

VP_SET(VISP_BUILD_SHARED_LIBS TRUE IF BUILD_SHARED_LIBS) # for header vpConfig.h
VP_SET(VISP_HAVE_DC1394_CAMERA_ENUMERATE TRUE IF (USE_DC1394 AND DC1394_CAMERA_ENUMERATE_FOUND)) # for header vpConfig.h
VP_SET(VISP_HAVE_DC1394_FIND_CAMERAS     TRUE IF (USE_DC1394 AND DC1394_FIND_CAMERAS_FOUND)) # for header vpConfig.h
VP_SET(VISP_HAVE_D3D9     TRUE IF USE_DIRECT3D) # for header vpConfig.h
VP_SET(VISP_HAVE_GTK      TRUE IF USE_GTK2) # for header vpConfig.h
VP_SET(VISP_HAVE_XRANDR   TRUE IF XRANDR) # for header vpConfig.h

# Check if libfreenect dependencies (ie libusb-1.0 and libpthread) are available
if(USE_LIBFREENECT AND USE_LIBUSB_1 AND USE_PTHREAD)
  if(LIBFREENECT_FOUND AND LIBUSB_1_FOUND AND PTHREAD_FOUND)
    set(VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES TRUE)

    # The material is found. Check if libfreenect is an old version
    include(CheckCXXSourceCompiles)
    set(CMAKE_REQUIRED_LIBRARIES ${LIBFREENECT_LIBRARIES} ${PTHREAD_LIBRARIES} ${LIBUSB_1_LIBRARIES})
    set(CMAKE_REQUIRED_INCLUDES ${LIBFREENECT_INCLUDE_DIRS} ${PTHREAD_INCLUDE_DIRS} ${LIBUSB_1_INCLUDE_DIRS})
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
    if(LIBFREENECT_IS_OLD_VERSION)
      set(VISP_HAVE_LIBFREENECT_OLD TRUE)  # for header vpConfig.h
    else()
      set(VISP_HAVE_LIBFREENECT_OLD FALSE) # for header vpConfig.h
    endif()
  endif()
endif()

# check OpenCV nonfree modules and version
if(USE_OPENCV)
  set(VISP_HAVE_OPENCV_VERSION "(${OpenCV_VERSION_MAJOR}<<16 | ${OpenCV_VERSION_MINOR}<<8 | ${OpenCV_VERSION_PATCH})") # for vpConfig.h
  if(OpenCV_VERSION)
    if(OpenCV_VERSION VERSION_LESS "2.4.0")
      message(STATUS "opencv nonfree module found")
      set(VISP_HAVE_OPENCV_NONFREE TRUE)  # for header vpConfig.h
    elseif(OPENCV_NONFREE_FOUND) # OpenCV < 3.0.0
      message(STATUS "opencv xfeatures2d module found")
      set(VISP_HAVE_OPENCV_NONFREE TRUE)  # for header vpConfig.h
    elseif(OPENCV_XFEATURES2D_FOUND) # OpenCV >= 3.0.0
      set(VISP_HAVE_OPENCV_XFEATURES2D TRUE)  # for header vpConfig.h
    else()
      message(STATUS "opencv nonfree or xfeature2d module not found")
    endif()
  else()
    message(STATUS "opencv nonfree not found")
    set(VISP_HAVE_OPENCV_VERSION "(0)") # for vpConfig.h
  endif()
endif()

# coin and gui
if(USE_SOWIN OR USE_SOXT OR (USE_SOQT AND USE_QT))
  set(VISP_HAVE_COIN3D_AND_GUI TRUE)         # for header vpConfig.h
endif()

# PCL and 3rd party advanced vars
if(USE_PCL)
  include(cmake/PCLTools.cmake)
endif()

# Find isnan macro (C-style)
VP_SET(VISP_HAVE_FUNC_ISNAN TRUE IF HAVE_FUNC_ISNAN) # for header vpConfig.h
# Find std::isnan function (cmath)
VP_SET(VISP_HAVE_FUNC_STD_ISNAN TRUE IF HAVE_FUNC_STD_ISNAN) # for header vpConfig.h
# Find _isnan function for MSVC
VP_SET(VISP_HAVE_FUNC__ISNAN TRUE IF HAVE_FUNC__ISNAN) # for header vpConfig.h
# Find isinf macro (C-style)
VP_SET(VISP_HAVE_FUNC_ISINF TRUE IF HAVE_FUNC_ISINF) # for header vpConfig.h
# Find std::isinf function (cmath)
VP_SET(VISP_HAVE_FUNC_STD_ISINF TRUE IF HAVE_FUNC_STD_ISINF) # for header vpConfig.h
# Find _finite function for MSVC
VP_SET(VISP_HAVE_FUNC__FINITE TRUE IF HAVE_FUNC__FINITE) # for header vpConfig.h
# Find round function (math.h)
VP_SET(VISP_HAVE_FUNC_ROUND TRUE IF HAVE_FUNC_ROUND) # for header vpConfig.h
# Find std::round function (cmath)
VP_SET(VISP_HAVE_FUNC_STD_ROUND TRUE IF HAVE_FUNC_STD_ROUND) # for header vpConfig.h
# Find erfc function (math.h)
VP_SET(VISP_HAVE_FUNC_ERFC TRUE IF HAVE_FUNC_ERFC) # for header vpConfig.h
# Find std::erfc function (cmath)
VP_SET(VISP_HAVE_FUNC_STD_ERFC TRUE IF HAVE_FUNC_STD_ERFC) # for header vpConfig.h
# Find strtof function (stdlib.h)
VP_SET(VISP_HAVE_FUNC_STRTOF TRUE IF HAVE_FUNC_STRTOF) # for header vpConfig.h
# Find log1p function (math.h)
VP_SET(VISP_HAVE_FUNC_LOG1P TRUE IF HAVE_FUNC_LOG1P) # for header vpConfig.h

VP_SET(VISP_BUILD_DEPRECATED_FUNCTIONS TRUE IF BUILD_DEPRECATED_FUNCTIONS) # for header vpConfig.h
VP_SET(VISP_MOMENTS_COMBINE_MATRICES TRUE IF ENABLE_MOMENTS_COMBINE_MATRICES) # for header vpConfig.h
VP_SET(VISP_USE_MSVC TRUE IF MSVC) # for header vpConfig.h
VP_SET(VISP_HAVE_CPP11_COMPATIBILITY TRUE IF USE_CPP11) # for header vpConfig.h
VP_SET(VISP_HAVE_BICLOPS_AND_GET_HOMED_STATE_FUNCTION TRUE IF (USE_BICLOPS AND BICLOPS_HAVE_GET_HOMED_STATE_FUNCTION)) # for header vpConfig.h

# libraries for Pioneer mobile robots
if(USE_ARIA AND UNIX)
  if(ARIA_FOUND AND USE_PTHREAD AND RT_FOUND AND DL_FOUND)
    set(VISP_HAVE_PIONEER TRUE) # for header vpConfig.h
  endif()
elseif(USE_ARIA AND NOT UNIX)
  set(VISP_HAVE_PIONEER TRUE) # for header vpConfig.h
endif()

# ----------------------------------------------------------------------------
# Finalization: generate configuration-based files
# ----------------------------------------------------------------------------

# Generate platform-dependent and configuration-dependent headers
include(cmake/VISPGenerateHeaders.cmake)

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
include(cmake/VISPGenerateConfig.cmake)

# Generate Info.plist for the IOS/OSX frameworks
if(APPLE_FRAMEWORK)
  include(cmake/VISPGenerateInfoPlist.cmake)
endif()

#----------------------------------------------------------------------
# For Dart server and tests
# We use CDash set through CTestConfig.cmake file
# Dashboards are sent to http://cdash.irisa.fr/CDash/index.php?project=ViSP
#----------------------------------------------------------------------
if(BUILD_TESTS)
  include(CTest)
endif()

#----------------------------------------------------------------------
# For CPack packaging tool
#----------------------------------------------------------------------

option(BUILD_PACKAGE "Configure ViSP packaging" OFF)
if(BUILD_PACKAGE)
  if(UNIX AND NOT APPLE AND NOT WIN32) # =linux
    option(BUILD_PACKAGE_DEBIAN "Build debian package" ON)
    option(BUILD_PACKAGE_RPM "Build rpm package" ON)
  endif()

  include(cmake/CPackConfig.cmake)
endif(BUILD_PACKAGE)


#----------------------------------------------------------------------
# Generate the package dependent visp-config shell script for projects which
# are not using CMake:
# Usage:
#    visp-config --cflags ...
#----------------------------------------------------------------------
include(cmake/VISPGeneratePkgConfigScript.cmake)

#----------------------------------------------------------------------
# Propagation in sub dirs to build demo, example, test, tutorial
#----------------------------------------------------------------------

set(VISP_DIR ${PROJECT_BINARY_DIR})
mark_as_advanced(VISP_DIR)
mark_as_advanced(VISP_INCLUDE_DIRS)

if(BUILD_DEMOS)
  add_subdirectory(demo)
  vp_add_subdirectories(VISP_CONTRIB_MODULES_PATH demo)
endif()
if(BUILD_EXAMPLES)
  add_subdirectory(example)
  vp_add_subdirectories(VISP_CONTRIB_MODULES_PATH example)
endif()
if(BUILD_TUTORIALS)
  add_subdirectory(tutorial)
  vp_add_subdirectories(VISP_CONTRIB_MODULES_PATH tutorial)
endif()

# ----------------------------------------------------------------------------
#  Update version of 3rd party for which info is missing
# ----------------------------------------------------------------------------
if(USE_JPEG)
  if(NOT JPEG_LIB_VERSION)
    vp_parse_header("${JPEG_INCLUDE_DIR}/jpeglib.h" JPEG_VERSION_LINES JPEG_LIB_VERSION)
  endif()
  if(NOT JPEG_LIB_VERSION)
    get_filename_component(JPEG_LIB_DIR ${JPEG_LIBRARIES} PATH)
    string(REPLACE "lib" "include" JPEG_INC_DIR ${JPEG_LIB_DIR})
    vp_parse_header("${JPEG_INC_DIR}/jconfig.h" JPEG_VERSION_LINES JPEG_LIB_VERSION)
  endif()
endif()
if(USE_OIS)
  vp_parse_header("${OIS_INCLUDE_DIR}/OISPrereqs.h" OIS_VERSION_LINES OIS_VERSION_MAJOR OIS_VERSION_MINOR OIS_VERSION_PATCH)
  set(OIS_VERSION "${OIS_VERSION_MAJOR}.${OIS_VERSION_MINOR}.${OIS_VERSION_PATCH}")
endif()
if(USE_EIGEN3)
  # Additionnal check to be sure that Eigen3 include dir os well detected
  if(NOT EXISTS ${EIGEN3_INCLUDE_DIR})
    message(WARNING "Eigen3 config file is detected in ${Eigen3_DIR}. Using ${Eigen3_DIR}/Eigen3Config.cmake doesn't allow to use a valid Eigen3 include dir: ${EIGEN3_INCLUDE_DIR}. Modify Eigen3_DIR to point to Eigen3Config.cmake file located in Eigen3 installation folder or turn USE_EIGEN3 OFF.")
  endif()
  if(NOT EIGEN3_VERSION_STRING)
    vp_parse_header("${EIGEN3_INCLUDE_DIR}/Eigen/src/Core/util/Macros.h" EIGEN3_VERSION_LINES EIGEN_WORLD_VERSION EIGEN_MAJOR_VERSION EIGEN_MINOR_VERSION)
    set(EIGEN3_VERSION_STRING "${EIGEN_WORLD_VERSION}.${EIGEN_MAJOR_VERSION}.${EIGEN_MINOR_VERSION}")
  endif()
endif()
if(USE_COIN3D)
  vp_parse_header("${COIN3D_INCLUDE_DIRS}/Inventor/C/basic.h" COIN_VERSION_LINES COIN_MAJOR_VERSION COIN_MINOR_VERSION COIN_MICRO_VERSION)
  set(COIN3D_VERSION "${COIN_MAJOR_VERSION}.${COIN_MINOR_VERSION}.${COIN_MICRO_VERSION}")
endif()

# ----------------------------------------------------------------------------
#  Autodetect if we are in a GIT repository
# ----------------------------------------------------------------------------
find_host_package(Git QUIET)

if(NOT DEFINED VISP_VCSVERSION AND GIT_FOUND)
  execute_process(COMMAND "${GIT_EXECUTABLE}" describe --tags --always --dirty --match "[0-9].[0-9].[0-9]*"
    WORKING_DIRECTORY "${VISP_SOURCE_DIR}"
    OUTPUT_VARIABLE VISP_VCSVERSION
    RESULT_VARIABLE GIT_RESULT
    ERROR_QUIET
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )
  if(NOT GIT_RESULT EQUAL 0)
    set(VISP_VCSVERSION "unknown")
  endif()
elseif(NOT DEFINED VISP_VCSVERSION)
  # We don't have git:
  set(VISP_VCSVERSION "unknown")
endif()

# ----------------------------------------------------------------------------
# Summary:
# ----------------------------------------------------------------------------
status("")
status("==========================================================")
status("General configuration information for ViSP ${VISP_VERSION}")
status("")
if(VISP_VCSVERSION)
  status("  Version control:" ${VISP_VCSVERSION})
endif()

if(VISP_CONTRIB_MODULES_PATH)
  set(__dump_extra_header OFF)
  foreach(p ${VISP_CONTRIB_MODULES_PATH})
    if(EXISTS ${p})
      if(NOT __dump_extra_header)
        set(__dump_extra_header ON)
        status("")
        status("  Extra modules:")
      else()
        status("")
      endif()
      set(EXTRA_MODULES_VCSVERSION "unknown")
      if(GIT_FOUND)
        execute_process(COMMAND "${GIT_EXECUTABLE}" describe --tags --always --dirty --match "[0-9].[0-9].[0-9]*"
          WORKING_DIRECTORY "${p}"
          OUTPUT_VARIABLE EXTRA_MODULES_VCSVERSION
          RESULT_VARIABLE GIT_RESULT
          ERROR_QUIET
          OUTPUT_STRIP_TRAILING_WHITESPACE
        )
        if(NOT GIT_RESULT EQUAL 0)
          set(EXTRA_MODULES_VCSVERSION "unknown")
        endif()
      endif()
      status("    Location (extra):" ${p})
      status("    Version control (extra):" ${EXTRA_MODULES_VCSVERSION})
    endif()
  endforeach()
  unset(__dump_extra_header)
endif()

# ========================== build platform ==========================
status("")
status("  Platform:")
if(NOT CMAKE_VERSION VERSION_LESS 2.8.11)
  string(TIMESTAMP TIMESTAMP "" UTC)
  if(TIMESTAMP)
    status("    Timestamp:"    ${TIMESTAMP})
  endif()
endif()
status("    Host:"             ${CMAKE_HOST_SYSTEM_NAME} ${CMAKE_HOST_SYSTEM_VERSION} ${CMAKE_HOST_SYSTEM_PROCESSOR})
if(CMAKE_CROSSCOMPILING)
  status("    Target:"         ${CMAKE_SYSTEM_NAME} ${CMAKE_SYSTEM_VERSION} ${CMAKE_SYSTEM_PROCESSOR})
endif()
status("    CMake:"            ${CMAKE_VERSION})
status("    CMake generator:"  ${CMAKE_GENERATOR})
status("    CMake build tool:" ${CMAKE_BUILD_TOOL})
if(MSVC)
  status("    MSVC:"           ${MSVC_VERSION})
endif()
if(CMAKE_GENERATOR MATCHES Xcode)
  status("    Xcode:"          ${XCODE_VERSION})
endif()
if(NOT CMAKE_GENERATOR MATCHES "Xcode|Visual Studio")
  status("    Configuration:"  ${CMAKE_BUILD_TYPE})
endif()

# ========================== C/C++ options ==========================
if(CMAKE_CXX_COMPILER_VERSION)
  set(VISP_COMPILER_STR "${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} (ver ${CMAKE_CXX_COMPILER_VERSION})")
elseif(CMAKE_COMPILER_IS_CLANGCXX)
  set(VISP_COMPILER_STR "${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} (ver ${CMAKE_CLANG_REGEX_VERSION})")
elseif(CMAKE_COMPILER_IS_GNUCXX)
  set(VISP_COMPILER_STR "${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1} (ver ${CMAKE_GCC_REGEX_VERSION})")
else()
  set(VISP_COMPILER_STR "${CMAKE_CXX_COMPILER} ${CMAKE_CXX_COMPILER_ARG1}")
endif()
string(STRIP "${VISP_COMPILER_STR}" VISP_COMPILER_STR)

status("")
status("  C/C++:")
status("    Built as dynamic libs?:" BUILD_SHARED_LIBS THEN "yes" ELSE "no")
status("    C++ Compiler:"           ${VISP_COMPILER_STR})
status("    C++ flags (Release):"    ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_RELEASE})
status("    C++ flags (Debug):"      ${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_DEBUG})
status("    C Compiler:"             ${CMAKE_C_COMPILER} ${CMAKE_C_COMPILER_ARG1})
status("    C flags (Release):"      ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_RELEASE})
status("    C flags (Debug):"        ${CMAKE_C_FLAGS} ${CMAKE_C_FLAGS_DEBUG})
if(WIN32)
  status("    Linker flags (Release):" ${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_RELEASE})
  status("    Linker flags (Debug):"   ${CMAKE_EXE_LINKER_FLAGS} ${CMAKE_EXE_LINKER_FLAGS_DEBUG})
else()
  status("    Linker flags (Release):" ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_RELEASE})
  status("    Linker flags (Debug):"   ${CMAKE_SHARED_LINKER_FLAGS} ${CMAKE_SHARED_LINKER_FLAGS_DEBUG})
endif()

# ========================== ViSP modules ==========================
status("")
status("  ViSP modules:")
string(REPLACE "visp_" "" VISP_MODULES_BUILD_ST          "${VISP_MODULES_BUILD}")
string(REPLACE "visp_" "" VISP_MODULES_DISABLED_USER_ST  "${VISP_MODULES_DISABLED_USER}")
string(REPLACE "visp_" "" VISP_MODULES_DISABLED_FORCE_ST "${VISP_MODULES_DISABLED_FORCE}")
set(VISP_MODULES_DISABLED_AUTO_ST "")
foreach(m ${VISP_MODULES_DISABLED_AUTO})
  set(__mdeps "")
  foreach(d ${VISP_MODULE_${m}_DEPS})
    if(d MATCHES "^visp_" AND NOT USE_${d})
      list(APPEND __mdeps ${d})
    endif()
  endforeach()
  if(__mdeps)
    list(APPEND VISP_MODULES_DISABLED_AUTO_ST "${m}(deps: ${__mdeps})")
  else()
    list(APPEND VISP_MODULES_DISABLED_AUTO_ST "${m}")
  endif()
endforeach()
string(REPLACE "visp_" "" VISP_MODULES_DISABLED_AUTO_ST  "${VISP_MODULES_DISABLED_AUTO_ST}")

status("    To be built:"            VISP_MODULES_BUILD          THEN ${VISP_MODULES_BUILD_ST}          ELSE "-")
status("    Disabled:"               VISP_MODULES_DISABLED_USER  THEN ${VISP_MODULES_DISABLED_USER_ST}  ELSE "-")
status("    Disabled by dependency:" VISP_MODULES_DISABLED_AUTO  THEN ${VISP_MODULES_DISABLED_AUTO_ST}  ELSE "-")
status("    Unavailable:"            VISP_MODULES_DISABLED_FORCE THEN ${VISP_MODULES_DISABLED_FORCE_ST} ELSE "-")

# ================== Windows RT features ==================
if(WIN32)
  status("")
  status("  Windows RT support:" WINRT THEN "yes" ELSE "no")
  if(WINRT)
    status("    Building for Microsoft platform: " ${CMAKE_SYSTEM_NAME})
    status("    Building for architectures: " ${CMAKE_VS_EFFECTIVE_PLATFORMS})
    status("    Building for version: " ${CMAKE_SYSTEM_VERSION})
  endif()
endif()

# ============================ Options ===========================
status("")
status("  Build options: ")
status("    Build deprecated:"           BUILD_DEPRECATED_FUNCTIONS      THEN "yes" ELSE "no")
status("    Build with moment combine:"  ENABLE_MOMENTS_COMBINE_MATRICES THEN "yes" ELSE "no")


# ===================== Optional 3rd parties =====================
status("")
status("  Mathematics: ")
status("    Use Lapack/blas:"        USE_LAPACK       THEN "yes (ver ${LAPACK_C_VERSION})" ELSE "no")
status("    Use Lapack (built-in):"  WITH_LAPACK      THEN "yes (ver ${LAPACK_VERSION})" ELSE "no")
status("    Use Eigen3:"             USE_EIGEN3       THEN "yes (ver ${EIGEN3_VERSION_STRING})" ELSE "no")
status("    Use OpenCV:"             USE_OPENCV       THEN "yes (ver ${OpenCV_VERSION})" ELSE "no")
status("    Use GSL:"                USE_GSL          THEN "yes (ver ${GSL_VERSION})" ELSE "no")
status("")
status("  Simulator: ")
status("    Ogre simulator: ")
status("    \\- Use Ogre3D:"         USE_OGRE         THEN "yes (ver ${OGRE_VERSION})" ELSE "no")
status("    \\- Use OIS:"            USE_OIS          THEN "yes (ver ${OIS_VERSION})" ELSE "no")
status("    Coin simulator: ")
status("    \\- Use Coin3D:"         USE_OGRE         THEN "yes (ver ${COIN3D_VERSION})" ELSE "no")
status("    \\- Use SoWin:"          USE_SOWIN        THEN "yes" ELSE "no")
status("    \\- Use SoXt:"           USE_SOXT         THEN "yes" ELSE "no")
status("    \\- Use SoQt:"           USE_SOQT         THEN "yes" ELSE "no")
status("    \\- Use Qt4:"            USE_QT AND DESIRED_QT_VERSION MATCHES 4  THEN "yes" ELSE "no")
status("    \\- Use Qt3:"            USE_QT AND DESIRED_QT_VERSION MATCHES 3  THEN "yes" ELSE "no")
status("")
status("  Media I/O: ")
status("    Use JPEG:"               USE_JPEG         THEN "yes (ver ${JPEG_LIB_VERSION})" ELSE "no")
status("    Use PNG:"                USE_PNG          THEN "yes (ver ${PNG_VERSION_STRING})" ELSE "no")
status("    \\- Use ZLIB:"           USE_ZLIB         THEN "yes (ver ${ZLIB_VERSION_STRING})" ELSE "no")
status("    Use FFMPEG:"             USE_FFMPEG       THEN "yes" ELSE "no")
status("    Use OpenCV:"             USE_OPENCV       THEN "yes (ver ${OpenCV_VERSION})" ELSE "no")
status("")
status("  Real robots: ")
status("    Use Afma4:"              USE_AFMA4        THEN "yes" ELSE "no")
status("    Use Afma6:"              USE_AFMA6        THEN "yes" ELSE "no")
status("    Use Viper650:"           USE_VIPER850     THEN "yes" ELSE "no")
status("    Use Viper850:"           USE_VIPER850     THEN "yes" ELSE "no")
status("    Use aria (Pioneer):"     USE_ARIA         THEN "yes" ELSE "no")
status("    Use PTU46:"              USE_PTU46        THEN "yes" ELSE "no")
status("    Use Biclops PT:"         USE_BICLOPS      THEN "yes" ELSE "no")
status("")
status("  GUI: ")
status("    Use X11:"                USE_X11          THEN "yes" ELSE "no")
status("    Use GTK:"                USE_GTK2         THEN "yes (ver ${GTK2_VERSION})" ELSE "no")
status("    Use OpenCV:"             USE_OPENCV       THEN "yes (ver ${OpenCV_VERSION})" ELSE "no")
status("    Use GDI:"                USE_GDI          THEN "yes" ELSE "no")
status("    Use Direct3D:"           USE_DIRECT3D     THEN "yes" ELSE "no")
status("")
status("  Cameras: ")
status("    Use DC1394-2.x:"         USE_DC1394       THEN "yes (ver ${DC1394_VERSION})" ELSE "no")
status("    Use CMU 1394:"           USE_CMU1394      THEN "yes" ELSE "no")
status("    Use V4L2:"               USE_V4L2         THEN "yes (ver ${V4L2_VERSION})" ELSE "no")
status("    Use directshow:"         USE_DIRECTSHOW   THEN "yes" ELSE "no")
status("    Use OpenCV:"             USE_OPENCV       THEN "yes (ver ${OpenCV_VERSION})" ELSE "no")
status("    Use Flycapture:"         USE_FLYCAPTURE   THEN "yes (ver ${FLYCAPTURE_VERSION})" ELSE "no")
status("")
status("  RGB-D sensors: ")
status("    Use Realsense:"          USE_REALSENSE    THEN "yes (ver ${REALSENSE_VERSION})" ELSE "no")
status("    \\- Use PCL:"            USE_PCL          THEN "yes (ver ${PCL_VERSION})" ELSE "no")
status("       \\- Use VTK:"         VTK_FOUND        THEN "yes (ver ${VTK_VERSION})" ELSE "no")
status("    Use Kinect:"             USE_LIBFREENECT AND USE_LIBUSB_1 AND USE_PTHREAD THEN "yes" ELSE "no")
status("    \\- Use libfreenect:"    USE_LIBFREENECT  THEN "yes (ver ${LIBFREENECT_VERSION})" ELSE "no")
status("    \\- Use libusb-1:"       USE_LIBUSB_1     THEN "yes (ver ${LIBUSB_1_VERSION})" ELSE "no")
status("    \\- Use pthread:"        USE_PTHREAD      THEN "yes" ELSE "no")
status("")
status("  F/T sensors: ")
status("    Use atidaq (built-in):"  WITH_ATIDAQ      THEN "yes (ver ${ATIDAQ_VERSION})" ELSE "no")
status("    Use comedi:"             USE_COMEDI       THEN "yes (ver ${COMEDI_VERSION})" ELSE "no")
status("")
status("  Detection: ")
status("    Use zbar:"               USE_ZBAR         THEN "yes (ver ${ZBAR_VERSION})" ELSE "no")
status("    Use dmtx:"               USE_DMTX         THEN "yes (ver ${DMTX_VERSION})" ELSE "no")
status("")
status("  Misc: ")
status("    Use clipper (built-in):" WITH_CLIPPER     THEN "yes (ver ${CLIPPER_VERSION})" ELSE "no")
status("    Use XML2:"               USE_XML2         THEN "yes (ver ${XML2_VERSION_STRING})" ELSE "no")
status("")
status("  Optimization: ")
status("    Use OpenMP:"             USE_OPENMP       THEN "yes" ELSE "no")
status("    Use pthread:"            USE_PTHREAD      THEN "yes" ELSE "no")
status("    Use cxx11:"              USE_CPP11        THEN "yes" ELSE "no")

# ========================== documentation ==========================
status("")
status("  Documentation: ")
status("    Use doxygen:"            DOXYGEN_FOUND    THEN "yes" ELSE "no")

# ========================== samples and tests ==========================
status("")
status("  Tests and samples:")
status("    Tests:"                  BUILD_TESTS      THEN "yes" ELSE "no")
status("    Demos:"                  BUILD_DEMOS      THEN "yes" ELSE "no")
status("    Examples:"               BUILD_EXAMPLES   THEN "yes" ELSE "no")
status("    Tutorials:"              BUILD_TUTORIALS  THEN "yes" ELSE "no")

# ========================== auxiliary ==========================
status("")
status("  Install path:" "${CMAKE_INSTALL_PREFIX}")
status("")

status("==========================================================")

vp_finalize_status()
