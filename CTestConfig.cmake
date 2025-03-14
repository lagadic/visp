#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 2 of the License, or
# (at your option) any later version.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact Inria about acquiring a ViSP Professional
# Edition License.
#
# See https://visp.inria.fr for more information.
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
# CDash configuration.
#
#############################################################################

set(CTEST_PROJECT_NAME "ViSP")
set(CTEST_NIGHTLY_START_TIME "00:00:00 GMT")

set(CTEST_DROP_METHOD "https")
set(CTEST_DROP_SITE "cdash-ci.inria.fr")
set(CTEST_DROP_LOCATION "/submit.php?project=ViSP")
set(CTEST_DROP_SITE_CDASH TRUE)

#--------------------------------------------------------------------
# Prepare valgrind suppression file
#--------------------------------------------------------------------
if(UNIX)
  # Prepare a temporary file to concatenate to
  file(WRITE "${VISP_BINARY_DIR}/valgrind.supp.in" "")

  # Concatenate all suppression files in
  file(GLOB suppression_files "${VISP_SOURCE_DIR}/platforms/scripts/valgrind/*.supp")
  foreach(f ${suppression_files})
    vp_cat_file(${f} "${VISP_BINARY_DIR}/valgrind.supp.in")
  endforeach()

  # Copy the temporary file to the final location
  configure_file("${VISP_BINARY_DIR}/valgrind.supp.in" "${VISP_BINARY_DIR}/valgrind.supp" COPYONLY)

  # Valgrind false positive suppression file
  set(MEMORYCHECK_SUPPRESSIONS_FILE "${VISP_BINARY_DIR}/valgrind.supp")
endif()

#--------------------------------------------------------------------
# BUILNAME variable construction
# This variable will be used to set the build name which will appear
# on the ViSP dashboard http://cdash.irisa.fr/CDash/
#--------------------------------------------------------------------
# Start with the short system name, e.g. "Linux", "FreeBSD" or "Windows"
if(BUILDNAME)
  set(BUILDNAME "${BUILDNAME}-${CMAKE_SYSTEM_NAME}")
else()
  # To suppress the first space if BUILDNAME is not set
  set(BUILDNAME "${CMAKE_SYSTEM_NAME}")
endif()

# Add i386 or amd64
if(CMAKE_SIZEOF_VOID_P EQUAL 8)
  set(BUILDNAME "${BUILDNAME}-amd64")
else()
  set(BUILDNAME "${BUILDNAME}-i386")
endif()

# Add the compiler name, e.g. "g++, msvc..."
if(VISP_RUNTIME)
  # msvc, mingw
  set(BUILDNAME "${BUILDNAME}-${VISP_RUNTIME}")
else()
  # g++
  set(BUILDNAME "${BUILDNAME}-${CMAKE_BASE_NAME}")
endif()

# Find out the version of gcc being used.
if(CMAKE_CXX_COMPILER_VERSION)
  set(BUILDNAME "${BUILDNAME}-${CMAKE_CXX_COMPILER_VERSION}")
endif()

# Add the type of library generation, e.g. "Dynamic or Static"
if(BUILD_SHARED_LIBS)
  set(BUILDNAME "${BUILDNAME}-Dyn")
else(BUILD_SHARED_LIBS)
  set(BUILDNAME "${BUILDNAME}-Sta")
endif()

# Add the build type, e.g. "Debug, Release..."
if(CMAKE_BUILD_TYPE)
  set(BUILDNAME "${BUILDNAME}-${CMAKE_BUILD_TYPE}")
endif()

#---- Robots ----
# Add specific Afma4 robots
if(VISP_HAVE_AFMA4)
  set(BUILDNAME "${BUILDNAME}-Afma4")
endif()

# Add specific Afma6 robots
if(VISP_HAVE_AFMA6)
  set(BUILDNAME "${BUILDNAME}-Afma6")
endif()

# Add specific Ptu46 robots
if(VISP_HAVE_PTU46)
  set(BUILDNAME "${BUILDNAME}-Ptu46")
endif()

# Add specific Biclops robots
if(VISP_HAVE_BICLOPS)
  set(BUILDNAME "${BUILDNAME}-Biclops")
endif()

# Add specific Pioneer robots
if(VISP_HAVE_PIONEER)
  set(BUILDNAME "${BUILDNAME}-aria")
endif()

# Add specific Haption Virtuose haptic device
if(VISP_HAVE_VIRTUOSE)
  set(BUILDNAME "${BUILDNAME}-virtuose")
endif()

#---- Framegrabers/Sensors ----
# Firewire dc1394-2.x
if(VISP_HAVE_DC1394)
  set(BUILDNAME "${BUILDNAME}-dc1394")
endif()
# Video 4 linux 2 (V4L2)
if(VISP_HAVE_V4L2)
  set(BUILDNAME "${BUILDNAME}-v4l2")
endif()
# Directshow
if(VISP_HAVE_DIRECTSHOW)
  set(BUILDNAME "${BUILDNAME}-dshow")
endif()
if(VISP_HAVE_CMU1394)
  set(BUILDNAME "${BUILDNAME}-CMU1394")
endif()
if(VISP_HAVE_LIBFREENECT)
  set(BUILDNAME "${BUILDNAME}-freenect")
endif()
if(VISP_HAVE_REALSENSE)
  set(BUILDNAME "${BUILDNAME}-rs")
endif()
if(VISP_HAVE_PCL)
  set(BUILDNAME "${BUILDNAME}-pcl")
endif()
if(VISP_HAVE_LIBUSB_1)
  set(BUILDNAME "${BUILDNAME}-usb")
endif()
if(VISP_HAVE_FLYCAPTURE)
  set(BUILDNAME "${BUILDNAME}-flycap")
endif()
if(VISP_HAVE_PYLON)
  set(BUILDNAME "${BUILDNAME}-pylon")
endif()
if(VISP_HAVE_UEYE)
  set(BUILDNAME "${BUILDNAME}-ueye")
endif()
if(VISP_HAVE_COMEDI)
  set(BUILDNAME "${BUILDNAME}-comedi")
endif()
if(VISP_HAVE_ATI)
  set(BUILDNAME "${BUILDNAME}-ati")
endif()

#---- Video-devices ----
# X11
if(VISP_HAVE_X11)
  set(BUILDNAME "${BUILDNAME}-X11")
endif()
# GTK
if(VISP_HAVE_GTK)
  set(BUILDNAME "${BUILDNAME}-gtk")
endif()
# GDI (Windows Graphics Device Interface)
if(VISP_HAVE_GDI)
  set(BUILDNAME "${BUILDNAME}-gdi")
endif()
# D3D (Direct3D9)
if(VISP_HAVE_D3D9)
  set(BUILDNAME "${BUILDNAME}-Direct3D")
endif()
# OpenCV
if(VISP_HAVE_OPENCV)
  if(OpenCV_VERSION)
    if(OPENCV_XFEATURES2D_FOUND OR OPENCV_XFEATURES2D_FOUND)
      set(BUILDNAME "${BUILDNAME}-OpenCV_contrib${OpenCV_VERSION}")
    else()
      set(BUILDNAME "${BUILDNAME}-OpenCV${OpenCV_VERSION}")
    endif()
  else()
    set(BUILDNAME "${BUILDNAME}-OpenCV")
  endif()
endif()

#---- Mathematics ----
# Lapack (Linear Algebra PACKage)
if(VISP_HAVE_LAPACK)
  set(BUILDNAME "${BUILDNAME}-lapack")
endif()
# GSL (Gnu Scientific Library)
if(VISP_HAVE_GSL)
  set(BUILDNAME "${BUILDNAME}-gsl")
endif()
if(VISP_HAVE_EIGEN3)
  set(BUILDNAME "${BUILDNAME}-eigen3")
endif()

#---- Simulator ----
# Ogre
if(VISP_HAVE_OGRE)
  set(BUILDNAME "${BUILDNAME}-Ogre")
endif()
if(VISP_HAVE_OIS)
  set(BUILDNAME "${BUILDNAME}-OIS")
endif()
# Coin
if(VISP_HAVE_COIN3D)
  set(BUILDNAME "${BUILDNAME}-Coin")
endif()
# SoQt
if(VISP_HAVE_SOQT)
  set(BUILDNAME "${BUILDNAME}-SoQt")
endif()
# Qt
if(VISP_HAVE_QT)
  set(BUILDNAME "${BUILDNAME}-Qt${DESIRED_QT_VERSION}")
endif()
# SoWin
if(VISP_HAVE_SOWIN)
  set(BUILDNAME "${BUILDNAME}-SoWin")
endif()
# SoXt
if(VISP_HAVE_SOXT)
  set(BUILDNAME "${BUILDNAME}-SoXt")
endif()

#---- Images ----
if(VISP_HAVE_JPEG)
  set(BUILDNAME "${BUILDNAME}-jpeg")
endif()
if(VISP_HAVE_PNG)
  set(BUILDNAME "${BUILDNAME}-png")
endif()

#---- Misc ----
# XML
if(VISP_HAVE_XML2)
  set(BUILDNAME "${BUILDNAME}-xml")
endif()
# PThread
if(VISP_HAVE_THREADS)
  set(BUILDNAME "${BUILDNAME}-threads")
endif()
# OpenMP
if(VISP_HAVE_OPENMP)
  set(BUILDNAME "${BUILDNAME}-OpenMP")
endif()
if(VISP_HAVE_DMTX)
  set(BUILDNAME "${BUILDNAME}-dmtx")
endif()
if(VISP_HAVE_ZBAR)
  set(BUILDNAME "${BUILDNAME}-zbar")
endif()
if(VISP_HAVE_APRILTAG)
  set(BUILDNAME "${BUILDNAME}-apriltag")
endif()

#---- Special compiler flags ----
if(ACTIVATE_WARNING_STRICT_OVERFLOW)
  set(BUILDNAME "${BUILDNAME}-Wov")
endif()
if(ACTIVATE_WARNING_FLOAT_EQUAL)
  set(BUILDNAME "${BUILDNAME}-Weq")
endif()
if(VISP_CXX_STANDARD EQUAL VISP_CXX_STANDARD_98)
  set(BUILDNAME "${BUILDNAME}-c98")
elseif(VISP_CXX_STANDARD EQUAL VISP_CXX_STANDARD_11)
  set(BUILDNAME "${BUILDNAME}-c11")
elseif(VISP_CXX_STANDARD EQUAL VISP_CXX_STANDARD_14)
  set(BUILDNAME "${BUILDNAME}-c14")
elseif(VISP_CXX_STANDARD EQUAL VISP_CXX_STANDARD_17)
  set(BUILDNAME "${BUILDNAME}-c17")
endif()
if(ENABLE_MOMENTS_COMBINE_MATRICES)
  set(BUILDNAME "${BUILDNAME}-Moment")
endif()
if(ENABLE_SSE2 OR ENABLE_SSE3 OR ENABLE_SSSE3)
  set(BUILDNAME "${BUILDNAME}-sse")
endif()

#---- Suffix contrib ----
if(VISP_CONTRIB_MODULES_PATH)
  set(BUILDNAME "${BUILDNAME}-contrib")
endif()

#message("BUILDNAME=${BUILDNAME}")
