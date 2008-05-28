#############################################################################
#
# $Id: CTestConfig.cmake,v 1.5 2008-05-28 12:55:03 fspindle Exp $
#
# Copyright (C) 1998-2008 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
#
# Description:
# CDash configuration.
#
# Authors:
# Fabien Spindler
#
#############################################################################

set(CTEST_PROJECT_NAME "ViSP")
set(CTEST_NIGHTLY_START_TIME "00:00:00 GMT")

set(CTEST_DROP_METHOD "http")
set(CTEST_DROP_SITE "cdash.inria.fr")
set(CTEST_DROP_LOCATION "/CDash/submit.php?project=ViSP")
set(CTEST_DROP_SITE_CDASH TRUE)

#--------------------------------------------------------------------
# BUILNAME variable construction
# This variable will be used to set the build name which will appear 
# on the ViSP dashboard http://cdash.irisa.fr/CDash/
#--------------------------------------------------------------------
# Start with the short system name, e.g. "Linux", "FreeBSD" or "Windows"
IF(BUILDNAME)
  SET(BUILDNAME "${BUILDNAME}-${CMAKE_SYSTEM_NAME}")
ELSE(BUILDNAME)
  # To suppress the first space if BUILDNAME is not set
  SET(BUILDNAME "${CMAKE_SYSTEM_NAME}")
ENDIF(BUILDNAME)

# Add the compiler name, e.g. "g++, msvc7..."
IF(MSVC70)
  SET(BUILDNAME "${BUILDNAME}-msvc70")
ELSEIF(MSVC71)
  SET(BUILDNAME "${BUILDNAME}-msvc71")
ELSEIF(MSVC80)
  SET(BUILDNAME "${BUILDNAME}-msvc80")
ELSEIF(BORLAND)
  SET(BUILDNAME "${BUILDNAME}-borland")
ELSEIF(MINGW)
  SET(BUILDNAME "${BUILDNAME}-mingw")
ELSE(MSVC70)
  SET(BUILDNAME "${BUILDNAME}-${CMAKE_BASE_NAME}")
ENDIF(MSVC70)

# Add the type of library generation, e.g. "Dynamic or Static"
IF(BUILD_SHARED_LIBS)
  SET(BUILDNAME "${BUILDNAME}-Dynamic")
ELSE(BUILD_SHARED_LIBS)
  SET(BUILDNAME "${BUILDNAME}-Static")
ENDIF(BUILD_SHARED_LIBS)

# Add the build type, e.g. "Debug, Release..."
# for Unix
IF(CMAKE_BUILD_TYPE)
  SET(BUILDNAME "${BUILDNAME}-${CMAKE_BUILD_TYPE}")
ENDIF(CMAKE_BUILD_TYPE)
# for Windows
IF(CMAKE_CONFIGURATION_TYPES)
  SET(BUILDNAME "${BUILDNAME}-${CMAKE_CONFIGURATION_TYPES}")
ENDIF(CMAKE_CONFIGURATION_TYPES)

#---- Robots ----
# Add specific Afma4 robots
IF(VISP_HAVE_AFMA4)
  SET(BUILDNAME "${BUILDNAME}-Afma4")
ENDIF(VISP_HAVE_AFMA4)
 
# Add specific Afma6 robots
IF(VISP_HAVE_AFMA6)
  SET(BUILDNAME "${BUILDNAME}-Afma6")
ENDIF(VISP_HAVE_AFMA6)

# Add specific Ptu46 robots
IF(VISP_HAVE_PTU46)
  SET(BUILDNAME "${BUILDNAME}-Ptu46")
ENDIF(VISP_HAVE_PTU46)

# Add specific Biclops robots
IF(VISP_HAVE_BICLOPS)
  SET(BUILDNAME "${BUILDNAME}-Biclops")
ENDIF(VISP_HAVE_BICLOPS)

#---- Framegrabers ----
# Firewire dc1394-2.x 
IF(VISP_HAVE_DC1394_2)
  SET(BUILDNAME "${BUILDNAME}-dc1394.2")
ENDIF(VISP_HAVE_DC1394_2)
# Firewire dc1394-1.x 
IF(VISP_HAVE_DC1394_1)
  SET(BUILDNAME "${BUILDNAME}-dc1394.1")
ENDIF(VISP_HAVE_DC1394_1)
# Video 4 linux 2 (V4L2)
IF(VISP_HAVE_V4L2)
  SET(BUILDNAME "${BUILDNAME}-v4l2")
ENDIF(VISP_HAVE_V4L2)
# Imaging Technology itifg
IF(VISP_HAVE_ITIFG8)
  SET(BUILDNAME "${BUILDNAME}-itifg8")
ENDIF(VISP_HAVE_ITIFG8)
# Directshow
IF(VISP_HAVE_DIRECTSHOW)
  SET(BUILDNAME "${BUILDNAME}-Directshow")
ENDIF(VISP_HAVE_DIRECTSHOW)
# Cfox
IF(VISP_HAVE_CFOX)
  SET(BUILDNAME "${BUILDNAME}-cfox")
ENDIF(VISP_HAVE_CFOX)

#---- Video-devices ----
# X11
IF(VISP_HAVE_X11)
  SET(BUILDNAME "${BUILDNAME}-X11")
ENDIF(VISP_HAVE_X11)
# GTK
IF(VISP_HAVE_GTK)
  SET(BUILDNAME "${BUILDNAME}-gtk")
ENDIF(VISP_HAVE_GTK)
# GDI (Windows Graphics Device Interface)
IF(VISP_HAVE_GDI)
  SET(BUILDNAME "${BUILDNAME}-gdi")
ENDIF(VISP_HAVE_GDI)
# D3D (Direct3D9)
IF(VISP_HAVE_D3D9)
  SET(BUILDNAME "${BUILDNAME}-Direct3D")
ENDIF(VISP_HAVE_D3D9)
# OpenCV
IF(VISP_HAVE_OPENCV)
  SET(BUILDNAME "${BUILDNAME}-OpenCV")
ENDIF(VISP_HAVE_OPENCV)

#---- Mathematics ----
# GSL (Gnu Scientific Library
IF(VISP_HAVE_GSL)
  SET(BUILDNAME "${BUILDNAME}-gsl")
ENDIF(VISP_HAVE_GSL)

#---- Simulator ----
# Coin
IF(VISP_HAVE_COIN)
  SET(BUILDNAME "${BUILDNAME}-Coin")
ENDIF(VISP_HAVE_COIN)
# SoQt
IF(VISP_HAVE_SOQT)
  SET(BUILDNAME "${BUILDNAME}-SoQt")
ENDIF(VISP_HAVE_SOQT)
# SoWin
IF(VISP_HAVE_SOWIN)
  SET(BUILDNAME "${BUILDNAME}-SoWin")
ENDIF(VISP_HAVE_SOWIN)
# SoXt
IF(VISP_HAVE_SOXT)
  SET(BUILDNAME "${BUILDNAME}-SoXt")
ENDIF(VISP_HAVE_SOXT)

#---- Misc ----
# XML
IF(VISP_HAVE_XML2)
  SET(BUILDNAME "${BUILDNAME}-xml")
ENDIF(VISP_HAVE_XML2)
# PThread
IF(VISP_HAVE_PTHREAD)
  SET(BUILDNAME "${BUILDNAME}-pthread")
ENDIF(VISP_HAVE_PTHREAD)

#MESSAGE("BUILDNAME=${BUILDNAME}")
