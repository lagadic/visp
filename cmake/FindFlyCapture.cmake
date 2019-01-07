#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
# Try to find Cerebellum Irisa library API to dial with the robot Afma6
#
# FLYCAPTURE_FOUND
# FLYCAPTURE_INCLUDE_DIRS
# FLYCAPTURE_LIBRARIES
# FLYCAPTURE_VERSION
#
# Authors:
# Fabien Spindler
#
#############################################################################

set(FLYCAPTURE_INC_SEARCH_PATH /usr/include/flycapture)
set(FLYCAPTURE_LIB_SEARCH_PATH /usr/lib)

set(FLYCAPTURE_VERSION "n/a")

if(MSVC)
  if(CMAKE_CL_64)
    list(APPEND FLYCAPTURE_INC_SEARCH_PATH "C:/Program Files/Point Grey Research/FlyCapture2/include")
    list(APPEND FLYCAPTURE_LIB_SEARCH_PATH "C:/Program Files/Point Grey Research/FlyCapture2/lib64")
  else()
    list(APPEND FLYCAPTURE_INC_SEARCH_PATH "C:/Program Files (x86)/Point Grey Research/FlyCapture2/include")
    list(APPEND FLYCAPTURE_LIB_SEARCH_PATH "C:/Program Files (x86)/Point Grey Research/FlyCapture2/lib")
  endif()
endif()

find_path(FLYCAPTURE_INCLUDE_DIRS FlyCapture2.h
  PATHS
    $ENV{FLYCAPTURE_HOME}/include/flycapture
    ${FLYCAPTURE_INC_SEARCH_PATH}
)

find_library(FLYCAPTURE_LIBRARIES
  NAMES flycapture FlyCapture2
  PATHS 
    $ENV{FLYCAPTURE_HOME}/lib
    ${FLYCAPTURE_LIB_SEARCH_PATH}
)

if(FLYCAPTURE_LIBRARIES AND FLYCAPTURE_INCLUDE_DIRS)
  set(FLYCAPTURE_FOUND TRUE)
  if(WIN32)
    vp_parse_header("${FLYCAPTURE_INCLUDE_DIRS}/FC1/PGRFlyCapture.h" FLYCAPTURE_VERSION_LINES PGRFLYCAPTURE_VERSION)
    set(FLYCAPTURE_VERSION ${PGRFLYCAPTURE_VERSION})
  endif()
else()
  set(FLYCAPTURE_FOUND FALSE)
endif()
  
mark_as_advanced(
  FLYCAPTURE_INCLUDE_DIRS
  FLYCAPTURE_LIBRARIES
  FLYCAPTURE_INC_SEARCH_PATH
  FLYCAPTURE_LIB_SEARCH_PATH
)
