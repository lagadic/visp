#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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
# Try to find Intel RealSense SDK 2.0 (librealsense2).
# Once run this will define:
#
# REALSENSE2_FOUND
# REALSENSE2_INCLUDE_DIRS
# REALSENSE2_LIBRARIES
# REALSENSE2_VERSION
#
#############################################################################


set(REALSENSE2_SEARCH_HINTS
  ${REALSENSE2_HOME}
  $ENV{REALSENSE2_HOME}
  ${REALSENSE2_DIR}
  $ENV{REALSENSE2_DIR}
)
set(REALSENSE2_SEARCH_PATHS "")
set(REALSENSE2_LIB_SUFFIXES lib)

if(MSVC)
  list(APPEND REALSENSE2_SEARCH_PATHS
    "C:/librealsense2"
    "C:/Program Files (x86)/Intel RealSense SDK 2.0"
    "C:/Program Files (x86)/RealSense SDK 2.0"
  )
  if(CMAKE_CL_64)
    list(INSERT REALSENSE2_LIB_SUFFIXES 0 lib/x64)
  else()
    list(INSERT REALSENSE2_LIB_SUFFIXES 0 lib/x86)
  endif()
endif()

find_path(REALSENSE2_INCLUDE_DIR librealsense2/rs.hpp
  HINTS ${REALSENSE2_SEARCH_HINTS}
  PATHS ${REALSENSE2_SEARCH_PATHS}
  PATH_SUFFIXES include
)

find_library(REALSENSE2_LIBRARY
  NAMES realsense2
  HINTS ${REALSENSE2_SEARCH_HINTS}
  PATHS ${REALSENSE2_SEARCH_PATHS}
  PATH_SUFFIXES ${REALSENSE2_LIB_SUFFIXES}
)

if(REALSENSE2_LIBRARY AND REALSENSE2_INCLUDE_DIR)
  set(REALSENSE2_INCLUDE_DIRS ${REALSENSE2_INCLUDE_DIR})
  set(REALSENSE2_LIBRARIES ${REALSENSE2_LIBRARY})
  set(REALSENSE2_FOUND TRUE)
  vp_parse_header("${REALSENSE2_INCLUDE_DIR}/librealsense2/rs.h" REALSENSE2_VERSION_LINES RS2_API_MAJOR_VERSION RS2_API_MINOR_VERSION RS2_API_PATCH_VERSION)
  set(REALSENSE2_VERSION "${RS2_API_MAJOR_VERSION}.${RS2_API_MINOR_VERSION}.${RS2_API_PATCH_VERSION}")
else()
  set(REALSENSE2_FOUND FALSE)
endif()

mark_as_advanced(
  REALSENSE2_INCLUDE_DIR
  REALSENSE2_LIBRARY
)
