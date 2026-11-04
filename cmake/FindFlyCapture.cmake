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
# Try to find FLIR FlyCapture SDK
#
# FLYCAPTURE_FOUND
# FLYCAPTURE_INCLUDE_DIRS
# FLYCAPTURE_LIBRARIES
# FLYCAPTURE_VERSION
#
#############################################################################

# Layouts handled, relative to each standard prefix and to FLYCAPTURE_HOME:
#   <prefix>/include/flycapture (e.g. /usr/include/flycapture), <prefix>/lib
#   <prefix>/include, <prefix>/lib or lib64 (Windows FlyCapture2 install)
set(FLYCAPTURE_SEARCH_HINTS
  ${FLYCAPTURE_HOME}
  $ENV{FLYCAPTURE_HOME}
)
set(FLYCAPTURE_SEARCH_PATHS "")
set(FLYCAPTURE_LIB_SUFFIXES lib)

set(FLYCAPTURE_VERSION "n/a")

if(MSVC)
  if(CMAKE_CL_64)
    list(APPEND FLYCAPTURE_SEARCH_PATHS "C:/Program Files/Point Grey Research/FlyCapture2")
    set(FLYCAPTURE_LIB_SUFFIXES lib64)
  else()
    list(APPEND FLYCAPTURE_SEARCH_PATHS "C:/Program Files (x86)/Point Grey Research/FlyCapture2")
  endif()
endif()

find_path(FLYCAPTURE_INCLUDE_DIR FlyCapture2.h
  HINTS ${FLYCAPTURE_SEARCH_HINTS}
  PATHS ${FLYCAPTURE_SEARCH_PATHS}
  PATH_SUFFIXES include/flycapture include flycapture
)

find_library(FLYCAPTURE_LIBRARY
  NAMES flycapture FlyCapture2 FlyCapture2_v100
  HINTS ${FLYCAPTURE_SEARCH_HINTS}
  PATHS ${FLYCAPTURE_SEARCH_PATHS}
  PATH_SUFFIXES ${FLYCAPTURE_LIB_SUFFIXES}
)

if(FLYCAPTURE_LIBRARY AND FLYCAPTURE_INCLUDE_DIR)
  set(FLYCAPTURE_INCLUDE_DIRS ${FLYCAPTURE_INCLUDE_DIR})
  set(FLYCAPTURE_LIBRARIES ${FLYCAPTURE_LIBRARY})
  set(FLYCAPTURE_FOUND TRUE)
else()
  set(FLYCAPTURE_FOUND FALSE)
endif()

mark_as_advanced(
  FLYCAPTURE_INCLUDE_DIR
  FLYCAPTURE_LIBRARY
)
