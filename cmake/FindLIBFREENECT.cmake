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
# Try to find libfreenect library for Kinect sensor
#
# LIBFREENECT_FOUND
# LIBFREENECT_INCLUDE_DIRS
# LIBFREENECT_LIBRARIES
# LIBFREENECT_VERSION
#
#############################################################################


# Layouts handled, relative to each standard prefix and to LIBFREENECT_HOME:
#   <prefix>/include/libfreenect (e.g. /usr/include/libfreenect)
#   <prefix>/lib, <prefix>/build/lib
set(LIBFREENECT_SEARCH_HINTS
  ${LIBFREENECT_HOME}
  $ENV{LIBFREENECT_HOME}
)
set(LIBFREENECT_INC_SEARCH_HINTS
  ${LIBFREENECT_SEARCH_HINTS}
  $ENV{LIBFREENECT_HPP_INCLUDE_DIR}
  $ENV{LIBFREENECT_H_INCLUDE_DIR}
)
set(LIBFREENECT_LIB_SEARCH_HINTS
  ${LIBFREENECT_SEARCH_HINTS}
  $ENV{LIBFREENECT_LIBRARY_DIR}
)
set(LIBFREENECT_SEARCH_PATHS "")
if(WIN32)
  list(APPEND LIBFREENECT_SEARCH_PATHS "C:/libfreenect")
endif()

find_path(LIBFREENECT_HPP_INCLUDE_DIR libfreenect.hpp
  HINTS ${LIBFREENECT_INC_SEARCH_HINTS}
  PATHS ${LIBFREENECT_SEARCH_PATHS}
  PATH_SUFFIXES include include/libfreenect libfreenect
)
find_path(LIBFREENECT_H_INCLUDE_DIR libfreenect.h
  HINTS ${LIBFREENECT_INC_SEARCH_HINTS}
  PATHS ${LIBFREENECT_SEARCH_PATHS}
  PATH_SUFFIXES include include/libfreenect libfreenect
)
find_library(LIBFREENECT_LIBRARY freenect
  HINTS ${LIBFREENECT_LIB_SEARCH_HINTS}
  PATHS ${LIBFREENECT_SEARCH_PATHS}
  PATH_SUFFIXES lib build/lib
)

## --------------------------------
if(LIBFREENECT_LIBRARY AND LIBFREENECT_HPP_INCLUDE_DIR AND LIBFREENECT_H_INCLUDE_DIR)
  set(LIBFREENECT_INCLUDE_DIRS ${LIBFREENECT_HPP_INCLUDE_DIR} ${LIBFREENECT_H_INCLUDE_DIR})
  set(LIBFREENECT_LIBRARIES ${LIBFREENECT_LIBRARY})
  set(LIBFREENECT_FOUND TRUE)

  get_filename_component(LIBFREENECT_LIB_DIR ${LIBFREENECT_LIBRARY} PATH)
  vp_get_version_from_pkg("libfreenect" "${LIBFREENECT_LIB_DIR}/pkgconfig" LIBFREENECT_VERSION)
else()
  set(LIBFREENECT_FOUND FALSE)
endif()

mark_as_advanced(
  LIBFREENECT_HPP_INCLUDE_DIR
  LIBFREENECT_H_INCLUDE_DIR
  LIBFREENECT_LIBRARY
)
