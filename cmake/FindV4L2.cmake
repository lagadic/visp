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
# Try to find linux/videodev.h for Video For Linux Two framegrabbing
# capabilities.
# Once run this will define:
#
# V4L2_FOUND
# V4L2_INCLUDE_DIRS
# V4L2_LIBRARIES
# V4L2_VERSION
#
#############################################################################


if(NOT UNIX)
  # message("FindV4L2.cmake: only available for Unix.")
  set(V4L2_FOUND FALSE)
else()
  set(V4L2_SEARCH_HINTS
    ${V4L2_HOME}
    $ENV{V4L2_HOME}
    ${V4L2_DIR}
    $ENV{V4L2_DIR}
  )

  find_path(V4L2_INCLUDE_VIDEODEV2 linux/videodev2.h
    HINTS ${V4L2_SEARCH_HINTS}
    PATH_SUFFIXES include
  )
  #message("DBG V4L2_INCLUDE_VIDEODEV2=${V4L2_INCLUDE_VIDEODEV2}")

  find_path(V4L2_INCLUDE_LIBV4L2 libv4l2.h
    HINTS ${V4L2_SEARCH_HINTS}
    PATH_SUFFIXES include
  )
  #message("DBG V4L2_INCLUDE_LIBV4L2=${V4L2_INCLUDE_LIBV4L2}")

  find_library(V4L2_LIBRARY_LIBV4L2
    NAMES v4l2
    HINTS ${V4L2_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )

  find_library(V4L2_LIBRARY_LIBV4LCONVERT
    NAMES v4lconvert
    HINTS ${V4L2_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )

  if(V4L2_INCLUDE_VIDEODEV2 AND V4L2_INCLUDE_LIBV4L2 AND V4L2_LIBRARY_LIBV4L2 AND V4L2_LIBRARY_LIBV4LCONVERT)
    set(V4L2_INCLUDE_DIRS ${V4L2_INCLUDE_VIDEODEV2} ${V4L2_INCLUDE_LIBV4L2})
    set(V4L2_LIBRARIES ${V4L2_LIBRARY_LIBV4L2} ${V4L2_LIBRARY_LIBV4LCONVERT})
    set(V4L2_FOUND TRUE)

    get_filename_component(V4L2_LIB_DIR ${V4L2_LIBRARY_LIBV4L2} PATH)
    vp_get_version_from_pkg("libv4l2" "${V4L2_LIB_DIR}/pkgconfig" V4L2_VERSION)
  else()
    set(V4L2_FOUND FALSE)
  endif()

  mark_as_advanced(
    V4L2_INCLUDE_VIDEODEV2
    V4L2_INCLUDE_LIBV4L2
    V4L2_LIBRARY_LIBV4L2
    V4L2_LIBRARY_LIBV4LCONVERT
  )
endif()
