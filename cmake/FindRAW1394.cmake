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
# Try to find libraw1394 for IEEE1394 camera under Linux.
# Once run this will define:
#
# RAW1394_FOUND
# RAW1394_INCLUDE_DIRS
# RAW1394_LIBRARIES
#
#############################################################################


if(NOT UNIX)
  # message("FindRAW1394.cmake: libraw1394 only available for Unix.")
  set(RAW1394_FOUND FALSE)
else()
  set(RAW1394_SEARCH_HINTS
    ${RAW1394_HOME}
    $ENV{RAW1394_HOME}
  )

  find_path(RAW1394_INCLUDE_DIR libraw1394/raw1394.h
    HINTS ${RAW1394_SEARCH_HINTS}
    PATH_SUFFIXES include
  )
  #message("DBG RAW1394_INCLUDE_DIR=${RAW1394_INCLUDE_DIR}")

  find_library(RAW1394_LIBRARY
    NAMES raw1394
    HINTS ${RAW1394_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )
  #message("DBG RAW1394_LIBRARY=${RAW1394_LIBRARY}")

  if(RAW1394_LIBRARY AND RAW1394_INCLUDE_DIR)
    set(RAW1394_INCLUDE_DIRS ${RAW1394_INCLUDE_DIR})
    set(RAW1394_LIBRARIES ${RAW1394_LIBRARY})
    set(RAW1394_FOUND TRUE)
  else()
    set(RAW1394_FOUND FALSE)
  endif()

  mark_as_advanced(
    RAW1394_INCLUDE_DIR
    RAW1394_LIBRARY
  )
endif()
