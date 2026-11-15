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
# Try to find Cerebellum CA linux library material
#
# CALINUX_FOUND
# CALINUX_INCLUDE_DIRS
# CALINUX_LIBRARIES
#
#############################################################################


if(NOT UNIX)
  # message("FindCALINUX.cmake: only available for Unix.")
  set(CALINUX_FOUND FALSE)
else()
  set(CALINUX_SEARCH_HINTS
    ${CALINUX_HOME}
    $ENV{CALINUX_HOME}
  )
  # Inria specific install locations
  set(CALINUX_SEARCH_PATHS
    /local/soft/Cerebellum/CALinux/current
    /home/soft/Cerebellum/CALinux/current
    /home/soft/cerebellum/CALinux
  )

  find_path(CALINUX_INCLUDE_DIR calinux.h
    HINTS ${CALINUX_SEARCH_HINTS}
    PATHS ${CALINUX_SEARCH_PATHS}
    PATH_SUFFIXES include
  )
  #message("DBG CALINUX_INCLUDE_DIR=${CALINUX_INCLUDE_DIR}")

  find_library(CALINUX_LIBRARY
    NAMES calinux
    HINTS ${CALINUX_SEARCH_HINTS}
    PATHS ${CALINUX_SEARCH_PATHS}
    PATH_SUFFIXES lib
  )
  #message("DBG CALINUX_LIBRARY=${CALINUX_LIBRARY}")

  if(CALINUX_LIBRARY AND CALINUX_INCLUDE_DIR)
    set(CALINUX_INCLUDE_DIRS ${CALINUX_INCLUDE_DIR})
    set(CALINUX_LIBRARIES ${CALINUX_LIBRARY})
    set(CALINUX_FOUND TRUE)
  else()
    set(CALINUX_FOUND FALSE)
  endif()

  mark_as_advanced(
    CALINUX_INCLUDE_DIR
    CALINUX_LIBRARIES
    CALINUX_LIBRARY
  )
endif()
