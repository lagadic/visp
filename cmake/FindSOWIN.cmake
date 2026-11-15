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
# Try to find SoWin library.
# Once run this will define:
#
# SOWIN_FOUND
# SOWIN_INCLUDE_DIRS
# SOWIN_LIBRARIES
# SOWIN_LIBRARY_RELEASE
# SOWIN_LIBRARY_DEBUG
#
#############################################################################


set(SOWIN_FOUND FALSE)

if(WIN32)
  set(SOWIN_SEARCH_HINTS
    ${COINDIR}
    $ENV{COINDIR}
    ${COIN_DIR}
    $ENV{COIN_DIR}
    ${SOWIN_DIR}
    $ENV{SOWIN_DIR}
  )

  find_library(SOWIN_LIBRARY_RELEASE
    NAMES sowin1 #only shared libraries under windows
    HINTS ${SOWIN_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )
  find_library(SOWIN_LIBRARY_DEBUG
    NAMES sowin1d #only shared libraries under windows
    HINTS ${SOWIN_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )

  find_path(SOWIN_INCLUDE_DIR Inventor/Win/SoWin.h
    HINTS ${SOWIN_SEARCH_HINTS}
    PATH_SUFFIXES include
  )

  set(SOWIN_LIBRARIES "")
  if(SOWIN_LIBRARY_DEBUG)
    list(APPEND SOWIN_LIBRARIES debug ${SOWIN_LIBRARY_DEBUG})
  endif()
  if(SOWIN_LIBRARY_RELEASE)
    list(APPEND SOWIN_LIBRARIES optimized ${SOWIN_LIBRARY_RELEASE})
  endif()

  if(SOWIN_LIBRARIES AND SOWIN_INCLUDE_DIR)
    set(SOWIN_INCLUDE_DIRS ${SOWIN_INCLUDE_DIR})
    set(SOWIN_FOUND TRUE)
  endif()

  mark_as_advanced(
    SOWIN_LIBRARY_DEBUG
    SOWIN_LIBRARY_RELEASE
    SOWIN_INCLUDE_DIR
  )
endif()
