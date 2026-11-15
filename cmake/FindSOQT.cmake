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
# Try to find SoQt library.
# Once run this will define:
#
# SOQT_FOUND
# SOQT_INCLUDE_DIRS
# SOQT_LIBRARIES
#
#############################################################################


set(SOQT_FOUND FALSE)

# Layouts handled, relative to each standard prefix and to COIN_DIR, SOQT_DIR, COINDIR or SOQTDIR:
#   <prefix>/include, <prefix>/include/Coin2, <prefix>/include/Coin3 (e.g. /usr/include/Coin3)
#   <prefix>/lib
#   /Library/Frameworks/SoQt.framework/Headers and Libraries (macOS)
set(SOQT_SEARCH_HINTS
  ${COIN_DIR}
  $ENV{COIN_DIR}
  ${SOQT_DIR}
  $ENV{SOQT_DIR}
  ${COINDIR}
  $ENV{COINDIR}
  ${SOQTDIR}
  $ENV{SOQTDIR}
)
set(SOQT_SEARCH_PATHS "")
if(APPLE)
  list(APPEND SOQT_SEARCH_PATHS /Library/Frameworks/SoQt.framework)
endif()

# detection of the SoQt headers location
find_path(SOQT_INCLUDE_DIR Inventor/Qt/SoQt.h
  HINTS ${SOQT_SEARCH_HINTS}
  PATHS ${SOQT_SEARCH_PATHS}
  PATH_SUFFIXES include include/Coin2 include/Coin3 Coin2 Coin3 Headers
)
#message("DBG SOQT_INCLUDE_DIR=${SOQT_INCLUDE_DIR}")

if(WIN32)
  # Detection of the SoQt library on Windows
  find_library(SOQT_LIBRARY_RELEASE
    NAMES soqt1 #only shared libraries under windows
    HINTS ${SOQT_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )
  find_library(SOQT_LIBRARY_DEBUG
    NAMES soqt1d #only shared libraries under windows
    HINTS ${SOQT_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )

  if(SOQT_INCLUDE_DIR AND SOQT_LIBRARY_RELEASE)
    set(SOQT_INCLUDE_DIRS ${SOQT_INCLUDE_DIR})
    list(APPEND SOQT_LIBRARIES optimized ${SOQT_LIBRARY_RELEASE})
    set(SOQT_FOUND TRUE)
  endif()

  if(SOQT_INCLUDE_DIR AND SOQT_LIBRARY_DEBUG)
    set(SOQT_INCLUDE_DIRS ${SOQT_INCLUDE_DIR})
    list(APPEND SOQT_LIBRARIES debug ${SOQT_LIBRARY_DEBUG})
    set(SOQT_FOUND TRUE)
  endif()

  mark_as_advanced(
    SOQT_LIBRARY_DEBUG
    SOQT_LIBRARY_RELEASE
    SOQT_INCLUDE_DIR
  )
else()
  # Detection of the SoQt library on Unix
  find_library(SOQT_LIBRARY
    NAMES SoQt
    HINTS ${SOQT_SEARCH_HINTS}
    PATHS ${SOQT_SEARCH_PATHS}
    PATH_SUFFIXES lib Libraries
  )
  #message(STATUS "DBG SOQT_LIBRARY=${SOQT_LIBRARY}")

  if(SOQT_INCLUDE_DIR AND SOQT_LIBRARY)
    set(SOQT_INCLUDE_DIRS ${SOQT_INCLUDE_DIR})
    set(SOQT_LIBRARIES ${SOQT_LIBRARY})
    set(SOQT_FOUND TRUE)
  endif()

  mark_as_advanced(
    SOQT_LIBRARY
    SOQT_INCLUDE_DIR
  )
endif()
