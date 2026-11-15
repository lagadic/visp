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
# Try to find Aria libraries and headers to control Pioneer mobile robots.
# Once run this will define:
#
# ARIA_FOUND
# ARIA_INCLUDE_DIRS
# ARIA_LIBRARIES
#
#############################################################################

# Layouts handled, relative to each standard prefix (/usr, /usr/local,
# C:/Program Files, ${CMAKE_PREFIX_PATH}, ...):
#   <prefix>/include/Aria.h            (ARIA_HOME or custom prefix)
#   <prefix>/include/Aria/Aria.h       (e.g. /usr/include/Aria)
#   <prefix>/Aria/include/Aria.h       (e.g. /usr/local/Aria/include)
#   <prefix>/MobileRobots/Aria/include (Windows)
set(ARIA_INCLUDE_SUFFIXES
  include
  Aria
  Aria/include
  MobileRobots/Aria/include
)

if(CMAKE_CL_64)
  set(ARIA_LIB_SUFFIXES lib64 Aria/lib64 MobileRobots/Aria/lib64)
else()
  set(ARIA_LIB_SUFFIXES lib Aria/lib MobileRobots/Aria/lib)
endif()

find_path(ARIA_INCLUDE_DIR Aria.h
  HINTS
    ${ARIA_HOME}
    $ENV{ARIA_HOME}
  PATH_SUFFIXES
    ${ARIA_INCLUDE_SUFFIXES}
)
#message("DBG ARIA_INCLUDE_DIR=${ARIA_INCLUDE_DIR}")

if(UNIX)
  find_library(ARIA_LIBRARY
    NAMES Aria
    HINTS
      ${ARIA_HOME}
      $ENV{ARIA_HOME}
    PATH_SUFFIXES
      ${ARIA_LIB_SUFFIXES}
  )
  #message("DBG ARIA_LIBRARY=${ARIA_LIBRARY}")
else()
  if(MSVC12)
    set(ARIA_SUFFIX_NAME VC12)
  elseif(MSVC11)
    set(ARIA_SUFFIX_NAME VC11)
  elseif(MSVC10)
    set(ARIA_SUFFIX_NAME VC10)
  elseif(MSVC90)
    set(ARIA_SUFFIX_NAME VC9)
  elseif(MSVC80)
    set(ARIA_SUFFIX_NAME VC8)
  elseif(MSVC71)
    set(ARIA_SUFFIX_NAME VC71)
  else()
    set(ARIA_SUFFIX_NAME "")
  endif()

  find_library(ARIA_LIBRARY_DEBUG
    NAMES AriaDebug${ARIA_SUFFIX_NAME}
    HINTS
      ${ARIA_HOME}
      $ENV{ARIA_HOME}
    PATH_SUFFIXES
      ${ARIA_LIB_SUFFIXES}
  )
  find_library(ARIA_LIBRARY_RELEASE
    NAMES Aria${ARIA_SUFFIX_NAME}
    HINTS
      ${ARIA_HOME}
      $ENV{ARIA_HOME}
    PATH_SUFFIXES
      ${ARIA_LIB_SUFFIXES}
  )
endif()

if(UNIX)
  if(ARIA_LIBRARY AND ARIA_INCLUDE_DIR)
    set(ARIA_INCLUDE_DIRS ${ARIA_INCLUDE_DIR})
    set(ARIA_LIBRARIES ${ARIA_LIBRARY})
    set(ARIA_FOUND TRUE)
  else()
    set(ARIA_FOUND FALSE)
  endif()
else()
  set(ARIA_LIBRARIES "")
  if(ARIA_LIBRARY_RELEASE AND ARIA_INCLUDE_DIR)
    set(ARIA_INCLUDE_DIRS ${ARIA_INCLUDE_DIR})
    list(APPEND ARIA_LIBRARIES optimized ${ARIA_LIBRARY_RELEASE})
    set(ARIA_FOUND TRUE)
  endif()
  if(ARIA_LIBRARY_DEBUG AND ARIA_INCLUDE_DIR)
    set(ARIA_INCLUDE_DIRS ${ARIA_INCLUDE_DIR})
    list(APPEND ARIA_LIBRARIES debug ${ARIA_LIBRARY_DEBUG})
    set(ARIA_FOUND TRUE)
  endif()
endif()

mark_as_advanced(
  ARIA_INCLUDE_DIR
  ARIA_LIBRARY
  ARIA_LIBRARY_DEBUG
  ARIA_LIBRARY_RELEASE
)
