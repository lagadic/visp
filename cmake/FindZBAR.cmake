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
# Try to find zbar library.
# Once run this will define:
#
# ZBAR_FOUND
# ZBAR_INCLUDE_DIRS
# ZBAR_LIBRARIES
# ZBAR_VERSION
#
#############################################################################


set(ZBAR_SEARCH_HINTS
  ${ZBAR_DIR}
  $ENV{ZBAR_DIR}
)
set(ZBAR_SEARCH_PATHS "")

if(MSVC)
  list(APPEND ZBAR_SEARCH_PATHS
    "C:/ZBarWin64"
    "C:/workspace/ZBarWin64"
    "C:/zbar/ZBarWin64"
  )
  if(CMAKE_CL_64)
    set(_zbar_lib_dir x64)
  else()
    set(_zbar_lib_dir .)
  endif()
endif()

find_path(ZBAR_INCLUDE_DIR zbar.h
  HINTS ${ZBAR_SEARCH_HINTS}
  PATHS ${ZBAR_SEARCH_PATHS}
  PATH_SUFFIXES include
)

if(MSVC)
  find_library(ZBAR_LIBRARIES_OPT
    NAMES libzbar64-0
    HINTS ${ZBAR_SEARCH_HINTS}
    PATHS ${ZBAR_SEARCH_PATHS}
    PATH_SUFFIXES ${_zbar_lib_dir} ${_zbar_lib_dir}/Release
  )
  find_library(ZBAR_LIBRARIES_DBG
    NAMES libzbar64-0
    HINTS ${ZBAR_SEARCH_HINTS}
    PATHS ${ZBAR_SEARCH_PATHS}
    PATH_SUFFIXES ${_zbar_lib_dir} ${_zbar_lib_dir}/Debug
  )

  if(ZBAR_LIBRARIES_OPT AND ZBAR_LIBRARIES_DBG)
    set(ZBAR_LIBRARIES optimized ${ZBAR_LIBRARIES_OPT} debug ${ZBAR_LIBRARIES_DBG})
  elseif(ZBAR_LIBRARIES_OPT)
    set(ZBAR_LIBRARIES ${ZBAR_LIBRARIES_OPT})
  elseif(ZBAR_LIBRARIES_DBG)
    set(ZBAR_LIBRARIES ${ZBAR_LIBRARIES_DBG})
  endif()

  mark_as_advanced(ZBAR_LIBRARIES_OPT ZBAR_LIBRARIES_DBG)
else()
  find_library(ZBAR_LIBRARY
    NAMES zbar
    HINTS ${ZBAR_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )
  if(ZBAR_LIBRARY)
    set(ZBAR_LIBRARIES ${ZBAR_LIBRARY})
  endif()
  mark_as_advanced(ZBAR_LIBRARY)
endif()

if(ZBAR_INCLUDE_DIR AND ZBAR_LIBRARIES)
  set(ZBAR_INCLUDE_DIRS ${ZBAR_INCLUDE_DIR})
  set(ZBAR_FOUND TRUE)
  if(MSVC)
    vp_parse_header("${ZBAR_INCLUDE_DIR}/config.h" ZBAR_VERSION_LINES LIB_VERSION_MAJOR LIB_VERSION_MINOR LIB_VERSION_REVISION)
    set(ZBAR_VERSION "${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_REVISION}")
  else()
    get_filename_component(ZBAR_LIB_DIR ${ZBAR_LIBRARY} PATH)
    vp_get_version_from_pkg("zbar" "${ZBAR_LIB_DIR}/pkgconfig" ZBAR_VERSION)
  endif()
else()
  set(ZBAR_FOUND FALSE)
endif()

mark_as_advanced(ZBAR_INCLUDE_DIR)
