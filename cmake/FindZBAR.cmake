#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
# See http://visp.inria.fr for more information.
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
# Authors:
# Fabien Spindler
#
#############################################################################

set(ZBAR_INC_SEARCH_PATH "")
set(ZBAR_LIB_SEARCH_PATH "")

if(MSVC)
  list(APPEND ZBAR_INC_SEARCH_PATH $ENV{ZBAR_DIR}/include)
  list(APPEND ZBAR_INC_SEARCH_PATH "C:/ZBarWin64/include")
  list(APPEND ZBAR_INC_SEARCH_PATH "C:/workspace/ZBarWin64/include")
  list(APPEND ZBAR_INC_SEARCH_PATH "C:/zbar/ZBarWin64/include")
  
  if(CMAKE_CL_64)
    list(APPEND ZBAR_LIB_SEARCH_PATH $ENV{ZBAR_DIR}/x64)
    list(APPEND ZBAR_LIB_SEARCH_PATH "C:/ZBarWin64/x64")
    list(APPEND ZBAR_LIB_SEARCH_PATH "C:/workspace/ZBarWin64/x64")
    list(APPEND ZBAR_LIB_SEARCH_PATH "C:/zbar/ZBarWin64/x64")
  else()
    list(APPEND ZBAR_LIB_SEARCH_PATH $ENV{ZBAR_DIR})
    list(APPEND ZBAR_LIB_SEARCH_PATH "C:/ZBarWin64")
    list(APPEND ZBAR_LIB_SEARCH_PATH "C:/workspace/ZBarWin64")
    list(APPEND ZBAR_LIB_SEARCH_PATH "C:/zbar/ZBarWin64")
  endif()

else()
  list(APPEND ZBAR_INC_SEARCH_PATH $ENV{ZBAR_DIR}/include)
  list(APPEND ZBAR_INC_SEARCH_PATH /usr/include)
  list(APPEND ZBAR_INC_SEARCH_PATH /usr/local/include)

  list(APPEND ZBAR_LIB_SEARCH_PATH /usr/lib)
  list(APPEND ZBAR_LIB_SEARCH_PATH /usr/local/lib)
endif()

find_path(ZBAR_INCLUDE_DIRS zbar.h
  PATHS
    ${ZBAR_INC_SEARCH_PATH}
)

if(MSVC)
  set(ZBAR_LIB_SEARCH_PATH_OPT ${ZBAR_LIB_SEARCH_PATH})
  set(ZBAR_LIB_SEARCH_PATH_DBG ${ZBAR_LIB_SEARCH_PATH})
  foreach(p ${ZBAR_LIB_SEARCH_PATH})
    list(APPEND ZBAR_LIB_SEARCH_PATH_OPT ${p}/Release)
    list(APPEND ZBAR_LIB_SEARCH_PATH_DBG ${p}/Debug)
  endforeach()
  find_library(ZBAR_LIBRARIES_OPT
    NAMES libzbar64-0
    PATHS
      ${ZBAR_LIB_SEARCH_PATH_OPT}
  )
  find_library(ZBAR_LIBRARIES_DBG
    NAMES libzbar64-0
    PATHS
      ${ZBAR_LIB_SEARCH_PATH_DBG}
  )

  if(ZBAR_LIBRARIES_OPT AND ZBAR_LIBRARIES_DBG)
    set(ZBAR_LIBRARIES optimized ${ZBAR_LIBRARIES_OPT})
    list(APPEND ZBAR_LIBRARIES debug ${ZBAR_LIBRARIES_DBG})
  elseif(ZBAR_LIBRARIES_OPT)
    list(APPEND ZBAR_LIBRARIES ${ZBAR_LIBRARIES_OPT})
  elseif(ZBAR_LIBRARIES_DBG)
    list(APPEND ZBAR_LIBRARIES ${ZBAR_LIBRARIES_DBG})
  endif()

  mark_as_advanced(ZBAR_LIBRARIES_OPT ZBAR_LIBRARIES_DBG)
else()
  find_library(ZBAR_LIBRARIES
    NAMES zbar
    PATHS
      ${ZBAR_LIB_SEARCH_PATH}
  )
endif()

#message("ZBAR_INCLUDE_DIRS: ${ZBAR_INCLUDE_DIRS}")
#message("ZBAR_LIBRARIES: ${ZBAR_LIBRARIES}")
if(ZBAR_INCLUDE_DIRS AND ZBAR_LIBRARIES)
  set(ZBAR_FOUND TRUE)
else()
  set(ZBAR_FOUND FALSE)
endif()

if(ZBAR_FOUND)
  if(MSVC)
    vp_parse_header("${ZBAR_INCLUDE_DIRS}/config.h" ZBAR_VERSION_LINES LIB_VERSION_MAJOR LIB_VERSION_MINOR LIB_VERSION_REVISION)
    set(ZBAR_VERSION "${LIB_VERSION_MAJOR}.${LIB_VERSION_MINOR}.${LIB_VERSION_REVISION}")
  else()
    get_filename_component(ZBAR_LIB_DIR ${ZBAR_LIBRARIES} PATH)
    vp_get_version_from_pkg("zbar" "${ZBAR_LIB_DIR}/pkgconfig" ZBAR_VERSION)
  endif()
endif()

mark_as_advanced(
  ZBAR_INCLUDE_DIRS
  ZBAR_LIBRARIES
)

