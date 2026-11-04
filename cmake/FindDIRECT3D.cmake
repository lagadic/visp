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
# Try to find Direct3D on Windows. This code comes originaly
# from http://www.cmake.org/Wiki/CMakeUserFindDirectShow .
# Once loaded this will define:
#
# DIRECT3D_FOUND        - system has Direct3D
# DIRECT3D_INCLUDE_DIRS - include directory for Direct3D
# DIRECT3D_LIBRARIES    - libraries you need to link to
#
#############################################################################


set(DIRECT3D_FOUND FALSE)

# Direct3D is only available on Windows platforms
if(WIN32 AND NOT MINGW)
  set(DIRECT3D_SEARCH_HINTS
    ${DXSDK_DIR}
    $ENV{DXSDK_DIR}
  )
  set(DIRECT3D_SEARCH_PATHS
    "C:/Program Files/Microsoft DirectX SDK"
    "C:/DXSDK"
  )
  if(CMAKE_CL_64)
    set(DIRECT3D_LIB_SUFFIXES Lib/x64 Lib)
  else()
    set(DIRECT3D_LIB_SUFFIXES Lib/x86 Lib)
  endif()

  find_path(DIRECT3D_INCLUDE_DIR d3dx9.h
    HINTS ${DIRECT3D_SEARCH_HINTS}
    PATHS ${DIRECT3D_SEARCH_PATHS}
    PATH_SUFFIXES Include
    NO_DEFAULT_PATH
    DOC "What is the path where the file d3dx9.h can be found"
  )

  # if Direct3D include dir found, then find Direct3D libraries
  if(DIRECT3D_INCLUDE_DIR)
    find_library(DIRECT3D_d3d9_LIBRARY d3d9
      HINTS ${DIRECT3D_SEARCH_HINTS}
      PATHS ${DIRECT3D_SEARCH_PATHS}
      PATH_SUFFIXES ${DIRECT3D_LIB_SUFFIXES}
      DOC "Where can the Direct3D d3d9 library be found"
    )
    find_library(DIRECT3D_d3dx9_LIBRARY d3dx9
      HINTS ${DIRECT3D_SEARCH_HINTS}
      PATHS ${DIRECT3D_SEARCH_PATHS}
      PATH_SUFFIXES ${DIRECT3D_LIB_SUFFIXES}
      DOC "Where can the Direct3D d3dx9 library be found"
    )

    if(DIRECT3D_d3d9_LIBRARY AND DIRECT3D_d3dx9_LIBRARY)
      set(DIRECT3D_FOUND TRUE)
    endif()
  endif()

  mark_as_advanced(
    DIRECT3D_INCLUDE_DIR
    DIRECT3D_d3d9_LIBRARY
    DIRECT3D_d3dx9_LIBRARY
  )
endif()

if(DIRECT3D_FOUND)
  set(DIRECT3D_INCLUDE_DIRS ${DIRECT3D_INCLUDE_DIR})
  set(DIRECT3D_LIBRARIES
    "${DIRECT3D_d3d9_LIBRARY}"
    "${DIRECT3D_d3dx9_LIBRARY}"
  )
elseif(DIRECT3D_FIND_REQUIRED AND NOT DIRECT3D_FIND_QUIETLY)
  message(FATAL_ERROR "Direct3D required, please specify its location.")
endif()
