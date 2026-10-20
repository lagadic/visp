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
# Try to find GDI (Graphics Device Interface) on Windows.
# Once loaded this will define:
#
# GDI_FOUND        - system has GDI
# GDI_LIBRARIES    - libraries you need to link to
#
#############################################################################


set(GDI_FOUND FALSE)

# GDI (Graphics Device Interface) is only available on Windows platforms
if(WIN32)
  if(MINGW)
    set(GDI_SEARCH_HINTS
      ${MINGW_DIR}
      $ENV{MINGW_DIR}
    )
    set(GDI_SEARCH_PATHS
      "C:/mingw64"
      "C:/mingw64/x86_64-w64-mingw32"
      "C:/MinGW"
      "C:/mingw/mingw"
      "C:/mingw32"
      "C:/mingw32/i686-w64-mingw32"
    )
    set(GDI_LIB_SUFFIXES lib)
  else()
    if(CMAKE_CL_64)
      set(_gdi_arch x64)
      set(GDI_LIB_SUFFIXES Lib/x64)
    else()
      set(_gdi_arch x86)
      set(GDI_LIB_SUFFIXES Lib)
    endif()
    # Windows Kits 8.0 and later, most recent SDK first
    file(GLOB _gdi_winkits_path "C:/Program Files (x86)/Windows Kits/*/Lib/*/um/${_gdi_arch}")
    list(REVERSE _gdi_winkits_path)

    set(GDI_SEARCH_HINTS
      ${WINSDK_DIR}
      $ENV{WINSDK_DIR}
      ${WINSDK_HOME}
      $ENV{WINSDK_HOME}
      ${DXSDK_DIR}
      $ENV{DXSDK_DIR}
    )
    set(GDI_SEARCH_PATHS
      "C:/Program Files/Microsoft SDKs/Windows/v6.0"
      "C:/Program Files/Microsoft SDKs/Windows/v6.0a"
      "C:/Program Files/Microsoft SDKs/Windows/v6.1"
      "C:/Program Files/Microsoft SDKs/Windows/v7.0A"
      "C:/Program Files/Microsoft SDKs/Windows/v7.1"
      "C:/Program Files/Microsoft SDKs/Windows/v7.1A"
      "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.0A"
      "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.1"
      "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.1A"
      ${_gdi_winkits_path}
      "C:/Program Files/Microsoft Platform SDK"
      "C:/DXSDK"
    )
  endif()

  find_library(GDI_LIBRARY gdi32
    HINTS ${GDI_SEARCH_HINTS}
    PATHS ${GDI_SEARCH_PATHS}
    PATH_SUFFIXES ${GDI_LIB_SUFFIXES}
    DOC "Where can the GDI (Graphics Device Interface) library be found"
  )

  # if GDI libraries found, then we're ok
  if(GDI_LIBRARY)
    set(GDI_FOUND TRUE)
    set(GDI_LIBRARIES ${GDI_LIBRARY})
  endif()

  mark_as_advanced(GDI_LIBRARY)
endif()
