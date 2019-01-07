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
# Try to find GDI (Graphics Device Interface) on Windows.
# Once loaded this will define:
#
# GDI_FOUND        - system has GDI
# GDI_LIBRARIES    - libraries you need to link to
#
# Authors:
# Fabien Spindler
#
#############################################################################

SET(GDI_FOUND "NO")

# GDI (Graphics Device Interface)  is only available on Windows platforms
IF(WIN32)
  IF(MINGW)
    FIND_LIBRARY(GDI_LIBRARY gdi32
                 "C:/MinGW/lib"
                 "C:/mingw/mingw/lib"
                 "$ENV{MINGW_DIR}/lib"
                 "$ENV{MINGW_DIR}/mingw/lib"
                 DOC "Where can the GDI (Graphics Device Interface) library be found"
                 NO_DEFAULT_PATH
                )
  ELSE(MINGW)
    IF(CMAKE_CL_64)
      # Generic path search
      FIND_LIBRARY(GDI_LIBRARY gdi32
                   "$ENV{WINSDK_DIR}/Lib/x64"
                   "$ENV{WINSDK_HOME}/Lib/x64"
                   "$ENV{DXSDK_DIR}/Lib/x64"
                   "C:/Program Files/Microsoft SDKs/Windows/v6.0/Lib/x64"
                   "C:/Program Files/Microsoft SDKs/Windows/v6.0a/Lib/x64"
                   "C:/Program Files/Microsoft SDKs/Windows/v6.1/Lib/x64"
                   "C:/Program Files/Microsoft SDKs/Windows/v7.0A/Lib/x64"
                   "C:/Program Files/Microsoft SDKs/Windows/v7.1/Lib/x64"
                   "C:/Program Files/Microsoft SDKs/Windows/v7.1A/Lib/x64"
                   "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.0A/Lib/x64"
                   "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.1A/Lib/x64"
                   "C:/Program Files (x86)/Windows Kits/8.0/Lib/win8/um/x64"
                   "C:/Program Files (x86)/Windows Kits/8.1/Lib/winv6.3/um/x64"
                   "C:/Program Files (x86)/Windows Kits/10/Lib/10.0.15063.0/um/x64"
                   "C:/Program Files (x86)/Windows Kits/10/Lib/10.0.14393.0/um/x86"
                   "C:/Program Files/Microsoft Platform SDK/Lib/x64"
                   "C:/DXSDK/Include/Lib/x64"
                   DOC "Where can the GDI (Graphics Device Interface) library be found"
                  )

      # Specific path search for Visual Studio .NET 2003
      IF(MSVC71)
        IF(NOT GDI_LIBRARY)
          FIND_LIBRARY(GDI_LIBRARY gdi32
                       "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Lib"
                      )
        ENDIF(NOT GDI_LIBRARY)
      ENDIF(MSVC71)

      # Specific path search for Visual Studio 2005
      IF(MSVC80)
        IF(NOT GDI_LIBRARY)
          FIND_LIBRARY(GDI_LIBRARY gdi32
                       "C:/Program Files/Microsoft Visual Studio 8/VC/PlatformSDK/Lib/AMD64"         )
        ENDIF(NOT GDI_LIBRARY)
      ENDIF(MSVC80)

      # Specific path search for Visual Studio 2008
      IF(MSVC90)
        IF(NOT GDI_LIBRARY)
          FIND_LIBRARY(GDI_LIBRARY gdi32
                       "C:/Program Files/Microsoft Visual Studio 9/VC/PlatformSDK/Lib/AMD64"         )
        ENDIF(NOT GDI_LIBRARY)
      ENDIF(MSVC90)

    ELSE(CMAKE_CL_64)
      # Generic path search
      FIND_LIBRARY(GDI_LIBRARY gdi32
                   "$ENV{WINSDK_DIR}/Lib"
                   "$ENV{WINSDK_HOME}/Lib"
                   "$ENV{DXSDK_DIR}/Lib"
                   "C:/Program Files/Microsoft SDKs/Windows/v6.0a/Lib"
                   "C:/Program Files/Microsoft SDKs/Windows/v6.0/Lib"
                   "C:/Program Files/Microsoft SDKs/Windows/v6.1/Lib"
                   "C:/Program Files/Microsoft SDKs/Windows/v7.0A/Lib"
                   "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.0A/Lib"
                   "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.1/Lib"
                   "C:/Program Files (x86)/Microsoft SDKs/Windows/v7.1A/Lib"
                   "C:/Program Files (x86)/Windows Kits/8.0/Lib/win8/um/x86"
                   "C:/Program Files (x86)/Windows Kits/8.1/Lib/winv6.3/um/x86"
                   "C:/Program Files (x86)/Windows Kits/10/Lib/10.0.15063.0/um/x86"
                   "C:/Program Files (x86)/Windows Kits/10/Lib/10.0.14393.0/um/x86"
                   "C:/Program Files/Microsoft Platform SDK/Lib"
                   "C:/DXSDK/Include/Lib"
                   DOC "Where can the GDI (Graphics Device Interface) library be found"
                  )

      # Specific path search for Visual Studio .NET 2003
      IF(MSVC71)
        IF(NOT GDI_LIBRARY)
          FIND_LIBRARY(GDI_LIBRARY gdi32
                       "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Lib"
                      )
        ENDIF(NOT GDI_LIBRARY)
      ENDIF(MSVC71)

      # Specific path search for Visual Studio 2005
      IF(MSVC80)
        IF(NOT GDI_LIBRARY)
          FIND_LIBRARY(GDI_LIBRARY gdi32
                       "C:/Program Files/Microsoft Visual Studio 8/VC/PlatformSDK/Lib"
                      )
        ENDIF(NOT GDI_LIBRARY)
      ENDIF(MSVC80)

      # Specific path search for Visual Studio 2008
      IF(MSVC90)
        IF(NOT GDI_LIBRARY)
          FIND_LIBRARY(GDI_LIBRARY gdi32
                       "C:/Program Files/Microsoft Visual Studio 9/VC/PlatformSDK/Lib"
                      )
        ENDIF(NOT GDI_LIBRARY)
      ENDIF(MSVC90)
    ENDIF(CMAKE_CL_64)
  ENDIF(MINGW)
  # if GDI libraries found, then we're ok
  IF(GDI_LIBRARY)
    SET(GDI_FOUND "YES")
    SET(GDI_LIBRARIES ${GDI_LIBRARY})

  ENDIF(GDI_LIBRARY)

  MARK_AS_ADVANCED(GDI_LIBRARY)
ENDIF(WIN32)
