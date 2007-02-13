#############################################################################
#
# $Id: FindDIRECTSHOW.cmake,v 1.2 2007-02-13 09:17:41 fspindle Exp $
#
# Copyright (C) 1998-2006 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
#
# Description:
# Try to find Direct Show on Windows. This code comes originaly
# from http://www.cmake.org/Wiki/CMakeUserFindDirectShow .
# Once loaded this will define:
#
# DIRECTSHOW_FOUND        - system has DirectShow
# DIRECTSHOW_INCLUDE_DIR  - include directory for DirectShow
# DIRECTSHOW_LIBRARIES    - libraries you need to link to
#
# Authors:
# Fabien Spindler
#
#############################################################################

SET(DIRECTSHOW_FOUND "NO")

# DirectShow is only available on Windows platforms
IF(WIN32 AND NOT MINGW)
  # Find DirectX Include Directory
  FIND_PATH(DIRECTX_INCLUDE_DIR ddraw.h
    "C:/DXSDK/include"
    "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Include"
    "C:/Program Files/Microsoft DirectX SDK (February 2006)/Include"
    "C:/Program Files/Microsoft DirectX 9.0 SDK (June 2005)/Include"
    DOC "What is the path where the file ddraw.h can be found"
  )

  # if DirectX found, then find DirectShow include directory
  IF(DIRECTX_INCLUDE_DIR)
    FIND_PATH(DIRECTSHOW_INCLUDE_DIR dshow.h
      "C:/DXSDK/include"
      "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Include"
      "C:/Program Files/Microsoft Platform SDK/Include"
      DOC "What is the path where the file dshow.h can be found"
    )

    # if DirectShow include dir found, then find DirectShow libraries
    IF(DIRECTSHOW_INCLUDE_DIR)
      FIND_LIBRARY(DIRECTSHOW_strmiids_LIBRARY strmiids
        "C:/DXSDK/lib"
        "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Lib"
        "C:/Program Files/Microsoft Platform SDK/Lib"
        DOC "Where can the DirectShow strmiids library be found"
      )
      FIND_LIBRARY(DIRECTSHOW_quartz_LIBRARY quartz
        "C:/DXSDK/lib"
        "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Lib"
        "C:/Program Files/Microsoft Platform SDK/Lib"
        DOC "Where can the DirectShow quartz library be found"
      )

      # if DirectShow libraries found, then we're ok
      IF(DIRECTSHOW_strmiids_LIBRARY)
      IF(DIRECTSHOW_quartz_LIBRARY)
        # everything found
        SET(DIRECTSHOW_FOUND "YES")
      ENDIF(DIRECTSHOW_quartz_LIBRARY)
      ENDIF(DIRECTSHOW_strmiids_LIBRARY)
    ENDIF(DIRECTSHOW_INCLUDE_DIR)
  ENDIF(DIRECTX_INCLUDE_DIR)
  MARK_AS_ADVANCED(
    DIRECTX_INCLUDE_DIR
    DIRECTSHOW_INCLUDE_DIR
    DIRECTSHOW_strmiids_LIBRARY
    DIRECTSHOW_quartz_LIBRARY
  )


ENDIF(WIN32 AND NOT MINGW)


#---------------------------------------------------------------------
IF(DIRECTSHOW_FOUND)
  SET(DIRECTSHOW_INCLUDE_DIR
    ${DIRECTSHOW_INCLUDE_DIR}
    ${DIRECTX_INCLUDE_DIR}
  )

  SET(DIRECTSHOW_LIBRARIES
    ${DIRECTSHOW_strmiids_LIBRARY}
    ${DIRECTSHOW_quartz_LIBRARY}
  )
ELSE(DIRECTSHOW_FOUND)
  # make FIND_PACKAGE friendly
  IF(NOT DIRECTSHOW_FIND_QUIETLY)
    IF(DIRECTSHOW_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR
              "DirectShow required, please specify it's location.")
    ELSE(DIRECTSHOW_FIND_REQUIRED)
      MESSAGE(STATUS "DirectShow was not found.")
    ENDIF(DIRECTSHOW_FIND_REQUIRED)
  ENDIF(NOT DIRECTSHOW_FIND_QUIETLY)
ENDIF(DIRECTSHOW_FOUND)
