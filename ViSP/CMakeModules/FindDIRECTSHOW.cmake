#############################################################################
#
# $Id: FindDIRECTSHOW.cmake,v 1.9 2008-06-27 14:38:36 asaunier Exp $
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
# find DirectX
  FIND_PATH(DIRECTX_INCLUDE_PATH ddraw.h
    "$ENV{DXSDK_DIR}/Include"
    "C:/Program Files/Microsoft SDKs/Windows/v6.1/Include"
    "C:/Program Files/Microsoft SDKs/Windows/v6.0a/Include"
    "C:/Program Files/Microsoft SDKs/Windows/v6.0/Include"
    "C:/DXSDK/include"
    "C:/Program Files/Microsoft Platform SDK/Include"
    "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Include"
    NO_DEFAULT_PATH
    DOC "What is the path where the file ddraw.h can be found"
    )
  FIND_PATH(DIRECTX_INCLUDE_PATH ddraw.h
    DOC "What is the path where the file ddraw.h can be found"
    )

  # find DirectShow include directory
  FIND_PATH(DIRECTSHOW_dshow_INCLUDE_PATH dshow.h
    "$ENV{WINSDK_HOME}/Include"
    "$ENV{DXSDK_DIR}/Include"
    "C:/Program Files/Microsoft SDKs/Windows/v6.1/Include"
    "C:/Program Files/Microsoft SDKs/Windows/v6.0a/Include"
    "C:/Program Files/Microsoft SDKs/Windows/v6.0/Include"
    "C:/DXSDK/include"
    "C:/Program Files/Microsoft Platform SDK/Include"
    "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Include"
    NO_DEFAULT_PATH
    DOC "What is the path where the file dshow.h can be found"
    )
  FIND_PATH(DIRECTSHOW_dshow_INCLUDE_PATH dshow.h
    DOC "What is the path where the file dshow.h can be found"
    )
    
  FIND_PATH(DIRECTSHOW_qedit_INCLUDE_PATH qedit.h
    "$ENV{WINSDK_HOME}/Include"
    "$ENV{DXSDK_DIR}/Include"
    "C:/Program Files/Microsoft SDKs/Windows/v6.1/Include"
    "C:/Program Files/Microsoft SDKs/Windows/v6.0a/Include"
    "C:/Program Files/Microsoft SDKs/Windows/v6.0/Include"
    "C:/DXSDK/include"
    "C:/Program Files/Microsoft Platform SDK/Include"
    "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Include"
    NO_DEFAULT_PATH
    DOC "What is the path where the file qedit.h can be found"
    )
  FIND_PATH(DIRECTSHOW_qedit_INCLUDE_PATH qedit.h
    DOC "What is the path where the file qedit.h can be found"
    )
    
  FIND_PATH(DIRECTSHOW_atlbase_INCLUDE_PATH atlbase.h
    DOC "What is the path where the file atlbase.h can be found"
    )
# Specific path search for Visual Studio .NET 2003
      IF(MSVC71)
        IF(NOT DIRECTSHOW_atlbase_INCLUDE_PATH)
          FIND_PATH(DIRECTSHOW_atlbase_INCLUDE_PATH atlbase.h
                       "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/atlmfc/include"
                       "$ENV{VS71COMNTOOLS}/../../Vc7/atlmfc/include"
                      DOC "What is the path where the file atlbase.h can be found"
                    )
        ENDIF(NOT DIRECTSHOW_atlbase_INCLUDE_PATH)
      ENDIF(MSVC71)

# Specific path search for Visual Studio 2005
      IF(MSVC80)
        IF(NOT DIRECTSHOW_atlbase_INCLUDE_PATH)
          FIND_PATH(DIRECTSHOW_atlbase_INCLUDE_PATH atlbase.h
                       "C:/Program Files/Microsoft Visual Studio 8/VC/atlmfc/include"
                       "$ENV{VS80COMNTOOLS}/../../VC/atlmfc/include"
                      DOC "What is the path where the file atlbase.h can be found"
                    )
        ENDIF(NOT DIRECTSHOW_atlbase_INCLUDE_PATH)
      ENDIF(MSVC80)

# Specific path search for Visual Studio 2008
      IF(MSVC90)
        IF(NOT DIRECTSHOW_atlbase_INCLUDE_PATH)
          FIND_PATH(DIRECTSHOW_atlbase_INCLUDE_PATH atlbase.h
                       "C:/Program Files/Microsoft Visual Studio 9.0/VC/atlmfc/include"
                       "$ENV{VS90COMNTOOLS}/../../VC/atlmfc/include"
                      DOC "What is the path where the file atlbase.h can be found"
                    )
        ENDIF(NOT DIRECTSHOW_atlbase_INCLUDE_PATH)
      ENDIF(MSVC90)
  IF(DIRECTX_INCLUDE_PATH AND DIRECTSHOW_dshow_INCLUDE_PATH AND DIRECTSHOW_qedit_INCLUDE_PATH AND DIRECTSHOW_atlbase_INCLUDE_PATH)
    SET(DIRECTSHOW_INCLUDE_DIR ${DIRECTX_INCLUDE_PATH}
                               ${DIRECTSHOW_dshow_INCLUDE_PATH}
                               ${DIRECTSHOW_qedit_INCLUDE_PATH}
                               ${DIRECTSHOW_atlbase_INCLUDE_PATH})
  ENDIF(DIRECTX_INCLUDE_PATH AND DIRECTSHOW_dshow_INCLUDE_PATH AND DIRECTSHOW_qedit_INCLUDE_PATH AND DIRECTSHOW_atlbase_INCLUDE_PATH)
  # if DirectShow include dir found, then find DirectShow libraries
  IF(DIRECTSHOW_INCLUDE_DIR)
    IF(CMAKE_CL_64)
      FIND_LIBRARY(DIRECTSHOW_strmiids_LIBRARY
        NAMES strmiids
        PATHS
        "$ENV{WINSDK_HOME}/Lib/x64"
        "$ENV{DXSDK_DIR}/Lib/x64"
        "C:/Program Files/Microsoft SDKs/Windows/v6.1/Lib/x64"
        "C:/Program Files/Microsoft SDKs/Windows/v6.0a/Lib/x64"
        "C:/Program Files/Microsoft SDKs/Windows/v6.0/Lib/x64"
        "C:/DXSDK/lib/x64"
      	"C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Lib/x64"
        "C:/Program Files/Microsoft Platform SDK/Lib/x64"
	       NO_DEFAULT_PATH
        DOC "Where can the DirectShow strmiids library be found"
	   )
    ELSE(CMAKE_CL_64)
      FIND_LIBRARY(DIRECTSHOW_strmiids_LIBRARY 
        NAMES strmiids
        PATHS
        "$ENV{WINSDK_HOME}/Lib"
        "$ENV{WINSDK_HOME}/Lib/x86"
        "$ENV{DXSDK_DIR}/Lib"
        "$ENV{DXSDK_DIR}/Lib/x86"
        "C:/Program Files/Microsoft SDKs/Windows/v6.1/Lib"
        "C:/Program Files/Microsoft SDKs/Windows/v6.1/Lib/x86"
        "C:/Program Files/Microsoft SDKs/Windows/v6.0a/Lib"
        "C:/Program Files/Microsoft SDKs/Windows/v6.0a/Lib/x86"
        "C:/Program Files/Microsoft SDKs/Windows/v6.0/Lib"
        "C:/Program Files/Microsoft SDKs/Windows/v6.0/Lib/x86"
        "C:/DXSDK/lib"
        "C:/DXSDK/lib/x64"
        "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Lib"
        "C:/Program Files/Microsoft Platform SDK/Lib"
	       NO_DEFAULT_PATH
        DOC "Where can the DirectShow strmiids library be found"
	   )    
    ENDIF(CMAKE_CL_64)  
    FIND_LIBRARY(DIRECTSHOW_strmiids_LIBRARY
      NAMES strmiids
      DOC "Where can the DirectShow strmiids library be found"
      )
    

    IF(CMAKE_CL_64)
      FIND_LIBRARY(DIRECTSHOW_quartz_LIBRARY
        NAMES quartz
        PATHS
        "$ENV{WINSDK_HOME}/Lib/x64"
        "$ENV{DXSDK_DIR}/Lib/x64"
        "C:/DXSDK/lib/x64"
        "C:/Program Files/Microsoft SDKs/Windows/v6.1/Lib/x64"
        "C:/Program Files/Microsoft SDKs/Windows/v6.0a/Lib/x64"
        "C:/Program Files/Microsoft SDKs/Windows/v6.0/Lib/x64"
        "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Lib/x64"
        "C:/Program Files/Microsoft Platform SDK/Lib/x64"
         NO_DEFAULT_PATH
        DOC "Where can the DirectShow quartz library be found"
      )
    ELSE(CMAKE_CL_64)
      FIND_LIBRARY(DIRECTSHOW_quartz_LIBRARY
        NAMES quartz
        PATHS
        "$ENV{WINSDK_HOME}/Lib/x86"
        "$ENV{DXSDK_DIR}/Lib/x86"
        "C:/DXSDK/lib/x86"
        "C:/Program Files/Microsoft SDKs/Windows/v6.1/Lib/x86"
        "C:/Program Files/Microsoft SDKs/Windows/v6.0a/Lib/x86"
        "C:/Program Files/Microsoft SDKs/Windows/v6.0/Lib/x86"
        "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Lib/x86"
        "C:/Program Files/Microsoft Platform SDK/Lib/x86"
        "$ENV{WINSDK_HOME}/Lib"
        "$ENV{DXSDK_DIR}/Lib"
        "C:/DXSDK/lib"
        "C:/Program Files/Microsoft SDKs/Windows/v6.1/Lib"
        "C:/Program Files/Microsoft SDKs/Windows/v6.0a/Lib"
        "C:/Program Files/Microsoft SDKs/Windows/v6.0/Lib"
        "C:/Program Files/Microsoft Visual Studio .NET 2003/Vc7/PlatformSDK/Lib"
        "C:/Program Files/Microsoft Platform SDK/Lib"
        NO_DEFAULT_PATH
        DOC "Where can the DirectShow quartz library be found"
	     )
    ENDIF(CMAKE_CL_64)
    FIND_LIBRARY(DIRECTSHOW_quartz_LIBRARY
      NAMES quartz
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

  MARK_AS_ADVANCED(
    DIRECTSHOW_INCLUDE_DIR
    DIRECTSHOW_strmiids_LIBRARY
    DIRECTSHOW_quartz_LIBRARY
    DIRECTX_INCLUDE_PATH
    DIRECTSHOW_dshow_INCLUDE_PATH
    DIRECTSHOW_qedit_INCLUDE_PATH
    DIRECTSHOW_atlbase_INCLUDE_PATH
    )

ENDIF(WIN32 AND NOT MINGW)


#---------------------------------------------------------------------
IF(DIRECTSHOW_FOUND)
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
    ENDIF(DIRECTSHOW_FIND_REQUIRED)
  ENDIF(NOT DIRECTSHOW_FIND_QUIETLY)
ENDIF(DIRECTSHOW_FOUND)
