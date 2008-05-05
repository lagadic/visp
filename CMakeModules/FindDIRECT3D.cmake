#############################################################################
#
# $Id: FindDIRECT3D.cmake,v 1.6 2008-05-05 14:34:42 fspindle Exp $
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
# Try to find Direct3D on Windows. This code comes originaly
# from http://www.cmake.org/Wiki/CMakeUserFindDirectShow .
# Once loaded this will define:
#
# DIRECT3D_FOUND        - system has DirectShow
# DIRECT3D_INCLUDE_DIR  - include directory for DirectShow
# DIRECT3D_LIBRARIES    - libraries you need to link to
#
# Authors:
# Bruno Renier
#
#############################################################################

SET(DIRECT3D_FOUND "NO")

# Direct3D is only available on Windows platforms
IF(WIN32 AND NOT MINGW)
  # Find Direct3D Include Directory
  FIND_PATH(DIRECT3D_INCLUDE_DIR d3dx9.h
 	"$ENV{DXSDK_DIR}/Include"
    	"C:/Program Files/Microsoft DirectX SDK/Include"
    	"C:/DXSDK/Include"
   	NO_DEFAULT_PATH
   	DOC "What is the path where the file d3dx9.h can be found"
  )
  FIND_PATH(DIRECT3D_INCLUDE_DIR d3dx9.h
   	NO_DEFAULT_PATH
   	DOC "What is the path where the file d3dx9.h can be found"
  )

    # if Direct3D include dir found, then find Direct3D libraries
    IF(DIRECT3D_INCLUDE_DIR)
      IF(CMAKE_CL_64)
	FIND_LIBRARY(DIRECT3D_d3d9_LIBRARY d3d9
	  "$ENV{DXSDK_DIR}/Lib"	
	  "$ENV{DXSDK_DIR}/Lib/x64"
	  "C:/Program Files/Microsoft DirectX SDK/Lib/x64"
        "C:/DXSDK/Include/Lib/x64"
	  NO_DEFAULT_PATH
        DOC "Where can the Direct3D d3d9 library be found"
      )
	ELSE(CMAKE_CL_64)
	FIND_LIBRARY(DIRECT3D_d3d9_LIBRARY d3d9
	  "$ENV{DXSDK_DIR}/Lib"	
	  "$ENV{DXSDK_DIR}/Lib/x86"
	  "C:/Program Files/Microsoft DirectX SDK/Lib/x86"
        "C:/DXSDK/Include/Lib/x86"
	  NO_DEFAULT_PATH
        DOC "Where can the Direct3D d3d9 library be found"
      )
	ENDIF(CMAKE_CL_64)
      FIND_LIBRARY(DIRECT3D_d3d9_LIBRARY d3d9
        DOC "Where can the Direct3D d3d9 library be found"
      )

	IF(CMAKE_CL_64)
      FIND_LIBRARY(DIRECT3D_d3dx9_LIBRARY d3dx9
	  "$ENV{DXSDK_DIR}/Lib/"
	  "$ENV{DXSDK_DIR}/Lib/x64"
	  "C:/Program Files/Microsoft DirectX SDK/Lib/x64"
        "C:/DXSDK/Include/Lib/x64"
	  NO_DEFAULT_PATH
        DOC "Where can the Direct3D d3dx9 library be found"
      )
	ELSE(CMAKE_CL_64)
      FIND_LIBRARY(DIRECT3D_d3dx9_LIBRARY d3dx9
	  "$ENV{DXSDK_DIR}/Lib/"
	  "$ENV{DXSDK_DIR}/Lib/x86"
	  "C:/Program Files/Microsoft DirectX SDK/Lib/x86"
        "C:/DXSDK/Include/Lib/x86"
	  NO_DEFAULT_PATH
        DOC "Where can the Direct3D d3dx9 library be found"
      )
	ENDIF(CMAKE_CL_64)	 
      FIND_LIBRARY(DIRECT3D_d3dx9_LIBRARY d3dx9
        DOC "Where can the Direct3D d3dx9 library be found"
      )

      # if Direct3D libraries found, then we're ok
      IF(DIRECT3D_d3d9_LIBRARY)
      IF(DIRECT3D_d3dx9_LIBRARY)
        # everything found
        SET(DIRECT3D_FOUND "YES")
      ENDIF(DIRECT3D_d3dx9_LIBRARY)
      ENDIF(DIRECT3D_d3d9_LIBRARY)
    ENDIF(DIRECT3D_INCLUDE_DIR)

  MARK_AS_ADVANCED(
    DIRECT3D_INCLUDE_DIR
    DIRECT3D_d3d9_LIBRARY
    DIRECT3D_d3dx9_LIBRARY
  )


ENDIF(WIN32 AND NOT MINGW)


#---------------------------------------------------------------------
IF(DIRECT3D_FOUND)

  SET(DIRECT3D_LIBRARIES
    "${DIRECT3D_d3d9_LIBRARY}"
    "${DIRECT3D_d3dx9_LIBRARY}"
  )
ELSE(DIRECT3D_FOUND)
  # make FIND_PACKAGE friendly
  IF(NOT DIRECT3D_FIND_QUIETLY)
    IF(DIRECT3D_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR
              "Direct3D required, please specify it's location.")
    ENDIF(DIRECT3D_FIND_REQUIRED)
  ENDIF(NOT DIRECT3D_FIND_QUIETLY)
ENDIF(DIRECT3D_FOUND)
