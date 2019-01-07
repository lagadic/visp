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
# Try to find libjpeg library.
# Once run this will define: 
#
# JPEG_FOUND
# JPEG_INCLUDE_DIRS
# JPEG_LIBRARIES
#
# Authors:
# Nicolas Melchior
#
#############################################################################

# detection of the Libjpeg headers location
if(MINGW)
  find_path(JPEG_INCLUDE_DIR 
    NAMES
    jpeglib.h
    PATHS
    "$ENV{MINGW_DIR}/include"
    C:/mingw/include
    )
  # Detection of the Libjpeg library on Unix
  find_library(JPEG_LIBRARY
    NAMES
    jpeg libjpeg
    PATHS
    "$ENV{MINGW_DIR}/lib"
    "$ENV{MINGW_DIR}/lib64"
    C:/mingw/lib64
    )
else()
  find_path(JPEG_INCLUDE_DIR 
    NAMES
    jpeglib.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{LIBJPEG_DIR}/include
    $ENV{LIBJPEG_DIR}
    "C:/Program Files/GnuWin32/include"
    )
  # Detection of the Libjpeg library on Unix
  find_library(JPEG_LIBRARY
    NAMES
    jpeg libjpeg
    PATHS
    /usr/lib
    /usr/local/lib
    /lib
    $ENV{LIBJPEG_DIR}/lib
    $ENV{LIBJPEG_DIR}/Release
    $ENV{LIBJPEG_DIR}
    "C:/Program Files/GnuWin32/lib"
    )
endif()
  #MESSAGE("JPEG_INCLUDE_DIR=${JPEG_INCLUDE_DIR}")
  #MESSAGE("JPEG_LIBRARY=${JPEG_LIBRARY}")


  MARK_AS_ADVANCED(
    JPEG_LIBRARY
    JPEG_INCLUDE_DIR
  )
  
## --------------------------------
  
IF(JPEG_LIBRARY AND JPEG_INCLUDE_DIR)
    # The material is found. Check if it works on the requested architecture
    include(CheckCXXSourceCompiles)
	
    SET(CMAKE_REQUIRED_LIBRARIES ${JPEG_LIBRARY})
    SET(CMAKE_REQUIRED_INCLUDES ${JPEG_INCLUDE_DIR})
    CHECK_CXX_SOURCE_COMPILES("
      #include <stdio.h>  
      #include <jpeglib.h> // Contrib for jpeg image io
      #include <jerror.h>  
      int main()
      {
        struct jpeg_decompress_struct cinfo;
        struct jpeg_error_mgr jerr;
        cinfo.err = jpeg_std_error(&jerr);
      }
      " JPEG_BUILD_TEST) 
    #MESSAGE("JPEG_BUILD_TEST: ${JPEG_BUILD_TEST}")
    IF(JPEG_BUILD_TEST)
      SET(JPEG_INCLUDE_DIRS ${JPEG_INCLUDE_DIR})
      SET(JPEG_LIBRARIES  ${JPEG_LIBRARY})
      SET(JPEG_FOUND TRUE)
    ELSE()
      SET(JPEG_FOUND FALSE)
      #MESSAGE("libjpeg library found but not compatible with architecture.")
    ENDIF() 

ELSE(JPEG_LIBRARY AND JPEG_INCLUDE_DIR)
  SET(JPEG_FOUND FALSE)
ENDIF(JPEG_LIBRARY AND JPEG_INCLUDE_DIR)
