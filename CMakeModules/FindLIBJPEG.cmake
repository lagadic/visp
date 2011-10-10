#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
# 
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
# See the file LICENSE.txt at the root directory of this source
# distribution for additional information about the GNU GPL.
#
# For using ViSP with software that can not be combined with the GNU
# GPL, please contact INRIA about acquiring a ViSP Professional 
# Edition License.
#
# See http://www.irisa.fr/lagadic/visp/visp.html for more information.
# 
# This software was developed at:
# INRIA Rennes - Bretagne Atlantique
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# France
# http://www.irisa.fr/lagadic
#
# If you have questions regarding the use of this file, please contact
# INRIA at visp@inria.fr
# 
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Description:
# Try to find libjpeg library.
# Once run this will define: 
#
# LIBJPEG_FOUND
# LIBJPEG_INCLUDE_DIR
# LIBJPEG_LIBRARIES
#
# Authors:
# Nicolas Melchior
#
#############################################################################


# detection of the Libjpeg headers location
  FIND_PATH(LIBJPEG_INCLUDE_PATH 
    NAMES
    jpeglib.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{LIBJPEG_DIR}/include
    $ENV{LIBJPEG_DIR}
    "C:/Program Files/GnuWin32/include"
    )
  #MESSAGE("LIBJPEG_HEADER=${LIBJPEG_INCLUDE_PATH}")

  # Detection of the Libjpeg library on Unix
  FIND_LIBRARY(LIBJPEG_LIBRARY
    NAMES
    jpeg
    PATHS
    /usr/lib
    /usr/local/lib
    /lib
    $ENV{LIBJPEG_DIR}/lib
    $ENV{LIBJPEG_DIR}/Release
    $ENV{LIBJPEG_DIR}
    "C:/Program Files/GnuWin32/lib"
    )
  #MESSAGE("LIBJPEG_LIBRARY=${LIBJPEG_LIBRARY}")


  MARK_AS_ADVANCED(
    LIBJPEG_LIBRARY
    LIBJPEG_INCLUDE_PATH
  )
  
## --------------------------------
  
IF(LIBJPEG_LIBRARY AND LIBJPEG_INCLUDE_PATH)
    # The material is found. Check if it works on the requested architecture
    include(CheckCXXSourceCompiles)
	
    SET(CMAKE_REQUIRED_LIBRARIES ${LIBJPEG_LIBRARY})
    SET(CMAKE_REQUIRED_INCLUDES ${LIBJPEG_INCLUDE_PATH})
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
      " LIBJPEG_BUILD_TEST) 
    #MESSAGE("LIBJPEG_BUILD_TEST: ${LIBJPEG_BUILD_TEST}")
    IF(LIBJPEG_BUILD_TEST)
      SET(LIBJPEG_INCLUDE_DIR ${LIBJPEG_INCLUDE_PATH})
      SET(LIBJPEG_LIBRARIES  ${LIBJPEG_LIBRARY})
      SET(LIBJPEG_FOUND TRUE)
    ELSE()
      SET(LIBJPEG_FOUND FALSE)
      #MESSAGE("libjpeg library found but not compatible with architecture.")
    ENDIF() 

ELSE(LIBJPEG_LIBRARY AND LIBJPEG_INCLUDE_PATH)
  SET(LIBJPEG_FOUND FALSE)
ENDIF(LIBJPEG_LIBRARY AND LIBJPEG_INCLUDE_PATH)
