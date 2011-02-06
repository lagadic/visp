#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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
# Try to find libpng library.
# To use the libpng library, the zlib library is required.
# Once run this will define: 
#
# LIBPNG_FOUND
# LIBPNG_INCLUDE_DIR
# LIBPNG_LIBRARIES
#
# Authors:
# Nicolas Melchior
#
#############################################################################


# detection of the Libpng headers location
  FIND_PATH(LIBPNG_INCLUDE_PATH 
    NAMES
    png.h
    PATHS
    "/usr/include"
    "/usr/local/include"
    $ENV{LIBPNG_DIR}/include
    $ENV{LIBPNG_DIR}
    )
  #MESSAGE("LIBPNG_HEADER=${LIBPNG_INCLUDE_PATH}")

  # Detection of the Libpng library on Unix
  FIND_LIBRARY(LIBPNG_LIBRARY
    NAMES
    png libpng
    PATHS
    /usr/lib
    /usr/local/lib
    /lib
    $ENV{LIBPNG_DIR}/lib
    $ENV{LIBPNG_DIR}/Release
    $ENV{LIBPNG_DIR}
    )
  #MESSAGE("LIBPNG_LIBRARY=${LIBPNG_LIBRARY}")


  MARK_AS_ADVANCED(
    LIBPNG_LIBRARY
    LIBPNG_INCLUDE_PATH
  )
  
## --------------------------------
  
IF(LIBPNG_LIBRARY AND LIBPNG_INCLUDE_PATH)
  FIND_PACKAGE(ZLIB)
  IF(ZLIB_FOUND)

    # The material is found. Check if it works on the requested architecture
    include(CheckCXXSourceCompiles)
	
    SET(CMAKE_REQUIRED_LIBRARIES ${LIBPNG_LIBRARY})
    SET(CMAKE_REQUIRED_INCLUDES ${LIBPNG_INCLUDE_DIR})
    CHECK_CXX_SOURCE_COMPILES("
      #include <png.h> // Contrib for png image io
      int main()
      {
        /* create a png read struct */
        png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
      }
      " LIBPNG_BUILD_TEST) 
    #MESSAGE("LIBPNG_BUILD_TEST: ${LIBPNG_BUILD_TEST}")
    IF(LIBPNG_BUILD_TEST)
      SET(LIBPNG_INCLUDE_DIR ${LIBPNG_INCLUDE_PATH})
      SET(LIBPNG_LIBRARIES ${LIBPNG_LIBRARY})
      SET(LIBPNG_FOUND TRUE)
    ELSE()
      SET(LIBPNG_FOUND FALSE)
      #MESSAGE("libpng library found but not compatible with architecture.")
    ENDIF()

    
  ELSE(ZLIB_FOUND)
    MESSAGE("To use the libpng library, the zlib library is required")
  ENDIF(ZLIB_FOUND)

ELSE(LIBPNG_LIBRARY AND LIBPNG_INCLUDE_PATH)
  SET(LIBPNG_FOUND FALSE)
ENDIF(LIBPNG_LIBRARY AND LIBPNG_INCLUDE_PATH)
