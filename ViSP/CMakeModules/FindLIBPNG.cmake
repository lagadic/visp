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
# Try to find libpng library.
# To use the libpng library, the zlib library is required.
# Once run this will define: 
#
# LIBPNG_FOUND
# LIBPNG_INCLUDE_DIRS
# LIBPNG_LIBRARIES
#
# Authors:
# Nicolas Melchior
#
#############################################################################


# detection of the Libpng headers location
FIND_PATH(LIBPNG_INCLUDE_DIR 
  NAMES
    png.h
  PATHS
    $ENV{LIBPNG_DIR}/include
    $ENV{LIBPNG_DIR}
    $ENV{LIBPNG_INCLUDE_DIR}
    "/usr/include"
    "/usr/local/include"
    "C:/Program Files/libpng/include"
  )
#MESSAGE("LIBPNG_INCLUDE_DIR=${LIBPNG_INCLUDE_DIR}")

IF(UNIX)
  # Detection of the Libpng library on Unix
  FIND_LIBRARY(LIBPNG_LIBRARY
    NAMES
      png15 libpng15 png12 libpng12 png libpng
    PATHS
      $ENV{LIBPNG_DIR}/lib
      $ENV{LIBPNG_DIR}/Release
      $ENV{LIBPNG_DIR}
      $ENV{LIBPNG_LIBRARY_DIR}
      /usr/lib
      /usr/local/lib
      /lib
      "C:/Program Files/libpng/lib"
    )
  #MESSAGE("LIBPNG_LIBRARY=${LIBPNG_LIBRARY}")
ELSE(UNIX)
  FIND_LIBRARY(LIBPNG_LIBRARY_RELEASE
    NAMES
      png15 libpng15 png12 libpng12 png libpng
    PATHS
      $ENV{LIBPNG_DIR}/lib
      $ENV{LIBPNG_DIR}/Release
      $ENV{LIBPNG_DIR}
      $ENV{LIBPNG_LIBRARY_DIR}
      /usr/lib
      /usr/local/lib
      /lib
      "C:/Program Files/libpng/lib"
    )
  FIND_LIBRARY(LIBPNG_LIBRARY_DEBUG
    NAMES
      png15d libpng15d png12d libpng12d pngd libpngd
    PATHS
      $ENV{LIBPNG_DIR}/lib
      $ENV{LIBPNG_DIR}/Debug
      $ENV{LIBPNG_DIR}
      $ENV{LIBPNG_LIBRARY_DIR}
      /usr/lib
      /usr/local/lib
      /lib
      "C:/Program Files/libpng/lib"
    )
  #MESSAGE("LIBPNG_LIBRARY_RELEASE=${LIBPNG_LIBRARY_RELEASE}")
  #MESSAGE("LIBPNG_LIBRARY_DEBUG=${LIBPNG_LIBRARY_DEBUG}")
ENDIF(UNIX)
## --------------------------------
  
SET(LIBPNG_FOUND FALSE)

FIND_PACKAGE(ZLIB QUIET)
IF(NOT ZLIB_FOUND)
  FIND_PACKAGE(ZLIB2)
ENDIF()

IF(UNIX)
  IF(LIBPNG_LIBRARY AND LIBPNG_INCLUDE_DIR)
    SET(LIBPNG_INCLUDE_DIRS ${LIBPNG_INCLUDE_DIR})
    SET(LIBPNG_LIBRARIES ${LIBPNG_LIBRARY})
    SET(LIBPNG_FOUND TRUE)
  ENDIF()
ELSE(UNIX)
  IF(LIBPNG_LIBRARY_RELEASE AND LIBPNG_INCLUDE_DIR)
    SET(LIBPNG_INCLUDE_DIRS ${LIBPNG_INCLUDE_DIR})
    LIST(APPEND LIBPNG_LIBRARIES optimized)
    LIST(APPEND LIBPNG_LIBRARIES ${LIBPNG_LIBRARY_RELEASE})
    SET(LIBPNG_FOUND TRUE)
  ENDIF()
  IF(LIBPNG_LIBRARY_DEBUG AND LIBPNG_INCLUDE_DIR)
    SET(LIBPNG_INCLUDE_DIRS ${LIBPNG_INCLUDE_DIR})
    LIST(APPEND LIBPNG_LIBRARIES debug)
    LIST(APPEND LIBPNG_LIBRARIES ${LIBPNG_LIBRARY_DEBUG})
    SET(LIBPNG_FOUND TRUE)
  ENDIF()
ENDIF(UNIX)	  

IF(ZLIB_FOUND OR ZLIB2_FOUND)
 IF(LIBPNG_FOUND)
  # The material is found. Check if it works on the requested architecture
  include(CheckCXXSourceCompiles)
	
  #MESSAGE(LIBPNG_LIBRARIES: ${LIBPNG_LIBRARIES})
  #MESSAGE(ZLIB_LIBRARIES: ${ZLIB_LIBRARIES})

  SET(CMAKE_REQUIRED_LIBRARIES ${LIBPNG_LIBRARIES} ${ZLIB_LIBRARIES})
  SET(CMAKE_REQUIRED_INCLUDES ${LIBPNG_INCLUDE_DIRS}) 
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
    SET(LIBPNG_FOUND TRUE)
  ELSE()
    SET(LIBPNG_FOUND FALSE)
    MESSAGE("libpng library found but not compatible with architecture.")
  ENDIF()
 ELSEIF(LIBPNG_FOUND)
  MESSAGE("To use libpng library, you should also install zlib library")
 ENDIF()
ENDIF()


MARK_AS_ADVANCED(
  LIBPNG_LIBRARY
  LIBPNG_LIBRARY_DEBUG
  LIBPNG_LIBRARY_RELEASE
  LIBPNG_INCLUDE_DIR
)
