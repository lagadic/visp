#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2015 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
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
# Try to find libpng library.
# To use the libpng library, the zlib library is required.
# Once run this will define: 
#
# PNG_FOUND
# PNG_INCLUDE_DIR
# PNG_LIBRARIES
#
# Authors:
# Nicolas Melchior
#
#############################################################################


# detection of the Libpng headers location
if(MINGW)
  find_path(PNG_INCLUDE_DIR 
    NAMES
      png.h
    PATHS
      "C:/mingw/include/libpng14"
      "$ENV{MINGW_DIR}/include/libpng14"
  )
else()
  find_path(PNG_INCLUDE_DIR 
    NAMES
      png.h
    PATHS
      "$ENV{LIBPNG_DIR}/include"
      "$ENV{LIBPNG_DIR}"
      "$ENV{LIBPNG_INCLUDE_DIR}"
      "/usr/include"
      "/usr/local/include"
      "C:/Program Files/libpng/include"
  )
endif()
#MESSAGE("PNG_INCLUDE_DIR=${PNG_INCLUDE_DIR}")

if(UNIX)
  # Detection of the Libpng library on Unix
  find_library(PNG_LIBRARY
    NAMES
      png15 libpng15 libpng14 png12 libpng12 png libpng
    PATHS
      "$ENV{LIBPNG_DIR}/lib"
      "$ENV{LIBPNG_DIR}/Release"
      "$ENV{LIBPNG_DIR}"
      "$ENV{LIBPNG_LIBRARY_DIR}"
      /usr/lib
      /usr/local/lib
      /lib
      "C:/Program Files/libpng/lib"
    )
elseif(MINGW)
  # Detection of the Libpng library on mingw
  find_library(PNG_LIBRARY
    NAMES
      png15 libpng15 libpng14 png12 libpng12 png libpng
    PATHS
      "C:/mingw/lib64"
      "$ENV{MINGW_DIR}/lib64"
    )
  #MESSAGE("PNG_LIBRARY=${PNG_LIBRARY}")
else()
  FIND_LIBRARY(PNG_LIBRARY_RELEASE
    NAMES
      png15 libpng15 png12 libpng12 png libpng
    PATHS
      $ENV{LIBPNG_DIR}/lib
      $ENV{LIBPNG_DIR}/Release
      $ENV{LIBPNG_DIR}
      $ENV{LIBPNG_LIBRARY_RELEASE_DIR}
      /usr/lib
      /usr/local/lib
      /lib
      "C:/Program Files/libpng/lib"
    )
  FIND_LIBRARY(PNG_LIBRARY_DEBUG
    NAMES
      png15d libpng15d png12d libpng12d pngd libpngd
    PATHS
      $ENV{LIBPNG_DIR}/lib
      $ENV{LIBPNG_DIR}/Debug
      $ENV{LIBPNG_DIR}
      $ENV{LIBPNG_LIBRARY_DEBUG_DIR}
      /usr/lib
      /usr/local/lib
      /lib
      "C:/Program Files/libpng/lib"
    )
  #MESSAGE("PNG_LIBRARY_RELEASE=${PNG_LIBRARY_RELEASE}")
  #MESSAGE("PNG_LIBRARY_DEBUG=${PNG_LIBRARY_DEBUG}")
ENDIF(UNIX)
## --------------------------------

SET(PNG_FOUND FALSE)

FIND_PACKAGE(ZLIB)
if(NOT ZLIB_FOUND)
  FIND_PACKAGE(MyZLIB)    
endif()

IF(UNIX)
  IF(PNG_LIBRARY AND PNG_INCLUDE_DIR)
    SET(PNG_INCLUDE_DIRS ${PNG_INCLUDE_DIR})
    SET(PNG_LIBRARIES ${PNG_LIBRARY})
    SET(PNG_FOUND TRUE)
  ENDIF()
ELSE(UNIX)
  SET(PNG_LIBRARIES "")
  IF(PNG_LIBRARY_RELEASE AND PNG_INCLUDE_DIR)
    SET(PNG_INCLUDE_DIRS ${PNG_INCLUDE_DIR})
    LIST(APPEND PNG_LIBRARIES optimized ${PNG_LIBRARY_RELEASE})
    SET(PNG_FOUND TRUE)
  ENDIF()
  IF(PNG_LIBRARY_DEBUG AND PNG_INCLUDE_DIR)
    SET(PNG_INCLUDE_DIRS ${PNG_INCLUDE_DIR})
    LIST(APPEND PNG_LIBRARIES debug ${PNG_LIBRARY_DEBUG})
    SET(PNG_FOUND TRUE)
  ENDIF()
ENDIF(UNIX)	  

IF(ZLIB_FOUND)
 IF(PNG_FOUND)
  # The material is found. Check if it works on the requested architecture
  include(CheckCXXSourceCompiles)
	
  #MESSAGE(PNG_LIBRARIES: ${PNG_LIBRARIES})
  #MESSAGE(ZLIB_LIBRARIES: ${ZLIB_LIBRARIES})

  SET(CMAKE_REQUIRED_LIBRARIES ${PNG_LIBRARIES} ${ZLIB_LIBRARIES})
  SET(CMAKE_REQUIRED_INCLUDES ${PNG_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS}) 
  CHECK_CXX_SOURCE_COMPILES("
    #include <png.h> // Contrib for png image io
    int main()
    {
      /* create a png read struct */
      png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
    }
    " PNG_BUILD_TEST) 
  #MESSAGE("PNG_BUILD_TEST: ${PNG_BUILD_TEST}")
  IF(PNG_BUILD_TEST)
    SET(PNG_FOUND TRUE)
  ELSE()
    SET(PNG_FOUND FALSE)
    MESSAGE("libpng library found but not compatible with architecture.")
  ENDIF()
 ELSEIF(PNG_FOUND)
  MESSAGE("To use libpng library, you should also install zlib library")
 ENDIF()
ENDIF()


MARK_AS_ADVANCED(
  PNG_LIBRARY
  PNG_LIBRARY_DEBUG
  PNG_LIBRARY_RELEASE
  PNG_INCLUDE_DIR
)
