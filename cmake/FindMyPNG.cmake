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
# if you have questions regarding the use of this file, please contact
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
# PNG_VERSION_STRING
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
#message("PNG_INCLUDE_DIR=${PNG_INCLUDE_DIR}")

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
  #message("PNG_LIBRARY=${PNG_LIBRARY}")
else()
  find_library(PNG_LIBRARY_RELEASE
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
  find_library(PNG_LIBRARY_DEBUG
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
  #message("PNG_LIBRARY_RELEASE=${PNG_LIBRARY_RELEASE}")
  #message("PNG_LIBRARY_DEBUG=${PNG_LIBRARY_DEBUG}")
endif(UNIX)
## --------------------------------

set(PNG_FOUND FALSE)

find_package(ZLIB)
if(NOT ZLIB_FOUND)
  find_package(MyZLIB)    
endif()

if(UNIX)
  if(PNG_LIBRARY AND PNG_INCLUDE_DIR)
    set(PNG_INCLUDE_DIRS ${PNG_INCLUDE_DIR})
    set(PNG_LIBRARIES ${PNG_LIBRARY})

    get_filename_component(PNG_LIB_DIR ${PNG_LIBRARY} DIRECTORY)
    vp_get_version_from_pkg("libpng" "${PNG_LIB_DIR}/pkgconfig" PNG_VERSION_STRING)

    set(PNG_FOUND TRUE)
	
  endif()
else(UNIX)
  set(PNG_LIBRARIES "")
  if(PNG_LIBRARY_RELEASE AND PNG_INCLUDE_DIR)
    set(PNG_INCLUDE_DIRS ${PNG_INCLUDE_DIR})
    list(APPEND PNG_LIBRARIES optimized ${PNG_LIBRARY_RELEASE})
    set(PNG_FOUND TRUE)
  endif()
  if(PNG_LIBRARY_DEBUG AND PNG_INCLUDE_DIR)
    set(PNG_INCLUDE_DIRS ${PNG_INCLUDE_DIR})
    list(APPEND PNG_LIBRARIES debug ${PNG_LIBRARY_DEBUG})
    set(PNG_FOUND TRUE)
  endif()
endif(UNIX)	  

if(PNG_FOUND)
  vp_parse_header2(PNG "${PNG_INCLUDE_DIR}/png.h" PNG_LIBPNG_VER_STRING)
endif()

if(ZLIB_FOUND)
  if(PNG_FOUND)
    # The material is found. Check if it works on the requested architecture
    include(CheckCXXSourceCompiles)
	
    #message(PNG_LIBRARIES: ${PNG_LIBRARIES})
    #message(ZLIB_LIBRARIES: ${ZLIB_LIBRARIES})

    set(CMAKE_REQUIRED_LIBRARIES ${PNG_LIBRARIES} ${ZLIB_LIBRARIES})
    set(CMAKE_REQUIRED_INCLUDES ${PNG_INCLUDE_DIRS} ${ZLIB_INCLUDE_DIRS}) 
    CHECK_CXX_SOURCE_COMPILES("
      #include <png.h> // Contrib for png image io
      int main()
      {
        /* create a png read struct */
        png_structp png_ptr = png_create_read_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
      }
      " PNG_BUILD_TEST) 
    #message("PNG_BUILD_TEST: ${PNG_BUILD_TEST}")
    if(PNG_BUILD_TEST)
      set(PNG_FOUND TRUE)
    else()
      set(PNG_FOUND FALSE)
      message("libpng library found but not compatible with architecture.")
    endif()
  elseif(PNG_FOUND)
    message("To use libpng library, you should also install zlib library")
  endif()
endif()

mark_as_advanced(
  PNG_LIBRARY
  PNG_LIBRARY_DEBUG
  PNG_LIBRARY_RELEASE
  PNG_INCLUDE_DIR
)
