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
# Try to find zlib library. 
# This file should be used only if FindZLIB.cmake provided with CMake
# is not able to detect zlib.
# The detection can be eased by setting ZLIB_HOME environment variable 
# especially under windows.
#
# ZLIB_FOUND
# ZLIB_INCLUDE_DIRS
# ZLIB_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

if(MINGW)
  find_path(ZLIB_INCLUDE_DIR zlib.h
    "$ENV{MINGW_DIR}/include"
    C:/mingw/include
    )
else()
  find_path(ZLIB_INCLUDE_DIR zlib.h
    $ENV{ZLIB_DIR}/include
    $ENV{ZLIB_INCLUDE_DIR}
    /usr/include
    /usr/local/include
    "C:/Program Files/zlib/include"
    )
endif()

if(UNIX)
  find_library(ZLIB_LIBRARY z zlib
    $ENV{ZLIB_DIR}/lib
    $ENV{ZLIB_LIBRARY_DIR}
    /lib
    /usr/lib
    /usr/local/lib
    "C:/Program Files/zlib/lib"
    )
elseif(MINGW)
  find_library(ZLIB_LIBRARY z zlib
    "$ENV{MINGW_DIR}/lib64"
    C:/mingw/lib64
    )
else()
  find_library(ZLIB_LIBRARY_RELEASE z zlib
    $ENV{ZLIB_DIR}/lib
    $ENV{ZLIB_LIBRARY_RELEASE_DIR}
    /lib
    /usr/lib
    /usr/local/lib
    "C:/Program Files/zlib/lib"
    "$ENV{MINGW_DIR}/lib64"
    C:/mingw/lib64
    )

  FIND_LIBRARY(ZLIB_LIBRARY_DEBUG zlibd
    $ENV{ZLIB_DIR}/lib
    $ENV{ZLIB_LIBRARY_DEBUG_DIR}
    /lib
    /usr/lib
    /usr/local/lib
    "C:/Program Files/zlib/lib"
    )
endif()

if(UNIX OR MINGW)
  if(ZLIB_LIBRARY AND ZLIB_INCLUDE_DIR)
    set(ZLIB_INCLUDE_DIRS ${ZLIB_INCLUDE_DIR})
    set(ZLIB_LIBRARIES ${ZLIB_LIBRARY})
    set(ZLIB_FOUND TRUE)
  else()
    set(ZLIB_FOUND FALSE)
  endif()
else()
  set(ZLIB_LIBRARIES "")
  if(ZLIB_LIBRARY_RELEASE AND ZLIB_INCLUDE_DIR)
    set(ZLIB_INCLUDE_DIRS ${ZLIB_INCLUDE_DIR})
    list(APPEND ZLIB_LIBRARIES optimized)
    list(APPEND ZLIB_LIBRARIES ${ZLIB_LIBRARY_RELEASE})
    set(ZLIB_FOUND TRUE)
  endif()
  if(ZLIB_LIBRARY_DEBUG AND ZLIB_INCLUDE_DIR)
    set(ZLIB_INCLUDE_DIRS ${ZLIB_INCLUDE_DIR})
    list(APPEND ZLIB_LIBRARIES debug)
    list(APPEND ZLIB_LIBRARIES ${ZLIB_LIBRARY_DEBUG})
    set(ZLIB_FOUND TRUE)
  endif()
endif()

mark_as_advanced(
  ZLIB_INCLUDE_DIR
  ZLIB_LIBRARY
  ZLIB_LIBRARY_RELEASE
  ZLIB_LIBRARY_DEBUG
  )




