#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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

FIND_PATH(ZLIB_INCLUDE_DIR zlib.h
  $ENV{ZLIB_DIR}/include
  $ENV{ZLIB_INCLUDE_DIR}
  /usr/include
  /usr/local/include
  "C:/Program Files/zlib/include"
  )

IF(UNIX)
  FIND_LIBRARY(ZLIB_LIBRARY z zlib
    $ENV{ZLIB_DIR}/lib
    $ENV{ZLIB_LIBRARY_DIR}
    /lib
    /usr/lib
    /usr/local/lib
    "C:/Program Files/zlib/lib"
    )
  #MESSAGE("ZLIB_LIBRARY=${ZLIB_LIBRARY}")
  #MESSAGE("ZLIB_INCLUDE_DIRS=${ZLIB_INCLUDE_DIRS}")
ELSE(UNIX)
  FIND_LIBRARY(ZLIB_LIBRARY_RELEASE zlib
    $ENV{ZLIB_DIR}/lib
    $ENV{ZLIB_LIBRARY_RELEASE_DIR}
    /lib
    /usr/lib
    /usr/local/lib
    "C:/Program Files/zlib/lib"
    )

  FIND_LIBRARY(ZLIB_LIBRARY_DEBUG zlibd
    $ENV{ZLIB_DIR}/lib
    $ENV{ZLIB_LIBRARY_DEBUG_DIR}
    /lib
    /usr/lib
    /usr/local/lib
    "C:/Program Files/zlib/lib"
    )
  #MESSAGE("ZLIB_LIBRARY_RELEASE=${ZLIB_LIBRARY_RELEASE}")
  #MESSAGE("ZLIB_LIBRARY_DEBUG=${ZLIB_LIBRARY_DEBUG}")

ENDIF(UNIX)
## --------------------------------


IF(UNIX)
  IF(ZLIB_LIBRARY AND ZLIB_INCLUDE_DIR)
    SET(ZLIB_INCLUDE_DIRS ${ZLIB_INCLUDE_DIR})
    SET(ZLIB_LIBRARIES ${ZLIB_LIBRARY})
    SET(ZLIB_FOUND TRUE)
    #MESSAGE("ZLIB_LIBRARIES=${ZLIB_LIBRARIES}")
    #MESSAGE("ZLIB_INCLUDE_DIRS=${ZLIB_INCLUDE_DIRS}")
  ELSE()
    SET(ZLIB_FOUND FALSE)
  ENDIF()
ELSE(UNIX)
  SET(ZLIB_LIBRARIES "")
  IF(ZLIB_LIBRARY_RELEASE AND ZLIB_INCLUDE_DIR)
    SET(ZLIB_INCLUDE_DIRS ${ZLIB_INCLUDE_DIR})
    LIST(APPEND ZLIB_LIBRARIES optimized)
    LIST(APPEND ZLIB_LIBRARIES ${ZLIB_LIBRARY_RELEASE})
    SET(ZLIB_FOUND TRUE)
  ENDIF()
  IF(ZLIB_LIBRARY_DEBUG AND ZLIB_INCLUDE_DIR)
    SET(ZLIB_INCLUDE_DIRS ${ZLIB_INCLUDE_DIR})
    LIST(APPEND ZLIB_LIBRARIES debug)
    LIST(APPEND ZLIB_LIBRARIES ${ZLIB_LIBRARY_DEBUG})
    SET(ZLIB_FOUND TRUE)
  ENDIF()
ENDIF(UNIX)

MARK_AS_ADVANCED(
  ZLIB_INCLUDE_DIR
  ZLIB_LIBRARY
  ZLIB_LIBRARY_RELEASE
  ZLIB_LIBRARY_DEBUG
  )




