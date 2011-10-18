#############################################################################
#
# $Id: FindLIBUSB-1.cmake 3376 2011-10-13 12:40:49Z fspindle $
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
# Try to find zlib library. 
# This file should be used only if FindZLIB.cmake provided with CMake
# is not able to detect zlib.
# The detection can be eased by setting ZLIB_HOME environment variable 
# especially under windows.
#
# ZLIB2_FOUND
# ZLIB2_INCLUDE_DIRS
# ZLIB2_LIBRARIES
#
# Authors:
# Fabien Spindler

FIND_PATH(ZLIB2_INCLUDE_DIR zlib.h
  $ENV{ZLIB_DIR}/include
  $ENV{ZLIB_INCLUDE_DIR}
  /usr/include
  /usr/local/include
  "C:/Program Files/zlib/include"
  )

IF(UNIX)
  FIND_LIBRARY(ZLIB2_LIBRARY z zlib
    $ENV{ZLIB_DIR}/lib
    $ENV{ZLIB_LIBRARY_DIR}
    /lib
    /usr/lib
    /usr/local/lib
    "C:/Program Files/zlib/lib"
    )
  #MESSAGE("ZLIB2_LIBRARY=${ZLIB2_LIBRARY}")
  #MESSAGE("ZLIB2_INCLUDE_DIRS=${ZLIB2_INCLUDE_DIRS}")
ELSE(UNIX)
  FIND_LIBRARY(ZLIB2_LIBRARY_RELEASE zlib
    $ENV{ZLIB_DIR}/lib
    $ENV{ZLIB_LIBRARY_RELEASE_DIR}
    /lib
    /usr/lib
    /usr/local/lib
    "C:/Program Files/zlib/lib"
    )

  FIND_LIBRARY(ZLIB2_LIBRARY_DEBUG zlibd
    $ENV{ZLIB_DIR}/lib
    $ENV{ZLIB_LIBRARY_DEBUG_DIR}
    /lib
    /usr/lib
    /usr/local/lib
    "C:/Program Files/zlib/lib"
    )
  #MESSAGE("ZLIB2_LIBRARY_RELEASE=${ZLIB2_LIBRARY_RELEASE}")
  #MESSAGE("ZLIB2_LIBRARY_DEBUG=${ZLIB2_LIBRARY_DEBUG}")

ENDIF(UNIX)
## --------------------------------


IF(UNIX)
  IF(ZLIB2_LIBRARY AND ZLIB2_INCLUDE_DIR)
    SET(ZLIB2_INCLUDE_DIRS ${ZLIB2_INCLUDE_DIR})
    SET(ZLIB2_LIBRARIES ${ZLIB2_LIBRARY})
    SET(ZLIB2_FOUND TRUE)
    #MESSAGE("ZLIB2_LIBRARIES=${ZLIB2_LIBRARIES}")
    #MESSAGE("ZLIB2_INCLUDE_DIRS=${ZLIB2_INCLUDE_DIRS}")
  ELSE()
    SET(ZLIB2_FOUND FALSE)
  ENDIF()
ELSE(UNIX)
  SET(ZLIB2_LIBRARIES "")
  IF(ZLIB2_LIBRARY_RELEASE AND ZLIB2_INCLUDE_DIR)
    SET(ZLIB2_INCLUDE_DIRS ${ZLIB2_INCLUDE_DIR})
    LIST(APPEND ZLIB2_LIBRARIES optimized)
    LIST(APPEND ZLIB2_LIBRARIES ${ZLIB2_LIBRARY_RELEASE})
    SET(ZLIB2_FOUND TRUE)
  ENDIF()
  IF(ZLIB2_LIBRARY_DEBUG AND ZLIB2_INCLUDE_DIR)
    SET(ZLIB2_INCLUDE_DIRS ${ZLIB2_INCLUDE_DIR})
    LIST(APPEND ZLIB2_LIBRARIES debug)
    LIST(APPEND ZLIB2_LIBRARIES ${ZLIB2_LIBRARY_DEBUG})
    SET(ZLIB2_FOUND TRUE)
  ENDIF()
ENDIF(UNIX)

MARK_AS_ADVANCED(
  ZLIB2_INCLUDE_DIR
  ZLIB2_LIBRARY
  ZLIB2_LIBRARY_RELEASE
  ZLIB2_LIBRARY_DEBUG
  )




