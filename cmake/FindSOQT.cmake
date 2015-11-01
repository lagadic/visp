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
# Try to find SoQt library.
# Once run this will define: 
#
# SOQT_FOUND
# SOQT_INCLUDE_DIRS
# SOQT_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

set(SOQT_FOUND FALSE)

# detection of the SoQt headers location
FIND_PATH(SOQT_INCLUDE_DIR Inventor/Qt/SoQt.h
  "$ENV{COIN_DIR}/include"
  "$ENV{SOQT_DIR}/include"
  "$ENV{COIN_DIR}"
  "$ENV{SOQT_DIR}"
  "$ENV{COINDIR}/include"
  "$ENV{SOQTDIR}/include"
  /usr/include/Coin2
  /Library/Frameworks/SoQt.framework/Headers
  )
#MESSAGE("DBG SOQT_INCLUDE_DIR=${SOQT_INCLUDE_DIR}")

IF(WIN32)
  # Detection of the SoQt library on Windows
  FIND_LIBRARY(SOQT_LIBRARY_RELEASE
    NAMES soqt1 #only shared libraries under windows
    PATHS
    "$ENV{COIN_DIR}/lib"
    "$ENV{SOQT_DIR}/lib"
    "$ENV{COINDIR}/lib"
    "$ENV{SOQTDIR}/lib"
    )
  FIND_LIBRARY(SOQT_LIBRARY_DEBUG
    NAMES soqt1d #only shared libraries under windows
    PATHS
    "$ENV{COIN_DIR}/lib"
    "$ENV{SOQT_DIR}/lib"
    "$ENV{COINDIR}/lib"
    "$ENV{SOQTDIR}/lib"
    )  

  if(SOQT_INCLUDE_DIR AND SOQT_LIBRARY_RELEASE)
    set(SOQT_INCLUDE_DIRS ${SOQT_INCLUDE_DIR})
    list(APPEND SOQT_LIBRARIES optimized ${SOQT_LIBRARY_RELEASE})
    set(SOQT_FOUND TRUE)
  endif()

  if(SOQT_INCLUDE_DIR AND SOQT_LIBRARY_DEBUG)
    set(SOQT_INCLUDE_DIRS ${SOQT_INCLUDE_DIR})
    list(APPEND SOQT_LIBRARIES debug ${SOQT_LIBRARY_DEBUG})
    set(SOQT_FOUND TRUE)
  endif()

  MARK_AS_ADVANCED(
    SOQT_LIBRARIES
    SOQT_LIBRARY_DEBUG
    SOQT_LIBRARY_RELEASE
    SOQT_INCLUDE_DIR
    )

ELSE(WIN32)
  # Detection of the SoQt library on Unix
  FIND_LIBRARY(SOQT_LIBRARY
    NAMES SoQt
    PATHS
    "$ENV{COIN_DIR}/lib"
    "$ENV{SOQT_DIR}/lib"
    "$ENV{COINDIR}/lib"
    "$ENV{SOQTDIR}/lib"
    /usr/lib
    /usr/local/lib
    /lib
    /Library/Frameworks/SoQt.framework/Libraries
    )

  if(SOQT_INCLUDE_DIR AND SOQT_LIBRARY)
    set(SOQT_INCLUDE_DIRS ${SOQT_INCLUDE_DIR})
    set(SOQT_LIBRARIES ${SOQT_LIBRARY})
    set(SOQT_FOUND TRUE)
  endif()

  MARK_AS_ADVANCED(
    SOQT_LIBRARIES
    SOQT_LIBRARY
    SOQT_INCLUDE_DIR
  )
  #MESSAGE(STATUS "DBG SOQT_LIBRARY=${SOQT_LIBRARY}")

ENDIF(WIN32)
  
#MESSAGE(STATUS "SOQT_FOUND : ${SOQT_FOUND}")
