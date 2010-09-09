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
# See the file LICENSE.GPL at the root directory of this source
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
# Try to find SoQt library.
# Once run this will define: 
#
# SOQT_FOUND
# SOQT_INCLUDE_DIR
# SOQT_LIBRARIES
# SOQT_LIBRARY_RELEASE
# SOQT_LIBRARY_DEBUG
#
# Authors:
# Fabien Spindler
#
#############################################################################


# detection of the SoQt headers location
FIND_PATH(SOQT_INCLUDE_PATH Inventor/Qt/SoQt.h
  "$ENV{COIN_DIR}/include"
  "$ENV{SOQT_DIR}/include"
  "$ENV{COIN_DIR}"
  "$ENV{SOQT_DIR}"
  "$ENV{COINDIR}/include"
  "$ENV{SOQTDIR}/include"
  /usr/include/Coin2
  /Library/Frameworks/SoQt.framework/Headers
  )
#MESSAGE("DBG SOQT_INCLUDE_PATH=${SOQT_INCLUDE_PATH}")

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

  MARK_AS_ADVANCED(
    SOQT_LIBRARIES
    SOQT_LIBRARY_DEBUG
    SOQT_LIBRARY_RELEASE
    SOQT_INCLUDE_PATH
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

  MARK_AS_ADVANCED(
    SOQT_LIBRARIES
    SOQT_LIBRARY
    SOQT_INCLUDE_PATH
  )
  #MESSAGE(STATUS "DBG SOQT_LIBRARY=${SOQT_LIBRARY}")

ENDIF(WIN32)
  
## --------------------------------
  
IF(SOQT_LIBRARY OR SOQT_LIBRARY_DEBUG OR SOQT_LIBRARY_RELEASE)
    
  IF(SOQT_INCLUDE_PATH)
    SET(SOQT_INCLUDE_DIR ${SOQT_INCLUDE_PATH})

    IF(WIN32)
      IF(SOQT_LIBRARY_RELEASE AND NOT SOQT_LIBRARY_DEBUG)
        SET(SOQT_LIBRARY_RELEASE ${SOQT_LIBRARY_RELEASE})
      ENDIF(SOQT_LIBRARY_RELEASE AND NOT SOQT_LIBRARY_DEBUG)

      IF(SOQT_LIBRARY_DEBUG AND NOT SOQT_LIBRARY_RELEASE)
        SET(SOQT_LIBRARY_DEBUG ${SOQT_LIBRARY_DEBUG})			 
      ENDIF(SOQT_LIBRARY_DEBUG AND NOT SOQT_LIBRARY_RELEASE)
	
      IF(SOQT_LIBRARY_RELEASE AND SOQT_LIBRARY_DEBUG)
        SET(SOQT_LIBRARY_RELEASE ${SOQT_LIBRARY_RELEASE})
        SET(SOQT_LIBRARY_DEBUG ${SOQT_LIBRARY_DEBUG})
      ENDIF(SOQT_LIBRARY_RELEASE AND SOQT_LIBRARY_DEBUG)

    ELSE(WIN32)
      SET(SOQT_LIBRARIES ${SOQT_LIBRARY})
    ENDIF(WIN32)
    
    SET(SOQT_FOUND TRUE)

  ELSE(SOQT_INCLUDE_PATH)
    SET(SOQT_FOUND FALSE)
    #MESSAGE("Can not find SoQt includes")
  ENDIF(SOQT_INCLUDE_PATH)
   
ELSE(SOQT_LIBRARY OR SOQT_LIBRARY_DEBUG OR SOQT_LIBRARY_RELEASE)
    SET(SOQT_FOUND FALSE)
    #MESSAGE("SoQt library not found.")
ENDIF(SOQT_LIBRARY OR SOQT_LIBRARY_DEBUG OR SOQT_LIBRARY_RELEASE)


#MESSAGE(STATUS "SOQT_FOUND : ${SOQT_FOUND}")
