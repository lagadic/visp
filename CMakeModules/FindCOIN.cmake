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
# Try to find Coin library. Try first to see if Coin3D-3 is available. If not,
# check for Coin3D-2.
# Once run this will define: 
#
# COIN_FOUND
# COIN_LIBRARIES
# COIN_INCLUDE_DIR
# COIN_LIBRARY_DEBUG
# COIN_LIBRARY_RELEASE
#
# Authors:
# Fabien Spindler
#
#############################################################################

MACRO(TRY_COMPILE_WITH_COIN COIN_LIB COIN_INC)

  # Try to compile a sample code using Coin release library
  include(CheckCXXSourceCompiles)
	
  SET(CMAKE_REQUIRED_LIBRARIES "${COIN_LIB}")
  SET(CMAKE_REQUIRED_INCLUDES  "${COIN_INC}")
  SET(CMAKE_REQUIRED_DEFINITIONS "-DCOIN_DLL")
  #MESSAGE("COIN_LIB: ${COIN_LIB}")
  #MESSAGE("COIN_INC: ${COIN_INC}")
  CHECK_CXX_SOURCE_COMPILES("
  #include <Inventor/nodes/SoSeparator.h>
  int main(){
    SoSeparator *scene = new SoSeparator;
    return 0;
  }
  " IS_COMPILER_COMPATIBLE) 
  #MESSAGE("DBG1 IS_COMPILER_COMPATIBLE ${IS_COMPILER_COMPATIBLE}")

ENDMACRO(TRY_COMPILE_WITH_COIN)

IF(UNIX OR WIN32) 
  MARK_AS_ADVANCED(
    COIN_INCLUDE_DIR
    COIN_LIBRARY_RELEASE
    COIN_LIBRARY_DEBUG
    COIN_LIBRARY
  )

  FIND_PATH(COIN_INCLUDE_DIR Inventor/nodes/SoSeparator.h
    $ENV{COIN_DIR}/include
    $ENV{COINDIR}/include
    $ENV{COIN_DIR}
    $ENV{COINDIR}
    $ENV{COIN3D_DIR}/include
    $ENV{COIN3DDIR}/include
    /usr/include 
    /usr/include/Coin2
    /Library/Frameworks/Inventor.framework/Headers	
    )
  #MESSAGE("DBG COIN_INCLUDE_DIR=${COIN_INCLUDE_DIR}")

  
  IF(WIN32)
    # Try first to find Coin3D-3 and only if not found Coin3D-2
    FIND_LIBRARY(COIN_LIBRARY_RELEASE
      NAMES coin3 #only shared libraries under windows
      PATHS
      "$ENV{COINDIR}/lib"	  
      "$ENV{COIN_DIR}/lib"	  
      "$ENV{COIN3DDIR}/lib"   
      "$ENV{COIN3D_DIR}/lib"   
      )

    FIND_LIBRARY(COIN_LIBRARY_DEBUG
      NAMES coin3d #only shared libraries under windows
      PATHS
      "$ENV{COINDIR}/lib"	  
      "$ENV{COIN_DIR}/lib"	  
      "$ENV{COIN3DDIR}/lib"   
      "$ENV{COIN3D_DIR}/lib"   
      )

    #MESSAGE("DBG COIN_LIBRARY_RELEASE=${COIN_LIBRARY_RELEASE}")
    #MESSAGE("DBG COIN_LIBRARY_DEBUG=${COIN_LIBRARY_DEBUG}")

  ELSE(WIN32)
    FIND_LIBRARY(COIN_LIBRARY
      NAMES Coin 
      PATHS
      "$ENV{COINDIR}/lib"	  
      "$ENV{COIN_DIR}/lib"	  
      "$ENV{COIN3DDIR}/lib"   
      "$ENV{COIN3D_DIR}/lib"
      /Library/Frameworks/Inventor.framework/Libraries
      )
    
    #MESSAGE("DBG COIN_LIBRARY=${COIN_LIBRARY}")
  ENDIF(WIN32)
  
  ## --------------------------------
  
  IF(COIN_LIBRARY OR COIN_LIBRARY_RELEASE OR COIN_LIBRARY_DEBUG AND COIN_INCLUDE_DIR)

    IF(WIN32)

      IF(COIN_LIBRARY_RELEASE)

	TRY_COMPILE_WITH_COIN(${COIN_LIBRARY_RELEASE} ${COIN_INCLUDE_DIR})

	IF(NOT IS_COMPILER_COMPATIBLE)
	  SET(COIN_LIBRARY_RELEASE FALSE)
	ENDIF(NOT IS_COMPILER_COMPATIBLE)
      ENDIF(COIN_LIBRARY_RELEASE)

      
      # Try to found COIN3D-2
      IF(NOT COIN_LIBRARY_RELEASE)
        FIND_LIBRARY(COIN_LIBRARY_RELEASE
          NAMES coin2 #only shared libraries under windows
          PATHS
          "$ENV{COINDIR}/lib"	  
          "$ENV{COIN_DIR}/lib"	  
          "$ENV{COIN3DDIR}/lib"   
          "$ENV{COIN3D_DIR}/lib"   
        )
        TRY_COMPILE_WITH_COIN(${COIN_LIBRARY_RELEASE} ${COIN_INCLUDE_DIR})

	IF(NOT IS_COMPILER_COMPATIBLE)
	  SET(COIN_LIBRARY_RELEASE FALSE)
	ENDIF(NOT IS_COMPILER_COMPATIBLE)
      ENDIF(NOT COIN_LIBRARY_RELEASE)

      IF(COIN_LIBRARY_DEBUG)

	TRY_COMPILE_WITH_COIN(${COIN_LIBRARY_DEBUG} ${COIN_INCLUDE_DIR})

	IF(NOT IS_COMPILER_COMPATIBLE)
	  SET(COIN_LIBRARY_DEBUG FALSE)
	ENDIF(NOT IS_COMPILER_COMPATIBLE)
      ENDIF(COIN_LIBRARY_DEBUG)

      # Try to found COIN3D-2
      IF(NOT COIN_LIBRARY_DEBUG)
        FIND_LIBRARY(COIN_LIBRARY_DEBUG
          NAMES coin2d #only shared libraries under windows
          PATHS
          "$ENV{COINDIR}/lib"	  
          "$ENV{COIN_DIR}/lib"	  
          "$ENV{COIN3DDIR}/lib"   
          "$ENV{COIN3D_DIR}/lib"   
        )
	TRY_COMPILE_WITH_COIN(${COIN_LIBRARY_DEBUG} ${COIN_INCLUDE_DIR})

	IF(NOT IS_COMPILER_COMPATIBLE)
	  SET(COIN_LIBRARY_DEBUG FALSE)
	ENDIF(NOT IS_COMPILER_COMPATIBLE)

      ENDIF(NOT COIN_LIBRARY_DEBUG)

      IF(NOT COIN_LIBRARY_RELEASE AND NOT COIN_LIBRARY_DEBUG)
	SET(COIN_FOUND FALSE)
      ENDIF(NOT COIN_LIBRARY_RELEASE AND NOT COIN_LIBRARY_DEBUG)
      
    ELSE(WIN32)

      TRY_COMPILE_WITH_COIN(${COIN_LIBRARY} ${COIN_INCLUDE_DIR})

      IF(IS_COMPILER_COMPATIBLE)
	SET(COIN_LIBRARIES ${COIN_LIBRARY})
	SET(COIN_FOUND TRUE)
	MARK_AS_ADVANCED(
	  COIN_LIBRARIES
     	  COIN_LIBRARY
    	  )
      ELSE(IS_COMPILER_COMPATIBLE)
	SET(COIN_FOUND FALSE)
      ENDIF(IS_COMPILER_COMPATIBLE)
    ENDIF(WIN32)
    SET(COIN_FOUND TRUE)
  ELSE(COIN_LIBRARY OR COIN_LIBRARY_RELEASE OR COIN_LIBRARY_DEBUG AND COIN_INCLUDE_DIR)
    SET(COIN_FOUND FALSE)
    #MESSAGE("Coin library not found.")
  ENDIF(COIN_LIBRARY OR COIN_LIBRARY_RELEASE OR COIN_LIBRARY_DEBUG AND COIN_INCLUDE_DIR)

  #MESSAGE(STATUS "COIN_FOUND : ${COIN_FOUND}")

ELSE(UNIX OR WIN32)
  SET(COIN_FOUND FALSE)
ENDIF(UNIX OR WIN32)
