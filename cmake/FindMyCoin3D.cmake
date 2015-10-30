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
# Try to find Coin library. Try first to see if Coin3D-3 is available. If not,
# check for Coin3D-2.
# Once run this will define: 
#
# COIN3D_FOUND
# COIN3D_LIBRARIES
# COIN3D_INCLUDE_DIRS
#
# Authors:
# Fabien Spindler
#
#############################################################################

MACRO(TRY_COMPILE_WITH_COIN COIN3D_LIB COIN3D_INC)

  # Try to compile a sample code using Coin release library
  include(CheckCXXSourceCompiles)
	
  SET(CMAKE_REQUIRED_LIBRARIES "${COIN3D_LIB}")
  SET(CMAKE_REQUIRED_INCLUDES  "${COIN3D_INC}")
  SET(CMAKE_REQUIRED_DEFINITIONS "-DCOIN_DLL")
  #MESSAGE("COIN3D_LIB: ${COIN3D_LIB}")
  #MESSAGE("COIN3D_INC: ${COIN3D_INC}")
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
  FIND_PATH(COIN3D_INCLUDE_DIR Inventor/nodes/SoSeparator.h
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
  #MESSAGE("DBG COIN3D_INCLUDE_DIR=${COIN3D_INCLUDE_DIR}")

  
  IF(WIN32)
    # Try first to find Coin3D-3 and only if not found Coin3D-2
    FIND_LIBRARY(COIN3D_LIBRARY_RELEASE
      NAMES coin3 coin4 #only shared libraries under windows
      PATHS
      "$ENV{COINDIR}/lib"	  
      "$ENV{COIN_DIR}/lib"	  
      "$ENV{COIN3DDIR}/lib"   
      "$ENV{COIN3D_DIR}/lib"   
      )

    FIND_LIBRARY(COIN3D_LIBRARY_DEBUG
      NAMES coin3d coin4d #only shared libraries under windows
      PATHS
      "$ENV{COINDIR}/lib"	  
      "$ENV{COIN_DIR}/lib"	  
      "$ENV{COIN3DDIR}/lib"   
      "$ENV{COIN3D_DIR}/lib"   
      )

    #MESSAGE("DBG COIN3D_LIBRARY_RELEASE=${COIN3D_LIBRARY_RELEASE}")
    #MESSAGE("DBG COIN3D_LIBRARY_DEBUG=${COIN3D_LIBRARY_DEBUG}")

  ELSE(WIN32)
    FIND_LIBRARY(COIN3D_LIBRARY
      NAMES Coin 
      PATHS
      "$ENV{COINDIR}/lib"	  
      "$ENV{COIN_DIR}/lib"	  
      "$ENV{COIN3DDIR}/lib"   
      "$ENV{COIN3D_DIR}/lib"
      /Library/Frameworks/Inventor.framework/Libraries
      )
    
    #MESSAGE("DBG COIN3D_LIBRARY=${COIN3D_LIBRARY}")
  ENDIF(WIN32)
  
  MARK_AS_ADVANCED(
    COIN3D_INCLUDE_DIR
    COIN3D_LIBRARY_RELEASE
    COIN3D_LIBRARY_DEBUG
    COIN3D_LIBRARY
  )
  ## --------------------------------
  
  IF(COIN3D_LIBRARY OR COIN3D_LIBRARY_RELEASE OR COIN3D_LIBRARY_DEBUG AND COIN3D_INCLUDE_DIR)

    IF(WIN32)

      IF(COIN3D_LIBRARY_RELEASE)

	TRY_COMPILE_WITH_COIN(${COIN3D_LIBRARY_RELEASE} ${COIN3D_INCLUDE_DIR})

	IF(NOT IS_COMPILER_COMPATIBLE)
	  SET(COIN3D_LIBRARY_RELEASE FALSE)
	ENDIF(NOT IS_COMPILER_COMPATIBLE)
      ENDIF(COIN3D_LIBRARY_RELEASE)

      
      # Try to found COIN3D-2
      IF(NOT COIN3D_LIBRARY_RELEASE)
        FIND_LIBRARY(COIN3D_LIBRARY_RELEASE
          NAMES coin2 #only shared libraries under windows
          PATHS
          "$ENV{COINDIR}/lib"	  
          "$ENV{COIN_DIR}/lib"	  
          "$ENV{COIN3DDIR}/lib"   
          "$ENV{COIN3D_DIR}/lib"   
        )
        TRY_COMPILE_WITH_COIN(${COIN3D_LIBRARY_RELEASE} ${COIN3D_INCLUDE_DIR})

	IF(NOT IS_COMPILER_COMPATIBLE)
	  SET(COIN3D_LIBRARY_RELEASE FALSE)
	ENDIF(NOT IS_COMPILER_COMPATIBLE)
      ENDIF(NOT COIN3D_LIBRARY_RELEASE)

      IF(COIN3D_LIBRARY_DEBUG)

	TRY_COMPILE_WITH_COIN(${COIN3D_LIBRARY_DEBUG} ${COIN3D_INCLUDE_DIR})

	IF(NOT IS_COMPILER_COMPATIBLE)
	  SET(COIN3D_LIBRARY_DEBUG FALSE)
	ENDIF(NOT IS_COMPILER_COMPATIBLE)
      ENDIF(COIN3D_LIBRARY_DEBUG)

      # Try to found COIN3D-2
      IF(NOT COIN3D_LIBRARY_DEBUG)
        FIND_LIBRARY(COIN3D_LIBRARY_DEBUG
          NAMES coin2d #only shared libraries under windows
          PATHS
          "$ENV{COINDIR}/lib"	  
          "$ENV{COIN_DIR}/lib"	  
          "$ENV{COIN3DDIR}/lib"   
          "$ENV{COIN3D_DIR}/lib"   
        )
	TRY_COMPILE_WITH_COIN(${COIN3D_LIBRARY_DEBUG} ${COIN3D_INCLUDE_DIR})

	IF(NOT IS_COMPILER_COMPATIBLE)
	  SET(COIN3D_LIBRARY_DEBUG FALSE)
	ENDIF(NOT IS_COMPILER_COMPATIBLE)

      ENDIF(NOT COIN3D_LIBRARY_DEBUG)

      IF(NOT COIN3D_LIBRARY_RELEASE AND NOT COIN3D_LIBRARY_DEBUG)
	SET(COIN3D_FOUND FALSE)
      ENDIF(NOT COIN3D_LIBRARY_RELEASE AND NOT COIN3D_LIBRARY_DEBUG)
      
    ELSE(WIN32)

      TRY_COMPILE_WITH_COIN(${COIN3D_LIBRARY} ${COIN3D_INCLUDE_DIR})

      IF(IS_COMPILER_COMPATIBLE)
	SET(COIN3D_LIBRARIES ${COIN3D_LIBRARY})
	SET(COIN3D_FOUND TRUE)
	MARK_AS_ADVANCED(
	  COIN3D_LIBRARIES
     	  COIN3D_LIBRARY
    	  )
      ELSE(IS_COMPILER_COMPATIBLE)
	SET(COIN3D_FOUND FALSE)
      ENDIF(IS_COMPILER_COMPATIBLE)
    ENDIF(WIN32)

    SET(COIN3D_INCLUDE_DIRS ${COIN3D_INCLUDE_DIR})
    SET(COIN3D_FOUND TRUE)
  ELSE(COIN3D_LIBRARY OR COIN3D_LIBRARY_RELEASE OR COIN3D_LIBRARY_DEBUG AND COIN3D_INCLUDE_DIR)
    SET(COIN3D_FOUND FALSE)
    #MESSAGE("Coin library not found.")
  ENDIF(COIN3D_LIBRARY OR COIN3D_LIBRARY_RELEASE OR COIN3D_LIBRARY_DEBUG AND COIN3D_INCLUDE_DIR)

  #MESSAGE(STATUS "COIN3D_FOUND : ${COIN3D_FOUND}")
  IF (COIN3D_FOUND)
    IF (COIN3D_LIBRARY_DEBUG AND COIN3D_LIBRARY_RELEASE)
      SET(COIN3D_LIBRARIES optimized ${COIN3D_LIBRARY_RELEASE}
                           debug ${COIN3D_LIBRARY_DEBUG})
    ELSE (COIN3D_LIBRARY_DEBUG AND COIN3D_LIBRARY_RELEASE)
      IF (COIN3D_LIBRARY_DEBUG)
        SET (COIN3D_LIBRARIES ${COIN3D_LIBRARY_DEBUG})
      ENDIF (COIN3D_LIBRARY_DEBUG)
      IF (COIN3D_LIBRARY_RELEASE)
        SET (COIN3D_LIBRARIES ${COIN3D_LIBRARY_RELEASE})
      ENDIF (COIN3D_LIBRARY_RELEASE)
    ENDIF (COIN3D_LIBRARY_DEBUG AND COIN3D_LIBRARY_RELEASE)
  ENDIF(COIN3D_FOUND)


ELSE(UNIX OR WIN32)
  SET(COIN3D_FOUND FALSE)
ENDIF(UNIX OR WIN32)
