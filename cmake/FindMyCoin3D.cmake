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

macro(TRY_COMPILE_WITH_COIN COIN3D_LIB COIN3D_INC)

  # Try to compile a sample code using Coin release library
  include(CheckCXXSourceCompiles)
  set(CMAKE_REQUIRED_LIBRARIES "${COIN3D_LIB}")
  set(CMAKE_REQUIRED_INCLUDES  "${COIN3D_INC}")
  set(CMAKE_REQUIRED_DEFINITIONS "-DCOIN_DLL")
  #message("COIN3D_LIB: ${COIN3D_LIB}")
  #message("COIN3D_INC: ${COIN3D_INC}")
  check_cxx_source_compiles("
  #include <Inventor/nodes/SoSeparator.h>
  int main(){
    SoSeparator *scene = new SoSeparator;
    return 0;
  }
  " IS_COMPILER_COMPATIBLE) 
  #message("DBG1 IS_COMPILER_COMPATIBLE ${IS_COMPILER_COMPATIBLE}")
endmacro()

if(UNIX OR WIN32)
  find_path(COIN3D_INCLUDE_DIR Inventor/nodes/SoSeparator.h
    $ENV{COIN_DIR}/include
    $ENV{COINDIR}/include
    $ENV{COIN_DIR}
    $ENV{COINDIR}
    $ENV{COIN3D_DIR}/include
    $ENV{COIN3DDIR}/include
    /usr/include
    /usr/include/Coin2
    /usr/include/Coin3
    /Library/Frameworks/Inventor.framework/Headers
    )
  #message("DBG COIN3D_INCLUDE_DIR=${COIN3D_INCLUDE_DIR}")

  if(WIN32)
    # Try first to find Coin3D-3 and only if not found Coin3D-2
    find_library(COIN3D_LIBRARY_RELEASE
      NAMES coin3 coin4 #only shared libraries under windows
      PATHS
      "$ENV{COINDIR}/lib"
      "$ENV{COIN_DIR}/lib"
      "$ENV{COIN3DDIR}/lib"
      "$ENV{COIN3D_DIR}/lib"
      )

    find_library(COIN3D_LIBRARY_DEBUG
      NAMES coin3d coin4d #only shared libraries under windows
      PATHS
      "$ENV{COINDIR}/lib"
      "$ENV{COIN_DIR}/lib"
      "$ENV{COIN3DDIR}/lib"
      "$ENV{COIN3D_DIR}/lib"
      )

    #MESSAGE("DBG COIN3D_LIBRARY_RELEASE=${COIN3D_LIBRARY_RELEASE}")
    #MESSAGE("DBG COIN3D_LIBRARY_DEBUG=${COIN3D_LIBRARY_DEBUG}")

  else(WIN32)
    find_library(COIN3D_LIBRARY
      NAMES Coin
      PATHS
      "$ENV{COINDIR}/lib"
      "$ENV{COIN_DIR}/lib"
      "$ENV{COIN3DDIR}/lib"
      "$ENV{COIN3D_DIR}/lib"
      /Library/Frameworks/Inventor.framework/Libraries
      )

    #MESSAGE("DBG COIN3D_LIBRARY=${COIN3D_LIBRARY}")
  endif(WIN32)

  mark_as_advanced(
    COIN3D_INCLUDE_DIR
    COIN3D_LIBRARY_RELEASE
    COIN3D_LIBRARY_DEBUG
    COIN3D_LIBRARY
  )
  ## --------------------------------

  if((COIN3D_LIBRARY OR COIN3D_LIBRARY_RELEASE OR COIN3D_LIBRARY_DEBUG) AND COIN3D_INCLUDE_DIR)

    if(WIN32)

      if(COIN3D_LIBRARY_RELEASE)

	TRY_COMPILE_WITH_COIN(${COIN3D_LIBRARY_RELEASE} ${COIN3D_INCLUDE_DIR})

        if(NOT IS_COMPILER_COMPATIBLE)
          set(COIN3D_LIBRARY_RELEASE FALSE)
        endif()
      endif()

      # Try to found COIN3D-2
      if(NOT COIN3D_LIBRARY_RELEASE)
        find_library(COIN3D_LIBRARY_RELEASE
          NAMES coin2 #only shared libraries under windows
          PATHS
          "$ENV{COINDIR}/lib"
          "$ENV{COIN_DIR}/lib"
          "$ENV{COIN3DDIR}/lib"
          "$ENV{COIN3D_DIR}/lib"
        )
        TRY_COMPILE_WITH_COIN(${COIN3D_LIBRARY_RELEASE} ${COIN3D_INCLUDE_DIR})

        if(NOT IS_COMPILER_COMPATIBLE)
          set(COIN3D_LIBRARY_RELEASE FALSE)
        endif()
      endif()

      if(COIN3D_LIBRARY_DEBUG)

	TRY_COMPILE_WITH_COIN(${COIN3D_LIBRARY_DEBUG} ${COIN3D_INCLUDE_DIR})

        if(NOT IS_COMPILER_COMPATIBLE)
          set(COIN3D_LIBRARY_DEBUG FALSE)
        endif()
      endif()

      # Try to found COIN3D-2
      if(NOT COIN3D_LIBRARY_DEBUG)
        find_library(COIN3D_LIBRARY_DEBUG
          NAMES coin2d #only shared libraries under windows
          PATHS
          "$ENV{COINDIR}/lib"
          "$ENV{COIN_DIR}/lib"
          "$ENV{COIN3DDIR}/lib"
          "$ENV{COIN3D_DIR}/lib"
        )
	TRY_COMPILE_WITH_COIN(${COIN3D_LIBRARY_DEBUG} ${COIN3D_INCLUDE_DIR})

        if(NOT IS_COMPILER_COMPATIBLE)
          set(COIN3D_LIBRARY_DEBUG FALSE)
        endif()
      endif()

      if(NOT COIN3D_LIBRARY_RELEASE AND NOT COIN3D_LIBRARY_DEBUG)
        set(COIN3D_FOUND FALSE)
      endif()

    else(WIN32)

      TRY_COMPILE_WITH_COIN(${COIN3D_LIBRARY} ${COIN3D_INCLUDE_DIR})

      if(IS_COMPILER_COMPATIBLE)
        set(COIN3D_LIBRARIES ${COIN3D_LIBRARY})
        set(COIN3D_FOUND TRUE)
        mark_as_advanced(
	  COIN3D_LIBRARIES
     	  COIN3D_LIBRARY
    	  )
      else(IS_COMPILER_COMPATIBLE)
        set(COIN3D_FOUND FALSE)
      endif()
    endif(WIN32)

    set(COIN3D_INCLUDE_DIRS ${COIN3D_INCLUDE_DIR})
    set(COIN3D_FOUND TRUE)
  else((COIN3D_LIBRARY OR COIN3D_LIBRARY_RELEASE OR COIN3D_LIBRARY_DEBUG) AND COIN3D_INCLUDE_DIR)
    set(COIN3D_FOUND FALSE)
    #MESSAGE("Coin library not found.")
  endif((COIN3D_LIBRARY OR COIN3D_LIBRARY_RELEASE OR COIN3D_LIBRARY_DEBUG) AND COIN3D_INCLUDE_DIR)

  #MESSAGE(STATUS "COIN3D_FOUND : ${COIN3D_FOUND}")
  if(COIN3D_FOUND)
    if(COIN3D_LIBRARY_DEBUG AND COIN3D_LIBRARY_RELEASE)
      set(COIN3D_LIBRARIES optimized ${COIN3D_LIBRARY_RELEASE}
                           debug ${COIN3D_LIBRARY_DEBUG})
    else()
      if(COIN3D_LIBRARY_DEBUG)
        set(COIN3D_LIBRARIES ${COIN3D_LIBRARY_DEBUG})
      endif()
      if(COIN3D_LIBRARY_RELEASE)
        set(COIN3D_LIBRARIES ${COIN3D_LIBRARY_RELEASE})
      endif()
    endif()
  endif()

else(UNIX OR WIN32)
  set(COIN3D_FOUND FALSE)
endif(UNIX OR WIN32)
