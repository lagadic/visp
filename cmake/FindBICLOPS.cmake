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
# Try to find libBiclops, libPMD and libUtils for Biclops head.
# Once run this will define: 
#
# BICLOPS_FOUND
# BICLOPS_INCLUDE_DIRS
# BICLOPS_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################


IF(NOT UNIX AND NOT WIN32)
  # MESSAGE("FindBICLOPS.cmake: macro only for Unix and Windows for the moment.")
  SET(BICLOPS_FOUND FALSE)
ELSE(NOT UNIX AND NOT WIN32)

  FIND_PATH(BICLOPS_INCLUDE_DIR Biclops.h
    $ENV{BICLOPS_HOME}/include
    /usr/include )
  	
  #MESSAGE("DBG BICLOPS_INCLUDE_DIR=${BICLOPS_INCLUDE_DIR}")  

  FIND_PATH(PMD_INCLUDE_DIR PMD.h
    $ENV{BICLOPS_HOME}/include
    $ENV{BICLOPS_PMD_HOME}/include
    /usr/include )

  FIND_PATH(UTILS_INCLUDE_DIR utility.h
    $ENV{BICLOPS_HOME}/include
    $ENV{BICLOPS_UTILS_HOME}/include
    /usr/include )

  FIND_LIBRARY(BICLOPS_LIBRARY
    NAMES Biclops libBiclops libBiclopsD
    PATHS 
    $ENV{BICLOPS_HOME}/lib
    /usr/lib
    )

  FIND_LIBRARY(PMD_LIBRARY
    NAMES PMD libPMD libPMDD
    PATHS 
    $ENV{BICLOPS_HOME}/lib
    $ENV{BICLOPS_PMD_HOME}/lib
    /usr/lib
    )

  FIND_LIBRARY(UTILS_LIBRARY
    NAMES Utils libUtils libUtilsD
    PATHS 
    $ENV{BICLOPS_HOME}/lib
    $ENV{BICLOPS_UTILS_HOME}/lib
    /usr/lib
    )
  #MESSAGE("DBG BICLOPS_LIBRARY=${BICLOPS_LIBRARY}")

  ## --------------------------------

  IF(BICLOPS_LIBRARY AND PMD_LIBRARY AND UTILS_LIBRARY)
    SET(BICLOPS_LIBRARIES ${BICLOPS_LIBRARY} ${PMD_LIBRARY} ${UTILS_LIBRARY})
  ELSE(BICLOPS_LIBRARY AND PMD_LIBRARY AND UTILS_LIBRARY)
#     MESSAGE(SEND_ERROR "Biclops library not found. Set USE_BICLOPS option OFF")
  ENDIF(BICLOPS_LIBRARY AND PMD_LIBRARY AND UTILS_LIBRARY)

  IF(NOT BICLOPS_INCLUDE_DIR)
#     MESSAGE(SEND_ERROR "Biclops include dir not found. Set USE_BICLOPS option OFF")
  ENDIF(NOT BICLOPS_INCLUDE_DIR)
 
  IF(NOT PMD_INCLUDE_DIR)
#     MESSAGE(SEND_ERROR "PMD include dir not found. Set USE_BICLOPS option OFF")
  ENDIF(NOT PMD_INCLUDE_DIR)
  
  IF(NOT UTILS_INCLUDE_DIR)
#     MESSAGE(SEND_ERROR "Utils include dir not found. Set USE_BICLOPS option OFF ")
  ENDIF(NOT UTILS_INCLUDE_DIR)

  IF(BICLOPS_INCLUDE_DIR AND PMD_INCLUDE_DIR AND UTILS_INCLUDE_DIR)
    SET(BICLOPS_INCLUDE_DIRS ${BICLOPS_INCLUDE_DIR} ${PMD_INCLUDE_DIR} ${UTILS_INCLUDE_DIR})
  ENDIF(BICLOPS_INCLUDE_DIR AND PMD_INCLUDE_DIR AND UTILS_INCLUDE_DIR)

  IF(BICLOPS_LIBRARIES AND BICLOPS_INCLUDE_DIR AND PMD_INCLUDE_DIR AND UTILS_INCLUDE_DIR)
    #message("BICLOPS_LIBRARIES: ${BICLOPS_LIBRARIES}")
    #message("BICLOPS_INCLUDE_DIRS: ${BICLOPS_INCLUDE_DIRS}")

    # Try to compile a sample code using Biclops library to see if GetHomedState() is available
    #SET(BICLOPS_HAVE_GET_HOMED_STATE_FUNCTION FALSE)
    include(CheckCXXSourceCompiles)

    SET(CMAKE_REQUIRED_LIBRARIES ${BICLOPS_LIBRARIES})
    SET(CMAKE_REQUIRED_INCLUDES  ${BICLOPS_INCLUDE_DIRS})
    CHECK_CXX_SOURCE_COMPILES("
      #include <PMDUtils.h>
      int main(){
        PMDAxisControl *axis;
        axis->GetHomedState();
        return 0;
      }
     " BICLOPS_HAVE_GET_HOMED_STATE_FUNCTION)

    #MESSAGE("DBG1 BICLOPS_HAVE_GET_HOMED_STATE_FUNCTION ${BICLOPS_HAVE_GET_HOMED_STATE_FUNCTION}")

    SET(BICLOPS_FOUND TRUE)
  ELSE(BICLOPS_LIBRARIES AND BICLOPS_INCLUDE_DIR AND PMD_INCLUDE_DIR AND UTILS_INCLUDE_DIR)
    SET(BICLOPS_FOUND FALSE)
  ENDIF(BICLOPS_LIBRARIES AND BICLOPS_INCLUDE_DIR AND PMD_INCLUDE_DIR AND UTILS_INCLUDE_DIR)

  MARK_AS_ADVANCED(
    BICLOPS_INCLUDE_DIR
    BICLOPS_LIBRARIES
    BICLOPS_LIBRARY
    PMD_INCLUDE_DIR
    UTILS_INCLUDE_DIR
    PMD_LIBRARY
    UTILS_LIBRARY
    )
ENDIF(NOT UNIX AND NOT WIN32)
