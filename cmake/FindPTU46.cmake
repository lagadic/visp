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
# Try to find libraries for Irisa's Ptu-46 pan-tilt head.
# Once run this will define: 
#
# PTU46_FOUND
# PTU46_INCLUDE_DIRS
# PTU46_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindPTU46.cmake: Ptu-46 only available for Unix.")
  SET(PTU46_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(PTU46_INCLUDE_DIR ptu.h
    $ENV{PTU46_HOME}/include
    /udd/fspindle/robot/Ptu-Evi/current/include
    /local/soft/Ptu-Evi/current/include
    )
  #MESSAGE("DBG PTU46_INCLUDE_DIR=${PTU46_INCLUDE_DIR}")  
  
  FIND_LIBRARY(PTUPTU46_LIBRARY
    NAMES ptu
    PATHS 
    $ENV{PTU46_HOME}/lib
    /udd/fspindle/robot/Ptu-Evi/current/lib
    /local/soft/Ptu-Evi/current/lib
    )
  FIND_LIBRARY(EVIPTU46_LIBRARY
    NAMES evi
    PATHS 
    $ENV{PTU46_HOME}/lib
    /udd/fspindle/robot/Ptu-Evi/current/lib
    /local/soft/Ptu-Evi/current/lib
    )
  FIND_LIBRARY(SERIALPTU46_LIBRARY
    NAMES serial
    PATHS 
    $ENV{PTU46_HOME}/lib
    /udd/fspindle/robot/Ptu-Evi/current/lib
    /local/soft/Ptu-Evi/current/lib
    )
  #MESSAGE("DBG PTU46_LIBRARY=${PTU46_LIBRARY}")
  
  ## --------------------------------
  
  IF(PTUPTU46_LIBRARY AND EVIPTU46_LIBRARY AND SERIALPTU46_LIBRARY)
    # The material is found. Check if it works on the requested architecture
    include(CheckCXXSourceCompiles)
	
    SET(CMAKE_REQUIRED_LIBRARIES ${PTUPTU46_LIBRARY} ${SERIALPTU46_LIBRARY})
    SET(CMAKE_REQUIRED_INCLUDES ${PTU46_INCLUDE_DIR})
    CHECK_CXX_SOURCE_COMPILES("
      #include <ptu.h> // Contrib for Ptu-46 robot
      int main()
      {
        Ptu ptu;
	return 0;
      }
      " PTU46_BUILD_TEST) 
    #MESSAGE("PTU46_BUILD_TEST: ${PTU46_BUILD_TEST}")
    IF(PTU46_BUILD_TEST)
      SET(PTU46_LIBRARIES ${PTUPTU46_LIBRARY} ${EVIPTU46_LIBRARY} 
        ${SERIALPTU46_LIBRARY})
#    ELSE()
#      MESSAGE("Ptu-46 library found but not compatible with architecture.")
    ENDIF()

  ELSE(PTUPTU46_LIBRARY AND EVIPTU46_LIBRARY AND SERIALPTU46_LIBRARY)
#     MESSAGE(SEND_ERROR "Ptu-46 library not found.")
  ENDIF(PTUPTU46_LIBRARY AND EVIPTU46_LIBRARY AND SERIALPTU46_LIBRARY)
  
  IF(NOT PTU46_INCLUDE_DIR)
#     MESSAGE(SEND_ERROR "Ptu-46 include dir not found.")
  ENDIF(NOT PTU46_INCLUDE_DIR)
  
  IF(PTU46_LIBRARIES AND PTU46_INCLUDE_DIR)
    SET(PTU46_INCLUDE_DIRS ${PTU46_INCLUDE_DIR})
    SET(PTU46_FOUND TRUE)
  ELSE(PTU46_LIBRARIES AND PTU46_INCLUDE_DIR)
    SET(PTU46_FOUND FALSE)
  ENDIF(PTU46_LIBRARIES AND PTU46_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    PTU46_INCLUDE_DIR
    PTU46_LIBRARIES
    PTUPTU46_LIBRARY
    EVIPTU46_LIBRARY
    SERIALPTU46_LIBRARY
    )
ENDIF(NOT UNIX)
