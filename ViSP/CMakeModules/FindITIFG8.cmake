#############################################################################
#
# $Id$
#
# Copyright (C) 1998-2006 Inria. All rights reserved.
#
# This software was developed at:
# IRISA/INRIA Rennes
# Projet Lagadic
# Campus Universitaire de Beaulieu
# 35042 Rennes Cedex
# http://www.irisa.fr/lagadic
#
# This file is part of the ViSP toolkit.
#
# This file may be distributed under the terms of the Q Public License
# as defined by Trolltech AS of Norway and appearing in the file
# LICENSE included in the packaging of this file.
#
# Licensees holding valid ViSP Professional Edition licenses may
# use this file in accordance with the ViSP Commercial License
# Agreement provided with the Software.
#
# This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
# WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
#
# Contact visp@irisa.fr if any conditions of this licensing are
# not clear to you.
#
# Description:
# Try to find device driver itifg-8.x for Coreco Imaging frame grabber boards.
# Once run this will define: 
#
# ITIFG8_FOUND
# ITIFG8_INCLUDE_DIR
# ITIFG8_LIBRARIES
# ITIFG8_VERSION
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindITIFG8.cmake: only available for Unix.")
  SET(ITIFG8_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(ITIFG8_INCLUDE_DIR itifgExt.h libitifg.h
    $ENV{ITIFG8_HOME}/include
    /usr/local/include
    /usr/include
    )
  #MESSAGE("DBG ITIFG8_INCLUDE_DIR=${ITIFG8_INCLUDE_DIR}")  
  
  FIND_LIBRARY(ITIFG8_LIBRARY
    NAMES itifg
    PATHS 
    $ENV{ITIFG8_HOME}/lib
    /usr/local/lib
    /usr/lib
    )

  FIND_LIBRARY(FL_LIBRARY
    NAMES fl
    PATHS 
    /usr/local/lib
    /usr/lib
    )
  #MESSAGE("DBG ITIFG8_LIBRARY=${ITIFG8_LIBRARY}")
  
  ## --------------------------------
  
  IF(ITIFG8_LIBRARY AND FL_LIBRARY)
    SET(ITIFG8_LIBRARIES ${ITIFG8_LIBRARY} ${FL_LIBRARY})
  ELSE(ITIFG8_LIBRARY AND FL_LIBRARY)
    MESSAGE(SEND_ERROR "itifg8 library not found. Please turn USE_ITIFG8 off.")
  ENDIF(ITIFG8_LIBRARY AND FL_LIBRARY)
  
  IF(NOT ITIFG8_INCLUDE_DIR)
    MESSAGE(SEND_ERROR "itifg8 include dir not found. Please turn USE_ITIFG8 off.")
  ENDIF(NOT ITIFG8_INCLUDE_DIR)
  
  IF(ITIFG8_LIBRARIES AND ITIFG8_INCLUDE_DIR)
    SET(ITIFG8_FOUND TRUE)
  ELSE(ITIFG8_LIBRARIES AND ITIFG8_INCLUDE_DIR)
    SET(ITIFG8_FOUND FALSE)
  ENDIF(ITIFG8_LIBRARIES AND ITIFG8_INCLUDE_DIR)

  # If library found, check for the version number
  IF(ITIFG8_FOUND)
    #MESSAGE("DBG ITIFG8_LIBRARIES=${ITIFG8_LIBRARIES}")
    # CMake is giving me problems using TRY_COMPILE with the CMAKE_FLAGS
    # for the :STRING syntax if I have multiple values contained in a
    # single variable. This is a problem for the ITIFG8_LIBRARIES variable
    # because it does just that. When I feed this variable to the command,
    # only the first value gets the appropriate modifier and the rest 
    # get dropped. To get multiple single variables to work, I must 
    # separate them with a "\;"
    TRY_RUN(ITIFG8_VERSION_RUN_RESULT ITIFG8_VERSION_COMPILE_RESULT
      ${CMAKE_BINARY_DIR}
      ${CMAKE_MODULE_PATH}/checkForItifg8Version.cpp
      CMAKE_FLAGS 
        -DINCLUDE_DIRECTORIES=${ITIFG8_INCLUDE_DIR}
        -DLINK_DIRECTORIES=${ITIFG8_LIBRARY}
        -DLINK_LIBRARIES=${ITIFG8_LIBRARY}
#        -DLINK_LIBRARIES=${ITIFG8_LIBRARY}\;${FL_LIBRARY}
      OUTPUT_VARIABLE ITIFG8_VERSION_OUTPUT)
    #MESSAGE("DBG ITIFG8_VERSION_COMPILE_RESULT=${ITIFG8_VERSION_COMPILE_RESULT}")  
    #MESSAGE("DBG ITIFG8_VERSION_RUN_RESULT=${ITIFG8_VERSION_RUN_RESULT}")  
    #MESSAGE("DBG ITIFG8_VERSION_OUTPUT=${ITIFG8_VERSION_OUTPUT}")
    IF(ITIFG8_VERSION_COMPILE_RESULT)
      SET(ITIFG8_VERSION ${ITIFG8_VERSION_RUN_RESULT})
    ELSE(ITIFG8_VERSION_COMPILE_RESULT)
      MESSAGE(SEND_ERROR "itifg8 driver was found, but it's version cannot be retrieved. Please turn USE_ITIFG8 off.")
      SET(ITIFG8_FOUND FALSE)
    ENDIF(ITIFG8_VERSION_COMPILE_RESULT)
  ENDIF(ITIFG8_FOUND)

  MARK_AS_ADVANCED(
    ITIFG8_INCLUDE_DIR
    ITIFG8_LIBRARIES
    ITIFG8_LIBRARY
    ITIFG8_VERSION
    FL_LIBRARY
    )
ENDIF(NOT UNIX)
