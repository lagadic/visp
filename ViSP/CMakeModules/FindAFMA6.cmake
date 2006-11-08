#############################################################################
#
# $Id: FindAFMA6.cmake,v 1.3 2006-11-08 14:53:13 fspindle Exp $
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
# This file is part of the ViSP toolkit
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
# Try to find libraries for Irisa's Afma 6 ddl cartesian robot 
# Once run this will define: 
#
# AFMA6_FOUND
# AFMA6_INCLUDE_DIR
# AFMA6_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindAFMA6.cmake: Afma6 only available for Unix.")
  SET(AFMA6_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(AFMA6_INCLUDE_DIR main_Afma6.h
    $ENV{AFMA6_HOME}/include
    /udd/fspindle/robot/Afma6/current/include
    /local/soft/Afma6/current/include 
    )
  #MESSAGE("DBG AFMA6_INCLUDE_DIR=${AFMA6_INCLUDE_DIR}")  
  
  FIND_LIBRARY(AFMA6_ROBOT_LIBRARY
    NAMES robotAfma6
    PATHS 
    $ENV{AFMA6_HOME}/lib
    /udd/fspindle/robot/Afma6/current/lib
    /local/soft/Afma6/current/lib
    )
  FIND_LIBRARY(AFMA6_TOOLS_LIBRARY
    NAMES toolsAfma6
    PATHS 
    $ENV{AFMA6_HOME}/lib
    /udd/fspindle/robot/Afma6/current/lib
    /local/soft/Afma6/current/lib
    )
  FIND_LIBRARY(AFMA6_UPRIM_LIBRARY
    NAMES uprimAfma6
    PATHS 
    $ENV{AFMA6_HOME}/lib
    /udd/fspindle/robot/Afma6/current/lib
    /local/soft/Afma6/current/lib
    )
  FIND_LIBRARY(AFMA6_BIT3_LIBRARY
    NAMES bit3Afma6
    PATHS 
    $ENV{AFMA6_HOME}/lib
    /udd/fspindle/robot/Afma6/current/lib
    /local/soft/Afma6/current/lib
    )
  #MESSAGE("DBG AFMA6_LIBRARY=${AFMA6_LIBRARY}")
  
  ## --------------------------------
  
  IF(AFMA6_ROBOT_LIBRARY AND AFMA6_TOOLS_LIBRARY AND AFMA6_UPRIM_LIBRARY 
      AND AFMA6_BIT3_LIBRARY)
    SET(AFMA6_LIBRARIES ${AFMA6_ROBOT_LIBRARY} ${AFMA6_TOOLS_LIBRARY}
      ${AFMA6_UPRIM_LIBRARY} ${AFMA6_BIT3_LIBRARY})
  ELSE(AFMA6_ROBOT_LIBRARY AND AFMA6_TOOLS_LIBRARY AND AFMA6_UPRIM_LIBRARY 
      AND AFMA6_BIT3_LIBRARY)
    MESSAGE(SEND_ERROR "Afma6 library not found.")
  ENDIF(AFMA6_ROBOT_LIBRARY AND AFMA6_TOOLS_LIBRARY AND AFMA6_UPRIM_LIBRARY 
      AND AFMA6_BIT3_LIBRARY)
  
  IF(NOT AFMA6_INCLUDE_DIR)
    MESSAGE(SEND_ERROR "Afma6 include dir not found.")
  ENDIF(NOT AFMA6_INCLUDE_DIR)
  
  IF(AFMA6_LIBRARIES AND AFMA6_INCLUDE_DIR)
    SET(AFMA6_FOUND TRUE)
  ELSE(AFMA6_LIBRARIES AND AFMA6_INCLUDE_DIR)
    SET(AFMA6_FOUND FALSE)
  ENDIF(AFMA6_LIBRARIES AND AFMA6_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    AFMA6_INCLUDE_DIR
    AFMA6_LIBRARIES
    AFMA6_LIBRARY
    AFMA6_ROBOT_LIBRARY
    AFMA6_TOOLS_LIBRARY
    AFMA6_UPRIM_LIBRARY
    AFMA6_BIT3_LIBRARY
    )
ENDIF(NOT UNIX)
