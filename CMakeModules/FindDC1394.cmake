#############################################################################
#
# $Id: FindDC1394.cmake,v 1.3 2006-06-27 10:14:23 fspindle Exp $
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
# Try to find libDC1394 for IEEE1394 camera. First search for libdc1394-2.x
# and if not found, search for libdc1394-1.x
# Once run this will define: 
#
# DC1394_FOUND
# DC1394_VERSION
# DC1394_INCLUDE_DIR
# DC1394_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindDC1394.cmake: libdc1394 only available for Unix.")
  SET(DC1394_FOUND FALSE)
ELSE(NOT UNIX)
  
  # Search for libdc1394-2.x
  FIND_PATH(DC1394_2_INCLUDE_DIR dc1394/dc1394_control.h
    $ENV{DC1394_HOME}/include
    /usr/include )
  #MESSAGE("DBG DC1394_2_INCLUDE_DIR=${DC1394_2_INCLUDE_DIR}")  

  FIND_LIBRARY(DC1394_2_LIBRARY
    NAMES dc1394
    PATHS 
    $ENV{DC1394_HOME}/lib
    /usr/lib
    )
  #MESSAGE("DBG DC1394_2_LIBRARY=${DC1394_2_LIBRARY}")

  IF(DC1394_2_LIBRARY AND DC1394_2_INCLUDE_DIR)
    SET(DC1394_FOUND TRUE)
    SET(DC1394_VERSION 2)
    SET(DC1394_LIBRARIES ${DC1394_2_LIBRARY})
    SET(DC1394_INCLUDE_DIR ${DC1394_2_INCLUDE_DIR})
  ELSE(DC1394_2_LIBRARY AND DC1394_2_INCLUDE_DIR)

    # Search for libdc1394-1.x
    FIND_PATH(DC1394_1_INCLUDE_DIR libdc1394/dc1394_control.h
      $ENV{DC1394_HOME}/include
      /usr/include )
    #MESSAGE("DBG DC1394_1_INCLUDE_DIR=${DC1394_INCLUDE_DIR}")  
  
    FIND_LIBRARY(DC1394_1_LIBRARY
      NAMES dc1394_control
      PATHS 
      $ENV{DC1394_HOME}/lib
      /usr/lib
      )
    #MESSAGE("DBG DC1394_1_LIBRARY=${DC1394_LIBRARY}")

    IF(DC1394_1_LIBRARY AND DC1394_1_INCLUDE_DIR)
      SET(DC1394_FOUND TRUE)
      SET(DC1394_VERSION 1)
      SET(DC1394_LIBRARIES ${DC1394_1_LIBRARY})
      SET(DC1394_INCLUDE_DIR ${DC1394_1_INCLUDE_DIR})
    ELSE(DC1394_1_LIBRARY AND DC1394_1_INCLUDE_DIR)
      SET(DC1394_FOUND FALSE)
    ENDIF(DC1394_1_LIBRARY AND DC1394_1_INCLUDE_DIR)

  ENDIF(DC1394_2_LIBRARY AND DC1394_2_INCLUDE_DIR)

  ## --------------------------------

  MARK_AS_ADVANCED(
    DC1394_1_LIBRARY
    DC1394_1_INCLUDE_DIR
    DC1394_2_LIBRARY
    DC1394_2_INCLUDE_DIR
    DC1394_INCLUDE_DIR
    DC1394_LIBRARIES
    DC1394_LIBRARY
    )
ENDIF(NOT UNIX)
