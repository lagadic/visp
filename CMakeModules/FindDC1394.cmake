#############################################################################
#
# $Id: FindDC1394.cmake,v 1.2 2006-05-30 08:35:00 fspindle Exp $
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
# Try to find libDC1394 for IEEE1394 camera.
# Once run this will define: 
#
# DC1394_FOUND
# DC1394_INCLUDE_DIR
# DC1394_LIBRARIESCOIN_FOUND
# COIN_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindDC1394.cmake: libdc1394 only available for Unix.")
  SET(DC1394_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(DC1394_INCLUDE_DIR libdc1394/dc1394_control.h
    $ENV{DC1394_HOME}/include
    /usr/include )
  #MESSAGE("DBG DC1394_INCLUDE_DIR=${DC1394_INCLUDE_DIR}")  
  
  FIND_LIBRARY(DC1394_LIBRARY
    NAMES dc1394_control
    PATHS 
    $ENV{DC1394_HOME}/lib
    /usr/lib
    )
  #MESSAGE("DBG DC1394_LIBRARY=${DC1394_LIBRARY}")
  
  ## --------------------------------
  
  IF(DC1394_LIBRARY)
    SET(DC1394_LIBRARIES ${DC1394_LIBRARY})
  ELSE(DC1394_LIBRARY)
    #MESSAGE("libdc1394 library not found.")
  ENDIF(DC1394_LIBRARY)
  
  IF(NOT DC1394_INCLUDE_DIR)
    #MESSAGE("libdc1394 include dir not found.")
  ENDIF(NOT DC1394_INCLUDE_DIR)
  
  IF(DC1394_LIBRARIES AND DC1394_INCLUDE_DIR)
    SET(DC1394_FOUND TRUE)
  ELSE(DC1394_LIBRARIES AND DC1394_INCLUDE_DIR)
    SET(DC1394_FOUND FALSE)
  ENDIF(DC1394_LIBRARIES AND DC1394_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    DC1394_INCLUDE_DIR
    DC1394_LIBRARIES
    DC1394_LIBRARY
    )
ENDIF(NOT UNIX)
