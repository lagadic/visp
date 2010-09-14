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
# Try to find libraries for Irisa's SBS Bit3 vertical bus driver for
# Afma 4 and 6 robots.
# Once run this will define: 
#
# BIT3_FOUND
# BIT3_INCLUDE_DIR
# BIT3_LIBRARIES
# BIT3_DEFS
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindBIT3.cmake: only available for Unix.")
  SET(BIT3_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(BIT3_INCLUDE_DIR btapi.h
    $ENV{BIT3_HOME}/include
    /local/driver/bit3-617/1003/current/include 
    )
  #MESSAGE("DBG BIT3_INCLUDE_DIR=${BIT3_INCLUDE_DIR}")  
  
  FIND_LIBRARY(BIT3_LIBRARY
    NAMES btp
    PATHS 
    $ENV{BIT3_HOME}/lib
    /local/driver/bit3-617/1003/current/include 
    )

  #MESSAGE("DBG BIT3_LIBRARY=${BIT3_LIBRARY}")
  
  ## --------------------------------
  
  IF(BIT3_LIBRARY)
    SET(BIT3_LIBRARIES ${BIT3_LIBRARY})
  ELSE(BIT3_LIBRARY)
    MESSAGE(SEND_ERROR "Bit3 library not found.")
  ENDIF(BIT3_LIBRARY)
  
  IF(NOT BIT3_INCLUDE_DIR)
    MESSAGE(SEND_ERROR "Bit3 include dir not found.")
  ENDIF(NOT BIT3_INCLUDE_DIR)
  
  IF(BIT3_LIBRARIES AND BIT3_INCLUDE_DIR)
    SET(BIT3_INCLUDE_DIR ${BIT3_INCLUDE_DIR})
    SET(BIT3_DEFS "-DBT1003") 
    SET(BIT3_FOUND TRUE)
  ELSE(BIT3_LIBRARIES AND BIT3_INCLUDE_DIR)
    SET(BIT3_DEFS "") 
    SET(BIT3_FOUND FALSE)
  ENDIF(BIT3_LIBRARIES AND BIT3_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    BIT3_INCLUDE_DIR
    BIT3_LIBRARIES
    BIT3_LIBRARY
    BIT3_DEFS
    )
ENDIF(NOT UNIX)
