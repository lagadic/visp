#############################################################################
#
# $Id: FindBIT3.cmake,v 1.4 2008-02-05 09:08:53 fspindle Exp $
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
