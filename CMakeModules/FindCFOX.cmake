#############################################################################
#
# $Id: FindCFOX.cmake,v 1.3 2007-05-10 16:33:23 fspindle Exp $
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
# Try to find cfox library for Apple OS X (http://cfox.sourceforge.net/).
# Once run this will define: 
#
# CFOX_FOUND
# CFOX_INCLUDE_DIR
# CFOX_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT APPLE)
  SET(CFOX_FOUND FALSE)
ELSE(NOT APPLE)
  
  FIND_PATH(CFOX_INCLUDE_DIR cfox/cfox.h
    /sw/include/cfox
    /sw/include
    )
  #MESSAGE(STATUS "DBG CFOX_INCLUDE_DIR=${CFOX_INCLUDE_DIR}")  
  
  FIND_LIBRARY(CFOX_LIBRARY
    NAMES cfox
    PATHS
    /sw/lib
    )

  #MESSAGE(STATUS "DBG CFOX_LIBRARY=${CFOX_LIBRARY}")
  
  ## --------------------------------
  
  IF(CFOX_LIBRARY)
    SET(CFOX_LIBRARIES ${CFOX_LIBRARY})
  ELSE(CFOX_LIBRARY)
    MESSAGE(SEND_ERROR "cfox library not found.")
  ENDIF(CFOX_LIBRARY)
  
  IF(NOT CFOX_INCLUDE_DIR)
    MESSAGE(SEND_ERROR "cfox include dir not found.")
  ENDIF(NOT CFOX_INCLUDE_DIR)
  
  IF(CFOX_LIBRARIES AND CFOX_INCLUDE_DIR)
    SET(CFOX_FOUND TRUE)
  ELSE(CFOX_LIBRARIES AND CFOX_INCLUDE_DIR)
    SET(CFOX_FOUND FALSE)
  ENDIF(CFOX_LIBRARIES AND CFOX_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    CFOX_INCLUDE_DIR
    CFOX_LIBRARIES
    )
  #MESSAGE(STATUS "CFOX_FOUND : ${CFOX_FOUND}")

ENDIF(NOT APPLE)
