#############################################################################
#
# $Id$
#
# Copyright (C) 2008 Inria. All rights reserved.
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
# Try to find Cerebellum Irisa library API to dial with the robot Afma6
#
# IRISA_FOUND
# IRISA_INCLUDE_DIR
# IRISA_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindIRISA.cmake: only available for Unix.")
  SET(IRISA_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(IRISA_INCLUDE_DIR irisa_Afma6.h
    $ENV{IRISA_HOME}/include
    /local/soft/Cerebellum/Irisa/current/src
    )
  #MESSAGE("DBG IRISA_INCLUDE_DIR=${IRISA_INCLUDE_DIR}")  
  
  FIND_LIBRARY(IRISA_LIBRARY
    NAMES irisa
    PATHS 
    $ENV{IRISA_HOME}/lib
    /local/soft/Cerebellum/Irisa/current/src
    )

  #MESSAGE("DBG IRISA_LIBRARY=${IRISA_LIBRARY}")
  
  ## --------------------------------
  
  IF(IRISA_LIBRARY)
    SET(IRISA_LIBRARIES ${IRISA_LIBRARY})
  ELSE(IRISA_LIBRARY)
    #MESSAGE(SEND_ERROR "Irisa library not found.")
  ENDIF(IRISA_LIBRARY)
  
  IF(NOT IRISA_INCLUDE_DIR)
    #MESSAGE(SEND_ERROR "Irisa include dir not found.")
  ENDIF(NOT IRISA_INCLUDE_DIR)
  
  IF(IRISA_LIBRARIES AND IRISA_INCLUDE_DIR)
    SET(IRISA_FOUND TRUE)
  ELSE(IRISA_LIBRARIES AND IRISA_INCLUDE_DIR)
    SET(IRISA_FOUND FALSE)
  ENDIF(IRISA_LIBRARIES AND IRISA_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    IRISA_INCLUDE_DIR
    IRISA_LIBRARIES
    IRISA_LIBRARY
    )
ENDIF(NOT UNIX)
