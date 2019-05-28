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
# Try to find Cerebellum Irisa library API to dial with the robot Afma6
#
# IRISA_FOUND
# IRISA_INCLUDE_DIRS
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
    $ENV{IRISA_HOME}/src
    /local/soft/Cerebellum/Irisa/current/src
    /home/soft/Cerebellum/Irisa/current/src
    )
  #MESSAGE("DBG IRISA_INCLUDE_DIR=${IRISA_INCLUDE_DIR}")  
  
  FIND_LIBRARY(IRISA_LIBRARY
    NAMES irisa
    PATHS 
    $ENV{IRISA_HOME}/src
    /local/soft/Cerebellum/Irisa/current/src
    /home/soft/Cerebellum/Irisa/current/src
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
    SET(IRISA_INCLUDE_DIRS ${IRISA_INCLUDE_DIR})
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
