#############################################################################
#
# $Id$
#
# Copyright (C) 1998-2010 Inria. All rights reserved.
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
# Try to find Cerebellum CA linux library material 
#
# CALINUX_FOUND
# CALINUX_INCLUDE_DIR
# CALINUX_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

IF(NOT UNIX)
  # MESSAGE("FindCALINUX.cmake: only available for Unix.")
  SET(CALINUX_FOUND FALSE)
ELSE(NOT UNIX)
  
  FIND_PATH(CALINUX_INCLUDE_DIR calinux.h
    $ENV{CALINUX_HOME}/include
    /local/soft/Cerebellum/CALinux/current/include
    )
  #MESSAGE("DBG CALINUX_INCLUDE_DIR=${CALINUX_INCLUDE_DIR}")  
  
  FIND_LIBRARY(CALINUX_LIBRARY
    NAMES calinux
    PATHS 
    $ENV{CALINUX_HOME}/lib
    /local/soft/Cerebellum/CALinux/current/lib
    )

  #MESSAGE("DBG CALINUX_LIBRARY=${CALINUX_LIBRARY}")
  
  ## --------------------------------
  
  IF(CALINUX_LIBRARY)
    SET(CALINUX_LIBRARIES ${CALINUX_LIBRARY})
  ELSE(CALINUX_LIBRARY)
#     MESSAGE(SEND_ERROR "Calinux library not found.")
  ENDIF(CALINUX_LIBRARY)
  
  IF(NOT CALINUX_INCLUDE_DIR)
#     MESSAGE(SEND_ERROR "Calinux include dir not found.")
  ENDIF(NOT CALINUX_INCLUDE_DIR)
  
  IF(CALINUX_LIBRARIES AND CALINUX_INCLUDE_DIR)
    SET(CALINUX_FOUND TRUE)
  ELSE(CALINUX_LIBRARIES AND CALINUX_INCLUDE_DIR)
    SET(CALINUX_FOUND FALSE)
  ENDIF(CALINUX_LIBRARIES AND CALINUX_INCLUDE_DIR)
  
  MARK_AS_ADVANCED(
    CALINUX_INCLUDE_DIR
    CALINUX_LIBRARIES
    CALINUX_LIBRARY
    )
ENDIF(NOT UNIX)
