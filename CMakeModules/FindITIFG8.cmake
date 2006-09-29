#############################################################################
#
# $Id: FindITIFG8.cmake,v 1.1 2006-09-29 12:49:44 fspindle Exp $
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
  
  MARK_AS_ADVANCED(
    ITIFG8_INCLUDE_DIR
    ITIFG8_LIBRARIES
    ITIFG8_LIBRARY
    FL_LIBRARY
    )
ENDIF(NOT UNIX)
