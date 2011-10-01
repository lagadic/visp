#############################################################################
#
# $Id$
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
# Try to find Cerebellum Irisa library API to dial with the robot Afma6
#
# LIBFREENECT_FOUND
# LIBFREENECT_INCLUDE_DIR
# LIBFREENECT_LIBRARIES
#
# Authors:
# Celine Teuliere
# Fabien Spindler

IF(WIN32)
  FIND_PATH(LIBFREENECT_INCLUDE_DIR libfreenect.h
    $ENV{LIBFREENECT_HOME}/include
    )
  FIND_LIBRARY(LIBFREENECT_LIBRARY libfreenect
    $ENV{LIBFREENECT_HOME}/lib
    "c:/libfreenect/lib"
    )
ELSE(WIN32) 
  FIND_PATH(LIBFREENECT_INCLUDE_DIR libfreenect.h
    $ENV{LIBFREENECT_HOME}/include/
    /usr/include
    /usr/local/include
    )
  FIND_LIBRARY(LIBFREENECT_LIBRARY libfreenect.a
    $ENV{LIBFREENECT_HOME}/lib
    $ENV{LIBFREENECT_HOME}/build/lib
    /usr/lib
    /usr/local/lib
    )
ENDIF(WIN32)

## --------------------------------

IF(LIBFREENECT_LIBRARY)
  SET(LIBFREENECT_LIBRARIES ${LIBFREENECT_LIBRARY})
ENDIF(LIBFREENECT_LIBRARY)

IF(LIBFREENECT_LIBRARIES AND LIBFREENECT_INCLUDE_DIR)
  SET(LIBFREENECT_INCLUDE_DIR ${LIBFREENECT_INCLUDE_DIR})
  SET(LIBFREENECT_DIR ${LIBFREENECT_INCLUDE_DIR})
  SET(LIBFREENECT_FOUND TRUE)
ELSE(LIBFREENECT_LIBRARIES AND LIBFREENECT_INCLUDE_DIR)
  SET(LIBFREENECT_FOUND FALSE)
ENDIF(LIBFREENECT_LIBRARIES AND LIBFREENECT_INCLUDE_DIR)

MARK_AS_ADVANCED(
  LIBFREENECT_INCLUDE_DIR
  LIBFREENECT_LIBRARIES
  LIBFREENECT_LIBRARY
  )
