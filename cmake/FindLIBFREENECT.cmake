#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2015 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
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
# LIBFREENECT_FOUND
# LIBFREENECT_INCLUDE_DIRS
# LIBFREENECT_LIBRARIES
#
# Authors:
# Celine Teuliere
# Fabien Spindler

IF(WIN32)
  FIND_PATH(LIBFREENECT_HPP_INCLUDE_DIR libfreenect.hpp
    $ENV{LIBFREENECT_HOME}/include
    $ENV{LIBFREENECT_HPP_INCLUDE_DIR}
    $ENV{LIBFREENECT_H_INCLUDE_DIR}
    )
  FIND_PATH(LIBFREENECT_H_INCLUDE_DIR libfreenect.h
    $ENV{LIBFREENECT_HOME}/include
    $ENV{LIBFREENECT_HPP_INCLUDE_DIR}
    $ENV{LIBFREENECT_H_INCLUDE_DIR}
    )
  FIND_LIBRARY(LIBFREENECT_LIBRARY freenect
    $ENV{LIBFREENECT_HOME}/lib
    $ENV{LIBFREENECT_LIBRARY_DIR}
    "c:/libfreenect/lib"
    )
ELSE() 
  FIND_PATH(LIBFREENECT_HPP_INCLUDE_DIR libfreenect.hpp
    $ENV{LIBFREENECT_HOME}/include
    /usr/include
    /usr/local/include
    /usr/include/libfreenect
    /usr/local/include/libfreenect
    )
  FIND_PATH(LIBFREENECT_H_INCLUDE_DIR libfreenect.h
    $ENV{LIBFREENECT_HOME}/include
    /usr/include
    /usr/local/include
    /usr/include/libfreenect
    /usr/local/include/libfreenect
    )
  FIND_LIBRARY(LIBFREENECT_LIBRARY freenect
    $ENV{LIBFREENECT_HOME}/lib
    $ENV{LIBFREENECT_HOME}/build/lib
    /usr/lib
    /usr/local/lib
    )
ENDIF()

## --------------------------------
IF(LIBFREENECT_LIBRARY AND LIBFREENECT_HPP_INCLUDE_DIR AND LIBFREENECT_H_INCLUDE_DIR)
  SET(LIBFREENECT_INCLUDE_DIRS ${LIBFREENECT_HPP_INCLUDE_DIR} ${LIBFREENECT_H_INCLUDE_DIR})
  SET(LIBFREENECT_LIBRARIES ${LIBFREENECT_LIBRARY})
  SET(LIBFREENECT_FOUND TRUE)
ELSE()
  SET(LIBFREENECT_FOUND FALSE)
ENDIF()

MARK_AS_ADVANCED(
  LIBFREENECT_INCLUDE_DIRS
  LIBFREENECT_HPP_INCLUDE_DIR
  LIBFREENECT_H_INCLUDE_DIR
  LIBFREENECT_LIBRARIES
  LIBFREENECT_LIBRARY
  )
