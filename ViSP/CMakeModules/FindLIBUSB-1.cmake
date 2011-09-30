#############################################################################
#
# $Id: FindIRISA.cmake 3057 2011-02-11 13:17:26Z fspindle $
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
# LIBUSB_1_FOUND
# LIBUSB_1_INCLUDE_DIR
# LIBUSB_1_LIBRARIES
#
# Authors:
# Celine Teuliere
# Fabien Spindler

IF(WIN32)
  FIND_PATH(LIBUSB_1_INCLUDE_DIR libusb.h
    $ENV{LIBUSB_1_HOME}/include
    )
  FIND_LIBRARY(LIBUSB_1_LIBRARY libusb
    $ENV{LIBUSB_1_HOME}/lib
    "c:/libusb/lib"
    )
ELSE(WIN32) 
  FIND_PATH(LIBUSB_1_INCLUDE_DIR libusb.h
    $ENV{LIBUSB_1_HOME}/include/
    usr/include/libusb-1.0/
    usr/include/
    )
  FIND_LIBRARY(LIBUSB_1_LIBRARY libusb-1.0
    $ENV{LIBUSB_1_HOME}/lib
    $ENV{LIBUSB_1_HOME}/build/lib
    /lib/
    /lib64/
    /usr/lib
    /usr/local/lib
    )
ENDIF(WIN32)

## --------------------------------

IF(LIBUSB_1_LIBRARY)
  SET(LIBUSB_1_LIBRARIES ${LIBUSB_1_LIBRARY})
ENDIF(LIBUSB_1_LIBRARY)

IF(LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIR)
  SET(LIBUSB_1_INCLUDE_DIR ${LIBUSB_1_INCLUDE_DIR})
  SET(LIBUSB_1_DIR ${LIBUSB_1_INCLUDE_DIR})
  SET(LIBUSB_1_FOUND TRUE)
ELSE(LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIR)
  SET(LIBUSB_1_FOUND FALSE)
ENDIF(LIBUSB_1_LIBRARIES AND LIBUSB_1_INCLUDE_DIR)

MARK_AS_ADVANCED(
  LIBUSB_1_INCLUDE_DIR
  LIBUSB_1_LIBRARIES
  LIBUSB_1_LIBRARY
  )




