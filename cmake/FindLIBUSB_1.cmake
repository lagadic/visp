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
# Try to find libusb-1.0 library. Headers include dir is not searched.
#
# LIBUSB_1_FOUND
# LIBUSB_1_LIBRARIES
# LIBUSB_1_INCLUDE_DIRS
#
# Authors:
# Celine Teuliere
# Fabien Spindler

IF(WIN32)
  FIND_LIBRARY(LIBUSB_1_LIBRARY libusb
    $ENV{LIBUSB_1_HOME}
    $ENV{LIBUSB_1_HOME}/lib
    "c:/libusb/lib"
    )
ELSE(WIN32) 
  FIND_LIBRARY(LIBUSB_1_LIBRARY usb-1.0
    $ENV{LIBUSB_1_HOME}/lib
    $ENV{LIBUSB_1_HOME}/build/lib
    /lib/
    /lib64/
    /usr/lib
    /usr/local/lib
    )
  FIND_PATH(LIBUSB_1_INCLUDE_DIR libusb.h
    $ENV{LIBUSB_1_HOME}/include/libusb-1.0
    $ENV{LIBUSB_1_HOME}/build/include/libusb-1.0
    /usr/include/libusb-1.0
    /usr/local/include/libusb-1.0
    )
ENDIF(WIN32)

## --------------------------------

IF(LIBUSB_1_LIBRARY AND LIBUSB_1_INCLUDE_DIR)
  SET(LIBUSB_1_INCLUDE_DIRS ${LIBUSB_1_INCLUDE_DIR})
  SET(LIBUSB_1_LIBRARIES ${LIBUSB_1_LIBRARY})
  SET(LIBUSB_1_FOUND TRUE)
ELSE()
  SET(LIBUSB_1_FOUND FALSE)
ENDIF()

MARK_AS_ADVANCED(
  LIBUSB_1_INCLUDE_DIRS
  LIBUSB_1_INCLUDE_DIR
  LIBUSB_1_LIBRARIES
  LIBUSB_1_LIBRARY
  )




