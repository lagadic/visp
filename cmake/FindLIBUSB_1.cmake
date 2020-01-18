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
# Try to find libusb-1.0 library. Headers include dir is not searched.
#
# LIBUSB_1_FOUND
# LIBUSB_1_LIBRARIES
# LIBUSB_1_INCLUDE_DIRS
# LIBUSB_1_VERSION
#
# Authors:
# Celine Teuliere
# Fabien Spindler
#############################################################################

if(WIN32)
  find_library(LIBUSB_1_LIBRARY libusb
    $ENV{LIBUSB_1_HOME}
    $ENV{LIBUSB_1_HOME}/lib
    $ENV{LIBUSB_1_DIR}
    $ENV{LIBUSB_1_DIR}/lib
    "c:/libusb/lib"
    )
else()
  find_library(LIBUSB_1_LIBRARY usb-1.0
    $ENV{LIBUSB_1_HOME}/lib
    $ENV{LIBUSB_1_HOME}/build/lib
    $ENV{LIBUSB_1_DIR}/lib
    $ENV{LIBUSB_1_DIR}/build/lib
    /lib/
    /lib64/
    /usr/lib
    /usr/local/lib
    )
  find_path(LIBUSB_1_INCLUDE_DIR libusb.h
    $ENV{LIBUSB_1_HOME}/include/libusb-1.0
    $ENV{LIBUSB_1_HOME}/build/include/libusb-1.0
    $ENV{LIBUSB_1_DIR}/include/libusb-1.0
    $ENV{LIBUSB_1_DIR}/build/include/libusb-1.0
    /usr/include/libusb-1.0
    /usr/local/include/libusb-1.0
    )
endif()

## --------------------------------

if(LIBUSB_1_LIBRARY AND LIBUSB_1_INCLUDE_DIR)
  set(LIBUSB_1_INCLUDE_DIRS ${LIBUSB_1_INCLUDE_DIR})
  set(LIBUSB_1_LIBRARIES ${LIBUSB_1_LIBRARY})
  set(LIBUSB_1_FOUND TRUE)

  get_filename_component(LIBUSB_1_LIB_DIR ${LIBUSB_1_LIBRARY} PATH)
  vp_get_version_from_pkg("libusb" "${LIBUSB_1_LIB_DIR}/pkgconfig" LIBUSB_1_VERSION)
  if(NOT LIBUSB_1_VERSION)
    vp_get_version_from_pkg("libusb-1.0" "${LIBUSB_1_LIB_DIR}/pkgconfig" LIBUSB_1_VERSION)
  endif()
else()
  set(LIBUSB_1_FOUND FALSE)
endif()

mark_as_advanced(
  LIBUSB_1_INCLUDE_DIRS
  LIBUSB_1_INCLUDE_DIR
  LIBUSB_1_LIBRARIES
  LIBUSB_1_LIBRARY
  )




