#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2026 by Inria. All rights reserved.
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
# See https://visp.inria.fr for more information.
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
# Try to find libusb-1.0 library.
#
# LIBUSB_1_FOUND
# LIBUSB_1_LIBRARIES
# LIBUSB_1_INCLUDE_DIRS
# LIBUSB_1_VERSION
#
#############################################################################


# Layouts handled, relative to each standard prefix and to LIBUSB_1_HOME or LIBUSB_1_DIR:
#   <prefix>/include/libusb-1.0 (e.g. /usr/include/libusb-1.0), <prefix>/lib
#   <prefix>/build/include/libusb-1.0, <prefix>/build/lib
set(LIBUSB_1_SEARCH_HINTS
  ${LIBUSB_1_HOME}
  $ENV{LIBUSB_1_HOME}
  ${LIBUSB_1_DIR}
  $ENV{LIBUSB_1_DIR}
)
set(LIBUSB_1_SEARCH_PATHS "")
set(LIBUSB_1_LIB_NAMES usb-1.0)
if(WIN32)
  list(APPEND LIBUSB_1_SEARCH_PATHS "C:/libusb")
  list(APPEND LIBUSB_1_LIB_NAMES libusb-1.0 libusb)
endif()

find_library(LIBUSB_1_LIBRARY
  NAMES ${LIBUSB_1_LIB_NAMES}
  HINTS ${LIBUSB_1_SEARCH_HINTS}
  PATHS ${LIBUSB_1_SEARCH_PATHS}
  PATH_SUFFIXES lib build/lib
)
find_path(LIBUSB_1_INCLUDE_DIR libusb.h
  HINTS ${LIBUSB_1_SEARCH_HINTS}
  PATHS ${LIBUSB_1_SEARCH_PATHS}
  PATH_SUFFIXES libusb-1.0 include/libusb-1.0 build/include/libusb-1.0
)

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
  LIBUSB_1_INCLUDE_DIR
  LIBUSB_1_LIBRARY
)
