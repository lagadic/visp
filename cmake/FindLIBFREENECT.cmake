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
# LIBFREENECT_FOUND
# LIBFREENECT_INCLUDE_DIRS
# LIBFREENECT_LIBRARIES
# LIBFREENECT_VERSION
#
# Authors:
# Celine Teuliere
# Fabien Spindler

if(WIN32)
  find_path(LIBFREENECT_HPP_INCLUDE_DIR libfreenect.hpp
    $ENV{LIBFREENECT_HOME}/include
    $ENV{LIBFREENECT_HPP_INCLUDE_DIR}
    $ENV{LIBFREENECT_H_INCLUDE_DIR}
    )
  find_path(LIBFREENECT_H_INCLUDE_DIR libfreenect.h
    $ENV{LIBFREENECT_HOME}/include
    $ENV{LIBFREENECT_HPP_INCLUDE_DIR}
    $ENV{LIBFREENECT_H_INCLUDE_DIR}
    )
  find_library(LIBFREENECT_LIBRARY freenect
    $ENV{LIBFREENECT_HOME}/lib
    $ENV{LIBFREENECT_LIBRARY_DIR}
    "c:/libfreenect/lib"
    )
else()
  find_path(LIBFREENECT_HPP_INCLUDE_DIR libfreenect.hpp
    $ENV{LIBFREENECT_HOME}/include
    /usr/include
    /usr/local/include
    /usr/include/libfreenect
    /usr/local/include/libfreenect
    )
  find_path(LIBFREENECT_H_INCLUDE_DIR libfreenect.h
    $ENV{LIBFREENECT_HOME}/include
    /usr/include
    /usr/local/include
    /usr/include/libfreenect
    /usr/local/include/libfreenect
    )
  find_library(LIBFREENECT_LIBRARY freenect
    $ENV{LIBFREENECT_HOME}/lib
    $ENV{LIBFREENECT_HOME}/build/lib
    /usr/lib
    /usr/local/lib
    )
endif()

## --------------------------------
if(LIBFREENECT_LIBRARY AND LIBFREENECT_HPP_INCLUDE_DIR AND LIBFREENECT_H_INCLUDE_DIR)
  set(LIBFREENECT_INCLUDE_DIRS ${LIBFREENECT_HPP_INCLUDE_DIR} ${LIBFREENECT_H_INCLUDE_DIR})
  set(LIBFREENECT_LIBRARIES ${LIBFREENECT_LIBRARY})
  set(LIBFREENECT_FOUND TRUE)

  get_filename_component(LIBFREENECT_LIB_DIR ${LIBFREENECT_LIBRARY} PATH)
  vp_get_version_from_pkg("libfreenect" "${LIBFREENECT_LIB_DIR}/pkgconfig" LIBFREENECT_VERSION)
else()
  set(LIBFREENECT_FOUND FALSE)
endif()

mark_as_advanced(
  LIBFREENECT_INCLUDE_DIRS
  LIBFREENECT_HPP_INCLUDE_DIR
  LIBFREENECT_H_INCLUDE_DIR
  LIBFREENECT_LIBRARIES
  LIBFREENECT_LIBRARY
  )
