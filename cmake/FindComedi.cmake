#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
# Try to find comedi library
#
# COMEDI_FOUND
# COMEDI_INCLUDE_DIRS
# COMEDI_LIBRARIES
# COMEDI_VERSION
#
#############################################################################

find_path(COMEDI_INCLUDE_DIRS comedilib.h
  PATHS
    $ENV{COMEDI_HOME}/include
    /usr/include
    /usr/local/include
)

find_library(COMEDI_LIBRARIES
  NAMES comedi
  PATHS
    $ENV{COMEDI_HOME}/lib
    /usr/lib
    /usr/local/lib
)

if(COMEDI_LIBRARIES AND COMEDI_INCLUDE_DIRS)
  set(COMEDI_FOUND TRUE)
  vp_parse_header("${COMEDI_INCLUDE_DIRS}/comedilib_version.h" COMEDILIB_VERSION_LINES COMEDILIB_VERSION_MAJOR COMEDILIB_VERSION_MINOR COMEDILIB_VERSION_MICRO)
  set(COMEDI_VERSION "${COMEDILIB_VERSION_MAJOR}.${COMEDILIB_VERSION_MINOR}.${COMEDILIB_VERSION_MICRO}")
else()
  set(COMEDI_FOUND FALSE)
endif()

mark_as_advanced(
  COMEDI_INCLUDE_DIRS
  COMEDI_LIBRARIES
)
