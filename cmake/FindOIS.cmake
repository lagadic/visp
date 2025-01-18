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
# Try to find OIS 3rd party used by vpAROgre class
# Once run this will define:
#
# OIS_FOUND
# OIS_INCLUDE_DIRS
# OIS_LIBRARIES
# OIS_VERSION
#
#############################################################################

find_path(OIS_INCLUDE_DIR
  NAMES ois/OIS.h
  PATHS
    $ENV{OIS_DIR}/include/ois
    ${OIS_DIR}/include/ois
    /usr/include
    /usr/local/include
)

find_library(OIS_LIBRARY
  NAMES OIS
  PATHS
    $ENV{OIS_DIR}/lib
    ${OIS_DIR}/lib
    /usr/lib
    /usr/local/lib
    )

if(OIS_LIBRARY AND OIS_INCLUDE_DIR)
  set(OIS_INCLUDE_DIRS ${OIS_INCLUDE_DIR})
  set(OIS_LIBRARIES ${OIS_LIBRARY})
  set(OIS_FOUND TRUE)

  get_filename_component(OIS_LIB_DIR ${OIS_LIBRARY} PATH)
  vp_get_version_from_pkg("OIS" "${OIS_LIB_DIR}/pkgconfig" OIS_VERSION)

  if(NOT OIS_VERSION)
    vp_parse_header("${OIS_INCLUDE_DIR}/ois/OISPrereqs.h" OIS_VERSION_LINES OIS_VERSION_MAJOR OIS_VERSION_MINOR OIS_VERSION_PATCH)
    set(OIS_VERSION "${OIS_VERSION_MAJOR}.${OIS_VERSION_MINOR}.${OIS_VERSION_PATCH}")
  endif()
else()
  set(OIS_FOUND FALSE)
endif()

mark_as_advanced(
  OIS_INCLUDE_DIR
  OIS_LIBRARY
)
