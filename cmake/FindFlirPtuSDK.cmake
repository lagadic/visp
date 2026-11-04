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
# Try to find FLIR Pan Tilt Unit SDK
#
# FLIRPTUSDK_FOUND
# FLIRPTUSDK_INCLUDE_DIRS
# FLIRPTUSDK_LIBRARIES
#
#############################################################################

# Layouts handled, relative to each standard prefix and to FLIRPTUSDK_HOME:
#   <prefix>/include/cerial (e.g. /usr/include/cerial)
#   <FLIRPTUSDK_HOME>, <FLIRPTUSDK_HOME>/cerial
set(FLIRPTUSDK_SEARCH_HINTS
  ${FLIRPTUSDK_HOME}
  $ENV{FLIRPTUSDK_HOME}
)
set(FLIRPTUSDK_LIB_SUFFIXES "")
if(MSVC)
  if(CMAKE_CL_64)
    list(APPEND FLIRPTUSDK_LIB_SUFFIXES build/visual_studio_2008/x64/Release)
  else()
    list(APPEND FLIRPTUSDK_LIB_SUFFIXES build/visual_studio_2008/Release)
  endif()
endif()

find_path(FLIRPTUSDK_CPI_INCLUDE_DIR cpi.h
  HINTS ${FLIRPTUSDK_SEARCH_HINTS}
  PATH_SUFFIXES cerial
)

find_path(FLIRPTUSDK_CERIAL_INCLUDE_DIR cerial.h
  HINTS ${FLIRPTUSDK_SEARCH_HINTS}
  PATH_SUFFIXES cerial
)

find_library(FLIRPTUSDK_LIBRARY
  NAMES CPI2.lib libcpi.a
  HINTS ${FLIRPTUSDK_SEARCH_HINTS}
  PATH_SUFFIXES ${FLIRPTUSDK_LIB_SUFFIXES}
)

if(FLIRPTUSDK_LIBRARY AND FLIRPTUSDK_CPI_INCLUDE_DIR AND FLIRPTUSDK_CERIAL_INCLUDE_DIR)
  set(FLIRPTUSDK_INCLUDE_DIRS ${FLIRPTUSDK_CPI_INCLUDE_DIR} ${FLIRPTUSDK_CERIAL_INCLUDE_DIR})
  set(FLIRPTUSDK_LIBRARIES ${FLIRPTUSDK_LIBRARY})
  set(FLIRPTUSDK_FOUND TRUE)
else()
  set(FLIRPTUSDK_FOUND FALSE)
endif()

mark_as_advanced(
  FLIRPTUSDK_CPI_INCLUDE_DIR
  FLIRPTUSDK_CERIAL_INCLUDE_DIR
  FLIRPTUSDK_LIBRARY
)
