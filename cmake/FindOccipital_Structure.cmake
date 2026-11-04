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
# Try to find Occipital Structure SDK to work with Structure Core device.
# Once run this will define:
#
# OCCIPITAL_STRUCTURE_FOUND
# OCCIPITAL_STRUCTURE_INCLUDE_DIRS
# OCCIPITAL_STRUCTURE_LIBRARIES
#
#############################################################################


set(OCCIPITAL_STRUCTURE_SEARCH_HINTS
  ${OCCIPITAL_STRUCTURE_DIR}
  $ENV{OCCIPITAL_STRUCTURE_DIR}
)
set(OCCIPITAL_STRUCTURE_SEARCH_PATHS "")
set(OCCIPITAL_STRUCTURE_LIB_SUFFIXES "")

if(MSVC)
  list(APPEND OCCIPITAL_STRUCTURE_SEARCH_PATHS "C:/Program Files (x86)/Structure SDK")
  list(APPEND OCCIPITAL_STRUCTURE_LIB_SUFFIXES Libraries/Structure/Windows/x86_64)
elseif(APPLE)
  list(APPEND OCCIPITAL_STRUCTURE_LIB_SUFFIXES Libraries/Structure/macOS/x86_64)
elseif(UNIX)
  if(X86_64)
    list(APPEND OCCIPITAL_STRUCTURE_LIB_SUFFIXES Libraries/Structure/Linux/x86_64)
  endif()
  if(ARM OR AARCH64)
    list(APPEND OCCIPITAL_STRUCTURE_LIB_SUFFIXES Libraries/Structure/Linux/arm64)
  endif()
else()
  return()
endif()

find_path(OCCIPITAL_STRUCTURE_INCLUDE_DIR
  NAMES ST/CaptureSession.h
  HINTS ${OCCIPITAL_STRUCTURE_SEARCH_HINTS}
  PATHS ${OCCIPITAL_STRUCTURE_SEARCH_PATHS}
  PATH_SUFFIXES Libraries/Structure/Headers
)

find_library(OCCIPITAL_STRUCTURE_LIBRARY
  NAMES Structure
  HINTS ${OCCIPITAL_STRUCTURE_SEARCH_HINTS}
  PATHS ${OCCIPITAL_STRUCTURE_SEARCH_PATHS}
  PATH_SUFFIXES ${OCCIPITAL_STRUCTURE_LIB_SUFFIXES}
)

if(OCCIPITAL_STRUCTURE_LIBRARY AND OCCIPITAL_STRUCTURE_INCLUDE_DIR)
  set(OCCIPITAL_STRUCTURE_INCLUDE_DIRS ${OCCIPITAL_STRUCTURE_INCLUDE_DIR})
  set(OCCIPITAL_STRUCTURE_LIBRARIES ${OCCIPITAL_STRUCTURE_LIBRARY})
  set(OCCIPITAL_STRUCTURE_FOUND TRUE)
else()
  set(OCCIPITAL_STRUCTURE_FOUND FALSE)
endif()

mark_as_advanced(
  OCCIPITAL_STRUCTURE_INCLUDE_DIR
  OCCIPITAL_STRUCTURE_LIBRARY
)
