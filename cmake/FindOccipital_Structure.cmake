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
# Try to find Occipital Structure SDK to work with Structure Core device.
# Once run this will define:
#
# OCCIPITAL_STRUCTURE_FOUND
# OCCIPITAL_STRUCTURE_INCLUDE_DIRS
# OCCIPITAL_STRUCTURE_LIBRARIES
#
#############################################################################

set(OCCIPITAL_STRUCTURE_INC_SEARCH_PATH /usr/local/include)
set(OCCIPITAL_STRUCTURE_LIB_SEARCH_PATH /usr/local/lib)

if(MSVC)
#  list(APPEND OCCIPITAL_STRUCTURE_INC_SEARCH_PATH "C:/libST/Libraries/Structure/Headers/ST")

  list(APPEND OCCIPITAL_STRUCTURE_INC_SEARCH_PATH $ENV{OCCIPITAL_STRUCTURE_HOME}/Libraries/Structure/Headers/)
  list(APPEND OCCIPITAL_STRUCTURE_INC_SEARCH_PATH $ENV{OCCIPITAL_STRUCTURE_DIR}/Libraries/Structure/Headers/)
  list(APPEND OCCIPITAL_STRUCTURE_INC_SEARCH_PATH "C:/Program Files (x86)/Structure SDK/Libraries/Structure/Headers/")

  list(APPEND OCCIPITAL_STRUCTURE_LIB_SEARCH_PATH $ENV{OCCIPITAL_STRUCTURE_HOME}/Libraries/Structure/Windows/x86_64/)
  list(APPEND OCCIPITAL_STRUCTURE_LIB_SEARCH_PATH $ENV{OCCIPITAL_STRUCTURE_DIR}/Libraries/Structure/Windows/x86_64/)

#  if(CMAKE_CL_64)
#    list(APPEND OCCIPITAL_STRUCTURE_LIB_SEARCH_PATH "C:/Program Files (x86)/Structure SDK/Libraries/Structure/Libraries/Windows/x86_64")
#  else()
#    list(APPEND OCCIPITAL_STRUCTURE_LIB_SEARCH_PATH "C:/Program Files (x86)/Structure SDK/Libraries/Structure/Libraries/Windows/x86")
#  endif()
else()
  list(APPEND OCCIPITAL_STRUCTURE_INC_SEARCH_PATH /usr/include)
  list(APPEND OCCIPITAL_STRUCTURE_LIB_SEARCH_PATH /usr/lib)

  list(APPEND OCCIPITAL_STRUCTURE_INC_SEARCH_PATH $ENV{OCCIPITAL_STRUCTURE_HOME}/Libraries/Structure/Headers/)
  list(APPEND OCCIPITAL_STRUCTURE_LIB_SEARCH_PATH $ENV{OCCIPITAL_STRUCTURE_HOME}/Libraries/Structure/Linux/x86_64/)
  list(APPEND OCCIPITAL_STRUCTURE_INC_SEARCH_PATH $ENV{OCCIPITAL_STRUCTURE_DIR}/Libraries/Structure/Headers/)
  list(APPEND OCCIPITAL_STRUCTURE_LIB_SEARCH_PATH $ENV{OCCIPITAL_STRUCTURE_DIR}/Libraries/Structure/Linux/x86_64)
  list(APPEND OCCIPITAL_STRUCTURE_INC_SEARCH_PATH ${OCCIPITAL_STRUCTURE_DIR}/Libraries/Structure/Headers/)
  list(APPEND OCCIPITAL_STRUCTURE_LIB_SEARCH_PATH ${OCCIPITAL_STRUCTURE_DIR}/Libraries/Structure/Linux)
endif()

find_path(OCCIPITAL_STRUCTURE_INCLUDE_DIRS
  NAMES ST/CaptureSession.h
  PATHS
    ${OCCIPITAL_STRUCTURE_INC_SEARCH_PATH}
)

find_library(OCCIPITAL_STRUCTURE_LIBRARIES
  NAMES libStructure.so
  PATHS
    ${OCCIPITAL_STRUCTURE_LIB_SEARCH_PATH}
)

if(OCCIPITAL_STRUCTURE_LIBRARIES AND OCCIPITAL_STRUCTURE_INCLUDE_DIRS)
  set(OCCIPITAL_STRUCTURE_FOUND TRUE)
else()
  set(OCCIPITAL_STRUCTURE_FOUND FALSE)
endif()

mark_as_advanced(
  OCCIPITAL_STRUCTURE_INCLUDE_DIRS
  OCCIPITAL_STRUCTURE_LIBRARIES
  OCCIPITAL_STRUCTURE_INC_SEARCH_PATH
  OCCIPITAL_STRUCTURE_LIB_SEARCH_PATH
)
