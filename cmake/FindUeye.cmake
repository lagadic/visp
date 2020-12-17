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
# Try to find IDS uEye Software Suite.
# Once run this will define:
#
# UEYE_FOUND
# UEYE_INCLUDE_DIRS
# UEYE_LIBRARIES
# UEYE_VERSION
#
#############################################################################

set(UEYE_INC_SEARCH_PATH)
set(UEYE_LIB_SEARCH_PATH)

list(APPEND UEYE_INC_SEARCH_PATH $ENV{UEYE_HOME}/include)
list(APPEND UEYE_LIB_SEARCH_PATH $ENV{UEYE_HOME}/lib)
list(APPEND UEYE_INC_SEARCH_PATH $ENV{UEYE_DIR}/include)
list(APPEND UEYE_LIB_SEARCH_PATH $ENV{UEYE_DIR}/lib)
list(APPEND UEYE_INC_SEARCH_PATH ${UEYE_DIR}/include)
list(APPEND UEYE_LIB_SEARCH_PATH ${UEYE_DIR}/lib)


if(MSVC)
  list(APPEND UEYE_INC_SEARCH_PATH "C:/Program Files/IDS/uEye/Develop/include")
  list(APPEND UEYE_LIB_SEARCH_PATH "C:/Program Files/IDS/uEye/Develop/lib")
else()
  list(APPEND UEYE_INC_SEARCH_PATH /usr/include)
  list(APPEND UEYE_LIB_SEARCH_PATH /usr/lib)
endif()

find_path(UEYE_INCLUDE_DIRS ueye.h
  PATHS
    ${UEYE_INC_SEARCH_PATH}
)

set(lib_suffix)
if(MSVC)
  if(CMAKE_CL_64)
    set(lib_suffix "_64")
  endif()
endif()

find_library(UEYE_LIBRARY_API
  NAMES ueye_api${lib_suffix}
  PATHS
    ${UEYE_LIB_SEARCH_PATH}
)

if(UEYE_LIBRARY_API AND UEYE_INCLUDE_DIRS)
  set(UEYE_FOUND TRUE)
  set(UEYE_LIBRARIES ${UEYE_LIBRARY_API})
  vp_parse_header4(UEYE "${UEYE_INCLUDE_DIRS}/ueye.h" "UEYE_VERSION_CODE" UEYE_VERSION)
else()
  set(UEYE_FOUND FALSE)
endif()

mark_as_advanced(
  UEYE_INCLUDE_DIRS
  UEYE_LIBRARIES
  UEYE_LIBRARY_API
  UEYE_INC_SEARCH_PATH
  UEYE_LIB_SEARCH_PATH
)
