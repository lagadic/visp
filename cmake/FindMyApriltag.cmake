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
# Try to find Apriltag library.
# This search engine is only there as a workaround to detect version 3.3.0 of apriltag, known to have a problem
# in the apriltagTarget.cmake file that causes find_package(apriltag) to crash.
#
# MyApriltag_FOUND
# MyApriltag_INCLUDE_DIRS
# MyApriltag_LIBRARIES
# MyApriltag_VERSION_MAJOR
# MyApriltag_VERSION_MINOR
# MyApriltag_VERSION_PATCH
# MyApriltag_VERSION
#
#############################################################################


set(MyApriltag_SEARCH_HINTS
  ${apriltag_DIR}
  $ENV{apriltag_DIR}
  $ENV{CONDA_PREFIX}
)
set(MyApriltag_LIB_SUFFIXES lib)
if(CMAKE_LIBRARY_ARCHITECTURE)
  list(APPEND MyApriltag_LIB_SUFFIXES lib/${CMAKE_LIBRARY_ARCHITECTURE})
endif()

find_path(MyApriltag_INCLUDE_DIRS apriltag.h
  HINTS ${MyApriltag_SEARCH_HINTS}
  PATH_SUFFIXES include/apriltag apriltag
)

find_library(MyApriltag_LIBRARIES
  NAMES apriltag
  HINTS ${MyApriltag_SEARCH_HINTS}
  PATH_SUFFIXES ${MyApriltag_LIB_SUFFIXES}
)

if(MyApriltag_LIBRARIES)
  # Try to detect version from apriltagConfigVersion.cmake, searched from the hints
  # and from the install prefix of the library (<prefix>/lib or <prefix>/lib/<arch>)
  get_filename_component(_apriltag_lib_dir ${MyApriltag_LIBRARIES} DIRECTORY)
  set(MyApriltag_CONFIG_SUFFIXES
    share/apriltag/cmake
    share/cmake/apriltag
    lib/cmake/apriltag
    lib/apriltag/cmake
  )
  if(CMAKE_LIBRARY_ARCHITECTURE)
    list(APPEND MyApriltag_CONFIG_SUFFIXES lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/apriltag)
  endif()
  find_path(MyApriltagConfigVersion_DIR apriltagConfigVersion.cmake
    HINTS
      ${MyApriltag_SEARCH_HINTS}
      ${_apriltag_lib_dir}/..
      ${_apriltag_lib_dir}/../..
    PATH_SUFFIXES ${MyApriltag_CONFIG_SUFFIXES}
  )

  if(MyApriltagConfigVersion_DIR)
    set(MyApriltagConfigVersion_FILE "${MyApriltagConfigVersion_DIR}/apriltagConfigVersion.cmake")
    if(EXISTS ${MyApriltagConfigVersion_FILE})
      file(READ ${MyApriltagConfigVersion_FILE} FILE_CONTENT)
      string(REGEX MATCH "set\\(PACKAGE_VERSION \"([0-9]+)\\.([0-9]+)\\.([0-9]+)\"\\)" _ "${FILE_CONTENT}")

      set(MyApriltag_VERSION_MAJOR ${CMAKE_MATCH_1})
      set(MyApriltag_VERSION_MINOR ${CMAKE_MATCH_2})
      set(MyApriltag_VERSION_PATCH ${CMAKE_MATCH_3})
      set(MyApriltag_VERSION "${MyApriltag_VERSION_MAJOR}.${MyApriltag_VERSION_MINOR}.${MyApriltag_VERSION_PATCH}")

      if(NOT MyApriltag_VERSION)
        set(MyApriltag_VERSION "n/a")
      endif()
    else()
      set(MyApriltag_VERSION "n/a")
    endif()
  else()
    set(MyApriltag_VERSION "n/a")
  endif()
endif()

include(FindPackageHandleStandardArgs)

find_package_handle_standard_args(MyApriltag
  REQUIRED_VARS MyApriltag_LIBRARIES MyApriltag_INCLUDE_DIRS
  VERSION_VAR MyApriltag_VERSION
  FOUND_VAR MyApriltag_FOUND
)

mark_as_advanced(
  MyApriltag_INCLUDE_DIRS
  MyApriltag_LIBRARIES
  MyApriltagConfigVersion_DIR
)
