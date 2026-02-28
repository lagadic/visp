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

set(MyApriltag_INCLUDE_SEARCH_PATHS
  $ENV{apriltag_DIR}/include
  ${apriltag_DIR}/include
  /usr/include
  /usr/local/include
)

set(MyApriltag_LIBRARIES_SEARCH_PATHS
  $ENV{apriltag_DIR}/lib
  $ENV{apriltag_DIR}/lib/x86_64-linux-gnu
  ${apriltag_DIR}/lib
  ${apriltag_DIR}/lib/x86_64-linux-gnu
  /usr/lib
  /usr/local/lib
  /usr/lib/x86_64-linux-gnu
)

set(MyApriltag_CMAKE_CONFIG_SEARCH_PATHS
  $ENV{apriltag_DIR}/share
  ${apriltag_DIR}/share
  /usr/share
  /usr/local/share
  ${MyApriltag_LIBRARIES_SEARCH_PATHS}
)

find_path(MyApriltag_INCLUDE_DIRS apriltag.h PATHS ${MyApriltag_INCLUDE_SEARCH_PATHS} PATH_SUFFIXES apriltag)

find_library(MyApriltag_LIBRARIES NAMES "apriltag" PATHS ${MyApriltag_LIBRARIES_SEARCH_PATHS})

if(MyApriltag_LIBRARIES)
  # try to detect version
  find_path(MyApriltagConfigVersion_DIR apriltagConfigVersion.cmake
    PATHS ${MyApriltag_CMAKE_CONFIG_SEARCH_PATHS}
    PATH_SUFFIXES "cmake/apriltag" "apriltag/cmake")
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

if(MyApriltag_FOUND AND NOT TARGET apriltag::apriltag)
  add_library(apriltag::apriltag UNKNOWN IMPORTED)
  set_target_properties(apriltag::apriltag PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${MyApriltag_INCLUDE_DIRS}"
    IMPORTED_LOCATION "${MyApriltag_LIBRARIES}"
  )
endif()

mark_as_advanced(MyApriltag_INCLUDE_DIRS MyApriltag_LIBRARIES MyApriltag_VERSION)
