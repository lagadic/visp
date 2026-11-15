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
# Try to find libraries for Irisa's Ptu-46 pan-tilt head.
# Once run this will define:
#
# PTU46_FOUND
# PTU46_INCLUDE_DIRS
# PTU46_LIBRARIES
#
#############################################################################


if(NOT UNIX)
  # message("FindPTU46.cmake: Ptu-46 only available for Unix.")
  set(PTU46_FOUND FALSE)
else()
  set(PTU46_SEARCH_HINTS
    ${PTU46_HOME}
    $ENV{PTU46_HOME}
  )
  # Inria specific install locations
  set(PTU46_SEARCH_PATHS
    /udd/fspindle/robot/Ptu-Evi/current
    /local/soft/Ptu-Evi/current
  )

  find_path(PTU46_INCLUDE_DIR ptu.h
    HINTS ${PTU46_SEARCH_HINTS}
    PATHS ${PTU46_SEARCH_PATHS}
    PATH_SUFFIXES include
  )
  #message("DBG PTU46_INCLUDE_DIR=${PTU46_INCLUDE_DIR}")

  find_library(PTUPTU46_LIBRARY
    NAMES ptu
    HINTS ${PTU46_SEARCH_HINTS}
    PATHS ${PTU46_SEARCH_PATHS}
    PATH_SUFFIXES lib
  )
  find_library(EVIPTU46_LIBRARY
    NAMES evi
    HINTS ${PTU46_SEARCH_HINTS}
    PATHS ${PTU46_SEARCH_PATHS}
    PATH_SUFFIXES lib
  )
  find_library(SERIALPTU46_LIBRARY
    NAMES serial
    HINTS ${PTU46_SEARCH_HINTS}
    PATHS ${PTU46_SEARCH_PATHS}
    PATH_SUFFIXES lib
  )

  if(PTU46_INCLUDE_DIR AND PTUPTU46_LIBRARY AND EVIPTU46_LIBRARY AND SERIALPTU46_LIBRARY)
    # The material is found. Check if it works on the requested architecture
    include(CheckCXXSourceCompiles)

    set(CMAKE_REQUIRED_LIBRARIES ${PTUPTU46_LIBRARY} ${SERIALPTU46_LIBRARY})
    set(CMAKE_REQUIRED_INCLUDES ${PTU46_INCLUDE_DIR})
    check_cxx_source_compiles("
      #include <ptu.h> // Contrib for Ptu-46 robot
      int main()
      {
        Ptu ptu;
        return 0;
      }
      " PTU46_BUILD_TEST)
    #message("PTU46_BUILD_TEST: ${PTU46_BUILD_TEST}")

    if(PTU46_BUILD_TEST)
      set(PTU46_INCLUDE_DIRS ${PTU46_INCLUDE_DIR})
      set(PTU46_LIBRARIES ${PTUPTU46_LIBRARY} ${EVIPTU46_LIBRARY} ${SERIALPTU46_LIBRARY})
    endif()
  endif()

  if(PTU46_INCLUDE_DIRS AND PTU46_LIBRARIES)
    set(PTU46_FOUND TRUE)
  else()
    set(PTU46_FOUND FALSE)
  endif()

  mark_as_advanced(
    PTU46_INCLUDE_DIR
    PTUPTU46_LIBRARY
    EVIPTU46_LIBRARY
    SERIALPTU46_LIBRARY
  )
endif()
