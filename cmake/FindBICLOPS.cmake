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
# Try to find libBiclops, libPMD and libUtils for Biclops head.
# Once run this will define:
#
# BICLOPS_FOUND
# BICLOPS_INCLUDE_DIRS
# BICLOPS_LIBRARIES
#
#############################################################################


if(NOT UNIX AND NOT WIN32)
  # message("FindBICLOPS.cmake: macro only for Unix and Windows for the moment.")
  set(BICLOPS_FOUND FALSE)
else()
  set(BICLOPS_SEARCH_HINTS
    ${BICLOPS_HOME}
    $ENV{BICLOPS_HOME}
  )
  set(PMD_SEARCH_HINTS
    ${BICLOPS_SEARCH_HINTS}
    ${BICLOPS_PMD_HOME}
    $ENV{BICLOPS_PMD_HOME}
  )
  set(UTILS_SEARCH_HINTS
    ${BICLOPS_SEARCH_HINTS}
    ${BICLOPS_UTILS_HOME}
    $ENV{BICLOPS_UTILS_HOME}
  )

  find_path(BICLOPS_INCLUDE_DIR Biclops.h
    HINTS ${BICLOPS_SEARCH_HINTS}
    PATH_SUFFIXES include
  )
  #message("DBG BICLOPS_INCLUDE_DIR=${BICLOPS_INCLUDE_DIR}")

  find_path(PMD_INCLUDE_DIR PMD.h
    HINTS ${PMD_SEARCH_HINTS}
    PATH_SUFFIXES include
  )

  find_path(UTILS_INCLUDE_DIR utility.h
    HINTS ${UTILS_SEARCH_HINTS}
    PATH_SUFFIXES include
  )

  find_library(BICLOPS_LIBRARY
    NAMES Biclops libBiclops libBiclopsD
    HINTS ${BICLOPS_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )

  find_library(PMD_LIBRARY
    NAMES PMD libPMD libPMDD
    HINTS ${PMD_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )

  find_library(UTILS_LIBRARY
    NAMES Utils libUtils libUtilsD
    HINTS ${UTILS_SEARCH_HINTS}
    PATH_SUFFIXES lib
  )
  #message("DBG BICLOPS_LIBRARY=${BICLOPS_LIBRARY}")

  if(BICLOPS_LIBRARY AND PMD_LIBRARY AND UTILS_LIBRARY)
    set(BICLOPS_LIBRARIES ${BICLOPS_LIBRARY} ${PMD_LIBRARY} ${UTILS_LIBRARY})
  endif()

  if(BICLOPS_INCLUDE_DIR AND PMD_INCLUDE_DIR AND UTILS_INCLUDE_DIR)
    set(BICLOPS_INCLUDE_DIRS ${BICLOPS_INCLUDE_DIR} ${PMD_INCLUDE_DIR} ${UTILS_INCLUDE_DIR})
  endif()

  if(BICLOPS_LIBRARIES AND BICLOPS_INCLUDE_DIRS)
    #message("BICLOPS_LIBRARIES: ${BICLOPS_LIBRARIES}")
    #message("BICLOPS_INCLUDE_DIRS: ${BICLOPS_INCLUDE_DIRS}")

    # Try to compile a sample code using Biclops library to see if GetHomedState() is available
    include(CheckCXXSourceCompiles)

    set(CMAKE_REQUIRED_LIBRARIES ${BICLOPS_LIBRARIES})
    set(CMAKE_REQUIRED_INCLUDES  ${BICLOPS_INCLUDE_DIRS})
    check_cxx_source_compiles("
      #include <PMDUtils.h>
      int main(){
        PMDAxisControl *axis;
        axis->GetHomedState();
        return 0;
      }
     " BICLOPS_HAVE_GET_HOMED_STATE_FUNCTION)

    #message("DBG1 BICLOPS_HAVE_GET_HOMED_STATE_FUNCTION ${BICLOPS_HAVE_GET_HOMED_STATE_FUNCTION}")

    set(BICLOPS_FOUND TRUE)
  else()
    set(BICLOPS_FOUND FALSE)
  endif()

  mark_as_advanced(
    BICLOPS_INCLUDE_DIR
    BICLOPS_LIBRARIES
    BICLOPS_LIBRARY
    PMD_INCLUDE_DIR
    UTILS_INCLUDE_DIR
    PMD_LIBRARY
    UTILS_LIBRARY
  )
endif()
