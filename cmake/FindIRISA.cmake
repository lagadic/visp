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
# Try to find Cerebellum Irisa library API to dial with the robot Afma6
#
# IRISA_FOUND
# IRISA_INCLUDE_DIRS
# IRISA_LIBRARIES
#
#############################################################################


if(NOT UNIX)
  # message("FindIRISA.cmake: only available for Unix.")
  set(IRISA_FOUND FALSE)
else()
  set(IRISA_SEARCH_HINTS
    ${IRISA_HOME}
    $ENV{IRISA_HOME}
  )
  # Inria specific install locations
  set(IRISA_SEARCH_PATHS
    /local/soft/Cerebellum/Irisa/current
    /home/soft/Cerebellum/Irisa/current
    /home/soft/cerebellum/Irisa
  )

  find_path(IRISA_INCLUDE_DIR irisa_Afma6.h
    HINTS ${IRISA_SEARCH_HINTS}
    PATHS ${IRISA_SEARCH_PATHS}
    PATH_SUFFIXES src
  )
  #message("DBG IRISA_INCLUDE_DIR=${IRISA_INCLUDE_DIR}")

  find_library(IRISA_LIBRARY
    NAMES irisa
    HINTS ${IRISA_SEARCH_HINTS}
    PATHS ${IRISA_SEARCH_PATHS}
    PATH_SUFFIXES src
  )
  #message("DBG IRISA_LIBRARY=${IRISA_LIBRARY}")

  if(IRISA_LIBRARY AND IRISA_INCLUDE_DIR)
    set(IRISA_INCLUDE_DIRS ${IRISA_INCLUDE_DIR})
    set(IRISA_LIBRARIES ${IRISA_LIBRARY})
    set(IRISA_FOUND TRUE)
  else()
    set(IRISA_FOUND FALSE)
  endif()

  mark_as_advanced(
    IRISA_INCLUDE_DIR
    IRISA_LIBRARY
  )
endif()
