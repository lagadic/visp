#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
# Try to find Cerebellum CA linux library material
#
# CALINUX_FOUND
# CALINUX_INCLUDE_DIRS
# CALINUX_LIBRARIES
#
# Authors:
# Fabien Spindler
#
#############################################################################

if(NOT UNIX)
  # message("FindCALINUX.cmake: only available for Unix.")
  set(CALINUX_FOUND FALSE)
else()

  find_path(CALINUX_INCLUDE_DIR calinux.h
    $ENV{CALINUX_HOME}/include
    /local/soft/Cerebellum/CALinux/current/include
    /home/soft/Cerebellum/CALinux/current/include
    /home/soft/cerebellum/CALinux/include
  )
  #message("DBG CALINUX_INCLUDE_DIR=${CALINUX_INCLUDE_DIR}")

  find_library(CALINUX_LIBRARY
    NAMES calinux
    PATHS
      $ENV{CALINUX_HOME}/lib
      /local/soft/Cerebellum/CALinux/current/lib
      /home/soft/Cerebellum/CALinux/current/lib
      /home/soft/cerebellum/CALinux/lib
  )

  #message("DBG CALINUX_LIBRARY=${CALINUX_LIBRARY}")

  ## --------------------------------

  if(CALINUX_LIBRARY)
    set(CALINUX_LIBRARIES ${CALINUX_LIBRARY})
  else()
#   message(SEND_ERROR "Calinux library not found.")
  endif()

  if(NOT CALINUX_INCLUDE_DIR)
#    message(SEND_ERROR "Calinux include dir not found.")
  endif()

  if(CALINUX_LIBRARIES AND CALINUX_INCLUDE_DIR)
    set(CALINUX_INCLUDE_DIRS ${CALINUX_INCLUDE_DIR})
    set(CALINUX_FOUND TRUE)
  else()
    set(CALINUX_FOUND FALSE)
  endif()

  mark_as_advanced(
    CALINUX_INCLUDE_DIR
    CALINUX_LIBRARIES
    CALINUX_LIBRARY
  )
endif()
