#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
# Try to find Rapa Pololu Maestro library material
# See https://github.com/jbitoniau/RapaPololuMaestro
#
# RapaPololuMaestro_FOUND
# RapaPololuMaestro_INCLUDE_DIRS
# RapaPololuMaestro_LIBRARIES
#
#############################################################################

find_path(RapaPololuMaestro_INCLUDE_DIR RPMSerialInterface.h
  PATHS
    $ENV{RapaPololuMaestro_DIR}/include
    ${RapaPololuMaestro_DIR}/include
    /usr/include
    /usr/local/include
)

find_library(RapaPololuMaestro_LIBRARY
  NAMES RapaPololuMaestro
  PATHS
    $ENV{RapaPololuMaestro_DIR}/lib
    ${RapaPololuMaestro_DIR}/lib
    /usr/lib
    /usr/local/lib
)

if(RapaPololuMaestro_INCLUDE_DIR AND RapaPololuMaestro_LIBRARY)
  set(RapaPololuMaestro_FOUND TRUE)
  set(RapaPololuMaestro_INCLUDE_DIRS ${RapaPololuMaestro_INCLUDE_DIR})
  set(RapaPololuMaestro_LIBRARIES ${RapaPololuMaestro_LIBRARY})
else()
  set(RapaPololuMaestro_FOUND FALSE)
endif()

mark_as_advanced(
  RapaPololuMaestro_INCLUDE_DIR
  RapaPololuMaestro_LIBRARY
  RapaPololuMaestro_INCLUDE_DIRS
  RapaPololuMaestro_LIBRARIES
  )
