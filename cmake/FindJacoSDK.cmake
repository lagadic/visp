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
# Try to find Kinova Jaco SDK
#
# JACOSDK_FOUND
# JACOSDK_INCLUDE_DIRS
# JACOSDK_LIBRARIES
#
#############################################################################


# Layouts handled:
#   <JACOSDK_HOME>
#   <prefix>/JACO-SDK/API for each standard prefix (e.g. /usr/JACO-SDK/API)
#   /opt/JACO-SDK/API, C:/Program Files (x86)/JACO-SDK/API
set(JACOSDK_SEARCH_HINTS
  ${JACOSDK_HOME}
  $ENV{JACOSDK_HOME}
)
set(JACOSDK_SEARCH_PATHS
  /opt
  "C:/Program Files (x86)"
)

find_path(JACOSDK_INCLUDE_DIR
  NAMES CommandLayer.h CommunicationLayer.h KinovaTypes.h
  HINTS ${JACOSDK_SEARCH_HINTS}
  PATHS ${JACOSDK_SEARCH_PATHS}
  PATH_SUFFIXES JACO-SDK/API
)

if(JACOSDK_INCLUDE_DIR)
  set(JACOSDK_INCLUDE_DIRS ${JACOSDK_INCLUDE_DIR})
  if(UNIX)
    set(JACOSDK_LIBRARIES ${CMAKE_DL_LIBS})
  else()
    set(JACOSDK_LIBRARIES "")
  endif()
  set(JACOSDK_FOUND TRUE)
else()
  set(JACOSDK_FOUND FALSE)
endif()

mark_as_advanced(
  JACOSDK_INCLUDE_DIR
)
