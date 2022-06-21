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
# Try to find Vicon SDK
# Once run this will define:
#
# VICON_FOUND
# VICON_INCLUDE_DIRS
# VICON_LIBRARIES
# VICON_VERSION
#
#############################################################################

find_path(VICON_INCLUDE_DIR
  NAMES DataStreamClient.h
  PATHS
    $ENV{VICON_DIR}
    ${VICON_DIR}
    /usr/include
    /usr/local/include
)

find_library(VICON_LIBRARY
  NAMES ViconDataStreamSDK_CPP
  PATHS
    $ENV{VICON_DIR}
    ${VICON_DIR}
    $ENV{VICON_DIR}/bin/Release
    ${VICON_DIR}/bin/Release
    /usr/lib
    /usr/local/lib
    )

if(VICON_LIBRARY AND VICON_INCLUDE_DIR)
    set(VICON_INCLUDE_DIRS ${VICON_INCLUDE_DIR})
    set(VICON_LIBRARIES ${VICON_LIBRARY})
    set(VICON_FOUND TRUE)
else()
    set(VICON_FOUND FALSE)
endif()
