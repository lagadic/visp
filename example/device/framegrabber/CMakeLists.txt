#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
# ViSP configuration file.
#
#############################################################################

cmake_minimum_required(VERSION 3.10)

project(example-device-framegrabber)

find_package(VISP REQUIRED visp_core visp_sensor visp_io visp_gui)

set(example_cpp
  getRealSense2Info.cpp
  grab1394Two.cpp
  grab1394CMU.cpp
  grabDisk.cpp
  grabDiskFloat.cpp
  grabV4l2.cpp
  grabDirectShow.cpp
  grabDirectShowMulti.cpp
  grabFlyCapture.cpp
  grabRealSense2.cpp
  grabRealSense2_T265.cpp
  grabV4l2MultiCpp11Thread.cpp
  readRealSenseData.cpp
  saveRealSenseData.cpp
)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()
