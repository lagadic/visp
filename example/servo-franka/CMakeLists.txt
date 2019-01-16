#############################################################################
#
# ViSP, open source Visual Servoing Platform software.
# Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
# ViSP configuration file.
#
# Authors:
# Fabien Spindler
#
#############################################################################

project(example-servo-franka)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_robot visp_vision visp_io visp_gui visp_vs visp_visual_features visp_detection)

set(example_cpp
  frankaGripper.cpp
  frankaMoveToPosition.cpp
  frankaSavePosition.cpp
  servoFrankaPBVS.cpp
)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

if(USE_REALSENSE2)
  if(UNIX)
    # Add specific build flag to turn off warnings coming from PCL 3rd party
    vp_set_source_file_compile_flag(servoFrankaPBVS.cpp -Wno-unused-function -Wno-reorder -Wno-sign-compare -Wno-overloaded-virtual)
  else()
    visp_set_source_file_compile_flag(servoFrankaPBVS.cpp /wd4244)
  endif()
endif()
