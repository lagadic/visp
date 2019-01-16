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

project(example-device-framegrabber)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_sensor visp_io visp_gui)

set(example_cpp
  getRealSense2Info.cpp
  grab1394Two.cpp
  grab1394CMU.cpp
  grabDisk.cpp
  grabV4l2.cpp
  grabDirectShow.cpp
  grabDirectShowMulti.cpp
  grabFlyCapture.cpp
  grabOpenCV.cpp
  grabRealSense.cpp
  grabRealSense2.cpp
  grabV4l2MultiCpp11Thread.cpp
  readRealSenseData.cpp
  saveRealSenseData.cpp
)

if(VISP_HAVE_FLYCAPTURE)
  # Add specific build flag to turn off warnings coming from PointGrey flycapture 3rd party
  visp_set_source_file_compile_flag(grabFlyCapture.cpp -Wno-unknown-pragmas -Wno-ignored-qualifiers)
endif()

if(VISP_HAVE_REALSENSE OR VISP_HAVE_REALSENSE2)
  if(UNIX)
    visp_set_source_file_compile_flag(getRealSense2Info.cpp -Wno-unused-function -Wno-sign-compare -Wno-reorder -Wno-overloaded-virtual)
    # Add specific build flag to turn off warnings coming from PCL 3rd party
    visp_set_source_file_compile_flag(grabRealSense.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-sign-conversion -Wno-float-equal -Wno-unused-parameter -Wno-pessimizing-move -Wno-comment)
    visp_set_source_file_compile_flag(grabRealSense2.cpp -Wno-sign-compare -Wno-reorder -Wno-unused-function -Wno-pessimizing-move -Wno-unused-parameter -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-comment -Wno-overloaded-virtual)
    visp_set_source_file_compile_flag(readRealSenseData.cpp -Wno-deprecated-declarations -Wno-inconsistent-missing-override -Wno-missing-field-initializers)
    visp_set_source_file_compile_flag(saveRealSenseData.cpp -Wno-sign-compare -Wno-reorder -Wno-unused-function -Wno-pessimizing-move -Wno-unused-parameter -Wno-comment -Wno-overloaded-virtual)
  else()
    visp_set_source_file_compile_flag(getRealSense2Info.cpp /wd4244)
    visp_set_source_file_compile_flag(grabRealSense2.cpp /wd4244)
    visp_set_source_file_compile_flag(saveRealSenseData.cpp /wd4244)
  endif()
else()
  visp_set_source_file_compile_flag(readRealSenseData.cpp -Wno-deprecated-declarations -Wno-comment)
endif()

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()
