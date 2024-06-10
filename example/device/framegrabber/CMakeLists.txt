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
# ViSP configuration file.
#
#############################################################################

cmake_minimum_required(VERSION 3.5)

project(example-device-framegrabber)

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
  grabRealSense2.cpp
  grabRealSense2_T265.cpp
  grabV4l2MultiCpp11Thread.cpp
  readRealSenseData.cpp
  saveRealSenseData.cpp
)

if(VISP_HAVE_FLYCAPTURE)
  # Add specific build flag to turn off warnings coming from PointGrey flycapture 3rd party
  visp_set_source_file_compile_flag(grabFlyCapture.cpp -Wno-unknown-pragmas -Wno-ignored-qualifiers)
endif()

if(VISP_HAVE_REALSENSE OR VISP_HAVE_REALSENSE2)
  # Add specific build flag to turn off warnings coming from PCL 3rd party
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4244")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "/wd4267")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-comment")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-copy")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-declarations")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-missing-field-initializers")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-float-equal")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-inconsistent-missing-override")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-overloaded-virtual")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-pessimizing-move")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-reorder")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-shadow")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-sign-compare")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-sign-conversion")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-function")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unused-parameter")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unqualified-std-cast-call")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-missing-field-initializers")

  visp_set_source_file_compile_flag(getRealSense2Info.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  visp_set_source_file_compile_flag(grabRealSense2.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  visp_set_source_file_compile_flag(grabRealSense2_T265.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  visp_set_source_file_compile_flag(readRealSenseData.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  visp_set_source_file_compile_flag(saveRealSenseData.cpp ${CXX_FLAGS_MUTE_WARNINGS})
endif()

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()
