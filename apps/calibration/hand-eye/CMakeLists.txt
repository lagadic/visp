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
# ViSP overall configuration file.
#
#############################################################################

cmake_minimum_required(VERSION 3.10)

project(apps-calibration-hand-eye)

find_package(VISP REQUIRED visp_vision visp_io OPTIONAL_COMPONENTS visp_detection visp_gui visp_robot visp_sensor)

set(apps_cpp
  visp-compute-chessboard-poses.cpp
  visp-compute-eye-in-hand-calibration.cpp
  visp-compute-eye-to-hand-calibration.cpp
)

if(VISP_HAVE_FRANKA)
  list(APPEND apps_cpp visp-acquire-franka-calib-data.cpp)
endif()

if(VISP_HAVE_UR_RTDE)
  list(APPEND apps_cpp visp-acquire-universal-robots-calib-data.cpp)
endif()

foreach(cpp ${apps_cpp})
  visp_add_app(${cpp})
endforeach()

if(VISP_DETECTION_FOUND)
  visp_add_app(visp-compute-apriltag-poses.cpp)
endif()

# Copy the data folder to the same location than the target
visp_copy_dir(visp-compute-eye-in-hand-calibration.cpp "${CMAKE_CURRENT_SOURCE_DIR}" data-eye-in-hand)
visp_copy_dir(visp-compute-eye-to-hand-calibration.cpp "${CMAKE_CURRENT_SOURCE_DIR}" data-eye-to-hand)
