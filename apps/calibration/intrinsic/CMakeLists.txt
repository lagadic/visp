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

project(apps-calibration-intrinsic)

find_package(VISP REQUIRED visp_core visp_vision visp_gui visp_io)

set(apps_cpp
  visp-calibrate-camera.cpp
)

foreach(cpp ${apps_cpp})
  visp_add_app(${cpp})
endforeach()

# copy the data
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/default-chessboard.cfg")
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/default-circles.cfg")
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/camera_calibration_show_extrinsics.py")

# Copy the data files to the same location than the target
foreach(data ${data2copy})
  visp_copy_data(visp-calibrate-camera.cpp ${data})
endforeach()

visp_copy_dir(visp-calibrate-camera.cpp "${CMAKE_CURRENT_SOURCE_DIR}" data-chessboard)
visp_copy_dir(visp-calibrate-camera.cpp "${CMAKE_CURRENT_SOURCE_DIR}" data-circles)
