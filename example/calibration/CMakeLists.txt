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

project(example-calibration)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_vision visp_gui visp_io)

set(example_cpp
  calibrate-camera.cpp
  calibrate-hand-eye.cpp
)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

# Add test
# To run these tests automaticaly using the crontab under Unix or the task
# manager under windows, we have to add the -d option to turn off the display
# of acquired images
add_test(calibrate-hand-eye calibrate-hand-eye)

# copy the data
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/default-chessboard.cfg" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/default-circles.cfg" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-01.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-02.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-03.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-04.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/chessboard-05.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/circles-01.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/circles-02.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/circles-03.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/circles-04.png" )
list(APPEND data2copy "${CMAKE_CURRENT_SOURCE_DIR}/circles-05.png" )

# Copy the data files to the same location than the target
foreach(data ${data2copy})
  visp_copy_data(calibrate-camera.cpp ${data})
endforeach()
