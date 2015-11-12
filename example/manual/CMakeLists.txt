#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2015 by Inria. All rights reserved.
#
# This software is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# ("GPL") version 2 as published by the Free Software Foundation.
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
# ViSP manual examples build. 
#
# Authors:
# Fabien Spindler
#
#############################################################################

project(example-manual)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_sensor visp_vs visp_robot visp_ar visp_vision visp_io visp_gui)

set(example_cpp
  geometric-features/manGeometricFeatures.cpp
  hello-world/CMake/HelloWorld.cpp
  image-manipulation/manDisplay.cpp
  image-manipulation/manGrab1394-2.cpp
  image-manipulation/manGrabDirectShow.cpp
  image-manipulation/manGrabDisk.cpp
  image-manipulation/manGrabV4l2.cpp
  ogre/HelloWorldOgre.cpp
  ogre/HelloWorldOgreAdvanced.cpp
  simulation/manServo4PointsDisplay.cpp
  simulation/manSimu4Dots.cpp
  simulation/manSimu4Points.cpp
  moments/manServoMomentsSimple.cpp
)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()
