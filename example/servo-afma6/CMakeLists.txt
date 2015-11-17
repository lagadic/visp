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
# ViSP configuration file. 
#
# Authors:
# Fabien Spindler
#
#############################################################################

project(example-servo-afma6)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_core visp_blob visp_vs visp_robot visp_sensor visp_vision visp_gui)

set(example_cpp
  servoAfma6Ellipse2DCamVelocity.cpp
  servoAfma6FourPoints2DArtVelocity.cpp
  servoAfma6FourPoints2DCamVelocityInteractionDesired.cpp
  servoAfma6FourPoints2DCamVelocityInteractionCurrent.cpp
  servoAfma6Line2DCamVelocity.cpp
  servoAfma6Point2DArtVelocity.cpp
  servoAfma6Point2DCamVelocity.cpp
  servoAfma6Points2DCamVelocityEyeToHand.cpp
  servoAfma6SquareLines2DCamVelocity.cpp
  servoAfma6TwoLines2DCamVelocity.cpp
  servoAfma62DhalfCamVelocity.cpp
  servoAfma6Cylinder2DCamVelocity.cpp
  servoAfma6Cylinder2DCamVelocitySecondaryTask.cpp
  servoAfma6Segment2DCamVelocity.cpp
)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()
