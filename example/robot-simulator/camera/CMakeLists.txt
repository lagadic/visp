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

project(example-robot-simulator-camera)

cmake_minimum_required(VERSION 2.6)

if(CMAKE_SYSTEM_NAME MATCHES WindowsPhone OR CMAKE_SYSTEM_NAME MATCHES WindowsStore)
  set(WINRT TRUE)
endif(CMAKE_SYSTEM_NAME MATCHES WindowsPhone OR CMAKE_SYSTEM_NAME MATCHES WindowsStore)

if(WINRT)
  add_definitions(-DWINRT)

  if(CMAKE_SYSTEM_NAME MATCHES WindowsPhone)
    set(WINRT_PHONE TRUE)
    add_definitions(-DWINRT_PHONE)
  elseif(CMAKE_SYSTEM_NAME MATCHES WindowsStore)
    set(WINRT_STORE TRUE)
    add_definitions(-DWINRT_STORE)
  endif()

  if(CMAKE_SYSTEM_VERSION MATCHES 10 OR CMAKE_SYSTEM_VERSION MATCHES 10.0)
    set(WINRT_10 TRUE)
    add_definitions(-DWINRT_10)
  elseif(CMAKE_SYSTEM_VERSION MATCHES 8.1)
    set(WINRT_8_1 TRUE)
    add_definitions(-DWINRT_8_1)
  elseif(CMAKE_SYSTEM_VERSION MATCHES 8.0)
    set(WINRT_8_0 TRUE)
    add_definitions(-DWINRT_8_0)
  endif()
endif()

find_package(VISP REQUIRED visp_core visp_vs visp_robot visp_io visp_gui)

set(example_cpp
  servoSimu3D_cdMc_CamVelocity.cpp
  servoSimu3D_cMcd_CamVelocity.cpp
  servoSimuCircle2DCamVelocity.cpp
  servoSimuCircle2DCamVelocityDisplay.cpp
  servoSimuCylinder2DCamVelocityDisplay.cpp
  servoSimuFourPoints2DCamVelocity.cpp
  servoSimuFourPoints2DCamVelocityDisplay.cpp
  servoSimuLine2DCamVelocityDisplay.cpp
  servoSimuPoint2DCamVelocity1.cpp
  servoSimuPoint2DCamVelocity2.cpp
  servoSimuPoint2DCamVelocity3.cpp
  servoSimuPoint2DhalfCamVelocity1.cpp
  servoSimuPoint2DhalfCamVelocity2.cpp
  servoSimuPoint2DhalfCamVelocity3.cpp
  servoSimuPoint3DCamVelocity.cpp
  servoSimuSphere2DCamVelocity.cpp
  servoSimuSphere2DCamVelocityDisplay.cpp
  servoSimuSquareLine2DCamVelocityDisplay.cpp
  servoSimuThetaUCamVelocity.cpp
)

if(NOT DEFINED WINRT_8_1)
  list(APPEND example_cpp servoSimu3D_cdMc_CamVelocityWithoutVpServo.cpp)
  list(APPEND example_cpp servoSimu3D_cMcd_CamVelocityWithoutVpServo.cpp)
  list(APPEND example_cpp servoSimuCylinder2DCamVelocityDisplaySecondaryTask.cpp)
  list(APPEND example_cpp servoSimuSphere2DCamVelocityDisplaySecondaryTask.cpp)
  list(APPEND example_cpp servoSimuFourPoints2DPolarCamVelocityDisplay.cpp)
endif()

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

# Add test
add_test(servoSimu3D_cdMc_CamVelocity			servoSimu3D_cdMc_CamVelocity			)
add_test(servoSimu3D_cdMc_CamVelocityWithoutVpServo	servoSimu3D_cdMc_CamVelocityWithoutVpServo	)
add_test(servoSimu3D_cMcd_CamVelocity			servoSimu3D_cMcd_CamVelocity			)
add_test(servoSimu3D_cMcd_CamVelocityWithoutVpServo	servoSimu3D_cMcd_CamVelocityWithoutVpServo	)
add_test(servoSimuCircle2DCamVelocity			servoSimuCircle2DCamVelocity			)
add_test(servoSimuCircle2DCamVelocityDisplay		servoSimuCircle2DCamVelocityDisplay	-c ${OPTION_TO_DESACTIVE_DISPLAY}	)
add_test(servoSimuCylinder2DCamVelocityDisplay		servoSimuCylinder2DCamVelocityDisplay	-c ${OPTION_TO_DESACTIVE_DISPLAY}	)
add_test(servoSimuFourPoints2DCamVelocity		servoSimuFourPoints2DCamVelocity		)
add_test(servoSimuFourPoints2DCamVelocityDisplay	servoSimuFourPoints2DCamVelocityDisplay -c ${OPTION_TO_DESACTIVE_DISPLAY}	)
add_test(servoSimuLine2DCamVelocityDisplay		servoSimuLine2DCamVelocityDisplay	-c ${OPTION_TO_DESACTIVE_DISPLAY}	)
add_test(servoSimuPoint2DCamVelocity1			servoSimuPoint2DCamVelocity1			)
add_test(servoSimuPoint2DCamVelocity2			servoSimuPoint2DCamVelocity2			)
add_test(servoSimuPoint2DCamVelocity3			servoSimuPoint2DCamVelocity3			)
add_test(servoSimuPoint2DhalfCamVelocity1		servoSimuPoint2DhalfCamVelocity1		)
add_test(servoSimuPoint2DhalfCamVelocity2		servoSimuPoint2DhalfCamVelocity2		)
add_test(servoSimuPoint2DhalfCamVelocity3		servoSimuPoint2DhalfCamVelocity3		)
add_test(servoSimuPoint3DCamVelocity			servoSimuPoint3DCamVelocity			)
add_test(servoSimuSphere2DCamVelocity			servoSimuSphere2DCamVelocity			)
add_test(servoSimuSphere2DCamVelocityDisplay		servoSimuSphere2DCamVelocityDisplay	-c ${OPTION_TO_DESACTIVE_DISPLAY}	)
add_test(servoSimuSphere2DCamVelocityDisplaySecondaryTask servoSimuSphere2DCamVelocityDisplaySecondaryTask -c ${OPTION_TO_DESACTIVE_DISPLAY}	)
add_test(servoSimuThetaUCamVelocity			servoSimuThetaUCamVelocity			)
