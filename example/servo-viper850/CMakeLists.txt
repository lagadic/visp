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

project(example-servo-viper850)

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

find_package(VISP REQUIRED visp_core visp_blob visp_vs visp_robot visp_sensor visp_vision visp_gui)

set(example_cpp
  servoViper850FourPointsKinect.cpp
  servoViper850Point2DArtVelocity.cpp
  servoViper850Point2DCamVelocity.cpp
  servoViper850Point2DCamVelocityKalman.cpp
)

if(NOT DEFINED WINRT_8_1)
  list(APPEND example_cpp servoViper850FourPoints2DArtVelocityInteractionCurrent.cpp)
  list(APPEND example_cpp servoViper850FourPoints2DArtVelocityInteractionDesired.cpp)
  list(APPEND example_cpp servoViper850FourPoints2DCamVelocityInteractionCurrent.cpp)
  list(APPEND example_cpp servoViper850Point2DArtVelocity-jointAvoidance-gpa.cpp)
  list(APPEND example_cpp servoViper850Point2DArtVelocity-jointAvoidance-basic.cpp)
  list(APPEND example_cpp servoViper850Point2DArtVelocity-jointAvoidance-large.cpp)
endif()

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()
