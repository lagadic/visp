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

project(example-servo-afma6)

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

find_package(VISP REQUIRED visp_core visp_blob visp_detection visp_vs visp_robot visp_sensor visp_vision visp_gui)

set(example_cpp
  servoAfma6Ellipse2DCamVelocity.cpp
  servoAfma6FourPoints2DArtVelocity.cpp
  servoAfma6AprilTagIBVS.cpp
  servoAfma6AprilTagPBVS.cpp
  servoAfma6Line2DCamVelocity.cpp
  servoAfma6Point2DArtVelocity.cpp
  servoAfma6Point2DCamVelocity.cpp
  servoAfma6Points2DCamVelocityEyeToHand.cpp
  servoAfma6SquareLines2DCamVelocity.cpp
  servoAfma6TwoLines2DCamVelocity.cpp
  servoAfma62DhalfCamVelocity.cpp
  servoAfma6Cylinder2DCamVelocity.cpp
  servoAfma6Segment2DCamVelocity.cpp
  servoAfma6MegaposePBVS.cpp
)

if(NOT DEFINED WINRT_8_1)
  list(APPEND example_cpp servoAfma6FourPoints2DCamVelocityLs_des.cpp)
  list(APPEND example_cpp servoAfma6FourPoints2DCamVelocityLs_cur.cpp)
  list(APPEND example_cpp servoAfma6Cylinder2DCamVelocitySecondaryTask.cpp)
endif()

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

if(VISP_HAVE_REALSENSE2)
  # Add specific build flag to turn off warnings coming from librealsense 3rd party
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-copy-with-user-provided-copy")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-deprecated-declarations")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-ignored-qualifiers")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-sign-compare")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-unqualified-std-cast-call")
  list(APPEND CXX_FLAGS_MUTE_WARNINGS "-Wno-missing-field-initializers")

  visp_set_source_file_compile_flag(servoAfma6AprilTagIBVS.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  visp_set_source_file_compile_flag(servoAfma6AprilTagPBVS.cpp ${CXX_FLAGS_MUTE_WARNINGS})
  visp_set_source_file_compile_flag(servoAfma6MegaposePBVS.cpp ${CXX_FLAGS_MUTE_WARNINGS})
endif()
