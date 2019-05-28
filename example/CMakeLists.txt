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
# ViSP overall configuration file. 
#
# Authors:
# Fabien Spindler
#
#############################################################################

project(ViSP-examples)

cmake_minimum_required(VERSION 2.6)

find_package(VISP)

if(MSVC)
  if(NOT VISP_SHARED)
    foreach(flag_var
            CMAKE_C_FLAGS CMAKE_C_FLAGS_DEBUG CMAKE_C_FLAGS_RELEASE
            CMAKE_C_FLAGS_MINSIZEREL CMAKE_C_FLAGS_RELWITHDEBINFO
            CMAKE_CXX_FLAGS CMAKE_CXX_FLAGS_DEBUG CMAKE_CXX_FLAGS_RELEASE
            CMAKE_CXX_FLAGS_MINSIZEREL CMAKE_CXX_FLAGS_RELWITHDEBINFO)
      if(${flag_var} MATCHES "/MD")
        string(REGEX REPLACE "/MD" "/MT" ${flag_var} "${${flag_var}}")
      endif()
      if(${flag_var} MATCHES "/MDd")
        string(REGEX REPLACE "/MDd" "/MTd" ${flag_var} "${${flag_var}}")
      endif()
    endforeach(flag_var)

    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /NODEFAULTLIB:atlthunk.lib /NODEFAULTLIB:msvcrt.lib /NODEFAULTLIB:msvcrtd.lib")
    set(CMAKE_EXE_LINKER_FLAGS_DEBUG "${CMAKE_EXE_LINKER_FLAGS_DEBUG} /NODEFAULTLIB:libcmt.lib")
    set(CMAKE_EXE_LINKER_FLAGS_RELEASE "${CMAKE_EXE_LINKER_FLAGS_RELEASE} /NODEFAULTLIB:libcmtd.lib")
  endif()
endif()

visp_add_subdirectory(calibration              REQUIRED_DEPS visp_core visp_vision visp_io visp_gui)
visp_add_subdirectory(coin-simulator           REQUIRED_DEPS visp_core visp_robot visp_vs visp_ar visp_io)
visp_add_subdirectory(device/display           REQUIRED_DEPS visp_core visp_gui visp_io)
visp_add_subdirectory(device/framegrabber      REQUIRED_DEPS visp_core visp_sensor visp_io visp_gui)
visp_add_subdirectory(device/kinect            REQUIRED_DEPS visp_core visp_sensor visp_gui)
visp_add_subdirectory(device/laserscanner      REQUIRED_DEPS visp_core visp_sensor visp_io)
visp_add_subdirectory(device/light             REQUIRED_DEPS visp_core visp_robot visp_io)
visp_add_subdirectory(direct-visual-servoing   REQUIRED_DEPS visp_core visp_robot visp_visual_features visp_io visp_gui)
visp_add_subdirectory(homography               REQUIRED_DEPS visp_core visp_vision visp_io)
visp_add_subdirectory(image                    REQUIRED_DEPS visp_core visp_io)
visp_add_subdirectory(key-point                REQUIRED_DEPS visp_core visp_vision visp_sensor visp_io visp_gui)
visp_add_subdirectory(manual                   REQUIRED_DEPS visp_core visp_sensor visp_vs visp_robot visp_ar visp_vision visp_io visp_gui)
visp_add_subdirectory(math                     REQUIRED_DEPS visp_core visp_vision visp_io)
visp_add_subdirectory(moments/image            REQUIRED_DEPS visp_core visp_vs visp_robot visp_gui)
visp_add_subdirectory(moments/points           REQUIRED_DEPS visp_core visp_vs visp_robot visp_gui)
visp_add_subdirectory(moments/polygon          REQUIRED_DEPS visp_core visp_vs visp_robot visp_gui)
visp_add_subdirectory(ogre-simulator           REQUIRED_DEPS visp_core visp_vision visp_ar visp_blob visp_io visp_gui)
visp_add_subdirectory(parse-argv               REQUIRED_DEPS visp_core visp_io)
visp_add_subdirectory(pose-estimation          REQUIRED_DEPS visp_core visp_blob visp_vision visp_io visp_gui)
visp_add_subdirectory(robot-simulator/afma6    REQUIRED_DEPS visp_core visp_vs visp_robot visp_io visp_gui)
visp_add_subdirectory(robot-simulator/camera   REQUIRED_DEPS visp_core visp_vs visp_robot visp_io visp_gui)
visp_add_subdirectory(robot-simulator/viper850 REQUIRED_DEPS visp_core visp_vs visp_robot visp_io visp_gui)
visp_add_subdirectory(servo-afma4              REQUIRED_DEPS visp_core visp_blob visp_vs visp_robot visp_sensor visp_gui)
visp_add_subdirectory(servo-afma6              REQUIRED_DEPS visp_core visp_blob visp_vs visp_robot visp_sensor visp_vision visp_gui)
visp_add_subdirectory(servo-biclops            REQUIRED_DEPS visp_core visp_blob visp_vs visp_robot visp_sensor visp_gui)
visp_add_subdirectory(servo-franka             REQUIRED_DEPS visp_core visp_robot visp_vision visp_io visp_gui visp_vs visp_visual_features visp_detection)
visp_add_subdirectory(servo-pioneer            REQUIRED_DEPS visp_core visp_blob visp_vs visp_robot visp_sensor visp_gui)
visp_add_subdirectory(servo-ptu46              REQUIRED_DEPS visp_core visp_blob visp_vs visp_robot visp_sensor visp_gui)
visp_add_subdirectory(servo-viper650           REQUIRED_DEPS visp_core visp_blob visp_vs visp_robot visp_sensor visp_vision visp_gui)
visp_add_subdirectory(servo-viper850           REQUIRED_DEPS visp_core visp_blob visp_vs visp_robot visp_sensor visp_vision visp_gui)
visp_add_subdirectory(tools                    REQUIRED_DEPS visp_core visp_robot visp_io visp_gui)
visp_add_subdirectory(tracking                 REQUIRED_DEPS visp_core visp_io visp_gui)
visp_add_subdirectory(video                    REQUIRED_DEPS visp_core visp_io visp_gui)
visp_add_subdirectory(wireframe-simulator      REQUIRED_DEPS visp_core visp_robot visp_io visp_gui)
