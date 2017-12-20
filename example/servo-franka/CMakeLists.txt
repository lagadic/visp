#############################################################################
#
# This file is part of the ViSP software.
# Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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

project(example-servo-franka)

cmake_minimum_required(VERSION 2.6)

find_package(VISP REQUIRED visp_robot)

set(example_cpp
  franka_echo_robot_state.cpp
  franka_execute_trajectory.cpp
  franka_generate_cartesian_pose_motion.cpp
  franka_generate_cartesian_velocity_motion.cpp
  franka_generate_consecutive_motions.cpp
  franka_generate_joint_position_motion.cpp
  franka_generate_joint_velocity_motion.cpp
  franka_grasp_object.cpp
  franka_joint_impedance_control.cpp
  franka_joint_point_to_point_motion.cpp
  franka_motion_with_control.cpp
  franka_print_joint_positions.cpp
)

foreach(cpp ${example_cpp})
  visp_add_target(${cpp})
  if(COMMAND visp_add_dependency)
    visp_add_dependency(${cpp} "examples")
  endif()
endforeach()

