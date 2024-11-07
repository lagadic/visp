/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Trajectory generator for torque control.
 *
 * Interface for the Franka robot.
 *
*****************************************************************************/
#ifndef _vpForceTorqueGenerator_impl_h_
#define _vpForceTorqueGenerator_impl_h_

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FRANKA
#include <array>
#include <atomic>
#include <iostream>
#include <vector>

#include <franka/exception.h>
#include <franka/robot.h>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot_state.h>

#include <visp3/robot/vpRobot.h>

BEGIN_VISP_NAMESPACE
class vpForceTorqueGenerator
{
public:
  vpForceTorqueGenerator() { }
  virtual ~vpForceTorqueGenerator() { }

  void control_thread(franka::Robot *robot, std::atomic_bool &stop, const std::string &log_folder,
                      const vpRobot::vpControlFrameType &frame, const std::array<double, 7> &tau_J_des,
                      const vpColVector &ft_cart_des, franka::RobotState &robot_state, std::mutex &mutex,
                      const double &filter_gain, const bool &activate_pi_controller);
};
END_VISP_NAMESPACE
#endif
#endif
