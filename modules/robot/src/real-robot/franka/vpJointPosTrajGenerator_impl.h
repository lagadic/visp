/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * Trajectory generator for joint positioning.
 *
 * This code was originally part of libfranka and adapted to use ViSP instead
 * of Eigen.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
#ifndef _vpJointPosTrajGenerator_impl_h_
#define _vpJointPosTrajGenerator_impl_h_

#include <array>
#include <iostream>
#include <atomic>

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpColVector.h>

#ifdef VISP_HAVE_FRANKA
#include <franka/exception.h>
#include <franka/robot.h>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot_state.h>

/**
 * An example showing how to generate a joint pose motion to a goal position. Adapted from:
 * Wisama Khalil and Etienne Dombre. 2002. Modeling, Identification and Control of Robots
 * (Kogan Page Science Paper edition).
 */
class vpJointPosTrajGenerator {
 public:
  /**
   * Creates a new MotionGenerator instance for a target q.
   *
   * @param[in] speed_factor General speed factor in range [0, 1].
   * @param[in] q_goal Target joint positions.
   */
  vpJointPosTrajGenerator(double speed_factor, const std::array<double, 7> &q_goal);

  /**
   * Sends joint position calculations
   *
   * @param[in] robot_state Current state of the robot.
   * @param[in] period Duration of execution.
   *
   * @return Joint positions for use inside a control loop.
   */
  franka::JointPositions operator()(const franka::RobotState& robot_state, franka::Duration period);

 private:
  bool calculateDesiredValues(double t, vpColVector &delta_q_d) const;
  void calculateSynchronizedValues();

  static constexpr double kDeltaQMotionFinished = 1e-6;
  vpColVector m_q_goal;
  vpColVector m_q_start;
  vpColVector m_delta_q;
  vpColVector m_dq_max_sync;
  vpColVector m_t_1_sync;
  vpColVector m_t_2_sync;
  vpColVector m_t_f_sync;
  vpColVector m_q_1;

  vpColVector m_dq_max;
  vpColVector m_ddq_max_start;
  vpColVector m_ddq_max_goal;

  double m_time = 0.0;
};

#endif
#endif
