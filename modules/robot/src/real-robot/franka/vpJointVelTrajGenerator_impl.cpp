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
 * Interface for the Franka robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include "vpJointVelTrajGenerator_impl.h"

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FRANKA
#include <cmath>
#include <iomanip>
#include <algorithm>

#include <franka/exception.h>
#include <franka/robot.h>
#include <franka/model.h>

#include <visp3/core/vpException.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpMatrix.h>

void vpJointVelTrajGenerator::control_thread(franka::Robot *robot,
                                             std::atomic_bool &stop,
                                             const std::string &log_folder,
                                             const vpRobot::vpControlFrameType &frame,
                                             const vpHomogeneousMatrix &eMc,
                                             const vpColVector &v_cart_des, // end-effector velocity
                                             const std::array<double, 7> &dq_des, // joint velocity
                                             const std::array<double, 7> &q_min,
                                             const std::array<double, 7> &q_max,
                                             const std::array<double, 7> &dq_max,
                                             const std::array<double, 7> &ddq_max,
                                             franka::RobotState &robot_state,
                                             std::mutex &mutex)
{
  double time = 0.0;
  double delta_t = 0.001;
  std::array<double, 7> q_prev;
  franka::Model model = robot->loadModel();
  vpMatrix eJe(6, 7), fJe(6, 7);
  vpVelocityTwistMatrix cVe(eMc.inverse());

  std::ofstream log_time;
  std::ofstream log_q_mes;
  std::ofstream log_dq_mes;
  std::ofstream log_dq_des;
  std::ofstream log_dq_cmd;
  std::ofstream log_v_des;

  auto joint_velocity_callback = [=, &log_time, &log_q_mes, &log_dq_mes, &log_dq_des, &log_dq_cmd, &time, &q_prev, &dq_des, &stop, &robot_state, &mutex]
      (const franka::RobotState& state, franka::Duration period) -> franka::JointVelocities {

    time += period.toSec();

    static vpJointVelTrajGenerator joint_vel_traj_generator;

    if (time == 0.0) {
      if (! log_folder.empty()) {
        std::cout << "Save franka logs in \"" << log_folder << "\" folder" << std::endl;
        log_time.open(log_folder + "/time.log");
        log_q_mes.open(log_folder + "/q-mes.log");
        log_dq_mes.open(log_folder + "/dq-mes.log");
        log_dq_des.open(log_folder + "/dq-des.log");
        log_dq_cmd.open(log_folder + "/dq-cmd.log");
      }
      q_prev = state.q_d;
      joint_vel_traj_generator.init(state.q_d, q_min, q_max, dq_max, ddq_max, delta_t);
    }

    {
      std::lock_guard<std::mutex> lock(mutex);
      robot_state = state;
    }

    std::array<double, 7> q_cmd;
    std::array<double, 7> dq_cmd;

    auto dq_des_ = dq_des;
    if (stop) { // Stop asked
      for (auto & dq_ : dq_des_) {
        dq_ = 0.0;
      }
    }

    joint_vel_traj_generator.applyVel(dq_des_, q_cmd, dq_cmd);

    if (! log_folder.empty()) {
      log_time << time << std::endl;
      log_q_mes << std::fixed << std::setprecision(8) << state.q_d[0] << " " << state.q_d[1] << " " << state.q_d[2] << " " << state.q_d[3] << " " << state.q_d[4] << " " << state.q_d[5] << " " << state.q_d[6] << std::endl;
      log_dq_mes << std::fixed << std::setprecision(8) << state.dq_d[0] << " " << state.dq_d[1] << " " << state.dq_d[2] << " " << state.dq_d[3] << " " << state.dq_d[4] << " " << state.dq_d[5] << " " << state.dq_d[6] << std::endl;
      log_dq_cmd << std::fixed << std::setprecision(8) << dq_cmd[0] << " " << dq_cmd[1] << " " << dq_cmd[2] << " " << dq_cmd[3] << " " << dq_cmd[4] << " " << dq_cmd[5] << " " << dq_cmd[6] << std::endl;
      log_dq_des << std::fixed << std::setprecision(8) << dq_des_[0] << " " << dq_des_[1] << " " << dq_des_[2] << " " << dq_des_[3] << " " << dq_des_[4] << " " << dq_des_[5] << " " << dq_des_[6] << std::endl;
    }

    franka::JointVelocities velocities = {dq_cmd[0], dq_cmd[1], dq_cmd[2], dq_cmd[3], dq_cmd[4], dq_cmd[5], dq_cmd[6]};

    if (stop) {
      unsigned int nb_joint_stop = 0;
      const double q_eps = 1e-6; // Motion finished
      for(size_t i=0; i < 7; i++) {
        if (std::abs(state.q_d[i] - q_prev[i]) < q_eps) {
          nb_joint_stop ++;
        }
      }

      if (nb_joint_stop == 7) {
        if (! log_folder.empty()) {
          log_time.close();
          log_q_mes.close();
          log_dq_mes.close();
          log_dq_des.close();
          log_dq_cmd.close();
        }
        return franka::MotionFinished(velocities);
      }
    }

    q_prev = state.q_d;

#if (VISP_HAVE_FRANKA_VERSION < 0x000500)
    // state.q_d contains the last joint velocity command received by the robot.
    // In case of packet loss due to bad connection or due to a slow control loop
    // not reaching the 1kHz rate, even if your desired velocity trajectory
    // is smooth, discontinuities might occur.
    // Saturating the acceleration computed with respect to the last command received
    // by the robot will prevent from getting discontinuity errors.
    // Note that if the robot does not receive a command it will try to extrapolate
    // the desired behavior assuming a constant acceleration model
    return limitRate(ddq_max, velocities.dq, state.dq_d);
#else
    // With libfranka 0.5.0 franka::control() enables limit_rate by default
    return velocities;
#endif
  };

  auto cartesian_velocity_callback = [=, &log_time, &log_q_mes, &log_dq_mes, &log_dq_des,  &log_dq_cmd, &log_v_des, &time, &model, &q_prev, &v_cart_des, &stop, &robot_state, &mutex]
      (const franka::RobotState& state, franka::Duration period) -> franka::JointVelocities {

    time += period.toSec();

    static vpJointVelTrajGenerator joint_vel_traj_generator;

    if (time == 0.0) {
      if (! log_folder.empty()) {
        std::cout << "Save franka logs in \"" << log_folder << "\" folder" << std::endl;
        log_time.open(log_folder + "/time.log");
        log_q_mes.open(log_folder + "/q-mes.log");
        log_dq_mes.open(log_folder + "/dq-mes.log");
        log_dq_des.open(log_folder + "/dq-des.log");
        log_dq_cmd.open(log_folder + "/dq-cmd.log");
        log_v_des.open(log_folder + "v-des.log");
      }
      q_prev = state.q_d;
      joint_vel_traj_generator.init(state.q_d, q_min, q_max, dq_max, ddq_max, delta_t);
    }

    {
      std::lock_guard<std::mutex> lock(mutex);
      robot_state = state;
    }

    // Get robot Jacobian
    if (frame == vpRobot::END_EFFECTOR_FRAME || frame == vpRobot::TOOL_FRAME) {
      std::array<double, 42> jacobian = model.bodyJacobian(franka::Frame::kEndEffector, state);
      // Convert row-major to col-major
      for (size_t i = 0; i < 6; i ++) { // TODO make a function
        for (size_t j = 0; j < 7; j ++) {
          eJe[i][j] = jacobian[j*6 + i];
        }
      }
    }
    else if (frame == vpRobot::REFERENCE_FRAME) {
      std::array<double, 42> jacobian = model.zeroJacobian(franka::Frame::kEndEffector, state);
      // Convert row-major to col-major
      for (size_t i = 0; i < 6; i ++) { // TODO make a function
        for (size_t j = 0; j < 7; j ++) {
          fJe[i][j] = jacobian[j*6 + i];
        }
      }
    }

    // Compute joint velocity
    vpColVector q_dot;
    if (frame == vpRobot::END_EFFECTOR_FRAME) {
      q_dot = eJe.pseudoInverse() * v_cart_des; // TODO introduce try catch
    }
    else if (frame == vpRobot::TOOL_FRAME) {
      q_dot = (cVe * eJe).pseudoInverse() * v_cart_des; // TODO introduce try catch
    }
    else if (frame == vpRobot::REFERENCE_FRAME) {
      q_dot = (cVe * fJe).pseudoInverse() * v_cart_des; // TODO introduce try catch
    }

    std::array<double, 7> dq_des;
    for (size_t i = 0; i < 7; i++) // TODO create a function to convert
      dq_des[i] = q_dot[i];

    std::array<double, 7> q_cmd;
    std::array<double, 7> dq_cmd;

    auto dq_des_ = dq_des;
    if (stop) { // Stop asked
      for (auto & dq_ : dq_des_) {
        dq_ = 0.0;
      }
    }

    joint_vel_traj_generator.applyVel(dq_des_, q_cmd, dq_cmd);

    if (! log_folder.empty()) {
      log_time << time << std::endl;
      log_q_mes << std::fixed << std::setprecision(8) << state.q_d[0] << " " << state.q_d[1] << " " << state.q_d[2] << " " << state.q_d[3] << " " << state.q_d[4] << " " << state.q_d[5] << " " << state.q_d[6] << std::endl;
      log_dq_mes << std::fixed << std::setprecision(8) << state.dq_d[0] << " " << state.dq_d[1] << " " << state.dq_d[2] << " " << state.dq_d[3] << " " << state.dq_d[4] << " " << state.dq_d[5] << " " << state.dq_d[6] << std::endl;
      log_dq_cmd << std::fixed << std::setprecision(8) << dq_cmd[0] << " " << dq_cmd[1] << " " << dq_cmd[2] << " " << dq_cmd[3] << " " << dq_cmd[4] << " " << dq_cmd[5] << " " << dq_cmd[6] << std::endl;
      log_dq_des << std::fixed << std::setprecision(8) << dq_des_[0] << " " << dq_des_[1] << " " << dq_des_[2] << " " << dq_des_[3] << " " << dq_des_[4] << " " << dq_des_[5] << " " << dq_des_[6] << std::endl;
      log_v_des << std::fixed << std::setprecision(8) << v_cart_des[0] << " " << v_cart_des[1] << " " << v_cart_des[2] << " " << v_cart_des[3] << " " << v_cart_des[4] << " " << v_cart_des[5] << std::endl;
    }

    franka::JointVelocities velocities = {dq_cmd[0], dq_cmd[1], dq_cmd[2], dq_cmd[3], dq_cmd[4], dq_cmd[5], dq_cmd[6]};

    if (stop) {
      unsigned int nb_joint_stop = 0;
      const double q_eps = 1e-6; // Motion finished
      for(size_t i=0; i < 7; i++) {
        if (std::abs(state.q_d[i] - q_prev[i]) < q_eps) {
          nb_joint_stop ++;
        }
      }
      if (nb_joint_stop == 7) {
        if (! log_folder.empty()) {
          log_time.close();
          log_q_mes.close();
          log_dq_mes.close();
          log_dq_des.close();
          log_dq_cmd.close();
          log_v_des.close();
        }
        return franka::MotionFinished(velocities);
      }
    }

    q_prev = state.q_d;

#if (VISP_HAVE_FRANKA_VERSION < 0x000500)
    // state.q_d contains the last joint velocity command received by the robot.
    // In case of packet loss due to bad connection or due to a slow control loop
    // not reaching the 1kHz rate, even if your desired velocity trajectory
    // is smooth, discontinuities might occur.
    // Saturating the acceleration computed with respect to the last command received
    // by the robot will prevent from getting discontinuity errors.
    // Note that if the robot does not receive a command it will try to extrapolate
    // the desired behavior assuming a constant acceleration model
    return limitRate(ddq_max, velocities.dq, state.dq_d);
#else
    // With libfranka 0.5.0 franka::control enables limit_rate by default
    return velocities;
#endif
  };

#if !(VISP_HAVE_FRANKA_VERSION < 0x000500)
  double cutoff_frequency = 10;
#endif
  switch (frame) {
  case vpRobot::JOINT_STATE: {
    int nbAttempts = 10;
    for (int attempt = 1; attempt <= nbAttempts; attempt++) {
      try {
#if (VISP_HAVE_FRANKA_VERSION < 0x000500)
        robot->control(joint_velocity_callback);
#else
        robot->control(joint_velocity_callback, franka::ControllerMode::kJointImpedance, true, cutoff_frequency);
#endif
        break;
      } catch (const franka::ControlException &e) {
        std::cerr << "Warning: communication error: " << e.what() << "\nRetry attempt: " << attempt << std::endl;
        robot->automaticErrorRecovery();
        if (attempt == nbAttempts)
          throw;
      }
    }
    break;
  }
  case vpRobot::CAMERA_FRAME:
  case vpRobot::REFERENCE_FRAME:
  case vpRobot::END_EFFECTOR_FRAME: {
    int nbAttempts = 10;
    for (int attempt = 1; attempt <= nbAttempts; attempt++) {
      try {
#if (VISP_HAVE_FRANKA_VERSION < 0x000500)
        robot->control(cartesian_velocity_callback);
#else
        robot->control(cartesian_velocity_callback, franka::ControllerMode::kJointImpedance, true, cutoff_frequency);
#endif
        break;
      } catch (const franka::ControlException &e) {
        std::cerr << "Warning: communication error: " << e.what() << "\nRetry attempt: " << attempt << std::endl;
        robot->automaticErrorRecovery();
        if (attempt == nbAttempts)
          throw;
      }
    }
    break;
  }
  case vpRobot::MIXT_FRAME: {
    throw vpException(vpException::fatalError, "Velocity controller not supported");
  }
  }
}

void vpJointVelTrajGenerator::init(const std::array<double, 7> &q,
                                   const std::array<double, 7> &q_min,
                                   const std::array<double, 7> &q_max,
                                   const std::array<double, 7> &dq_max,
                                   const std::array<double, 7> &ddq_max,
                                   const double delta_t)
{
  if (m_njoints != q_min.size() || m_njoints != q_max.size()
      || m_njoints != dq_max.size() || m_njoints != ddq_max.size()) {
    throw(vpException(vpException::dimensionError, "Inconsistent number of joints"));
  }
  m_q_min = q_min;
  m_q_max = q_max;
  m_dq_max = dq_max;
  m_ddq_max = ddq_max;

  m_delta_t = delta_t;

  m_q_final = m_q_cmd = q;
  m_q_cmd_prev = m_q_cmd;

  m_dq_des        = {0, 0, 0, 0, 0, 0, 0};
  m_dq_des_prev   = {0, 0, 0, 0, 0, 0, 0};
  m_delta_q       = {0, 0, 0, 0, 0, 0, 0};
  m_delta_q_max   = {0, 0, 0, 0, 0, 0, 0};
  m_sign          = {0, 0, 0, 0, 0, 0, 0};
  m_dist_to_final = {0, 0, 0, 0, 0, 0, 0};
  m_dist_AD       = {0, 0, 0, 0, 0, 0, 0};
  m_flagSpeed     = {false, false, false, false, false, false, false};
  m_status        = {FLAGSTO, FLAGSTO, FLAGSTO, FLAGSTO, FLAGSTO, FLAGSTO, FLAGSTO};

  for (size_t i=0; i<m_njoints; i++) {
    m_delta_q_acc[i] = m_ddq_max[i] * m_delta_t * m_delta_t;
  }
}


/*!
 * Compute the joint position and velocity to reach desired joint velocity.
 * \param dq_des : Desired joint velocity
 * \param q_cmd : Position to apply.
 * \param dq_cmd : Velocity to apply.
 */
void vpJointVelTrajGenerator::applyVel(const std::array<double, 7> &dq_des,
                                       std::array<double, 7> &q_cmd,
                                       std::array<double, 7> &dq_cmd)
{
  for (size_t i=0; i < m_njoints; i++) {
    m_dq_des[i] = dq_des[i];

    if (m_dq_des[i] != m_dq_des_prev[i]) {

      m_flagJointLimit = false;

      if (m_dq_des[i] > m_dq_max[i]) {
        m_dq_des[i] = m_dq_max[i];
      }
      else if (m_dq_des[i] < (-m_dq_max[i])) {
        m_dq_des[i] = -m_dq_max[i];
      }

      if (m_flagSpeed[i] == false) {
        // Change from stop to new vel with acc control
        if ( m_status[i] == FLAGSTO) // If stop
        {
          if (m_dq_des[i] > 0)
          {
            m_delta_q_max[i] = m_dq_des[i]*m_delta_t;
            m_sign[i] = 1;
            m_q_final[i] = m_q_max[i] - m_offset_joint_limit;
            m_delta_q[i] = 0;
            m_status[i] = FLAGACC;
          }
          else if (m_dq_des[i] < 0)
          {
            m_delta_q_max[i] = - m_dq_des[i]*m_delta_t;
            m_sign[i] = -1;
            m_q_final[i] = m_q_min[i] + m_offset_joint_limit;
            m_delta_q[i] = 0;
            m_status[i] = FLAGACC;
          }
        }

        // Change of direction
        else if ( (m_dq_des[i] * m_sign[i]) < 0) {
          m_flagSpeed[i] = true;
          m_status[i] = FLAGDEC;
          m_delta_q_max[i] = 0;
        }
        else {
          // Acceleration or deceleration
          if ( m_sign[i] == 1) {
            if ( m_dq_des[i] > m_dq_des_prev[i])
              m_status[i] = FLAGACC;
            else
              m_status[i] = FLAGDEC;
            m_delta_q_max[i] = m_dq_des[i]*m_delta_t;
          }
          else {
            if ( m_dq_des[i] > m_dq_des_prev[i])
              m_status[i] = FLAGDEC;
            else
              m_status[i] = FLAGACC;
            m_delta_q_max[i] = - m_dq_des[i]*m_delta_t;
          }
        }

        // Update distance to accelerate or decelerate
        int n = (int) (m_delta_q_max[i] / m_delta_q_acc[i]);
        m_dist_AD[i]=n*(m_delta_q_max[i]-(n+1)*m_delta_q_acc[i]/2);
      }
      m_dq_des_prev[i] = m_dq_des[i];
    }
  }

  /*
   * Compute new command in case of
   *		- acceleration
   *		- deceleration
   *		- stop
   */
  for (size_t i=0; i < m_njoints; i++) {
    // Security joint limit
    m_dist_to_final[i] = ( m_q_final[i] - m_q_cmd[i]) * m_sign[i];
    if ((m_dist_to_final[i] - m_delta_q_max[i]) <=  m_dist_AD[i]) {
      if (m_dist_AD[i] > 0) {
        if (!m_flagJointLimit) printf("Joint limit flag axis %lu\n", (unsigned long)i);
        m_flagJointLimit = true;
        for(size_t k=0; k < m_njoints; k++)
        {
          if (m_status[k] != FLAGSTO) m_status[k] = FLAGDEC;
          m_delta_q_max[k] = 0;
        }
      }
    }
    /*
     * Deceleration.
     */
    if ( m_status[i] == FLAGDEC) {
      m_delta_q[i] -=  m_delta_q_acc[i];
      if (m_delta_q[i] <=  m_delta_q_max[i]) {
        if (m_delta_q_max[i] < m_delta_q_min)  {
          m_status[i] = FLAGSTO;
          m_delta_q[i] = 0.0;
          // Test if change of direction
          if (m_flagSpeed[i] == true) {
            if (m_dq_des[i] > 0) {
              m_delta_q_max[i] = m_dq_des[i]*m_delta_t;
              m_sign[i] = 1;
              m_q_final[i] = m_q_max[i] - m_offset_joint_limit;
            }
            else if (m_dq_des[i] < 0) {
              m_delta_q_max[i] = -m_dq_des[i]*m_delta_t;
              m_sign[i] = -1;
              m_q_final[i] = m_q_min[i] + m_offset_joint_limit;
            }
            m_status[i] = FLAGACC;
            m_flagSpeed[i] = false;

            int n = (int) (m_delta_q_max[i] / m_delta_q_acc[i]);
            m_dist_AD[i]=n*(m_delta_q_max[i]-(n+1)*m_delta_q_acc[i]/2);
          }
        }
        else if ((m_delta_q_max[i] > 0) && !m_flagJointLimit)  {
          if (m_delta_q_max[i] < (m_delta_q[i] + 2*m_delta_q_acc[i])) {
            m_delta_q[i] = m_delta_q_max[i];
            m_status[i] = FLAGCTE;
          }
          else if (!m_flagJointLimit) {
            /* acceleration moins rapide*/
            m_delta_q[i] += (2*m_delta_q_acc[i]);
            m_status[i] = FLAGACC;
          }
        }
      }
    }
    /*
     * Acceleration.
     */
    else if (m_status[i] == FLAGACC) {
      m_delta_q[i] += m_delta_q_acc[i];

      if (m_delta_q[i] >= m_delta_q_max[i]) {
        m_delta_q[i] = m_delta_q_max[i];
        m_status[i] = FLAGCTE;
      }
    }
    /*
     * Constant velocity
     */
    m_q_cmd[i] += m_sign[i] * m_delta_q[i];
  }

  // Test si un axe arrive pres des butees. Si oui, arret de tous les axes
  for (size_t i=0; i < m_njoints;i++) {
    double butee = m_q_min[i] + m_offset_joint_limit;
    if (m_q_cmd[i] < butee) {
      for (size_t j=0; j < m_njoints;j++) {
        m_q_cmd[j] -= m_sign[j]*m_delta_q[j];
      }
      m_q_cmd[i] = butee;
      printf("Joint limit axis %lu\n", (unsigned long)i);
      break;
    }
    butee = (float) (m_q_max[i] - m_offset_joint_limit);
    if (m_q_cmd[i] > butee) {
      for (size_t j=0; j < m_njoints; j++) {
        m_q_cmd[j] -= m_sign[j]*m_delta_q[j];
      }
      m_q_cmd[i] = butee;
      printf("Joint limit axis %lu\n", (unsigned long)i);
      break;
    }
  }

  q_cmd = m_q_cmd;

  // Compute velocity command
  for (size_t i = 0; i < m_q_cmd.size(); i++) {
    dq_cmd[i] = (m_q_cmd[i] - m_q_cmd_prev[i]) / m_delta_t;
  }

  m_q_cmd_prev = m_q_cmd;
}

/**
 * Limits the rate of an input vector of per-joint commands considering the maximum allowed time
 * derivatives.
 *
 * @param[in] max_derivatives Per-joint maximum allowed time derivative.
 * @param[in] desired_values Desired values of the current time step.
 * @param[in] last_desired_values Desired values of the previous time step.
 *
 * @return Rate-limited vector of desired values.
 */
std::array<double, 7> vpJointVelTrajGenerator::limitRate(const std::array<double, 7>& max_derivatives,
                                                         const std::array<double, 7>& desired_values,
                                                         const std::array<double, 7>& last_desired_values) {
  std::array<double, 7> limited_values{};
  for (size_t i = 0; i < 7; i++) {
    double desired_difference = (desired_values[i] - last_desired_values[i]) / 1e-3;
    limited_values[i] =
        last_desired_values[i] +
        std::max(std::min(desired_difference, max_derivatives[i]), -max_derivatives[i]) * 1e-3;
  }
  return limited_values;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpJointVelTrajGenerator.cpp.o) has no symbols
void dummy_vpJointVelTrajGenerator(){};
#endif // VISP_HAVE_FRANKA

