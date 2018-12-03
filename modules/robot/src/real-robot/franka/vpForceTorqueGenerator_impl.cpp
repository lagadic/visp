/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Interface for the Franka robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include "vpForceTorqueGenerator_impl.h"

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

#include <Eigen/Core> // TODO: Remove when initial_position var is removed


//auto force_control_callback = [=, &log_time, &log_tau_cmd, &log_tau_d, &log_tau_mes, &time, &model, &initial_position, &stop, &zero_torques, &robot_state, &tau_error_integral, &desired_mass, &mutex]
//                                  (const franka::RobotState& state, franka::Duration period) -> franka::Torques {
//  time += period.toSec();

//  if (time == 0.0) {
//    if (! log_folder.empty()) {
//      std::cout << "Save franka logs in \"" << log_folder << "\" folder" << std::endl;
//      log_time.open(log_folder + "/time.log");
//      log_tau_cmd.open(log_folder + "/tau_cmd.log");
//      log_tau_d.open(log_folder + "/tau_d.log");
//      log_tau_mes.open(log_folder + "/tau_mes.log");
//    }
//  }

//  {
//    std::lock_guard<std::mutex> lock(mutex);
//    robot_state = state;
//  }

//  if (time == 0.0) {
//    initial_position = get_position(state);
//  }

//  if (time > 0 && (get_position(state) - initial_position).norm() > 0.01) {
//    throw std::runtime_error("Aborting; too far away from starting pose!");
//  }

//  // get state variables
//  std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);

//  Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
//  Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_measured(state.tau_J.data());
//  Eigen::Map<const Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());

//  Eigen::VectorXd tau_d(7), desired_force_torque(6), tau_cmd(7), tau_ext(7);
//  desired_force_torque.setZero();
//  desired_force_torque(2) = desired_mass * -9.81;
//  tau_ext << tau_measured - gravity - initial_tau_ext;
//  tau_d << jacobian.transpose() * desired_force_torque;
//  tau_error_integral += period.toSec() * (tau_d - tau_ext);
//  // FF + PI control
//  tau_cmd << tau_d + k_p * (tau_d - tau_ext) + k_i * tau_error_integral;

//  // Smoothly update the mass to reach the desired target value
//  desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass;

//  std::array<double, 7> tau_d_array{};
//  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;

//  if (! log_folder.empty()) {
//    log_time << time << std::endl;
//    log_tau_cmd << std::fixed << std::setprecision(8) << tau_cmd[0] << " " << tau_cmd[1] << " " << tau_cmd[2] << " " << tau_cmd[3] << " " << tau_cmd[4] << " " << tau_cmd[5] << " " << tau_cmd[6] << std::endl;
//    log_tau_d   << std::fixed << std::setprecision(8) << tau_d[0] << " " << tau_d[1] << " " << tau_d[2] << " " << tau_d[3] << " " << tau_d[4] << " " << tau_d[5] << " " << tau_d[6] << std::endl;
//    log_tau_mes << std::fixed << std::setprecision(8) << tau_ext[0] << " " << tau_ext[1] << " " << tau_ext[2] << " " << tau_ext[3] << " " << tau_ext[4] << " " << tau_ext[5] << " " << tau_ext[6] << std::endl;
//  }

//  if (stop) {
//    if (! log_folder.empty()) {
//      log_time.close();
//      log_tau_cmd.close();
//      log_tau_d.close();
//      log_tau_mes.close();
//    }
//    return franka::MotionFinished(zero_torques);
//  }

//  return tau_d_array;
//};

void vpForceTorqueGenerator::control_thread(franka::Robot *robot, std::atomic_bool &stop,
                                            const std::string &log_folder,
                                            const vpRobot::vpControlFrameType &frame,
                                            const std::array<double, 7> &tau_J_des,
                                            const vpColVector &ft_cart_des,
                                            franka::RobotState &robot_state,
                                            std::mutex &mutex)
{
  double time = 0.0;

  Eigen::Vector3d initial_position;
  franka::Model model = robot->loadModel();

//  constexpr double target_mass{1.0};    // NOLINT(readability-identifier-naming)
  constexpr double k_p{1.0};            // NOLINT(readability-identifier-naming)
  constexpr double k_i{2.0};            // NOLINT(readability-identifier-naming)
  constexpr double filter_gain{0.001};  // NOLINT(readability-identifier-naming)
//  double desired_mass{0.0};
  Eigen::VectorXd initial_tau_ext(7), tau_error_integral(7);

  // Bias torque sensor
  std::array<double, 7> gravity_array = model.gravity(robot_state);
  Eigen::Map<Eigen::Matrix<double, 7, 1> > initial_tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > initial_gravity(gravity_array.data());
  initial_tau_ext = initial_tau_measured - initial_gravity;

  franka::Torques zero_torques{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Eigen::VectorXd tau_joint_d(7), tau_cart_d(6);
  for(size_t i = 0; i < 7; i++) {
    tau_joint_d[i] = 0;
  }
  for(size_t i = 0; i < 6; i++) {
    tau_cart_d[i] = 0;
  }

  // init integrator
  tau_error_integral.setZero();

  std::ofstream log_time;
  std::ofstream log_tau_cmd;
  std::ofstream log_tau_d;
  std::ofstream log_tau_mes;

//  auto get_position = [](const franka::RobotState& robot_state) {
//    return Eigen::Vector3d(robot_state.O_T_EE[12], robot_state.O_T_EE[13],
//                           robot_state.O_T_EE[14]);
//  };

  auto force_joint_control_callback = [=, &log_time, &log_tau_cmd, &log_tau_d, &log_tau_mes, &time, &model, &initial_position,
      &stop, &zero_torques, &robot_state, &tau_joint_d, &tau_error_integral,
      &mutex, &tau_J_des]
      (const franka::RobotState& state, franka::Duration period) -> franka::Torques {
    time += period.toSec();

    if (time == 0.0) {
      if (! log_folder.empty()) {
        std::cout << "Save franka logs in \"" << log_folder << "\" folder" << std::endl;
        log_time.open(log_folder + "/time.log");
        log_tau_cmd.open(log_folder + "/tau_cmd.log");
        log_tau_d.open(log_folder + "/tau_d.log");
        log_tau_mes.open(log_folder + "/tau_mes.log");
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex);
      robot_state = state;
    }

//    if (time == 0.0) {
//      initial_position = get_position(state);
//    }

//    if (time > 0 && (get_position(state) - initial_position).norm() > 0.01) {
//      throw std::runtime_error("Aborting; too far away from starting pose!");
//    }

    // get state variables
//    std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);

//    Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_measured(state.tau_J.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());

    Eigen::VectorXd /*tau_d(7), desired_force_torque(6),*/ tau_cmd(7), tau_ext(7), desired_tau(7);
//    desired_force_torque.setZero();
//    desired_force_torque(2) = desired_mass * -9.81;
    tau_ext << tau_measured - gravity - initial_tau_ext;
//    tau_d << jacobian.transpose() * desired_force_torque;
//    for(size_t i = 0; i < 7; i++) {
//      tau_d[i] = tau_J_des[i];
//    }
    tau_error_integral += period.toSec() * (tau_joint_d - tau_ext);
    // FF + PI control
    tau_cmd << tau_joint_d + k_p * (tau_joint_d - tau_ext) + k_i * tau_error_integral;

    // Smoothly update the mass to reach the desired target value
//    desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass;
    for(size_t i = 0; i < 7; i++) {
      tau_joint_d[i] = filter_gain * tau_J_des[i] + (1 - filter_gain) * tau_joint_d[i];
    }

    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;

    if (! log_folder.empty()) {
      log_time << time << std::endl;
      log_tau_cmd << std::fixed << std::setprecision(8) << tau_cmd[0] << " " << tau_cmd[1] << " " << tau_cmd[2] << " " << tau_cmd[3] << " " << tau_cmd[4] << " " << tau_cmd[5] << " " << tau_cmd[6] << std::endl;
      log_tau_d   << std::fixed << std::setprecision(8) << tau_joint_d[0] << " " << tau_joint_d[1] << " " << tau_joint_d[2] << " " << tau_joint_d[3] << " " << tau_joint_d[4] << " " << tau_joint_d[5] << " " << tau_joint_d[6] << std::endl;
      log_tau_mes << std::fixed << std::setprecision(8) << tau_ext[0] << " " << tau_ext[1] << " " << tau_ext[2] << " " << tau_ext[3] << " " << tau_ext[4] << " " << tau_ext[5] << " " << tau_ext[6] << std::endl;
    }

    if (stop) {
      if (! log_folder.empty()) {
        log_time.close();
        log_tau_cmd.close();
        log_tau_d.close();
        log_tau_mes.close();
      }
      return franka::MotionFinished(zero_torques);
    }

    return tau_d_array;
  };

  auto force_cart_control_callback = [=, &log_time, &log_tau_cmd, &log_tau_d, &log_tau_mes, &time, &model, /*&initial_position, */
      &stop, &zero_torques, &robot_state, &tau_cart_d, &tau_error_integral,
      &mutex, &ft_cart_des]
      (const franka::RobotState& state, franka::Duration period) -> franka::Torques {
    time += period.toSec();

    if (time == 0.0) {
      if (! log_folder.empty()) {
        std::cout << "Save franka logs in \"" << log_folder << "\" folder" << std::endl;
        log_time.open(log_folder + "/time.log");
        log_tau_cmd.open(log_folder + "/tau_cmd.log");
        log_tau_d.open(log_folder + "/tau_d.log");
        log_tau_mes.open(log_folder + "/tau_mes.log");
      }
    }

    {
      std::lock_guard<std::mutex> lock(mutex);
      robot_state = state;
    }

//    if (time == 0.0) {
//      initial_position = get_position(state);
//    }

//    if (time > 0 && (get_position(state) - initial_position).norm() > 0.01) {
//      throw std::runtime_error("Aborting; too far away from starting pose!");
//    }

    // get state variables
    std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);

    Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_measured(state.tau_J.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());

    Eigen::VectorXd tau_d(7), /*desired_force_torque(6),*/ tau_cmd(7), tau_ext(7), desired_tau(7);
//    desired_force_torque.setZero();
//    desired_force_torque(2) = desired_mass * -9.81;
    tau_ext << tau_measured - gravity - initial_tau_ext;
    tau_d << jacobian.transpose() * tau_cart_d;
//    for(size_t i = 0; i < 7; i++) {
//      tau_d[i] = tau_J_des[i];
//    }
    tau_error_integral += period.toSec() * (tau_d - tau_ext);
    // FF + PI control
    tau_cmd << tau_d + k_p * (tau_d - tau_ext) + k_i * tau_error_integral;

    // Smoothly update the mass to reach the desired target value
//    desired_mass = filter_gain * target_mass + (1 - filter_gain) * desired_mass;
    for(size_t i = 0; i < 7; i++) {
      tau_cart_d[i] = filter_gain * ft_cart_des[i] + (1 - filter_gain) * tau_cart_d[i];
    }

    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;

    if (! log_folder.empty()) {
      log_time << time << std::endl;
      log_tau_cmd << std::fixed << std::setprecision(8) << tau_cmd[0] << " " << tau_cmd[1] << " " << tau_cmd[2] << " " << tau_cmd[3] << " " << tau_cmd[4] << " " << tau_cmd[5] << " " << tau_cmd[6] << std::endl;
      log_tau_d   << std::fixed << std::setprecision(8) << tau_d[0] << " " << tau_d[1] << " " << tau_d[2] << " " << tau_d[3] << " " << tau_d[4] << " " << tau_d[5] << " " << tau_d[6] << std::endl;
      log_tau_mes << std::fixed << std::setprecision(8) << tau_ext[0] << " " << tau_ext[1] << " " << tau_ext[2] << " " << tau_ext[3] << " " << tau_ext[4] << " " << tau_ext[5] << " " << tau_ext[6] << std::endl;
    }

    if (stop) {
      if (! log_folder.empty()) {
        log_time.close();
        log_tau_cmd.close();
        log_tau_d.close();
        log_tau_mes.close();
      }
      return franka::MotionFinished(zero_torques);
    }

    return tau_d_array;
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
        robot->control(force_control_callback);
#else
        robot->control(force_joint_control_callback, true, cutoff_frequency);
#endif
        break;
      } catch (const franka::ControlException &e) {
        std::cerr << "Warning: communication error: " << e.what() << "\nRetry attempt: " << attempt << std::endl;
        robot->automaticErrorRecovery();
        if (attempt == nbAttempts)
          throw e;
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
        robot->control(force_control_callback);
#else
        robot->control(force_cart_control_callback, true, cutoff_frequency);
#endif
        break;
      } catch (const franka::ControlException &e) {
        std::cerr << "Warning: communication error: " << e.what() << "\nRetry attempt: " << attempt << std::endl;
        robot->automaticErrorRecovery();
        if (attempt == nbAttempts)
          throw e;
      }
    }
    break;
  }
  case vpRobot::MIXT_FRAME: {
    throw vpException(vpException::fatalError, "Force/torque controller not supported");
  }
  }
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_robot.a(vpForceTorqueGenerator.cpp.o) has no symbols
void dummy_vpForceTorqueGenerator(){};
#endif // VISP_HAVE_FRANKA

