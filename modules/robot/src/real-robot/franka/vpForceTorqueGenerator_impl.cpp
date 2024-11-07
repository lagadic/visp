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

#include "vpForceTorqueGenerator_impl.h"

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FRANKA
#include <algorithm>
#include <cmath>
#include <iomanip>

#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpTime.h>

#include <Eigen/Core> // TODO: Remove when initial_position var is removed

BEGIN_VISP_NAMESPACE
vpColVector ft_cart_des_prev(6, 0);

void vpForceTorqueGenerator::control_thread(franka::Robot *robot, std::atomic_bool &stop, const std::string &log_folder,
                                            const vpRobot::vpControlFrameType &frame,
                                            const std::array<double, 7> &tau_J_des, const vpColVector &ft_cart_des,
                                            franka::RobotState &robot_state, std::mutex &mutex,
                                            const double &filter_gain, const bool &activate_pi_controller)
{
  double time = 0.0;

  Eigen::Vector3d initial_position;
  franka::Model model = robot->loadModel();

  Eigen::VectorXd initial_tau_ext(7), tau_error_integral(7);
  double k_p = 1.0;
  double k_i = 2.0;

  // Bias torque sensor
  std::array<double, 7> gravity_array = model.gravity(robot_state);
  Eigen::Map<Eigen::Matrix<double, 7, 1> > initial_tau_measured(robot_state.tau_J.data());
  Eigen::Map<Eigen::Matrix<double, 7, 1> > initial_gravity(gravity_array.data());
  initial_tau_ext = initial_tau_measured - initial_gravity;

  franka::Torques zero_torques { {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0} };
  Eigen::VectorXd tau_joint_d(7), tau_cart_d(6);
  for (size_t i = 0; i < 7; i++) {
    tau_joint_d[i] = 0;
  }
  for (size_t i = 0; i < 6; i++) {
    tau_cart_d[i] = 0;
  }

  // init integrator
  tau_error_integral.setZero();

  if (!activate_pi_controller) {
    k_p = 0.;
    k_i = 0.;
  }

  if (!log_folder.empty()) {
    std::cout << "Save franka logs in \"" << log_folder << "\" folder" << std::endl;
    std::cout << "Use gnuplot tool to visualize logs:" << std::endl;
    std::cout << "$ cd " << log_folder << std::endl;
    std::cout << "$ gnuplot plot.gp" << std::endl;
    std::cout << "<press return key in the terminal to view next plot>" << std::endl;

    std::ofstream gnuplot;
    gnuplot.open(log_folder + "/plot.gp");
    gnuplot << "set st data li\n" << std::endl;
    gnuplot << "plot ";
    for (size_t i = 0; i < 7; i++) {
      gnuplot << "'tau_cmd.log' u " << i + 1 << " title \"tau_cmd" << i + 1 << "\", ";
    }
    gnuplot << "\npause -1\n" << std::endl;
    gnuplot << "plot ";
    for (size_t i = 0; i < 7; i++) {
      gnuplot << "'tau_d.log' u " << i + 1 << " title \"tau_d" << i + 1 << "\", ";
    }
    gnuplot << "\npause -1\n" << std::endl;
    gnuplot << "plot ";
    for (size_t i = 0; i < 7; i++) {
      gnuplot << "'tau_mes.log' u " << i + 1 << " title \"tau_mes" << i + 1 << "\", ";
    }
    gnuplot << "\npause -1\n" << std::endl;

    if (frame == vpRobot::CAMERA_FRAME || frame == vpRobot::REFERENCE_FRAME || frame == vpRobot::END_EFFECTOR_FRAME) {
      for (size_t i = 0; i < 7; i++) {
        gnuplot << "plot 'tau_d.log' u " << i + 1 << " title \"tau_d" << i + 1 << "\", 'tau_mes.log' u " << i + 1
          << " title \"tau_mes" << i + 1 << "\"" << std::endl;
        gnuplot << "\npause -1\n" << std::endl;
      }
    }

    gnuplot.close();
  }

  std::ofstream log_time;
  std::ofstream log_tau_cmd;
  std::ofstream log_tau_d;
  std::ofstream log_tau_mes;
  std::ofstream log_tau_diff;
  std::ofstream log_tau_diff_prev;

  auto force_joint_control_callback =
    [=, &log_time, &log_tau_cmd, &log_tau_d, &log_tau_mes, &time, &model, &initial_position, &stop, &zero_torques,
    &robot_state, &tau_joint_d, &tau_error_integral, &mutex,
    &tau_J_des](const franka::RobotState &state, franka::Duration period) -> franka::Torques {
    time += period.toSec();

    if (time == 0.0) {
      if (!log_folder.empty()) {
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

    Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_measured(state.tau_J.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());

    Eigen::VectorXd tau_cmd(7), tau_ext(7), desired_tau(7);
    tau_ext << tau_measured - gravity - initial_tau_ext;

    tau_error_integral += period.toSec() * (tau_joint_d - tau_ext);

    // Smoothly update the mass to reach the desired target value
    for (size_t i = 0; i < 7; i++) {
      tau_joint_d[i] = filter_gain * tau_J_des[i] + (1 - filter_gain) * tau_joint_d[i];
    }

    // FF + PI control
    tau_cmd << tau_joint_d + k_p * (tau_joint_d - tau_ext) + k_i * tau_error_integral;

    std::array<double, 7> tau_d_array {};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;

    if (!log_folder.empty()) {
      log_time << time << std::endl;
      log_tau_cmd << std::fixed << std::setprecision(8) << tau_cmd[0] << " " << tau_cmd[1] << " " << tau_cmd[2] << " "
        << tau_cmd[3] << " " << tau_cmd[4] << " " << tau_cmd[5] << " " << tau_cmd[6] << std::endl;
      log_tau_d << std::fixed << std::setprecision(8) << tau_joint_d[0] << " " << tau_joint_d[1] << " "
        << tau_joint_d[2] << " " << tau_joint_d[3] << " " << tau_joint_d[4] << " " << tau_joint_d[5] << " "
        << tau_joint_d[6] << std::endl;
      log_tau_mes << std::fixed << std::setprecision(8) << tau_ext[0] << " " << tau_ext[1] << " " << tau_ext[2] << " "
        << tau_ext[3] << " " << tau_ext[4] << " " << tau_ext[5] << " " << tau_ext[6] << std::endl;
    }

    if (stop) {
      if (!log_folder.empty()) {
        log_time.close();
        log_tau_cmd.close();
        log_tau_d.close();
        log_tau_mes.close();
      }
      return franka::MotionFinished(zero_torques);
    }

    return tau_d_array;
    };

  auto force_cart_control_callback = [=, &log_time, &log_tau_cmd, &log_tau_d, &log_tau_mes, &log_tau_diff,
    &log_tau_diff_prev, &time, &model, /*&initial_position, */
    &stop, &zero_torques, &robot_state, &tau_cart_d, &tau_error_integral, &mutex,
    &ft_cart_des](const franka::RobotState &state,
                  franka::Duration period) -> franka::Torques {
  time += period.toSec();

  Eigen::VectorXd tau_d(7), tau_cmd(7), tau_ext(7), desired_tau(7);

  if (time == 0.0) {
    tau_d << 0, 0, 0, 0, 0, 0, 0;
    tau_ext << 0, 0, 0, 0, 0, 0, 0;

    if (!log_folder.empty()) {
      log_time.open(log_folder + "/time.log");
      log_tau_cmd.open(log_folder + "/tau_cmd.log");
      log_tau_d.open(log_folder + "/tau_d.log");
      log_tau_mes.open(log_folder + "/tau_mes.log");
      log_tau_diff.open(log_folder + "/tau_diff.log");
      log_tau_diff_prev.open(log_folder + "/tau_diff_prev.log");
    }
  }

  {
    std::lock_guard<std::mutex> lock(mutex);
    robot_state = state;
  }

  // get state variables
  std::array<double, 42> jacobian_array = model.zeroJacobian(franka::Frame::kEndEffector, state);

  Eigen::Map<const Eigen::Matrix<double, 6, 7> > jacobian(jacobian_array.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > tau_measured(state.tau_J.data());
  Eigen::Map<const Eigen::Matrix<double, 7, 1> > gravity(gravity_array.data());

  tau_ext << tau_measured - gravity - initial_tau_ext;

  tau_error_integral += period.toSec() * (tau_d - tau_ext);

  // Apply force with gradually increasing the force
  for (size_t i = 0; i < 7; i++) {
    tau_cart_d[i] = filter_gain * ft_cart_des[i] + (1 - filter_gain) * tau_cart_d[i];
  }

  tau_d << jacobian.transpose() * tau_cart_d;

  // FF + PI control
  tau_cmd << tau_d + k_p * (tau_d - tau_ext) + k_i * tau_error_integral;

  std::array<double, 7> tau_d_array {};
  Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_cmd;

  if (!log_folder.empty()) {
    log_time << time << std::endl;
    log_tau_cmd << std::fixed << std::setprecision(8) << tau_cmd[0] << " " << tau_cmd[1] << " " << tau_cmd[2] << " "
      << tau_cmd[3] << " " << tau_cmd[4] << " " << tau_cmd[5] << " " << tau_cmd[6] << std::endl;
    log_tau_d << std::fixed << std::setprecision(8) << tau_d[0] << " " << tau_d[1] << " " << tau_d[2] << " "
      << tau_d[3] << " " << tau_d[4] << " " << tau_d[5] << " " << tau_d[6] << std::endl;
    log_tau_mes << std::fixed << std::setprecision(8) << tau_ext[0] << " " << tau_ext[1] << " " << tau_ext[2] << " "
      << tau_ext[3] << " " << tau_ext[4] << " " << tau_ext[5] << " " << tau_ext[6] << std::endl;
    log_tau_diff << std::fixed << std::setprecision(8);
    for (size_t i = 0; i < ft_cart_des.size(); i++) {
      log_tau_diff << ft_cart_des[i] - tau_cart_d[i] << " ";
    }
    log_tau_diff << std::endl;

    log_tau_diff_prev << std::fixed << std::setprecision(8);
    for (size_t i = 0; i < ft_cart_des.size(); i++) {
      log_tau_diff_prev << ft_cart_des[i] - ft_cart_des_prev[i] << " ";
    }
    log_tau_diff_prev << std::endl;
  }

  if (stop) {
    if (!log_folder.empty()) {
      log_time.close();
      log_tau_cmd.close();
      log_tau_d.close();
      log_tau_mes.close();
      log_tau_diff.close();
      log_tau_diff_prev.close();
    }
    return franka::MotionFinished(zero_torques);
  }

  ft_cart_des_prev = ft_cart_des;

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
        robot->control(force_joint_control_callback, true, cutoff_frequency);
        break;
      }
      catch (const franka::ControlException &e) {
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
        robot->control(force_cart_control_callback, true, cutoff_frequency);
        break;
      }
      catch (const franka::ControlException &e) {
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
END_VISP_NAMESPACE
#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work around to avoid warning: libvisp_robot.a(vpForceTorqueGenerator.cpp.o) has no symbols
void dummy_vpForceTorqueGenerator() { };
#endif // VISP_HAVE_FRANKA
