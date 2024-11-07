/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * Test Franka robot behavior
 */

/*!
  \example testFrankaCartForceTorque-2.cpp

  Test Panda robot from Franka Emika cartesian force/torque controller implemented in vpRobotFranka.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_FRANKA)

#include <visp3/robot/vpRobotFranka.h>

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  std::string robot_ip = "192.168.1.1";
  std::string log_folder;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
      robot_ip = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--log_folder" && i + 1 < argc) {
      log_folder = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0] << " [--ip 192.168.1.1] [--log_folder <folder>] [--help] [-h]"
        << "\n";
      return EXIT_SUCCESS;
    }
  }

  try {
    vpRobotFranka robot;
    robot.setLogFolder(log_folder);
    robot.connect(robot_ip);

    std::cout << "WARNING: This example will move the robot! " << std::endl
      << "- Please make sure to have the user stop button at hand!" << std::endl
      << "- Please make also sure the end-effector is in contact with a flat surface such as a foam board!"
      << std::endl
      << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    /*
     * Apply joint torque
     */
    vpColVector ft_d(6, 0);
    ft_d[2] = -2;

    double t0 = vpTime::measureTimeSecond();
    double delta_t = 12.0; // Time in second

    std::cout << "Apply cartesian force/torque in a loop for " << delta_t / 2. << " sec : " << ft_d.t() << std::endl;
    robot.setRobotState(vpRobot::STATE_FORCE_TORQUE_CONTROL);
    double filter_gain = 0.15;
    bool activate_pi_controller = true;
    do {
      robot.setForceTorque(vpRobot::END_EFFECTOR_FRAME, ft_d, filter_gain,
                           activate_pi_controller); // Use low level PI controller
      if (vpTime::measureTimeSecond() - t0 > delta_t / 2.) {
        ft_d[2] = -10;
        static bool change_ft = true;
        if (change_ft) {
          std::cout << "Apply cartesian force/torque in a loop for " << delta_t / 2. << " sec : " << ft_d.t()
            << std::endl;
        }
        change_ft = false;
      }
      vpTime::wait(10); // wait 10 ms
    } while (vpTime::measureTimeSecond() - t0 < delta_t);

    robot.setRobotState(vpRobot::STATE_STOP);
    vpTime::wait(100);
  }
  catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
      << " or if you specified the right IP using --ip command"
      << " line option set by default to 192.168.1.1. " << std::endl;
    return EXIT_FAILURE;
  }
  catch (const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "The end" << std::endl;
  return EXIT_SUCCESS;
}

#else
int main() { std::cout << "ViSP is not build with libfranka..." << std::endl; }
#endif
