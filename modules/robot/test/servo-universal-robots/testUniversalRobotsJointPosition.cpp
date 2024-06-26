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
 * Test Universal Robot behavior.
 */

/*!
  \example testUniversalRobotsJointPosition.cpp

  Test robot from Universal Robot joint positioning controller implemented in vpRobotUniversalRobots.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_UR_RTDE)

#include <visp3/robot/vpRobotUniversalRobots.h>

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  std::string robot_ip = "192.168.0.100";

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
      robot_ip = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0] << " [--ip " << robot_ip << "] [--help] [-h]"
        << "\n";
      return EXIT_SUCCESS;
    }
  }

  try {
    vpRobotUniversalRobots robot;
    robot.connect(robot_ip);

    std::cout << "WARNING: This example will move the robot! "
      << "Please make sure to have the user stop button at hand!" << std::endl
      << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // Get current position
    vpColVector q_init, q;
    robot.getPosition(vpRobot::JOINT_STATE, q);

    // Backup initial joint position
    q_init = q;

    // Enable position controller
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);

    // Move last joint to 0
    q[5] = vpMath::rad(0);
    std::cout << "Move to joint position [rad]: " << q.t() << std::endl;
    robot.setPosition(vpRobot::JOINT_STATE, q);

    // Move last joint to +90 deg
    q[5] = vpMath::rad(90);
    std::cout << "Move to joint position [rad]: " << q.t() << std::endl;
    robot.setPosition(vpRobot::JOINT_STATE, q);

    // Move last joint to -90 deg
    q[5] = vpMath::rad(-90);
    std::cout << "Move to joint position [rad]: " << q.t() << std::endl;
    robot.setPosition(vpRobot::JOINT_STATE, q);

    // Move back to initial position
    std::cout << "Move to joint position [rad]: " << q_init.t() << std::endl;
    robot.setPosition(vpRobot::JOINT_STATE, q_init);
  }
  catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (const std::exception &e) {
    std::cout << "ur_rtde exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "The end" << std::endl;
  return EXIT_SUCCESS;
}

#else
int main()
{
  std::cout << "ViSP is not build with libur_rtde 3rd party used to control a robot from Universal Robots..."
    << std::endl;
}
#endif
