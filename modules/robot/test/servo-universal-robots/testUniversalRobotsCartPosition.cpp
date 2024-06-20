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
  \example testUniversalRobotsCartPosition.cpp

  Test robot from Universal Robots cartesian positioning controller implemented in vpRobotUniversalRobots.
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
    std::cout << "Connected robot model: " << robot.getRobotModel() << std::endl;

    std::cout << "WARNING: This example will move the robot! "
      << "Please make sure to have the user stop button at hand!" << std::endl
      << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    /*
     * Move to a safe position
     */
    vpColVector q(6, 0);
    q[0] = 0;
    q[1] = -M_PI_2;
    q[2] = M_PI_2;
    q[3] = -M_PI_2;
    q[4] = -M_PI_2;
    q[5] = 0;
    std::cout << "Move to joint position: " << q.t() << std::endl;
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    robot.setPosition(vpRobot::JOINT_STATE, q);

    // Get current cartesian position
    vpHomogeneousMatrix fMe = robot.get_fMe();

    // Target 10 cm up in the Z-Axis of the end-effector
    fMe[2][3] += 0.1;

    // Move to cartesian position
    robot.setPosition(vpRobot::END_EFFECTOR_FRAME, vpPoseVector(fMe));

    // Come back to initial position
    robot.setPosition(vpRobot::JOINT_STATE, q);

    // Get current cartesian position
    vpHomogeneousMatrix fMc = robot.get_fMc();

    // Target 10 cm up in the Z-Axis of the camera frame
    fMc[2][3] += 0.1;

    // Move to cartesian position
    robot.setPosition(vpRobot::CAMERA_FRAME, vpPoseVector(fMc));

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
