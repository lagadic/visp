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
 *   Test Franka robot behavior
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testFrankaJointVelocity-2.cpp

  Test Panda robot from Franka Emika joint velocity controller implemented in vpRobotFranka.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_FRANKA)

#include <visp3/robot/vpRobotFranka.h>

int main(int argc, char **argv)
{
  std::string robot_ip = "192.168.1.1";

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
      robot_ip = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << argv[0] << " [--ip 192.168.1.1] [--help] [-h]"
                           << "\n";
      return EXIT_SUCCESS;
    }
  }

  try {
    vpRobotFranka robot;
    robot.connect(robot_ip);

    std::cout << "WARNING: This example will move the robot! "
              << "Please make sure to have the user stop button at hand!" << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    /*
     * Move to a safe position
     */
    vpColVector q(7, 0);
    q[3] = -M_PI_2;
    q[5] = M_PI_2;
    q[6] = M_PI_4;
    std::cout << "Move to joint position: " << q.t() << std::endl;
    robot.setPosition(vpRobot::JOINT_STATE, q);

    /*
     * Move in joint velocity
     */
    vpColVector dq_d(7, 0);
    dq_d[0] = vpMath::rad(-2.);
    dq_d[1] = vpMath::rad(3.);
    dq_d[2] = vpMath::rad(-4.);
    dq_d[3] = vpMath::rad(3.);
    dq_d[4] = vpMath::rad(-3.);
    dq_d[5] = vpMath::rad(4.);
    dq_d[6] = vpMath::rad(6.);
    double t0 = vpTime::measureTimeSecond();
    double delta_t = 4.0; // Time in second

    std::cout << "Apply joint vel in a loop for " << delta_t << " sec : " << dq_d.t() << std::endl;
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    do {
      robot.setVelocity(vpRobot::JOINT_STATE, dq_d);

      vpTime::wait(10);
    } while (vpTime::measureTimeSecond() - t0 < delta_t);

    for (size_t i=0; i < 7; i++)
      dq_d[i] = -dq_d[i];

    std::cout << "Apply new joint vel for " << delta_t << " sec : " << dq_d.t() << std::endl;
    robot.setVelocity(vpRobot::JOINT_STATE, dq_d);
    vpTime::wait(4*1000);

    std::cout << "Ask to stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
  }
  catch(const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch(const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
              << " or if you specified the right IP using --ip command"
              << " line option set by default to 192.168.1.1. " << std::endl;
    return EXIT_FAILURE;
  }
  catch(const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "The end" << std::endl;
  return EXIT_SUCCESS;
}

#else
int main()
{
  std::cout << "ViSP is not build with libfranka..." << std::endl;
}
#endif
