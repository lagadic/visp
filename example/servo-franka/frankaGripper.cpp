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
 * Franka robot tool.
 *
 *****************************************************************************/

/*!
  \example frankaGripper.cpp
  Move Panda robot from Franka Emika to a position specified from a file.
*/

#include <iostream>

#include <visp3/robot/vpRobotFranka.h>

#if defined(VISP_HAVE_FRANKA)

int main(int argc, char **argv)
{
  std::string opt_robot_ip = "192.168.1.1";
  double opt_grasping_width = 0.;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
      opt_robot_ip = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--grasping_width" && i + 1 < argc) {
      opt_grasping_width = std::atof(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Open/close Panda gripper to grasp an object." << std::endl;
      std::cout << argv[0] << " [--ip <default " << opt_robot_ip << ">] [--gripper <0: close, 1: open>] [--grasping_width <object width in meter>] [--help] [-h]\n" << std::endl;
      std::cout << "Example to close gripper grasping a 4cm width object :\n"
                << argv[0] << " --ip 192.168.100.1 --grasping_width 0.04\n" << std::endl;

      return EXIT_SUCCESS;
    }
  }

  vpRobotFranka robot;

  try {
    robot.connect(opt_robot_ip);

    std::cout << "Perform gripper homing..." << std::endl;
    robot.gripperHoming();

    std::cout << "Close gripper..." << std::endl;
    robot.gripperClose();
    vpTime::sleepMs(3000);
    std::cout << "Open gripper to 5cm..." << std::endl;
    robot.gripperMove(0.05);
    vpTime::sleepMs(3000);
    std::cout << "Open gripper completely..." << std::endl;
    robot.gripperOpen();
    vpTime::sleepMs(3000);

    if (opt_grasping_width > 0) {
      std::cout << "Grasp " << opt_grasping_width << "m object width..." << std::endl;
      robot.gripperGrasp(opt_grasping_width);
      vpTime::sleepMs(3000);
    }

    std::cout << "Release gripper..." << std::endl;
    robot.gripperRelease();
    vpTime::sleepMs(1000);
    std::cout << "The end!" << std::endl;
  }
  catch(const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    std::cout << "Stop the robot " << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
    return EXIT_FAILURE;
  }
  catch(const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
              << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. " << std::endl;
    return EXIT_FAILURE;
  }
  catch(const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return 0;
}
#else
int main()
{
#if !defined(VISP_HAVE_FRANKA)
  std::cout << "Install libfranka." << std::endl;
#endif
  return 0;
}
#endif

