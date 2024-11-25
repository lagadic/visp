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
 * Franka robot tool.
 */

/*!
  \example frankaGripper.cpp
  Move Panda robot from Franka Emika to a position specified from a file.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotFranka.h>

#if defined(VISP_HAVE_FRANKA)

typedef enum
{
  Gripper_Home,
  Gripper_Open,
  Gripper_Close,
  Gripper_Grasp,
  Gripper_Release,
  Gripper_Test,
  Gripper_None
} GripperState_t;

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string opt_robot_ip = "192.168.1.1";
  double opt_grasping_width = 0.;
  double opt_grasping_speed = 0.1;
  double opt_grasping_force = 60;
  GripperState_t opt_gripper_state = Gripper_None;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
      ++ i;
      opt_robot_ip = std::string(argv[i]);
    }
    else if (std::string(argv[i]) == "--home") {
      opt_gripper_state = Gripper_Home;
    }
    else if (std::string(argv[i]) == "--open") {
      opt_gripper_state = Gripper_Open;
    }
    else if (std::string(argv[i]) == "--close") {
      opt_gripper_state = Gripper_Close;
    }
    else if (std::string(argv[i]) == "--grasp" && i + 1 < argc) {
      ++ i;
      opt_gripper_state = Gripper_Grasp;
      opt_grasping_width = std::atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--grasp-speed" && i + 1 < argc) {
      ++ i;
      opt_grasping_speed = std::atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--grasp-force" && i + 1 < argc) {
      ++ i;
      opt_grasping_force = std::atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--test" && i + 1 < argc) {
      ++ i;
      opt_gripper_state = Gripper_Test;
      opt_grasping_width = std::atof(argv[i]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "SYNOPSYS" << std::endl
        << "  " << argv[0]
        << " [--ip <default " << opt_robot_ip << ">]"
        << " [--home]"
        << " [--open]"
        << " [--close]"
        << " [--grasp <width>]"
        << " [--grasp-speed <speed>]"
        << " [--grasp-force <speed>]"
        << " [--release]"
        << " [--test <width>]"
        << " [--help] [-h]\n" << std::endl;
      std::cout << "DESCRIPTION" << std::endl
        << "  Control Panda gripper." << std::endl
        << std::endl
        << "  --ip <controller ip>" << std::endl
        << "    Franka controller ip address" << std::endl
        << "    Default: " << opt_robot_ip << std::endl
        << std::endl
        << "  --home" << std::endl
        << "    Performs homing of the gripper." << std::endl
        << std::endl
        << "  --open" << std::endl
        << "    Performs opening of the gripper." << std::endl
        << std::endl
        << "  --close" << std::endl
        << "    Performs closing of the gripper." << std::endl
        << std::endl
        << "  --grasp <object width>" << std::endl
        << "    Performs grasping of an object." << std::endl
        << "    Object width is to specify in [m]." << std::endl
        << "    Default width: " << opt_grasping_width << " [m]" << std::endl
        << std::endl
        << "  --grasp-speed <speed>" << std::endl
        << "    Closing speed in [m/s] applied during grasping." << std::endl
        << "    Default: " << opt_grasping_speed << " [m/s]" << std::endl
        << std::endl
        << "  --grasp-force <force>" << std::endl
        << "    Force in [N] applied during grasping." << std::endl
        << "    Default: " << opt_grasping_force << " [N]" << std::endl
        << std::endl
        << "  --release" << std::endl
        << "    Release an object that is grasped." << std::endl
        << std::endl
        << "  --test <object width>" << std::endl
        << "    Performs a gripper test on an object width specified in [m]." << std::endl
        << "    Default width: " << opt_grasping_width << " [m]" << std::endl
        << std::endl
        << "  --help, -h" << std::endl
        << "    Print this helper message." << std::endl
        << std::endl;
      std::cout << "EXAMPLE" << std::endl
        << "  To grasp a 4cm width object by first opening the gripper, then grasping the object:\n"
        << "  " << argv[0] << " --ip 192.168.100.1 --open\n"
        << "  " << argv[0] << " --ip 192.168.100.1 --grasp 0.04\n"
        << std::endl;

      return EXIT_SUCCESS;
    }
  }

  if (opt_gripper_state == Gripper_None) {
    std::cout << "Specify which action you want to achieve. Run \"" << argv[0]
      << " --help\" to see how to use this binary." << std::endl;
    return EXIT_SUCCESS;
  }
  else if ((opt_gripper_state == Gripper_Grasp || opt_gripper_state == Gripper_Test) && opt_grasping_width <= 0) {
    std::cout << "Object with in meter should be > 0. Run \"" << argv[0] << " --help\" to see how to use this binary."
      << std::endl;
    return EXIT_SUCCESS;
  }

  vpRobotFranka robot;

  try {
    robot.connect(opt_robot_ip);

    if (opt_gripper_state == Gripper_Home) {
      std::cout << "Gripper homing..." << std::endl;
      robot.gripperHoming();
    }
    else if (opt_gripper_state == Gripper_Close) {
      std::cout << "Gripper closing..." << std::endl;
      robot.gripperClose();
    }
    else if (opt_gripper_state == Gripper_Open) {
      std::cout << "Gripper opening..." << std::endl;
      robot.gripperOpen();
    }
    else if (opt_gripper_state == Gripper_Grasp) {
      std::cout << "Gripper grasp " << opt_grasping_width << "m object width, with speed: " << opt_grasping_speed << " and force: " << opt_grasping_force << "N..." << std::endl;
      robot.gripperGrasp(opt_grasping_width, opt_grasping_speed, opt_grasping_force);
    }
    else if (opt_gripper_state == Gripper_Release) {
      std::cout << "Gripper release object..." << std::endl;
      robot.gripperRelease();
    }
    else if (opt_gripper_state == Gripper_Test) {
      std::cout << "Test gripper performing the following actions:" << std::endl;
      std::cout << "- Gripper homing..." << std::endl;
      robot.gripperHoming();
      vpTime::sleepMs(1000);
      std::cout << "- Gripper closing..." << std::endl;
      robot.gripperClose();
      vpTime::sleepMs(1000);
      std::cout << "- Gripper open 5cm..." << std::endl;
      robot.gripperMove(0.05);
      vpTime::sleepMs(3000);
      std::cout << "- Gripper opening..." << std::endl;
      robot.gripperOpen();
      vpTime::sleepMs(3000);
      std::cout << "- Gripper grasp " << opt_grasping_width << "m object width, with speed: " << opt_grasping_speed << " and force: " << opt_grasping_force << "N..." << std::endl;
      robot.gripperGrasp(opt_grasping_width, opt_grasping_speed, opt_grasping_force);
      vpTime::sleepMs(3000);
      std::cout << "- Gripper release object..." << std::endl;
      robot.gripperRelease();
    }
    std::cout << "The end!" << std::endl;
  }
  catch (const vpException &e) {
    std::cout << "ViSP exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
  catch (const franka::NetworkException &e) {
    std::cout << "Franka network exception: " << e.what() << std::endl;
    std::cout << "Check if you are connected to the Franka robot"
      << " or if you specified the right IP using --ip command line option set by default to 192.168.1.1. "
      << std::endl;
    return EXIT_FAILURE;
  }
  catch (const std::exception &e) {
    std::cout << "Franka exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  return EXIT_SUCCESS;
}
#else
int main()
{
#if !defined(VISP_HAVE_FRANKA)
  std::cout << "Install libfranka 3rd party, configure and build again ViSP to use this example..." << std::endl;
#endif
  return EXIT_SUCCESS;
}
#endif
