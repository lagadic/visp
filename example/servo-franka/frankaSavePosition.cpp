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
 * Franka robot tool.
 *
 *****************************************************************************/

/*!
  \example frankaSavePosition.cpp
  Save Panda robot position in file position.pos.
*/

#include <iostream>

#include <visp3/robot/vpRobotFranka.h>

#if defined(VISP_HAVE_FRANKA)

int main(int argc, char **argv)
{
  std::string opt_robot_ip = "192.168.1.1";
  std::string opt_position_filename = "position.pos";

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
      opt_robot_ip = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--save" && i + 1 < argc) {
      opt_position_filename = std::string(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "Save Panda robot position in a file." << std::endl;
      std::cout << "Usage:\n" << std::endl;
      std::cout << argv[0] << " [--ip <default " << opt_robot_ip << ">] [--save <position file name>] [--help] [-h]\n" << std::endl;
      std::cout << "Example:\n" << argv[0] << " --ip 192.168.100.1 --save position.pos\n" << std::endl;
                            
      return EXIT_SUCCESS;
    }
  }

  vpRobotFranka robot;

  try {
    robot.connect(opt_robot_ip);

    vpColVector q;
    robot.getPosition(vpRobot::JOINT_STATE, q);
    robot.savePosFile(opt_position_filename, q);

    std::cout << "Robot position saved in \"" << opt_position_filename << "\""<< std::endl;
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

