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
 * Example with Kinova Jaco robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example servoKinovaJaco.cpp

  Example with Kinova Jaco robot.

*/

#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotKinova.h>

int main(int argc, char*argv[])
{
#ifdef VISP_HAVE_JACOSDK
  std::string opt_plugin_path;
  bool opt_verbose = false;

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--plugin" && i + 1 < argc) {
      opt_plugin_path = std::string(argv[i + 1]);;
    }
    else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "SYNOPSYS" << std::endl
        << "  " << argv[0] << " [--plugin <path>] "
        << "[--verbose] [--help] [-v] [-h]\n" << std::endl;
      std::cout << "DESCRIPTION" << std::endl
        << "  --plugin <path>" << std::endl
        << "      Path to Jaco SDK .so or .dll plugin location" << std::endl << std::endl
        << "  --verbose, -v" << std::endl
        << "      Enable verbose mode to print addition information" << std::endl << std::endl
        << "  --help, -h" << std::endl
        << "      Print this helper message." << std::endl;

      return EXIT_SUCCESS;
    }
    }

  try {
    vpRobotKinova robot;
    robot.setVerbose(true);

    robot.loadPlugin();

    robot.homing();

    vpColVector q;
    robot.getPosition(vpRobot::JOINT_STATE, q);
    q[0] += vpMath::rad(30);
    robot.setPosition(vpRobot::JOINT_STATE, q);



  }
  catch (const vpException &e) {
    std::cout << "Catch exception: " << e.getStringMessage() << std::endl;
  }

  std::cout << "The end" << std::endl;
#else
  std::cout << "Install Jaco SDK, configure and build again ViSP to use this example..." << std::endl;
#endif
}
