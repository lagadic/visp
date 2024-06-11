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
 * Example with Kinova Jaco robot.
 *
*****************************************************************************/

/*!
  \example servoKinovaJacoCart.cpp

  Example that allows to control Kinova Jaco robot in cartesian.

*/

#include <iostream>

#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpRobotKinova.h>

int main(int argc, char *argv[])
{
#ifdef VISP_HAVE_JACOSDK
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string opt_plugin_path = "./";
  vpRobotKinova::CommandLayer opt_command_layer = vpRobotKinova::CMD_LAYER_UNSET;
  bool opt_verbose = false;
  unsigned int opt_dof = 6; // Consider a 6 DoF robot by default

  for (int i = 1; i < argc; i++) {
    if (std::string(argv[i]) == "--plugin" && i + 1 < argc) {
      opt_plugin_path = std::string(argv[i + 1]);
      ;
    }
    if ((std::string(argv[i]) == "--command_layer" || std::string(argv[i]) == "-l") && i + 1 < argc) {
      if (std::string(argv[i + 1]) == "usb") {
        opt_command_layer = vpRobotKinova::CMD_LAYER_USB;
      }
      else if (std::string(argv[i + 1]) == "ethernet") {
        opt_command_layer = vpRobotKinova::CMD_LAYER_ETHERNET;
      }
      else {
        opt_command_layer = vpRobotKinova::CMD_LAYER_UNSET;
      }
    }
    else if (std::string(argv[i]) == "--dof") {
      opt_dof = static_cast<unsigned int>(std::atoi(argv[i + 1]));
    }
    else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
      opt_verbose = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "SYNOPSYS" << std::endl
        << "  " << argv[0] << " [--plugin <path>] [--command_layer <name>] [--dof <4,6,7>] "
        << "[--verbose] [--help] [-v] [-h]\n"
        << std::endl;
      std::cout << "DESCRIPTION" << std::endl
        << "  --plugin <path>" << std::endl
        << "      Path to Jaco SDK .so or .dll plugin location. Default: \"./\"." << std::endl
        << std::endl
        << "  --command_layer <name>, -l <name>" << std::endl
        << "      Command layer name, either \"usb\" or \"ethernet\"." << std::endl
        << std::endl
        << "  --dof" << std::endl
        << "      Degrees of freedom. Possible values are 4, 6 or 7. Default value: 6." << std::endl
        << std::endl
        << "  --verbose, -v" << std::endl
        << "      Enable verbose mode to print addition information." << std::endl
        << std::endl
        << "  --help, -h" << std::endl
        << "      Print this helper message." << std::endl
        << std::endl;
      std::cout << "EXAMPLE" << std::endl
#ifdef __linux__
        << "  " << argv[0] << " --plugin /opt/JACO-SDK/API"
#elif _WIN32
        << "  " << argv[0] << " --plugin \"C:\\Program Files(x86)\\JACO - SDK\\API\\x64\""
#endif
        << " --command_layer usb" << std::endl
        << std::endl;

      return EXIT_SUCCESS;
    }
  }

  try {

    vpRobotKinova robot;
    robot.setDoF(opt_dof);
    robot.setVerbose(opt_verbose);
    robot.setPluginLocation(opt_plugin_path);
    robot.setCommandLayer(opt_command_layer);

    unsigned int n_devices = robot.connect();

    if (!n_devices) {
      std::cout << "There is no Kinova device connected." << std::endl;
      return EXIT_SUCCESS;
    }

    // Move robot to home position
    robot.homing();

    // Control robot in joint velocity
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    vpColVector vcart(6);
    vcart[1] = -0.10; // send 10 cm/s on along Y axis

    // Sent new joint velocities each 5 ms
    for (unsigned int i = 0; i < 300; i++) {
      // We send the velocity vector as long as we want the robot to move along that vector
      robot.setVelocity(vpRobot::END_EFFECTOR_FRAME, vcart);
      vpTime::wait(5);
    }

    // Control robot in joint position
    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    vpColVector p, p1, p2;

    // Move robot to home position
    robot.homing();

    // Get current cartesian position
    robot.getPosition(vpRobot::END_EFFECTOR_FRAME, p);

    // Move to first cartesian position
    p1 = p;
    p1[1] -= 0.1;
    robot.setPosition(vpRobot::END_EFFECTOR_FRAME, p1);

    // Move to second cartesian position
    p2 = p;
    p2[1] += 0.15;
    robot.setPosition(vpRobot::END_EFFECTOR_FRAME, p2);

    // Move back to home position
    robot.setPosition(vpRobot::END_EFFECTOR_FRAME, p);
  }
  catch (const vpException &e) {
    std::cout << "Catch exception: " << e.getStringMessage() << std::endl;
  }

  std::cout << "The end" << std::endl;
  return EXIT_SUCCESS;
#else
  (void)(argc);
  (void)(argv);
  std::cout << "Install Jaco SDK, configure and build again ViSP to use this example..." << std::endl;
#endif
}
