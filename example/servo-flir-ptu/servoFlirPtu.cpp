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
 *   Tests the control law
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FLIR_PTU_SDK

#include <visp3/robot/vpRobotFlirPtu.h>
#include <visp3/core/vpTime.h>

int main(int argc, char *argv[0])
{
  try {
    std::string opt_portname;
    int opt_baudrate = 9600;

    for (int i = 1; i < argc; i++) {
      if ((std::string(argv[i]) == "--portname" || std::string(argv[i]) == "-p") && (i + 1 < argc)) {
        opt_portname = std::string(argv[i + 1]);
      }
      else if ((std::string(argv[i]) == "--baudrate" || std::string(argv[i]) == "-b") && (i + 1 < argc)) {
        opt_baudrate = std::atoi(argv[i + 1]);
      }
      else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
        std::cout << "SYNOPSIS" << std::endl
                  << "  " << argv[0] << " [--portname <portname>] [--baudrate <rate>] [--help] [-h]" << std::endl << std::endl
                  << "DESCRIPTION" << std::endl
                  << "  --portname, -p <portname>" << std::endl
                  << "    Set serial or tcp port name." << std::endl << std::endl
                  << "  --baudrate, -b <rate>" << std::endl
                  << "    Set serial communication baud rate. Default: " << opt_baudrate << "." << std::endl << std::endl
                  << "  --help, -h" << std::endl
                  << "    Print this helper message. " << std::endl << std::endl
#ifdef _WIN32
                  << "EXAMPLE" << std::endl
                  << "  " << argv[0] << " -p COM1" << std::endl;
#else
                  << "EXAMPLE" << std::endl
                  << "  " << argv[0] << " -p /dev/ttyUSB0" << std::endl;
#endif


        return EXIT_SUCCESS;
      }
    }

    if (opt_portname.empty()) {
      std::cout << "Error, portname unspecified. Run " << argv[0] << " --help" << std::endl;
      return EXIT_SUCCESS;
    }

    vpRobotFlirPtu robot;
    vpColVector q(2), q_mes;

    std::cout << "Try to connect FLIR PTU to port: " << opt_portname << " with baudrate: " << opt_baudrate << std::endl;
    robot.connect(opt_portname, opt_baudrate);

    std::cout << "Pan  pos min/max [deg]: " << vpMath::deg(robot.getPosMin()[0]) << " " << vpMath::deg(robot.getPosMax()[0]) << std::endl;
    std::cout << "Tilt pos min/max [deg]: " << vpMath::deg(robot.getPosMin()[1]) << " " << vpMath::deg(robot.getPosMax()[1]) << std::endl;
    std::cout << "Pan/tilt vel max [deg/s]: " << vpMath::deg(robot.getVelMax()[0]) << " " << vpMath::deg(robot.getVelMax()[1]) << std::endl;

    robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
    robot.setMaxRotationVelocity(robot.getVelMax()[0] / 2.);

    q = 0;
    std::cout << "Set joint position [deg]: " << vpMath::deg(q[0]) << " " << vpMath::deg(q[1]) << std::endl;
    robot.setPositioningVelocity(50);
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
    robot.getPosition(vpRobot::ARTICULAR_FRAME, q_mes);

    std::cout << "Position reached [deg]: " << vpMath::deg(q_mes[0]) << " " << vpMath::deg(q_mes[1]) << std::endl;
    std::cout << "Positionning achieved. Enter a caracter to continue" << std::endl;
    std::cin.get();

    q[0] = vpMath::rad(10); // Pan  position in rad
    q[1] = vpMath::rad(20); // Tilt position in rad

    std::cout << "Set joint position [deg]: " << vpMath::deg(q[0]) << " " << vpMath::deg(q[1]) << std::endl;
    robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
    robot.getPosition(vpRobot::ARTICULAR_FRAME, q_mes);

    std::cout << "Position reached [deg]: " << vpMath::deg(q_mes[0]) << " " << vpMath::deg(q_mes[1]) << std::endl;
    std::cout << "Positionning achieved. Enter a caracter to continue" << std::endl;
    std::cin.get();

    vpColVector qdot(2);
    qdot[0] = vpMath::rad(-10); // Pan  velocity in rad/s
    qdot[1] = vpMath::rad(-10); // Tilt velocity in rad/s

    std::cout << "Set velocity [deg/s]: " << vpMath::deg(qdot[0]) << " " << vpMath::deg(qdot[1]) << std::endl;
    robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);
    robot.setVelocity(vpRobot::JOINT_STATE, qdot) ;
    vpTime::sleepMs(2000);

    robot.stopMotion();

    std::cout << "The end" << std::endl;

  } catch (const vpException &e) {
    std::cout << "Catch Flir Ptu exception: " << e.getMessage() << std::endl;
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
#else
int main()
{
  std::cout << "You do not have an Flir Ptu robot connected to your computer..." << std::endl;
  return EXIT_SUCCESS;
}

#endif

