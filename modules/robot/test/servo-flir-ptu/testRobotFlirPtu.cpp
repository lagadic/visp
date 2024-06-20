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
 * Test FLIR PTU interface.
 */

/*!
  \example testRobotFlirPtu.cpp

  Test that show how to control FLIR PTU pan/tilt axis in position and velocity.
*/

#include <iostream>

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_FLIR_PTU_SDK

#include <visp3/robot/vpRobotFlirPtu.h>

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  std::string opt_portname;
  int opt_baudrate = 9600;
  bool opt_network = false;
  bool opt_reset = false;

  if (argc == 1) {
    std::cout << "To see how to use this test, run: " << argv[0] << " --help" << std::endl;
    return EXIT_SUCCESS;
  }

  for (int i = 1; i < argc; i++) {
    if ((std::string(argv[i]) == "--portname" || std::string(argv[i]) == "-p") && (i + 1 < argc)) {
      opt_portname = std::string(argv[i + 1]);
    }
    else if ((std::string(argv[i]) == "--baudrate" || std::string(argv[i]) == "-b") && (i + 1 < argc)) {
      opt_baudrate = std::atoi(argv[i + 1]);
    }
    else if ((std::string(argv[i]) == "--network" || std::string(argv[i]) == "-n")) {
      opt_network = true;
    }
    else if ((std::string(argv[i]) == "--reset" || std::string(argv[i]) == "-r")) {
      opt_reset = true;
    }
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "SYNOPSIS" << std::endl
        << "  " << argv[0] << " [--portname <portname>] [--baudrate <rate>] [--network] [--reset] [--help] [-h]"
        << std::endl
        << std::endl
        << "DESCRIPTION" << std::endl
        << "  --portname, -p <portname>" << std::endl
        << "    Set serial or tcp port name." << std::endl
        << std::endl
        << "  --baudrate, -b <rate>" << std::endl
        << "    Set serial communication baud rate. Default: " << opt_baudrate << "." << std::endl
        << std::endl
        << "  --network, -n" << std::endl
        << "    Get PTU network information (Hostname, IP, Gateway) and exit. " << std::endl
        << std::endl
        << "  --reset, -r" << std::endl
        << "    Reset PTU axis and exit. " << std::endl
        << std::endl
        << "  --help, -h" << std::endl
        << "    Print this helper message. " << std::endl
        << std::endl
        << "EXAMPLE" << std::endl
        << "  - How to get network IP" << std::endl
#ifdef _WIN32
        << "    $ " << argv[0] << " -p /dev/ttyUSB0 --network" << std::endl
#else
        << "    $ " << argv[0] << " --portname COM1 --network" << std::endl
#endif
        << "    Try to connect FLIR PTU to port: /dev/ttyUSB0 with baudrate: 9600" << std::endl
        << "       PTU HostName: PTU-5" << std::endl
        << "       PTU IP      : 169.254.110.254" << std::endl
        << "       PTU Gateway : 0.0.0.0" << std::endl
        << "  - How to run this binary using serial communication" << std::endl
#ifdef _WIN32
        << "    $ " << argv[0] << " --portname COM1" << std::endl
#else
        << "    $ " << argv[0] << " --portname /dev/ttyUSB0" << std::endl
#endif
        << "  - How to run this binary using network communication" << std::endl
        << "    $ " << argv[0] << " --portname tcp:169.254.110.254" << std::endl;

      return EXIT_SUCCESS;
    }
  }

  if (opt_portname.empty()) {
    std::cout << "Error, portname unspecified. Run " << argv[0] << " --help" << std::endl;
    return EXIT_SUCCESS;
  }

  vpRobotFlirPtu robot;

  try {
    vpColVector q(2), q_mes;
    int answer;

    std::cout << "Try to connect FLIR PTU to port: " << opt_portname << " with baudrate: " << opt_baudrate << std::endl;
    robot.connect(opt_portname, opt_baudrate);

    if (opt_network) {
      std::cout << "PTU HostName: " << robot.getNetworkHostName() << std::endl;
      std::cout << "PTU IP      : " << robot.getNetworkIP() << std::endl;
      std::cout << "PTU Gateway : " << robot.getNetworkGateway() << std::endl;
      return EXIT_SUCCESS;
    }

    if (opt_reset) {
      std::cout << "Reset PTU axis" << std::endl;
      robot.reset();
      return EXIT_SUCCESS;
    }

    {
      std::cout << "** Test limits getter" << std::endl;

      std::cout << "Pan  pos min/max [deg]: " << vpMath::deg(robot.getPanPosLimits()[0]) << " "
        << vpMath::deg(robot.getPanPosLimits()[1]) << std::endl;
      std::cout << "Tilt pos min/max [deg]: " << vpMath::deg(robot.getTiltPosLimits()[0]) << " "
        << vpMath::deg(robot.getTiltPosLimits()[1]) << std::endl;
      std::cout << "Pan/tilt vel max [deg/s]: " << vpMath::deg(robot.getPanTiltVelMax()[0]) << " "
        << vpMath::deg(robot.getPanTiltVelMax()[1]) << std::endl
        << std::endl;
    }

    {
      std::cout << "** Test limits setter" << std::endl;
      // Reduce pan/tilt position limits wrt factory settings
      vpColVector pan_pos_limits(2), tilt_pos_limits(2);
      pan_pos_limits[0] = vpMath::rad(-90);
      pan_pos_limits[1] = vpMath::rad(90);
      tilt_pos_limits[0] = vpMath::rad(-20);
      tilt_pos_limits[1] = vpMath::rad(20);

      robot.setPanPosLimits(pan_pos_limits);
      robot.setTiltPosLimits(tilt_pos_limits);

      std::cout << "Modified user min/max limits: " << std::endl;
      std::cout << "Pan  pos min/max [deg]: " << vpMath::deg(robot.getPanPosLimits()[0]) << " "
        << vpMath::deg(robot.getPanPosLimits()[1]) << std::endl;
      std::cout << "Tilt pos min/max [deg]: " << vpMath::deg(robot.getTiltPosLimits()[0]) << " "
        << vpMath::deg(robot.getTiltPosLimits()[1]) << std::endl;
      std::cout << "Pan/tilt vel max [deg/s]: " << vpMath::deg(robot.getPanTiltVelMax()[0]) << " "
        << vpMath::deg(robot.getPanTiltVelMax()[1]) << std::endl
        << std::endl;
    }

    {
      std::cout << "** Test position getter" << std::endl;
      robot.getPosition(vpRobot::ARTICULAR_FRAME, q_mes);
      std::cout << "Current position [deg]: " << vpMath::deg(q_mes[0]) << " " << vpMath::deg(q_mes[1]) << std::endl;

      std::cout << "Initialisation done." << std::endl << std::endl;
    }

    {
      std::cout << "** Test joint positioning" << std::endl;
      robot.setRobotState(vpRobot::STATE_POSITION_CONTROL);
      robot.setMaxRotationVelocity(std::min<double>(robot.getPanTiltVelMax()[0], robot.getPanTiltVelMax()[1]) /
                                   2.); // 50% of the slowest axis

      q = 0;
      std::cout << "Set joint position [deg]: " << vpMath::deg(q[0]) << " " << vpMath::deg(q[1]) << std::endl;
      std::cout << "Enter a caracter to apply" << std::endl;
      scanf("%d", &answer);

      robot.setPositioningVelocity(50);
      robot.setPosition(vpRobot::JOINT_STATE, q);
      robot.getPosition(vpRobot::JOINT_STATE, q_mes);

      std::cout << "Position reached [deg]: " << vpMath::deg(q_mes[0]) << " " << vpMath::deg(q_mes[1]) << std::endl
        << std::endl;
    }

    {
      std::cout << "** Test joint positioning" << std::endl;
      q[0] = vpMath::rad(10); // Pan  position in rad
      q[1] = vpMath::rad(20); // Tilt position in rad

      std::cout << "Set joint position: " << vpMath::deg(q[0]) << " " << vpMath::deg(q[1]) << "[deg]" << std::endl;
      std::cout << "Enter a caracter to apply" << std::endl;
      scanf("%d", &answer);

      robot.setPosition(vpRobot::ARTICULAR_FRAME, q);
      robot.getPosition(vpRobot::ARTICULAR_FRAME, q_mes);

      std::cout << "Position reached [deg]: " << vpMath::deg(q_mes[0]) << " " << vpMath::deg(q_mes[1]) << std::endl
        << std::endl;
    }

    {
      std::cout << "** Test joint velocity" << std::endl;

      vpColVector qdot(2);
      qdot[0] = vpMath::rad(-10); // Pan  velocity in rad/s
      qdot[1] = vpMath::rad(0);   // Tilt velocity in rad/s

      std::cout << "Set velocity for 4s: " << vpMath::deg(qdot[0]) << " " << vpMath::deg(qdot[1]) << " [deg/s]"
        << std::endl;
      std::cout << "Enter a caracter to apply" << std::endl;
      scanf("%d", &answer);

      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

      double t_start = vpTime::measureTimeMs();
      do {
        robot.setVelocity(vpRobot::JOINT_STATE, qdot);
        vpTime::sleepMs(40);
      } while (vpTime::measureTimeMs() - t_start < 4000);

      robot.setRobotState(vpRobot::STATE_STOP);
      robot.getPosition(vpRobot::ARTICULAR_FRAME, q_mes);
      std::cout << "Position reached: " << vpMath::deg(q_mes[0]) << " " << vpMath::deg(q_mes[1]) << " [deg]"
        << std::endl
        << std::endl;
    }

    {
      std::cout << "** Test cartesian velocity with robot Jacobien eJe" << std::endl;

      vpColVector v_e(6, 0);
      v_e[4] = vpMath::rad(5); // wy_e
      v_e[5] = vpMath::rad(5); // wz_e

      std::cout << "Set cartesian velocity in end-effector frame for 4s: " << v_e[0] << " " << v_e[1] << " " << v_e[2]
        << " [m/s] " << vpMath::deg(v_e[3]) << " " << vpMath::deg(v_e[4]) << " " << vpMath::deg(v_e[5])
        << " [deg/s]" << std::endl;
      std::cout << "Enter a caracter to apply" << std::endl;
      scanf("%d", &answer);

      robot.setRobotState(vpRobot::STATE_VELOCITY_CONTROL);

      double t_start = vpTime::measureTimeMs();
      do {
        vpColVector qdot = robot.get_eJe().pseudoInverse() * v_e;
        robot.setVelocity(vpRobot::JOINT_STATE, qdot);
        vpTime::sleepMs(40);
      } while (vpTime::measureTimeMs() - t_start < 4000);

      robot.setRobotState(vpRobot::STATE_STOP);
      robot.getPosition(vpRobot::ARTICULAR_FRAME, q_mes);
      std::cout << "Position reached: " << vpMath::deg(q_mes[0]) << " " << vpMath::deg(q_mes[1]) << " [deg]"
        << std::endl
        << std::endl;
    }

    std::cout << "** The end" << std::endl;
  }
  catch (const vpRobotException &e) {
    std::cout << "Catch Flir Ptu signal exception: " << e.getMessage() << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
  }
  catch (const vpException &e) {
    std::cout << "Catch Flir Ptu exception: " << e.getMessage() << std::endl;
    robot.setRobotState(vpRobot::STATE_STOP);
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
