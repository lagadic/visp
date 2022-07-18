/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2022 by Inria. All rights reserved.
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
 * Simple example to demonstrate how to control in velocity using mavsdk
 * a drone equipped with a Pixhawk connected to a Jetson TX2.
 *
 *****************************************************************************/

/*!
 * \example testPixhawkDroneVelocityControl.cpp
 *
 * This code shows how to takeoff, control in velocity and land with a drone
 * equipped with a Pixhawk connected to a Jetson TX2 that runs this test using ViSP.
 * The drone is localized thanks to Qualisys Mocap. Communication between the Jetson
 * and the Pixhawk is based on Mavlink using mavsdk 3rd party.
 */

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MAVSDK) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)

#include <visp3/robot/vpRobotMavsdk.h>

using std::chrono::seconds;
using std::this_thread::sleep_for;

void usage(const std::string &bin_name)
{
  std::cerr << "Usage : " << bin_name << " <connection information>\n"
            << "Connection URL format should be :\n"
            << "  - For TCP : tcp://[server_host][:server_port]\n"
            << "  - For UDP : udp://[bind_host][:bind_port]\n"
            << "  - For Serial : serial:///path/to/serial/dev[:baudrate]\n"
            << "For example, to connect to the simulator use URL: udp://:14540\n";
}

int main(int argc, char **argv)
{
  if (argc != 2) {
    usage(argv[0]);
    return EXIT_SUCCESS;
  }

  auto drone = vpRobotMavsdk(argv[1]);

  drone.takeOff();
  vpColVector vel_command{0.0, 0.0, 0.0, 0.0};

  drone.setForwardSpeed(0.3);
  std::cout << "Should stop moving" << std::endl;
  sleep_for(seconds(4));

  /*std::cout << "Go at 0.3m/s forward during 3 sec.\n";
  vel_command[0] = 0.3;
  drone.setVelocity(vel_command, 3.0);

  std::cout << "Go up at 0.1 m/s for 5 sec.\n";
  vel_command[0] = 0.0;
  vel_command[2] = -0.1;
  drone.setVelocity(vel_command, 5.0);

  std::cout << "turn 180° \n";
  vel_command[2] = 0.0;
  vel_command[3] = 3.14 / 3;
  drone.setVelocity(vel_command, 3.0);

  std::cout << "Go at 1.0 m/s forward during 1 sec.\n";
  vel_command[3] = 0.0;
  vel_command[0] = 1.0;
  drone.setVelocity(vel_command, 1.0);

  std::cout << "Standing still for 2 sec.\n";
  vel_command[3] = 0.0;
  vel_command[0] = 0.0;
  drone.setVelocity(vel_command, 2.0);*/

  // drone.cutMotors();

  drone.land();

  return EXIT_SUCCESS;
}

#else

int main()
{
#ifndef VISP_HAVE_MAVSDK
  std::cout << "\nThis example requires mavsdk library. You should install it, configure and rebuid ViSP.\n"
            << std::endl;
#endif
#if !(VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)
  std::cout
      << "\nThis example requires at least cxx17. You should enable cxx17 during ViSP configuration with cmake and "
         "rebuild ViSP.\n"
      << std::endl;
#endif
  return EXIT_SUCCESS;
}

#endif // #if defined(VISP_HAVE_MAVSDK)
