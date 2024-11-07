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
 * Simple example to demonstrate how to control in velocity using mavsdk
 * a drone equipped with a Pixhawk connected to a Jetson TX2.
 */

/*!
 * \example testPixhawkRoverVelocityControl.cpp
 *
 * This code shows how to control a rover equipped with a Pixhawk connected to a computer that is
 * runing this test.
 * Communication between the computer and the Pixhawk is based on Mavlink
 * using MAVSDK 3rd party.
 */

#include <iostream>

#include <visp3/core/vpConfig.h>

// Check if std:c++17 or higher
#if defined(VISP_HAVE_MAVSDK) && ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L))) \
  && defined(VISP_HAVE_THREADS)

#include <thread>
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
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  if (argc != 2) {
    usage(argv[0]);
    return EXIT_SUCCESS;
  }

  auto robot = vpRobotMavsdk(argv[1]);

  if (!robot.setGPSGlobalOrigin(48.117266, -1.6777926, 40.0)) {
    return EXIT_FAILURE;
  }

  std::cout << "Vehicle has flying capability: " << (robot.hasFlyingCapability() ? "yes" : "no") << std::endl;
  robot.arm();

  double delta_north = 1.;
  double delta_east = 0.;
  double delta_down = 0.;
  double delta_yaw = 0.;

  std::cout << "Move 1 meter north" << std::endl;;
  robot.setPositionRelative(delta_north, delta_east, delta_down, delta_yaw);

  vpColVector frd_vel { 0.0, 0.0, 0.0, 0.0 };
  frd_vel[0] = -0.3;             // forward vel m/s
  //frd_vel[3]= vpMath::rad(10.);

  std::cout << "Go at 0.3m/s backward during 3 sec.\n";
  robot.setVelocity(frd_vel);
  vpTime::wait(3000);

  std::cout << "Go at 0.3m/s forward and rotate 10 deg/s along yaw during 2 sec.\n";
  frd_vel[0] = 0.3;              // forward vel m/s
  frd_vel[3] = vpMath::rad(10.); // yaw vel 10 deg/s converted in rad/s

  double t = vpTime::measureTimeMs();
  do {
    vpTime::sleepMs(20);
    robot.setVelocity(frd_vel);
  } while (vpTime::measureTimeMs() - t < 2000.); //

  robot.disarm();
  return EXIT_SUCCESS;
}

#else

int main()
{
#ifndef VISP_HAVE_MAVSDK
  std::cout << "\nThis example requires mavsdk library. You should install it, configure and rebuid ViSP.\n"
    << std::endl;
#endif
#if !((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))
  std::cout
    << "\nThis example requires at least cxx17. You should enable cxx17 during ViSP configuration with cmake and "
    "rebuild ViSP.\n"
    << std::endl;
#endif
  return EXIT_SUCCESS;
}

#endif // #if defined(VISP_HAVE_MAVSDK)
