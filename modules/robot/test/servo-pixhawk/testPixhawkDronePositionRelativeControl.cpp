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
 * Simple example to demonstrate how to control in position using mavsdk
 * a drone equipped with a Pixhawk connected to a Jetson TX2.
 */

/*!
 * \example testPixhawkDronePositionRelativeControl.cpp
 *
 * This code shows how to takeoff, fly to the corners of a 1. m size square and land
 * a drone equipped with a Pixhawk connected to a Jetson TX2 that runs this test using ViSP.
 * The drone is localized thanks to Qualisys Mocap. Communication between the Jetson and the Pixhawk
 * is based on Mavlink using MAVSDK 3rd party.
 */

#include <iostream>

#include <visp3/core/vpConfig.h>

// Check if std:c++17 or higher
#if defined(VISP_HAVE_MAVSDK) && ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

#include <visp3/robot/vpRobotMavsdk.h>

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

  auto drone = vpRobotMavsdk(argv[1]);
  drone.setAutoLand(true);
  drone.setTakeOffAlt(1.0);
  drone.setVerbose(true);

  if (!drone.takeOff()) {
    std::cout << "Takeoff failed" << std::endl;
    return EXIT_FAILURE;
  }

  drone.takeControl(); // Start PX4 offboard

  // Get position
  float ned_north, ned_east, ned_down, ned_yaw;
  drone.getPosition(ned_north, ned_east, ned_down, ned_yaw);
  std::cout << "Vehicle position in NED frame: " << ned_north << " " << ned_east << " " << ned_down << " [m] and "
    << vpMath::deg(ned_yaw) << " [deg]" << std::endl;

  vpHomogeneousMatrix ned_M_frd;
  drone.getPosition(ned_M_frd);
  vpRxyzVector rxyz(ned_M_frd.getRotationMatrix());
  std::cout << "Vehicle position in NED frame: " << ned_M_frd.getTranslationVector().t() << " [m] and "
    << vpMath::deg(rxyz).t() << " [deg]" << std::endl;

// Set position in NED frame
  drone.setPositioningIncertitude(0.10, vpMath::rad(5.));

  drone.setPositionRelative(0.0, 1.0, 0.0, 0.0);  // Right
  drone.setPositionRelative(1.0, 0.0, 0.0, 0.0);  // Front
  drone.setPositionRelative(0.0, -1.0, 0.0, 0.0); // Left
  drone.setPositionRelative(-1.0, 0.0, 0.0, 0.0); // Rear

  // Land drone during destruction
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
