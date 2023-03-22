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
 * Simple example to demonstrate how takeoff and land using mavsdk on
 * a drone equipped with a Pixhawk connected to a Jetson TX2.
 *
 *****************************************************************************/

/*!
 * \example testPixhawkDroneTakeoff.cpp
 *
 * This code shows how to takeoff and land a drone equipped with a Pixhawk
 * connected to a Jetson TX2 that runs this test using ViSP. The drone is localized
 * thanks to Qualisys Mocap. Communication between the Jetson and the Pixhawk
 * is based on Mavlink using MAVSDK 3rd party.
 */

#include <iostream>

#include <visp3/core/vpConfig.h>

#if defined(VISP_HAVE_MAVSDK) && (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_17)

#include <visp3/robot/vpRobotMavsdk.h>

void usage(const std::string &bin_name)
{
  std::cerr << "Usage : " << bin_name << " <connection information>\n"
            << "Connection information format should be :\n"
            << "  - For TCP: tcp://[server_host][:server_port]\n"
            << "  - For UDP: udp://[bind_host][:bind_port]\n"
            << "  - For Serial: serial:///path/to/serial/dev[:baudrate]\n"
            << "For example, to connect to the simulator use URL: udp://:14540\n";
}

int main(int argc, char **argv)
{
  if (argc != 2) {
    usage(argv[0]);
    return EXIT_SUCCESS;
  }

  auto drone = vpRobotMavsdk(argv[1]);

  drone.setTakeOffAlt(1.);
  drone.setVerbose(true);
  drone.takeControl(); // Start off-board or guided mode

  // Drone takeoff
  if (! drone.takeOff() )
  {
    std::cout << "Takeoff failed" << std::endl;
    return EXIT_FAILURE;
  }

  // Get position in NED local frame after takeoff
  float ned_north, ned_east, ned_down, ned_yaw;
  drone.getPosition(ned_north, ned_east, ned_down, ned_yaw);
  std::cout << "Vehicle position in NED frame: " << ned_north << " " << ned_east << " " << ned_down << " [m] and " << vpMath::deg(ned_yaw) << " [deg]" << std::endl;

  vpHomogeneousMatrix ned_M_frd;
  drone.getPosition(ned_M_frd);
  vpRxyzVector rxyz(ned_M_frd.getRotationMatrix());
  std::cout << "Vehicle position in NED frame: " 
            << ned_M_frd.getTranslationVector().t() << " [m] and " 
            << vpMath::deg(rxyz).t() << " [deg]"<< std::endl;

  std::cout << "Hold position for 4 sec" << std::endl;
  drone.holdPosition();
  vpTime::wait(4000);

  // Land drone
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
