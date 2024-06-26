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
 * Test to control from keyboard a drone equipped with a Pixhawk thanks to mavsdk.
 */

/*!
 * \example testPixhawkDroneKeyboard.cpp
 *
 * This code shows how to setup keyboard control of a drone equipped with a Pixhawk
 * connected to a Jetson TX2 that runs this test using ViSP. The drone is localized
 * thanks to Qualisys Mocap. Communication between the Jetson and the Pixhawk
 * is based on Mavlink using MAVSDK 3rd party.
 */

#include <iostream>

#include <visp3/core/vpConfig.h>

// Check if std:c++17 or higher
#if defined(VISP_HAVE_MAVSDK) && ((__cplusplus >= 201703L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201703L)))

#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpKeyboard.h>
#include <visp3/robot/vpRobotMavsdk.h>

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif
bool handleKeyboardInput(vpRobotMavsdk &drone, int key, bool &flying, double &lastCommandTime)
{
  bool running = true;
  double currentTime = vpTime::measureTimeMs();
  if (drone.isRunning()) {
    switch (key) {
    case 'q':
      // Quit
      std::cout << "sending command" << std::endl;
      drone.land();
      flying = false;
      running = false;
      lastCommandTime = vpTime::measureTimeMs();
      break;

    case 'a':
      // Land
      if (flying == true) {
        std::cout << "sending command" << std::endl;
        drone.land();
        flying = false;
        lastCommandTime = vpTime::measureTimeMs();
      }
      break;

    case 'e':
      // Emergency
      std::cout << "sending command" << std::endl;
      drone.kill();
      flying = false;
      running = false;
      lastCommandTime = vpTime::measureTimeMs();
      break;

    case 't':
      // Takeoff
      std::cout << "sending command" << std::endl;
      drone.takeOff();
      flying = true;
      lastCommandTime = vpTime::measureTimeMs();
      vpTime::wait(100);
      drone.takeControl();
      break;

    case ' ':
      // Down
      if (flying == true) {
        drone.setVerticalSpeed(0.2);
        lastCommandTime = vpTime::measureTimeMs();
      }
      break;

    case 'u':
      // Up
      if (flying == true) {
        drone.setVerticalSpeed(-0.2);
        lastCommandTime = vpTime::measureTimeMs();
      }
      break;

    case 'd':
      // turn Right
      if (flying == true) {
        drone.setYawSpeed(0.4);
        lastCommandTime = vpTime::measureTimeMs();
      }
      break;

    case 'g':
      // turn Left
      if (flying == true) {
        drone.setYawSpeed(-0.4);
        lastCommandTime = vpTime::measureTimeMs();
      }
      break;

    case 'i':
      // go Forward
      if (flying == true) {
        drone.setForwardSpeed(0.2);
        lastCommandTime = vpTime::measureTimeMs();
      }
      break;

    case 'k':
      // go Backwards
      if (flying == true) {
        drone.setForwardSpeed(-0.2);
        lastCommandTime = vpTime::measureTimeMs();
      }
      break;

    case 'j':
      // go Left
      if (flying == true) {
        drone.setLateralSpeed(-0.2);
        lastCommandTime = vpTime::measureTimeMs();
      }
      break;

    case 'l':
      // go Right
      if (flying == true) {
        drone.setLateralSpeed(0.2);
        lastCommandTime = vpTime::measureTimeMs();
      }
      break;

    default:
      // No inputs -> drone stops moving
      if ((flying == true) && (currentTime - lastCommandTime > 1500.)) { // We stop moving after 1.5s without commands.
        std::cout << "1.5 s without order, sending command : stop moving." << std::endl;
        drone.stopMoving();
        lastCommandTime = vpTime::measureTimeMs();
      }
      break;
    }
    vpTime::wait(40); // We wait 40ms to give the drone the time to process the command
  }
  else {
    running = false;
  }
  return running;
}

int main(int argc, char **argv)
{
  try {
    std::string opt_connecting_info = "udp://192.168.30.111:14552";

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--co" && i + 1 < argc) {
        opt_connecting_info = std::string(argv[i + 1]);
        i++;
      }
      else if (argc >= 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
        std::cout << "\nUsage:\n"
          << "  " << argv[0] << "[--co <connection information>] [--help] [-h]\n"
          << std::endl
          << "Description:\n"
          << "  --co <connection information>\n"
          << "      - UDP: udp://[host][:port]\n"
          << "      - TCP: tcp://[host][:port]\n"
          << "      - serial: serial://[path][:baudrate]\n"
          << "      - Default: udp://192.168.30.111:14552).\n\n"
          << "      For example, to connect to the simulator use URL: udp://:14552\n"
          << "  --help, -h\n"
          << "      Print help message.\n"
          << std::endl;
        return EXIT_SUCCESS;
      }
      else {
        std::cout << "Error : unknown parameter " << argv[i] << std::endl
          << "See " << argv[0] << " --help" << std::endl;
        return EXIT_SUCCESS;
      }
    }

    std::cout << std::endl
      << "WARNING: this program does no sensing or avoiding of obstacles, "
      << "the drone WILL collide with any objects in the way! Make sure the "
      << "drone has approximately 3 meters of free space on all sides." << std::endl
      << std::endl;

// Connect to the drone
    vpRobotMavsdk drone(opt_connecting_info);

    if (drone.isRunning()) {
      int k = 0;
      bool running = true;
      bool flying = false;
      double lastCommandTime = vpTime::measureTimeMs();

      std::cout << "\nConfiguring drone settings ...\n" << std::endl;

      drone.setTakeOffAlt(1.0);

      vpKeyboard keyboard;
      std::cout << "\n| Control the drone with the keyboard :\n"
        "|   't' to takeoff / 'l' to land / 'e' for emergency stop\n"
        "|   ('space','u','d','g') and ('i','k','j','l') to move\n"
        "|   'q' to quit.\n"
        << std::endl;

      while (running && drone.isRunning()) {

        k = '0'; // If no key is hit, we send a non-assigned key
        if (keyboard.kbhit()) {
          k = keyboard.getchar();
        }
        running = handleKeyboardInput(drone, k, flying, lastCommandTime);
      }
      std::cout << "\nQuitting ...\n" << std::endl;

    }
    else {
      std::cout << "ERROR : failed to setup drone control." << std::endl;
      return EXIT_FAILURE;
    }
  }
  catch (const vpException &e) {
    std::cout << "\nCaught an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
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
