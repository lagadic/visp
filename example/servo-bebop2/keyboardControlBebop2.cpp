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
 * Example that shows how to setup keyboard control of Parrot Bebop 2 drone in ViSP.
 *
 * Authors:
 * Gatien Gaumerais
 *
*****************************************************************************/

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/io/vpKeyboard.h>

#include <visp3/robot/vpRobotBebop2.h>

#include <iostream>

#ifdef VISP_HAVE_ARSDK

#ifdef ENABLE_VISP_NAMESPACE
using namespace VISP_NAMESPACE_NAME;
#endif

/*!

  Sends movement commands to the \e drone based on a keyboard input \e key.
  Returns a boolean indicating if the program should be stopping or not.

  \param[in] drone : the drone to control.
  \param[in] key : key input to handle.
*/
bool handleKeyboardInput(vpRobotBebop2 &drone, int key)
{
  bool running = true;
  if (drone.isRunning()) {
    switch (key) {
    case 'q':
      // Quit
      drone.land();
      running = false;
      break;

    case 'e':
      // Emergency
      drone.cutMotors();
      running = false;
      break;

    case 't':
      // Takeoff
      drone.takeOff(false);
      break;

    case ' ':
      // Landing
      drone.land();
      break;

    case 'i':
      // Up
      drone.setVerticalSpeed(50);
      break;

    case 'k':
      // Down
      drone.setVerticalSpeed(-50);
      break;

    case 'l':
      // Right
      drone.setYawSpeed(50);
      break;

    case 'j':
      // Left
      drone.setYawSpeed(-50);
      break;

    case 'r':
      // Forward
      drone.setPitch(50);
      break;

    case 'f':
      // Backward
      drone.setPitch(-50);
      break;

    case 'd':
      // Roll left
      drone.setRoll(-50);
      break;

    case 'g':
      // Roll right
      drone.setRoll(50);
      break;

    default:
      // No inputs -> drone stops moving
      drone.stopMoving();
      break;
    }
    vpTime::wait(25); // We wait 25ms to give the drone the time to process the command
  }
  else {
    running = false;
  }
  return running;
}

/*!

  \example keyboardControlBebop2.cpp example showing how to setup keyboard control of Parrot Bebop 2 drone.

  WARNING: this program does no sensing or avoiding of obstacles, the drone
  WILL collide with any objects in the way! Make sure the drone has about
  3-4 meters of free space around it before starting the program.

  This program makes the drone controllable with a keyboard, with display of the video streaming.
*/
int main(int argc, char **argv)
{
  try {

    std::string ip_address = "192.168.42.1";

    int stream_res = 0; // Default 480p resolution

    bool verbose = false;

    for (int i = 1; i < argc; i++) {
      if (std::string(argv[i]) == "--ip" && i + 1 < argc) {
        ip_address = std::string(argv[i + 1]);
        i++;
      }
      else if (std::string(argv[i]) == "--hd_stream") {
        stream_res = 1;
      }
      else if (std::string(argv[i]) == "--verbose" || std::string(argv[i]) == "-v") {
        verbose = true;
      }
      else if (argc >= 2 && (std::string(argv[1]) == "--help" || std::string(argv[1]) == "-h")) {
        std::cout << "\nUsage:\n"
          << "  " << argv[0] << "[--ip <drone ip>] [--hd_stream] [--verbose] [-v] [--help] [-h]\n"
          << std::endl
          << "Description:\n"
          << "  --ip <drone ip>\n"
          << "      Ip address of the drone to which you want to connect (default : 192.168.42.1).\n\n"
          << "  --hd_stream\n"
          << "      Enables HD 720p streaming instead of default 480p.\n"
          << "  --verbose, -v\n"
          << "      Enables verbose (drone information messages and velocity commands\n"
          << "      are then displayed).\n\n"
          << "  --help, -h\n"
          << "      Print help message.\n"
          << std::endl;
        return EXIT_SUCCESS;
      }
      else {
        std::cout << "Error : unknown parameter " << argv[i] << std::endl
          << "See " << argv[0] << " --help" << std::endl;
        return EXIT_FAILURE;
      }
    }

    std::cout << "\nWARNING: this program does no sensing or avoiding of obstacles, "
      "the drone WILL collide with any objects in the way! Make sure the "
      "drone has approximately 3 meters of free space on all sides.\n"
      << std::endl;

    vpRobotBebop2 drone(verbose, true,
                        ip_address); // Create the drone with low verbose level, settings reset and corresponding IP

    if (drone.isRunning()) {

      int k = 0;
      bool running = true;

      std::cout << "\nConfiguring drone settings ...\n" << std::endl;

      drone.setMaxTilt(10); // Setting the max roll and pitch values, the drone speed will depend on it

      drone.doFlatTrim(); // Flat trim calibration

#ifdef VISP_HAVE_FFMPEG
      drone.setVideoResolution(stream_res); // Setting desired stream video resolution
      drone.setStreamingMode(0);            // Set streaming mode 0 : lowest latency
      std::cout << "\nWaiting for streaming to start ...\n" << std::endl;
      drone.startStreaming();

      // Prepare image for display
      vpImage<vpRGBa> I(1, 1, 0);
      drone.getRGBaImage(I);
      vpDisplayX display(I, 100, 100, "DRONE VIEW");
      vpDisplay::display(I);
      vpDisplay::flush(I);
#endif

      vpKeyboard keyboard;
      std::cout << "\n| Control the drone with the keyboard :\n"
        "|   't' to takeoff / spacebar to land / 'e' for emergency stop\n"
        "|   ('r','f','d','g') and ('i','k','j','l') to move\n"
        "|   'q' to quit.\n"
        << std::endl;

      while (running && drone.isRunning() && drone.isStreaming()) {

        k = '0'; // If no key is hit, we send a non-assigned key
        if (keyboard.kbhit()) {
          k = keyboard.getchar();
        }
        running = handleKeyboardInput(drone, k);

#ifdef VISP_HAVE_FFMPEG
        drone.getRGBaImage(I);
        vpDisplay::display(I);
        vpDisplay::displayText(I, 10, 10, "Press q to quit", vpColor::red);
        vpDisplay::flush(I);
#endif
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
  std::cout << "\nThis example requires Parrot ARSDK3 library. You should install it.\n" << std::endl;
  return EXIT_SUCCESS;
}

#endif // #if defined(VISP_HAVE_ARSDK)
