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
 * Example that shows how to setup keyboard control of Parrot Bebop 2 drone in ViSP.
 *
 * Authors:
 * Fabien Spindler
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
    vpTime::wait(25); // We wait 25ms to let the time to the drone to process the command
  } else {
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
int main()
{
  try {

    std::cout << "\nWARNING: this program does no sensing or avoiding of "
                 "obstacles, \n"
                 "the drone WILL collide with any objects in the way! Make sure "
                 "the \n"
                 "drone has approximately 3 meters of free space on all sides.\n"
              << std::endl;

    vpRobotBebop2 drone(false); // Create the drone with low verbose level

    if (drone.isRunning()) {

      vpKeyboard keyboard;

      int k = 0;
      bool running = true;

      drone.setMaxTilt(10); // Setting the max roll and pitch values, the drone speed will depend on it
      drone.doFlatTrim();   // Flat trim calibration

      drone.setStreamingMode(0); // Set streaming mode 0 : lowest latency
      drone.startStreaming();

      // Prepare image for display
      vpImage<vpRGBa> I(1, 1, 0);
      drone.getRGBaImage(I);
      vpDisplayX display(I, 100, 100, "DRONE VIEW");
      vpDisplay::display(I);
      vpDisplay::flush(I);

      while (running && drone.isRunning() && drone.isStreaming()) {

        k = '0'; // If no key is hit, we send a non assigned key
        if (keyboard.kbhit()) {
          k = keyboard.getchar();
        }
        running = handleKeyboardInput(drone, k);

        drone.getRGBaImage(I);
        vpDisplay::display(I);
        vpDisplay::displayText(I, 10, 10, "Press q to quit", vpColor::red);
        vpDisplay::flush(I);
      }

    } else {
      std::cout << "ERROR : failed to setup drone control." << std::endl;
      return EXIT_FAILURE;
    }

  } catch (const vpException &e) {
    std::cout << "Caught an exception: " << e << std::endl;
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
