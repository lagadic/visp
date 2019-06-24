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
 * Interface for the Irisa's Afma6 robot.
 *
 * Authors:
 * Gatien Gaumerais
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testRobotBebop2.cpp

  Example to control Parrot Bebop2.
*/

#include <iostream>

#include <visp3/robot/vpRobotBebop2.h>
#include <visp3/core/vpTime.h>

#ifdef VISP_HAVE_ARSDK

void startDroneKeyboardControl(vpRobotBebop2 &drone)
{
  int k = 0;
  WINDOW *win = initscr();
  raw();
  keypad(stdscr, TRUE);
  noecho();
  timeout(100);

  int y, x;
  getyx(win, y, x);
  move(y + 1, 0); // Move to next line
  clrtoeol();     // Clear line
  mvprintw(y + 1, 0,
           "Control the drone with the keyboard : 't' to takeoff ; Spacebar to land ; 'e' for emergency ; Arrow keys "
           "and ('r','f','d','g') to "
           "move ; 'q' to quit ;");
  getyx(win, y, x);
  move(y + 1, 0);

  while (drone.isRunning()) {
    k = getch();
    drone.handleKeyboardInput(k);
  }
  endwin();
}

int main()
{
  vpRobotBebop2 drone;

  drone.startStreaming();

  vpTime::wait(2000);

  // startDroneKeyboardControl(drone);

  // vpTime::wait(2000);
  //  drone.setMaxTilt(10);
  //  vpTime::wait(2000);

  drone.takeOff();
  //  vpTime::wait(50000);

  drone.setPosition(0.0f, 0.3f, 0.0f, static_cast<float>(vpMath::rad(0.0)), true);

  //  vpTime::wait(10000);

  drone.land();

  std::cout << "-- End of test --" << std::endl;
#else
  std::cout << "Install Parrot ARSDK, configure and build ViSP to use this example..." << std::endl;
#endif
}
