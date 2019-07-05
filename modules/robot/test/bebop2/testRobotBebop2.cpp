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

#include <curses.h>
#include <iostream>

#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobotBebop2.h>

#ifdef VISP_HAVE_ARSDK

void startDroneKeyboardControl(vpRobotBebop2 &drone)
{
  int k = 0;

  drone.setMaxTilt(10);
  drone.doFlatTrim();
  drone.startStreaming();

  WINDOW *win = initscr();
  raw();
  keypad(stdscr, TRUE);
  noecho();
  timeout(30);

  vpImage<unsigned char> I(1, 1, 0);
  drone.getGrayscaleImage(I);
  vpDisplayX display(I, 100, 100, "DRONE VIEW");
  vpDisplay::display(I);
  vpDisplay::flush(I);

  int y, x;
  getyx(win, y, x);
  move(y + 1, 0); // Move to next line
  clrtoeol();     // Clear line
  mvprintw(y + 1, 0,
           "Control the drone with the keyboard : 't' to takeoff ; Spacebar to land ; Arrow keys "
           "and ('r','f','d','g') to "
           "move ; 'q' to quit ;");
  getyx(win, y, x);
  move(y + 1, 0);

  while (drone.isRunning() && drone.isStreaming()) {

    drone.getGrayscaleImage(I);
    vpDisplay::display(I);
    vpDisplay::displayText(I, 10, 10, "Press q to quit", vpColor::red);
    vpDisplay::flush(I);

    k = getch();
    drone.handleKeyboardInput(k);
  }
  endwin();
}

int main()
{
  vpRobotBebop2 drone(false);

  if (drone.isRunning()) {
    startDroneKeyboardControl(drone);
  } else {
    std::cout << "Error : failed to setup drone control" << std::endl;
  }

  //  drone.startStreaming();
  //  vpTime::wait(20000);

  //  vpImage<unsigned char> I;

  //  drone.getImage(I);
  //  vpDisplayX display(I, 100, 100, "DRONE VIEW");
  //  vpDisplay::display(I);
  //  vpDisplay::flush(I);

  //  double t = vpTime::measureTimeMs();
  //  do {
  //    drone.getGrayscaleImage(I);
  //    vpDisplay::display(I);
  //    vpDisplay::displayText(I, 10, 10, "Click to exit", vpColor::red);
  //    vpDisplay::flush(I);
  //    if (vpDisplay::getClick(I, false)) {
  //      break;
  //    }
  //  } while (vpTime::measureTimeMs() - t < 20 * 1000);

  // vpTime::wait(2000);
  //  drone.setMaxTilt(10);
  //  vpTime::wait(2000);

  //  drone.takeOff();
  //  vpTime::wait(10000);

  //  drone.setPosition(1.0f, 0.5f, 0.0f, static_cast<float>(vpMath::rad(45.0)), true);

  //  vpHomogeneousMatrix M(1., 0.5, 0, 0, 0, vpMath::rad(45));
  //  drone.setPosition(M, true);

#if 0
  if (drone.isRunning()) {
    drone.doFlatTrim();
    drone.takeOff();
    vpColVector vel(4, 0.0);
    vel[0] = 0.1;
    //  vel[5] = vpMath::rad(10);
    double delta_t = 0.040;
    double t = vpTime::measureTimeMs();
    do {
      drone.setVelocity(vel, 1);
      vpTime::wait(delta_t * 1000);
    } while (vpTime::measureTimeMs() - t < 10 * 1000);

    //  vpColVector vel(6, 0.0);
    //  vel[0] = 0.01;
    //  vel[5] = vpMath::rad(90.0);

    //  drone.setVelocity(vel, 1);
    //  vpTime::wait(300000);
    drone.land();
  }
#endif

  std::cout << "-- End of test --" << std::endl;
#else
  std::cout << "Install Parrot ARSDK, configure and build ViSP to use this example..." << std::endl;
#endif
}
