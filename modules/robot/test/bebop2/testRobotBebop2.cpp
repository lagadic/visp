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

#include <visp3/core/vpTime.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/robot/vpRobotBebop2.h>

int main()
{
#ifdef VISP_HAVE_ARSDK
  vpRobotBebop2 drone(false);

  if (drone.isRunning()) {

    drone.takeOff();
    vpTime::wait(1000);
    drone.setRoll(20);
    vpTime::wait(2000);
    drone.stopMoving();
    vpTime::wait(1000);
    drone.land();

#if 0
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
#endif

  } else {
    std::cout << "Error : failed to setup drone control" << std::endl;
  }

  std::cout << "-- End of test --" << std::endl;
#else
  std::cout << "Install Parrot ARSDK, configure and build ViSP to use this example..." << std::endl;
#endif
}
