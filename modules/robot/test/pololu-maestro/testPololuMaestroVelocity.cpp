/*
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
 * Common test for Pololu Maestro PanTiltUnit velocity control.
 */

#include <iostream>

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_POLOLU

#include "RPMSerialInterface.h"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <visp3/robot/vpServoPololuMaestro.h>

int main()
{
  std::chrono::seconds sec(1);

  const std::string &device = "/dev/ttyACM0";
  int baudrate = 9600;

  // Creating the servo object on channel 0.
  vpServoPololuMaestro servo1(device, baudrate, 0);

  // Servo objet will first move in one direction at a velocity of 10 for 3 sec and move back in the other direction for 3 sec.
  servo1.setVelocityCmd(10);
  std::this_thread::sleep_for(3 * sec);
  servo1.setVelocityCmd(-10);
  std::this_thread::sleep_for(3 * sec);

  // Adding a second servo to the test on channel 1.
  vpServoPololuMaestro servo2(device, baudrate, 1);

  // Both servo objet will first move in one direction at a velocity of 10 for 3 sec and move back in the other direction for 3 sec.
  servo1.setVelocityCmd(10);
  servo2.setVelocityCmd(10);
  std::this_thread::sleep_for(3 * sec);
  servo1.setVelocityCmd(-10);
  servo2.setVelocityCmd(-10);
  std::this_thread::sleep_for(3 * sec);

  // Stopping the velocity command.
  servo1.stopVelCmd();
  servo2.stopVelCmd();

  return EXIT_SUCCESS;
}

#else
int main()
{
  std::cout << "ViSP doesn't support Rapa Pololu Servo 3rd party library" << std::endl;
}
#endif
