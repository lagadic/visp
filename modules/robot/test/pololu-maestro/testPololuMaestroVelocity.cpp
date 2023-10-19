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

#ifdef VISP_HAVE_RAPA_POLOLU_MAESTRO

#include "RPMSerialInterface.h"
#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <visp3/robot/vpServoPololuMaestro.h>

int main()
{
  std::chrono::seconds sec(1);

  // Create the interface.
  std::string error_msg;
  const std::string &dev = "/dev/ttyACM0";
  RPM::SerialInterface *serialInterface = RPM::SerialInterface::createSerialInterface(dev, 9600, &error_msg);

  std::cout << error_msg;
  std::cout << "Serial is: " << dev << " Started!\n";
  std::cout << serialInterface->isOpen() << "\n";

  // Checking that the serial connection is open.
  if (!serialInterface->isOpen()) {
    std::cout << "Serial Communication Failed!\n";
    return EXIT_FAILURE;
  }

  // Creating the servo object.
  vpServoPololuMaestro servo1(serialInterface, 0);

  // Servo objet will first move in one direction at a velocity of 10 for 3 sec and move back in the other direction for 3 sec.
  servo1.setVelocityCmd(10);
  std::this_thread::sleep_for(3 * sec);
  servo1.setVelocityCmd(-10);
  std::this_thread::sleep_for(3 * sec);

  // Adding a second servo to the test.
  vpServoPololuMaestro servo2(serialInterface, 1);

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
int main() { std::cout << "ViSP doesn't support Rapa Pololu Servo 3rd party library" << std::endl; }
#endif
