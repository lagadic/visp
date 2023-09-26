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
 * Common test for Pololu Maestro PanTiltUnit position control.
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
    return -1;
  }

  // Creating the servo object.
  vpServoPololuMaestro servo1(serialInterface, 0);

  // Getting the ranges of the servo.
  int min;
  int max;
  int range;

  servo1.getRangePWM(min, max, range);
  std::cout << "Ranges are, min: " << min << " max: " << max << " range: " << range <<std::endl;

  // Servo objet will first go to min range wait 3 seconds and go to max range.
  servo1.setPositionPWM(min, 0);
  std::this_thread::sleep_for(3 * sec);
  servo1.setPositionPWM(max, 0);
  std::this_thread::sleep_for(3 * sec);

  // Adding a second servo to the test.
  vpServoPololuMaestro servo2(serialInterface, 1);

  // Both servos will first go to min range wait 3 seconds and go to max range.
  servo1.setPositionPWM(min);
  servo2.setPositionPWM(min, 0);
  std::this_thread::sleep_for(3 * sec);
  servo1.setPositionPWM(max, 0);
  servo2.setPositionPWM(max, 0);
  std::this_thread::sleep_for(3 * sec);
}

#else
int main() { std::cout << "ViSP doesn't support Rapa Pololu Servo 3rd party library" << std::endl; }
#endif
