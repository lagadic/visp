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
 * Interface for the Reflex Takktile 2 hand from Right Hand Robotics.
 *
*****************************************************************************/

/*!
  \example takktile2-read-data.cpp

  Example that reads data from Reflex Takktile 2 hand from Right Hand Robotics.

*/

#include <iostream>
#include <string>

#include <visp3/core/vpConfig.h>
#include <visp3/robot/vpReflexTakktile2.h>

int main(int argc, char *argv[])
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif

  std::string opt_network_interface = "eth0";
  std::string opt_finger_file_name = "yaml/finger_calibrate.yaml";
  std::string opt_tactile_file_name = "yaml/tactile_calibrate.yaml";
  std::string opt_motor_file_name = "yaml/motor_constants.yaml";

  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--network")
      opt_network_interface = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--finger")
      opt_finger_file_name = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--tactile")
      opt_tactile_file_name = atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--motor")
      opt_motor_file_name = atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0]
        << " [--network <interface name>] "
        " [--finger <calib file name>]"
        " [--tactile <calib file name>]"
        " [--motor <constants file name>]"
        " [--help] [-h]\n"
        << std::endl;
      std::cout << "Options:" << std::endl;
      std::cout << "  --network <interface name>" << std::endl;
      std::cout << "\tNetwork interface name. Default: " << opt_network_interface << std::endl << std::endl;
      std::cout << "  --finger <calib file name>" << std::endl;
      std::cout << "\tFinger calibration file name. Default: " << opt_finger_file_name << std::endl << std::endl;
      std::cout << "  --tactile <calib file name>" << std::endl;
      std::cout << "\tTactile calibration file name. Default: " << opt_tactile_file_name << std::endl << std::endl;
      std::cout << "  --motor <constants file name>" << std::endl;
      std::cout << "\tMotor constants file name. Default: " << opt_motor_file_name << std::endl << std::endl;
      std::cout << "  --help, -h" << std::endl;
      std::cout << "\tPrint this helper." << std::endl;

      return EXIT_SUCCESS;
    }
  }
#ifdef VISP_HAVE_TAKKTILE2
  vpReflexTakktile2 reflex;
  reflex.setNetworkInterface(opt_network_interface);
  reflex.setFingerConfigFile(opt_finger_file_name);
  reflex.setTactileConfigFile(opt_tactile_file_name);
  reflex.setMotorConfigFile(opt_motor_file_name);

  reflex.open();

  // Hit CTRL-C to stop
  while (true) {

    std::cout << reflex.getHandInfo() << std::endl;
    reflex.wait(50);
  }

#else
  std::cout << "ViSP is not built to support Right Hand Reflex Takktile2 hand" << std::endl;
#endif

  return EXIT_SUCCESS;
}
