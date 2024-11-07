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
 * Test for Virtuose SDK wrapper.
 */

/*!
  \example testVirtuose.cpp
    Test for reading the Virtuose's joint values inside the main function
  after checking the emergency button.
*/

#include <visp3/robot/vpVirtuose.h>

int main(int argc, char **argv)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
#if defined(VISP_HAVE_VIRTUOSE)
  std::string opt_ip = "localhost";
  int opt_port = 5000;
  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--ip")
      opt_ip = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--port")
      opt_port = std::atoi(argv[i + 1]);
    else if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") {
      std::cout << "\nUsage: " << argv[0]
        << " [--ip <localhost>] [--port <port>]"
        " [--help] [-h]\n"
        << std::endl
        << "Description: " << std::endl
        << " --ip <localhost>" << std::endl
        << "\tHost IP address. Default value: \"localhost\"." << std::endl
        << std::endl
        << " --port <port>" << std::endl
        << "\tCommunication port. Default value: 5000." << std::endl
        << "\tSuggested values: " << std::endl
        << "\t- 5000 to communicate with the Virtuose." << std::endl
        << "\t- 53210 to communicate with the Virtuose equipped with the Glove." << std::endl
        << std::endl;
      return EXIT_SUCCESS;
    }
  }

  try {
    vpVirtuose virtuose;
    std::cout << "Try to connect to " << opt_ip << " port " << opt_port << std::endl;
    virtuose.setIpAddressAndPort(opt_ip, opt_port);
    virtuose.init();

    bool emergStop = virtuose.getEmergencyStop();
    if (emergStop) {
      std::cout << "The system is operational." << std::endl;
      vpColVector q = virtuose.getArticularPosition();
      std::cout << "The current joint values are : " << q.t() << std::endl;
    }
    else
      std::cout << "The system is not operational. \nPlease plug the "
      "emergency stop to the system (or untrigger it)."
      << std::endl;
  }
  catch (const vpException &e) {
    std::cout << "Catch an exception: " << e.getStringMessage() << std::endl;
    return EXIT_FAILURE;
  }
#else
  (void)argc;
  (void)argv;
  std::cout << "You should install Virtuose SDK to use this binary..." << std::endl;
#endif
  return EXIT_SUCCESS;
}
