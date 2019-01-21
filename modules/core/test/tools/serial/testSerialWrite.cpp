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
 * Test serial port communication.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

/*!
  \example testSerialWrite.cpp
  \brief Test that writes data on serial port.
*/

#include <iostream>
#include <stdlib.h>
#include <visp3/core/vpSerial.h>

int main(int argc, char **argv)
{
#if !defined(_WIN32)
  std::string port;

  unsigned long baud = 9600;
  for (int i = 0; i < argc; i++) {
    if (std::string(argv[i]) == "--port")
      port = std::string(argv[i + 1]);
    else if (std::string(argv[i]) == "--baud") {
      baud = (unsigned long)atol(argv[i + 1]);
    }
    else if (std::string(argv[i]) == "--help") {
      std::cout << "\nUsage: " << argv[0] << " [--port <serial name>] [--baud <baud rate>] [--help]\n" << std::endl;
      return 0;
    }
  }

  if (port.empty()) {
    std::cout << "\nSerial port not specified." << std::endl;
    std::cout << "\nUsage: " << argv[0] << " [--port <serial name>] [--baud <baud rate>] [--help]\n" << std::endl;
    return 0;
  }

  std::cout << "Try to connect to port \"" << port << "\" with baud rate " << baud << std::endl;
  vpSerial serial(port, baud);

  serial.write("hello\n");

#else
  (void) argc;
  (void) argv;
  std::cout << "Serial test is only working on unix-like OS." << std::endl;
#endif
  return EXIT_SUCCESS;
}
