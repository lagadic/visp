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
 * Test for UDP server.
 *
 *****************************************************************************/

/*!
  \example testUDPServer.cpp

  Example of a UDP server.
*/

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <iterator>
#include <sstream>
#include <vector>
#include <visp3/core/vpUDPServer.h>

namespace
{
struct DataType {
  double double_val;
  int int_val;

  DataType() : double_val(0.0), int_val(0) {}
  DataType(const double dbl, const int i) : double_val(dbl), int_val(i) {}
};
}

int main()
{
// inet_ntop() used in vpUDPClient is not supported on win XP
#ifdef VISP_HAVE_FUNC_INET_NTOP
  try {
    int port = 50037;
    vpUDPServer server(port);

    std::string msg = "", hostInfo = "";
    // Receive and send custom data type
    int res = server.receive(msg, hostInfo);
    if (res) {
      DataType data_type;
      memcpy(&data_type.double_val, msg.c_str(), sizeof(data_type.double_val));
      memcpy(&data_type.int_val, msg.c_str() + sizeof(data_type.double_val), sizeof(data_type.int_val));
      std::cout << "Server received double_val: " << data_type.double_val << " ; int_val: " << data_type.int_val
                << " from: " << hostInfo << std::endl;

      // Get address and port
      std::istringstream iss(hostInfo);
      std::vector<std::string> tokens;
      std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                std::back_inserter(tokens));
      data_type.double_val += 1.5;
      data_type.int_val += 2;
      char data[sizeof(data_type.double_val) + sizeof(data_type.int_val)];
      memcpy(data, &data_type.double_val, sizeof(data_type.double_val));
      memcpy(data + sizeof(data_type.double_val), &data_type.int_val, sizeof(data_type.int_val));
      msg = std::string(data, sizeof(data_type.double_val) + sizeof(data_type.int_val));

      server.send(msg, tokens[1], atoi(tokens[2].c_str()));
    }

    // Receive and send message
    while (true) {
      int res = server.receive(msg, hostInfo, 5000);
      if (res) {
        std::cout << "Server received: " << msg << " from: " << hostInfo << std::endl;
        std::cout << "Reply to the client: Echo: " << msg << std::endl;

        // Get address and port
        std::istringstream iss(hostInfo);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(),
                  std::back_inserter(tokens));
        server.send("Echo: " + msg, tokens[1], atoi(tokens[2].c_str()));
      } else if (res == 0) {
        std::cout << "Receive timeout" << std::endl;
      } else {
        std::cerr << "Error server.receive()!" << std::endl;
      }
    }

    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
#else
  std::cout << "This test doesn't work on win XP where inet_ntop() is not available" << std::endl;
  return EXIT_SUCCESS;
#endif
}
