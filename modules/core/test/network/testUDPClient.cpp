/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Test for UDP client.
 *
 *****************************************************************************/

/*!
  \example testUDPClient.cpp

  Example of a UDP client.
*/

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <visp3/core/vpUDPClient.h>

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
    std::string servername = "127.0.0.1";
    unsigned int port = 50037;
    vpUDPClient client(servername, port);

    // Send custom data type
    DataType data_type(1234.56789, 123450);
    char data[sizeof(data_type.double_val) + sizeof(data_type.int_val)];
    memcpy(data, &data_type.double_val, sizeof(data_type.double_val));
    memcpy(data + sizeof(data_type.double_val), &data_type.int_val, sizeof(data_type.int_val));
    std::string msg(data, sizeof(data_type.double_val) + sizeof(data_type.int_val));
    if (client.send(msg) != (int)sizeof(data_type.double_val) + sizeof(data_type.int_val))
      std::cerr << "Error client.send()!" << std::endl;

    if (client.receive(msg)) {
      data_type.double_val = *reinterpret_cast<const double *>(msg.c_str());
      data_type.int_val = *reinterpret_cast<const int *>(msg.c_str() + sizeof(data_type.double_val));

      std::cout << "Receive from the server double_val: " << data_type.double_val << " ; int_val: " << data_type.int_val
                << std::endl;
    }

    // Send user message
    while (true) {
      std::cout << "Enter the message to send:" << std::endl;
      std::string msg = "";
      std::getline(std::cin, msg);
      if (client.send(msg) != (int)msg.size())
        std::cerr << "Error client.send()!" << std::endl;
      if (client.receive(msg))
        std::cout << "Receive from the server: " << msg << std::endl;
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
