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
 * UDP Client
 *
 *****************************************************************************/

#ifndef __vpUDPClient_h__
#define __vpUDPClient_h__

#include <visp3/core/vpConfig.h>

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#else
#include <winsock2.h>
#endif

#include <visp3/core/vpException.h>

#define VP_MAX_UDP_PAYLOAD 508

/*!
  \class vpUDPClient

  \ingroup group_core_network

  \brief This class implements a basic (IPv4) User Datagram Protocol (UDP)
client.

  More information <a href="https://tools.ietf.org/html/rfc768">here</a>,
  <a
href="https://www.beej.us/guide/bgnet/output/html/singlepage/bgnet.html">here</a>
  or <a
href="https://www.ibm.com/support/knowledgecenter/en/SSLTBW_2.1.0/com.ibm.zos.v2r1.hala001/samples.htm">here</a>:
  <blockquote>
  This User Datagram  Protocol  (UDP)  is  defined  to  make  available  a
  datagram   mode  of  packet-switched   computer   communication  in  the
  environment  of  an  interconnected  set  of  computer  networks.   This
  protocol  assumes  that the Internet  Protocol  (IP)  [1] is used as the
  underlying protocol.

  This protocol  provides  a procedure  for application  programs  to send
  messages  to other programs  with a minimum  of protocol mechanism.  The
  protocol  is transaction oriented, and delivery and duplicate protection
  are not guaranteed.  Applications requiring ordered reliable delivery of
  streams of data should use the Transmission Control Protocol (TCP) [2].
  </blockquote>

  Example of a client's code, sending a basic message and receiving the
  server answer:

  \code
#include <cstdlib>
#include <iostream>
#include <visp3/core/vpUDPClient.h>

int main() {
  try {
    std::string servername = "127.0.0.1";
    unsigned int port = 50037;
    vpUDPClient client(servername, port);

    while (true) {
      std::cout << "Enter the message to send:" << std::endl;
      std::string msg = "";
      std::getline(std::cin, msg);
      if (client.send(msg) != (int) msg.size())
        std::cerr << "Error client.send()!" << std::endl;
      if (client.receive(msg))
        std::cout << "Receive from the server: " << msg << std::endl;
    }
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
  \endcode

  If you want to send a complex data type, you can either send the ASCII
representation or send directly the byte data. In the last case, you should
have to handle that both the server and the client have the same data type
representation. Be careful also with the endianness of the network / host.

  Here an example using a structure of data, assuming that both the server and
the client have the same architecture (probably you should write your own
serialization / deserialization functions for the data you want to send /
receive):

  \code
#include <cstdlib>
#include <iostream>
#include <cstring>
#include <visp3/core/vpUDPClient.h>

struct DataType {
  double double_val;
  int int_val;
  DataType() : double_val(0.0), int_val(0) {}
  DataType(const double dbl, const int i) : double_val(dbl), int_val(i) {}
};

int main() {
  try {
    std::string servername = "127.0.0.1";
    unsigned int port = 50037;
    vpUDPClient client(servername, port);
    DataType data_type(1234.56789, 123450);
    char data[sizeof(data_type.double_val)+sizeof(data_type.int_val)];

    memcpy(data, &data_type.double_val, sizeof(data_type.double_val));
    memcpy(data+sizeof(data_type.double_val), &data_type.int_val, sizeof(data_type.int_val));

    std::string msg(data, sizeof(data_type.double_val)+sizeof(data_type.int_val));
    if (client.send(msg) != (int) sizeof(data_type.double_val)+sizeof(data_type.int_val))
      std::cerr << "Error client.send()!" << std::endl;
    if (client.receive(msg)) {
      data_type.double_val = *reinterpret_cast<const double *>(msg.c_str());
      data_type.int_val
        = *reinterpret_cast<const int *>(msg.c_str()+sizeof(data_type.double_val));
      std::cout << "Receive from the server double_val: " << data_type.double_val
                << " ; int_val: " << data_type.int_val << std::endl;
    }
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cerr << "Catch an exception: " << e.what() << std::endl;
    return EXIT_FAILURE;
  }
}
  \endcode

  \sa vpUDPServer
*/
class VISP_EXPORT vpUDPClient
{
public:
  vpUDPClient(const std::string &hostname, const int port);
  ~vpUDPClient();

  int receive(std::string &msg, const int timeoutMs = 0);
  int send(const std::string &msg);

private:
  char m_buf[VP_MAX_UDP_PAYLOAD];
  struct sockaddr_in m_serverAddress;
  int m_serverLength;
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  int m_socketFileDescriptor;
#else
  SOCKET m_socketFileDescriptor;
  WSADATA m_wsa;
#endif

  void init(const std::string &hostname, const int port);
};

#endif
