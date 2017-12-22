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
 * UDP Server
 *
 *****************************************************************************/

#ifndef __vpUDPServer_h__
#define __vpUDPServer_h__

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
  \class vpUDPServer

  \ingroup group_core_network

  \brief This class implements a basic (IPv4) User Datagram Protocol (UDP)
server.

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

  Example of a server's code, receiving a basic message and sending an
  echo message to the client:

  \code
#include <cstdlib>
#include <iostream>
#include <iterator>
#include <sstream>
#include <vector>
#include <visp3/core/vpUDPServer.h>

int main() {
  try {
    int port = 50037;
    vpUDPServer server(port);

    while (true) {
    std::string msg = "", hostInfo = "";
      int res = server.receive(msg, hostInfo, 5000);
      if (res) {
        std::cout << "Server received: " << msg << " from: " << hostInfo << std::endl;
        std::cout << "Reply to the client: Echo: " << msg << std::endl;

        //Get address and port
        std::istringstream iss(hostInfo);
        std::vector<std::string> tokens;
        std::copy(std::istream_iterator<std::string>(iss),
                  std::istream_iterator<std::string>(),
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
#include <cstring>
#include <iostream>
#include <iterator>
#include <sstream>
#include <vector>
#include <visp3/core/vpUDPServer.h>

struct DataType {
  double double_val;
  int int_val;

  DataType() : double_val(0.0), int_val(0) {}
  DataType(const double dbl, const int i) : double_val(dbl), int_val(i) {}
};

int main() {
  try {
    int port = 50037;
    vpUDPServer server(port);

    std::string msg = "", hostInfo = "";
    int res = server.receive(msg, hostInfo);
    if (res) {
      DataType data_type;
      memcpy(&data_type.double_val, msg.c_str(),
sizeof(data_type.double_val)); memcpy(&data_type.int_val,
msg.c_str()+sizeof(data_type.double_val), sizeof(data_type.int_val));
      std::cout << "Server received double_val: " << data_type.double_val << "
; int_val: " << data_type.int_val << " from: " << hostInfo << std::endl;

      //Get address and port
      std::istringstream iss(hostInfo);
      std::vector<std::string> tokens;
      std::copy(std::istream_iterator<std::string>(iss),
                std::istream_iterator<std::string>(),
                std::back_inserter(tokens));
      data_type.double_val += 1.5;
      data_type.int_val += 2;
      char data[sizeof(data_type.double_val)+sizeof(data_type.int_val)];
      memcpy(data, &data_type.double_val, sizeof(data_type.double_val));
      memcpy(data+sizeof(data_type.double_val), &data_type.int_val,
sizeof(data_type.int_val)); msg = std::string(data,
sizeof(data_type.double_val)+sizeof(data_type.int_val));

      server.send(msg, tokens[1], atoi(tokens[2].c_str()));
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
class VISP_EXPORT vpUDPServer
{
public:
  vpUDPServer(const int port);
  vpUDPServer(const std::string &hostname, const int port);
  ~vpUDPServer();

  int receive(std::string &msg, const int timeoutMs = 0);
  int receive(std::string &msg, std::string &hostInfo, const int timeoutMs = 0);
  int send(const std::string &msg, const std::string &hostname, const int port);

private:
  char m_buf[VP_MAX_UDP_PAYLOAD];
  struct sockaddr_in m_clientAddress;
  int m_clientLength;
  struct sockaddr_in m_serverAddress;
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  int m_socketFileDescriptor;
#else
  SOCKET m_socketFileDescriptor;
  WSADATA m_wsa;
#endif

  void init(const std::string &hostname, const int port);
};

#endif
