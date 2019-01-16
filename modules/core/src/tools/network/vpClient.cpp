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
 * TCP Client
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#include <visp3/core/vpClient.h>

// inet_ntop() not supported on win XP
#ifdef VISP_HAVE_FUNC_INET_NTOP

vpClient::vpClient() : vpNetwork(), numberOfAttempts(0) {}

/*!
  Disconnect the client from all the servers, and close the sockets.
*/
vpClient::~vpClient() { stop(); }

/*!
  Connect to the server represented by the given hostname, and at a given
  port.

  \sa vpClient::connectToIP();

  \param hostname : Hostname of the server.
  \param port_serv : Port used for the connection.

  \return True if the connection has been etablished, false otherwise.
*/
bool vpClient::connectToHostname(const std::string &hostname, const unsigned int &port_serv)
{
  // get server host information from hostname
  struct hostent *server = gethostbyname(hostname.c_str());

  if (server == NULL) {
    std::string noSuchHostMessage("ERROR, ");
    noSuchHostMessage.append(hostname);
    noSuchHostMessage.append(": no such host\n");
    vpERROR_TRACE(noSuchHostMessage.c_str(), "vpClient::connectToHostname("
                                             "const std::string &hostname, "
                                             "const int &port_serv)");
    return false;
  }

  vpNetwork::vpReceptor serv;

  serv.socketFileDescriptorReceptor = socket(AF_INET, SOCK_STREAM, 0);

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  if (serv.socketFileDescriptorReceptor < 0) {
#else
  if (serv.socketFileDescriptorReceptor == INVALID_SOCKET) {
#endif
    vpERROR_TRACE("ERROR opening socket", "vpClient::connectToHostname()");
    return false;
  }

  memset((char *)&serv.receptorAddress, '\0', sizeof(serv.receptorAddress));
  serv.receptorAddress.sin_family = AF_INET;
  memmove((char *)&serv.receptorAddress.sin_addr.s_addr, (char *)server->h_addr, (unsigned)server->h_length);
  serv.receptorAddress.sin_port = htons((unsigned short)port_serv);
  serv.receptorIP = inet_ntoa(*(in_addr *)server->h_addr);

  return connectServer(serv);
}

/*!
  Connect to the server represented by the given ip, and at a given port.

  \sa vpClient::connectToHostname()

  \param ip : IP of the server.
  \param port_serv : Port used for the connection.

  \return True if the connection has been etablished, false otherwise.
*/
bool vpClient::connectToIP(const std::string &ip, const unsigned int &port_serv)
{
  vpNetwork::vpReceptor serv;

  serv.socketFileDescriptorReceptor = socket(AF_INET, SOCK_STREAM, 0);

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  if (serv.socketFileDescriptorReceptor < 0) {
#else
  if (serv.socketFileDescriptorReceptor == INVALID_SOCKET) {
#endif
    vpERROR_TRACE("ERROR opening socket", "vpClient::connectToIP()");
    return false;
  }

  memset((char *)&serv.receptorAddress, '\0', sizeof(serv.receptorAddress));
  serv.receptorAddress.sin_family = AF_INET;
  serv.receptorAddress.sin_addr.s_addr = inet_addr(ip.c_str());
  serv.receptorAddress.sin_port = htons((unsigned short)port_serv);

  return connectServer(serv);
}

/*!
  Deconnect from the server at a specific index.

  \param index : Index of the server.
*/
void vpClient::deconnect(const unsigned int &index)
{
  if (index < receptor_list.size()) {
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    shutdown(receptor_list[index].socketFileDescriptorReceptor, SHUT_RDWR);
#else // _WIN32
    shutdown(receptor_list[index].socketFileDescriptorReceptor, SD_BOTH);
#endif
    receptor_list.erase(receptor_list.begin() + (int)index);
  }
}

/*!
  Stops the server and close the sockets.
*/
void vpClient::stop()
{
  for (unsigned int i = 0; i < receptor_list.size(); i++) {
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    shutdown(receptor_list[i].socketFileDescriptorReceptor, SHUT_RDWR);
#else // _WIN32
    shutdown(receptor_list[i].socketFileDescriptorReceptor, SD_BOTH);
#endif
    receptor_list.erase(receptor_list.begin() + (int)i);
    i--;
  }
}

/*!
  Print the servers.
*/
void vpClient::print() { vpNetwork::print("Server"); }

// Private function
bool vpClient::connectServer(vpNetwork::vpReceptor &serv)
{
  serv.receptorAddressSize = sizeof(serv.receptorAddress);

  numberOfAttempts = 15;
  unsigned int ind = 1;
  int connectionResult = -1;

  while (ind <= numberOfAttempts) {
    std::cout << "Attempt number " << ind << "..." << std::endl;

    connectionResult =
        connect(serv.socketFileDescriptorReceptor, (sockaddr *)&serv.receptorAddress, serv.receptorAddressSize);
    if (connectionResult >= 0)
      break;

    ind++;
    vpTime::wait(1000);
  }

  if (connectionResult < 0) {
    vpERROR_TRACE("ERROR connecting, the server may not be waiting for "
                  "connection at this port.",
                  "vpClient::connectServer()");

    return false;
  }

  receptor_list.push_back(serv);

#ifdef SO_NOSIGPIPE
  // Mac OS X does not have the MSG_NOSIGNAL flag. It does have this
  // connections based version, however.
  if (serv.socketFileDescriptorReceptor > 0) {
    int set_option = 1;
    if (0 == setsockopt(serv.socketFileDescriptorReceptor, SOL_SOCKET, SO_NOSIGPIPE, &set_option, sizeof(set_option))) {
    } else {
      std::cout << "Failed to set socket signal option" << std::endl;
    }
  }
#endif // SO_NOSIGPIPE

  std::cout << "Connected!" << std::endl;
  return true;
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpClient.cpp.o) has no symbols
void dummy_vpClient(){};
#endif
