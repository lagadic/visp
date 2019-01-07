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
 * TCP Server
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#include <visp3/core/vpServer.h>

// inet_ntop() not supported on win XP
#ifdef VISP_HAVE_FUNC_INET_NTOP

#if defined(__APPLE__) && defined(__MACH__) // Apple OSX and iOS (Darwin)
#include <TargetConditionals.h>             // To detect OSX or IOS using TARGET_OS_IPHONE or TARGET_OS_IOS macro
#endif

/*!
  Construct a server on the machine launching it.
*/
vpServer::vpServer() : adress(), port(0), started(false), max_clients(10)
{
  int protocol = 0;
  emitter.socketFileDescriptorEmitter = socket(AF_INET, SOCK_STREAM, protocol);
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  if (emitter.socketFileDescriptorEmitter < 0)
#else
  if (emitter.socketFileDescriptorEmitter == INVALID_SOCKET)
#endif
  {
    vpERROR_TRACE("vpServer::vpServer(), cannot open socket.");
  }
  emitter.emitterAddress.sin_family = AF_INET;
  emitter.emitterAddress.sin_addr.s_addr = INADDR_ANY;
  emitter.emitterAddress.sin_port = 0;

  adress = inet_ntoa(emitter.emitterAddress.sin_addr);
  port = emitter.emitterAddress.sin_port;
}

/*!
  Construct a server on the machine launching it, with a specified port.

  \param port_serv : server's port.
*/
vpServer::vpServer(const int &port_serv) : adress(), port(0), started(false), max_clients(10)
{
  int protocol = 0;
  emitter.socketFileDescriptorEmitter = socket(AF_INET, SOCK_STREAM, protocol);
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  if (emitter.socketFileDescriptorEmitter < 0)
#else
  if (emitter.socketFileDescriptorEmitter == INVALID_SOCKET)
#endif
  {
    vpERROR_TRACE("vpServer::vpServer(const int &port_serv), cannot open socket.");
  }
  emitter.emitterAddress.sin_family = AF_INET;
  emitter.emitterAddress.sin_addr.s_addr = INADDR_ANY; // inet_addr("127.0.0.1");;
  emitter.emitterAddress.sin_port = htons((unsigned short)port_serv);

  adress = inet_ntoa(emitter.emitterAddress.sin_addr);
  port = port_serv;
}

/*!
  Construct a server on the machine at a given adress, with a specified port.

  \param adress_serv : server's adress.
  \param port_serv : server's port.
*/
vpServer::vpServer(const std::string &adress_serv, const int &port_serv)
  : adress(), port(0), started(false), max_clients(10)
{
  int protocol = 0;
  emitter.socketFileDescriptorEmitter = socket(AF_INET, SOCK_STREAM, protocol);
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  if (emitter.socketFileDescriptorEmitter < 0)
#else
  if (emitter.socketFileDescriptorEmitter == INVALID_SOCKET)
#endif
  {
    vpERROR_TRACE("vpServer::vpServer(const std::string &adress_serv,const "
                  "int &port_serv), cannot open socket.");
  }
  emitter.emitterAddress.sin_family = AF_INET;
  emitter.emitterAddress.sin_addr.s_addr = inet_addr(adress_serv.c_str());
  emitter.emitterAddress.sin_port = htons((unsigned short)port_serv);

  adress = adress_serv;
  port = port_serv;
}

/*!
  Shutdown the server.
*/
vpServer::~vpServer()
{
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  close(emitter.socketFileDescriptorEmitter);
#else // Win32
  closesocket((unsigned)emitter.socketFileDescriptorEmitter);
#endif

  for (unsigned int i = 0; i < receptor_list.size(); i++)
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    close(receptor_list[i].socketFileDescriptorReceptor);
#else // Win32
    closesocket((unsigned)receptor_list[i].socketFileDescriptorReceptor);
#endif
}

/*!
  Enable the server to wait for clients (on the limit of the maximum limit).

  \return True if the server has started, false otherwise.
*/
bool vpServer::start()
{
  int serverStructLength = sizeof(emitter.emitterAddress);
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  int bindResult = bind(emitter.socketFileDescriptorEmitter, (struct sockaddr *)&emitter.emitterAddress,
                        (unsigned)serverStructLength);
#else // Win32
  int bindResult = bind((unsigned)emitter.socketFileDescriptorEmitter, (struct sockaddr *)&emitter.emitterAddress,
                        serverStructLength);
#endif

  if (bindResult < 0) {
    std::cout << "Error id : " << bindResult << std::endl;
    std::string errorMessage("vpServer::vpServer(), cannot bind to port");
    char posrtNumberString[32];
    sprintf(posrtNumberString, "%d", port);
    errorMessage += " ";
    errorMessage += posrtNumberString;
    errorMessage += " The port may be already used.";
    vpERROR_TRACE(errorMessage.c_str());
    return false;
  }

#ifdef SO_NOSIGPIPE
  // Mac OS X does not have the MSG_NOSIGNAL flag. It does have this
  // connections based version, however.
  if (emitter.socketFileDescriptorEmitter > 0) {
    int set_option = 1;
    if (0 ==
        setsockopt(emitter.socketFileDescriptorEmitter, SOL_SOCKET, SO_NOSIGPIPE, &set_option, sizeof(set_option))) {
    } else {
      std::cout << "Failed to set socket signal option" << std::endl;
    }
  }
#endif // SO_NOSIGPIPE

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  listen(emitter.socketFileDescriptorEmitter, (int)max_clients);
#else // Win32
  listen((unsigned)emitter.socketFileDescriptorEmitter, (int)max_clients);
#endif

  std::cout << "Server ready" << std::endl;

  started = true;

  return true;
}

/*!
  Check if a client has connected or deconnected the server

  \return True if a client connected or deconnected, false otherwise OR server
  not started yet.
*/
bool vpServer::checkForConnections()
{
  if (!started)
    if (!start()) {
      return false;
    }

  tv.tv_sec = tv_sec;
#if TARGET_OS_IPHONE
  tv.tv_usec = (int)tv_usec;
#else
  tv.tv_usec = tv_usec;
#endif

  FD_ZERO(&readFileDescriptor);

  socketMax = emitter.socketFileDescriptorEmitter;
  FD_SET((unsigned)emitter.socketFileDescriptorEmitter, &readFileDescriptor);

  for (unsigned int i = 0; i < receptor_list.size(); i++) {
    FD_SET((unsigned)receptor_list[i].socketFileDescriptorReceptor, &readFileDescriptor);

    if (i == 0)
      socketMax = receptor_list[i].socketFileDescriptorReceptor;

    if (socketMax < receptor_list[i].socketFileDescriptorReceptor)
      socketMax = receptor_list[i].socketFileDescriptorReceptor;
  }

  int value = select((int)socketMax + 1, &readFileDescriptor, NULL, NULL, &tv);
  if (value == -1) {
    // vpERROR_TRACE( "vpServer::run(), select()" );
    return false;
  } else if (value == 0) {
    return false;
  } else {
    if (FD_ISSET((unsigned int)emitter.socketFileDescriptorEmitter, &readFileDescriptor)) {
      vpNetwork::vpReceptor client;
      client.receptorAddressSize = sizeof(client.receptorAddress);
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
      client.socketFileDescriptorReceptor = accept(
          emitter.socketFileDescriptorEmitter, (struct sockaddr *)&client.receptorAddress, &client.receptorAddressSize);
#else // Win32
      client.socketFileDescriptorReceptor =
          accept((unsigned int)emitter.socketFileDescriptorEmitter, (struct sockaddr *)&client.receptorAddress,
                 &client.receptorAddressSize);
#endif

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
      if ((client.socketFileDescriptorReceptor) == -1)
#else
      if ((client.socketFileDescriptorReceptor) == INVALID_SOCKET)
#endif
        vpERROR_TRACE("vpServer::run(), accept()");

      client.receptorIP = inet_ntoa(client.receptorAddress.sin_addr);
      printf("New client connected : %s\n", inet_ntoa(client.receptorAddress.sin_addr));
      receptor_list.push_back(client);

      return true;
    } else {
      for (unsigned int i = 0; i < receptor_list.size(); i++) {
        if (FD_ISSET((unsigned int)receptor_list[i].socketFileDescriptorReceptor, &readFileDescriptor)) {
          char deco;
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
          ssize_t numbytes = recv(receptor_list[i].socketFileDescriptorReceptor, &deco, 1, MSG_PEEK);
#else // Win32
          int numbytes = recv((unsigned int)receptor_list[i].socketFileDescriptorReceptor, &deco, 1, MSG_PEEK);
#endif

          if (numbytes == 0) {
            std::cout << "Disconnected : " << inet_ntoa(receptor_list[i].receptorAddress.sin_addr) << std::endl;
            receptor_list.erase(receptor_list.begin() + (int)i);
            return 0;
          }
        }
      }
    }
  }

  return false;
}

/*!
  Print the connected clients.
*/
void vpServer::print() { vpNetwork::print("Client"); }

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpServer.cpp.o) has no symbols
void dummy_vpServer(){};
#endif
