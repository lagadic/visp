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
 * UDP Server
 *
 *****************************************************************************/

#include <cstring>
#include <sstream>

#include <visp3/core/vpConfig.h>

// inet_ntop() not supported on win XP
#ifdef VISP_HAVE_FUNC_INET_NTOP

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#  include <arpa/inet.h>
#  include <errno.h>
#  include <netdb.h>
#  include <unistd.h>
#  define DWORD int
#  define WSAGetLastError() strerror(errno)
#else
#  if defined(__MINGW32__)
#    define _WIN32_WINNT _WIN32_WINNT_VISTA // 0x0600
#  endif
#  include <Ws2tcpip.h>
#endif

#include <visp3/core/vpUDPServer.h>

/*!
  Create a (IPv4) UDP server.

  \param port : Server port number.
  \note The server will listen to all the interfaces (see INADDR_ANY).
*/
vpUDPServer::vpUDPServer(const int port)
  : m_clientAddress(), m_clientLength(0), m_serverAddress(), m_socketFileDescriptor(0)
#if defined(_WIN32)
    ,
    m_wsa()
#endif
{
  init("", port);
}

/*!
  Create a UDP server.

  \param hostname : Server hostname or IP address.
  \param port : Server port number.
*/
vpUDPServer::vpUDPServer(const std::string &hostname, const int port)
  : m_clientAddress(), m_clientLength(0), m_serverAddress(), m_socketFileDescriptor(0)
#if defined(_WIN32)
    ,
    m_wsa()
#endif
{
  init(hostname, port);
}

vpUDPServer::~vpUDPServer()
{
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  close(m_socketFileDescriptor);
#else
  closesocket(m_socketFileDescriptor);
  WSACleanup();
#endif
}

void vpUDPServer::init(const std::string &hostname, const int port)
{
#if defined(_WIN32)
  if (WSAStartup(MAKEWORD(2, 2), &m_wsa) != 0) {
    std::stringstream ss;
    ss << "Failed WSAStartup for the server, error code: " << WSAGetLastError();
    throw vpException(vpException::fatalError, ss.str());
  }
#endif

  /* socket: create the socket */
  m_socketFileDescriptor = socket(AF_INET, SOCK_DGRAM, 0);
#if defined(_WIN32)
  if (m_socketFileDescriptor == INVALID_SOCKET)
#else
  if (m_socketFileDescriptor < 0)
#endif
    throw vpException(vpException::fatalError, "Error opening UDP socket for the server!");

/* setsockopt: Handy debugging trick that lets
 * us rerun the server immediately after we kill it;
 * otherwise we have to wait about 20 secs.
 * Eliminates "ERROR on binding: Address already in use" error.
 */
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  int optval = 1;
  setsockopt(m_socketFileDescriptor, SOL_SOCKET, SO_REUSEADDR, (const void *)&optval, sizeof(int));
#else
  const char optval = 1;
  setsockopt(m_socketFileDescriptor, SOL_SOCKET, SO_REUSEADDR, (const char *)&optval, sizeof(int));
#endif

  /* build the server's Internet address */
  memset(&m_serverAddress, 0, sizeof(m_serverAddress));
  if (hostname.empty()) {
    m_serverAddress.sin_family = AF_INET;
    m_serverAddress.sin_addr.s_addr = htonl(INADDR_ANY);
    m_serverAddress.sin_port = htons((unsigned short)port);
  } else {
    std::stringstream ss;
    ss << port;
    struct addrinfo hints;
    struct addrinfo *result = NULL;
    struct addrinfo *ptr = NULL;

    memset(&hints, 0, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    hints.ai_protocol = IPPROTO_UDP;

    DWORD dwRetval = getaddrinfo(hostname.c_str(), ss.str().c_str(), &hints, &result);
    if (dwRetval != 0) {
      ss.str("");
      ss << "getaddrinfo failed with error: " << dwRetval;
      throw vpException(vpException::fatalError, ss.str());
    }

    for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {
      if (ptr->ai_family == AF_INET && ptr->ai_socktype == SOCK_DGRAM) {
        m_serverAddress = *(struct sockaddr_in *)ptr->ai_addr;
        break;
      }
    }

    freeaddrinfo(result);
  }

  /* bind: associate the parent socket with a port */
  if (bind(m_socketFileDescriptor, (struct sockaddr *)&m_serverAddress, sizeof(m_serverAddress)) < 0)
    throw vpException(vpException::fatalError, "Error on binding on the server!");

  m_clientLength = sizeof(m_clientAddress);
}

/*!
  Receive data sent by a client.

  \param msg : ASCII message or byte data according to the data sent by the
  client. \param timeoutMs : Timeout in millisecond (if zero, the call is
  blocking).

  \return The message length / size of the byte array sent received, or -1 if
  there is an error, or 0 if there is a timeout. \note See
  vpUDPClient::receive for an example.
*/
int vpUDPServer::receive(std::string &msg, const int timeoutMs)
{
  std::string hostInfo = "";
  return receive(msg, hostInfo, timeoutMs);
}

/*!
  Receive data sent by a client.

  \param msg : ASCII message or byte data according to the data send by the
  client. \param hostInfo : Information about the client ("client_name
  client_ip client_port"). \param timeoutMs : Timeout in millisecond (if zero,
  the call is blocking).

  \return The message length / size of the byte array sent received, or -1 if
  there is an error, or 0 if there is a timeout. \note See
  vpUDPClient::receive for an example.
*/
int vpUDPServer::receive(std::string &msg, std::string &hostInfo, const int timeoutMs)
{
  fd_set s;
  FD_ZERO(&s);
  FD_SET(m_socketFileDescriptor, &s);
  struct timeval timeout;
  if (timeoutMs > 0) {
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_usec = (timeoutMs % 1000) * 1000;
  }
  int retval = select((int)m_socketFileDescriptor + 1, &s, NULL, NULL, timeoutMs > 0 ? &timeout : NULL);

  if (retval == -1) {
    std::cerr << "Error select!" << std::endl;
    return -1;
  }

  if (retval > 0) {
/* recvfrom: receive a UDP datagram from a client */
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    int length = static_cast<int>(recvfrom(m_socketFileDescriptor, m_buf, sizeof(m_buf), 0, (struct sockaddr *)&m_clientAddress,
                                           (socklen_t *)&m_clientLength));
#else
    int length =
        recvfrom(m_socketFileDescriptor, m_buf, sizeof(m_buf), 0, (struct sockaddr *)&m_clientAddress, &m_clientLength);
#endif
    if (length <= 0) {
      return length < 0 ? -1 : 0;
    }

    msg = std::string(m_buf, length);

    /* getnameinfo: determine who sent the datagram */
    char hostname[NI_MAXHOST];
    char servInfo[NI_MAXSERV];
    DWORD dwRetval = getnameinfo((struct sockaddr *)&m_clientAddress, sizeof(struct sockaddr), hostname, NI_MAXHOST,
                                 servInfo, NI_MAXSERV, NI_NUMERICSERV);

    std::string hostName = "", hostIp = "", hostPort = "";
    if (dwRetval != 0) {
      std::cerr << "getnameinfo failed with error: " << WSAGetLastError() << std::endl;
    } else {
      hostName = hostname;
      hostPort = servInfo;
    }

    char result[INET_ADDRSTRLEN];
    const char *ptr = inet_ntop(AF_INET, (void *)&m_clientAddress.sin_addr, result, sizeof(result));
    if (ptr == NULL) {
      std::cerr << "inet_ntop failed with error: " << WSAGetLastError() << std::endl;
    } else {
      hostIp = result;
    }

    std::stringstream ss;
    ss << hostName << " " << hostIp << " " << hostPort;
    hostInfo = ss.str();

    return length;
  }

  // Timeout
  return 0;
}

/*!
  Send data to a client.

  \param msg : ASCII message or byte data.
  \param hostname : Client hostname (hostname or ip address).
  \param port : Client port number.

  \return The message length / size of the byte array sent.
  \note See vpUDPClient::send for an example.
*/
int vpUDPServer::send(const std::string &msg, const std::string &hostname, const int port)
{
  if (msg.size() > VP_MAX_UDP_PAYLOAD) {
    std::cerr << "Message is too long!" << std::endl;
    return 0;
  }

  // Create client address
  memset(&m_clientAddress, 0, sizeof(m_clientAddress));
  std::stringstream ss;
  ss << port;
  struct addrinfo hints;
  struct addrinfo *result = NULL;
  struct addrinfo *ptr = NULL;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_INET;
  hints.ai_socktype = SOCK_DGRAM;
  hints.ai_protocol = IPPROTO_UDP;

  DWORD dwRetval = getaddrinfo(hostname.c_str(), ss.str().c_str(), &hints, &result);
  if (dwRetval != 0) {
    ss.str("");
    ss << "getaddrinfo failed with error: " << dwRetval;
    throw vpException(vpException::fatalError, ss.str());
  }

  for (ptr = result; ptr != NULL; ptr = ptr->ai_next) {
    if (ptr->ai_family == AF_INET && ptr->ai_socktype == SOCK_DGRAM) {
      m_clientAddress = *(struct sockaddr_in *)ptr->ai_addr;
      break;
    }
  }

  freeaddrinfo(result);

/* send the message to the client */
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  return static_cast<int>(sendto(m_socketFileDescriptor, msg.c_str(), msg.size(), 0, (struct sockaddr *)&m_clientAddress,
                                 m_clientLength));
#else
  return sendto(m_socketFileDescriptor, msg.c_str(), (int)msg.size(), 0, (struct sockaddr *)&m_clientAddress,
                m_clientLength);
#endif
}

#elif !defined(VISP_BUILD_SHARED_LIBS)
// Work arround to avoid warning: libvisp_core.a(vpUDPServer.cpp.o) has no symbols
void dummy_vpUDPServer(){};
#endif
