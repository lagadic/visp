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
 * TCP Network
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#ifndef vpNetwork_H
#define vpNetwork_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpRequest.h>

#include <iostream>
#include <stdio.h>
#include <string.h>
#include <vector>

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#else
#include <io.h>
//#  include<winsock.h>
#include <winsock2.h>
//#  pragma comment(lib, "ws2_32.lib") // Done by CMake in main CMakeLists.txt
#endif

#if defined(__APPLE__) && defined(__MACH__) // Apple OSX and iOS (Darwin)
#include <TargetConditionals.h>             // To detect OSX or IOS using TARGET_OS_IPHONE or TARGET_OS_IOS macro
#endif

/*!
  \class vpNetwork

  \ingroup group_core_network

  \brief This class represents a Transmission Control Protocol (TCP) network.

  TCP provides reliable, ordered delivery of a stream of bytes from a program
  on one computer to another program on another computer.

  \warning This class shouldn't be used directly. You better use vpClient and
  vpServer to simulate your network. Some exemples are provided in these
  classes.

  \sa vpServer
  \sa vpNetwork
*/
class VISP_EXPORT vpNetwork
{
protected:
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  struct vpReceptor {
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    int socketFileDescriptorReceptor;
    socklen_t receptorAddressSize;
#else
    SOCKET socketFileDescriptorReceptor;
    int receptorAddressSize;
#endif
    struct sockaddr_in receptorAddress;
    std::string receptorIP;

    vpReceptor() : socketFileDescriptorReceptor(0), receptorAddressSize(), receptorAddress(), receptorIP() {}
  };

  struct vpEmitter {
    struct sockaddr_in emitterAddress;
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
    int socketFileDescriptorEmitter;
#else
    SOCKET socketFileDescriptorEmitter;
#endif
    vpEmitter() : emitterAddress(), socketFileDescriptorEmitter(0)
    {
      emitterAddress.sin_family = AF_INET;
      emitterAddress.sin_addr.s_addr = INADDR_ANY;
      emitterAddress.sin_port = 0;
      socketFileDescriptorEmitter = 0;
    }
  };
#endif

  //######## PARAMETERS ########
  //#                          #
  //############################

  vpEmitter emitter;
  std::vector<vpReceptor> receptor_list;
  fd_set readFileDescriptor;
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  int socketMax;
#else
  SOCKET socketMax;
#endif

  // Message Handling
  std::vector<vpRequest *> request_list;

  unsigned int max_size_message;
  std::string separator;
  std::string beginning;
  std::string end;
  std::string param_sep;

  std::string currentMessageReceived;

  struct timeval tv;
  long tv_sec;
  long tv_usec;

  bool verboseMode;

private:
  std::vector<int> _handleRequests();
  int _handleFirstRequest();

  void _receiveRequest();
  void _receiveRequestFrom(const unsigned int &receptorEmitting);
  int _receiveRequestOnce();
  int _receiveRequestOnceFrom(const unsigned int &receptorEmitting);

public:
  vpNetwork();
  virtual ~vpNetwork();

  void addDecodingRequest(vpRequest *);

  int getReceptorIndex(const char *name);

  /*!
    Get the Id of the request at the index ind.

    \param ind : Index of the request.

    \return Id of the request.
  */
  std::string getRequestIdFromIndex(const int &ind)
  {
    if (ind >= (int)request_list.size() || ind < 0)
      return "";
    return request_list[(unsigned)ind]->getId();
  }

  /*!
    Get the maximum size that the emitter can receive (in request mode).

    \sa vpNetwork::setMaxSizeReceivedMessage()

    \return Acutal max size value.
  */
  unsigned int getMaxSizeReceivedMessage() { return max_size_message; }

  void print(const char *id = "");

  template <typename T> int receive(T *object, const unsigned int &sizeOfObject = sizeof(T));
  template <typename T>
  int receiveFrom(T *object, const unsigned int &receptorEmitting, const unsigned int &sizeOfObject = sizeof(T));

  std::vector<int> receiveRequest();
  std::vector<int> receiveRequestFrom(const unsigned int &receptorEmitting);
  int receiveRequestOnce();
  int receiveRequestOnceFrom(const unsigned int &receptorEmitting);

  std::vector<int> receiveAndDecodeRequest();
  std::vector<int> receiveAndDecodeRequestFrom(const unsigned int &receptorEmitting);
  int receiveAndDecodeRequestOnce();
  int receiveAndDecodeRequestOnceFrom(const unsigned int &receptorEmitting);

  void removeDecodingRequest(const char *);

  template <typename T> int send(T *object, const int unsigned &sizeOfObject = sizeof(T));
  template <typename T> int sendTo(T *object, const unsigned int &dest, const unsigned int &sizeOfObject = sizeof(T));

  int sendRequest(vpRequest &req);
  int sendRequestTo(vpRequest &req, const unsigned int &dest);

  int sendAndEncodeRequest(vpRequest &req);
  int sendAndEncodeRequestTo(vpRequest &req, const unsigned int &dest);

  /*!
    Change the maximum size that the emitter can receive (in request mode).

    \sa vpNetwork::getMaxSizeReceivedMessage()

    \param s : new maximum size value.
  */
  void setMaxSizeReceivedMessage(const unsigned int &s) { max_size_message = s; }

  /*!
    Change the time the emitter spend to check if he receives a message from a
    receptor. Initially this value is set to 10usec.

    \sa vpNetwork::setTimeoutUSec()

    \param sec : new value in second.
  */
  void setTimeoutSec(const long &sec) { tv_sec = sec; }

  /*!
    Change the time the emitter spend to check if he receives a message from a
    receptor. Initially this value is set to 10usec.

    \sa vpNetwork::setTimeoutSec()

    \param usec : new value in micro second.
  */
  void setTimeoutUSec(const long &usec) { tv_usec = usec; }

  /*!
    Set the verbose mode.

    \param mode : Change the verbose mode. True to turn on, False to turn off.
  */
  void setVerbose(const bool &mode) { verboseMode = mode; }
};

//######## Definition of Template Functions ########
//#                                                #
//##################################################

/*!
  Receives a object. The size of the received object is suppose to be the size
  of the type of the object. Note that a received message can correspond to a
  deconnection signal.

  \warning Using this function means that you know what kind of object you are
  suppose to receive, and when you are suppose to receive. If the emitter has
  several receptors. It might be a problem, and in that case you better use
  the "request" option.

  \sa vpNetwork::receiveRequest()
  \sa vpNetwork::receiveRequestOnce()
  \sa vpNetwork::receiveAndDecodeRequest()
  \sa vpNetwork::receiveAndDecodeRequestOnce()

  \param object : Received object.
  \param sizeOfObject : Size of the received object.

  \return the number of bytes received, or -1 if an error occured.
*/
template <typename T> int vpNetwork::receive(T *object, const unsigned int &sizeOfObject)
{
  if (receptor_list.size() == 0) {
    if (verboseMode)
      vpTRACE("No receptor");
    return -1;
  }

  tv.tv_sec = tv_sec;
#if TARGET_OS_IPHONE
  tv.tv_usec = (int)tv_usec;
#else
  tv.tv_usec = tv_usec;
#endif

  FD_ZERO(&readFileDescriptor);

  for (unsigned int i = 0; i < receptor_list.size(); i++) {
    FD_SET((unsigned int)receptor_list[i].socketFileDescriptorReceptor, &readFileDescriptor);

    if (i == 0)
      socketMax = receptor_list[i].socketFileDescriptorReceptor;

    if (socketMax < receptor_list[i].socketFileDescriptorReceptor)
      socketMax = receptor_list[i].socketFileDescriptorReceptor;
  }

  int value = select((int)socketMax + 1, &readFileDescriptor, NULL, NULL, &tv);
  int numbytes = 0;

  if (value == -1) {
    if (verboseMode)
      vpERROR_TRACE("Select error");
    return -1;
  } else if (value == 0) {
    // Timeout
    return 0;
  } else {
    for (unsigned int i = 0; i < receptor_list.size(); i++) {
      if (FD_ISSET((unsigned int)receptor_list[i].socketFileDescriptorReceptor, &readFileDescriptor)) {
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
        numbytes = recv(receptor_list[i].socketFileDescriptorReceptor, (char *)(void *)object, sizeOfObject, 0);
#else
        numbytes = recv((unsigned int)receptor_list[i].socketFileDescriptorReceptor, (char *)(void *)object,
                        (int)sizeOfObject, 0);
#endif
        if (numbytes <= 0) {
          std::cout << "Disconnected : " << inet_ntoa(receptor_list[i].receptorAddress.sin_addr) << std::endl;
          receptor_list.erase(receptor_list.begin() + (int)i);
          return numbytes;
        }

        break;
      }
    }
  }

  return numbytes;
}

/*!
  Receives a object from a receptor, by specifying its size or not.
  Note that a received message can correspond to a deconnection signal.

  \warning Using this function means that you know what kind of object you are
  suppose to receive, and when you are suppose to receive. If the emitter has
  several receptors. It might be a problem, and in that case you better use
  the "request" mode.

  \sa vpNetwork::getReceptorIndex()
  \sa vpNetwork::receiveRequestFrom()
  \sa vpNetwork::receiveRequestOnceFrom()
  \sa vpNetwork::receiveAndDecodeRequestFrom()
  \sa vpNetwork::receiveAndDecodeRequestOnceFrom()

  \param object : Received object.
  \param receptorEmitting : Index of the receptor emitting the message.
  \param sizeOfObject : Size of the received object.

  \return the number of bytes received, or -1 if an error occured.
*/
template <typename T>
int vpNetwork::receiveFrom(T *object, const unsigned int &receptorEmitting, const unsigned int &sizeOfObject)
{
  if (receptor_list.size() == 0 || receptorEmitting > (unsigned int)receptor_list.size() - 1) {
    if (verboseMode)
      vpTRACE("No receptor at the specified index");
    return -1;
  }

  tv.tv_sec = tv_sec;
#if TARGET_OS_IPHONE
  tv.tv_usec = (int)tv_usec;
#else
  tv.tv_usec = tv_usec;
#endif

  FD_ZERO(&readFileDescriptor);

  socketMax = receptor_list[receptorEmitting].socketFileDescriptorReceptor;
  FD_SET((unsigned int)receptor_list[receptorEmitting].socketFileDescriptorReceptor, &readFileDescriptor);

  int value = select((int)socketMax + 1, &readFileDescriptor, NULL, NULL, &tv);
  int numbytes = 0;

  if (value == -1) {
    if (verboseMode)
      vpERROR_TRACE("Select error");
    return -1;
  } else if (value == 0) {
    // timeout
    return 0;
  } else {
    if (FD_ISSET((unsigned int)receptor_list[receptorEmitting].socketFileDescriptorReceptor, &readFileDescriptor)) {
#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
      numbytes =
          recv(receptor_list[receptorEmitting].socketFileDescriptorReceptor, (char *)(void *)object, sizeOfObject, 0);
#else
      numbytes = recv((unsigned int)receptor_list[receptorEmitting].socketFileDescriptorReceptor,
                      (char *)(void *)object, (int)sizeOfObject, 0);
#endif
      if (numbytes <= 0) {
        std::cout << "Disconnected : " << inet_ntoa(receptor_list[receptorEmitting].receptorAddress.sin_addr)
                  << std::endl;
        receptor_list.erase(receptor_list.begin() + (int)receptorEmitting);
        return numbytes;
      }
    }
  }

  return numbytes;
}

/*!
  Send an object. The size of the received object is suppose to be the size of
  its type. Note that sending object containing pointers, virtual methods,
  etc, won't probably work.

  \warning Using this function means that, in the other side of the network,
  it knows what kind of object it is suppose to receive, and when it is
  suppose to receive. If the emitter has several receptors. It might be a
  problem, and in that case you better use the "request" option.

  \sa vpNetwork::sendTo()
  \sa vpNetwork::sendRequest()
  \sa vpNetwork::sendRequestTo()
  \sa vpNetwork::sendAndEncodeRequest()
  \sa vpNetwork::sendAndEncodeRequestTo()

  \param object : Received object.
  \param sizeOfObject : Size of the object

  \return The number of bytes sent, or -1 if an error happened.
*/
template <typename T> int vpNetwork::send(T *object, const unsigned int &sizeOfObject)
{
  if (receptor_list.size() == 0) {
    if (verboseMode)
      vpTRACE("No receptor !");
    return 0;
  }

  int flags = 0;
//#if ! defined(APPLE) && ! defined(SOLARIS) && ! defined(_WIN32)
#if defined(__linux__)
  flags = MSG_NOSIGNAL; // Only for Linux
#endif

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  return sendto(receptor_list[0].socketFileDescriptorReceptor, (const char *)(void *)object, sizeOfObject, flags,
                (sockaddr *)&receptor_list[0].receptorAddress, receptor_list[0].receptorAddressSize);
#else
  return sendto(receptor_list[0].socketFileDescriptorReceptor, (const char *)(void *)object, (int)sizeOfObject, flags,
                (sockaddr *)&receptor_list[0].receptorAddress, receptor_list[0].receptorAddressSize);
#endif
}

/*!
  Send an object. The size has to be specified.

  \warning Using this function means that, in the other side of the network,
  it knows what kind of object it is suppose to receive, and when it is
  suppose to receive. If the emitter has several receptors. It might be a
  problem, and in that case you better use the "request" option.

  \sa vpNetwork::getReceptorIndex()
  \sa vpNetwork::send()
  \sa vpNetwork::sendRequest()
  \sa vpNetwork::sendRequestTo()
  \sa vpNetwork::sendAndEncodeRequest()
  \sa vpNetwork::sendAndEncodeRequestTo()

  \param object : Object to send.
  \param dest : Index of the receptor that you are sending the object.
  \param sizeOfObject : Size of the object.

  \return The number of bytes sent, or -1 if an error happened.
*/
template <typename T> int vpNetwork::sendTo(T *object, const unsigned int &dest, const unsigned int &sizeOfObject)
{
  if (receptor_list.size() == 0 || dest > (unsigned int)receptor_list.size() - 1) {
    if (verboseMode)
      vpTRACE("No receptor at the specified index.");
    return 0;
  }

  int flags = 0;
//#if ! defined(APPLE) && ! defined(SOLARIS) && ! defined(_WIN32)
#if defined(__linux__)
  flags = MSG_NOSIGNAL; // Only for Linux
#endif

#if !defined(_WIN32) && (defined(__unix__) || defined(__unix) || (defined(__APPLE__) && defined(__MACH__))) // UNIX
  return sendto(receptor_list[dest].socketFileDescriptorReceptor, (const char *)(void *)object, sizeOfObject, flags,
                (sockaddr *)&receptor_list[dest].receptorAddress, receptor_list[dest].receptorAddressSize);
#else
  return sendto(receptor_list[dest].socketFileDescriptorReceptor, (const char *)(void *)object, (int)sizeOfObject,
                flags, (sockaddr *)&receptor_list[dest].receptorAddress, receptor_list[dest].receptorAddressSize);
#endif
}

#endif
