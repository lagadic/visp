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
 * TCP Client
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#ifndef vpClient_H
#define vpClient_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpNetwork.h>
#include <visp3/core/vpRequest.h>
#include <visp3/core/vpTime.h>

/*!
  \class vpClient

  \ingroup group_core_network

  \brief This class represents a Transmission Control Protocol (TCP) client.

  TCP provides reliable, ordered delivery of a stream of bytes from a program
  on one computer to another program on another computer.

  Exemple of client's code, receiving and sending basic message
  It corresponds to the client used in the first exemple of vpServer class'
documentation:

  \code
#include <iostream>
#include <visp3/core/vpClient.h>

int main(int argc, char **argv)
{
  std::string servername = "localhost";
  unsigned int port = 35000;

  vpClient client;
  client.connectToHostname(servername, port);
  //client.connectToIP("127.0.0.1",port);

  int val = 0;

  while(1)
  {
    // Sending the new value to the first client
    if(client.send(&val) != sizeof(int))
      std::cout << "Error while sending" << std::endl;
    else
      std::cout << Sending : " << val << std::endl;

    // Receiving a value from the first client
    if(client.receive(&val) != sizeof(int))
      std::cout << "Error while receiving" << std::endl;
    else
      std::cout << "Received : " << val << std::endl;
  }

  return 0;
}
  \endcode

  Exemple of client's code, sending a vpImage on request form.
  It correspond to the server used in the second exemple of vpServer class'
documentation.

  \code
#include <iostream>
#include <visp3/core/vpClient.h>
#include <visp3/core/vpImage.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/sensor/vpV4l2Grabber.h>

#include "vpRequestImage.h" //See vpRequest class documentation

int main(int argc, char **argv)
{
#if defined(VISP_HAVE_V4L2)
  std::string servername = "localhost";
  unsigned int port = 35000;

  vpImage<unsigned char> I; // Create a gray level image container

  // Create a grabber based on v4l2 third party lib (for usb cameras under
  // Linux)
  vpV4l2Grabber g;
  g.setScale(1);
  g.setInput(0);
  g.open(I);

  // Create an image viewer
#if defined(VISP_HAVE_X11)
  vpDisplayX d(I, -1, -1, "Camera frame");
#elif defined(VISP_HAVE_GDI) //Win32
  vpDisplayGDI d(I, -1, -1, "Camera frame");
#endif

  vpClient client;
  client.connectToHostname(servername, port);
  //client.connectToIP("127.0.0.1",port);

  vpRequestImage reqImage(&I);

  while(1)
  {
    double t = vpTime::measureTimeMs();
    // Acquire a new image
    g.acquire(I);

    vpDisplay::display(I);
    vpDisplay::flush(I);

    client.sendAndEncodeRequest(reqImage);

    // A click in the viewer to exit
    if ( vpDisplay::getClick(I, false) )
      break;
  }

  return 0;
#endif
}
  \endcode

  \sa vpClient
  \sa vpRequest
  \sa vpNetwork
*/
class VISP_EXPORT vpClient : public vpNetwork
{
private:
  //######## PARAMETERS ########
  //#                          #
  //############################

  unsigned int numberOfAttempts;

  //######## Private Functions ########
  //#                                 #
  //###################################

  bool connectServer(vpNetwork::vpReceptor &serv);

public:
  vpClient();
  virtual ~vpClient();

  bool connectToHostname(const std::string &hostname, const unsigned int &port_serv);
  bool connectToIP(const std::string &ip, const unsigned int &port_serv);

  void deconnect(const unsigned int &index = 0);
  /*!
    Get the actual number of attempts to connect to the server.

    \sa vpClient::setNumberOfAttempts()

    \return Actual number of attempts.
  */
  unsigned int getNumberOfAttempts() { return numberOfAttempts; }

  /*!
    Get the number of server that the client is connected on.

    \return Number of servers.
  */
  unsigned int getNumberOfServers() { return (unsigned int)receptor_list.size(); }

  void print();

  /*!
    Set the number of attempts to connect to the server.

    \sa vpClient::getNumberOfAttempts()

    \param nb : Number of attempts.
  */
  void setNumberOfAttempts(const unsigned int &nb) { numberOfAttempts = nb; }

  void stop();
};

#endif
