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
 * TCP Server
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

#ifndef vpServer_H
#define vpServer_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpNetwork.h>

/*!
  \class vpServer

  \ingroup group_core_network

  \brief This class represents a Transmission Control Protocol (TCP) server.

  TCP provides reliable, ordered delivery of a stream of bytes from a program
  on one computer to another program on another computer.

  Exemple of server's code, receiving and sending basic message.
  It corresponds to the client used in the first exemple of vpClient class'
  documentation.

  \code
#include <iostream>
#include <visp3/core/vpServer.h>

int main(int argc,const char** argv)
{
  int port = 35000;
  vpServer serv(port); //Launch the server on localhost
  serv.start();

  bool run = true;
  int val;

  while(run){
    serv.checkForConnections();

    if(serv.getNumberOfClients() > 0)
    {
      // Receiving a value from the first client
      if(serv.receive(&val) != sizeof(int))
        std::cout << "Error while receiving" << std::endl;
      else
        std::cout << "Received : " << val << std::endl;

      val = val+1;
      // Sending the new value to the first client
      if(serv.send(&val) != sizeof(int))
        std::cout << "Error while sending" << std::endl;
      else
        std::cout << "Sending : " << val << std::endl;
    }
  }

  return 0;
}
  \endcode

  Exemple of server's code, receiving a vpImage on request form.
  It correspond to the client used in the second exemple of vpClient class'
documentation.

  \code
#include <visp3/core/vpServer.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayX.h>

#include "vpRequestImage.h" //See vpRequest class documentation

int main(int argc,const char** argv)
{
  int port = 35000;

  std::cout << "Port: " << port << std::endl;
  vpServer serv(port);
  serv.start();

#if defined(VISP_HAVE_X11)
  vpDisplayX display;
#elif defined(VISP_HAVE_GDI) //Win32
  vpDisplayGDI display;
#endif

  vpImage<unsigned char> I;

  vpRequestImage reqImage(&I);
  serv.addDecodingRequest(&reqImage);

  bool run = true;

  while(run){
    serv.checkForConnections();

    if(serv.getNumberOfClients() > 0)
    {
      int index = serv.receiveAndDecodeRequestOnce();
      std::string id = serv.getRequestIdFromIndex(index);

      if(id == reqImage.getId())
      {
#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GDI)
        if (! display.isInitialised() )
          display.init(I, -1, -1, "Remote display");
#endif

        vpDisplay::display(I) ;
        vpDisplay::flush(I);

        // A click in the viewer to exit
        if ( vpDisplay::getClick(I, false) )
          run = false;
      }
    }
  }

  return 0;
}
  \endcode

  \sa vpClient
  \sa vpRequest
  \sa vpNetwork
*/
class VISP_EXPORT vpServer : public vpNetwork
{
private:
  //######## PARAMETERS ########
  //#                          #
  //############################
  std::string adress;
  int port;
  bool started;
  unsigned int max_clients;

public:
  vpServer();
  explicit vpServer(const int &port);
  vpServer(const std::string &adress_serv, const int &port_serv);

  virtual ~vpServer();

  bool checkForConnections();

  /*!
    Check if the server is started.

    \sa vpServer::start()

    \return True if the server is started, false otherwise.
  */
  bool isStarted() { return started; }

  /*!
    Get the maximum number of clients that can be connected to the server.

    \sa vpServer::setMaxNumberOfClients()

    \return Maximum number of clients.
  */
  unsigned int getMaxNumberOfClients() { return max_clients; }

  /*!
    Get the number of clients connected to the server.

    \return Number of clients connected.
  */
  unsigned int getNumberOfClients() { return (unsigned int)receptor_list.size(); }

  void print();

  bool start();

  /*!
    Set the maximum number of clients that can be connected to the server.

    \sa vpServer::getMaxNumberOfClients()

    \param l : Maximum number of clients.
  */
  void setMaxNumberOfClients(unsigned int &l) { max_clients = l; }
};

#endif
