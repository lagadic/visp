/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Test for TCP client/Server.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
  \example testServer.cpp

  Example of a TCP client/server.
*/

#include <iostream>
#include <visp3/core/vpServer.h>

int main()
{
  try {
    int port = 35000;
    vpServer serv(port); //Launch the server on localhost
    serv.start();

    bool run = true;
    int val;

    while(run){
      serv.checkForConnections();

      if(serv.getNumberOfClients() > 0)
      {
        if(serv.receive(&val) != sizeof(int)) //Receiving a value from the first client
          std::cout << "Error while receiving" << std::endl;
        else
          std::cout << "Received : " << val << std::endl;

        val = val+1;
        if(serv.send(&val) != sizeof(int)) //Sending the new value to the first client
          std::cout << "Error while sending" << std::endl;
        else
          std::cout << "Sending : " << val << std::endl;
      }
    }
    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
