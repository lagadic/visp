/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2012 by INRIA. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 *
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Test for TCP client/Server.
 *
 * Authors:
 * Aurelien Yol
 *
 *****************************************************************************/

/*!
  \example testClient.cpp

  Example of a TCP client/server.
*/

#include <iostream>
#include <visp/vpClient.h>

int main()
{
  std::string servername = "localhost";
  unsigned int port = 35000;

  vpClient client;
  client.connectToHostname(servername, port);
  //client.connectToIP("127.0.0.1",port);

  int val = 0;

  while(1)
  {
    if(client.send(&val) != sizeof(int)) //Sending the new value to the first client
      std::cout << "Error while sending" << std::endl;
    else
      std::cout << "Sending : " << val << std::endl;
    
    if(client.receive(&val) != sizeof(int)) //Receiving a value from the first client
        std::cout << "Error while receiving" << std::endl;
    else
      std::cout << "Received : " << val << std::endl;
  }

  return 0;
}
