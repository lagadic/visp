/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
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
 * Send a command to the car-like Cycab mobile robot.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/
/*!
  \example servoCycabBasic.cpp

  Send a command to the car-like Cycab mobile robot.

*/

#include <iostream>

#include "visp/vpConfig.h"
#include "visp/vpRobotCycab.h"
#include "visp/vpMath.h"

#ifdef VISP_HAVE_CYCAB

int main()
{
  try {
    double v = 0.1, phi = vpMath::rad(0);
    double x, y;
    double x_min=2000, x_max=0, y_min=2000, y_max=0;
        
    vpRobotCycab robot;
    
    std::cout << "Send the command v: " << v << " m/s and phi: " 
	      << vpMath::deg(phi) << " deg" << std::endl;
    while (true) {
    
      robot.setCommand(v, phi);
      robot.getJoystickPosition(x, y);
      //std::cout << "x: " << x << " y: " << y << std::endl;
      if (x < x_min) x_min = x;
      if (y < y_min) y_min = y;
      if (x > x_max) x_max = x;
      if (y > y_max) y_max = y;
      std::cout << "x_min: " << x_min << " x_max: " << x_max 
		<< " y_min: " << y_min << " y_max: " << y_max 
		<< std::endl;
    
      usleep(10000);
    }
  }
  catch(...) {
    std::cerr << "An exception was catched" << std::endl;
   
    return 1;
  }

  return 0;
}
		
#else // VISP_HAVE_CYCAB		
int main()
{
  std::cout << "Sorry, you don't have access to the Cycab car-like mobile robot." 
	    << std::endl;
}
#endif // VISP_HAVE_CYCAB

