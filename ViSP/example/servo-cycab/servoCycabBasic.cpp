/****************************************************************************
 *
 * $Id$
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
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

