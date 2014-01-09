/****************************************************************************
 *
 * $Id: plot.cpp 3530 2012-01-03 10:52:12Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Example which describes how to use the vpPlot class
 *
 * Author:
 * Nicolas Melchior
 *
 *****************************************************************************/


/*!
  \example plot3d.cpp

  Plot 3D curves example.
*/

#include <iostream>
#include <visp/vpConfig.h>
#include <visp/vpPlot.h>

int main ()
{
#if defined(VISP_HAVE_DISPLAY)
  try {
    //Create a window with one graphic
    vpPlot plot(1);

    // Change the default font
    //plot.setFont("-misc-fixed-bold-r-semicondensed--0-0-75-75-c-0-iso8859-10");

    //The graphic contains 2 curves
    plot.initGraph(0,2);

    //Set the graphic parameters
    plot.setTitle(0, "First graphic");
    plot.setUnitX(0, "time (s)");
    plot.setUnitY(0, "y");
    plot.setUnitZ(0, "z");
    plot.setLegend(0,0, "y^2+z^2=1 and y(0) = 1");
    plot.setLegend(0,1, "y^2+z^2=1 and y(0) = -1");
    plot.setColor(0,0,vpColor::red);
    plot.setColor(0,1,vpColor::green);

    double x = 0;
    double y = 1;
    double z = 0 ;
    double dx = 0.08;
    double dy = 0.04;
    double zsign = 1.0;

    unsigned long iter = 0;

    std::cout << "Hit CTRL-C to exit...";
    while(1) {
      if (iter < 300) {
        //y*y+z*z = 1
        if (fabs(y) < 1.0)
          z = sqrt(1.0-y*y);
        else z = 0;

        //Add points to the graphic
        plot.plot(0,0, x, y,z*zsign);
        plot.plot(0,1, x, -y,-z*zsign);

        x += dx;

        if (fabs(y) >= 1.0 )
          dy = -dy;
        y += dy;
        if (fabs(y) >= 1.0 )
          zsign = -zsign;
      }
      else {
        // Tip: to allows modifying the point of view with the mouse we
        // plot always the last point
        plot.plot(0,0, x, y,z*zsign);
        plot.plot(0,1, x, -y,-z*zsign);
      }
      iter ++;
    }

    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
#else
  std::cout << "Plot functionalities are not avalaible since no display is available." << std::endl;
#endif
}
