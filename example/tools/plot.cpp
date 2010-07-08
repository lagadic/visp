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
 * Example which describes how to use the vpPlot class
 *
 * Author:
 * Nicolas Melchior
 *
 *****************************************************************************/


/*!
  \example plot.cpp

  Plot example.
*/

#include <visp/vpConfig.h>
#include <visp/vpDebug.h>

#if defined UNIX

#if defined(VISP_HAVE_LIBPLOTTER)

#include <visp/vpPlot.h>
#include <visp/vpMath.h>

int main ()
{
  vpPlot plot(2);

  //Initialize the number of curve for each graphic
  plot.initGraph(0,1);
  plot.initGraph(1,1);

  //Set the color of the curves
  plot.setColor(0,0,vpColor::green);
  plot.setColor(1,0,vpColor::red);

  //Set the titles of the graphic
  char title[40];
  strncpy( title, "cos function", 40 );
  plot.setTitle(0,title);
  strncpy( title, "sin function", 40 );
  plot.setTitle(1, title);

  //Set the legend of each curves
  char legend[40];
  strncpy( legend, "cos x", 40 );
  plot.setLegend(0,0,legend);
  strncpy( legend, "sin x", 40 );
  plot.setLegend(1,0, legend);

  //Set the x axis legend of each curves
  char unit[40];
  strncpy( unit, "x", 40 );
  plot.setUnitX(0,unit);
  strncpy( unit, "x", 40 );
  plot.setUnitX(1,unit);

  //Set the y axis legend of each curves
  strncpy( unit, "y", 40 );
  plot.setUnitY(0,unit);
  strncpy( unit, "y", 40 );
  plot.setUnitY(1,unit);

  //Initialize the graphics
  plot.initRange(0,0,2*M_PI,0.1,-1.5,1.5,0.5,true,true);
  plot.initRange(1,0,2*M_PI,0.1,-1.5,1.5,0.5,true,true);

  //Plot the cosinus and sinus functions
  double i = 0;
  while(i <= 2*M_PI)
  {
    double co = cos(i); 
    double si = sin(i);
    plot.plot(0,0,i,co);
    plot.plot(1,0,i,si);
    i+=0.1;
  }
  
  vpDisplay::getClick(plot.I);

  //Save the datas as text files
  plot.saveData(0, "dataCos.txt");
  plot.saveData(0, "dataSin.txt");
}

#else
int
main()
{
  vpTRACE("Sorry, since libplotter was not detected, plotting functionalities are not available...");
  return 0;
}
#endif

#else
int
main()
{
  vpTRACE("Sorry, plotting functionalities are only available on unix...");
  return 0;
}
#endif
