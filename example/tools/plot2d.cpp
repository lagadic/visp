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
 * Example which describes how to use the vpPlot class
 *
 * Author:
 * Nicolas Melchior
 *
 *****************************************************************************/

/*!
  \example plot2d.cpp

  Plot 2D curves example.
*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>

#include <visp3/core/vpMath.h>
#include <visp3/gui/vpPlot.h>

int main()
{
#if defined(VISP_HAVE_DISPLAY)
  try {
    vpPlot plot(2, 700, 700, 100, 200, "Curves...");

    // Change the default font
    //  plot.setFont("-misc-fixed-bold-r-semicondensed--0-0-75-75-c-0-iso8859-10");

    // Initialize the number of curve for each graphic
    plot.initGraph(0, 1);
    plot.initGraph(1, 1);

    // Set the color of the curves
    plot.setColor(0, 0, vpColor::green);
    plot.setColor(1, 0, vpColor::red);

    // Set the titles of the graphic
    char title[40];
    strncpy(title, "cos function", 40);
    plot.setTitle(0, title);
    strncpy(title, "sin function", 40);
    plot.setTitle(1, title);

    // Set the legend of each curves
    char legend[40];
    strncpy(legend, "cos x", 40);
    plot.setLegend(0, 0, legend);
    strncpy(legend, "sin x", 40);
    plot.setLegend(1, 0, legend);

    // Set the x axis legend of each curves
    char unit[40];
    strncpy(unit, "x", 40);
    plot.setUnitX(0, unit);
    strncpy(unit, "x", 40);
    plot.setUnitX(1, unit);

    // Set the y axis legend of each curves
    strncpy(unit, "y", 40);
    plot.setUnitY(0, unit);
    strncpy(unit, "y", 40);
    plot.setUnitY(1, unit);

    // Plot the cosinus and sinus functions
    double i = 0;
    while (i <= 20 * 2 * M_PI) {
      double co = cos(i);
      double si = sin(i);
      plot.plot(0, 0, i, co);
      plot.plot(1, 0, i, si);
      i += 0.1;
    }

    vpDisplay::getClick(plot.I);

    // Save the datas as text files
    plot.saveData(0, "dataCos.txt", "# ");
    plot.saveData(1, "dataSin.txt", "# ");
    return EXIT_SUCCESS;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }

#else
  std::cout << "Plot functionalities are not avalaible since no display is "
               "available."
            << std::endl;
  return EXIT_SUCCESS;
#endif
}
