/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Test display polygon lines
 *
 *****************************************************************************/

/*!
  \example testDisplayPolygonLines.cpp

  Test display polygon lines.
*/

#include <cstdlib>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpPolygon.h>
#include <visp3/core/vpImageConvert.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/core/vpRect.h>

// List of allowed command line options
#define GETOPTARGS "cdh"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display);

/*!
  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

 */
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Display polygon lines.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c\n\
     Disable the mouse click. Useful to automate the \n\
     execution of this program without humain intervention.\n\
\n\
  -d                                             \n\
     Disable the image display. This can be useful \n\
     for automatic tests. \n\
\n\
  -h\n\
     Print the help.\n\n");

  if (badparam) {
    fprintf(stderr, "ERROR: \n" );
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}

/*!
  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param click_allowed : Enable/disable mouse click.
  \param display : Set as true, activates the image display. This is
  the default configuration. When set to false, the display is
  disabled. This can be useful for automatic tests using crontab
  under Unix or using the task manager under Windows.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display)
{
  const char *optarg_;
  int c;

  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {
    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg_); return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg_ << std::endl << std::endl;
    return false;
  }

  return true;
}

int main(int argc, const char ** argv)
{
#ifdef VISP_HAVE_DISPLAY
  bool opt_click_allowed = true;
  bool opt_display = true;

  // Read the command line options
  if (getOptions(argc, argv, opt_click_allowed, opt_display) == false) {
    return EXIT_FAILURE;
  }

  if (opt_display && opt_click_allowed) {
    vpImage<unsigned char> I(480, 640, 127);
    vpImage<vpRGBa> I_color(480, 640);

#if defined(VISP_HAVE_X11)
    vpDisplayX d, d2;
#elif defined(VISP_HAVE_GTK)
    vpDisplayGTK d, d2;
#elif defined(VISP_HAVE_GDI)
    vpDisplayGDI d, d2;
#elif defined(VISP_HAVE_D3D9)
    vpDisplayD3D d, d2;
#elif defined(VISP_HAVE_OPENCV)
    vpDisplayOpenCV d, d2;
#endif
    d.init(I, 0, 0, "Grayscale image");

    vpDisplay::display(I);
    vpDisplay::displayText(I, 20, 20, "Left click to draw a polygon, right click when it is finished.", vpColor::red);
    vpDisplay::flush(I);

    vpPolygon polygon;
    polygon.initClick(I);

    vpDisplay::display(I);
    vpDisplay::displayText(I, 20, 20, "Shape is not closed. Click to display dashed lines.", vpColor::red);
    vpDisplay::displayLine(I, polygon.getCorners(), false, vpColor::red, 2);
    vpDisplay::flush(I);
    vpDisplay::getClick(I);

    vpDisplay::display(I);
    vpDisplay::displayText(I, 20, 20, "Shape is closed. Click to draw on color image.", vpColor::red);
    vpDisplay::displayDotLine(I, polygon.getCorners(), true, vpColor::red, 2);
    vpDisplay::flush(I);
    vpDisplay::getClick(I);

    d2.init(I_color, I.getWidth(), 0, "Color image");
    //Create colormap
    for (unsigned int i = 0; i < I_color.getHeight(); i++) {
      double hue = i / (double) I_color.getHeight(), saturation = 1.0, value = 1.0;
      unsigned char rgb[3];
      vpImageConvert::HSVToRGB(&hue, &saturation, &value, rgb, 1);

      for (unsigned int j = 0; j < I_color.getWidth(); j++) {
        I_color[i][j].R = rgb[0];
        I_color[i][j].G = rgb[1];
        I_color[i][j].B = rgb[2];
      }
    }

    vpDisplay::display(I_color);
    vpDisplay::displayText(I_color, 20, 20, "Left click to draw a polygon, right click when it is finished.", vpColor::black);
    vpDisplay::flush(I_color);

    polygon.initClick(I_color);

    vpDisplay::display(I_color);
    vpDisplay::displayText(I_color, 20, 20, "Shape is closed. Click to display dashed lines.", vpColor::black);
    vpDisplay::displayLine(I_color, polygon.getCorners(), true, vpColor::red, 2);
    vpDisplay::flush(I_color);
    vpDisplay::getClick(I_color);

    vpDisplay::display(I_color);
    vpDisplay::displayText(I_color, 20, 20, "Shape is not closed. Click to quit.", vpColor::black);
    vpDisplay::displayDotLine(I_color, polygon.getCorners(), false, vpColor::red, 2);
    vpDisplay::flush(I_color);
    vpDisplay::getClick(I_color);
  }
#else
  (void)argc;
  (void)argv;
#endif
  return EXIT_SUCCESS;
}
