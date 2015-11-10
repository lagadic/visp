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
 * Example which test the polygon.
 *
 * Author:
 * Romain Tallonneau
 *
 *****************************************************************************/


#include <visp3/core/vpConfig.h>
#include <visp3/core/vpPolygon.h>
#include <visp3/io/vpParseArgv.h>
#include <visp3/core/vpImagePoint.h>

#include <visp3/core/vpDisplay.h>
#include <visp3/gui/vpDisplayX.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayGDI.h>


#include <math.h>

#include <iostream>
#include <string>
#include <vector>

//! List of allowed command line options
#define GETOPTARGS	"cdh"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv, bool& opt_display, bool& opt_click);

/*!

Print the program options.

\param name : Program name.
\param badparam : Bad parameter name.

 */
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
test the generic 2D polygons.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n						      \
", name);

  fprintf(stdout, "\n\
OPTIONS: \n\
  -c \n\
     Disable mouse click.\n\
\n\
  -d \n\
     Turn off display.\n\
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
  \param opt_display : optionnal flag to turn off the display.
  \param opt_click : activates the mouse click.
  \return false if the program has to be stopped, true otherwise.
*/
bool getOptions(int argc, const char **argv, bool& opt_display, bool& opt_click)
{
  const char *optarg_;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c': opt_click = false; break;
    case 'd': opt_display = false; break;
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



/* -------------------------------------------------------------------------- */
/*                               MAIN FUNCTION                                */
/* -------------------------------------------------------------------------- */

int 
main(int argc, const char** argv)
{
  try {
    bool opt_display = true;
    bool opt_click = true;
    vpImage<unsigned char> I(480, 640, 255);

    // Read the command line options
    if (getOptions(argc, argv, opt_display, opt_click) == false) {
      return (-1);
    }

    std::vector <vpImagePoint> vec1;
    vec1.push_back(vpImagePoint(200, 200));
    vec1.push_back(vpImagePoint(200, 400));
    vec1.push_back(vpImagePoint(320, 400));
    vec1.push_back(vpImagePoint(380, 300));
    vec1.push_back(vpImagePoint(280, 280));
    vpPolygon p1;
    p1.buildFrom(vec1);

    std::vector <vpImagePoint> vec2;
    vec2.push_back(vpImagePoint(20, 20));
    vec2.push_back(vpImagePoint(100, 20));
    vec2.push_back(vpImagePoint(100, 100));
    vec2.push_back(vpImagePoint(20, 100));
    vpPolygon p2(vec2);


    std::vector <vpImagePoint> vec3;
    vpPolygon p3(vec3);

#if defined VISP_HAVE_X11
    vpDisplayX display;
#elif defined VISP_HAVE_GTK
    vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
    vpDisplayGDI display;
#else
    opt_display = false;
#endif

    std::cout << " Polygon 1 : " << std::endl;
    std::cout << " area : " << p1.getArea() << std::endl;
    std::cout << " center : " << p1.getCenter() << std::endl << std::endl;

    std::cout << " Polygon 2 : " << std::endl;
    std::cout << " area : " << p2.getArea() << std::endl;
    std::cout << " center : " << p2.getCenter() << std::endl << std::endl;

    std::cout << " Polygon 3 : " << std::endl;
    std::cout << " area : " << p3.getArea() << std::endl;
    std::cout << " center : " << p3.getCenter() << std::endl;


    if(opt_display){
#if (defined VISP_HAVE_X11) || (defined VISP_HAVE_GTK) || (defined VISP_HAVE_GDI)
      display.init(I, 10, 10, "Test vpPolygon");
#endif
      vpDisplay::display(I);
      p1.display(I, vpColor::green, 1);
      vpDisplay::displayCross(I, p1.getCenter(), 5, vpColor::green);
      p2.display(I, vpColor::red, 1);
      vpDisplay::displayCross(I, p2.getCenter(), 5, vpColor::red);
      p3.display(I, vpColor::blue, 1);
      vpDisplay::displayCross(I, p3.getCenter(), 5, vpColor::lightBlue);
      vpDisplay::displayText(I, vpImagePoint(10, 10), "Click to finish", vpColor::red);
      vpDisplay::flush(I);

      if (opt_click)
        vpDisplay::getClick(I);


      vpDisplay::display(I);
      vpDisplay::displayText(I, vpImagePoint(10, 10), "Left click to add a point", vpColor::red);
      vpDisplay::displayText(I, vpImagePoint(20, 10), "Right click to build the polygon", vpColor::red);
      vpDisplay::flush(I);
      if (opt_click) {
        vpPolygon p4;
        p4.initClick(I);
        p4.display(I, vpColor::green, 1);
        std::cout << std::endl;
        std::cout << " Polygon 4 : " << std::endl;
        std::cout << " area : " << p4.getArea() << std::endl;
        std::cout << " center : " << p4.getCenter() << std::endl;
        std::cout << "Click to continue." << std::endl;
        vpDisplay::flush(I);
        vpDisplay::getClick(I);

        vpRect bbox = p4.getBoundingBox();
        for(unsigned int i= (unsigned int)floor(bbox.getTop()); i<(unsigned int)ceil(bbox.getBottom()); ++i){
          for(unsigned int j=(unsigned int)floor(bbox.getLeft()); j<(unsigned int)ceil(bbox.getRight()); ++j){
            if(p4.isInside(vpImagePoint(i, j))){
              vpDisplay::displayPoint(I, vpImagePoint(i, j), vpColor::orange);
            }
          }
        }
        vpDisplay::flush(I);
        std::cout << "Click to finish." << std::endl;

        vpDisplay::getClick(I);
      }
    }

    return 0;
  }
  catch(vpException e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}


