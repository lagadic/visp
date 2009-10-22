/****************************************************************************
 *
 * $Id: BSpline.cpp,v 1.1 2008-06-17 08:08:28 asaunier Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
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
 * Exemple of a B-Spline curve.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/
/*!
  \file BSpline.cpp

  \brief Describe a curve thanks to a BSpline.
*/

/*!
  \example BSpline.cpp

  Describe a curve thanks to a BSpline.
*/


#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#include <visp/vpBSpline.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImagePoint.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>

#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>

// List of allowed command line options
#define GETOPTARGS	"cdh"


/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.
  \param ipath : Input image path.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Describe a curve thanks to a BSpline.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c\n\
     Disable the mouse click. Usefull to automaze the \n\
     execution of this program without humain intervention.\n\
\n\
  -d \n\
     Turn off the display.\n\
\n\
  -h\n\
     Print the help.\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}


/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.
  \param ipath : Input image path.
  \param click_allowed : Mouse click activation.
  \param display : Display activation.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv, bool &click_allowed, bool &display)
{
  const char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
    case 'c': click_allowed = false; break;
    case 'd': display = false; break;
    case 'h': usage(argv[0], NULL); return false; break;

    default:
      usage(argv[0], optarg);
      return false; break;
    }
  }

  if ((c == 1) || (c == -1)) {
    // standalone param or error
    usage(argv[0], NULL);
    std::cerr << "ERROR: " << std::endl;
    std::cerr << "  Bad argument " << optarg << std::endl << std::endl;
    return false;
  }

  return true;
}


int
main(int argc, const char ** argv)
{
  bool opt_click_allowed = true;
  bool opt_display = true;

  // Read the command line options
  if (getOptions(argc, argv, opt_click_allowed,
		 opt_display) == false) {
    exit (-1);
  }

  // Declare an image, this is a gray level image (unsigned char)
  // it size is not defined yet, it will be defined when the image will
  // read on the disk
  vpImage<unsigned char> I(540,480);

  // We open a window using either X11, GTK or GDI.
#if defined VISP_HAVE_X11
  vpDisplayX display;
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display;
#elif defined VISP_HAVE_GDI
  vpDisplayGDI display;
#endif

  if (opt_display) {
    try{
      // Display size is automatically defined by the image (I) size
      display.init(I, 100, 100,"Display image") ;
      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;
    }
    catch(...)
    {
      vpERROR_TRACE("Error while displaying the image") ;
      exit(-1);
    }
  }

  vpBSpline bSpline;
  vpList<double> knots;
  knots.addRight(0);
  knots.addRight(0);
  knots.addRight(0);
  knots.addRight(1);
  knots.addRight(2);
  knots.addRight(3);
  knots.addRight(4);
  knots.addRight(4);
  knots.addRight(5);
  knots.addRight(5);
  knots.addRight(5);
  
  vpList<vpImagePoint> controlPoints;
  vpImagePoint pt;
  pt.set_ij(50,300);
  controlPoints.addRight(pt);
  pt.set_ij(100,130);
  controlPoints.addRight(pt);
  pt.set_ij(150,400);
  controlPoints.addRight(pt);
  pt.set_ij(200,370);
  controlPoints.addRight(pt);
  pt.set_ij(250,120);
  controlPoints.addRight(pt);
  pt.set_ij(300,250);
  controlPoints.addRight(pt);
  pt.set_ij(350,200);
  controlPoints.addRight(pt);
  pt.set_ij(400,300);
  controlPoints.addRight(pt);
  
  bSpline.set_p(2);
  bSpline.set_knots(knots);
  bSpline.set_controlPoints(controlPoints);

  std::cout << "The parameters are :" <<std::endl;
  std::cout << "p : " << bSpline.get_p() <<std::endl;
  std::cout << "" <<std::endl;
  std::cout << "The knot vector :" <<std::endl;
  (bSpline.get_knots()).display();
  std::cout << "The control points are :" <<std::endl;
  (bSpline.get_controlPoints()).display();

  int i = bSpline.findSpan(5/2.0);
  std::cout << "The knot interval number for the value u = 5/2 is : " << i <<std::endl;
 
  vpBasisFunction *N = NULL;
  N = bSpline.computeBasisFuns(5/2.0);
  std::cout << "The nonvanishing basis functions N(u=5/2) are :" << std::endl; 
  for (int j = 0; j < bSpline.get_p()+1; j++)
    std::cout << N[j].value << std::endl;
  
  vpBasisFunction **N2 = NULL;
  N2 = bSpline.computeDersBasisFuns(5/2.0, 2);
  std::cout << "The first derivatives of the basis functions N'(u=5/2) are :" << std::endl;
  for (int j = 0; j < bSpline.get_p()+1; j++)
    std::cout << N2[1][j].value << std::endl; 
  
  std::cout << "The second derivatives of the basis functions N''(u=5/2) are :" << std::endl;
  for (int j = 0; j < bSpline.get_p()+1; j++)
    std::cout << N2[2][j].value << std::endl; 
  
  if (opt_display && opt_click_allowed)
  {
    double u = 0.0;
    vpImagePoint pt;
    while (u <= 5)
    {
      pt = bSpline.computeCurvePoint(u);
      vpDisplay::displayCross(I,pt,4,vpColor::red);
      u+=0.01;
    }
    controlPoints.front();
    for (int j = 0; j < controlPoints.nbElements(); j++)
    {
      pt = controlPoints.value();
      vpDisplay::displayCross(I,pt,4,vpColor::green);
      controlPoints.next();
    }
    vpDisplay::flush(I) ;
    vpDisplay::getClick(I);
  }
  
  if (N != NULL) delete[] N;
  if (N2 != NULL) delete[] N2;
  
  return 0;
}