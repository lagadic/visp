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
 * Exemple of a Nurbs curve.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/
/*!
  \file Nurbs.cpp

  \brief Describe a curve thanks to a Nurbs.
*/

/*!
  \example Nurbs.cpp

  Describe a curve thanks to a BSpline.
*/

#include <cstdlib>

#include <visp/vpDebug.h>
#include <visp/vpConfig.h>

#include <visp/vpNurbs.h>

#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpImagePoint.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpDisplayOpenCV.h> // Should be after #include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayD3D.h>
#include <visp/vpDisplayX.h>

#include <visp/vpParseArgv.h>
#include <visp/vpIoTools.h>

#if defined(VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_GDI) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_D3D9)

// List of allowed command line options
#define GETOPTARGS	"cdh"


/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Describe a curve thanks to a Nurbs.\n\
\n\
SYNOPSIS\n\
  %s [-c] [-d] [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -c\n\
     Disable the mouse click. Useful to automaze the \n\
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
  vpImage<unsigned char> I2(540,480);
  vpImage<unsigned char> I3(540,480);

  // We open a window using either X11, GTK or GDI.
#if defined VISP_HAVE_X11
  vpDisplayX display[3];
#elif defined VISP_HAVE_GDI
  vpDisplayGDI display[3];
#elif defined VISP_HAVE_GTK
  vpDisplayGTK display[3];
#elif defined VISP_HAVE_OPENCV
  vpDisplayOpenCV display[3];
#endif

  if (opt_display) {
    try{
      // Display size is automatically defined by the image (I) size
      display[0].init(I, 100, 100,"Points as control points") ;
      vpDisplay::display(I) ;
      vpDisplay::flush(I) ;
    }
    catch(...)
    {
      vpERROR_TRACE("Error while displaying the image") ;
      exit(-1);
    }
  }

  vpNurbs Nurbs;
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
  vpList<double> weights;
  vpImagePoint pt;
  pt.set_ij(50,300);
  controlPoints.addRight(pt);
  weights.addRight(1);
  pt.set_ij(100,130);
  controlPoints.addRight(pt);
  weights.addRight(5);
  pt.set_ij(150,400);
  controlPoints.addRight(pt);
  weights.addRight(0.2);
  pt.set_ij(200,370);
  controlPoints.addRight(pt);
  weights.addRight(10);
  pt.set_ij(250,120);
  controlPoints.addRight(pt);
  weights.addRight(1);
  pt.set_ij(300,250);
  controlPoints.addRight(pt);
  weights.addRight(2);
  pt.set_ij(350,200);
  controlPoints.addRight(pt);
  weights.addRight(3);
  pt.set_ij(400,300);
  controlPoints.addRight(pt);
  weights.addRight(1);
  
  Nurbs.set_p(2);
  Nurbs.set_knots(knots);
  Nurbs.set_controlPoints(controlPoints);
  Nurbs.set_weights(weights);

  std::cout << "The parameters are :" <<std::endl;
  std::cout << "p : " << Nurbs.get_p() <<std::endl;
  std::cout << "" <<std::endl;
  std::cout << "The knot vector :" <<std::endl;
  (Nurbs.get_knots()).display();
  std::cout << "The control points are :" <<std::endl;
  (Nurbs.get_controlPoints()).display();
  std::cout << "The associated weights are :" <<std::endl;
  (Nurbs.get_controlPoints()).display();

  unsigned int i = Nurbs.findSpan(5/2.0);
  std::cout << "The knot interval number for the value u = 5/2 is : " << i <<std::endl;
 
  vpBasisFunction *N = NULL;
  N = Nurbs.computeBasisFuns(5/2.0);
  std::cout << "The nonvanishing basis functions N(u=5/2) are :" << std::endl; 
  for (unsigned int j = 0; j < Nurbs.get_p()+1; j++)
    std::cout << N[j].value << std::endl;
  
  vpBasisFunction **N2 = NULL;
  N2 = Nurbs.computeDersBasisFuns(5/2.0, 2);
  std::cout << "The first derivatives of the basis functions N'(u=5/2) are :" << std::endl;
  for (unsigned int j = 0; j < Nurbs.get_p()+1; j++)
    std::cout << N2[1][j].value << std::endl; 
  
  std::cout << "The second derivatives of the basis functions N''(u=5/2) are :" << std::endl;
  for (unsigned int j = 0; j < Nurbs.get_p()+1; j++)
    std::cout << N2[2][j].value << std::endl; 
  
  if (opt_display && opt_click_allowed)
  {
    double u = 0.0;
    vpImagePoint pt;
    while (u <= 5)
    {
      pt = Nurbs.computeCurvePoint(u);
      vpDisplay::displayCross(I,pt,4,vpColor::red);
      u+=0.01;
    }
    controlPoints.front();
    for (unsigned int j = 0; j < controlPoints.nbElements(); j++)
    {
      pt = controlPoints.value();
      vpDisplay::displayCross(I,pt,4,vpColor::green);
      controlPoints.next();
    }
    vpDisplay::flush(I) ;
    vpDisplay::getClick(I);
  }
  
  if (opt_display) {
    try{
      // Display size is automatically defined by the image (I) size
      display[1].init(I2, 100, 100,"Points interpolation") ;
      vpDisplay::display(I2) ;
      vpDisplay::flush(I2) ;
    }
    catch(...)
    {
      vpERROR_TRACE("Error while displaying the image") ;
      exit(-1);
    }
  }
  
  Nurbs.globalCurveInterp(controlPoints);
  
  if (opt_display && opt_click_allowed)
  {
    double u = 0.0;
    vpImagePoint pt;
    while (u <= 1)
    {
      pt = Nurbs.computeCurvePoint(u);
      vpDisplay::displayCross(I2,pt,4,vpColor::red);
      u+=0.01;
    }
    controlPoints.front();
    for (unsigned int j = 0; j < controlPoints.nbElements(); j++)
    {
      pt = controlPoints.value();
      vpDisplay::displayCross(I2,pt,4,vpColor::green);
      controlPoints.next();
    }
    vpDisplay::flush(I2) ;
    vpDisplay::getClick(I2);
  }
  
  
  if (opt_display) {
    try{
      // Display size is automatically defined by the image (I) size
      display[2].init(I3, 100, 100,"Points approximation") ;
      vpDisplay::display(I3) ;
      vpDisplay::flush(I3) ;
    }
    catch(...)
    {
      vpERROR_TRACE("Error while displaying the image") ;
      exit(-1);
    }
  }
  
  Nurbs.globalCurveApprox(controlPoints,5);
  
  if (opt_display && opt_click_allowed)
  {
    double u = 0.0;
    vpImagePoint pt;
    while (u <= 1)
    {
      pt = Nurbs.computeCurvePoint(u);
      vpDisplay::displayCross(I3,pt,4,vpColor::red);
      u+=0.01;
    }
    controlPoints.front();
    for (unsigned int j = 0; j < controlPoints.nbElements(); j++)
    {
      pt = controlPoints.value();
      vpDisplay::displayCross(I3,pt,4,vpColor::green);
      controlPoints.next();
    }
    vpDisplay::flush(I3) ;
    vpDisplay::getClick(I3);
  }
  
  if (N != NULL) delete[] N;
  if (N2 != NULL)
  {
    for (int j = 0; j <= 2; j++)
      delete[] N2[j];
    delete[] N2;
  }
  
  return 0;
}

#else
int main()
{
  std::cout << "This example requires a video device. " 
	    << std::endl
	    << "You should install X11, GTK, OpenCV, GDI or Direct3D" 
	    << std::endl
	    << "to be able to execute this example."
	    << std::endl;
  return 0;
}
#endif
