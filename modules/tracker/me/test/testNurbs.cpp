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
 * Exemple of a Nurbs curve.
 *
 * Authors:
 * Nicolas Melchior
 *
 *****************************************************************************/
/*!
  \example testNurbs.cpp

  \brief Describe a curve thanks to a Nurbs.
*/

#include <visp3/core/vpDebug.h>

#include <visp3/me/vpNurbs.h>

#include <visp3/core/vpImage.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/io/vpImageIo.h>
#ifdef VISP_HAVE_MODULE_GUI
#include <visp3/gui/vpDisplayD3D.h>
#include <visp3/gui/vpDisplayGDI.h>
#include <visp3/gui/vpDisplayGTK.h>
#include <visp3/gui/vpDisplayOpenCV.h>
#include <visp3/gui/vpDisplayX.h>
#endif

#include <cstdlib>
#include <visp3/core/vpIoTools.h>
#include <visp3/io/vpParseArgv.h>
#if defined(VISP_HAVE_DISPLAY)

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
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
    case 'c':
      click_allowed = false;
      break;
    case 'd':
      display = false;
      break;
    case 'h':
      usage(argv[0], NULL);
      return false;
      break;

    default:
      usage(argv[0], optarg_);
      return false;
      break;
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

int main(int argc, const char **argv)
{
  try {
    bool opt_click_allowed = true;
    bool opt_display = true;

    // Read the command line options
    if (getOptions(argc, argv, opt_click_allowed, opt_display) == false) {
      exit(-1);
    }

    // Declare an image, this is a gray level image (unsigned char)
    // it size is not defined yet, it will be defined when the image will
    // read on the disk
    vpImage<unsigned char> I(540, 480);
    vpImage<unsigned char> I2(540, 480);
    vpImage<unsigned char> I3(540, 480);

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
      // Display size is automatically defined by the image (I) size
      display[0].init(I, 100, 100, "Points as control points");
      vpDisplay::display(I);
      vpDisplay::flush(I);
    }

    vpNurbs Nurbs;
    std::list<double> knots;
    knots.push_back(0);
    knots.push_back(0);
    knots.push_back(0);
    knots.push_back(1);
    knots.push_back(2);
    knots.push_back(3);
    knots.push_back(4);
    knots.push_back(4);
    knots.push_back(5);
    knots.push_back(5);
    knots.push_back(5);

    std::list<vpImagePoint> controlPoints;
    std::list<double> weights;
    vpImagePoint pt;
    pt.set_ij(50, 300);
    controlPoints.push_back(pt);
    weights.push_back(1);
    pt.set_ij(100, 130);
    controlPoints.push_back(pt);
    weights.push_back(5);
    pt.set_ij(150, 400);
    controlPoints.push_back(pt);
    weights.push_back(0.2);
    pt.set_ij(200, 370);
    controlPoints.push_back(pt);
    weights.push_back(10);
    pt.set_ij(250, 120);
    controlPoints.push_back(pt);
    weights.push_back(1);
    pt.set_ij(300, 250);
    controlPoints.push_back(pt);
    weights.push_back(2);
    pt.set_ij(350, 200);
    controlPoints.push_back(pt);
    weights.push_back(3);
    pt.set_ij(400, 300);
    controlPoints.push_back(pt);
    weights.push_back(1);

    Nurbs.set_p(2);
    Nurbs.set_knots(knots);
    Nurbs.set_controlPoints(controlPoints);
    Nurbs.set_weights(weights);

    std::cout << "The parameters are :" << std::endl;
    std::cout << "p : " << Nurbs.get_p() << std::endl;
    std::cout << "" << std::endl;
    std::cout << "The knot vector :" << std::endl;
    std::list<double> knots_cur;
    Nurbs.get_knots(knots_cur);
    unsigned int i_display = 0;
    for (std::list<double>::const_iterator it = knots_cur.begin(); it != knots_cur.end(); ++it, ++i_display) {
      std::cout << i_display << " ---> " << *it << std::endl;
    }
    std::cout << "The control points are :" << std::endl;
    std::list<vpImagePoint> controlPoints_cur;
    Nurbs.get_controlPoints(controlPoints_cur);
    i_display = 0;
    for (std::list<vpImagePoint>::const_iterator it = controlPoints_cur.begin(); it != controlPoints_cur.end();
         ++it, ++i_display) {
      std::cout << i_display << " ---> " << *it << std::endl;
    }
    std::cout << "The associated weights are :" << std::endl;
    std::list<double> weights_cur;
    Nurbs.get_weights(weights_cur);
    i_display = 0;
    for (std::list<double>::const_iterator it = weights_cur.begin(); it != weights_cur.end(); ++it, ++i_display) {
      std::cout << i_display << " ---> " << *it << std::endl;
    }

    unsigned int i = Nurbs.findSpan(5 / 2.0);
    std::cout << "The knot interval number for the value u = 5/2 is : " << i << std::endl;

    vpBasisFunction *N = NULL;
    N = Nurbs.computeBasisFuns(5 / 2.0);
    std::cout << "The nonvanishing basis functions N(u=5/2) are :" << std::endl;
    for (unsigned int j = 0; j < Nurbs.get_p() + 1; j++)
      std::cout << N[j].value << std::endl;

    vpBasisFunction **N2 = NULL;
    N2 = Nurbs.computeDersBasisFuns(5 / 2.0, 2);
    std::cout << "The first derivatives of the basis functions N'(u=5/2) are :" << std::endl;
    for (unsigned int j = 0; j < Nurbs.get_p() + 1; j++)
      std::cout << N2[1][j].value << std::endl;

    std::cout << "The second derivatives of the basis functions N''(u=5/2) are :" << std::endl;
    for (unsigned int j = 0; j < Nurbs.get_p() + 1; j++)
      std::cout << N2[2][j].value << std::endl;

    if (opt_display && opt_click_allowed) {
      double u = 0.0;
      while (u <= 5) {
        pt = Nurbs.computeCurvePoint(u);
        vpDisplay::displayCross(I, pt, 4, vpColor::red);
        u += 0.01;
      }
      for (std::list<vpImagePoint>::const_iterator it = controlPoints.begin(); it != controlPoints.end(); ++it) {
        vpDisplay::displayCross(I, *it, 4, vpColor::green);
      }

      vpDisplay::flush(I);
      vpDisplay::getClick(I);
    }

    if (opt_display) {
      try {
        // Display size is automatically defined by the image (I) size
        display[1].init(I2, 100, 100, "Points interpolation");
        vpDisplay::display(I2);
        vpDisplay::flush(I2);
      } catch (...) {
        vpERROR_TRACE("Error while displaying the image");
        exit(-1);
      }
    }

    Nurbs.globalCurveInterp(controlPoints);

    if (opt_display && opt_click_allowed) {
      double u = 0.0;
      while (u <= 1) {
        pt = Nurbs.computeCurvePoint(u);
        vpDisplay::displayCross(I2, pt, 4, vpColor::red);
        u += 0.01;
      }

      for (std::list<vpImagePoint>::const_iterator it = controlPoints.begin(); it != controlPoints.end(); ++it) {
        vpDisplay::displayCross(I2, *it, 4, vpColor::green);
      }
      vpDisplay::flush(I2);
      vpDisplay::getClick(I2);
    }

    if (opt_display) {
      try {
        // Display size is automatically defined by the image (I) size
        display[2].init(I3, 100, 100, "Points approximation");
        vpDisplay::display(I3);
        vpDisplay::flush(I3);
      } catch (...) {
        vpERROR_TRACE("Error while displaying the image");
        exit(-1);
      }
    }

    Nurbs.globalCurveApprox(controlPoints, 5);

    if (opt_display && opt_click_allowed) {
      double u = 0.0;
      while (u <= 1) {
        pt = Nurbs.computeCurvePoint(u);
        vpDisplay::displayCross(I3, pt, 4, vpColor::red);
        u += 0.01;
      }

      for (std::list<vpImagePoint>::const_iterator it = controlPoints.begin(); it != controlPoints.end(); ++it) {
        vpDisplay::displayCross(I3, *it, 4, vpColor::green);
      }

      vpDisplay::flush(I3);
      vpDisplay::getClick(I3);
    }

    if (N != NULL)
      delete[] N;
    if (N2 != NULL) {
      for (int j = 0; j <= 2; j++)
        delete[] N2[j];
      delete[] N2;
    }

    return 0;
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}

#else
int main()
{
  std::cout << "This example requires a video device. " << std::endl
            << "You should install X11, GTK, OpenCV, GDI or Direct3D" << std::endl
            << "to be able to execute this example." << std::endl;
  return 0;
}
#endif
