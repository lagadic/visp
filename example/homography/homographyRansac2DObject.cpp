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
 * Example of the Ransac homography estimation algorithm.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file homographyRansac2DObject.cpp

  \brief Example of the Ransac homography estimation algorithm using
  vpHomography class.

*/

/*!
  \example homographyRansac2DObject.cpp

  Example of the Ransac homography estimation algorithm using vpHomography
  class.

*/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpThetaUVector.h>
#include <visp3/vision/vpHomography.h>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoint.h>

#include <stdlib.h>
#include <visp3/core/vpRansac.h>
#include <visp3/io/vpParseArgv.h>
// List of allowed command line options
#define GETOPTARGS "h"

void usage(const char *name, const char *badparam);
bool getOptions(int argc, const char **argv);

/*!

  Print the program options.

  \param name : Program name.
  \param badparam : Bad parameter name.

*/
void usage(const char *name, const char *badparam)
{
  fprintf(stdout, "\n\
Test the Ransac homography estimation algorithm.\n\
\n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -h\n\
     Print the help.\n");

  if (badparam) {
    fprintf(stderr, "ERROR: \n");
    fprintf(stderr, "\nBad parameter [%s]\n", badparam);
  }
}
/*!

  Set the program options.

  \param argc : Command line number of parameters.
  \param argv : Array of command line parameters.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, const char **argv)
{
  const char *optarg_;
  int c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg_)) > 1) {

    switch (c) {
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
    // Read the command line options
    if (getOptions(argc, argv) == false) {
      exit(-1);
    }

    double L = 0.1;
    unsigned int nbpt = 11;

    std::vector<vpPoint> P(nbpt); //  Point to be tracked
    std::vector<double> xa(nbpt), ya(nbpt), xb(nbpt), yb(nbpt);

    P[0].setWorldCoordinates(-L, -L, 0);    // inlier
    P[1].setWorldCoordinates(2 * L, -L, 0); // inlier
    P[2].setWorldCoordinates(L, L, 0);      // inlier
    P[3].setWorldCoordinates(-L, 3 * L, 0); // inlier
    P[4].setWorldCoordinates(0, 0, L);
    P[5].setWorldCoordinates(L, -2 * L, L);
    P[6].setWorldCoordinates(L, -4 * L, 2 * L);
    P[7].setWorldCoordinates(-2 * L, -L, -3 * L);
    P[8].setWorldCoordinates(-5 * L, -5 * L, 0); // inlier
    P[9].setWorldCoordinates(-2 * L, +3 * L, 4 * L);
    P[10].setWorldCoordinates(-2 * L, -0.5 * L, 0); // inlier

    std::vector<bool> inliers_ground_truth(nbpt, false);
    inliers_ground_truth[0] = true;
    inliers_ground_truth[1] = true;
    inliers_ground_truth[2] = true;
    inliers_ground_truth[3] = true;
    inliers_ground_truth[8] = true;
    inliers_ground_truth[10] = true;

    vpHomogeneousMatrix bMo(0, 0, 1, 0, 0, 0);
    vpHomogeneousMatrix aMb(0.1, 0.1, 0.1, vpMath::rad(10), 0, vpMath::rad(40));
    vpHomogeneousMatrix aMo = aMb * bMo;
    for (unsigned int i = 0; i < nbpt; i++) {
      P[i].project(aMo);
      xa[i] = P[i].get_x();
      ya[i] = P[i].get_y();
    }

    for (unsigned int i = 0; i < nbpt; i++) {
      P[i].project(bMo);
      xb[i] = P[i].get_x();
      yb[i] = P[i].get_y();
    }
    std::cout << "-------------------------------" << std::endl;

    vpRotationMatrix aRb;
    vpTranslationVector aTb;
    vpColVector n;
    std::cout << "Compare with built homography H = R + t/d n " << std::endl;
    vpPlane bp(0, 0, 1, 1);
    vpHomography aHb_built(aMb, bp);
    std::cout << "aHb built from the displacement: \n" << aHb_built / aHb_built[2][2] << std::endl;

    aHb_built.computeDisplacement(aRb, aTb, n);
    std::cout << "Rotation aRb: " << std::endl;
    std::cout << aRb << std::endl;
    std::cout << "Translation: aTb" << std::endl;
    std::cout << (aTb).t() << std::endl;
    std::cout << "Normal to the plane: n" << std::endl;
    std::cout << (n).t() << std::endl;

    std::cout << "-------------------------------" << std::endl;
    vpHomography aHb;
    std::vector<bool> inliers;
    double residual;
    // Suppose px=1000. Set the threshold to 2 pixels => 2/1000
    // In the data we have 6 inliers. We request that at least 6 are retrieved
    vpHomography::ransac(xb, yb, xa, ya, aHb, inliers, residual, 6, 2. / 1000);

    std::cout << "aHb estimated using ransac:\n" << aHb << std::endl;
    std::cout << "Inliers indexes (should be 0,1,2,3,8,10): ";
    for (unsigned int i = 0; i < inliers.size(); i++)
      if (inliers[i])
        std::cout << i << ",";
    std::cout << std::endl;

    if (inliers == inliers_ground_truth) {
      std::cout << "Ransac estimation succeed" << std::endl;
      return EXIT_SUCCESS;
    } else {
      std::cout << "Ransac estimation fails" << std::endl;
      return EXIT_FAILURE;
    }
  } catch (const vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return EXIT_FAILURE;
  }
}
