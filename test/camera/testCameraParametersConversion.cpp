/****************************************************************************
 *
 * $Id: testCameraParametersConversion.cpp,v 1.5 2008-02-01 15:11:40 fspindle Exp $
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
 * Performs various tests on the vpPixelMeterConversion and
 * vpPixelMeterConversion class.
 *
 * Authors:
 * Anthony saunier
 *
 *****************************************************************************/



/*!
  \file testCameraParametersConversion.cpp

  Performs various tests on the vpPixelMeterConversion and
  vpPixelMeterConversion class.
*/

// List of allowed command line options
#define GETOPTARGS	"h"

#include <visp/vpMath.h>
#include <visp/vpDebug.h>
#include <visp/vpParseArgv.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpMeterPixelConversion.h>
#include <visp/vpPixelMeterConversion.h>
#include <visp/vpMath.h>
/*!

  Print the program options.

*/
void usage(char *name, char *badparam)
{
  fprintf(stdout, "\n\
  Performs various tests on the vpPixelMeterConversion and\n\
  vpPixelMeterConversion class.\n\
\n\
SYNOPSIS\n\
  %s [-h]\n", name);

  fprintf(stdout, "\n\
OPTIONS:                                               Default\n\
  -h\n\
     Print the help.\n");

  if (badparam)
    fprintf(stdout, "\nERROR: Bad parameter [%s]\n", badparam);
}
/*!

  Set the program options.

  \return false if the program has to be stopped, true otherwise.

*/
bool getOptions(int argc, char **argv)
{
  char *optarg;
  int	c;
  while ((c = vpParseArgv::parse(argc, argv, GETOPTARGS, &optarg)) > 1) {

    switch (c) {
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
main(int argc, char ** argv)
{
  // Read the command line options
  if (getOptions(argc, argv) == false) {
    exit (-1);
  }

  vpCameraParameters cam;
  double px,py,u0,v0;
  px = 1657.429131;
  py = 1658.818598;
  u0 = 322.2437833;
  v0 = 230.8012737;
  vpCameraParameters camDist;
  double px_dist,py_dist,u0_dist,v0_dist,kud_dist,kdu_dist;
  px_dist = 1624.824731;
  py_dist = 1625.263641;
  u0_dist = 324.0923411;
  v0_dist = 245.2421388;
  kud_dist = -0.1741532338;
  kdu_dist = 0.1771165148;

  cam.initPersProjWithoutDistortion(px,py,u0,v0);
  camDist.initPersProjWithDistortion(px_dist,py_dist,u0_dist,v0_dist,
                                     kud_dist, kdu_dist);

  double u1 = 320;
  double v1 = 240;
  double x1 = 0, y1 = 0;
  double u2 = 0, v2 = 0;
  vpPixelMeterConversion::convertPoint(cam,u1,v1,x1,y1);
  vpMeterPixelConversion::convertPoint(cam,x1,y1,u2,v2);
  if(!vpMath::equal(u1,u2) || !vpMath::equal(v1,v2)){
    vpTRACE("Error in convertPoint without distortion:\n"
        "u1 = %f, u2 = %f\n"
        "v1 = %f, v2 = %f\n",u1,u2,v1,v2);
    return -1;
  }
  vpTRACE("convertPoint without distortion :\n"
      "u1 - u2 = %.20f\n"
      "v1 - v2 = %.20f\n",u1 - u2,v1 - v2);

  vpPixelMeterConversion::convertPoint(camDist,u1,v1,x1,y1);
  vpMeterPixelConversion::convertPoint(camDist,x1,y1,u2,v2);
  if(!vpMath::equal(u1,u2) || !vpMath::equal(v1,v2)){
    vpTRACE("Error in convertPoint with distortion :\n"
            "u1 = %f, u2 = %f\n"
            "v1 = %f, v2 = %f\n",u1,u2,v1,v2);
    return -1;
  }
  vpTRACE("convertPoint with distortion :\n"
      "u1 - u2 = %.20f\n"
      "v1 - v2 = %.20f\n",u1 - u2,v1 - v2);
  return 0;
}
