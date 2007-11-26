/****************************************************************************
 *
 * $Id: testCameraParametersConversion.cpp,v 1.1 2007-11-26 16:55:29 asaunier Exp $
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
  \file testPoint.cpp
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
  px = 1657.4686029633;
  py = 1658.8721388153;
  u0 = 322.1349473852;
  v0 = 231.1254084448;
  double px_mp,py_mp,u0_mp,v0_mp,kd_mp;
  px_mp = 1625.1177398878;
  py_mp = 1625.5769959452;
  u0_mp = 323.8980532066;
  v0_mp = 245.4230738551;
  kd_mp = -0.1721425305;
  double px_pm,py_pm,u0_pm,v0_pm,kd_pm;
  px_pm = 1625.2140904668;
  py_pm = 1625.6703202345;
  u0_pm = 323.8538688646;
  v0_pm = 245.3074694831;
  kd_pm = 0.1763145408;
  
  cam.init(px,py,u0,v0);
  cam.init_mp(px_mp,py_mp,u0_mp,v0_mp,kd_mp);
  cam.init_pm(px_pm,py_pm,u0_pm,v0_pm,kd_pm);

  double u1 = 320;
  double v1 = 240;
  double x1,y1;
  double u2,v2;
  vpPixelMeterConversion::convertPoint(cam,u1,v1,x1,y1,false);
  vpMeterPixelConversion::convertPoint(cam,x1,y1,u2,v2,false);
  if(!vpMath::equal(u1,u2) || !vpMath::equal(v1,v2)){
    vpTRACE("Error in convertPoint without distortion");
    return -1;
  }

  vpPixelMeterConversion::convertPoint(cam,u1,v1,x1,y1,true);
  vpMeterPixelConversion::convertPoint(cam,x1,y1,u2,v2,true);
  if(!vpMath::equal(u1,u2,0.2) || !vpMath::equal(v1,v2,0.2)){
    vpTRACE("Error in convertPoint with distortion :\n"
            "u1 = %f, u2 = %f\n"
            "v1 = %f, v2 = %f\n",u1,u2,v1,v2);
    return -1;
  }
  return 0;
}
