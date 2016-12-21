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

#include <visp3/core/vpMath.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpCameraParameters.h>
#include <visp3/core/vpMeterPixelConversion.h>
#include <visp3/core/vpPixelMeterConversion.h>
#include <visp3/core/vpMath.h>
#include <stdlib.h>
#include <stdio.h>

int main()
{
  try {
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
  catch(vpException &e) {
    std::cout << "Catch an exception: " << e << std::endl;
    return 1;
  }
}
