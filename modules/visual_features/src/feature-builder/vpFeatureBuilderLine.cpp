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
 * Conversion between tracker and visual feature line.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpFeatureBuilderLine.cpp
  \brief  conversion between tracker
  and visual feature Line
*/

#include <visp3/core/vpMath.h>
#include <visp3/visual_features/vpFeatureBuilder.h>

/*!
  Initialize a line feature thanks to a vpLine.
  A vpFeatureLine contains the parameters \f$(\rho,\theta)\f$ which are
  expressed in meter. It also contains the parameters of a plan equation
  \f$(A,B,C,D)\f$. In vpLine there are the parameters of two plans, but the
  one which have the biggest D parameter is copied in the vpFeatureLine
  parameters.

  \param s : Visual feature to initialize.

  \param t : The vpLine used to create the vpFeatureLine.
*/
void vpFeatureBuilder::create(vpFeatureLine &s, const vpLine &t)
{
  try {
    double A, B, C, D;
    s.setRhoTheta(t.getRho(), t.getTheta());

    if (fabs(t.cP[3]) > fabs(t.cP[7])) // |D1| > |D2|
    {
      A = t.cP[0];
      B = t.cP[1];
      C = t.cP[2];
      D = t.cP[3];
    } else {
      A = t.cP[4];
      B = t.cP[5];
      C = t.cP[6];
      D = t.cP[7];
    }

    s.setABCD(A, B, C, D);

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!
  Initialize a line feature thanks to a vpCylinder.
  A vpFeatureLine contains the parameters \f$(\rho,\theta)\f$ which are
  expressed in meter. It also contains the parameters of a plan equation
  \f$(A,B,C,D)\f$. These parameters are computed thanks to the parameters that
  are contained in vpCylinder. It is possible to choose which edge of the
  cylinder to use to initialize the vpFeatureLine.

  \param s : Visual feature to initialize.

  \param t : The vpLine used to create the vpFeatureLine.

  \param line : The cylinder edge used to create the line feature.
  It can be vpCylinder::line1 or vpCylinder::line2.
*/
void vpFeatureBuilder::create(vpFeatureLine &s, const vpCylinder &t, const int line)
{
  try {

    double a = t.getA();
    double b = t.getB();
    double c = t.getC();

    double x0 = t.getX();
    double y0 = t.getY();
    double z0 = t.getZ();

    double R = t.getR();

    double D =
        vpMath::sqr(x0) + vpMath::sqr(y0) + vpMath::sqr(z0) - vpMath::sqr(R) - vpMath::sqr(a * x0 + b * y0 + c * z0);

    double alpha1 = (1 - a * a) * x0 - a * b * y0 - a * c * z0;
    double beta1 = -a * b * x0 + (1 - b * b) * y0 - b * c * z0;
    double gamma1 = -a * c * x0 - b * c * y0 + (1 - c * c) * z0;

    D *= -1;

    if (D < 0) {
      alpha1 *= -1;
      beta1 *= -1;
      gamma1 *= -1;
      D *= -1;
    }

    s.setABCD(alpha1, beta1, gamma1, D);

    if (line == vpCylinder::line1) {

      s.setRhoTheta(t.getRho1(), t.getTheta1());

    } else {

      s.setRhoTheta(t.getRho2(), t.getTheta2());
    }
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

#ifdef VISP_HAVE_MODULE_ME
/*!
  Initialize a line feature thanks to a vpMeLine and the parameters of the
  camera. A vpFeatureLine contains the parameters \f$(\rho,\theta)\f$ which
  are expressed in meter. In vpMeLine these parameters are given in pixel. The
  conversion is done thanks to the camera parameters.

  \warning vpFeatureLine also contains the parameters of a plan equation
  \f$(A,B,C,D)\f$. These parameters are needed to compute the interaction
  matrix but can not be computed thanks to a vpMeLine. You have to compute and
  set these parameters outside the function.

  \param s : Visual feature to initialize.

  \param cam : The parameters of the camera used to acquire the image
  containing the line.

  \param t : The vpLine used to create the vpFeatureLine.

  The code below shows how to initialize a vpFeatureLine visual
  feature. First, we initialize the \f$(\rho,\theta)\f$, and lastly we
  set the parameters of the plane which is generally the result
  of a pose estimation.

  \code
  vpImage<unsigned char> I; // Image container
  vpCameraParameters cam;   // Default intrinsic camera parameters
  vpMeLine line;            // Moving-edges line tracker

  vpFeatureLine s;    // Point feature
  ...
  // Tracking on the dot
  line.track(I);

  // Initialize rho,theta visual feature
  vpFeatureBuilder::create(s, cam, line);

  // A pose estimation is requested to initialize A, B, C and D the
  //parameters of the equation plan.
  double A = 1;
  double B = 1;
  double C = 1;
  double D = 1;
  ....
  s.setABCD(A,B,C,D);
  \endcode
*/
void vpFeatureBuilder::create(vpFeatureLine &s, const vpCameraParameters &cam, const vpMeLine &t)
{
  try {
    double rhop = t.getRho();
    double thetap = t.getTheta();
    double rho;
    double theta;

    // Gives the rho and theta coordinates in the (u,v) coordinate system.
    if (thetap >= 0 && thetap < M_PI / 2) {
      thetap = M_PI / 2 - thetap;
    }

    else if (thetap >= M_PI / 2 && thetap < 3 * M_PI / 2) {
      thetap = 3 * M_PI / 2 + M_PI - thetap;
    }

    else if (thetap >= 3 * M_PI / 2 && thetap <= 2 * M_PI) {
      thetap = M_PI / 2 + 2 * M_PI - thetap;
    }

    // while (thetap > M_PI/2)  { thetap -= M_PI ; rhop *= -1 ; }
    // while (thetap < -M_PI/2) { thetap += M_PI ; rhop *= -1 ; }

    //  vpTRACE("pixel %f %f",rhop, thetap) ;
    vpPixelMeterConversion::convertLine(cam, rhop, thetap, rho, theta);

    while (theta > M_PI) {
      theta -= 2 * M_PI;
    }
    while (theta < -M_PI) {
      theta += 2 * M_PI;
    }
    //   vpTRACE("meter %f %f",rho, theta) ;
    /*

    while(theta < -M_PI)	theta += 2*M_PI ;
    while(theta >= M_PI)	theta -= 2*M_PI ;

    // If theta is between -90 and -180 get the equivalent
    // between 0 and 90
    if(theta <-M_PI/2)
    {
      theta += M_PI ;
      rho *= -1 ;
    }
    // If theta is between 90 and 180 get the equivalent
    // between 0 and -90
    if(theta >M_PI/2)
    {
      theta -= M_PI ;
      rho *= -1 ;
    }
    */
    s.buildFrom(rho, theta);
    //   vpTRACE("meter %f %f",rho, theta) ;

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}
#endif //#ifdef VISP_HAVE_MODULE_ME
