/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 * Interface with the image for feature display.
 *
*****************************************************************************/

#include <visp3/core/vpFeatureDisplay.h>

// Meter/pixel conversion
#include <visp3/core/vpMeterPixelConversion.h>

// display
#include <visp3/core/vpDisplay.h>

// math
#include <visp3/core/vpMath.h>

#include <visp3/core/vpImagePoint.h>

BEGIN_VISP_NAMESPACE
/*!
  Display a 2D point with coordinates (x, y) expressed in the image plane.
  These coordinates are obtained after perspective projection of the point.

  \param x, y : Point coordinates in the image plane.
  \param cam : Camera intrinsic parameters.
  \param I : Image.

  \param color : Color to use to display the feature.
  \param thickness : Thickness of the feature representation.
*/
void vpFeatureDisplay::displayPoint(double x, double y, const vpCameraParameters &cam, const vpImage<unsigned char> &I,
                                    const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip; // pixel coordinates in float
  vpMeterPixelConversion::convertPoint(cam, x, y, ip);
  vpDisplay::displayCross(I, ip, 15, color, thickness);
}

/*!
  Display a 2D line with coordinates \f$(\rho, \theta)\f$ expressed in the image plane.
  These coordinates are obtained after perspective projection of the line.

  \param rho, theta : Line parameters \f$(\rho, \theta)\f$ expressed in the image plane.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature.
  \param thickness : Thickness of the feature representation.
*/
void vpFeatureDisplay::displayLine(double rho, double theta, const vpCameraParameters &cam,
                                   const vpImage<unsigned char> &I, const vpColor &color, unsigned int thickness)
{
  //  --comment: x times cos(theta) plus y times sin(theta) minus rho equals 0
  double rhop, thetap;
  vpMeterPixelConversion::convertLine(cam, rho, theta, rhop, thetap);

  //  --comment: u times cos thetap plus v times sin thetap minus rhop equals 0
  double co = cos(thetap);
  double si = sin(thetap);
  double c = -rhop;

  double a = si;
  double b = co;
  vpImagePoint ip1, ip2;

  if (fabs(a) < fabs(b)) {
    ip1.set_ij(0, (-c) / b);
    double h = I.getHeight() - 1;
    ip2.set_ij(h, (-c - (a * h)) / b);
    vpDisplay::displayLine(I, ip1, ip2, color, thickness);
  }
  else {
    ip1.set_ij((-c) / a, 0);
    double w = I.getWidth() - 1;
    ip2.set_ij((-c - (b * w)) / a, w);
    vpDisplay::displayLine(I, ip1, ip2, color, thickness);
  }
}

/*!
  Display cylinder limbs as two 2D lines with coordinates \f$(\rho, \theta)\f$ expressed in the image plane.
  These coordinates are obtained after perspective projection of the cylinder.

  \param rho1, theta1 : Cylinder first limb parameters \f$(\rho, \theta)\f$ expressed in the image plane.
  \param rho2, theta2 : Cylinder second limb parameters \f$(\rho, \theta)\f$ expressed in the image plane.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature.
  \param thickness : Thickness of the feature representation.
*/
void vpFeatureDisplay::displayCylinder(double rho1, double theta1, double rho2, double theta2,
                                       const vpCameraParameters &cam, const vpImage<unsigned char> &I,
                                       const vpColor &color, unsigned int thickness)
{
  displayLine(rho1, theta1, cam, I, color, thickness);
  displayLine(rho2, theta2, cam, I, color, thickness);
}

/*!
  Display an ellipse with parameters \f$(x, y, n_{20}, n_{11}, n_{02})\f$ expressed in the image plane.
  These parameters are obtained after perspective projection of a 3D circle (see vpCircle) or a sphere (see vpSphere).

  \param x, y, n20, n11, n02 : Ellipse parameters where:
  - \f$(x,y)\f$ are the normalized coordinates of the ellipse center,
  respectively along the horizontal and vertical axis in the image plane.
  - \f$n_{20}, n_{11}, n_{02}\f$ are the second order centered moments of the ellipse normalized
  by its area (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where
  \f$\mu_{ij}\f$ are the centered moments and a the area) expressed in the image plane.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature.
  \param thickness : Thickness of the feature representation.

  \sa vpDisplay::displayEllipse()
*/
void vpFeatureDisplay::displayEllipse(double x, double y, double n20, double n11, double n02,
                                      const vpCameraParameters &cam, const vpImage<unsigned char> &I,
                                      const vpColor &color, unsigned int thickness)
{
  vpImagePoint center;
  double n20_p, n11_p, n02_p;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  vpCircle circle;
  circle.p[index_0] = x;
  circle.p[index_1] = y;
  circle.p[index_2] = n20;
  circle.p[index_3] = n11;
  circle.p[index_4] = n02;

  vpMeterPixelConversion::convertEllipse(cam, circle, center, n20_p, n11_p, n02_p);
  vpDisplay::displayEllipse(I, center, n20_p, n11_p, n02_p, true, color, thickness);
}

/*!
  Display a 2D point with coordinates (x, y) expressed in the image plane.
  These coordinates are obtained after perspective projection of the point.

  \param x, y : Point coordinates in the image plane.
  \param cam : Camera intrinsic parameters.
  \param I : Image.

  \param color : Color to use to display the feature.
  \param thickness : Thickness of the feature representation.
*/
void vpFeatureDisplay::displayPoint(double x, double y, const vpCameraParameters &cam, const vpImage<vpRGBa> &I,
                                    const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip; // pixel coordinates in float
  vpMeterPixelConversion::convertPoint(cam, x, y, ip);

  vpDisplay::displayCross(I, ip, 15, color, thickness);
}

/*!
  Display a 2D line with coordinates \f$(\rho, \theta)\f$ expressed in the image plane.
  These coordinates are obtained after perspective projection of the line.

  \param rho, theta : Line parameters \f$(\rho, \theta)\f$ expressed in the image plane.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature.
  \param thickness : Thickness of the feature representation.
*/
void vpFeatureDisplay::displayLine(double rho, double theta, const vpCameraParameters &cam, const vpImage<vpRGBa> &I,
                                   const vpColor &color, unsigned int thickness)
{
  // --comment: x times cos of theta plus y times sin of theta minus rho equals 0
  double rhop, thetap;
  vpMeterPixelConversion::convertLine(cam, rho, theta, rhop, thetap);

  // --comment: u times cos of thetap plus v times sin of thetap minus rhop equals 0
  double co = cos(thetap);
  double si = sin(thetap);
  double c = -rhop;

  double a = si;
  double b = co;
  vpImagePoint ip1, ip2;

  if (fabs(a) < fabs(b)) {
    ip1.set_ij(0, (-c) / b);
    double h = I.getHeight() - 1;
    ip2.set_ij(h, (-c - (a * h)) / b);
    vpDisplay::displayLine(I, ip1, ip2, color, thickness);
  }
  else {
    ip1.set_ij((-c) / a, 0);
    double w = I.getWidth() - 1;
    ip2.set_ij((-c - (b * w)) / a, w);
    vpDisplay::displayLine(I, ip1, ip2, color, thickness);
  }
}

/*!
  Display cylinder limbs as two 2D lines with coordinates \f$(\rho, \theta)\f$ expressed in the image plane.
  These coordinates are obtained after perspective projection of the cylinder.

  \param rho1, theta1 : Cylinder first limb parameters \f$(\rho, \theta)\f$ expressed in the image plane.
  \param rho2, theta2 : Cylinder second limb parameters \f$(\rho, \theta)\f$ expressed in the image plane.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature.
  \param thickness : Thickness of the feature representation.
*/
void vpFeatureDisplay::displayCylinder(double rho1, double theta1, double rho2, double theta2,
                                       const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color,
                                       unsigned int thickness)
{
  displayLine(rho1, theta1, cam, I, color, thickness);
  displayLine(rho2, theta2, cam, I, color, thickness);
}

/*!
  Display an ellipse with parameters \f$(x, y, n_{20}, n_{11}, n_{02})\f$ expressed in the image plane.
  These parameters are obtained after perspective projection of a 3D circle (see vpCircle) or a sphere (see vpSphere).

  \param x, y, n20, n11, n02 : Ellipse parameters where:
  - \f$(x,y)\f$ are the normalized coordinates of the ellipse center,
  respectively along the horizontal and vertical axis in the image plane.
  - \f$n_{20}, n_{11}, n_{02}\f$ are the second order centered moments of the ellipse normalized
  by its area (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where
  \f$\mu_{ij}\f$ are the centered moments and a the area) expressed in the image plane.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature.
  \param thickness : Thickness of the feature representation.

  \sa vpDisplay::displayEllipse()
*/
void vpFeatureDisplay::displayEllipse(double x, double y, double n20, double n11, double n02,
                                      const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color,
                                      unsigned int thickness)
{
  vpImagePoint center;
  double n20_p, n11_p, n02_p;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  vpCircle circle;
  circle.p[index_0] = x;
  circle.p[index_1] = y;
  circle.p[index_2] = n20;
  circle.p[index_3] = n11;
  circle.p[index_4] = n02;

  vpMeterPixelConversion::convertEllipse(cam, circle, center, n20_p, n11_p, n02_p);
  vpDisplay::displayEllipse(I, center, n20_p, n11_p, n02_p, true, color, thickness);
}
END_VISP_NAMESPACE
