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
 * Interface with the image for feature display.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <visp3/core/vpFeatureDisplay.h>

// Meter/pixel conversion
#include <visp3/core/vpMeterPixelConversion.h>

// display
#include <visp3/core/vpDisplay.h>

// Debug trace
#include <visp3/core/vpDebug.h>

// math
#include <visp3/core/vpMath.h>

#include <visp3/core/vpImagePoint.h>

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
  //    x cos(theta) + y sin(theta) - rho = 0
  double rhop, thetap;
  vpMeterPixelConversion::convertLine(cam, rho, theta, rhop, thetap);

  //    u cos(thetap) + v sin(thetap) - rhop = 0
  double co = cos(thetap);
  double si = sin(thetap);
  double c = -rhop;

  double a = si;
  double b = co;
  vpImagePoint ip1, ip2;

  if (fabs(a) < fabs(b)) {
    ip1.set_ij(0, (-c) / b);
    double h = I.getHeight() - 1;
    ip2.set_ij(h, (-c - a * h) / b);
    vpDisplay::displayLine(I, ip1, ip2, color, thickness);
  } else {
    ip1.set_ij((-c) / a, 0);
    double w = I.getWidth() - 1;
    ip2.set_ij((-c - b * w) / a, w);
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
  Display an ellipse with parameters \f$(x, y, \mu_{20}, \mu_{11}, \mu_{02})\f$ expressed in the image plane.
  These parameters are obtained after perspective projection of a 3D circle (see vpCircle) or a sphere (see vpSphere).

  \param x, y, mu20, mu11, mu02 : Ellipse parameters where:
  - \f$(x,y)\f$ are the normalized coordinates of the ellipse center,
  respectively along the horizontal and vertical axis in the image plane.
  - \f$\mu_{20}, \mu_{11}, \mu_{02}\f$ are the centered moments expressed in the image plane.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature.
  \param thickness : Thickness of the feature representation.

  \sa vpDisplay::displayEllipse()
*/
void vpFeatureDisplay::displayEllipse(double x, double y, double mu20, double mu11, double mu02,
                                      const vpCameraParameters &cam, const vpImage<unsigned char> &I,
                                      const vpColor &color, unsigned int thickness)
{
  vpImagePoint center;
  double mu20_p, mu11_p, mu02_p;
  vpCircle circle;
  circle.p[0] = x;
  circle.p[1] = y;
  circle.p[2] = mu20;
  circle.p[3] = mu11;
  circle.p[4] = mu02;

  vpMeterPixelConversion::convertEllipse(cam, circle, center, mu20_p, mu11_p, mu02_p);
  vpDisplay::displayEllipse(I, center, mu20_p, mu11_p, mu02_p, true, color, thickness);
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
  //    x cos(theta) + y sin(theta) - rho = 0
  double rhop, thetap;
  vpMeterPixelConversion::convertLine(cam, rho, theta, rhop, thetap);

  //    u cos(thetap) + v sin(thetap) - rhop = 0
  double co = cos(thetap);
  double si = sin(thetap);
  double c = -rhop;

  double a = si;
  double b = co;
  vpImagePoint ip1, ip2;

  if (fabs(a) < fabs(b)) {
    ip1.set_ij(0, (-c) / b);
    double h = I.getHeight() - 1;
    ip2.set_ij(h, (-c - a * h) / b);
    vpDisplay::displayLine(I, ip1, ip2, color, thickness);
  } else {
    ip1.set_ij((-c) / a, 0);
    double w = I.getWidth() - 1;
    ip2.set_ij((-c - b * w) / a, w);
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
  Display an ellipse with parameters \f$(x, y, \mu_{20}, \mu_{11}, \mu_{02})\f$ expressed in the image plane.
  These parameters are obtained after perspective projection of a 3D circle (see vpCircle) or a sphere (see vpSphere).

  \param x, y, mu20, mu11, mu02 : Ellipse parameters where:
  - \f$(x,y)\f$ are the normalized coordinates of the ellipse center,
  respectively along the horizontal and vertical axis in the image plane.
  - \f$\mu_{20}, \mu_{11}, \mu_{02}\f$ are the centered moments expressed in the image plane.
  \param cam : Camera intrinsic parameters.
  \param I : Image.
  \param color : Color to use to display the feature.
  \param thickness : Thickness of the feature representation.

  \sa vpDisplay::displayEllipse()
*/
void vpFeatureDisplay::displayEllipse(double x, double y, double mu20, double mu11, double mu02,
                                      const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color,
                                      unsigned int thickness)
{
  vpImagePoint center;
  double mu20_p, mu11_p, mu02_p;
  vpCircle circle;
  circle.p[0] = x;
  circle.p[1] = y;
  circle.p[2] = mu20;
  circle.p[3] = mu11;
  circle.p[4] = mu02;

  vpMeterPixelConversion::convertEllipse(cam, circle, center, mu20_p, mu11_p, mu02_p);
  vpDisplay::displayEllipse(I, center, mu20_p, mu11_p, mu02_p, true, color, thickness);
}
