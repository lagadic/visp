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
 * Visual feature circle.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp3/core/vpCircle.h>

#include <visp3/core/vpFeatureDisplay.h>

void vpCircle::init()
{
  oP.resize(7);
  cP.resize(7);

  p.resize(5);
}

/*!
  Set the parameters of the 3D circle in the object frame.

  \param oP_ : This 7-dim vector defines the parameters oP[0], oP[1], oP[2] corresponding
  to parameters oA, oB, oC of the plane with equation oA*(x-oX)+oB*(y-oY)+oC*(z-oZ)=0
  passing through the 3D sphere center.  oP[3], oP[4], oP[5] correspond to oX, oY, oZ the
  coordinates of the center of the sphere. oP[6] corresponds to the radius of
  the sphere.
*/
void vpCircle::setWorldCoordinates(const vpColVector &oP_) { this->oP = oP_; }

/*!
  Set the 3D circle coordinates in the object frame.

  \param oA, oB, oC : Parameters of the plane with equation oA*(x-oX)+oB*(y-oY)+oC*(z-oZ)=0
  passing through the 3D sphere center.
  \param oX : Coordinate of the center of the sphere along X-axis in the object frame.
  \param oY : Coordinate of the center of the sphere along Y-axis in the object frame.
  \param oZ : Coordinate of the center of the sphere along Z-axis in the object frame.
  \param R : Radius of the sphere.
*/
void vpCircle::setWorldCoordinates(double oA, double oB, double oC, double oX, double oY, double oZ, double R)
{
  oP[0] = oA;
  oP[1] = oB;
  oP[2] = oC;
  oP[3] = oX;
  oP[4] = oY;
  oP[5] = oZ;
  oP[6] = R;
}

/*!
 * Default constructor that initialize internal vectors.
 */
vpCircle::vpCircle() { init(); }

/*!
  Construct the circle from the intersection of a plane and a sphere.

  \param oP_ : This 7-dim vector defines the parameters oP[0], oP[1], oP[2] corresponding
  to parameters oA, oB, oC of the plane with equation oA*(x-oX)+oB*(y-oY)+oC*(z-oZ)=0
  passing through the 3D sphere center.  oP[3], oP[4], oP[5] correspond to oX, oY, oZ the
  coordinates of the center of the sphere. oP[6] corresponds to the radius of
  the sphere.

  \sa setWorldCoordinates()
*/
vpCircle::vpCircle(const vpColVector &oP_)
{
  init();
  setWorldCoordinates(oP_);
}

/*!
  Construct the 3D circle from the intersection of a plane and a sphere with
  coordinates expressed in the object frame.

  \param oA, oB, oC : Parameters of the plane with equation oA*(x-oX)+oB*(y-oY)+oC*(z-oZ)=0
  passing through the 3D sphere center.
  \param oX : Coordinate of the center of the sphere along X-axis in the object frame.
  \param oY : Coordinate of the center of the sphere along Y-axis in the object frame.
  \param oZ : Coordinate of the center of the sphere along Z-axis in the object frame.
  \param R : Radius of the sphere.

  \sa setWorldCoordinates()
*/
vpCircle::vpCircle(double oA, double oB, double oC, double oX, double oY, double oZ, double R)
{
  init();
  setWorldCoordinates(oA, oB, oC, oX, oY, oZ, R);
}

/*!
 * Default destructor that does nothing.
 */
vpCircle::~vpCircle() {}

/*!
  Perspective projection of the circle.

  From the 3D parameters of the circle in the camera frame available in cP,
  computes the 2D parameters of the ellipse resulting from the perspective
  projection in the image plane. Those 2D parameters are available in p
  vector.

  See vpCircle::projection(const vpColVector &, vpColVector &) for a more
  detailed description of the parameters.
  */
void vpCircle::projection() { projection(cP, p); }

/*!
  Perspective projection of the circle.
  \param cP_: 3D cercle input parameters expressed in the camera frame.
  This 7-dim vector contains the following parameters: cA, cB, cC, cX, cY, cZ, R where
  - cA, cB, cC are the parameters of the plane with equation cA*(x-cX)+cB*(y-cY)+cC*(z-cZ)=0
  passing through the 3D sphere center.
  - cX, cY, cZ are the 3D coordinates of the circle in the camera frame
  - R is the circle radius in [m].

  \param p_: 2D circle output parameters. This is a 5 dimension vector. It
  contains the following parameters: x, y, m20, m11, m02 where:
  - x, y are the normalized coordinates of the center of the ellipse (ie the
  perspective projection of a 3D circle becomes a 2D ellipse in the image) in
  the image plane.
  - mu20, mu11, mu02 are the second order centered moments of the ellipse.
  */
void vpCircle::projection(const vpColVector &cP_, vpColVector &p_) const
{
  p_.resize(5, false);

  vpColVector K(6);
  {
    double A = cP_[0];
    double B = cP_[1];
    double C = cP_[2];

    double X0 = cP_[3];
    double Y0 = cP_[4];
    double Z0 = cP_[5];

    double r = cP_[6];

    // projection
    double s = X0 * X0 + Y0 * Y0 + Z0 * Z0 - r * r;
    double det = A * X0 + B * Y0 + C * Z0;
    A = A / det;
    B = B / det;
    C = C / det;

    K[0] = 1 - 2 * A * X0 + A * A * s;
    K[1] = 1 - 2 * B * Y0 + B * B * s;
    K[2] = -A * Y0 - B * X0 + A * B * s;
    K[3] = -C * X0 - A * Z0 + A * C * s;
    K[4] = -C * Y0 - B * Z0 + B * C * s;
    K[5] = 1 - 2 * C * Z0 + C * C * s;
  }

  double det = K[2] * K[2] - K[0] * K[1];
  if (fabs(det) < 1e-8) {
    throw(vpException(vpException::divideByZeroError, "division par 0"));
  }

  double xc = (K[1] * K[3] - K[2] * K[4]) / det;
  double yc = (K[0] * K[4] - K[2] * K[3]) / det;

  double c = sqrt((K[0] - K[1]) * (K[0] - K[1]) + 4 * K[2] * K[2]);
  double s = 2 * (K[0] * xc * xc + 2 * K[2] * xc * yc + K[1] * yc * yc - K[5]);

  double A, B, E;

  if (fabs(K[2]) < std::numeric_limits<double>::epsilon()) {
    E = 0.0;
    if (K[0] > K[1]) {
      A = sqrt(s / (K[0] + K[1] + c));
      B = sqrt(s / (K[0] + K[1] - c));
    } else {
      A = sqrt(s / (K[0] + K[1] - c));
      B = sqrt(s / (K[0] + K[1] + c));
    }
  } else {
    E = (K[1] - K[0] + c) / (2 * K[2]);
    if (fabs(E) > 1.0) {
      A = sqrt(s / (K[0] + K[1] + c));
      B = sqrt(s / (K[0] + K[1] - c));
    } else {
      A = sqrt(s / (K[0] + K[1] - c));
      B = sqrt(s / (K[0] + K[1] + c));
      E = -1.0 / E;
    }
  }

  det = (1.0 + vpMath::sqr(E));
  double mu20 = (vpMath::sqr(A) + vpMath::sqr(B * E)) / det;
  double mu11 = (vpMath::sqr(A) - vpMath::sqr(B)) * E / det;
  double mu02 = (vpMath::sqr(B) + vpMath::sqr(A * E)) / det;

  p_[0] = xc;
  p_[1] = yc;
  p_[2] = mu20;
  p_[3] = mu11;
  p_[4] = mu02;
}

/*!
  From the 3D coordinates of the circle in the object frame oP that are set using for
  example vpCircle(double oA, double oB, double oC, double oX, double oY, double oZ, double R)
  or setWorldCoordinates(), compute the 3D coordinates of the circle in the camera frame cP = cMo * oP.

  \param cMo : Transformation from camera to object frame.
  \param cP_ : 3D normalized coordinates of the circle in the camera frame cP = (cA, cB, cC, cX, cY, cZ, R).
*/void vpCircle::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP_) const
{
  cP_.resize(7, false);

  double A, B, C;
  A = cMo[0][0] * oP[0] + cMo[0][1] * oP[1] + cMo[0][2] * oP[2];
  B = cMo[1][0] * oP[0] + cMo[1][1] * oP[1] + cMo[1][2] * oP[2];
  C = cMo[2][0] * oP[0] + cMo[2][1] * oP[1] + cMo[2][2] * oP[2];

  double X0, Y0, Z0;
  X0 = cMo[0][3] + cMo[0][0] * oP[3] + cMo[0][1] * oP[4] + cMo[0][2] * oP[5];
  Y0 = cMo[1][3] + cMo[1][0] * oP[3] + cMo[1][1] * oP[4] + cMo[1][2] * oP[5];
  Z0 = cMo[2][3] + cMo[2][0] * oP[3] + cMo[2][1] * oP[4] + cMo[2][2] * oP[5];
  double R = oP[6];

  cP_[0] = A;
  cP_[1] = B;
  cP_[2] = C;

  cP_[3] = X0;
  cP_[4] = Y0;
  cP_[5] = Z0;

  cP_[6] = R;
}

/*!
 * Perspective projection of the circle.
 * Internal circle parameters are modified in cP.
 *
 * \param cMo : Homogeneous transformation from camera frame to object frame.
 */
void vpCircle::changeFrame(const vpHomogeneousMatrix &cMo)
{
  double A, B, C;
  A = cMo[0][0] * oP[0] + cMo[0][1] * oP[1] + cMo[0][2] * oP[2];
  B = cMo[1][0] * oP[0] + cMo[1][1] * oP[1] + cMo[1][2] * oP[2];
  C = cMo[2][0] * oP[0] + cMo[2][1] * oP[1] + cMo[2][2] * oP[2];

  double X0, Y0, Z0;
  X0 = cMo[0][3] + cMo[0][0] * oP[3] + cMo[0][1] * oP[4] + cMo[0][2] * oP[5];
  Y0 = cMo[1][3] + cMo[1][0] * oP[3] + cMo[1][1] * oP[4] + cMo[1][2] * oP[5];
  Z0 = cMo[2][3] + cMo[2][0] * oP[3] + cMo[2][1] * oP[4] + cMo[2][2] * oP[5];
  double R = oP[6];

  cP[0] = A;
  cP[1] = B;
  cP[2] = C;

  cP[3] = X0;
  cP[4] = Y0;
  cP[5] = Z0;

  cP[6] = R;
}

/*!
 * Display the projection of a 3D circle in image \e I.
 *
 * \param I : Image used as background.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the circle.
 * \param thickness : Thickness used to draw the circle.
 */
void vpCircle::display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color,
                       unsigned int thickness)
{
  vpFeatureDisplay::displayEllipse(p[0], p[1], p[2], p[3], p[4], cam, I, color, thickness);
}

/*!
 * Display the projection of a sphere in image \e I.
 * This method is non destructive wrt. cP and p internal circle parameters.
 *
 * \param I : Image used as background.
 * \param cMo : Homogeneous transformation from camera frame to object frame.
 * The circle is considered as viewed from this camera position.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the sphere.
 * \param thickness : Thickness used to draw the circle.
 */
void vpCircle::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &color, unsigned int thickness)
{
  vpColVector _cP, _p;
  changeFrame(cMo, _cP);
  projection(_cP, _p);
  vpFeatureDisplay::displayEllipse(_p[0], _p[1], _p[2], _p[3], _p[4], cam, I, color, thickness);
}
//! for memory issue (used by the vpServo class only)
vpCircle *vpCircle::duplicate() const
{
  vpCircle *feature = new vpCircle(*this);
  return feature;
}

/*!
  Computes the coordinates of the point corresponding to the intersection
  between a circle and a line.

  \warning This functions assumes changeFrame() and projection() have already
  been called.

  \sa changeFrame(), projection()

  \param circle : Circle to consider for the intersection.
  \param cam : Camera parameters that have to be used for the intersection computation.
  \param rho : The rho parameter of the line.
  \param theta : The theta parameter of the line.
  \param i : resulting i-coordinate of the intersection point.
  \param j : resulting j-coordinate of the intersection
  point.
*/
void vpCircle::computeIntersectionPoint(const vpCircle &circle, const vpCameraParameters &cam, const double &rho,
                                        const double &theta, double &i, double &j)
{
  // This was taken from the code of art-v1. (from the artCylinder class)
  double px = cam.get_px();
  double py = cam.get_py();
  double u0 = cam.get_u0();
  double v0 = cam.get_v0();

  double mu11 = circle.p[3];
  double mu02 = circle.p[4];
  double mu20 = circle.p[2];
  double Xg = u0 + circle.p[0] * px;
  double Yg = v0 + circle.p[1] * py;

  // Find Intersection between line and ellipse in the image.

  // Optimised calculation for X
  double stheta = sin(theta);
  double ctheta = cos(theta);
  double sctheta = stheta * ctheta;
  double m11yg = mu11 * Yg;
  double ctheta2 = vpMath::sqr(ctheta);
  double m02xg = mu02 * Xg;
  double m11stheta = mu11 * stheta;
  j = ((mu11 * Xg * sctheta - mu20 * Yg * sctheta + mu20 * rho * ctheta - m11yg + m11yg * ctheta2 + m02xg -
        m02xg * ctheta2 + m11stheta * rho) /
       (mu20 * ctheta2 + 2.0 * m11stheta * ctheta + mu02 - mu02 * ctheta2));
  // Optimised calculation for Y
  double rhom02 = rho * mu02;
  double sctheta2 = stheta * ctheta2;
  double ctheta3 = ctheta2 * ctheta;
  i = (-(-rho * mu11 * stheta * ctheta - rhom02 + rhom02 * ctheta2 + mu11 * Xg * sctheta2 - mu20 * Yg * sctheta2 -
         ctheta * mu11 * Yg + ctheta3 * mu11 * Yg + ctheta * mu02 * Xg - ctheta3 * mu02 * Xg) /
       (mu20 * ctheta2 + 2.0 * mu11 * stheta * ctheta + mu02 - mu02 * ctheta2) / stheta);
}
