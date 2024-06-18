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
 * Visual feature circle.
 *
*****************************************************************************/

#include <visp3/core/vpCircle.h>

#include <visp3/core/vpFeatureDisplay.h>

BEGIN_VISP_NAMESPACE
void vpCircle::init()
{
  oP.resize(7);
  cP.resize(7);

  p.resize(5);
}

/*!
  Set the parameters of the 3D circle in the object frame.

  \param oP_ : This 7-dim vector defines the parameters oP[0], oP[1], oP[2] corresponding
  to parameters oA, oB, oC of the plane with equation oA*(X-oX)+oB*(Y-oY)+oC*(Z-oZ)=0
  passing through the 3D sphere center.  oP[3], oP[4], oP[5] correspond to oX, oY, oZ the
  coordinates of the center of the sphere. oP[6] corresponds to the radius of
  the sphere.
*/
void vpCircle::setWorldCoordinates(const vpColVector &oP_) { this->oP = oP_; }

/*!
  Set the 3D circle coordinates in the object frame.

  \param oA, oB, oC : Parameters of the plane with equation oA*(X-oX)+oB*(Y-oY)+oC*(Z-oZ)=0
  passing through the 3D sphere center.
  \param oX : Coordinate of the center of the sphere along X-axis in the object frame.
  \param oY : Coordinate of the center of the sphere along Y-axis in the object frame.
  \param oZ : Coordinate of the center of the sphere along Z-axis in the object frame.
  \param R : Radius of the sphere.
*/
void vpCircle::setWorldCoordinates(double oA, double oB, double oC, double oX, double oY, double oZ, double R)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  const unsigned int index_6 = 6;
  oP[index_0] = oA;
  oP[index_1] = oB;
  oP[index_2] = oC;
  oP[index_3] = oX;
  oP[index_4] = oY;
  oP[index_5] = oZ;
  oP[index_6] = R;
}

/*!
 * Default constructor that initialize internal vectors.
 */
vpCircle::vpCircle() { init(); }

/*!
  Construct the circle from the intersection of a plane and a sphere.

  \param oP_ : This 7-dim vector defines the parameters oP[0], oP[1], oP[2] corresponding
  to parameters oA, oB, oC of the plane with equation oA*(X-oX)+oB*(Y-oY)+oC*(Z-oZ)=0
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

  \param oA, oB, oC : Parameters of the plane with equation oA*(X-oX)+oB*(Y-oY)+oC*(Z-oZ)=0
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
vpCircle::~vpCircle() { }

/*!
  Perspective projection of the circle.

  From the 3D parameters of the circle in the camera frame available in cP,
  computes the 2D parameters of the ellipse resulting from the perspective
  projection in the image plane. Those 2D parameters are available in p
  vector.

  See vpCircle::projection(const vpColVector &, vpColVector &) const for a more
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
  contains the following parameters: x, y, n20, n11, n02 where:
  - x, y are the normalized coordinates of the ellipse centroid (ie the
  perspective projection of a 3D circle becomes a 2D ellipse in the image) in
  the image plane.
  - n20, n11, n02 which are the second order centered moments of
  the ellipse normalized by its area (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where
  \f$\mu_{ij}\f$ are the centered moments and a the area).
  */
void vpCircle::projection(const vpColVector &cP_, vpColVector &p_) const
{
  double det_threshold = 1e-10;
  p_.resize(5, false);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;

  vpColVector K(6);
  {
    double A = cP_[index_0];
    double B = cP_[index_1];
    double C = cP_[index_2];

    double X0 = cP_[index_3];
    double Y0 = cP_[index_4];
    double Z0 = cP_[index_5];

    double r = cP_[6];

    // projection
    double s = ((X0 * X0) + (Y0 * Y0) + (Z0 * Z0)) - (r * r);
    double det = (A * X0) + (B * Y0) + (C * Z0);
    A = A / det;
    B = B / det;
    C = C / det;

    K[index_0] = (1 - (2 * A * X0)) + (A * A * s);
    K[index_1] = (1 - (2 * B * Y0)) + (B * B * s);
    K[index_2] = ((-A * Y0) - (B * X0)) + (A * B * s);
    K[index_3] = ((-C * X0) - (A * Z0)) + (A * C * s);
    K[index_4] = ((-C * Y0) - (B * Z0)) + (B * C * s);
    K[index_5] = (1 - (2 * C * Z0)) + (C * C * s);
  }

  double det = (K[index_2] * K[index_2]) - (K[index_0] * K[index_1]);
  if (fabs(det) < det_threshold) {
    throw(vpException(vpException::divideByZeroError, "Division by 0 in vpCircle::projection."));
  }

  double xc = ((K[index_1] * K[index_3]) - (K[index_2] * K[index_4])) / det;
  double yc = ((K[index_0] * K[index_4]) - (K[index_2] * K[index_3])) / det;

  double c = sqrt(((K[index_0] - K[index_1]) * (K[index_0] - K[index_1])) + (4 * K[index_2] * K[index_2]));
  double s = 2 * (((K[index_0] * xc * xc) + (2 * K[index_2] * xc * yc) + (K[1] * yc * yc)) - K[index_5]);

  double A, B, E;

  if (fabs(K[index_2]) < std::numeric_limits<double>::epsilon()) {
    E = 0.0;
    if (K[0] > K[1]) {
      A = sqrt(s / ((K[0] + K[1]) + c));
      B = sqrt(s / ((K[0] + K[1]) - c));
    }
    else {
      A = sqrt(s / ((K[0] + K[1]) - c));
      B = sqrt(s / ((K[0] + K[1]) + c));
    }
  }
  else {
    E = ((K[1] - K[0]) + c) / (2 * K[index_2]);
    if (fabs(E) > 1.0) {
      A = sqrt(s / ((K[0] + K[1]) + c));
      B = sqrt(s / ((K[0] + K[1]) - c));
    }
    else {
      A = sqrt(s / ((K[0] + K[1]) - c));
      B = sqrt(s / ((K[0] + K[1]) + c));
      E = -1.0 / E;
    }
  }

  // Chaumette PhD Thesis 1990, eq 2.72 divided by 4 since n_ij = mu_ij_chaumette_thesis / 4
  det = 4 * (1.0 + vpMath::sqr(E));
  double n20 = (vpMath::sqr(A) + vpMath::sqr(B * E)) / det;
  double n11 = ((vpMath::sqr(A) - vpMath::sqr(B)) * E) / det;
  double n02 = (vpMath::sqr(B) + vpMath::sqr(A * E)) / det;

  p_[index_0] = xc;
  p_[index_1] = yc;
  p_[index_2] = n20;
  p_[index_3] = n11;
  p_[index_4] = n02;
}

/*!
  From the 3D coordinates of the circle in the object frame oP that are set using for
  example vpCircle(double oA, double oB, double oC, double oX, double oY, double oZ, double R)
  or setWorldCoordinates(), compute the 3D coordinates of the circle in a new object frame noP = noMo * oP.
  Internal parameters of the circle remain unchanged.

  \param noMo : Transformation from camera to object frame.
  \param noP : 3D normalized coordinates of the circle in the a new ojbect frame noP = (noA, noB, noC, noX, noY, noZ,
  R).
*/
void vpCircle::changeFrame(const vpHomogeneousMatrix &noMo, vpColVector &noP) const
{
  noP.resize(7, false);

  double A, B, C;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  const unsigned int index_6 = 6;
  A = (noMo[index_0][0] * oP[0]) + (noMo[index_0][1] * oP[1]) + (noMo[index_0][index_2] * oP[index_2]);
  B = (noMo[index_1][0] * oP[0]) + (noMo[index_1][1] * oP[1]) + (noMo[index_1][index_2] * oP[index_2]);
  C = (noMo[index_2][0] * oP[0]) + (noMo[index_2][1] * oP[1]) + (noMo[index_2][index_2] * oP[index_2]);

  double X0, Y0, Z0;
  X0 = noMo[index_0][index_3] + (noMo[index_0][0] * oP[3]) + (noMo[index_0][1] * oP[index_4]) + (noMo[index_0][index_2] * oP[index_5]);
  Y0 = noMo[index_1][index_3] + (noMo[index_1][0] * oP[3]) + (noMo[index_1][1] * oP[index_4]) + (noMo[index_1][index_2] * oP[index_5]);
  Z0 = noMo[index_2][index_3] + (noMo[index_2][0] * oP[3]) + (noMo[index_2][1] * oP[index_4]) + (noMo[index_2][index_2] * oP[index_5]);
  double R = oP[6];

  noP[index_0] = A;
  noP[index_1] = B;
  noP[index_2] = C;

  noP[index_3] = X0;
  noP[index_4] = Y0;
  noP[index_5] = Z0;

  noP[index_6] = R;
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
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  const unsigned int index_6 = 6;
  A = (cMo[index_0][0] * oP[0]) + (cMo[index_0][1] * oP[1]) + (cMo[index_0][index_2] * oP[index_2]);
  B = (cMo[index_1][0] * oP[0]) + (cMo[index_1][1] * oP[1]) + (cMo[index_1][index_2] * oP[index_2]);
  C = (cMo[index_2][0] * oP[0]) + (cMo[index_2][1] * oP[1]) + (cMo[index_2][index_2] * oP[index_2]);

  double X0, Y0, Z0;
  X0 = cMo[index_0][index_3] + (cMo[index_0][0] * oP[index_3]) + (cMo[index_0][1] * oP[index_4]) + (cMo[index_0][index_2] * oP[index_5]);
  Y0 = cMo[index_1][index_3] + (cMo[index_1][0] * oP[index_3]) + (cMo[index_1][1] * oP[index_4]) + (cMo[index_1][index_2] * oP[index_5]);
  Z0 = cMo[index_2][index_3] + (cMo[index_2][0] * oP[index_3]) + (cMo[index_2][1] * oP[index_4]) + (cMo[index_2][index_2] * oP[index_5]);
  double R = oP[6];

  cP[index_0] = A;
  cP[index_1] = B;
  cP[index_2] = C;

  cP[index_3] = X0;
  cP[index_4] = Y0;
  cP[index_5] = Z0;

  cP[index_6] = R;
}

/*!
 * Display the projection of a 3D circle in image \e I using internal coordinates in the image plane (x,y,n20,n11,n02)
 * available in `p` vector. These coordinates may be updated using projection().
 *
 * \param I : Image used as background.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the circle.
 * \param thickness : Thickness used to draw the circle.
 */
void vpCircle::display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color,
                       unsigned int thickness)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  vpFeatureDisplay::displayEllipse(p[index_0], p[index_1], p[index_2], p[index_3], p[index_4], cam, I, color, thickness);
}

/*!
 * Display the projection of a 3D circle in image \e I using internal coordinates in the image plane (x,y,n20,n11,n02)
 * available in `p` vector. These coordinates may be updated using projection().
 *
 * \param I : Image used as background.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the circle.
 * \param thickness : Thickness used to draw the circle.
 */
void vpCircle::display(const vpImage<vpRGBa> &I, const vpCameraParameters &cam, const vpColor &color,
                       unsigned int thickness)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  vpFeatureDisplay::displayEllipse(p[index_0], p[index_1], p[index_2], p[index_3], p[index_4], cam, I, color, thickness);
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
  vpColVector v_cP, v_p;
  changeFrame(cMo, v_cP);
  projection(v_cP, v_p);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  vpFeatureDisplay::displayEllipse(v_p[index_0], v_p[index_1], v_p[index_2], v_p[index_3], v_p[index_4], cam, I, color, thickness);
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
void vpCircle::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &color, unsigned int thickness)
{
  vpColVector v_cP, v_p;
  changeFrame(cMo, v_cP);
  projection(v_cP, v_p);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  vpFeatureDisplay::displayEllipse(v_p[index_0], v_p[index_1], v_p[index_2], v_p[index_3], v_p[index_4], cam, I, color, thickness);
}

//! For memory issue (used by the vpServo class only)
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

  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;

  double n11 = circle.p[index_3];
  double n02 = circle.p[index_4];
  double n20 = circle.p[index_2];
  double Xg = u0 + (circle.p[index_0] * px);
  double Yg = v0 + (circle.p[index_1] * py);

  // Find Intersection between line and ellipse in the image.

  // Optimised calculation for X
  double stheta = sin(theta);
  double ctheta = cos(theta);
  double sctheta = stheta * ctheta;
  double m11yg = n11 * Yg;
  double ctheta2 = vpMath::sqr(ctheta);
  double m02xg = n02 * Xg;
  double m11stheta = n11 * stheta;
  j = ((((((((n11 * Xg * sctheta) - (n20 * Yg * sctheta)) + (n20 * rho * ctheta)) - m11yg) + (m11yg * ctheta2) + m02xg) -
         (m02xg * ctheta2)) + (m11stheta * rho)) /
       (((n20 * ctheta2) + (2.0 * m11stheta * ctheta) + n02) - (n02 * ctheta2)));
  // Optimised calculation for Y
  double rhom02 = rho * n02;
  double sctheta2 = stheta * ctheta2;
  double ctheta3 = ctheta2 * ctheta;
  i = (-(((((((-rho * n11 * stheta * ctheta) - rhom02) + (rhom02 * ctheta2) + (n11 * Xg * sctheta2)) - (n20 * Yg * sctheta2)) -
           (ctheta * n11 * Yg)) + (ctheta3 * n11 * Yg) + (ctheta * n02 * Xg)) - (ctheta3 * n02 * Xg)) /
       (((n20 * ctheta2) + (2.0 * n11 * stheta * ctheta) + n02) - (n02 * ctheta2)) / stheta);
}
END_VISP_NAMESPACE
