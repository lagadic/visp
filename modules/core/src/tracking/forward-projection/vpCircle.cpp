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
  Set the world coordinates of the circle from the intersection of a plane and
  a sphere. We mean here the coordinates of the circle in the object frame

  \param oP_ : oP[0], oP[1], oP[2] correspond to A, B, C from the plane
  equation Ax + By + Cz = 0. oP[3], oP[4], oP[5] correspond to X, Y, Z the
  coordinates of the center of the sphere. oP[6] corresponds to the radius of
  the sphere.
*/
void vpCircle::setWorldCoordinates(const vpColVector &oP_) { this->oP = oP_; }

/*!
  Set the world coordinates of the circle from the intersection of a plane and
  a sphere. We mean here the coordinates of the circle in the object frame

  \param A : A from the plane equation Ax + By + Cz = 0.
  \param B : B from the plane equation Ax + By + Cz = 0.
  \param C : C from the plane equation Ax + By + Cz = 0.
  \param X0 : X Coordinate of the center of the sphere.
  \param Y0 : Y Coordinate of the center of the sphere.
  \param Z0 : Z Coordinate of the center of the sphere.
  \param R : Radius of the sphere.
*/
void vpCircle::setWorldCoordinates(const double A, const double B, const double C, const double X0, const double Y0,
                                   const double Z0, const double R)
{
  oP[0] = A;
  oP[1] = B;
  oP[2] = C;
  oP[3] = X0;
  oP[4] = Y0;
  oP[5] = Z0;
  oP[6] = R;
}

vpCircle::vpCircle() { init(); }

/*!
  Construct the circle from the intersection of a plane and a sphere.

  \param oP_ : oP[0], oP[1], oP[2] correspond to A, B, C from the plane
  equation Ax + By + Cz = 0. oP[3], oP[4], oP[5] correspond to X, Y, Z the
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
  Construct the circle from the intersection of a plane and a sphere.

  \param A : A from the plane equation Ax + By + Cz = 0.
  \param B : B from the plane equation Ax + By + Cz = 0.
  \param C : C from the plane equation Ax + By + Cz = 0.
  \param X0 : X Coordinate of the center of the sphere.
  \param Y0 : Y Coordinate of the center of the sphere.
  \param Z0 : Z Coordinate of the center of the sphere.
  \param R : Radius of the sphere.

  \sa setWorldCoordinates()
*/
vpCircle::vpCircle(const double A, const double B, const double C, const double X0, const double Y0, const double Z0,
                   const double R)
{
  init();
  setWorldCoordinates(A, B, C, X0, Y0, Z0, R);
}

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
  \param cP_: 3D cercle input parameters. This vector is of dimension 7. It
  contains the following parameters: A, B, C, X0, Y0, Z0, r where
  - A,B,C are the parameters of the plane with equation Ax+By+Cz+D=0
  containing the circle
  - X0,Y0,Z0 are the 3D coordinates of the cercle in the camera frame
  - r is the circle radius.

  \param p_: 2D circle output parameters. This is a 5 dimension vector. It
  contains the following parameters: xc, yc, m20, m11, m02 where:
  - xc,yc are the normalized coordinates of the center of the ellipse (ie the
  perspective projection of a 3D circle becomes a 2D ellipse in the image) in
  the image plane.
  - mu20,mu11,mu02 are the second order centered moments of the ellipse.
  */
void vpCircle::projection(const vpColVector &cP_, vpColVector &p_)
{
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

  //  {
  //    std::cout << "K dans vpCircle::projection(): " << std::endl;
  //    for (unsigned int i=1; i<6; i++)
  //      std::cout << K[i]/K[0] << std::endl;
  //  }
  double det = K[2] * K[2] - K[0] * K[1];
  if (fabs(det) < 1e-8) {
    vpERROR_TRACE("division par 0");
    throw(vpException(vpException::divideByZeroError, "division par 0"));
  }

  double xc = (K[1] * K[3] - K[2] * K[4]) / det;
  double yc = (K[0] * K[4] - K[2] * K[3]) / det;

  double c = sqrt((K[0] - K[1]) * (K[0] - K[1]) + 4 * K[2] * K[2]);
  double s = 2 * (K[0] * xc * xc + 2 * K[2] * xc * yc + K[1] * yc * yc - K[5]);

  double A, B, E;

  // if (fabs(K[2])<1e-6)
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

//! perspective projection of the circle
void vpCircle::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP_)
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

  cP_[0] = A;
  cP_[1] = B;
  cP_[2] = C;

  cP_[3] = X0;
  cP_[4] = Y0;
  cP_[5] = Z0;

  cP_[6] = R;

  // vpTRACE("_cP :") ; std::cout << _cP.t() ;
}

//! perspective projection of the circle
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

  // vpTRACE("_cP :") ; std::cout << _cP.t() ;
}

void vpCircle::display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color,
                       const unsigned int thickness)
{
  vpFeatureDisplay::displayEllipse(p[0], p[1], p[2], p[3], p[4], cam, I, color, thickness);
}

// non destructive wrt. cP and p
void vpCircle::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                       const vpColor &color, const unsigned int thickness)
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
  \param cam : Camera parameters that have to be used for the intersection
  computation. \param rho : The rho parameter of the line. \param theta : The
  theta parameter of the line. \param i : resulting i-coordinate of the
  intersection point. \param j : resulting j-coordinate of the intersection
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
