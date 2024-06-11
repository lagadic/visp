/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * Cylinder feature.
 *
*****************************************************************************/

#include <visp3/core/vpCylinder.h>
#include <visp3/core/vpFeatureDisplay.h>

BEGIN_VISP_NAMESPACE
void vpCylinder::init()
{
  oP.resize(7);
  cP.resize(7);

  p.resize(4);
}

/*!
  Set the cylinder parameters \f$^{o}{\bf P} =
  ({^o}A,{^o}B,{^o}C,{^o}X_0,{^o}Y_0,{^o}Z_0,R)\f$ expressed in the world
  frame.

  \param o_P : Vector of parameters \f$^{o}{\bf P}\f$.

  \code
  vpCylinder cylinder;
  double oA, oB, oC, oX, oY, oZ, R;
  ...
  vpColVector oP({oA, oB, oC, oX, oY, oZ, R});
  cylinder.setWorldCoordinates(oP);
  \endcode
*/
void vpCylinder::setWorldCoordinates(const vpColVector &o_P) { this->oP = o_P; }

/*!
  Set the cylinder parameters oP = (oA, oB, oC, oX, oY, oZ, R) expressed in the object
  frame.

  \param oA, oB, oC, oX, oY, oZ, R : Cylinder parameters in the object frame.

*/
void vpCylinder::setWorldCoordinates(double oA, double oB, double oC, double oX, double oY, double oZ, double R)
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
  Default constructor.
*/
vpCylinder::vpCylinder() { init(); }

/*!
  Create and initialize a cylinder with parameters oP = (oA, oB, oC, oX, oY, oZ, R) expressed in the object
  frame.

  \param o_P : 7-dim vector of parameters.

  \code
  vpCylinder cylinder;
  double oA, oB, oC, oX, oY, oZ, R;
  ...
  vpColVector oP({oA, oB, oC, oX, oY, oZ, R});
  vpCylinder cylinder(oP);
  \endcode
  \sa setWorldCoordinates(const vpColVector&)
*/
vpCylinder::vpCylinder(const vpColVector &o_P)
{
  init();
  setWorldCoordinates(o_P);
}

/*!
  Create and initialize a cylinder with parameters oP = (oA, oB, oC, oX, oY, oZ, R) expressed in the object
  frame.

  \param oA, oB, oC, oX, oY, oZ, R : Cylinder parameters expressed in the object frame.

  \sa setWorldCoordinates(const double,const double,const double,const
  double,const double,const double,const double)
*/
vpCylinder::vpCylinder(double oA, double oB, double oC, double oX, double oY, double oZ, double R)
{
  init();
  setWorldCoordinates(oA, oB, oC, oX, oY, oZ, R);
}

/*!
  Perspective projection of the cylinder.

  From the parameters of the cylinder in the camera frame, compute the
  perspective projection of the cylinder in the image plane.

  \code
  vpCylinder cylinder;
  vpColVector oP(7);
  double oA, oB, oC, oX, oY, oZ, R;
  ...
  vpColVector oP({oA, oB, oC, oX, oY, oZ, R});
  cylinder.setWorldCoordinates(oP); // Set the cylinder world frame parameters

  vpHomogeneousMatrix cMo;          // Camera to world frame transformation
  ...
  cylinder.changeFrame(cMo);        // Update internal cP parameters

  cylinder.projection();            // Compute the perspective projection
  \endcode

  \sa projection(const vpColVector &, vpColVector &)
  */
void vpCylinder::projection() { projection(cP, p); }

/*!
  Perspective projection of the cylinder.

  From the parameters of the cylinder in the camera frame \f$c{\bf P}\f$,
  compute the perspective projection of the cylinder in the image plane.

  \param cP_ [in] : Cylinder parameters in the camera frame.
  \param p_ [out] : Parameters of the cylinder in the image plane obtained by
  perspective projection.

  \exception vpException::fatalError : The camera is inside the cylinder.

  \code
  vpCylinder cylinder;
  vpColVector oP(7);
  // Initialize oP[] with A,B,C,X0,X0,Z0,R parameters
  cylinder.setWorldCoordinates(oP); // Set the cylinder world frame parameters

  vpHomogeneousMatrix cMo;          // Camera to world frame transformation
  vpColVector cP(7);                // Parameters of the cylinder in the
  camera frame cylinder.changeFrame(cMo, cP);    // Update cP parameters

  vpColVector p(4);                 // Parameters of the cylinder in the image
  plane cylinder.projection(cP, p);       // Compute the perspective
  projection and update p \endcode

  \sa projection()
  */
void vpCylinder::projection(const vpColVector &cP_, vpColVector &p_) const
{
  p_.resize(4, false);

  double co, si, e, x0, y0, z0;
  double A, B, C, X0, Y0, Z0, R;
  double s, a, b, c, zero;

  A = cP_[0];
  B = cP_[1];
  C = cP_[2];
  X0 = cP_[3];
  Y0 = cP_[4];
  Z0 = cP_[5];
  R = cP_[6];
  zero = A * X0 + B * Y0 + C * Z0; // should be zero for a good reprensetation of the cylinder

  s = X0 * X0 + Y0 * Y0 + Z0 * Z0 - R * R - zero * zero;
  if (s < 0) {
    throw vpException(vpException::fatalError, "The camera is inside the cylinder with s=%f!", s);
  }
  s = 1.0 / sqrt(s);
  a = X0 - A * zero; //(1-A*A)*X0 - A*B*Y0 - A*C*Z0;
  b = Y0 - B * zero; // - A*B*X0 + (1-B*B)*Y0 - B*C*Z0;
  c = Z0 - C * zero; //- A*C*X0  - B*C*Y0  + (1-C*C)*Z0;
  x0 = C * Y0 - B * Z0;
  y0 = A * Z0 - C * X0;
  z0 = B * X0 - A * Y0;

  // rho1 / theta1
  co = R * a * s - x0;
  si = R * b * s - y0;
  e = sqrt(co * co + si * si);
  p_[0] = -(R * c * s - z0) / e; // rho1
  p_[1] = atan2(si, co);         // theta 1

  // rho2 / theta2
  co = R * a * s + x0;
  si = R * b * s + y0;
  e = sqrt(co * co + si * si);
  p_[2] = -(R * c * s + z0) / e; // rho2
  p_[3] = atan2(si, co);         // theta2
}

/*!
  From the cylinder oP parameters expressed in the object frame,
  compute the cylinder internal parameters cP expressed in the
  camera frame.

  \param cMo : Camera to object frame transformation.

  \sa changeFrame(const vpHomogeneousMatrix &, vpColVector &) const
 */
void vpCylinder::changeFrame(const vpHomogeneousMatrix &cMo) { changeFrame(cMo, cP); }

/*!
  From the cylinder parameters oP expressed in the object frame,
  compute the cylinder parameters cP expressed in the camera
  frame.

  \param cMo : Camera to object frame transformation.
  \param cP_ [out] : Parameters cP expressed in the camera frame.

  \sa changeFrame(const vpHomogeneousMatrix &)
*/
void vpCylinder::changeFrame(const vpHomogeneousMatrix &cMo, vpColVector &cP_) const
{
  cP_.resize(7, false);

  double X1, Y1, Z1;
  double X2, Y2, Z2;
  double s, a, b, c;

  double oA, oB, oC, oX0, oY0, oZ0;
  oA = oP[0];
  oB = oP[1];
  oC = oP[2];
  oX0 = oP[3];
  oY0 = oP[4];
  oZ0 = oP[5];

  X1 = cMo[0][0] * oA + cMo[0][1] * oB + cMo[0][2] * oC;
  Y1 = cMo[1][0] * oA + cMo[1][1] * oB + cMo[1][2] * oC;
  Z1 = cMo[2][0] * oA + cMo[2][1] * oB + cMo[2][2] * oC;
  s = sqrt(X1 * X1 + Y1 * Y1 + Z1 * Z1);
  a = X1 / s;
  b = Y1 / s;
  c = Z1 / s;

  // set axis coordinates  in camera frame
  cP_[0] = a;
  cP_[1] = b;
  cP_[2] = c;

  X2 = cMo[0][3] + cMo[0][0] * oX0 + cMo[0][1] * oY0 + cMo[0][2] * oZ0;
  Y2 = cMo[1][3] + cMo[1][0] * oX0 + cMo[1][1] * oY0 + cMo[1][2] * oZ0;
  Z2 = cMo[2][3] + cMo[2][0] * oX0 + cMo[2][1] * oY0 + cMo[2][2] * oZ0;

  // adding the constraint X0 is the nearest point to the origin (A^T . X0 =
  // 0) using the projection operator (I - AA^T) orthogonal to A
  cP_[3] = (1 - a * a) * X2 - a * b * Y2 - a * c * Z2;
  cP_[4] = -a * b * X2 + (1 - b * b) * Y2 - b * c * Z2;
  cP_[5] = -a * c * X2 - b * c * Y2 + (1 - c * c) * Z2;

  /*  old version for the same onstraint
  if ( fabs(a) > 0.25 )
  {
    double xx, yy, zz, xx1, yy1;

    xx1 = a*Y2 - b*X2;
    yy1 = a*Z2 - c*X2;
    xx = -( b*xx1 + c*yy1);
    yy = (( a*a + c*c ) * xx1 - b*c*yy1 ) /a;
    zz = ( -b*c*xx1 + ( a*a + b*b )*yy1) /a;

    // set point coordinates  in camera frame
    _cP[3] = xx ;
    _cP[4] = yy ;
    _cP[5] = zz ;
  }
  else if ( fabs(b) >0.25 )
  {
    double xx, yy, zz, xx1, yy1;

    xx1 = a*Y2 - b*X2;
    yy1 = c*Y2 - b*Z2;
    xx = - (( b*b + c*c ) * xx1 - a*c*yy1 ) /b;
    yy = a*xx1 + c*yy1;
    zz = - ( -a*c*xx1 + (a*a + b*b) * yy1 ) /b;


    // set point coordinates  in camera frame
    _cP[3] = xx ;
    _cP[4] = yy ;
    _cP[5] = zz ;
  }
  else
  {
    double xx, yy, zz, xx1, yy1;

    xx1 = a*Z2 - c*X2;
    yy1 = b*Z2 - c*Y2;
    xx = (-( b*b + c*c ) * xx1 - a*c*yy1 ) /c;
    yy = ( a*b*xx1 - ( a*a + c*c )*yy1) /c;
    zz = a*xx1 + b*yy1;

    // set point coordinates  in camera frame
    _cP[3] = xx ;
    _cP[4] = yy ;
    _cP[5] = zz ;
  }
  */
  // radius
  cP_[6] = oP[6];
}

/*!
  Compute the Z coordinate for the given normalized coordinate in the camera
  frame.
*/
double vpCylinder::computeZ(double x, double y) const
{
  double A = x * x + y * y + 1 - ((getA() * x + getB() * y + getC()) * (getA() * x + getB() * y + getC()));
  double B = (x * getX() + y * getY() + getZ());
  double C = getX() * getX() + getY() * getY() + getZ() * getZ() - getR() * getR();

  return (B - std::sqrt(B * B - A * C)) / A;
}

//! For memory issue (used by the vpServo class only).
vpCylinder *vpCylinder::duplicate() const
{
  vpCylinder *feature = new vpCylinder(*this);
  return feature;
}

/*!
 * Display the projection of a 3D cylinder in image \e I as two lines corresponding to the limbs.
 * This method is non destructive wrt. cP and p internal 3D point parameters.
 *
 * \param I : Image used as background.
 * \param cMo : Homogeneous transformation from camera frame to object frame.
 * The point is considered as viewed from this camera position.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the sphere.
 * \param thickness : Thickness used to draw the sphere.
 */

void vpCylinder::display(const vpImage<unsigned char> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                         const vpColor &color, unsigned int thickness)
{

  vpColVector _cP(7), _p(4);
  changeFrame(cMo, _cP);
  projection(_cP, _p);
  vpFeatureDisplay::displayCylinder(_p[0], _p[1], _p[2], _p[3], cam, I, color, thickness);
}

/*!
 * Display the projection of a 3D cylinder in image \e I as two lines corresponding to the limbs.
 * This method is non destructive wrt. cP and p internal 3D point parameters.
 *
 * \param I : Image used as background.
 * \param cMo : Homogeneous transformation from camera frame to object frame.
 * The point is considered as viewed from this camera position.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the sphere.
 * \param thickness : Thickness used to draw the sphere.
 */

void vpCylinder::display(const vpImage<vpRGBa> &I, const vpHomogeneousMatrix &cMo, const vpCameraParameters &cam,
                         const vpColor &color, unsigned int thickness)
{

  vpColVector _cP(7), _p(4);
  changeFrame(cMo, _cP);
  projection(_cP, _p);
  vpFeatureDisplay::displayCylinder(_p[0], _p[1], _p[2], _p[3], cam, I, color, thickness);
}

/*!
 * Display the projection of a 3D cylinder in image \e I as two lines corresponding to the limbs.
 *
 * \param I : Image used as background.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the point.
 * \param thickness : Thickness used to draw the point.
 */
void vpCylinder::display(const vpImage<unsigned char> &I, const vpCameraParameters &cam, const vpColor &color,
                         unsigned int thickness)
{
  vpFeatureDisplay::displayCylinder(p[0], p[1], p[2], p[3], cam, I, color, thickness);
}

/*!
 * Display the projection of a 3D cylinder in image \e I as two lines corresponding to the limbs.
 *
 * \param I : Image used as background.
 * \param cam : Camera parameters.
 * \param color : Color used to draw the point.
 * \param thickness : Thickness used to draw the point.
 */
void vpCylinder::display(const vpImage<vpRGBa> &I, const vpCameraParameters &cam, const vpColor &color,
                         unsigned int thickness)
{
  vpFeatureDisplay::displayCylinder(p[0], p[1], p[2], p[3], cam, I, color, thickness);
}
END_VISP_NAMESPACE
