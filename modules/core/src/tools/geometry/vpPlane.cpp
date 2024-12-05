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
 * Plane geometrical structure.
 */

/*!
  \file vpPlane.cpp
  \brief definition of the vpPlane class member functions
*/

#include <visp3/core/vpPlane.h>

#include <cmath>  // std::fabs
#include <limits> // numeric_limits

BEGIN_VISP_NAMESPACE
/*!
  Copy operator.
*/
vpPlane &vpPlane::operator=(const vpPlane &p)
{
  A = p.A;
  B = p.B;
  C = p.C;
  D = p.D;

  return *this;
}

/*!
  Basic constructor that set the plane parameters A, B, C, D to zero.
*/
vpPlane::vpPlane() : A(0), B(0), C(0), D(0) { }

/*!
  Plane constructor from A, B, C, D parameters.

  A plane is given by the equation \f$A*X + B*Y + C*Z + D = 0\f$ where
  (X,Y,Z) are the coordinates of a point and \f$[A,B,C]^T\f$ is the normal
  vector of the plane.

  \param a, b, c, d : Parameters of the plane.

*/
vpPlane::vpPlane(double a, double b, double c, double d) : A(a), B(b), C(c), D(d) { }

/*!
  Copy constructor.
*/
vpPlane::vpPlane(const vpPlane &P) : A(0), B(0), C(0), D(0)
{
  setA(P.getA());
  setB(P.getB());
  setC(P.getC());
  setD(P.getD());
}

/*!
  Plane constructor from a point \e P on the plane and the \e normal to the plane.

  A plane is given by the equation \f$A*X + B*Y + C*Z + D = 0\f$ where
  (X,Y,Z) are the coordinates of a point and \f$[A,B,C]^T\f$ is the normal
  vector of the plane.

  \param P : A point with coordinates (X,Y,Z) on the plane. The \e frame
  parameter indicates if the coordinates of this point are
  expressed in the camera or object frame.

  \param normal : The normal to the plane.

  \param frame: Indicates if the plane should be initialized from the point P
  coordinates expressed in the camera or object frame.
  - When expressed in the camera frame we get the coordinates of the point using
    (`P.get_X()`, `P.get_Y()`, `P.get_Z()`).
  - When expressed in the object frame we get the coordinates of the point using
    (`P.get_oX()`, `P.get_oY()`, `P.get_oZ()`).
*/
vpPlane::vpPlane(const vpPoint &P, const vpColVector &normal, vpPlaneFrame frame) : A(0), B(0), C(0), D(0)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  // Equation of the plane is given by:
  A = normal[index_0];
  B = normal[index_1];
  C = normal[index_2];

  if (frame == vpPlane::camera_frame) {
    D = -((A * P.get_X()) + (B * P.get_Y()) + (C * P.get_Z()));
  }
  else {
    D = -((A * P.get_oX()) + (B * P.get_oY()) + (C * P.get_oZ()));
  }
}

/*!
  Initialize the plane with the parameters of an other plane \e P.

  \param P : Plane used as initializer.
*/
vpPlane &vpPlane::init(const vpPlane &P)
{
  setA(P.getA());
  setB(P.getB());
  setC(P.getC());
  setD(P.getD());

  return *this;
}

/*!
  Initialize the plane from a point \e P on the plane and the \e normal to the plane.

  \param P : A point with coordinates (X,Y,Z) on the plane. The \e frame
  parameter indicates if the coordinates of this point are
  expressed in the camera or object frame.

  \param normal : The normal to the plane.

  \param frame: Indicates if the plane should be initialized from the point P
  coordinates expressed in the camera (X, Y, Z) or object frame (oX, oY, oZ).
  - When expressed in the camera frame we get the coordinates of the point using
    (`P.get_X()`, `P.get_Y()`, `P.get_Z()`).
  - When expressed in the object frame we get the coordinates of the point using
    (`P.get_oX()`, `P.get_oY()`, `P.get_oZ()`).

  \sa vpPlane(const vpPoint&, const vpColVector &)
*/
vpPlane &vpPlane::init(const vpPoint &P, const vpColVector &normal, vpPlaneFrame frame)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  // Equation of the plane is given by:
  A = normal[index_0];
  B = normal[index_1];
  C = normal[index_2];

  if (frame == vpPlane::camera_frame) {
    D = -((A * P.get_X()) + (B * P.get_Y()) + (C * P.get_Z()));
  }
  else {
    D = -((A * P.get_oX()) + (B * P.get_oY()) + (C * P.get_oZ()));
  }

  return *this;
}

/*!
  Initialize the plane from a point \e P on the plane and the normal
  \e n to the plane.

  \param P : A point with coordinates (x,y,z) on the plane.
  The size of the vector should be 3, with P[0]=x, with P[1]=y, with P[2]=z.

  \param normal : The normal to the plane.

  \sa vpPlane(const vpPoint&, const vpColVector &)
*/
vpPlane &vpPlane::init(const vpColVector &P, const vpColVector &normal)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  // Equation of the plane is given by:
  A = normal[index_0];
  B = normal[index_1];
  C = normal[index_2];

  D = -((A * P[0]) + (B * P[1]) + (C * P[index_2]));

  return *this;
}

/*!
  Compute the equation of a plane given three point P, Q, R.

  The normal to the plane is given by:
  n = PQ x PR

  \param P,Q,R: Three points on the plane.
  \param frame: Indicates if the plane should be initialized from the points
  coordinates expressed in the camera or object frame.

*/
vpPlane &vpPlane::init(const vpPoint &P, const vpPoint &Q, const vpPoint &R, vpPlaneFrame frame)
{
  vpColVector a(3);
  vpColVector b(3);
  vpColVector n(3);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  if (frame == vpPlane::camera_frame) {
    // Calculate vector corresponding to PQ
    a[index_0] = P.get_X() - Q.get_X();
    a[index_1] = P.get_Y() - Q.get_Y();
    a[index_2] = P.get_Z() - Q.get_Z();

    // Calculate vector corresponding to PR
    b[index_0] = P.get_X() - R.get_X();
    b[index_1] = P.get_Y() - R.get_Y();
    b[index_2] = P.get_Z() - R.get_Z();
  }
  else {
    // Calculate vector corresponding to PQ
    a[index_0] = P.get_oX() - Q.get_oX();
    a[index_1] = P.get_oY() - Q.get_oY();
    a[index_2] = P.get_oZ() - Q.get_oZ();

    // Calculate vector corresponding to PR
    b[index_0] = P.get_oX() - R.get_oX();
    b[index_1] = P.get_oY() - R.get_oY();
    b[index_2] = P.get_oZ() - R.get_oZ();
  }
  // Calculate normal vector to plane PQ x PR
  n = vpColVector::cross(a, b);

  // Equation of the plane is given by:
  A = n[index_0];
  B = n[index_1];
  C = n[index_2];
  if (frame == vpPlane::camera_frame) {
    D = -((A * P.get_X()) + (B * P.get_Y()) + (C * P.get_Z()));
  }
  else {
    D = -((A * P.get_oX()) + (B * P.get_oY()) + (C * P.get_oZ()));
  }

  double norm = sqrt((A * A) + (B * B) + (C * C));
  A /= norm;
  B /= norm;
  C /= norm;
  D /= norm;

  return *this;
}

/*!
  Compute the equation of a plane given three point P, Q, R.

  The normal to the plane is given by:
  n = PQ x PR

  \param P,Q,R: Three points on the plane.
  \param frame: Indicates if the plane should be initialized from the points
  coordinates expressed in the camera or object frame.

  \sa init(const vpPoint &, const vpPoint &, const vpPoint &)
*/
vpPlane::vpPlane(const vpPoint &P, const vpPoint &Q, const vpPoint &R, vpPlaneFrame frame) : A(0), B(0), C(0), D(0)
{
  init(P, Q, R, frame);
}

/*!
 * Compute Z value of a 3D point located on the plane from its perspective projection coordinates.
 * \param[in] x, y : Coordinates of a point in the image plane. These coordinates are the one obtained by perspective projection of a 3D point.
 * \return Z coordinate in [m] of the corresponding 3D point.
 */
double vpPlane::computeZ(double x, double y) const
{
  return -getD() / ((getA() * x) + (getB() * y) + getC());
}

/*!
  Return the normal to the plane.

  A plane is given by the equation \f$A*X + B*Y + C*Z + D = 0\f$ where
  (x,y,z) is a point of R^3 and (A,B,C) are the coordinates of the normal.

  \sa getNormal(vpColVector &n)
*/
vpColVector vpPlane::getNormal() const
{
  const unsigned int val_3 = 3;
  vpColVector n(val_3);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  n[index_0] = A;
  n[index_1] = B;
  n[index_2] = C;

  return n;
}

/*!
  Return the normal to the plane.

  A plane is given by the equation \f$A*X + B*Y + C*Z + D = 0\f$ where
  (X,Y,Z) are the coordinates of a point and \f$[A,B,C]^T\f$ is the normal
  vector of the plane.

  \sa getNormal()

*/
void vpPlane::getNormal(vpColVector &n) const
{
  const unsigned int val_3 = 3;
  n.resize(val_3);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  n[index_0] = A;
  n[index_1] = B;
  n[index_2] = C;
}

/*!
  Compute the coordinates of the projection of a point on the plane.

  \param[in] P : Point to be projected on the plane.
  \param[out] Pproj : Projected point.
  \param[in] frame : Indicates if the point P coordinates are expressed in the camera or object frame.
  - When expressed in the camera frame we get the coordinates of the point using
    (`P.get_X()`, `P.get_Y()`, `P.get_Z()`).
  - When expressed in the object frame we get the coordinates of the point using
    (`P.get_oX()`, `P.get_oY()`, `P.get_oZ()`).
*/
void vpPlane::projectionPointOnPlan(const vpPoint &P, vpPoint &Pproj, vpPlaneFrame frame) const
{
  double x0, y0, z0;
  double rho;

  if (frame == vpPlane::camera_frame) {
    x0 = P.get_X() / P.get_W();
    y0 = P.get_Y() / P.get_W();
    z0 = P.get_Z() / P.get_W();

    rho = -((A * x0) + (B * y0) + (C * z0) + D) / ((A * A) + (B * B) + (C * C));

    Pproj.set_X(x0 + (A * rho));
    Pproj.set_Y(y0 + (B * rho));
    Pproj.set_Z(z0 + (C * rho));
    Pproj.set_W(1);
  }
  else {
    x0 = P.get_oX() / P.get_oW();
    y0 = P.get_oY() / P.get_oW();
    z0 = P.get_oZ() / P.get_oW();

    rho = -((A * x0) + (B * y0) + (C * z0) + D) / ((A * A) + (B * B) + (C * C));

    Pproj.set_oX(x0 + (A * rho));
    Pproj.set_oY(y0 + (B * rho));
    Pproj.set_oZ(z0 + (C * rho));
    Pproj.set_oW(1);
  }
}

double vpPlane::rayIntersection(const vpPoint &M0, const vpPoint &M1, vpColVector &H) const
{
  double k, scal;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;

  // --comment: if M0.get_X() diff 0 or M0.get_Y() diff 0 or M0.get_Z() diff 0
  if ((std::fabs(M0.get_X()) > std::numeric_limits<double>::epsilon()) ||
      (std::fabs(M0.get_Y()) > std::numeric_limits<double>::epsilon()) ||
      (std::fabs(M0.get_Z()) > std::numeric_limits<double>::epsilon())) {
    double R[3];
    R[index_0] = M1.get_X() - M0.get_X();
    R[index_1] = M1.get_Y() - M0.get_Y();
    R[index_2] = M1.get_Z() - M0.get_Z();

    scal = (getA() * R[index_0]) + (getB() * R[index_1]) + (getC() * R[index_2]);
    // --comment: if scal != 0
    if (std::fabs(scal) > std::numeric_limits<double>::epsilon()) {
      k = -((getA() * M0.get_X()) + (getB() * M0.get_Y()) + (getC() * M0.get_Z()) + getD()) / scal;
    }
    else {
      k = 0;
    }

    H[index_0] = M0.get_X() + (k * R[index_0]);
    H[index_1] = M0.get_Y() + (k * R[index_1]);
    H[index_2] = M0.get_Z() + (k * R[index_2]);
  }
  else {
    scal = (getA() * M1.get_X()) + (getB() * M1.get_Y()) + (getC() * M1.get_Z());
    // --comment: if scal != 0
    if (std::fabs(scal) > std::numeric_limits<double>::epsilon()) {
      k = -getD() / scal;
    }
    else {
      k = 0;
    }
    H[index_0] = k * M1.get_X();
    H[index_1] = k * M1.get_Y();
    H[index_2] = k * M1.get_Z();
  }

  return k;
}

double vpPlane::getIntersection(const vpColVector &M1, vpColVector &H) const
{
  double k, scal;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;

  scal = (A * M1[index_0]) + (B * M1[index_1]) + (C * M1[index_2]);
  // --comment: if scal != 0
  if (std::fabs(scal) > std::numeric_limits<double>::epsilon()) {
    k = -getD() / scal;
  }
  else {
    k = 0;
  }
  H[index_0] = k * M1[index_0];
  H[index_1] = k * M1[index_1];
  H[index_2] = k * M1[index_2];

  return k;
}

/*!

  Considering the plane in the Ro frame computes the equation of the
  plane in the Rc frame.

  \param cMo : Homogeneous transformation from Rc to Ro frames.

*/
void vpPlane::changeFrame(const vpHomogeneousMatrix &cMo)
{
  // Save current plane parameters
  double Ao = A;
  double Bo = B;
  double Co = C;
  double Do = D;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  A = (cMo[index_0][0] * Ao) + (cMo[index_0][1] * Bo) + (cMo[index_0][index_2] * Co);
  B = (cMo[index_1][0] * Ao) + (cMo[index_1][1] * Bo) + (cMo[index_1][index_2] * Co);
  C = (cMo[index_2][0] * Ao) + (cMo[index_2][1] * Bo) + (cMo[index_2][index_2] * Co);
  D = Do - ((cMo[index_0][index_3] * A) + (cMo[index_1][index_3] * B) + (cMo[index_2][index_3] * C));
}

/*!

  Print the plane parameters as a stream like "(A,B,C,D) " where
  A,B,C and D correspond to the parameters of the plane.

*/
VISP_EXPORT std::ostream &operator<<(std::ostream &os, const vpPlane &p)
{
  return (os << "(" << p.getA() << "," << p.getB() << "," << p.getC() << "," << p.getD() << ") ");
}
END_VISP_NAMESPACE
