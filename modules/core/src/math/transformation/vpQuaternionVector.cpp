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
 * Quaternion vector.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <algorithm>
#include <stdio.h>
#include <string.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpQuaternionVector.h>

// minimum value of sine
const double vpQuaternionVector::minimum = 0.0001;

/*!
  \file vpQuaternionVector.cpp
  \brief Defines a quaternion and common operations on it.
*/

/*! Default constructor that initialize all the 4 angles to zero. */
vpQuaternionVector::vpQuaternionVector() : vpRotationVector(4) {}

/*! Copy constructor. */
vpQuaternionVector::vpQuaternionVector(const vpQuaternionVector &q) : vpRotationVector(q) {}

//! Constructor from doubles.
vpQuaternionVector::vpQuaternionVector(const double x_, const double y_, const double z_, const double w_)
  : vpRotationVector(4)
{
  set(x_, y_, z_, w_);
}

//! Constructor from a 4-dimension vector of doubles.
vpQuaternionVector::vpQuaternionVector(const vpColVector &q) : vpRotationVector(4)
{
  buildFrom(q);
}

//! Constructor from a 4-dimension vector of doubles.
vpQuaternionVector::vpQuaternionVector(const std::vector<double> &q) : vpRotationVector(4)
{
  buildFrom(q);
}

/*!
  Constructs a quaternion from a rotation matrix.

  \param R : Matrix containing a rotation.
*/
vpQuaternionVector::vpQuaternionVector(const vpRotationMatrix &R) : vpRotationVector(4) { buildFrom(R); }

/*!
  Constructor that initialize \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler
  angles vector from a \f$\theta {\bf u}\f$ vector.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as
  input to initialize the Euler angles.
*/
vpQuaternionVector::vpQuaternionVector(const vpThetaUVector &tu) : vpRotationVector(4) { buildFrom(tu); }

/*!
  Manually change values of a quaternion.
  \param qx : x quaternion parameter.
  \param qy : y quaternion parameter.
  \param qz : z quaternion parameter.
  \param qw : w quaternion parameter.
*/
void vpQuaternionVector::set(const double qx, const double qy, const double qz, const double qw)
{
  data[0] = qx;
  data[1] = qy;
  data[2] = qz;
  data[3] = qw;
}
/*!
  Manually change values of a quaternion.
  \param qx : x quaternion parameter.
  \param qy : y quaternion parameter.
  \param qz : z quaternion parameter.
  \param qw : w quaternion parameter.

  \sa set()
*/
vpQuaternionVector vpQuaternionVector::buildFrom(const double qx, const double qy, const double qz, const double qw)
{
  set(qx, qy, qz, qw);
  return *this;
}

/*!
  Convert a \f$\theta {\bf u}\f$ vector into a quaternion.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as
  input.
  \return Quaternion vector.
*/
vpQuaternionVector vpQuaternionVector::buildFrom(const vpThetaUVector &tu)
{
  vpRotationMatrix R(tu);
  buildFrom(R);

  return *this;
}

/*!
  Construct a quaternion vector from a 4-dim vector (x,y,z,w).
*/
vpQuaternionVector vpQuaternionVector::buildFrom(const vpColVector &q)
{
  if (q.size() != 4) {
    throw(vpException(vpException::dimensionError,
                      "Cannot construct a quaternion vector from a %d-dimension col vector", q.size()));
  }
  for (unsigned int i = 0; i < 4; i++)
    data[i] = q[i];

  return *this;
}

/*!
  Construct a quaternion vector from a 4-dim vector (x,y,z,w).
*/
vpQuaternionVector vpQuaternionVector::buildFrom(const std::vector<double> &q)
{
  if (q.size() != 4) {
    throw(vpException(vpException::dimensionError,
                      "Cannot construct a quaternion vector from a %d-dimension std::vector", q.size()));
  }
  for (unsigned int i = 0; i < 4; i++)
    data[i] = q[i];

  return *this;
}

/*!
  Quaternion addition.

  Adds two quaternions. Addition is component-wise.

  \param q : quaternion to add.
*/
vpQuaternionVector vpQuaternionVector::operator+(const vpQuaternionVector &q) const
{
  return vpQuaternionVector(x() + q.x(), y() + q.y(), z() + q.z(), w() + q.w());
}
/*!
  Quaternion substraction.

  Substracts a quaternion from another. Substraction is component-wise.

  \param q : quaternion to substract.
*/
vpQuaternionVector vpQuaternionVector::operator-(const vpQuaternionVector &q) const
{
  return vpQuaternionVector(x() - q.x(), y() - q.y(), z() - q.z(), w() - q.w());
}

//! Negate operator. Returns a quaternion defined by (-x,-y,-z-,-w).
vpQuaternionVector vpQuaternionVector::operator-() const { return vpQuaternionVector(-x(), -y(), -z(), -w()); }

//! Multiplication by scalar. Returns a quaternion defined by (lx,ly,lz,lw).
vpQuaternionVector vpQuaternionVector::operator*(const double l) const
{
  return vpQuaternionVector(l * x(), l * y(), l * z(), l * w());
}

//! Multiply two quaternions.
vpQuaternionVector vpQuaternionVector::operator*(const vpQuaternionVector &rq) const
{
  return vpQuaternionVector(w() * rq.x() + x() * rq.w() + y() * rq.z() - z() * rq.y(),
                            w() * rq.y() + y() * rq.w() + z() * rq.x() - x() * rq.z(),
                            w() * rq.z() + z() * rq.w() + x() * rq.y() - y() * rq.x(),
                            w() * rq.w() - x() * rq.x() - y() * rq.y() - z() * rq.z());
}

//! Division by scalar. Returns a quaternion defined by (x/l,y/l,z/l,w/l).
vpQuaternionVector vpQuaternionVector::operator/(const double l) const
{
  if (vpMath::nul(l, std::numeric_limits<double>::epsilon())) {
    throw vpException(vpException::fatalError, "Division by scalar l==0 !");
  }

  return vpQuaternionVector(x() / l, y() / l, z() / l, w() / l);
}
/*!

  Copy operator that initializes a quaternion vector from a 4-dimension column
vector \e q.

  \param q : 4-dimension vector containing the values of the quaternion
vector.

\code
#include <visp3/core/vpQuaternionVector.h>

int main()
{
  vpColVector v(4);
  v[0] = 0.1;
  v[1] = 0.2;
  v[2] = 0.3;
  v[3] = 0.4;
  vpQuaternionVector q;
  q = v;
  // q is now equal to v : 0.1, 0.2, 0.3, 0.4
}
\endcode
*/
vpQuaternionVector &vpQuaternionVector::operator=(const vpColVector &q)
{
  if (q.size() != 4) {
    throw(vpException(vpException::dimensionError, "Cannot set a quaternion vector from a %d-dimension col vector",
                      q.size()));
  }
  for (unsigned int i = 0; i < 4; i++)
    data[i] = q[i];

  return *this;
}

/*!
  Constructs a quaternion from a rotation matrix.

  \param R : Rotation matrix.
*/
vpQuaternionVector vpQuaternionVector::buildFrom(const vpRotationMatrix &R)
{
  vpThetaUVector tu(R);
  vpColVector u;
  double theta;
  tu.extract(theta, u);

  theta *= 0.5;

  double sinTheta_2 = sin(theta);
  set(u[0] * sinTheta_2, u[1] * sinTheta_2, u[2] * sinTheta_2, cos(theta));
  return *this;
}

/*!
  Quaternion conjugate.

  \return The conjugate quaternion.
*/
vpQuaternionVector vpQuaternionVector::conjugate() const { return vpQuaternionVector(-x(), -y(), -z(), w()); }

/*!
  Quaternion inverse.

  \return The inverse quaternion.
*/
vpQuaternionVector vpQuaternionVector::inverse() const
{
  vpQuaternionVector q_inv;

  double mag_square = w() * w() + x() * x() + y() * y() + z() * z();
  if (!vpMath::nul(mag_square, std::numeric_limits<double>::epsilon())) {
    q_inv = this->conjugate() / mag_square;
  } else {
    std::cerr << "The current quaternion is null ! The inverse cannot be computed !" << std::endl;
  }

  return q_inv;
}

/*!
  Quaternion magnitude or norm.

  \return The magnitude or norm of the quaternion.
*/
double vpQuaternionVector::magnitude() const { return sqrt(w() * w() + x() * x() + y() * y() + z() * z()); }

/*!
  Normalize the quaternion.
*/
void vpQuaternionVector::normalize()
{
  double mag = magnitude();
  if (!vpMath::nul(mag, std::numeric_limits<double>::epsilon())) {
    set(x() / mag, y() / mag, z() / mag, w() / mag);
  }
}

//! Returns x-component of the quaternion.
double vpQuaternionVector::x() const { return data[0]; }
//! Returns y-component of the quaternion.
double vpQuaternionVector::y() const { return data[1]; }
//! Returns z-component of the quaternion.
double vpQuaternionVector::z() const { return data[2]; }
//! Returns w-component of the quaternion.
double vpQuaternionVector::w() const { return data[3]; }
