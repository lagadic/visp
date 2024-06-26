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
 * Quaternion vector.
 */

#include <algorithm>
#include <cassert>
#include <stdio.h>
#include <string.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpQuaternionVector.h>

BEGIN_VISP_NAMESPACE
// minimum value of sine
const double vpQuaternionVector::minimum = 0.0001;

/*!
  \file vpQuaternionVector.cpp
  \brief Defines a quaternion and common operations on it.
*/

/*! Default constructor that initialize all the 4 angles to zero. */
vpQuaternionVector::vpQuaternionVector() : vpRotationVector(4) { }

/*! Copy constructor. */
vpQuaternionVector::vpQuaternionVector(const vpQuaternionVector &q) : vpRotationVector(q) { }

//! Constructor from doubles.
vpQuaternionVector::vpQuaternionVector(double x_, double y_, double z_, double w_) : vpRotationVector(4)
{
  set(x_, y_, z_, w_);
}

//! Constructor from a 4-dimension vector of doubles.
vpQuaternionVector::vpQuaternionVector(const vpColVector &q) : vpRotationVector(4) { build(q); }

//! Constructor from a 4-dimension vector of doubles.
vpQuaternionVector::vpQuaternionVector(const std::vector<double> &q) : vpRotationVector(4) { build(q); }

/*!
  Constructs a quaternion from a rotation matrix.

  \param R : Matrix containing a rotation.
*/
vpQuaternionVector::vpQuaternionVector(const vpRotationMatrix &R) : vpRotationVector(4) { build(R); }

/*!
  Constructor that initialize \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler
  angles vector from a \f$\theta {\bf u}\f$ vector.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as
  input to initialize the Euler angles.
*/
vpQuaternionVector::vpQuaternionVector(const vpThetaUVector &tu) : vpRotationVector(4) { build(tu); }

/*!
  Manually change values of a quaternion.
  \param qx : x quaternion parameter.
  \param qy : y quaternion parameter.
  \param qz : z quaternion parameter.
  \param qw : w quaternion parameter.
*/
void vpQuaternionVector::set(double qx, double qy, double qz, double qw)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  data[index_0] = qx;
  data[index_1] = qy;
  data[index_2] = qz;
  data[index_3] = qw;
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  \deprecated You should use build(const double &, const double &, const double &, const double &) instead.
  Manually change values of a quaternion.
  \param qx : x quaternion parameter.
  \param qy : y quaternion parameter.
  \param qz : z quaternion parameter.
  \param qw : w quaternion parameter.

  \sa set()
*/
vpQuaternionVector vpQuaternionVector::buildFrom(const double qx, const double qy, const double qz, const double qw)
{
  build(qx, qy, qz, qw);
  return *this;
}

/*!
  \deprecated You should use build(const vpThetaUVector &) instead.
  Convert a \f$\theta {\bf u}\f$ vector into a quaternion.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as
  input.
  \return Quaternion vector.
*/
vpQuaternionVector vpQuaternionVector::buildFrom(const vpThetaUVector &tu)
{
  build(tu);
  return *this;
}

/*!
  \deprecated You should use build(const vpColVector &q) instead.
  Construct a quaternion vector from a 4-dim vector (x,y,z,w).
*/
vpQuaternionVector vpQuaternionVector::buildFrom(const vpColVector &q)
{
  build(q);
  return *this;
}

/*!
  \deprecated You should use build(const std::vector<double> &q) instead.
  Construct a quaternion vector from a 4-dim vector (x,y,z,w).
*/
vpQuaternionVector vpQuaternionVector::buildFrom(const std::vector<double> &q)
{
  build(q);
  return *this;
}

/*!
  \deprecated You should use build(const vpRotationMatrix &) instead.
  Constructs a quaternion from a rotation matrix.

  \param R : Rotation matrix.
*/
vpQuaternionVector vpQuaternionVector::buildFrom(const vpRotationMatrix &R)
{
  build(R);
  return *this;
}
#endif

/*!
  Manually change values of a quaternion.
  \param qx : x quaternion parameter.
  \param qy : y quaternion parameter.
  \param qz : z quaternion parameter.
  \param qw : w quaternion parameter.

  \sa set()
*/
vpQuaternionVector &vpQuaternionVector::build(const double &qx, const double &qy, const double &qz, const double &qw)
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
vpQuaternionVector &vpQuaternionVector::build(const vpThetaUVector &tu)
{
  vpRotationMatrix R(tu);
  build(R);

  return *this;
}

/*!
  Construct a quaternion vector from a 4-dim vector (x,y,z,w).
*/
vpQuaternionVector &vpQuaternionVector::build(const vpColVector &q)
{
  if (q.size() != 4) {
    throw(vpException(vpException::dimensionError,
                      "Cannot construct a quaternion vector from a %d-dimension col vector", q.size()));
  }
  const unsigned int val_4 = 4;
  for (unsigned int i = 0; i < val_4; ++i) {
    data[i] = q[i];
  }

  return *this;
}

/*!
  Construct a quaternion vector from a 4-dim vector (x,y,z,w).
*/
vpQuaternionVector &vpQuaternionVector::build(const std::vector<double> &q)
{
  if (q.size() != 4) {
    throw(vpException(vpException::dimensionError,
                      "Cannot construct a quaternion vector from a %d-dimension std::vector", q.size()));
  }

  const unsigned int val_4 = 4;
  for (unsigned int i = 0; i < val_4; ++i) {
    data[i] = q[i];
  }

  return *this;
}

/*!
  Constructs a quaternion from a rotation matrix.

  \param R : Rotation matrix.
*/
vpQuaternionVector &vpQuaternionVector::build(const vpRotationMatrix &R)
{
  vpThetaUVector tu(R);
  vpColVector u;
  double theta;
  tu.extract(theta, u);

  theta *= 0.5;

  double sinTheta_2 = sin(theta);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  set(u[index_0] * sinTheta_2, u[index_1] * sinTheta_2, u[index_2] * sinTheta_2, cos(theta));
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
  Quaternion subtraction.

  subtracts a quaternion from another. subtraction is component-wise.

  \param q : quaternion to subtract.
*/
vpQuaternionVector vpQuaternionVector::operator-(const vpQuaternionVector &q) const
{
  return vpQuaternionVector(x() - q.x(), y() - q.y(), z() - q.z(), w() - q.w());
}

//! Negate operator. Returns a quaternion defined by (-x,-y,-z-,-w).
vpQuaternionVector vpQuaternionVector::operator-() const { return vpQuaternionVector(-x(), -y(), -z(), -w()); }

//! Multiplication by scalar. Returns a quaternion defined by (lx,ly,lz,lw).
vpQuaternionVector vpQuaternionVector::operator*(double l) const
{
  return vpQuaternionVector(l * x(), l * y(), l * z(), l * w());
}

//! Multiply two quaternions.
vpQuaternionVector vpQuaternionVector::operator*(const vpQuaternionVector &rq) const
{
  return vpQuaternionVector(((w() * rq.x()) + (x() * rq.w()) + (y() * rq.z())) - (z() * rq.y()),
                            ((w() * rq.y()) + (y() * rq.w()) + (z() * rq.x())) - (x() * rq.z()),
                            ((w() * rq.z()) + (z() * rq.w()) + (x() * rq.y())) - (y() * rq.x()),
                            ((w() * rq.w()) - (x() * rq.x()) - (y() * rq.y())) - (z() * rq.z()));
}

//! Division by scalar. Returns a quaternion defined by (x/l,y/l,z/l,w/l).
vpQuaternionVector vpQuaternionVector::operator/(double l) const
{
  if (vpMath::nul(l, std::numeric_limits<double>::epsilon())) {
    throw vpException(vpException::fatalError, "Division by scalar l==0 !");
  }

  return vpQuaternionVector(x() / l, y() / l, z() / l, w() / l);
}
/*!
  Copy operator that initializes a quaternion vector from a 4-dimension column
  vector \e q.

  \param q : 4-dimension vector containing the values of the quaternion vector.

  \code
  #include <visp3/core/vpQuaternionVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
  const unsigned int val_4 = 4;
  for (unsigned int i = 0; i < val_4; ++i) {
    data[i] = q[i];
  }

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

  double mag_square = (w() * w()) + (x() * x()) + (y() * y()) + (z() * z());
  if (!vpMath::nul(mag_square, std::numeric_limits<double>::epsilon())) {
    q_inv = this->conjugate() / mag_square;
  }
  else {
    std::cerr << "The current quaternion is null ! The inverse cannot be computed !" << std::endl;
  }

  return q_inv;
}

/*!
  Quaternion magnitude or norm.

  \return The magnitude or norm of the quaternion.
*/
double vpQuaternionVector::magnitude() const { return sqrt((w() * w()) + (x() * x()) + (y() * y()) + (z() * z())); }

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

/*!
  Compute dot product between two quaternions.

  \param q0 : First quaternion.
  \param q1 : Second quaternion.

  \return The dot product between q0 and q1.
*/
double vpQuaternionVector::dot(const vpQuaternionVector &q0, const vpQuaternionVector &q1)
{
  return (q0.x() * q1.x()) + (q0.y() * q1.y()) + (q0.z() * q1.z()) + (q0.w() * q1.w());
}

//! Returns the x-component of the quaternion.
const double &vpQuaternionVector::x() const { const unsigned int index_0 = 0; return data[index_0]; }
//! Returns the y-component of the quaternion.
const double &vpQuaternionVector::y() const { const unsigned int index_1 = 1; return data[index_1]; }
//! Returns the z-component of the quaternion.
const double &vpQuaternionVector::z() const { const unsigned int index_2 = 2; return data[index_2]; }
//! Returns the w-component of the quaternion.
const double &vpQuaternionVector::w() const { const unsigned int index_3 = 3; return data[index_3]; }

//! Returns a reference to the x-component of the quaternion.
double &vpQuaternionVector::x() { const unsigned int index_0 = 0; return data[index_0]; }
//! Returns a reference to the y-component of the quaternion.
double &vpQuaternionVector::y() { const unsigned int index_1 = 1; return data[index_1]; }
//! Returns a reference to the z-component of the quaternion.
double &vpQuaternionVector::z() { const unsigned int index_2 = 2; return data[index_2]; }
//! Returns a reference to the w-component of the quaternion.
double &vpQuaternionVector::w() { const unsigned int index_3 = 3; return data[index_3]; }

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  Set vector from a list of 4 double angle values.
  \code
  #include <visp3/core/vpQuaternionVector.cpp>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpQuaternionVector q = {0, 0, 0, 1};
    std::cout << "q: " << q.t() << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  q: 0  0  0  1
  \endcode
  \sa operator<<()
*/
vpQuaternionVector &vpQuaternionVector::operator=(const std::initializer_list<double> &list)
{
  if (list.size() > size()) {
    throw(vpException(
      vpException::dimensionError,
      "Cannot set quaternion vector out of bounds. It has only %d values while you try to initialize with %d values",
      size(), list.size()));
  }
  std::copy(list.begin(), list.end(), data);
  return *this;
}
#endif

/*!
  Compute Quaternion Linear intERPolation (LERP).
  See the following references:
    - https://github.com/Rajawali/Rajawali/blob/3dd6e09af22de1889241083c1e82ec72ba85bd40/rajawali/src/main/java/org/rajawali3d/math/Quaternion.java#L898-L931
    - https://stackoverflow.com/a/46187052

  \note Shortest path will be use.

  \param q0 : Start quaternion.
  \param q1 : End quaternion.
  \param t : Interpolation value between [0, 1].

  \return The interpolated quaternion using the LERP method.
*/
vpQuaternionVector vpQuaternionVector::lerp(const vpQuaternionVector &q0, const vpQuaternionVector &q1, double t)
{
  assert(t >= 0 && t <= 1);

  double cosHalfTheta = dot(q0, q1);
  vpQuaternionVector q1_ = q1;
  if (cosHalfTheta < 0) {
    cosHalfTheta = -cosHalfTheta;
    q1_ = -q1;
  }

  vpQuaternionVector qLerp;
  qLerp.x() = q0.x() - (t * (q0.x() - q1.x()));
  qLerp.y() = q0.y() - (t * (q0.y() - q1.y()));
  qLerp.z() = q0.z() - (t * (q0.z() - q1.z()));
  qLerp.w() = q0.w() - (t * (q0.w() - q1.w()));

  return qLerp;
}

/*!
  Compute Quaternion Normalized Linear intERPolation (NLERP).
  See the following references:
    - https://github.com/Rajawali/Rajawali/blob/3dd6e09af22de1889241083c1e82ec72ba85bd40/rajawali/src/main/java/org/rajawali3d/math/Quaternion.java#L898-L931
    - https://stackoverflow.com/a/46187052

  \note Shortest path will be use.

  \param q0 : Start quaternion.
  \param q1 : End quaternion.
  \param t : Interpolation value between [0, 1].

  \return The interpolated quaternion using the NLERP method.
*/
vpQuaternionVector vpQuaternionVector::nlerp(const vpQuaternionVector &q0, const vpQuaternionVector &q1, double t)
{
  assert(t >= 0 && t <= 1);

  vpQuaternionVector qLerp = lerp(q0, q1, t);
  qLerp.normalize();

  return qLerp;
}

/*!
  Compute Quaternion Spherical Linear intERPolation (SLERP).
  See the following references:
    - https://en.wikipedia.org/wiki/Slerp#Quaternion_Slerp
    - https://github.com/Rajawali/Rajawali/blob/3dd6e09af22de1889241083c1e82ec72ba85bd40/rajawali/src/main/java/org/rajawali3d/math/Quaternion.java#L850-L896

  \note Shortest path will be use.

  \param q0 : Start quaternion.
  \param q1 : End quaternion.
  \param t : Interpolation value between [0, 1].

  \return The interpolated quaternion using the SLERP method.
*/
vpQuaternionVector vpQuaternionVector::slerp(const vpQuaternionVector &q0, const vpQuaternionVector &q1, double t)
{
  assert(t >= 0 && t <= 1);
  // Some additional references:
  // https://splines.readthedocs.io/en/latest/rotation/slerp.html
  // https://zeux.io/2015/07/23/approximating-slerp/
  // https://github.com/eigenteam/eigen-git-mirror/blob/36b95962756c1fce8e29b1f8bc45967f30773c00/Eigen/src/Geometry/Quaternion.h#L753-L790
  // https://www.euclideanspace.com/maths/algebra/realNormedAlgebra/quaternions/slerp/index.htm
  // http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
  // https://www.3dgep.com/understanding-quaternions/
  // https://blog.magnum.graphics/backstage/the-unnecessarily-short-ways-to-do-a-quaternion-slerp/

  double cosHalfTheta = dot(q0, q1);
  vpQuaternionVector q1_ = q1;
  if (cosHalfTheta < 0) {
    cosHalfTheta = -cosHalfTheta;
    q1_ = -q1;
  }

  double scale0 = 1 - t;
  double scale1 = t;

  if ((1 - cosHalfTheta) > 0.1) {
    double theta = std::acos(cosHalfTheta);
    double invSinTheta = 1 / std::sin(theta);

    scale0 = std::sin((1 - t) * theta) * invSinTheta;
    scale1 = std::sin(t * theta) * invSinTheta;
  }

  vpQuaternionVector qSlerp;
  qSlerp.x() = (scale0 * q0.x()) + (scale1 * q1_.x());
  qSlerp.y() = (scale0 * q0.y()) + (scale1 * q1_.y());
  qSlerp.z() = (scale0 * q0.z()) + (scale1 * q1_.z());
  qSlerp.w() = (scale0 * q0.w()) + (scale1 * q1_.w());
  qSlerp.normalize();

  return qSlerp;
}
END_VISP_NAMESPACE
