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
 * Quaternion definition.
 */

/*!
 * \file vpQuaternionVector.h
 *
 * \brief Class that consider the case of a quaternion and basic
 * operations on it.
 */

#ifndef VP_QUATERNION_VECTOR_H
#define VP_QUATERNION_VECTOR_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRotationVector.h>
#include <visp3/core/vpThetaUVector.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpQuaternionVector

  \ingroup group_core_transformations

  \brief Implementation of a rotation vector as quaternion angle
  minimal representation.

  Defines a quaternion and its basic operations.

  The vpQuaternionVector class is derived from vpRotationVector.

  A quaternion is defined by four double values: \f${\bf q} = (x, y, z, w)\f$.

  This class allows to compute a quaternion from a rotation matrix
  using either vpQuaternionVector(const vpRotationMatrix &) constructor
  or build() method.

  It also defines common operations on a quaternion such as:
  - multiplication (scalar and quaternion)
  - addition
  - subtraction.

  You can set values accessing each element:
  \code
  vpQuaternionVector q;
  q[0] = x
  q[1] = y;
  q[2] = z;
  q[3] = w;
  \endcode
  You can also initialize the vector using operator<<(double):
  \code
  tu << x, y, z, w;
  \endcode
  Or you can also initialize the vector from a list of doubles if ViSP is build with c++11 enabled:
  \code
  tu = {x, y, z, w};
  \endcode

  To get the values use:
  \code
  double x = q[0];
  double y = q[1];
  double z = q[2];
  double w = q[3];
  \endcode
  or use getter:
  \code
  double x = q.x();
  double y = q.y();
  double z = q.z();
  double w = q.w();
  \endcode
*/
class VISP_EXPORT vpQuaternionVector : public vpRotationVector
{
public:
  vpQuaternionVector();
  vpQuaternionVector(const vpQuaternionVector &q);
  vpQuaternionVector(const double qx, const double qy, const double qz, const double qw);
  VP_EXPLICIT vpQuaternionVector(const vpRotationMatrix &R);
  VP_EXPLICIT vpQuaternionVector(const vpThetaUVector &tu);
  VP_EXPLICIT vpQuaternionVector(const vpColVector &q);
  VP_EXPLICIT vpQuaternionVector(const std::vector<double> &q);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  VP_DEPRECATED vpQuaternionVector buildFrom(const double qx, const double qy, const double qz, const double qw);
  VP_DEPRECATED vpQuaternionVector buildFrom(const vpRotationMatrix &R);
  VP_DEPRECATED vpQuaternionVector buildFrom(const vpThetaUVector &tu);
  VP_DEPRECATED vpQuaternionVector buildFrom(const vpColVector &q);
  VP_DEPRECATED vpQuaternionVector buildFrom(const std::vector<double> &q);
#endif
  vpQuaternionVector &build(const double &qx, const double &qy, const double &qz, const double &qw);
  vpQuaternionVector &build(const vpRotationMatrix &R);
  vpQuaternionVector &build(const vpThetaUVector &tu);
  vpQuaternionVector &build(const vpColVector &q);
  vpQuaternionVector &build(const std::vector<double> &q);

  void set(double x, double y, double z, double w);

  const double &x() const;
  const double &y() const;
  const double &z() const;
  const double &w() const;

  double &x();
  double &y();
  double &z();
  double &w();

  vpQuaternionVector operator+(const vpQuaternionVector &q) const;
  vpQuaternionVector operator-(const vpQuaternionVector &q) const;
  vpQuaternionVector operator-() const;
  vpQuaternionVector operator*(double l) const;
  vpQuaternionVector operator*(const vpQuaternionVector &rq) const;
  vpQuaternionVector operator/(double l) const;
  vpQuaternionVector &operator=(const vpColVector &q);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpQuaternionVector &operator=(const vpQuaternionVector &) = default;
  vpQuaternionVector &operator=(const std::initializer_list<double> &list);
#endif

  vpQuaternionVector conjugate() const;
  vpQuaternionVector inverse() const;
  double magnitude() const;
  void normalize();

  static double dot(const vpQuaternionVector &q0, const vpQuaternionVector &q1);

  static vpQuaternionVector lerp(const vpQuaternionVector &q0, const vpQuaternionVector &q1, double t);
  static vpQuaternionVector nlerp(const vpQuaternionVector &q0, const vpQuaternionVector &q1, double t);
  static vpQuaternionVector slerp(const vpQuaternionVector &q0, const vpQuaternionVector &q1, double t);

private:
  static const double minimum;

};
END_VISP_NAMESPACE
#endif
