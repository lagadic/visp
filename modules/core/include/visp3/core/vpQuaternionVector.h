/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Quaternion definition.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#ifndef __QUATERNIONVECTOR_H__
#define __QUATERNIONVECTOR_H__

/*!
  \file vpQuaternionVector.h

  \brief Class that consider the case of a quaternion and basic
   operations on it.

*/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRotationVector.h>
#include <visp3/core/vpThetaUVector.h>

/*!
  \class vpQuaternionVector

  \ingroup group_core_transformations

  \brief Implementation of a rotation vector as quaternion angle
  minimal representation.

  Defines a quaternion and its basic operations.

  The vpQuaternionVector class is derived from vpRotationVector.

  A quaternion is defined by four values: \f${\bf q} = (x, y, z, w)\f$.

  This class allows to compute a quaternion from a rotation matrix
  using either vpQuaternionVector(const vpRotationMatrix &) constructor
  or buildFrom() method.

  It also defines common operations on a quaternion such as:
  - multiplication (scalar and quaternion)
  - addition
  - substraction.

  */
class VISP_EXPORT vpQuaternionVector : public vpRotationVector
{
private:
  static const double minimum;

public:
  vpQuaternionVector();
  vpQuaternionVector(const vpQuaternionVector &q);
  vpQuaternionVector(const double qx, const double qy, const double qz, const double qw);
  explicit vpQuaternionVector(const vpRotationMatrix &R);
  explicit vpQuaternionVector(const vpThetaUVector &tu);
  explicit vpQuaternionVector(const vpColVector &q);

  //! Destructor.
  virtual ~vpQuaternionVector(){};

  vpQuaternionVector buildFrom(const double qx, const double qy, const double qz, const double qw);
  vpQuaternionVector buildFrom(const vpRotationMatrix &R);
  vpQuaternionVector buildFrom(const vpThetaUVector &tu);
  void set(const double x, const double y, const double z, const double w);

  double x() const;
  double y() const;
  double z() const;
  double w() const;

  vpQuaternionVector operator+(const vpQuaternionVector &q) const;
  vpQuaternionVector operator-(const vpQuaternionVector &q) const;
  vpQuaternionVector operator-() const;
  vpQuaternionVector operator*(const double l) const;
  vpQuaternionVector operator*(const vpQuaternionVector &rq) const;
  vpQuaternionVector operator/(const double l) const;
  vpQuaternionVector &operator=(const vpColVector &q);

  vpQuaternionVector conjugate() const;
  vpQuaternionVector inverse() const;
  double magnitude() const;
  void normalize();
};

#endif
