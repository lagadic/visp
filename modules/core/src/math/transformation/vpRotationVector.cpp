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
 * Generic rotation vector (cannot be used as is !).
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <algorithm>
#include <math.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpRotationVector.h>
#include <visp3/core/vpRowVector.h>

/*!
  \file vpRotationVector.cpp
  \brief Class that consider the case of a generic rotation vector
  (cannot be used as is !).
*/

/*!
  Return the transpose of the rotation vector.

*/
vpRowVector vpRotationVector::t() const
{
  vpRowVector v(dsize);

  for (unsigned int i = 0; i < dsize; i++)
    v[i] = data[i];

  return v;
}

/*!
 * Converts the vpRotationVector to a std::vector.
 * \return The corresponding std::vector<double>.
 */
std::vector<double> vpRotationVector::toStdVector()
{
  std::vector<double> v(this->size());

  for (unsigned int i = 0; i < this->size(); i++)
    v[i] = data[i];
  return v;
}

/*!
  Operator that allows to multiply each element of a rotation vector by a
  scalar.

  \param x : The scalar.

  \return The rotation vector multiplied by the scalar as a column vector. The
  current rotation vector (*this) is unchanged.

*/
vpColVector vpRotationVector::operator*(double x) const
{
  vpColVector v(dsize);

  for (unsigned int i = 0; i < dsize; i++)
    v[i] = (*this)[i] * x;
  return v;
}

/*!
  \relates vpRotationVector
  Allows to multiply a scalar by rotaion vector.
*/
vpColVector operator*(const double &x, const vpRotationVector &v)
{
  vpColVector vout;
  vout = v * x;
  return vout;
}

/*!
  Return the sum square of all the elements \f$r_{i}\f$ of the rotation vector
  r(m).

  \return The value \f[\sum{i=0}^{m} r_i^{2}\f].
  */
double vpRotationVector::sumSquare() const
{
  double sum_square = 0.0;

  for (unsigned int i = 0; i < rowNum; i++) {
    double x = rowPtrs[i][0];
    sum_square += x * x;
  }

  return sum_square;
}
