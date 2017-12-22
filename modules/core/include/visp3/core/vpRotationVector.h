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
 * Generic rotation vector (cannot be used as is !).
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpRotationVECTOR_H
#define vpRotationVECTOR_H

/*!
  \file vpRotationVector.h
  \brief class that consider the case of a generic rotation vector
  (cannot be used as is !)
*/

#include <iostream>
#include <math.h>
#include <stdio.h>

#include <visp3/core/vpArray2D.h>

class vpRowVector;
class vpColVector;

/*!
  \class vpRotationVector

  \ingroup group_core_transformations

  \brief Implementation of a generic rotation vector.

  Class that consider the case of a generic rotation vector
  (cannot be used as is !) consisting in three or four angles.

  The vpRotationVector class is derived from vpArray2D<double>.
  The vpRotationVector class is also the base class of specific rotations
vectors such as vpThetaUVector, vpRxyzVector, vpRzyxVector, vpRzyzVector and
vpQuaternionVector.

  The code below shows how this class can be used to manipulate a
vpRxyzVector.

  \code
#include <iostream>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRxyzVector.h>

int main()
{
  vpRxyzVector r;         // By default initialized to zero
  // Rotation around x set to 45 degres converted in radians
  r[0] = vpMath::rad(45);
  // Rotation around y set to PI radians
  r[1] = M_PI;
  // Rotation around z set to 0 radians
  r[2] = 0;

  std::cout << "Rxyz rotation vector: " << r << std::endl;

  double rx = r[0];       // Get the value of the angle around x axis
  double ry = r[1];       // Get the value of the angle around y axis
  double rz = r[2];       // Get the value of the angle around z axis
}
  \endcode

*/

class VISP_EXPORT vpRotationVector : public vpArray2D<double>
{
public:
  //! Constructor that constructs a 0-size rotation vector.
  vpRotationVector() : vpArray2D<double>() {}

  //! Constructor that constructs a vector of size n and initialize all values
  //! to zero.
  explicit vpRotationVector(const unsigned int n) : vpArray2D<double>(n, 1) {}

  /*!
    Copy operator.
  */
  vpRotationVector(const vpRotationVector &v) : vpArray2D<double>(v) {}

  /*!
    Destructor.
  */
  virtual ~vpRotationVector(){};

  /** @name Inherited functionalities from vpRotationVector */
  //@{

  /*!
    Operator that allows to set the value of an element of the rotation
    vector: r[i] = value
  */
  inline double &operator[](unsigned int i) { return *(data + i); }
  /*!
    Operator that allows to get the value of an element of the rotation
    vector: value = r[i]
  */
  inline const double &operator[](unsigned int i) const { return *(data + i); }

  /*!
    Affectation of two vectors.
  */
  vpRotationVector &operator=(const vpRotationVector &v)
  {
    resize(v.size(), 1);
    for (unsigned int i = 0; i < v.size(); i++) {
      data[i] = v.data[i];
    }
    return *this;
  }
  vpColVector operator*(double x) const;

  double sumSquare() const;

  // Transpose of the rotation vector.
  vpRowVector t() const;

  //@}
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS
VISP_EXPORT
#endif
vpColVector operator*(const double &x, const vpRotationVector &v);

#endif
