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
 * Euler angles parameterization for the rotation.
 * Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <math.h>
#include <visp3/core/vpRzyzVector.h>

/*!
  \file vpRzyzVector.cpp
  \brief class that consider the case of the Rzyz angle parameterization for
  the rotation : Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
*/

/*! Default constructor that initialize all the 3 angles to zero. */
vpRzyzVector::vpRzyzVector() : vpRotationVector(3) {}
/*! Copy constructor. */
vpRzyzVector::vpRzyzVector(const vpRzyzVector &rzyz) : vpRotationVector(rzyz) {}

/*!
  Constructor from 3 angles (in radian).
  \param phi : \f$\varphi\f$ angle around the \f$z\f$ axis.
  \param theta : \f$\theta\f$ angle around the \f$y\f$ axis.
  \param psi : \f$\psi\f$ angle around the \f$z\f$ axis.
*/
vpRzyzVector::vpRzyzVector(const double phi, const double theta, const double psi) : vpRotationVector(3)
{
  buildFrom(phi, theta, psi);
}

/*!
  Constructor that initialize \f$R_{zyz}=(\varphi,\theta,\psi)\f$ Euler
  angles from a rotation matrix.
  \param R : Rotation matrix used to initialize the Euler angles.
*/
vpRzyzVector::vpRzyzVector(const vpRotationMatrix &R) : vpRotationVector(3) { buildFrom(R); }

/*!
  Constructor that initialize \f$R_{zyz}=(\varphi,\theta,\psi)\f$ Euler
  angles vector from a \f$\theta {\bf u}\f$ vector.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as
  input to initialize the Euler angles.
*/
vpRzyzVector::vpRzyzVector(const vpThetaUVector &tu) : vpRotationVector(3) { buildFrom(tu); }

/*! Copy constructor from a 3-dimension vector. */
vpRzyzVector::vpRzyzVector(const vpColVector &rzyz) : vpRotationVector(3)
{
  buildFrom(rzyz);
}

/*! Copy constructor from a 3-dimension vector. */
vpRzyzVector::vpRzyzVector(const std::vector<double> &rzyz) : vpRotationVector(3)
{
  buildFrom(rzyz);
}

/*!
  Convert a rotation matrix into a \f$R_{zyz}=(\varphi,\theta,\psi)\f$ Euler
  angles vector.

  \param R : Rotation matrix used as input.
  \return \f$R_{zyz}=(\varphi,\theta,\psi)\f$ Euler angles vector.
*/
vpRzyzVector vpRzyzVector::buildFrom(const vpRotationMatrix &R)
{
  double phi;
  if ((fabs(R[1][2]) < 1e-6) && (fabs(R[0][2]) < 1e-6))
    phi = 0;
  else
    phi = atan2(R[1][2], R[0][2]);
  double cphi = cos(phi);
  double sphi = sin(phi);

  double theta = atan2(cphi * R[0][2] + sphi * R[1][2], R[2][2]);

  double psi = atan2(-sphi * R[0][0] + cphi * R[1][0], -sphi * R[0][1] + cphi * R[1][1]);

  buildFrom(phi, theta, psi);

  return *this;
}

/*!
  Convert a \f$\theta {\bf u}\f$ vector into a
  \f$R_{zyz}=(\varphi,\theta,\psi)\f$ Euler angles vector. \param tu :
  \f$\theta {\bf u}\f$ representation of a rotation used here as input.
  \return \f$R_{zyz}=(\varphi,\theta,\psi)\f$ Euler angles vector.
*/
vpRzyzVector vpRzyzVector::buildFrom(const vpThetaUVector &tu)
{
  vpRotationMatrix R;
  R.buildFrom(tu);
  buildFrom(R);

  return *this;
}

/*!
  Construct a \f$R_{zyz}=(\varphi,\theta,\psi)\f$ Euler angles vectorfrom a 3-dim vector.
*/
vpRzyzVector vpRzyzVector::buildFrom(const vpColVector &rzyz)
{
  if (rzyz.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot construct a R-zyz vector from a %d-dimension col vector",
                      rzyz.size()));
  }
  for (unsigned int i = 0; i < 3; i++)
    data[i] = rzyz[i];


  return *this;
}

/*!
  Construct a \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler angles vector from a 3-dim vector.
*/
vpRzyzVector vpRzyzVector::buildFrom(const std::vector<double> &rzyz)
{
  if (rzyz.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot construct a R-zyx vector from a %d-dimension std::vector",
                      rzyz.size()));
  }
  for (unsigned int i = 0; i < 3; i++)
    data[i] = rzyz[i];

  return *this;
}

/*!

  Initialize each element of the vector to the same angle value \e v.

  \param v : Angle value to set for each element of the vector.

\code
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRzyzVector.h>

int main()
{
  vpRzyzVector r;

  // Initialise the rotation vector
  r = vpMath::rad( 45.f); // All the 3 angles are set to 45 degrees
}
\endcode
*/
vpRzyzVector &vpRzyzVector::operator=(double v)
{
  for (unsigned int i = 0; i < dsize; i++)
    data[i] = v;

  return *this;
}

/*!
  Construction from 3 angles (in radian).
  \param phi : \f$\varphi\f$ angle around the \f$z\f$ axis.
  \param theta : \f$\theta\f$ angle around the \f$y\f$ axis.
  \param psi : \f$\psi\f$ angle around the \f$z\f$ axis.
*/
void vpRzyzVector::buildFrom(const double phi, const double theta, const double psi)
{
  data[0] = phi;
  data[1] = theta;
  data[2] = psi;
}

/*!

  Copy operator that initializes a \f$R_{zyz}=(\varphi,\theta,\psi)\f$
  Euler angles vector from a 3-dimension column vector.

  \param rzyz : 3-dimension vector containing the values of the rotation
vector.

\code
#include <visp3/core/vpRzyzVector.h>

int main()
{
  vpColVector v(3);
  v[0] = 0.1;
  v[1] = 0.2;
  v[2] = 0.3;
  vpRzyxVector rzyz;
  rzyz = v;
  // rzyz is now equal to v : 0.1, 0.2, 0.3
}
\endcode
*/
vpRzyzVector &vpRzyzVector::operator=(const vpColVector &rzyz)
{
  if (rzyz.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot set a R-zyz vector from a %d-dimension col vector",
                      rzyz.size()));
  }
  for (unsigned int i = 0; i < 3; i++)
    data[i] = rzyz[i];

  return *this;
}
