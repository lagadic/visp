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
 * Rzyx angle parameterization for the rotation.
 * Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(x,psi)
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#include <math.h>
#include <visp3/core/vpRzyxVector.h>
/*!
  \file vpRzyxVector.cpp
  \brief class that consider the case of the  Rzyx angle parameterization for
  the  rotation : Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta,Rot(x,psi)

*/

/*! Default constructor that initialize all the 3 angles to zero. */
vpRzyxVector::vpRzyxVector() : vpRotationVector(3) {}

/*! Copy constructor. */
vpRzyxVector::vpRzyxVector(const vpRzyxVector &rzyx) : vpRotationVector(rzyx) {}

/*!
  Constructor from 3 angles (in radian).
  \param phi : \f$\varphi\f$ angle around the \f$z\f$ axis.
  \param theta : \f$\theta\f$ angle around the \f$y\f$ axis.
  \param psi : \f$\psi\f$ angle around the \f$x\f$ axis.
*/
vpRzyxVector::vpRzyxVector(const double phi, const double theta, const double psi) : vpRotationVector(3)
{
  buildFrom(phi, theta, psi);
}

/*!
  Constructor that initialize \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler
  angles from a rotation matrix.
  \param R : Rotation matrix used to initialize the Euler angles.
*/
vpRzyxVector::vpRzyxVector(const vpRotationMatrix &R) : vpRotationVector(3) { buildFrom(R); }

/*!
  Constructor that initialize \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler
  angles vector from a \f$\theta {\bf u}\f$ vector.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as
  input to initialize the Euler angles.
*/
vpRzyxVector::vpRzyxVector(const vpThetaUVector &tu) : vpRotationVector(3) { buildFrom(tu); }

/*! Copy constructor from a 3-dimension vector. */
vpRzyxVector::vpRzyxVector(const vpColVector &rzyx) : vpRotationVector(3)
{
  buildFrom(rzyx);
}

/*! Copy constructor from a 3-dimension vector. */
vpRzyxVector::vpRzyxVector(const std::vector<double> &rzyx) : vpRotationVector(3)
{
  buildFrom(rzyx);
}

/*!
  Convert a rotation matrix into a \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler
  angles vector.

  Source: R. Paul, Robot Manipulators: Mathematics, Programming, and Control.
  MIT Press, 1981, p. 71

  \param R : Rotation matrix used as input.
  \return \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler angles vector.
*/
vpRzyxVector vpRzyxVector::buildFrom(const vpRotationMatrix &R)
{
  double nx = R[0][0];
  double ny = R[1][0];

  double phi = atan2(ny, nx);
  double si = sin(phi);
  double co = cos(phi);

  double nz = R[2][0];
  double theta = atan2(-nz, co * nx + si * ny);

  double ax = R[0][2];
  double ay = R[1][2];
  double ox = R[0][1];
  double oy = R[1][1];

  double psi = atan2(si * ax - co * ay, -si * ox + co * oy);

  buildFrom(phi, theta, psi);

  return *this;
}

/*!
  Convert a \f$\theta {\bf u}\f$ vector into a
  \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler angles vector.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as input.
  \return \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler angles vector.
*/
vpRzyxVector vpRzyxVector::buildFrom(const vpThetaUVector &tu)
{
  vpRotationMatrix R;
  R.buildFrom(tu);
  buildFrom(R);

  return *this;
}

/*!
  Construction from 3 angles (in radian).
  \param phi : \f$\varphi\f$ angle around the \f$z\f$ axis.
  \param theta : \f$\theta\f$ angle around the \f$y\f$ axis.
  \param psi : \f$\psi\f$ angle around the \f$x\f$ axis.
*/
void vpRzyxVector::buildFrom(const double phi, const double theta, const double psi)
{
  data[0] = phi;
  data[1] = theta;
  data[2] = psi;
}

/*!
  Construct a \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler angles vectorfrom a 3-dim vector.
*/
vpRzyxVector vpRzyxVector::buildFrom(const vpColVector &rzyx)
{
  if (rzyx.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot construct a R-zyx vector from a %d-dimension col vector",
                      rzyx.size()));
  }
  for (unsigned int i = 0; i < 3; i++)
    data[i] = rzyx[i];


  return *this;
}

/*!
  Construct a \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler angles vector from a 3-dim vector.
*/
vpRzyxVector vpRzyxVector::buildFrom(const std::vector<double> &rzyx)
{
  if (rzyx.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot construct a R-zyx vector from a %d-dimension std::vector",
                      rzyx.size()));
  }
  for (unsigned int i = 0; i < 3; i++)
    data[i] = rzyx[i];

  return *this;
}

/*!

  Initialize each element of the vector to the same angle value \e v.

  \param v : Angle value to set for each element of the vector.

\code
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRzyxVector.h>

int main()
{
  vpRzyxVector v;

  // Initialise the rotation vector
  v = vpMath::rad( 45.f); // All the 3 angles are set to 45 degrees
}
\endcode
*/
vpRzyxVector &vpRzyxVector::operator=(double v)
{
  for (unsigned int i = 0; i < dsize; i++)
    data[i] = v;

  return *this;
}

/*!

  Copy operator that initializes a \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler
  angles vector from a 3-dimension column vector.

  \param rzyx : 3-dimension vector containing the values of the rotation
vector.

\code
#include <visp3/core/vpRzyxVector.h>

int main()
{
  vpColVector v(3);
  v[0] = 0.1;
  v[1] = 0.2;
  v[2] = 0.3;
  vpRzyxVector rzyx;
  rzyx = v;
  // rzyx is now equal to v : 0.1, 0.2, 0.3
}
\endcode
*/
vpRzyxVector &vpRzyxVector::operator=(const vpColVector &rzyx)
{
  if (rzyx.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot set a R-zyx vector from a %d-dimension col vector",
                      rzyx.size()));
  }
  for (unsigned int i = 0; i < 3; i++)
    data[i] = rzyx[i];

  return *this;
}
