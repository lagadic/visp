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
 * Rzyx angle parameterization for the rotation.
 * Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(x,psi)
 */

/*!
  \file vpRzyxVector.cpp
  \brief class that consider the case of the  Rzyx angle parameterization for
  the  rotation : Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta,Rot(x,psi)

*/

#include <math.h>
#include <visp3/core/vpRzyxVector.h>

BEGIN_VISP_NAMESPACE
const unsigned int vpRzyxVector::constr_val_3 = 3;
/*! Default constructor that initialize all the 3 angles to zero. */
vpRzyxVector::vpRzyxVector() : vpRotationVector(constr_val_3) { }

/*!
  Constructor from 3 angles (in radian).
  \param phi : \f$\varphi\f$ angle around the \f$z\f$ axis.
  \param theta : \f$\theta\f$ angle around the \f$y\f$ axis.
  \param psi : \f$\psi\f$ angle around the \f$x\f$ axis.
*/
vpRzyxVector::vpRzyxVector(double phi, double theta, double psi) : vpRotationVector(constr_val_3) { buildFrom(phi, theta, psi); }

/*!
  Constructor that initialize \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler
  angles from a rotation matrix.
  \param R : Rotation matrix used to initialize the Euler angles.
*/
vpRzyxVector::vpRzyxVector(const vpRotationMatrix &R) : vpRotationVector(constr_val_3) { buildFrom(R); }

/*!
  Constructor that initialize \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler
  angles vector from a \f$\theta {\bf u}\f$ vector.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as
  input to initialize the Euler angles.
*/
vpRzyxVector::vpRzyxVector(const vpThetaUVector &tu) : vpRotationVector(constr_val_3) { buildFrom(tu); }

/*! Copy constructor from a 3-dimension vector. */
vpRzyxVector::vpRzyxVector(const vpColVector &rzyx) : vpRotationVector(constr_val_3) { buildFrom(rzyx); }

/*! Copy constructor from a 3-dimension vector. */
vpRzyxVector::vpRzyxVector(const std::vector<double> &rzyx) : vpRotationVector(constr_val_3) { buildFrom(rzyx); }

/*!
  Convert a rotation matrix into a \f$ R_{zyx}=(\varphi,\theta,\psi) \f$ Euler angles vector.

  Source: R. Paul, Robot Manipulators: Mathematics, Programming, and Control.
  MIT Press, 1981, p. 71

  \param R : Rotation matrix used as input.
  \return Euler angles vector \f$ R_{zyx}=(\varphi,\theta,\psi) \f$.
*/
vpRzyxVector &vpRzyxVector::buildFrom(const vpRotationMatrix &R)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  double nx = R[index_0][index_0];
  double ny = R[index_1][index_0];

  double COEF_MIN_ROT = 1e-6;
  double phi;

  if ((fabs(nx) < COEF_MIN_ROT) && (fabs(ny) < COEF_MIN_ROT)) {
    phi = 0;
  }
  else {
    phi = atan2(ny, nx);
  }
  double si = sin(phi);
  double co = cos(phi);

  double nz = R[index_2][index_0];
  double theta = atan2(-nz, (co * nx) + (si * ny));

  double ax = R[index_0][index_2];
  double ay = R[index_1][index_2];
  double ox = R[index_0][index_1];
  double oy = R[index_1][index_1];

  double psi = atan2((si * ax) - (co * ay), (-si * ox) + (co * oy));

  buildFrom(phi, theta, psi);

  return *this;
}

/*!
  Convert a \f$\theta {\bf u}\f$ vector into a \f$ R_{zyx}=(\varphi,\theta,\psi) \f$ Euler angles vector.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as input.
  \return Euler angles vector \f$ R_{zyx}=(\varphi,\theta,\psi) \f$.
*/
vpRzyxVector &vpRzyxVector::buildFrom(const vpThetaUVector &tu)
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
vpRzyxVector &vpRzyxVector::buildFrom(const double &phi, const double &theta, const double &psi)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  data[index_0] = phi;
  data[index_1] = theta;
  data[index_2] = psi;
  return *this;
}

/*!
  Construct a \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler angles vector from a 3-dim vector.
*/
vpRzyxVector &vpRzyxVector::buildFrom(const vpColVector &rzyx)
{
  const unsigned int val_3 = 3;
  if (rzyx.size() != val_3) {
    throw(vpException(vpException::dimensionError, "Cannot construct a R-zyx vector from a %d-dimension col vector",
                      rzyx.size()));
  }
  for (unsigned int i = 0; i < val_3; ++i) {
    data[i] = rzyx[i];
  }

  return *this;
}

/*!
  Construct a \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler angles vector from a 3-dim vector.
*/
vpRzyxVector &vpRzyxVector::buildFrom(const std::vector<double> &rzyx)
{
  const unsigned int val_3 = 3;
  if (rzyx.size() != val_3) {
    throw(vpException(vpException::dimensionError, "Cannot construct a R-zyx vector from a %d-dimension std::vector",
                      rzyx.size()));
  }
  for (unsigned int i = 0; i < val_3; ++i) {
    data[i] = rzyx[i];
  }

  return *this;
}

/*!

  Initialize each element of the vector to the same angle value \e v.

  \param v : Angle value to set for each element of the vector.

  \code
  #include <visp3/core/vpMath.h>
  #include <visp3/core/vpRzyxVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
  for (unsigned int i = 0; i < dsize; ++i) {
    data[i] = v;
  }

  return *this;
}

/*!

  Copy operator that initializes a \f$R_{zyx}=(\varphi,\theta,\psi)\f$ Euler
  angles vector from a 3-dimension column vector.

  \param rzyx : 3-dimension vector containing the values of the rotation vector.

  \code
  #include <visp3/core/vpRzyxVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
  const unsigned int val_3 = 3;
  if (rzyx.size() != val_3) {
    throw(vpException(vpException::dimensionError, "Cannot set a R-zyx vector from a %d-dimension col vector",
                      rzyx.size()));
  }
  for (unsigned int i = 0; i < val_3; ++i) {
    data[i] = rzyx[i];
  }

  return *this;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  Set vector from a list of 3 double angle values in radians.
  \code
  #include <visp3/core/vpRzyxVector.cpp>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRzyxVector rzyx = {M_PI, 0, M_PI_2};
    std::cout << "rzyx: " << rzyx.t() << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  zyx: 3.141592654  0  1.570796327
  \endcode
  \sa operator<<()
*/
vpRzyxVector &vpRzyxVector::operator=(const std::initializer_list<double> &list)
{
  if (list.size() > size()) {
    throw(vpException(
      vpException::dimensionError,
      "Cannot set Euler x-y-z vector out of bounds. It has only %d values while you try to initialize with %d values",
      size(), list.size()));
  }
  std::copy(list.begin(), list.end(), data);
  return *this;
}
#endif
END_VISP_NAMESPACE
