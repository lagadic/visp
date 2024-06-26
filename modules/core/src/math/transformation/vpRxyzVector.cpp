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
 * Rxyz angle parameterization for the rotation.
 * Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta)Rot(z,psi).
 */

/*!
  \file vpRxyzVector.cpp
  \brief class that consider the case of the  Rxyz angle parameterization for
  the rotation : Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta)Rot(z,psi)
*/

#include <math.h>

#include <visp3/core/vpRxyzVector.h>

BEGIN_VISP_NAMESPACE
/*! Default constructor that initialize all the 3 angles to zero. */
vpRxyzVector::vpRxyzVector() : vpRotationVector(3) { }

/*! Copy constructor. */
vpRxyzVector::vpRxyzVector(const vpRxyzVector &rxyz) : vpRotationVector(rxyz) { }

/*!
  Constructor from 3 angles (in radian).
  \param phi : \f$\varphi\f$ angle around the \f$x\f$ axis.
  \param theta : \f$\theta\f$ angle around the \f$y\f$ axis.
  \param psi : \f$\psi\f$ angle around the \f$z\f$ axis.
*/
vpRxyzVector::vpRxyzVector(double phi, double theta, double psi) : vpRotationVector(3) { build(phi, theta, psi); }

/*!
  Constructor that initialize \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler
  angles from a rotation matrix.
  \param R : Rotation matrix used to initialize the Euler angles.
*/
vpRxyzVector::vpRxyzVector(const vpRotationMatrix &R) : vpRotationVector(3) { build(R); }

/*!
  Constructor that initialize \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler
  angles vector from a \f$\theta {\bf u}\f$ vector.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as
  input to initialize the Euler angles.
*/
vpRxyzVector::vpRxyzVector(const vpThetaUVector &tu) : vpRotationVector(3) { build(tu); }

/*! Copy constructor from a 3-dimension vector. */
vpRxyzVector::vpRxyzVector(const vpColVector &rxyz) : vpRotationVector(3) { build(rxyz); }

/*! Copy constructor from a 3-dimension vector. */
vpRxyzVector::vpRxyzVector(const std::vector<double> &rxyz) : vpRotationVector(3) { build(rxyz); }

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  \deprecated You should use build(const vpRotationMatrix &) instead.
  Convert a rotation matrix into a \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler
  angles vector.

  \param R : Rotation matrix used as input.
  \return \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector.
*/
vpRxyzVector vpRxyzVector::buildFrom(const vpRotationMatrix &R)
{
  build(R);
  return *this;
}

/*!
  \deprecated You should use build(const vpThetaUVector &) instead.
  Convert a \f$\theta {\bf u}\f$ vector into a
  \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as input.
  \return \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector.
*/
vpRxyzVector vpRxyzVector::buildFrom(const vpThetaUVector &tu)
{
  build(tu);
  return *this;
}

/*!
  \deprecated You should use build(const double &, const double &, const double &) instead.
  Construction from 3 angles (in radian).
  \param phi : \f$\varphi\f$ angle around the \f$x\f$ axis.
  \param theta : \f$\theta\f$ angle around the \f$y\f$ axis.
  \param psi : \f$\psi\f$ angle around the \f$z\f$ axis.
*/
void vpRxyzVector::buildFrom(double phi, double theta, double psi)
{
  build(phi, theta, psi);
}

/*!
  \deprecated You should use build(const vpColVector &) instead.
  Construct a \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector from a 3-dim vector.
*/
vpRxyzVector vpRxyzVector::buildFrom(const vpColVector &rxyz)
{
  build(rxyz);
  return *this;
}

/*!
  \deprecated You should use build(const std::vector<double> &) instead.
  Construct a \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector from a 3-dim vector.
*/
vpRxyzVector vpRxyzVector::buildFrom(const std::vector<double> &rxyz)
{
  build(rxyz);
  return *this;
}
#endif

/*!
  Convert a rotation matrix into a \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler
  angles vector.

  \param R : Rotation matrix used as input.
  \return \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector.
*/
vpRxyzVector &vpRxyzVector::build(const vpRotationMatrix &R)
{
  double COEF_MIN_ROT = 1e-6;
  double phi;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;

  if ((fabs(R[index_1][index_2]) < COEF_MIN_ROT) && (fabs(R[index_2][index_2]) < COEF_MIN_ROT)) {
    phi = 0;
  }
  else {
    phi = atan2(-R[index_1][index_2], R[index_2][index_2]);
  }

  double si = sin(phi);
  double co = cos(phi);
  double theta = atan2(R[index_0][index_2], (-si * R[index_1][index_2]) + (co * R[index_2][index_2]));
  double psi = atan2((co * R[index_1][index_0]) + (si * R[index_2][index_0]), (co * R[index_1][index_1]) + (si * R[index_2][index_1]));

  build(phi, theta, psi);

  return *this;
}

/*!
  Convert a \f$\theta {\bf u}\f$ vector into a
  \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector.
  \param tu : \f$\theta {\bf u}\f$ representation of a rotation used here as input.
  \return \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector.
*/
vpRxyzVector &vpRxyzVector::build(const vpThetaUVector &tu)
{
  vpRotationMatrix R;
  R.build(tu);
  build(R);

  return *this;
}

/*!
  Construction from 3 angles (in radian).
  \param phi : \f$\varphi\f$ angle around the \f$x\f$ axis.
  \param theta : \f$\theta\f$ angle around the \f$y\f$ axis.
  \param psi : \f$\psi\f$ angle around the \f$z\f$ axis.
*/
vpRxyzVector &vpRxyzVector::build(const double &phi, const double &theta, const double &psi)
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
  Construct a \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector from a 3-dim vector.
*/
vpRxyzVector &vpRxyzVector::build(const vpColVector &rxyz)
{
  if (rxyz.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot construct a R-xyz vector from a %d-dimension col vector",
                      rxyz.size()));
  }
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    data[i] = rxyz[i];
  }

  return *this;
}

/*!
  Construct a \f$R_{xyz}=(\varphi,\theta,\psi)\f$ Euler angles vector from a 3-dim vector.
*/
vpRxyzVector &vpRxyzVector::build(const std::vector<double> &rxyz)
{
  if (rxyz.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot construct a R-xyz vector from a %d-dimension std::vector",
                      rxyz.size()));
  }
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    data[i] = rxyz[i];
  }

  return *this;
}

/*!

  Initialize each element of the vector to the same angle value \e v.

  \param v : Angle value to set for each element of the vector.

  \code
  #include <visp3/core/vpMath.h>
  #include <visp3/core/vpRxyzVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRxyzVector v;

    // Initialise the rotation vector
    v = vpMath::rad( 45.f); // All the 3 angles are set to 45 degrees
  }
  \endcode
*/
vpRxyzVector &vpRxyzVector::operator=(double v)
{
  for (unsigned int i = 0; i < dsize; ++i) {
    data[i] = v;
  }

  return *this;
}

/*!

  Copy operator that initializes a \f$R_{xyz}=(\varphi,\theta,\psi)\f$
  Euler angles vector from a 3-dimension column vector.

  \param rxyz : 3-dimension vector containing the values of the rotation vector.

  \code
  #include <visp3/core/vpRxyzVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpColVector v(3);
    v[0] = 0.1;
    v[1] = 0.2;
    v[2] = 0.3;
    vpRxyzVector rxyz;
    rxyz = v;
    // rxyz is now equal to v : 0.1, 0.2, 0.3
  }
  \endcode
*/
vpRxyzVector &vpRxyzVector::operator=(const vpColVector &rxyz)
{
  if (rxyz.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot set a R-xyz vector from a %d-dimension col vector",
                      rxyz.size()));
  }
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    data[i] = rxyz[i];
  }

  return *this;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  Set vector from a list of 3 double angle values in radians.
  \code
  #include <visp3/core/vpRxyzVector.cpp>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRxyzVector rxyz = {M_PI, 0, M_PI_2};
    std::cout << "rxyz: " << rxyz.t() << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  rxyz: 3.141592654  0  1.570796327
  \endcode
  \sa operator<<()
*/
vpRxyzVector &vpRxyzVector::operator=(const std::initializer_list<double> &list)
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
