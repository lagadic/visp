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
 * Theta U parameterization for the rotation.
 */

/*!
  \file vpThetaUVector.cpp
  \brief class that consider the case of the Theta U parameterization for the rotation
*/

#include <algorithm>                         // for copy
#include <vector>                            // for vector
#include <cmath>                             // for sin, sqrt, cos, fabs, acos
#include <limits>                            // for numeric_limits

#include <visp3/core/vpConfig.h>             // for BEGIN_VISP_NAMESPACE, END...

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
#include <initializer_list>                  // for initializer_list
#endif

#include <visp3/core/vpThetaUVector.h>       // for vpThetaUVector
#include <visp3/core/vpArray2D.h>            // for vpArray2D
#include <visp3/core/vpColVector.h>          // for vpColVector, operator*
#include <visp3/core/vpException.h>          // for vpException
#include <visp3/core/vpHomogeneousMatrix.h>  // for vpHomogeneousMatrix
#include <visp3/core/vpMath.h>               // for vpMath
#include <visp3/core/vpPoseVector.h>         // for vpPoseVector
#include <visp3/core/vpRotationMatrix.h>     // for vpRotationMatrix
#include <visp3/core/vpRotationVector.h>     // for vpRotationVector

class vpQuaternionVector;
class vpRxyzVector;
class vpRzyxVector;
class vpRzyzVector;

BEGIN_VISP_NAMESPACE
const double vpThetaUVector::minimum = 0.0001;

/*! Default constructor that initialize all the 3 angles to zero. */
vpThetaUVector::vpThetaUVector() : vpRotationVector(3) { }
/*! Copy constructor. */
vpThetaUVector::vpThetaUVector(const vpThetaUVector &tu) : vpRotationVector(tu) { }
/*! Copy constructor from a 3-dimension vector. */
vpThetaUVector::vpThetaUVector(const vpColVector &tu) : vpRotationVector(3) { buildFrom(tu); }
/*!
  Initialize a \f$\theta {\bf u}\f$ vector from an homogeneous matrix.
*/
vpThetaUVector::vpThetaUVector(const vpHomogeneousMatrix &M) : vpRotationVector(3) { buildFrom(M); }
/*!
  Initialize a \f$\theta {\bf u}\f$ vector from a pose vector.
*/
vpThetaUVector::vpThetaUVector(const vpPoseVector &p) : vpRotationVector(3) { buildFrom(p); }
/*!
  Initialize a \f$\theta {\bf u}\f$ vector from a rotation matrix.
*/
vpThetaUVector::vpThetaUVector(const vpRotationMatrix &R) : vpRotationVector(3) { buildFrom(R); }

/*!
  Initialize a \f$\theta {\bf u}\f$ vector from an Euler z-y-x representation vector.
*/
vpThetaUVector::vpThetaUVector(const vpRzyxVector &rzyx) : vpRotationVector(3) { buildFrom(rzyx); }
/*!
  Initialize a \f$\theta {\bf u}\f$ vector from an Euler z-y-z representation vector.
*/
vpThetaUVector::vpThetaUVector(const vpRzyzVector &rzyz) : vpRotationVector(3) { buildFrom(rzyz); }
/*!
  Initialize a \f$\theta {\bf u}\f$ vector from an Euler x-y-z representation vector.
*/
vpThetaUVector::vpThetaUVector(const vpRxyzVector &rxyz) : vpRotationVector(3) { buildFrom(rxyz); }
/*!
  Initialize a \f$\theta {\bf u}\f$ vector from a quaternion representation vector.
*/
vpThetaUVector::vpThetaUVector(const vpQuaternionVector &q) : vpRotationVector(3) { buildFrom(q); }

/*!
  Build a \f$\theta {\bf u}\f$ vector from 3 angles in radians.
  \code
  #include <visp3/core/vpThetaUVector.cpp>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpThetaUVector tu(0, M_PI_2, M_PI);
    std::cout << "tu: " << tu.t() << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  tu: 0  1.570796327  3.141592654
  \endcode
*/
vpThetaUVector::vpThetaUVector(double tux, double tuy, double tuz) : vpRotationVector(3) { buildFrom(tux, tuy, tuz); }

/*!
  Build a \f$\theta {\bf u}\f$ vector from a vector of 3 angles in radian.
*/
vpThetaUVector::vpThetaUVector(const std::vector<double> &tu) : vpRotationVector(3) { buildFrom(tu); }

/*!
  Converts an homogeneous matrix into a \f$\theta {\bf u}\f$ vector.
*/
vpThetaUVector &vpThetaUVector::buildFrom(const vpHomogeneousMatrix &M)
{
  vpRotationMatrix R;

  M.extract(R);
  buildFrom(R);

  return *this;
}
/*!
  Converts a pose vector into a \f$\theta {\bf u}\f$ vector copying
  the \f$\theta {\bf u}\f$ values contained in the pose vector.
*/
vpThetaUVector &vpThetaUVector::buildFrom(const vpPoseVector &p)
{
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    data[i] = p[i + 3];
  }

  return *this;
}

/*!
  Converts a rotation matrix into a \f$\theta {\bf u}\f$ vector.
*/
vpThetaUVector &vpThetaUVector::buildFrom(const vpRotationMatrix &R)
{
  double s, c, theta;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;

  s = ((R[1][0] - R[0][1]) * (R[1][0] - R[0][1])) + ((R[index_2][0] - R[0][index_2]) * (R[index_2][0] - R[0][index_2])) +
    ((R[index_2][index_1] - R[index_1][index_2]) * (R[index_2][index_1] - R[index_1][index_2]));
  s = sqrt(s) / 2.0;
  c = ((R[index_0][index_0] + R[index_1][index_1] + R[index_2][index_2]) - 1.0) / 2.0;
  theta = atan2(s, c); /* theta in [0, PI] since s > 0 */

  // General case when theta != pi. If theta=pi, c=-1
  if ((1 + c) > minimum) // Since -1 <= c <= 1, no fabs(1+c) is required
  {
    double sinc = vpMath::sinc(s, theta);

    data[index_0] = (R[index_2][index_1] - R[index_1][index_2]) / (2 * sinc);
    data[index_1] = (R[index_0][index_2] - R[index_2][index_0]) / (2 * sinc);
    data[index_2] = (R[index_1][index_0] - R[index_0][index_1]) / (2 * sinc);
  }
  else /* theta near PI */
  {
    double x = 0;
    if ((R[0][0] - c) > std::numeric_limits<double>::epsilon()) {
      x = sqrt((R[0][0] - c) / (1 - c));
    }

    double y = 0;
    if ((R[1][1] - c) > std::numeric_limits<double>::epsilon()) {
      y = sqrt((R[1][1] - c) / (1 - c));
    }

    double z = 0;
    if ((R[index_2][index_2] - c) > std::numeric_limits<double>::epsilon()) {
      z = sqrt((R[index_2][index_2] - c) / (1 - c));
    }

    if ((x > y) && (x > z)) {
      if ((R[index_2][index_1] - R[index_1][index_2]) < 0) {
        x = -x;
      }
      if ((vpMath::sign(x) * vpMath::sign(y)) != (vpMath::sign(R[0][1] + R[1][0]))) {
        y = -y;
      }
      if ((vpMath::sign(x) * vpMath::sign(z)) != (vpMath::sign(R[index_0][index_2] + R[index_2][index_0]))) {
        z = -z;
      }
    }
    else if (y > z) {
      if ((R[index_0][index_2] - R[index_2][index_0]) < 0) {
        y = -y;
      }
      if ((vpMath::sign(y) * vpMath::sign(x)) != (vpMath::sign(R[index_1][index_0] + R[index_0][index_1]))) {
        x = -x;
      }
      if ((vpMath::sign(y) * vpMath::sign(z)) != (vpMath::sign(R[index_1][index_2] + R[index_2][index_1]))) {
        z = -z;
      }
    }
    else {
      if ((R[1][0] - R[0][1]) < 0) {
        z = -z;
      }
      if ((vpMath::sign(z) * vpMath::sign(x)) != (vpMath::sign(R[index_2][index_0] + R[index_0][index_2]))) {
        x = -x;
      }
      if ((vpMath::sign(z) * vpMath::sign(y)) != (vpMath::sign(R[index_2][index_1] + R[index_1][index_2]))) {
        y = -y;
      }
    }
    data[index_0] = theta * x;
    data[index_1] = theta * y;
    data[index_2] = theta * z;
  }

  return *this;
}
/*!
  Build a \f$\theta {\bf u}\f$ vector from an Euler z-y-x representation vector.
*/
vpThetaUVector &vpThetaUVector::buildFrom(const vpRzyxVector &rzyx)
{
  vpRotationMatrix R(rzyx);

  buildFrom(R);
  return *this;
}
/*!
  Build a \f$\theta {\bf u}\f$ vector from an Euler z-y-z representation vector.
*/
vpThetaUVector &vpThetaUVector::buildFrom(const vpRzyzVector &rzyz)
{
  vpRotationMatrix R(rzyz);

  buildFrom(R);
  return *this;
}
/*!
  Build a \f$\theta {\bf u}\f$ vector from an Euler x-y-z representation vector.
*/
vpThetaUVector &vpThetaUVector::buildFrom(const vpRxyzVector &rxyz)
{
  vpRotationMatrix R(rxyz);

  buildFrom(R);
  return *this;
}

/*!
  Build a \f$\theta {\bf u}\f$ vector from a quaternion representation vector.
*/
vpThetaUVector &vpThetaUVector::buildFrom(const vpQuaternionVector &q)
{
  vpRotationMatrix R(q);

  buildFrom(R);
  return *this;
}

/*!
  Build a \f$\theta {\bf u}\f$ vector from a 3-dim vectors.
*/
vpThetaUVector &vpThetaUVector::buildFrom(const std::vector<double> &tu)
{
  if (tu.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot construct a theta-u vector from a %d-dimension std::vector",
                      tu.size()));
  }
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    data[i] = tu[i];
  }

  return *this;
}

/*!
  Build a \f$\theta {\bf u}\f$ vector from a 3-dim vector.
*/
vpThetaUVector &vpThetaUVector::buildFrom(const vpColVector &tu)
{
  if (tu.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot construct a theta-u vector from a %d-dimension std::vector",
                      tu.size()));
  }
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    data[i] = tu[i];
  }

  return *this;
}

/*!
  Build a \f$\theta {\bf u}\f$ vector from 3 angles in radian.
*/
vpThetaUVector &vpThetaUVector::buildFrom(const double &tux, const double &tuy, const double &tuz)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  data[index_0] = tux;
  data[index_1] = tuy;
  data[index_2] = tuz;
  return *this;
}

/*!

  Initialize each element of the \f$\theta {\bf u}\f$ vector to the
  same angle value \e v.

  \param v : Angle value to set for each element of the \f$\theta {\bf u}\f$ vector.

  \code
  #include <visp3/core/vpMath.h>
  #include <visp3/core/vpThetaUVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpThetaUVector tu;

    // Initialise the theta U rotation vector
    tu = vpMath::rad( 45.f); // All the 3 angles are set to 45 degrees
  }
  \endcode
*/
vpThetaUVector &vpThetaUVector::operator=(double v)
{
  for (unsigned int i = 0; i < dsize; ++i) {
    data[i] = v;
  }

  return *this;
}

/*!

  Copy operator that initializes a \f$\theta {\bf u}\f$ vector from a
  3-dimension column vector \e tu.

  \param tu : 3-dimension vector containing the values of the \f$\theta {\bf u}\f$  vector.

  \code
  #include <visp3/core/vpThetaUVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpColVector v(3);
    v[0] = 0.1;
    v[1] = 0.2;
    v[2] = 0.3;
    vpThetaUVector tu;
    tu = v;
    // tu is now equal to v : 0.1, 0.2, 0.3
  }
  \endcode
*/
vpThetaUVector &vpThetaUVector::operator=(const vpColVector &tu)
{
  if (tu.size() != size()) {
    throw(vpException(vpException::dimensionError, "Cannot set a theta-u vector from a %d-dimension col vector",
                      tu.size()));
  }

  unsigned int l_size = size();
  for (unsigned int i = 0; i < l_size; ++i) {
    data[i] = tu[i];
  }

  return *this;
}

/*!

  Extract the rotation angle \f$ \theta \f$ and the unit vector
  \f$\bf u \f$ from the \f$ \theta {\bf u} \f$ representation.

  \param theta : Rotation angle \f$ \theta \f$ in rad.

  \param u : 3-dim unit vector \f${\bf u} = (u_{x},u_{y},u_{z})^{\top} \f$
  representing the rotation axis.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpThetaUVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpHomogeneousMatrix M(0, 0, 1., vpMath::rad(10), vpMath::rad(20), vpMath::rad(30));

    double theta;
    vpColVector u;
    M.getRotationMatrix().getThetaUVector().extract(theta, u);
    std::cout << "theta: " << theta << std::endl;
    std::cout << "u    : " << u.t() << std::endl;
  }
  \endcode

  \sa getTheta(), getU()
*/
void vpThetaUVector::extract(double &theta, vpColVector &u) const
{
  u.resize(3);

  theta = getTheta();
  // --comment: if theta equals 0
  if (std::fabs(theta) <= std::numeric_limits<double>::epsilon()) {
    u = 0;
    return;
  }
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    u[i] = data[i] / theta;
  }
}

/*!

  Get the rotation angle \f$ \theta \f$ from the \f$ \theta {\bf u} \f$
  representation.

  \return Rotation angle \f$ \theta \f$ in rad.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpThetaUVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpHomogeneousMatrix M(0, 0, 1., vpMath::rad(10), vpMath::rad(20), vpMath::rad(30));

    std::cout << "theta: " << M.getRotationMatrix().getThetaUVector().getTheta() << std::endl;
    std::cout << "u    : " << M.getRotationMatrix().getThetaUVector().getU().t() << std::endl;
  }
  \endcode

  \sa getTheta(), extract()
*/
double vpThetaUVector::getTheta() const
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  return sqrt((data[index_0] * data[index_0]) + (data[index_1] * data[index_1]) + (data[index_2] * data[index_2]));
}

/*!

  Get the unit vector \f$\bf u \f$ from the \f$ \theta {\bf u} \f$
  representation.

  \return 3-dim unit vector \f${\bf u} = (u_{x},u_{y},u_{z})^{\top} \f$
  representing the rotation axis.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpThetaUVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpHomogeneousMatrix M(0, 0, 1., vpMath::rad(10), vpMath::rad(20), pMath::rad(30));

    std::cout << "theta: " << M.getRotationMatrix().getThetaUVector().getTheta() << std::endl;
    std::cout << "u    : " << M.getRotationMatrix().getThetaUVector().getU().t() << std::endl;
  }
  \endcode

  \sa getTheta(), extract()
*/
vpColVector vpThetaUVector::getU() const
{
  vpColVector u(3);

  double theta = getTheta();
  // --comment: if theta equals 0
  if (std::fabs(theta) <= std::numeric_limits<double>::epsilon()) {
    u = 0;
    return u;
  }
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    u[i] = data[i] / theta;
  }
  return u;
}

/*!
  Perform rotation chaining / rotation multiplication using the theta.u rotation representation.
  See: <a href="https://math.stackexchange.com/a/1978136">this answer</a> for some details about the maths.
*/
vpThetaUVector vpThetaUVector::operator*(const vpThetaUVector &tu_b) const
{
  double a_2 = getTheta() / 2;
  vpColVector a_hat = getU();
  double b_2 = tu_b.getTheta() / 2;
  vpColVector b_hat = tu_b.getU();

  vpColVector a_hat_sin_2 = a_hat * std::sin(a_2);
  vpColVector b_hat_sin_2 = b_hat * std::sin(b_2);
  double c = 2 * std::acos((std::cos(a_2) * std::cos(b_2)) - (vpColVector::dotProd(a_hat_sin_2, b_hat_sin_2)));
  vpColVector d = ((std::sin(a_2) * std::cos(b_2) * a_hat) + (std::cos(a_2) * std::sin(b_2) * b_hat)) +
    (std::sin(a_2) * std::sin(b_2) * vpColVector::crossProd(a_hat, b_hat));
  d = (c * d) / std::sin(c / 2);

  return vpThetaUVector(d);
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  Set vector from a list of 3 double angle values in radians.
  \code
  #include <visp3/core/vpThetaUVector.cpp>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpThetaUVector tu = {M_PI, 0, M_PI_2};
    std::cout << "tu: " << tu.t() << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  tu: 3.141592654  0  1.570796327
  \endcode
  \sa operator<<()
*/
vpThetaUVector &vpThetaUVector::operator=(const std::initializer_list<double> &list)
{
  if (list.size() > size()) {
    throw(vpException(
      vpException::dimensionError,
      "Cannot set theta u vector out of bounds. It has only %d values while you try to initialize with %d values",
      size(), list.size()));
  }
  std::copy(list.begin(), list.end(), data);
  return *this;
}
#endif
END_VISP_NAMESPACE
