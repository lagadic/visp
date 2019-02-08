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
 * Theta U parameterization for the rotation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
\file vpThetaUVector.cpp
\brief class that consider the case of the Theta U parameterization for the
rotation
*/

#include <cmath>  // std::fabs
#include <limits> // numeric_limits

#include <visp3/core/vpThetaUVector.h>

const double vpThetaUVector::minimum = 0.0001;

/*! Default constructor that initialize all the 3 angles to zero. */
vpThetaUVector::vpThetaUVector() : vpRotationVector(3) {}
/*! Copy constructor. */
vpThetaUVector::vpThetaUVector(const vpThetaUVector &tu) : vpRotationVector(tu) {}
/*! Copy constructor from a 3-dimension vector. */
vpThetaUVector::vpThetaUVector(const vpColVector &tu) : vpRotationVector(3)
{
  buildFrom(tu);
}

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
  Build a \f$\theta {\bf u}\f$ vector from 3 angles in radian.
*/
vpThetaUVector::vpThetaUVector(const double tux, const double tuy, const double tuz) : vpRotationVector(3)
{
  buildFrom(tux, tuy, tuz);
}

/*!
  Build a \f$\theta {\bf u}\f$ vector from a vector of 3 angles in radian.
*/
vpThetaUVector::vpThetaUVector(const std::vector<double> &tu)
{
  buildFrom(tu);
}

/*!
  Converts an homogeneous matrix into a \f$\theta {\bf u}\f$ vector.
*/
vpThetaUVector vpThetaUVector::buildFrom(const vpHomogeneousMatrix &M)
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
vpThetaUVector vpThetaUVector::buildFrom(const vpPoseVector &p)
{
  for (unsigned int i = 0; i < 3; i++)
    data[i] = p[i + 3];

  return *this;
}

/*!
  Converts a rotation matrix into a \f$\theta {\bf u}\f$ vector.
*/
vpThetaUVector vpThetaUVector::buildFrom(const vpRotationMatrix &R)
{
  double s, c, theta;

  s = (R[1][0] - R[0][1]) * (R[1][0] - R[0][1]) + (R[2][0] - R[0][2]) * (R[2][0] - R[0][2]) +
      (R[2][1] - R[1][2]) * (R[2][1] - R[1][2]);
  s = sqrt(s) / 2.0;
  c = (R[0][0] + R[1][1] + R[2][2] - 1.0) / 2.0;
  theta = atan2(s, c); /* theta in [0, PI] since s > 0 */

  // General case when theta != pi. If theta=pi, c=-1
  if ((1 + c) > minimum) // Since -1 <= c <= 1, no fabs(1+c) is required
  {
    double sinc = vpMath::sinc(s, theta);

    data[0] = (R[2][1] - R[1][2]) / (2 * sinc);
    data[1] = (R[0][2] - R[2][0]) / (2 * sinc);
    data[2] = (R[1][0] - R[0][1]) / (2 * sinc);
  } else /* theta near PI */
  {
    double x = 0;
    if ( (R[0][0]-c) > std::numeric_limits<double>::epsilon() )
      x = sqrt((R[0][0]-c)/(1-c));

    double y = 0;
    if ( (R[1][1]-c) > std::numeric_limits<double>::epsilon() )
      y = sqrt((R[1][1]-c)/(1-c));

    double z = 0;
    if ( (R[2][2]-c) > std::numeric_limits<double>::epsilon() )
      z = sqrt((R[2][2]-c)/(1-c));

    if(x > y && x > z)
    {
        if ((R[2][1]-R[1][2]) < 0) x = -x;
        if(vpMath::sign(x)*vpMath::sign(y) != vpMath::sign(R[0][1]+R[1][0])) y = -y;
        if(vpMath::sign(x)*vpMath::sign(z) != vpMath::sign(R[0][2]+R[2][0])) z = -z;
    }
    else if(y > z)
    {
        if((R[0][2]-R[2][0]) < 0) y = -y;
        if(vpMath::sign(y)*vpMath::sign(x) != vpMath::sign(R[1][0]+R[0][1])) x = -x;
        if(vpMath::sign(y)*vpMath::sign(z) != vpMath::sign(R[1][2]+R[2][1])) z = -z;
    }
    else
    {
        if((R[1][0]-R[0][1]) < 0) z = -z;
        if(vpMath::sign(z)*vpMath::sign(x) != vpMath::sign(R[2][0]+R[0][2])) x = -x;
        if(vpMath::sign(z)*vpMath::sign(y) != vpMath::sign(R[2][1]+R[1][2])) y = -y;
    }
    data[0] = theta*x;
    data[1] = theta*y;
    data[2] = theta*z;
  }

  return *this;
}
/*!
  Build a \f$\theta {\bf u}\f$ vector from an Euler z-y-x representation vector.
*/
vpThetaUVector vpThetaUVector::buildFrom(const vpRzyxVector &rzyx)
{
  vpRotationMatrix R(rzyx);

  buildFrom(R);
  return *this;
}
/*!
  Build a \f$\theta {\bf u}\f$ vector from an Euler z-y-z representation vector.
*/
vpThetaUVector vpThetaUVector::buildFrom(const vpRzyzVector &rzyz)
{
  vpRotationMatrix R(rzyz);

  buildFrom(R);
  return *this;
}
/*!
  Build a \f$\theta {\bf u}\f$ vector from an Euler x-y-z representation vector.
*/
vpThetaUVector vpThetaUVector::buildFrom(const vpRxyzVector &rxyz)
{
  vpRotationMatrix R(rxyz);

  buildFrom(R);
  return *this;
}

/*!
  Build a \f$\theta {\bf u}\f$ vector from a quaternion representation vector.
*/
vpThetaUVector vpThetaUVector::buildFrom(const vpQuaternionVector &q)
{
  vpRotationMatrix R(q);

  buildFrom(R);
  return *this;
}

/*!
  Build a \f$\theta {\bf u}\f$ vector from a 3-dim vectors.
*/
vpThetaUVector vpThetaUVector::buildFrom(const std::vector<double> &tu)
{
  if (tu.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot construct a theta-u vector from a %d-dimension std::vector",
                      tu.size()));
  }
  for (unsigned int i = 0; i < 3; i++)
    data[i] = tu[i];

  return *this;
}

/*!
  Build a \f$\theta {\bf u}\f$ vector from a 3-dim vector.
*/
vpThetaUVector vpThetaUVector::buildFrom(const vpColVector &tu)
{
  if (tu.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot construct a theta-u vector from a %d-dimension std::vector",
                      tu.size()));
  }
  for (unsigned int i = 0; i < 3; i++)
    data[i] = tu[i];

  return *this;
}

/*!

  Initialize each element of the \f$\theta {\bf u}\f$ vector to the
  same angle value \e v.

  \param v : Angle value to set for each element of the \f$\theta {\bf
  u}\f$ vector.

\code
#include <visp3/core/vpMath.h>
#include <visp3/core/vpThetaUVector.h>

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
  for (unsigned int i = 0; i < dsize; i++)
    data[i] = v;

  return *this;
}

/*!

  Copy operator that initializes a \f$\theta {\bf u}\f$ vector from a
3-dimension column vector \e tu.

  \param tu : 3-dimension vector containing the values of the \f$\theta {\bf
u}\f$  vector.

\code
#include <visp3/core/vpThetaUVector.h>

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
  if (tu.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot set a theta-u vector from a %d-dimension col vector",
                      tu.size()));
  }
  for (unsigned int i = 0; i < 3; i++)
    data[i] = tu[i];

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
  // if (theta == 0) {
  if (std::fabs(theta) <= std::numeric_limits<double>::epsilon()) {
    u = 0;
    return;
  }
  for (unsigned int i = 0; i < 3; i++)
    u[i] = data[i] / theta;
}

/*!

  Get the rotation angle \f$ \theta \f$ from the \f$ \theta {\bf u} \f$
representation.

  \return Rotation angle \f$ \theta \f$ in rad.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpThetaUVector.h>

int main()
{
  vpHomogeneousMatrix M(0, 0, 1., vpMath::rad(10), vpMath::rad(20), vpMath::rad(30));

  std::cout << "theta: " << M.getRotationMatrix().getThetaUVector().getTheta() << std::endl;
  std::cout << "u    : " << M.getRotationMatrix().getThetaUVector().getU().t() << std::endl;
}
  \endcode

  \sa getTheta(), extract()
*/
double vpThetaUVector::getTheta() const { return sqrt(data[0] * data[0] + data[1] * data[1] + data[2] * data[2]); }

/*!

  Get the unit vector \f$\bf u \f$ from the \f$ \theta {\bf u} \f$
representation.

  \return 3-dim unit vector \f${\bf u} = (u_{x},u_{y},u_{z})^{\top} \f$
  representing the rotation axis.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpThetaUVector.h>

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
  // if (theta == 0) {
  if (std::fabs(theta) <= std::numeric_limits<double>::epsilon()) {
    u = 0;
    return u;
  }
  for (unsigned int i = 0; i < 3; i++)
    u[i] = data[i] / theta;
  return u;
}

/*!
  Build a \f$\theta {\bf u}\f$ vector from 3 angles in radian.
*/
void vpThetaUVector::buildFrom(const double tux, const double tuy, const double tuz)
{
  data[0] = tux;
  data[1] = tuy;
  data[2] = tuz;
}
