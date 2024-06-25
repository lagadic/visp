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
 * Euler angles parameterization for the rotation.
 * Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
 */

/*!
  \file vpRzyzVector.h
  \brief class that consider the case of the Rzyz angles parameterization
  for the  rotation

  Rzyz(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(z,psi)
*/

#ifndef VP_RZYZ_VECTOR_H
#define VP_RZYZ_VECTOR_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRotationVector.h>

BEGIN_VISP_NAMESPACE

class vpRotationMatrix;
class vpThetaUVector;

/*!
  \class vpRzyzVector

  \ingroup group_core_transformations

  \brief Implementation of a rotation vector as \f$R(z,y,z)\f$ Euler angle
  minimal representation.

  Class that consider the case of the Euler
  \f$(\varphi,\theta,\psi)\f$ angles using the z-y-z convention, where
  \f$(\varphi,\theta,\psi)\f$ are respectively the rotation angles
  around the \f$z\f$, \f$y\f$ and \f$z\f$ axis.

  \f[R_{zyz}(\varphi,\theta,\psi) = R_z(\varphi) \; R_y(\theta) \;
  R_z(\psi)\f]

  with

  \f[
  R_{z}(\varphi) = \left(
  \begin{array}{ccc}
  \cos \varphi & -\sin\varphi & 0\\
  \sin\varphi &\cos \varphi& 0 \\
  0 & 0 & 1
  \end{array}
  \right) \;
  R_{y}(\theta) = \left(
  \begin{array}{ccc}
  \cos \theta & 0 & \sin\theta\\
  0 & 1 & 0 \\
  -\sin\theta & 0 &\cos \theta
  \end{array}
  \right) \;
  R_{z}(\psi) = \left(
  \begin{array}{ccc}
  \cos \psi & -\sin\psi & 0\\
  \sin\psi &\cos \psi& 0 \\
  0 & 0 & 1
  \end{array}
  \right)
  \f]

  The rotation matrix corresponding to the z-y-z convention is given by:

  \f[
  R_{zyz}(\varphi,\theta,\psi) = \left(
  \begin{array}{ccc}
  \cos\varphi \cos\theta \cos\psi - \sin\varphi\sin\psi & -\cos\varphi
  \cos\theta \sin\psi -\sin\varphi\cos\psi & \cos\varphi \sin\theta \\
  \sin\varphi \cos\theta \cos\psi + \cos\varphi\sin\psi & -\sin\varphi
  \cos\theta \sin\psi +\cos\varphi\cos\psi & \sin\varphi \sin\theta \\
  -\sin\theta \cos\psi & \sin\theta \sin\psi & \cos\theta
  \end{array}
  \right)
  \f]

  The vpRzyzVector class is derived from vpRotationVector.

  From the implementation point of view, it is nothing more than an
  array of three doubles with values in [rad].

  You can set values [rad] accessing each element:
  \code
  vpRzyzVector rzyz;
  rzyz[0] = M_PI_4;
  rzyz[1] = M_PI_2;
  rzyz[2] = M_PI;
  \endcode
  You can also initialize the vector using operator<<(double):
  \code
  rzyz << M_PI_4, M_PI_2, M_PI;
  \endcode
  Or you can also initialize the vector from a list of doubles if ViSP is build with c++11 enabled:
  \code
  rzyz = {M_PI_4, M_PI_2, M_PI};
  \endcode

  To get the values [rad] use:
  \code
  double r1 = rzyz[0];
  double r2 = rzyz[1];
  double r3 = rzyz[2];
  \endcode

  The code below shows first how to initialize this representation of
  Euler angles, than how to construct a rotation matrix from a
  vpRzyzVector and finally how to extract the vpRzyzVector Euler angles
  from the build rotation matrix.

  \code
  #include <visp3/core/vpMath.h>
  #include <visp3/core/vpRotationMatrix.h>
  #include <visp3/core/vpRzyzVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRzyzVector rzyz;

    // Initialise the Euler angles
    rzyz[0] = vpMath::rad( 45.f); // phi   angle in rad/s around z axis
    rzyz[1] = vpMath::rad(-30.f); // theta angle in rad/s around y axis
    rzyz[2] = vpMath::rad( 90.f); // psi   angle in rad/s around z axis

    // Construct a rotation matrix from the Euler angles
    vpRotationMatrix R(rzyz);

    // Extract the Euler angles around z,y,z axis from a rotation matrix
    rzyz.build(R);

    // Print the extracted Euler angles. Values are the same than the
    // one used for initialization
    std::cout << rzyz;

    // Since the rotation vector is 3 values column vector, the
    // transpose operation produce a row vector.
    vpRowVector rzyz_t = rzyz.t();

    // Print the transpose row vector
    std::cout << rzyz_t << std::endl;
  }
  \endcode
*/
class VISP_EXPORT vpRzyzVector : public vpRotationVector
{
public:
  vpRzyzVector();
  vpRzyzVector(const vpRzyzVector &rzyz);

  // initialize a Rzyz vector from a rotation matrix
  VP_EXPLICIT vpRzyzVector(const vpRotationMatrix &R);

  // initialize a Rzyz vector from a ThetaU vector
  VP_EXPLICIT vpRzyzVector(const vpThetaUVector &tu);

  vpRzyzVector(double phi, double theta, double psi);
  VP_EXPLICIT vpRzyzVector(const vpColVector &rzyz);
  VP_EXPLICIT vpRzyzVector(const std::vector<double> &rzyz);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  // convert a rotation matrix into Rzyz vector
  VP_DEPRECATED vpRzyzVector buildFrom(const vpRotationMatrix &R);

  // convert a ThetaU vector into a Rzyz vector
  VP_DEPRECATED vpRzyzVector buildFrom(const vpThetaUVector &R);
  VP_DEPRECATED vpRzyzVector buildFrom(const vpColVector &rxyz);
  VP_DEPRECATED vpRzyzVector buildFrom(const std::vector<double> &rxyz);

  VP_DEPRECATED void buildFrom(double phi, double theta, double psi);
#endif
  // convert a rotation matrix into Rzyz vector
  vpRzyzVector &build(const vpRotationMatrix &R);

  // convert a ThetaU vector into a Rzyz vector
  vpRzyzVector &build(const vpThetaUVector &R);
  vpRzyzVector &build(const vpColVector &rxyz);
  vpRzyzVector &build(const std::vector<double> &rxyz);

  vpRzyzVector &build(const double &phi, const double &theta, const double &psi);

  vpRzyzVector &operator=(const vpColVector &rzyz);
  vpRzyzVector &operator=(double x);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpRzyzVector &operator=(const vpRzyzVector &) = default;
  vpRzyzVector &operator=(const std::initializer_list<double> &list);
#endif
};
END_VISP_NAMESPACE
#endif
