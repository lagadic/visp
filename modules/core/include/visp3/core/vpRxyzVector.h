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
  \file vpRxyzVector.h

  \brief Class that consider the case of the Rxyz angle
  parameterization for the rotation.

  Rxyz(phi,theta,psi) = Rot(x,phi)Rot(y,theta)Rot(z,psi)
 */

#ifndef VP_RXYZ_VECTOR_H
#define VP_RXYZ_VECTOR_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRotationVector.h>

BEGIN_VISP_NAMESPACE

class vpRotationVector;
class vpRotationMatrix;
class vpThetaUVector;

/*!
  \class vpRxyzVector

  \ingroup group_core_transformations

  \brief Implementation of a rotation vector as \f$R(x,y,z)\f$ Euler angle
  minimal representation.

  Class that consider the case of the Euler
  \f$(\varphi,\theta,\psi)\f$ angle using the x-y-z convention, where
  \f$(\varphi,\theta,\psi)\f$ are respectively the rotation angles around the
  \f$x\f$, \f$y\f$ and \f$z\f$ axis.

  \f[R_{xyz}(\varphi,\theta,\psi) = R_x(\varphi) \; R_y(\theta) \; R_z(\psi)\f]

  with

  \f[R_{x}(\varphi) = \left(
  \begin{array}{ccc}
  1 & 0 & 0 \\
  0 &\cos \varphi & -\sin\varphi \\
  0 &\sin \varphi & \cos\varphi \\
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
  \right)\f]

  The rotation matrix corresponding to the x-y-z convention is given by:

  \f[
  R_{xyz}(\varphi,\theta,\psi) = \left(
  \begin{array}{ccc}
  \cos\theta \cos\psi & -\cos\theta \sin\psi & \sin\theta \\
  \sin\varphi \sin\theta \cos\psi + \cos\varphi\sin\psi & -\sin\varphi
  \sin\theta \sin\psi +\cos\varphi\cos\psi & -\sin\varphi \cos\theta \\
  -\cos\varphi \sin\theta \cos\psi + \sin\varphi\sin\psi & \cos\varphi
  \sin\theta \sin\psi +\sin\varphi\cos\psi & \cos\varphi \cos\theta \end{array}
  \right)
  \f]

  The vpRxyzVector class is derived from vpRotationVector.

  From the implementation point of view, it is nothing more than an
  array of three doubles with values in [rad].

  You can set values [rad] accessing each element:
  \code
  vpRxyzVector rxyz;
  rxyz[0] = M_PI_4;
  rxyz[1] = M_PI_2;
  rxyz[2] = M_PI;
  \endcode
  You can also initialize the vector using operator<<(double):
  \code
  rxyz << M_PI_4, M_PI_2, M_PI;
  \endcode
  Or you can also initialize the vector from a list of doubles if ViSP is build with c++11 enabled:
  \code
  rxyz = {M_PI_4, M_PI_2, M_PI};
  \endcode

  To get the values [rad] use:
  \code
  double rx = rxyz[0];
  double ry = rxyz[1];
  double rz = rxyz[2];
  \endcode

  The code below shows first how to initialize this representation of
  Euler angles, than how to construct a rotation matrix from a
  vpRxyzVector and finally how to extract the vpRxyzVector Euler angles
  from the build rotation matrix.

  \code
  #include <iostream>
  #include <visp3/core/vpMath.h>
  #include <visp3/core/vpRotationMatrix.h>
  #include <visp3/core/vpRxyzVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRxyzVector rxyz;

    // Initialise the Euler angles
    rxyz[0] = vpMath::rad( 45.f); // phi   angle in rad around x axis
    rxyz[1] = vpMath::rad(-30.f); // theta angle in rad around y axis
    rxyz[2] = vpMath::rad( 90.f); // psi   angle in rad around z axis

    // Construct a rotation matrix from the Euler angles
    vpRotationMatrix R(rxyz);

    // Extract the Euler angles around x,y,z axis from a rotation matrix
    rxyz.build(R);

    // Print the extracted Euler angles. Values are the same than the
    // one used for initialization
    std::cout << rxyz;

    // Since the rotation vector is 3 values column vector, the
    // transpose operation produce a row vector.
    vpRowVector rxyz_t = rxyz.t();

    // Print the transpose row vector
    std::cout << rxyz_t << std::endl;
  }
  \endcode

*/

class VISP_EXPORT vpRxyzVector : public vpRotationVector
{
public:
  vpRxyzVector();
  vpRxyzVector(const vpRxyzVector &rxyz);
  vpRxyzVector(double phi, double theta, double psi);

  // initialize a Rxyz vector from a rotation matrix
  VP_EXPLICIT vpRxyzVector(const vpRotationMatrix &R);

  // initialize a Rxyz vector from a ThetaU vector
  VP_EXPLICIT vpRxyzVector(const vpThetaUVector &tu);
  VP_EXPLICIT vpRxyzVector(const vpColVector &rxyz);
  VP_EXPLICIT vpRxyzVector(const std::vector<double> &rxyz);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  // convert a rotation matrix into Rxyz vector
  VP_DEPRECATED vpRxyzVector buildFrom(const vpRotationMatrix &R);

  // convert a ThetaU vector into a Rxyz vector
  VP_DEPRECATED vpRxyzVector buildFrom(const vpThetaUVector &tu);
  VP_DEPRECATED vpRxyzVector buildFrom(const vpColVector &rxyz);
  VP_DEPRECATED vpRxyzVector buildFrom(const std::vector<double> &rxyz);

  VP_DEPRECATED void buildFrom(double phi, double theta, double psi);
#endif
  // convert a rotation matrix into Rxyz vector
  vpRxyzVector &build(const vpRotationMatrix &R);

  // convert a ThetaU vector into a Rxyz vector
  vpRxyzVector &build(const vpThetaUVector &tu);
  vpRxyzVector &build(const vpColVector &rxyz);
  vpRxyzVector &build(const std::vector<double> &rxyz);

  vpRxyzVector &build(const double &phi, const double &theta, const double &psi);

  vpRxyzVector &operator=(const vpColVector &rxyz);
  vpRxyzVector &operator=(double x);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpRxyzVector &operator=(const vpRxyzVector &) = default;
  vpRxyzVector &operator=(const std::initializer_list<double> &list);
#endif
};
END_VISP_NAMESPACE
#endif
