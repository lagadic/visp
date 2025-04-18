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
  \file vpRzyxVector.h

  \brief class that consider the case of the Rzyx angle
  parameterization for the rotation.

  Rzyx(phi,theta,psi) = Rot(z,phi)Rot(y,theta)Rot(x,psi)
*/

#ifndef VP_RZYX_VECTOR_H
#define VP_RZYX_VECTOR_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRotationVector.h>

BEGIN_VISP_NAMESPACE

class vpRotationMatrix;
class vpThetaUVector;

/*!
  \class vpRzyxVector

  \ingroup group_core_transformations

  \brief Implementation of a rotation vector as \f$R(z,y,x)\f$ Euler angle
  minimal representation.

  Class that consider the case of the Euler
  \f$(\varphi,\theta,\psi)\f$ angle using the z-y-x convention, where
  \f$(\varphi,\theta,\psi)\f$ are respectively the rotation angles around the
  \f$z\f$, \f$y\f$ and \f$x\f$ axis.

  \f[R_{zyx}(\varphi,\theta,\psi) = R_z(\varphi) \; R_y(\theta) \; R_x(\psi)\f]

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
  R_{x}(\psi) = \left(
  \begin{array}{ccc}
  1 & 0 & 0 \\
  0 &\cos \psi & -\sin\psi \\
  0 &\sin \psi & \cos\psi \\
  \end{array}
  \right)
  \f]

  The rotation matrix corresponding to the z-y-x convention is given by:

  \f[
  R_{zyx}(\varphi,\theta,\psi) = \left(
  \begin{array}{ccc}
  \cos\varphi \cos\theta & -\sin\varphi \cos\psi +
  \cos\varphi\sin\theta\sin\psi & \sin\varphi \sin\psi
  +\cos\varphi\sin\theta\cos\psi \\
  \sin\varphi \cos\theta & \cos\varphi\cos\psi + \sin\varphi\sin\theta
  \sin\psi & -\cos\varphi \sin\psi +\sin\varphi\sin\theta\cos\psi \\
  -\sin\theta & \cos\theta \sin\psi & \cos\theta \cos\psi
  \end{array}
  \right)
  \f]

  The vpRzyxVector class is derived from vpRotationVector.

  From the implementation point of view, it is nothing more than an
  array of three doubles with values in [rad].

  You can set values [rad] accessing each element:
  \code
  vpRzyxVector rzyx;
  rzyx[0] = M_PI_4;
  rzyx[1] = M_PI_2;
  rzyx[2] = M_PI;
  \endcode
  You can also initialize the vector using operator<<(double):
  \code
  rzyx << M_PI_4, M_PI_2, M_PI;
  \endcode
  Or you can also initialize the vector from a list of doubles if ViSP is build with c++11 enabled:
  \code
  rzyx = {M_PI_4, M_PI_2, M_PI};
  \endcode

  To get the values [rad] use:
  \code
  double rz = rzyx[0];
  double ry = rzyx[1];
  double rx = rzyx[2];
  \endcode

  The code below shows first how to initialize this representation of
  Euler angles, than how to construct a rotation matrix from a
  vpRzyxVector and finally how to extract the vpRzyxVector Euler angles
  from the build rotation matrix.

  \code
  #include <visp3/core/vpMath.h>
  #include <visp3/core/vpRotationMatrix.h>
  #include <visp3/core/vpRzyxVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRzyxVector rzyx;

    // Initialise the Euler angles
    rzyx[0] = vpMath::rad( 45.f); // phi   angle in rad/s around z axis
    rzyx[1] = vpMath::rad(-30.f); // theta angle in rad/s around y axis
    rzyx[2] = vpMath::rad( 90.f); // psi   angle in rad/s around x axis

    // Construct a rotation matrix from the Euler angles
    vpRotationMatrix R(rzyx);

    // Extract the Euler angles around z,y,x axis from a rotation matrix
    rzyx.buildFrom(R);

    // Print the extracted Euler angles. Values are the same than the
    // one used for initialization
    std::cout << rzyx;

    // Since the rotation vector is 3 values column vector, the
    // transpose operation produce a row vector.
    vpRowVector rzyx_t = rzyx.t();

    // Print the transpose row vector
    std::cout << rzyx_t << std::endl;
  }
  \endcode

*/
class VISP_EXPORT vpRzyxVector : public vpRotationVector
{
public:
  vpRzyxVector();
#if (VISP_CXX_STANDARD > VISP_CXX_STANDARD_98)
  vpRzyxVector(const vpRzyxVector &) = default;
  virtual ~vpRzyxVector() VP_OVERRIDE = default;
#else
  virtual ~vpRzyxVector() { }
#endif
  vpRzyxVector(double phi, double theta, double psi);

  // initialize a Rzyx vector from a rotation matrix
  VP_EXPLICIT vpRzyxVector(const vpRotationMatrix &R);

  // initialize a Rzyx vector from a ThetaU vector
  VP_EXPLICIT vpRzyxVector(const vpThetaUVector &tu);
  VP_EXPLICIT vpRzyxVector(const vpColVector &rzyx);
  VP_EXPLICIT vpRzyxVector(const std::vector<double> &rzyx);

  // convert a rotation matrix into Rzyx vector
  vpRzyxVector &buildFrom(const vpRotationMatrix &R);

  // convert a ThetaU vector into a Rzyx vector
  vpRzyxVector &buildFrom(const vpThetaUVector &R);
  vpRzyxVector &buildFrom(const vpColVector &rxyz);
  vpRzyxVector &buildFrom(const std::vector<double> &rxyz);

  vpRzyxVector &buildFrom(const double &phi, const double &theta, const double &psi);

  vpRzyxVector &operator=(const vpColVector &rzyx);
  vpRzyxVector &operator=(double x);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpRzyxVector &operator=(const vpRzyxVector &) = default;
  vpRzyxVector &operator=(const std::initializer_list<double> &list);
#endif
private:
  static const unsigned int constr_val_3;
};
END_VISP_NAMESPACE
#endif
