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
  \file vpThetaUVector.h
  \brief class that consider the case of the Theta U parameterization for the
  rotation
*/

#ifndef VP_THETAU_VECTOR_H
#define VP_THETAU_VECTOR_H

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpRotationVector.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpRzyxVector.h>

BEGIN_VISP_NAMESPACE

class vpHomogeneousMatrix;
class vpRotationMatrix;
class vpPoseVector;
class vpRzyxVector;
class vpRxyzVector;
class vpRzyzVector;
class vpColVector;
class vpRotationVector;
class vpQuaternionVector;

/*!
  \class vpThetaUVector

  \ingroup group_core_transformations

  \brief Implementation of a rotation vector as \f$\theta {\bf u}\f$
  axis-angle minimal representation.

  Class that consider the case of the \f$\theta {\bf u}\f$
  parameterization for the rotation.

  The vpThetaUVector class is derived from vpRotationVector.

  The \f$\theta {\bf u}\f$ representation is one of the minimal
  representation of a rotation matrix, where
  \f${\bf u} = (u_{x} \; u_{y} \; u_{z})^{\top}\f$
  is a unit vector representing the rotation
  axis and \f$\theta\f$ is the rotation angle.

  From the \f$\theta {\bf u}\f$ representation it is possible to build the
  rotation matrix \f${\bf R}\f$ using the Rodrigues formula:

  \f[
  {\bf R} =  {\bf I}_{3} + (1 - \cos{ \theta}) \; {\bf u u}^{\top} + \sin{
  \theta} \; [{\bf u}]_{\times} \f]

  with \f${\bf I}_{3}\f$ the identity matrix of dimension
  \f$3\times3\f$ and \f$[{\bf u}]_{\times}\f$ the skew matrix:

  \f[
  [{\bf u}]_{\times} = \left(
  \begin{array}{ccc}
  0 & -u_{z} & u_{y} \\
  u_{z} & 0 & -u_{x} \\
  -u_{y} & u_{x} & 0
  \end{array}
  \right)
  \f]

  From the implementation point of view, it is nothing more than an
  array of three floats with values in [rad].

  You can set values [rad] accessing each element:
  \code
  vpThetaUVector tu;
  tu[0] = M_PI_4;
  tu[1] = M_PI_2;
  tu[2] = M_PI;
  \endcode
  You can also initialize the vector using operator<<(double):
  \code
  tu << M_PI_4, M_PI_2, M_PI;
  \endcode
  Or you can also initialize the vector from a list of doubles if ViSP is build with c++11 enabled:
  \code
  tu = {M_PI_4, M_PI_2, M_PI};
  \endcode

  To get the values [rad] use:
  \code
  double tux = tu[0];
  double tuy = tu[1];
  double tuz = tu[2];
  \endcode

  The code below shows first how to initialize a \f$\theta {\bf u}\f$
  vector, than how to construct a rotation matrix from a vpThetaUVector
  and finally how to extract the theta U angles from the build rotation
  matrix.

  \code
  #include <iostream>
  #include <visp3/core/vpMath.h>
  #include <visp3/core/vpRotationMatrix.h>
  #include <visp3/core/vpThetaUVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpThetaUVector tu;

    // Initialise the theta U rotation vector
    tu[0] = vpMath::rad( 45.f);
    tu[1] = vpMath::rad(-30.f);
    tu[2] = vpMath::rad( 90.f);

    // Construct a rotation matrix from the theta U angles
    vpRotationMatrix R(tu);

    // Extract the theta U angles from a rotation matrix
    tu.build(R);

    // Print the extracted theta U angles. Values are the same than the
    // one used for initialization
    std::cout << tu;

    // Since the rotation vector is 3 values column vector, the
    // transpose operation produce a row vector.
    vpRowVector tu_t = tu.t();

    // Print the transpose row vector
    std::cout << tu_t << std::endl;
  }
  \endcode
*/
class VISP_EXPORT vpThetaUVector : public vpRotationVector
{
public:
  vpThetaUVector();
  vpThetaUVector(const vpThetaUVector &tu);

  // constructor initialize a Theta U vector from a homogeneous matrix
  VP_EXPLICIT vpThetaUVector(const vpHomogeneousMatrix &M);
  // constructor initialize a Theta U vector from a pose vector
  VP_EXPLICIT vpThetaUVector(const vpPoseVector &p);
  // constructor initialize a Theta U vector from a rotation matrix
  VP_EXPLICIT vpThetaUVector(const vpRotationMatrix &R);
  // constructor initialize a Theta U vector from a RzyxVector
  VP_EXPLICIT vpThetaUVector(const vpRzyxVector &rzyx);
  // constructor initialize a Theta U vector from a RzyzVector
  VP_EXPLICIT vpThetaUVector(const vpRzyzVector &rzyz);
  // constructor initialize a Theta U vector from a RxyzVector
  VP_EXPLICIT vpThetaUVector(const vpRxyzVector &rxyz);
  VP_EXPLICIT vpThetaUVector(const vpQuaternionVector &q);
  VP_EXPLICIT vpThetaUVector(const vpColVector &tu);
  VP_EXPLICIT vpThetaUVector(const std::vector<double> &tu);

  vpThetaUVector(double tux, double tuy, double tuz);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  // convert an homogeneous matrix into Theta U vector
  VP_DEPRECATED vpThetaUVector buildFrom(const vpHomogeneousMatrix &M);
  // convert a pose vector into Theta U vector
  VP_DEPRECATED vpThetaUVector buildFrom(const vpPoseVector &p);
  // convert a rotation matrix into Theta U vector
  VP_DEPRECATED vpThetaUVector buildFrom(const vpRotationMatrix &R);
  // convert an Rzyx vector into Theta U vector
  VP_DEPRECATED vpThetaUVector buildFrom(const vpRzyxVector &rzyx);
  // convert an Rzyz vector into Theta U vector
  VP_DEPRECATED vpThetaUVector buildFrom(const vpRzyzVector &zyz);
  // convert an Rxyz vector into Theta U vector
  VP_DEPRECATED vpThetaUVector buildFrom(const vpRxyzVector &xyz);
  VP_DEPRECATED vpThetaUVector buildFrom(const vpQuaternionVector &q);
  VP_DEPRECATED vpThetaUVector buildFrom(const vpColVector &tu);
  VP_DEPRECATED vpThetaUVector buildFrom(const std::vector<double> &tu);

  VP_DEPRECATED void buildFrom(double tux, double tuy, double tuz);
#endif
  // convert an homogeneous matrix into Theta U vector
  vpThetaUVector &build(const vpHomogeneousMatrix &M);
  // convert a pose vector into Theta U vector
  vpThetaUVector &build(const vpPoseVector &p);
  // convert a rotation matrix into Theta U vector
  vpThetaUVector &build(const vpRotationMatrix &R);
  // convert an Rzyx vector into Theta U vector
  vpThetaUVector &build(const vpRzyxVector &rzyx);
  // convert an Rzyz vector into Theta U vector
  vpThetaUVector &build(const vpRzyzVector &zyz);
  // convert an Rxyz vector into Theta U vector
  vpThetaUVector &build(const vpRxyzVector &xyz);
  vpThetaUVector &build(const vpQuaternionVector &q);
  vpThetaUVector &build(const vpColVector &tu);
  vpThetaUVector &build(const std::vector<double> &tu);

  vpThetaUVector &build(const double &tux, const double &tuy, const double &tuz);

  // extract the angle and the axis from the ThetaU representation
  void extract(double &theta, vpColVector &u) const;
  double getTheta() const;
  vpColVector getU() const;

  vpThetaUVector &operator=(const vpColVector &tu);
  vpThetaUVector &operator=(double x);
  vpThetaUVector operator*(const vpThetaUVector &tu_b) const;

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpThetaUVector &operator=(const std::initializer_list<double> &list);
#endif

private:
  static const double minimum;

};
END_VISP_NAMESPACE
#endif
