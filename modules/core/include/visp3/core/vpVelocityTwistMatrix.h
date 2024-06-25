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
 * Velocity twist transformation matrix.
 */

#ifndef VP_VELOCITY_TWIST_MATRIX_H
#define VP_VELOCITY_TWIST_MATRIX_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRotationMatrix.h>

BEGIN_VISP_NAMESPACE
class vpColVector;
class vpHomogeneousMatrix;
class vpMatrix;

/*!
  \class vpVelocityTwistMatrix

  \ingroup group_core_transformations

  This class derived from vpArray2D<double> implements the 6 by 6 matrix which
  transforms velocities from one frame to another. This matrix is also called
  velocity twist transformation matrix.

  The full velocity twist transformation matrix allows to compute the velocity
  at point <em>a</em> expressed in frame <em>a</em> knowing its velocity at
  point <em>b</em> expressed in frame <em>b</em>. This matrix is defined as: \f[
  ^a{\bf V}_b = \left[\begin{array}{cc}
  ^a{\bf R}_b & [^a{\bf t}_b]_\times \; ^a{\bf R}_b\\
  {\bf 0}_{3\times 3} & ^a{\bf R}_b
  \end{array}
  \right]
  \f]

  where \f$ ^a{\bf R}_b \f$ is a rotation matrix and
  \f$ ^a{\bf t}_b \f$ is a translation vector.

  There are different ways to initialize such a full velocity twist matrix. The following
  example shows how to proceed setting the translation and rotation matrix transformations:
  \code
  #include <visp3/core/vpVelocityTwistMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpTranslationVector cte(0.1, 0.2, 0.3);
    vpRotationMatrix cRe( {0,  0, -1,
                          0, -1,  0,
                          -1,  0,  0} );

    vpVelocityTwistMatrix cVe(cte, cRe);
    std::cout << "cVe:\n" << cVe << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  cVe:
  0  0  -1  -0.2  0.3  0
  0  -1  0  0.1  0  -0.3
  -1  0  0  0  -0.1  0.2
  0  0  0  0  0  -1
  0  0  0  0  -1  0
  0  0  0  -1  0  0
  \endcode

  When the point where the velocity is expressed doesn't change, the matrix
  becomes block diagonal. It allows than to compute the velocity at point
  <em>b</em> expressed in frame <em>a</em> knowing its
  velocity at point <em>b</em> expressed in frame <em>b</em> :
  \f[
  ^a{\bf V}_b = \left[\begin{array}{cc}
  ^a{\bf R}_b & {\bf 0}_{3\times 3} \\
  {\bf 0}_{3\times 3} & ^a{\bf R}_b
  \end{array}
  \right]
  \f]

  To initialize such a velocity twist matrix where translation is not taken into account you
  can proceed like in the following code:
  \code
  #include <visp3/core/vpVelocityTwistMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRotationMatrix cRe( {0,  0, -1,
                          0, -1,  0,
                          -1,  0,  0} );

    vpVelocityTwistMatrix cVe(cRe);
    std::cout << "cVe:\n" << cVe << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  cVe:
  0  0  -1  0  0  0
  0  -1  0  0  0  0
  -1  0  0  0  0  0
  0  0  0  0  0  -1
  0  0  0  0  -1  0
  0  0  0  -1  0  0
  \endcode

  The code below shows how to convert a velocity skew
  expressed at the origin of the camera frame into the origin of the fix frame
  using the full velocity twist matrix.

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpVelocityTwistMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpVelocityTwistMatrix fVc; // Twist transformation matrix from fix to camera frame

    vpHomogeneousMatrix fMc; // Fix to camera frame transformation
    // ... fMc need here to be initialized

    fVc.build(fMc);

    vpColVector c_v(6); // Velocity in the camera frame: vx,vy,vz,wx,wy,wz
    // ... c_v should here have an initial value

    vpColVector f_v(6); // Velocity in the fix frame: vx,vy,vz,wx,wy,wz

    // Compute the velocity in the fix frame
    f_v = fVc * c_v;
  }
  \endcode
*/
class VISP_EXPORT vpVelocityTwistMatrix : public vpArray2D<double>
{
  friend class vpMatrix;

public:
  // basic constructor
  vpVelocityTwistMatrix();
  // copy constructor
  vpVelocityTwistMatrix(const vpVelocityTwistMatrix &V);
  // constructor from an homogeneous transformation
  VP_EXPLICIT vpVelocityTwistMatrix(const vpHomogeneousMatrix &M, bool full = true);

  // Construction from Translation and rotation (matrix parameterization)
  vpVelocityTwistMatrix(const vpTranslationVector &t, const vpRotationMatrix &R);
  // Construction from Translation and rotation (ThetaU parameterization)
  vpVelocityTwistMatrix(const vpTranslationVector &t, const vpThetaUVector &thetau);
  vpVelocityTwistMatrix(double tx, double ty, double tz, double tux, double tuy, double tuz);

  VP_EXPLICIT vpVelocityTwistMatrix(const vpRotationMatrix &R);
  VP_EXPLICIT vpVelocityTwistMatrix(const vpThetaUVector &thetau);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  VP_DEPRECATED vpVelocityTwistMatrix buildFrom(const vpTranslationVector &t, const vpRotationMatrix &R);
  VP_DEPRECATED vpVelocityTwistMatrix buildFrom(const vpTranslationVector &t, const vpThetaUVector &thetau);
  VP_DEPRECATED vpVelocityTwistMatrix buildFrom(const vpHomogeneousMatrix &M, bool full = true);
  VP_DEPRECATED vpVelocityTwistMatrix buildFrom(const vpRotationMatrix &R);
  VP_DEPRECATED vpVelocityTwistMatrix buildFrom(const vpThetaUVector &thetau);
#endif
  vpVelocityTwistMatrix &build(const vpTranslationVector &t, const vpRotationMatrix &R);
  vpVelocityTwistMatrix &build(const vpTranslationVector &t, const vpThetaUVector &thetau);
  vpVelocityTwistMatrix &build(const vpHomogeneousMatrix &M, bool full = true);
  vpVelocityTwistMatrix &build(const vpRotationMatrix &R);
  vpVelocityTwistMatrix &build(const vpThetaUVector &thetau);

  void extract(vpRotationMatrix &R) const;
  void extract(vpTranslationVector &t) const;

  // Basic initialisation (identity)
  void eye();

  vpVelocityTwistMatrix inverse() const;
  void inverse(vpVelocityTwistMatrix &V) const;

  vpVelocityTwistMatrix operator*(const vpVelocityTwistMatrix &V) const;
  vpMatrix operator*(const vpMatrix &M) const;
  vpColVector operator*(const vpColVector &v) const;

  vpVelocityTwistMatrix &operator=(const vpVelocityTwistMatrix &V);

  int print(std::ostream &s, unsigned int length, char const *intro = 0) const;

  /*!
    This function is not applicable to a velocity twist matrix that is always
    a 6-by-6 matrix. \exception vpException::fatalError When this function is
    called.
    */
  void resize(unsigned int nrows, unsigned int ncols, bool flagNullify = true)
  {
    (void)nrows;
    (void)ncols;
    (void)flagNullify;
    throw(vpException(vpException::fatalError, "Cannot resize a velocity twist matrix"));
  }

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
     \deprecated Provided only for compat with previous releases.
     This function does nothing.
   */
  VP_DEPRECATED void init() { }
  /*!
     \deprecated You should rather use eye().
   */
  VP_DEPRECATED void setIdentity();
//@}
#endif
};
END_VISP_NAMESPACE
#endif
