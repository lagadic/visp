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
 * Twist transformation matrix that allows to transform forces from one
 * frame to an other.
 */

#ifndef VP_FORCE_TWIST_MATRIX_H
#define VP_FORCE_TWIST_MATRIX_H

#include <visp3/core/vpConfig.h>

BEGIN_VISP_NAMESPACE
class vpMatrix;
END_VISP_NAMESPACE

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRotationMatrix.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpForceTwistMatrix

  \ingroup group_core_transformations

  This class derived from vpArray2D<double> implements the 6 by 6 matrix which
  transforms force/torque from one frame to another. This matrix is also called
  force/torque twist transformation matrix.

  The full force/torque twist transformation matrix allows to compute the
  force/torque at point <em>a</em> expressed in frame <em>a</em> knowing its
  force/torque at point <em>b</em> expressed in frame <em>b</em>. This matrix
  is defined as:

  \f[
  ^a{\bf F}_b = \left[ \begin{array}{cc}
  ^a{\bf R}_b & {\bf 0}_{3\times 3}\\
  {[^a{\bf t}_b]}_{\times} \; ^a{\bf R}_b & ^a{\bf R}_b
  \end{array}
  \right]
  \f]

  where \f$ ^a{\bf R}_b \f$ is a rotation matrix and
  \f$ ^a{\bf t}_b \f$ is a translation vector.

  There are different ways to initialize such a full force/torque twist matrix. The following
  example shows how to proceed setting the translation and rotation matrix transformations:
  \code
  #include <visp3/core/vpForceTwistMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpTranslationVector stp(0.1, 0.2, 0.3);
    vpRotationMatrix sRp( {0,  0, -1,
                          0, -1,  0,
                          -1,  0,  0} );
    vpForceTwistMatrix sFp(stp, sRp);
    std::cout << "sFp:\n" << sFp << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  sFp:
  0  0  -1  0  0  0
  0  -1  0  0  0  0
  -1  0  0  0  0  0
  -0.2  0.3  0  0  0  -1
  0.1  0  -0.3  0  -1  0
  0  -0.1  0.2  -1  0  0
  \endcode

  When the point where the velocity is expressed doesn't change, the matrix
  becomes block diagonal. It allows than to compute the force/torque at point
  <em>b</em> expressed in frame <em>a</em> knowing its
  force/torque at point <em>b</em> expressed in frame <em>b</em> :
  \f[
  ^a{\bf F}_b = \left[ \begin{array}{cc}
  ^a{\bf R}_b & {\bf 0}_{3\times 3}\\
  {\bf 0}_{3\times 3} & ^a{\bf R}_b
  \end{array}
  \right]
  \f]

  To initialize such a force/torque twist matrix where translation is not taken into account you
  can proceed like in the following code:
  \code
  #include <visp3/core/vpForceTwistMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRotationMatrix sRp( {0,  0, -1,
                          0, -1,  0,
                          -1,  0,  0} );
    vpForceTwistMatrix sFp(sRp);
    std::cout << "sFp:\n" << sFp << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  sFp:
  0  0  -1  0  0  0
  0  -1  0  0  0  0
  -1  0  0  0  0  0
  0  0  0  0  0  -1
  0  0  0  0  -1  0
  0  0  0  -1  0  0
  \endcode

  The code belows shows for example how to convert a force/torque skew
  from probe frame to a sensor frame.

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpForceTwistMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    // Twist transformation matrix from sensor to probe frame
    vpForceTwistMatrix sFp;

    // Force/torque sensor frame to probe frame transformation
    vpHomogeneousMatrix sMp;
    // ... sMp need here to be initialized

    sFp.build(sMp);

    // Force/torque skew in the probe frame: fx,fy,fz,tx,ty,tz
    vpColVector p_H(6);
    // ... p_H should here have an initial value

    // Force/torque skew in the sensor frame: fx,fy,fz,tx,ty,tz
    vpColVector s_H(6);

    // Compute the value of the force/torque in the sensor frame
    s_H = sFp * p_H;
  }
  \endcode
*/
class VISP_EXPORT vpForceTwistMatrix : public vpArray2D<double>
{
public:
  // basic constructor
  vpForceTwistMatrix();
  // copy constructor
  vpForceTwistMatrix(const vpForceTwistMatrix &F);
  // constructor from an homogeneous transformation
  VP_EXPLICIT vpForceTwistMatrix(const vpHomogeneousMatrix &M, bool full = true);

  // Construction from Translation and rotation (matrix parameterization)
  vpForceTwistMatrix(const vpTranslationVector &t, const vpRotationMatrix &R);
  // Construction from Translation and rotation (ThetaU parameterization)
  vpForceTwistMatrix(const vpTranslationVector &t, const vpThetaUVector &thetau);
  vpForceTwistMatrix(double tx, double ty, double tz, double tux, double tuy, double tuz);

  VP_EXPLICIT vpForceTwistMatrix(const vpRotationMatrix &R);
  VP_EXPLICIT vpForceTwistMatrix(const vpThetaUVector &thetau);

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  VP_DEPRECATED vpForceTwistMatrix buildFrom(const vpTranslationVector &t, const vpRotationMatrix &R);
  VP_DEPRECATED vpForceTwistMatrix buildFrom(const vpTranslationVector &t, const vpThetaUVector &thetau);
  VP_DEPRECATED vpForceTwistMatrix buildFrom(const vpHomogeneousMatrix &M, bool full = true);

  VP_DEPRECATED vpForceTwistMatrix buildFrom(const vpRotationMatrix &R);
  VP_DEPRECATED vpForceTwistMatrix buildFrom(const vpThetaUVector &thetau);
#endif
  vpForceTwistMatrix &build(const vpTranslationVector &t, const vpRotationMatrix &R);
  vpForceTwistMatrix &build(const vpTranslationVector &t, const vpThetaUVector &thetau);
  vpForceTwistMatrix &build(const vpHomogeneousMatrix &M, bool full = true);

  vpForceTwistMatrix &build(const vpThetaUVector &thetau);
  vpForceTwistMatrix &build(const vpRotationMatrix &R);

  // Basic initialisation (identity)
  void eye();

  vpForceTwistMatrix operator*(const vpForceTwistMatrix &F) const;
  vpMatrix operator*(const vpMatrix &M) const;

  vpColVector operator*(const vpColVector &H) const;

  // copy operator from vpMatrix (handle with care)
  vpForceTwistMatrix &operator=(const vpForceTwistMatrix &H);

  int print(std::ostream &s, unsigned int length, char const *intro = nullptr) const;

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
