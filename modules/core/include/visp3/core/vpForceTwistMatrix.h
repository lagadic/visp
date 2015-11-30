/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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
 * Twist transformation matrix that allows to transform forces from one 
 * frame to an other.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpForceTwistMatrix_h
#define vpForceTwistMatrix_h

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRotationMatrix.h>


/*!
  \class vpForceTwistMatrix

  \ingroup group_core_transformations

  \brief Implementation of a force/torque twist matrix and operations on such kind of matrices.

  Class that consider the particular case of twist
  transformation matrix that allows to transform a force/troque vector
  from one frame to an other.

  The vpForceTwistMatrix class is derived from vpArray2D<double>.

  The twist transformation matrix that allows to transform the
  force/torque vector expressed at frame \f${\cal F}_b\f$ into the
  frame \f${\cal F}_a\f$ is a 6 by 6 matrix defined as 

  \f[
  ^a{\bf F}_b = \left[ \begin{array}{cc}
  ^a{\bf R}_b & {\bf 0}_{3\times 3}\\
  {[^a{\bf t}_b]}_{\times} \; ^a{\bf R}_b & ^a{\bf R}_b
  \end{array}
  \right]
  \f]

  \f$ ^a{\bf R}_b \f$ is a rotation matrix and
  \f$ ^a{\bf t}_b \f$ is a translation vector.

  The code belows shows for example how to convert a force/torque skew
  from probe frame to a sensor frame.

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpForceTwistMatrix.h>

int main()
{
  vpForceTwistMatrix sFp; // Twist transformation matrix from sensor to probe frame

  vpHomogeneousMatrix sMp; // Force/torque sensor frame to probe frame transformation
  // ... sMp need here to be initialized

  sFp.buildFrom(sMp); 
 
  vpColVector p_H(6); // Force/torque skew in the probe frame: fx,fy,fz,tx,ty,tz 
  // ... p_H should here have an initial value

  vpColVector s_H(6); // Force/torque skew in the sensor frame: fx,fy,fz,tx,ty,tz 

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
  vpForceTwistMatrix(const vpForceTwistMatrix &F) ;
  // constructor from an homogeneous transformation
  vpForceTwistMatrix(const vpHomogeneousMatrix &M) ;

  // Construction from Translation and rotation (ThetaU parameterization)
  vpForceTwistMatrix(const vpTranslationVector &t, const vpThetaUVector &thetau) ;
  // Construction from Translation and rotation (matrix parameterization)
  vpForceTwistMatrix(const vpTranslationVector &t, const vpRotationMatrix &R) ;
  vpForceTwistMatrix(const double tx,  const double ty,  const double tz,
                     const double tux, const double tuy, const double tuz) ;
  /*!
    Destructor.
  */
  virtual ~vpForceTwistMatrix() {};

  vpForceTwistMatrix buildFrom(const vpTranslationVector &t,
                               const vpRotationMatrix &R);
  vpForceTwistMatrix buildFrom(const vpTranslationVector &t,
                               const vpThetaUVector &thetau);
  vpForceTwistMatrix buildFrom(const vpHomogeneousMatrix &M) ;

  // Basic initialisation (identity)
  void eye() ;

  vpForceTwistMatrix operator*(const vpForceTwistMatrix &F) const ;
  vpMatrix operator*(const vpMatrix &M) const ;

  vpColVector operator*(const vpColVector &H) const ;

  // copy operator from vpMatrix (handle with care)
  vpForceTwistMatrix &operator=(const vpForceTwistMatrix &H);

  int print(std::ostream& s, unsigned int length, char const* intro=0) const;

  /*!
    This function is not applicable to a velocity twist matrix that is always a
    6-by-6 matrix.
    \exception vpException::fatalError When this function is called.
    */
  void resize(const unsigned int nrows, const unsigned int ncols,
              const bool flagNullify = true)
  {
    (void)nrows;
    (void)ncols;
    (void)flagNullify;
    throw(vpException(vpException::fatalError, "Cannot resize a velocity twist matrix"));
  };

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
     \deprecated Provided only for compat with previous releases.
     This function does nothing.
   */
  vp_deprecated void init() {};
  /*!
     \deprecated You should rather use eye().
   */
  vp_deprecated void setIdentity();
  //@}
#endif
} ;

#endif
