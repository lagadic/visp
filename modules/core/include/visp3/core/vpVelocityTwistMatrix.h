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
 * Velocity twist transformation matrix.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpVelocityRwistMatrix_h
#define vpVelocityRwistMatrix_h

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRotationMatrix.h>

class vpHomogeneousMatrix;
class vpColVector;

/*!
  \class vpVelocityTwistMatrix

  \ingroup group_core_transformations

  \brief Implementation of a velocity twist matrix and operations on such kind of matrices.

  Class that consider the particular case of twist
  transformation matrix that allows to transform a velocity skew from
  one frame to an other.

  The vpVelocityTwistMatrix class is derived from vpArray2D<double>.

  A twist transformation matrix is a 6x6 matrix that express a velocity in frame <em>a</em> knowing
  velocity in <em>b</em>. This matrix is defined as:
  \f[
  ^a{\bf V}_b = \left[\begin{array}{cc}
  ^a{\bf R}_b & [^a{\bf t}_b]_\times \; ^a{\bf R}_b\\
  {\bf 0}_{3\times 3} & ^a{\bf R}_b
  \end{array}
  \right]
  \f]

  where \f$ ^a{\bf R}_b \f$ is a rotation matrix and
  \f$ ^a{\bf t}_b \f$ is a translation vector.

  The vpVelocityTwistMatrix is derived from vpArray2D.

  The code belows shows for example how to convert a velocity skew
  from camera frame to a fix frame.

  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

int main()
{
  vpVelocityTwistMatrix fVc; // Twist transformation matrix from fix to camera frame

  vpHomogeneousMatrix fMc; // Fix to camera frame transformation
  // ... fMc need here to be initialized

  fVc.buildFrom(fMc); 
 
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
  vpVelocityTwistMatrix(const vpHomogeneousMatrix &M);

  // Construction from Translation and rotation (ThetaU parameterization)
  vpVelocityTwistMatrix(const vpTranslationVector &t, const vpThetaUVector &thetau) ;
  // Construction from Translation and rotation (matrix parameterization)
  vpVelocityTwistMatrix(const vpTranslationVector &t, const vpRotationMatrix &R);
  vpVelocityTwistMatrix(const double tx,  const double ty,  const double tz,
                        const double tux, const double tuy, const double tuz);
  /*!
    Destructor.
  */
  virtual ~vpVelocityTwistMatrix() {};


  vpVelocityTwistMatrix buildFrom(const vpTranslationVector &t,
                                  const vpRotationMatrix &R);
  vpVelocityTwistMatrix buildFrom(const vpTranslationVector &t,
                                  const vpThetaUVector &thetau);
  vpVelocityTwistMatrix buildFrom(const vpHomogeneousMatrix &M) ;

  void extract( vpRotationMatrix &R) const;
  void extract(vpTranslationVector &t) const;

  // Basic initialisation (identity)
  void eye() ;

  vpVelocityTwistMatrix inverse() const ;
  void inverse(vpVelocityTwistMatrix &V) const;

  vpVelocityTwistMatrix operator*(const vpVelocityTwistMatrix &V) const ;
  vpMatrix operator*(const vpMatrix &M) const ;
  vpColVector operator*(const vpColVector &v) const ;

  vpVelocityTwistMatrix &operator=(const vpVelocityTwistMatrix &V);

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
