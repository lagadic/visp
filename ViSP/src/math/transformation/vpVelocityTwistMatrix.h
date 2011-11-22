/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2011 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
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

#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRotationMatrix.h>


/*!
  \class vpVelocityTwistMatrix

  \ingroup TwistTransformation

  \brief Class that consider the particular case of twist
  transformation matrix that allows to transform a velocity skew from
  one frame to an other.

  The vpVelocityTwistMatrix is derived from vpMatrix.

  A twist transformation matrix is 6x6 matrix defined as
  \f[
  ^a{\bf V}_b = \left[\begin{array}{cc}
  ^a{\bf R}_b & [^a{\bf t}_b]_\times ^a{\bf R}_b\\
  {\bf 0}_{3\times 3} & ^a{\bf R}_b
  \end{array}
  \right]
  \f]
  that expressed a velocity in frame <em>a</em> knowing velocity in <em>b</em>.

  \f$ ^a{\bf R}_b \f$ is a rotation matrix and
  \f$ ^a{\bf t}_b \f$ is a translation vector.

  The code belows shows for example how to convert a velocity skew
  from camera frame to a fix frame.

  \code
#include <visp/vpColVector.h>
#include <visp/vpVelocityTwistMatrix.h>

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
class VISP_EXPORT vpVelocityTwistMatrix : public vpMatrix
{
  friend class vpMatrix;

 public:
  // basic constructor
  vpVelocityTwistMatrix()   ;
  // copy constructor
  vpVelocityTwistMatrix(const vpVelocityTwistMatrix &V) ;
  // constructor from an homogeneous transformation
  vpVelocityTwistMatrix(const vpHomogeneousMatrix &M) ;

  // Construction from Translation and rotation (ThetaU parameterization)
  vpVelocityTwistMatrix(const vpTranslationVector &t, const vpThetaUVector &thetau) ;
  // Construction from Translation and rotation (matrix parameterization)
  vpVelocityTwistMatrix(const vpTranslationVector &t, const vpRotationMatrix &R) ;
  vpVelocityTwistMatrix(const double tx,   const double ty,   const double tz,
			const double tux,  const double tuy,  const double tuz) ;

  // Basic initialisation (identity)
  void init() ;

  vpVelocityTwistMatrix buildFrom(const vpTranslationVector &t,
				  const vpRotationMatrix &R);
  vpVelocityTwistMatrix buildFrom(const vpTranslationVector &t,
				  const vpThetaUVector &thetau);
  vpVelocityTwistMatrix buildFrom(const vpHomogeneousMatrix &M) ;

  // Basic initialisation (identity)
  void setIdentity() ;

  vpVelocityTwistMatrix operator*(const vpVelocityTwistMatrix &V) const ;
  vpMatrix operator*(const vpMatrix &M) const ;

  vpColVector operator*(const vpColVector &v) const ;

  // copy operator from vpMatrix (handle with care)
  vpVelocityTwistMatrix &operator=(const vpVelocityTwistMatrix &V);

  //! invert the twist matrix
  vpVelocityTwistMatrix inverse() const ;
  //! invert the twist matrix
  void inverse(vpVelocityTwistMatrix &Wi) const;

  //! extract the rotational matrix from the twist matrix
  void extract( vpRotationMatrix &R) const;
  //! extract the translation vector from the twist matrix
  void extract(vpTranslationVector &t) const;
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
