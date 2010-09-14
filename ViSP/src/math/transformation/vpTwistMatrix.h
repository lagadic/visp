/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2010 by INRIA. All rights reserved.
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

#ifndef vpTWISTMATRIX_HH
#define vpTWISTMATRIX_HH

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRotationMatrix.h>

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

/*!
  \class vpTwistMatrix

  \ingroup TwistTransformation

  \brief Class that consider the particular case of twist
  transformation matrix that allows to transform a velocity skew from
  one frame to an other.

  \deprecated This class is deprecated. You shoud use
  vpVelocityTwistMatrix class instead.

  The vpTwistMatrix is derived from vpMatrix.

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
#include <visp/vpTwistMatrix.h>

int main()
{
  vpTwistMatrix fVc; // Twist transformation matrix from fix to camera frame

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
class VISP_EXPORT vpTwistMatrix : public vpMatrix
{
  friend class vpMatrix;

 public:
  // basic constructor
  vpTwistMatrix()   ;
  // copy constructor
  vpTwistMatrix(const vpTwistMatrix &V) ;
  // constructor from an homogeneous transformation
  vp_deprecated vpTwistMatrix(const vpHomogeneousMatrix &M) ;

  // Construction from Translation and rotation (ThetaU parameterization)
  vp_deprecated vpTwistMatrix(const vpTranslationVector &t, 
			      const vpThetaUVector &thetau) ;
  // Construction from Translation and rotation (matrix parameterization)
  vp_deprecated vpTwistMatrix(const vpTranslationVector &t, 
			      const vpRotationMatrix &R) ;
  vp_deprecated vpTwistMatrix(const double tx, const double ty, 
			      const double tz, const double tux,  
			      const double tuy,  const double tuz) ;

  // Basic initialisation (identity)
  void init() ;

  vpTwistMatrix buildFrom(const vpTranslationVector &t,
			  const vpRotationMatrix &R);
  vpTwistMatrix buildFrom(const vpTranslationVector &t,
			  const vpThetaUVector &thetau);
  vpTwistMatrix buildFrom(const vpHomogeneousMatrix &M) ;

  // Basic initialisation (identity)
  void setIdentity() ;

  vpTwistMatrix operator*(const vpTwistMatrix &V) const ;
  vpMatrix operator*(const vpMatrix &M) const ;

  vpColVector operator*(const vpColVector &v) const ;

  // copy operator from vpMatrix (handle with care)
  vpTwistMatrix &operator=(const vpTwistMatrix &V);
} ;

#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
