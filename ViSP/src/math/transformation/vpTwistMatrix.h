/****************************************************************************
 *
 * $Id: vpTwistMatrix.h,v 1.5 2008-09-26 15:20:55 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit.
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Twist transformation matrix.
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


/*!
  \class vpTwistMatrix

  \ingroup TwistTransformation

  \brief Class that consider the particular case of twist
  transformation matrix. 

  The vpTwistMatrix is derived from vpMatrix.

  A twist transformation matrix is 6x6 matrix defined as
  \f[
  ^a{\bf V}_b = \left(\begin{array}{cc}
  ^a{\bf R}_b & [^a{\bf t}_b]_\times ^a{\bf R}_b\\
  {\bf 0}_{1\times 3} & ^a{\bf R}_b
  \end{array}
  \right)
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
  vpTwistMatrix(const vpTwistMatrix &M) ;
  // constructor from an homogeneous transformation
  vpTwistMatrix(const vpHomogeneousMatrix &M) ;

  // Construction from Translation and rotation (ThetaU parameterization)
  vpTwistMatrix(const vpTranslationVector &t, const vpThetaUVector &thetau) ;
  // Construction from Translation and rotation (matrix parameterization)
  vpTwistMatrix(const vpTranslationVector &t, const vpRotationMatrix &R) ;
  vpTwistMatrix(const double tx,   const double ty,   const double tz,
		const double tux,  const double tuy,  const double tuz) ;

  // Basic initialisation (identity)
  void init() ;

  vpTwistMatrix buildFrom(const vpTranslationVector &t,
			  const vpRotationMatrix &R);
  vpTwistMatrix buildFrom(const vpTranslationVector &t,
			  const vpThetaUVector &thetau);
  vpTwistMatrix buildFrom(const vpHomogeneousMatrix &M) ;

  // Basic initialisation (identity)
  void setIdentity() ;

  vpTwistMatrix operator*(const vpTwistMatrix &M) const ;
  vpMatrix operator*(const vpMatrix &M) const ;

  vpColVector operator*(const vpColVector &v) const ;

  // Assigment
  vpTwistMatrix &operator<<(const vpTwistMatrix &M);

  // copy operator from vpMatrix (handle with care)
  vpTwistMatrix &operator=(const vpTwistMatrix &M);

  /*!
    @name Deprecated functions
  */
  vpTwistMatrix buildFrom(const vpTranslationVector &t,
			  const vpEulerVector &euler);

} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
