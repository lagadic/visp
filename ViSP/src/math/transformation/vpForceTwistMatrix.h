/****************************************************************************
 *
 * $Id: vpTwistMatrix.h 2453 2010-01-07 10:01:10Z nmelchio $
 *
 * Copyright (C) 1998-2010 Inria. All rights reserved.
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
 * Twist transformation matrix that allows to transform forces from one 
 * frame to an other.
 *
 * Authors:
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpForceTwistMatrix_h
#define vpForceTwistMatrix_h

#include <visp/vpConfig.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpRotationMatrix.h>


/*!
  \class vpForceTwistMatrix

  \ingroup TwistTransformation

  \brief Class that consider the particular case of twist
  transformation matrix that allows to transform a force/troque vector
  from one frame to an other.

  The vpForceTwistMatrix is derived from vpMatrix.

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
#include <visp/vpColVector.h>
#include <visp/vpForceTwistMatrix.h>

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
class VISP_EXPORT vpForceTwistMatrix : public vpMatrix
{
  friend class vpMatrix;

 public:
  // basic constructor
  vpForceTwistMatrix()   ;
  // copy constructor
  vpForceTwistMatrix(const vpForceTwistMatrix &F) ;
  // constructor from an homogeneous transformation
  vpForceTwistMatrix(const vpHomogeneousMatrix &M) ;

  // Construction from Translation and rotation (ThetaU parameterization)
  vpForceTwistMatrix(const vpTranslationVector &t, const vpThetaUVector &thetau) ;
  // Construction from Translation and rotation (matrix parameterization)
  vpForceTwistMatrix(const vpTranslationVector &t, const vpRotationMatrix &R) ;
  vpForceTwistMatrix(const double tx,   const double ty,   const double tz,
		     const double tux,  const double tuy,  const double tuz) ;

  // Basic initialisation (identity)
  void init() ;

  vpForceTwistMatrix buildFrom(const vpTranslationVector &t,
			       const vpRotationMatrix &R);
  vpForceTwistMatrix buildFrom(const vpTranslationVector &t,
			       const vpThetaUVector &thetau);
  vpForceTwistMatrix buildFrom(const vpHomogeneousMatrix &M) ;

  // Basic initialisation (identity)
  void setIdentity() ;

  vpForceTwistMatrix operator*(const vpForceTwistMatrix &F) const ;
  vpMatrix operator*(const vpMatrix &M) const ;

  vpColVector operator*(const vpColVector &H) const ;

  // copy operator from vpMatrix (handle with care)
  vpForceTwistMatrix &operator=(const vpForceTwistMatrix &H);
} ;

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
