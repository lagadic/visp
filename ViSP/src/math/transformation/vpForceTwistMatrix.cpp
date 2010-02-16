/****************************************************************************
 *
 * $Id$
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

#include <visp/vpForceTwistMatrix.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>


/*!
  \file vpForceTwistMatrix.cpp

  \brief Definition of the vpForceTwistMatrix. Class that consider the
  particular case of twist transformation matrix that allows to
  transform a force/torque skew from one frame to an other.
*/


/*!
  Copy operator.

  \param M : Force/torque twist matrix to copy.
*/
vpForceTwistMatrix &
vpForceTwistMatrix::operator=(const vpForceTwistMatrix &M)
{
  init() ;

  for (int i=0; i<6; i++) {
    for (int j=0; j<6; j++) {
      rowPtrs[i][j] = M.rowPtrs[i][j];
    }
  }

  return *this;
}


/*!
  Initialize the force/torque 6 by 6 twist matrix as identity. 
*/

void
vpForceTwistMatrix::init()
{
  int i,j ;

  try {
    resize(6,6) ;
  }
  catch(vpException me) {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

  for (i=0 ; i < 6 ; i++) {
    for (j=0 ; j < 6; j++) {
      if (i==j)
	(*this)[i][j] = 1.0 ;
      else
	(*this)[i][j] = 0.0;
    }
  }
}

/*!
  Initialize a force/torque twist transformation matrix as identity.
*/
vpForceTwistMatrix::vpForceTwistMatrix() : vpMatrix()
{
  init() ;
}

/*!

  Initialize a force/torque twist transformation matrix from another
  force/torque twist matrix.

  \param F : Force/torque twist matrix used as initializer.
*/
vpForceTwistMatrix::vpForceTwistMatrix(const vpForceTwistMatrix &F) : vpMatrix()
{
  init() ;
  *this = F ;
}

/*!

  Initialize a force/torque twist transformation matrix from an
  homogeneous matrix. Given the homogenous transformation

  \f[ ^a{\bf M}_b = \left(\begin{array}{cc} 
  ^a{\bf R}_b & ^a{\bf t}_b\\
  0 & 1
  \end{array} \right) \f]

  the force/torque twist matrix is given by :

  \f[
  ^a{\bf F}_b = \left[\begin{array}{cc}
  ^a{\bf R}_b & {\bf 0}_{3\times 3}\\
  {[^a{\bf t}_b]}_{\times} \; ^a{\bf R}_b & ^a{\bf R}_b
  \end{array}
  \right]
  \f]

  \param M : Homogeneous matrix \f$^a{\bf M}_b\f$ used to initialize the twist
  transformation matrix. 

*/
vpForceTwistMatrix::vpForceTwistMatrix(const vpHomogeneousMatrix &M) : vpMatrix()
{
  init() ;
  buildFrom(M);
}

/*!

  Initialize a force/torque twist transformation matrix from a translation vector
  \e t and a rotation vector with \f$\theta u \f$ parametrization.

  \param t : Translation vector.
  
  \param thetau : \f$\theta u\f$ rotation vector.

*/
vpForceTwistMatrix::vpForceTwistMatrix(const vpTranslationVector &t,
				       const vpThetaUVector &thetau) : vpMatrix()
{
  init() ;
  buildFrom(t, thetau) ;
}

/*!

  Initialize a force/torque twist transformation matrix from a translation vector
  \e t and a rotation matrix R.

  \param t : Translation vector.
  
  \param R : Rotation matrix.

*/
vpForceTwistMatrix::vpForceTwistMatrix(const vpTranslationVector &t,
				       const vpRotationMatrix &R)
{
  init() ;
  buildFrom(t,R) ;
}

/*!

  Initialize a force/torque twist transformation matrix from a translation vector
  \e t and a rotation vector with \f$\theta u \f$ parametrization.

  \param tx,ty,tz : Translation vector in meters.

  \param tux,tuy,tuz : \f$\theta u\f$ rotation vector expressed in radians.
*/
vpForceTwistMatrix::vpForceTwistMatrix(const double tx,
				       const double ty,
				       const double tz,
				       const double tux,
				       const double tuy,
				       const double tuz) : vpMatrix()
{
  init() ;
  vpTranslationVector T(tx,ty,tz) ;
  vpThetaUVector tu(tux,tuy,tuz) ;
  buildFrom(T,tu) ;  
}

/*!

  Set the twist transformation matrix to identity.

*/
void
vpForceTwistMatrix::setIdentity()
{
  init() ;
}


/*!

  Operator that allows to multiply a skew transformation matrix by an
  other skew transformation matrix.

\code
#include <visp/vpForceTwistMatrix.h>

int main()
{
  vpForceTwistMatrix aFb, bFc;
  // ... initialize the force/torque twist transformations aFb and bFc
  // Compute the force/torque transformation from frame a to c
  vpForceTwistMatrix aFc = aFb * bFc;
}
\endcode

*/
vpForceTwistMatrix
vpForceTwistMatrix::operator*(const vpForceTwistMatrix &F) const
{
  vpForceTwistMatrix Fout ;

  for (int i=0;i<6;i++) {
    for (int j=0;j<6;j++) {
      double s =0 ;
      for (int k=0;k<6;k++)
	s +=rowPtrs[i][k] * F.rowPtrs[k][j];
      Fout[i][j] = s ;
    }
  }
  return Fout;
}

/*!
  Operator that allows to multiply a skew transformation matrix by a matrix.

  \exception vpMatrixException::incorrectMatrixSizeError If M is not a 6 rows
  dimension matrix.
*/
vpMatrix
vpForceTwistMatrix::operator*(const vpMatrix &M) const
{

  if (6 != M.getRows())
  {
    vpERROR_TRACE("vpForceTwistMatrix mismatch in vpForceTwistMatrix/vpMatrix multiply") ;
    throw(vpMatrixException::incorrectMatrixSizeError) ;
  }

  vpMatrix p(6, M.getCols()) ;
  for (int i=0;i<6;i++) {
    for (int j=0;j<M.getCols();j++) {
      double s =0 ;
      for (int k=0;k<6;k++)
	s += rowPtrs[i][k] * M[k][j];
      p[i][j] = s ;
    }
  }
  return p;
}

/*!

  Operator that allows to multiply a skew transformation matrix by a
  column vector.

  \param H : Force/torque skew vector \f${\bf H} = [f_x, f_y, f_z, \tau_x, \tau_y, \tau_z] \f$.

  For example, this operator can be used to convert a force/torque skew from sensor
  frame into the probe frame :

  \f$^p{\bf H} = ^p{\bf F}_s \; ^s{\bf H}\f$

  The example below shows how to handle that transformation.
  
  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotViper850.h>
#include <visp/vpColVector.h>
#include <visp/vpForceTwistMatrix.h>

int main()
{

  // Get the force/torque measures from the sensor
  vpColVector sH(6); // Force/torque measures given by the sensor

#ifdef VISP_HAVE_VIPER850
  vpRobotViper850 robot;
  robot.getForceTorque(sH); // Get the force/torque measures 
#endif  

  // Set the transformation from sensor frame to the probe frame
  vpHomogeneousMatrix pMs;
  pMs[2][3] = -0.262; // tz only

  // Set the force/torque twist transformation
  vpForceTwistMatrix pFs(pMs); // Twist transformation matrix from probe to sensor frame

  // Compute the resulting force/torque in the probe frame
  vpColVector pH(6); // Force/torque in the probe frame
  pH = pFs * sH;
 
  return 0;
}
  \endcode

  \exception vpMatrixException::incorrectMatrixSizeError If \f$ \bf H \f$is not a 6
  dimension vector.

*/
vpColVector
vpForceTwistMatrix::operator*(const vpColVector &H) const
{
  vpColVector Hout(6);

  if (6 != H.getRows())
  {
    vpERROR_TRACE("vpForceTwistMatrix mismatch in vpForceTwistMatrix/vector multiply") ;
    throw(vpMatrixException::incorrectMatrixSizeError) ;
  }

  Hout = 0.0;

  for (int i=0;i<6;i++) {
    for (int j=0;j<6;j++) {
      Hout[i]+=rowPtrs[i][j] * H[j];
    }
  }
  
  return H ;
}


/*!

  Build a force/torque twist transformation matrix from a translation vector
  \e t and a rotation matrix M.

  \param t : Translation vector.
  
  \param R : Rotation matrix.

*/
vpForceTwistMatrix
vpForceTwistMatrix::buildFrom(const vpTranslationVector &t,
			 const vpRotationMatrix &R)
{
  int i, j;
  vpMatrix skewaR = t.skew(t)*R ;
  
  for (i=0 ; i < 3 ; i++) {
    for (j=0 ; j < 3 ; j++)	{
      (*this)[i][j] = R[i][j] ;
      (*this)[i+3][j+3] = R[i][j] ;
      (*this)[i+3][j] = skewaR[i][j] ;
    }
  }
  return (*this) ;
}

/*!

  Initialize a force/torque twist transformation matrix from a translation vector
  \e t and a rotation vector with \f$\theta u \f$ parametrization.

  \param t : Translation vector.
  
  \param thetau : \f$\theta u\f$ rotation vector.

*/
vpForceTwistMatrix
vpForceTwistMatrix::buildFrom(const vpTranslationVector &t,
			      const vpThetaUVector &thetau)
{
  vpRotationMatrix R ;
  R.buildFrom(thetau) ;
  buildFrom(t,R) ;
  return (*this) ;
}


/*!

  Initialize a force/torque twist transformation matrix from an homogeneous matrix
  \f$M\f$ with \f[ {\bf M} = \left[\begin{array}{cc} {\bf R} & {\bf t}
  \\ {\bf 0}_{1\times 3} & 1 \end{array} \right] \f]

  \param M : Homogeneous matrix \f$M\f$ used to initialize the force/torque twist
  transformation matrix.

*/
vpForceTwistMatrix
vpForceTwistMatrix::buildFrom(const vpHomogeneousMatrix &M)
{
  vpTranslationVector t ;
  vpRotationMatrix R ;
  M.extract(R) ;
  M.extract(t) ;
  
  buildFrom(t, R) ;
  return (*this) ;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
