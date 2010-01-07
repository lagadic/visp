/****************************************************************************
 *
 * $Id$
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
 *
 *****************************************************************************/

#include <visp/vpTwistMatrix.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>


/*!
  \file vpTwistMatrix.cpp
  \brief Definition of the vpTwistMatrix. Class that consider
   the particular case of twist transformation matrix
*/


/*!
  Copy operator.

  \param M : Twist matrix to copy.
*/
vpTwistMatrix &
vpTwistMatrix::operator=(const vpTwistMatrix &M)
{
  init() ;

  for (int i=0; i<6; i++)
  {
    for (int j=0; j<6; j++)
    {
      rowPtrs[i][j] = M.rowPtrs[i][j];
    }
  }

  return *this;
}


/*!
  Initialize a 6x6 matrix as identity. 
*/

void
vpTwistMatrix::init()
{
  int i,j ;

  try {
    resize(6,6) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

  for (i=0 ; i < 6 ; i++)
    for (j=0 ; j < 6; j++)
      if (i==j)
	(*this)[i][j] = 1.0 ;
      else
	(*this)[i][j] = 0.0;
}

/*!
  Initialize a twist transformation matrix as identity.
*/
vpTwistMatrix::vpTwistMatrix() : vpMatrix()
{
  init() ;
}

/*!
  Initialize a twist transformation matrix from another twist matrix.

  \param M : Twist matrix used as initializer.
*/
vpTwistMatrix::vpTwistMatrix(const vpTwistMatrix &M) : vpMatrix()
{
  init() ;
  *this = M ;
}

/*!

  Initialize a twist transformation matrix from an homogeneous matrix
  \f$M\f$ with \f[ {\bf M} = \left(\begin{array}{cc} {\bf R} & {\bf t}
  \\ {\bf 0}_{1\times 3} & 1 \end{array} \right) \f]

  \param M : Homogeneous matrix \f$M\f$ used to initialize the twist
  transformation matrix.

*/
vpTwistMatrix::vpTwistMatrix(const vpHomogeneousMatrix &M) : vpMatrix()
{
  init() ;
  buildFrom(M);
}

/*!

  Initialize a twist transformation matrix from a translation vector
  \e t and a rotation vector with \f$\theta u \f$ parametrization.

  \param t : Translation vector.
  
  \param thetau : \f$\theta u\f$ rotation vector.

*/
vpTwistMatrix::vpTwistMatrix(const vpTranslationVector &t,
			     const vpThetaUVector &thetau) : vpMatrix()
{
  init() ;
  buildFrom(t, thetau) ;
}

/*!

  Initialize a twist transformation matrix from a translation vector
  \e t and a rotation matrix M.

  \param t : Translation vector.
  
  \param R : Rotation matrix.

*/
vpTwistMatrix::vpTwistMatrix(const vpTranslationVector &t,
                             const vpRotationMatrix &R)
{
  init() ;
  buildFrom(t,R) ;
}

/*!

  Initialize a twist transformation matrix from a translation vector
  \e t and a rotation vector with \f$\theta u \f$ parametrization.

  \param tx,ty,tz : Translation vector in meters.

  \param tux,tuy,tuz : \f$\theta u\f$ rotation vector expressed in radians.
*/
vpTwistMatrix::vpTwistMatrix(const double tx,
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
vpTwistMatrix::setIdentity()
{
  init() ;
}


/*!

  Operator that allows to multiply a skew transformation matrix by an
  other skew transformation matrix.

*/
vpTwistMatrix
vpTwistMatrix::operator*(const vpTwistMatrix &M) const
{
  vpTwistMatrix p ;

  for (int i=0;i<6;i++)
    for (int j=0;j<6;j++)
    {
      double s =0 ;
      for (int k=0;k<6;k++)
	s +=rowPtrs[i][k] * M.rowPtrs[k][j];
      p[i][j] = s ;
    }
  return p;
}

/*!
  Operator that allows to multiply a skew transformation matrix by a matrix.

  As shown in the example below, this operator can be used to convert
  a joint velocity in the camera frame knowing the robot jacobian.

  \code
#include <visp/vpConfig.h>
#include <visp/vpRobotAfma6.h>
#include <visp/vpColVector.h>
#include <visp/vpTwistMatrix.h>

#ifdef VISP_HAVE_AFMA6

int main()
{
  vpRobotAfma6 robot;

  vpColVector q_vel(6); // Joint velocity on the 6 joints
  // ... q_vel need here to be initialized
  
  vpColVector c_v(6); // Velocity in the camera frame: vx,vy,vz,wx,wy,wz 

  vpTwistMatrix cVe;  // Velocity skew transformation from camera frame to end-effector
  robot.get_cVe(cVe);

  vpMatrix eJe;       // Robot jacobian
  robot.get_eJe(eJe);

  // Compute the velocity in the camera frame
  c_v = cVe * eJe * q_vel;

  return 0;
}
#endif  
  \endcode

  \exception vpMatrixException::incorrectMatrixSizeError If M is not a 6 rows
  dimension matrix.
*/
vpMatrix
vpTwistMatrix::operator*(const vpMatrix &M) const
{

  if (6 != M.getRows())
  {
    vpERROR_TRACE("vpTwistMatrix mismatch in vpTwistMatrix/vpMatrix multiply") ;
    throw(vpMatrixException::incorrectMatrixSizeError) ;
  }

  vpMatrix p(6, M.getCols()) ;
  for (int i=0;i<6;i++)
    for (int j=0;j<M.getCols();j++)
    {
      double s =0 ;
      for (int k=0;k<6;k++)
	s += rowPtrs[i][k] * M[k][j];
      p[i][j] = s ;
    }
  return p;
}

/*!

  Operator that allows to multiply a skew transformation matrix by a
  column vector.

  Operation c = A * v (A the skew transformation matrix is unchanged,
  c and v are 6 dimension vectors).

  \param v : Velocity skew vector.

  \exception vpMatrixException::incorrectMatrixSizeError If v is not a 6
  dimension vector.

*/
vpColVector
vpTwistMatrix::operator*(const vpColVector &v) const
{
  vpColVector c(6);

  if (6 != v.getRows())
  {
    vpERROR_TRACE("vpTwistMatrix mismatch in vpTwistMatrix/vector multiply") ;
    throw(vpMatrixException::incorrectMatrixSizeError) ;
  }

  c = 0.0;

  for (int i=0;i<6;i++) {
    for (int j=0;j<6;j++) {
      {
 	c[i]+=rowPtrs[i][j] * v[j];
      }
    }
  }

  return c ;
}


/*!

  Build a twist transformation matrix from a translation vector
  \e t and a rotation matrix M.

  \param t : Translation vector.
  
  \param R : Rotation matrix.

*/
vpTwistMatrix
vpTwistMatrix::buildFrom(const vpTranslationVector &t,
			 const vpRotationMatrix &R)
{
    int i, j;
    vpMatrix skewaR = t.skew(t)*R ;

    for (i=0 ; i < 3 ; i++)
	for (j=0 ; j < 3 ; j++)
	{
	    (*this)[i][j] = R[i][j] ;
	    (*this)[i+3][j+3] = R[i][j] ;
	    (*this)[i][j+3] = skewaR[i][j] ;

	}
    return (*this) ;
}

/*!

  Initialize a twist transformation matrix from a translation vector
  \e t and a rotation vector with \f$\theta u \f$ parametrization.

  \param t : Translation vector.
  
  \param thetau : \f$\theta u\f$ rotation vector.

*/
vpTwistMatrix
vpTwistMatrix::buildFrom(const vpTranslationVector &t,
			 const vpThetaUVector &thetau)
{
    vpRotationMatrix R ;
    R.buildFrom(thetau) ;
    buildFrom(t,R) ;
    return (*this) ;

}


/*!

  Initialize a twist transformation matrix from an homogeneous matrix
  \f$M\f$ with \f[ {\bf M} = \left(\begin{array}{cc} {\bf R} & {\bf t}
  \\ {\bf 0}_{1\times 3} & 1 \end{array} \right) \f]

  \param M : Homogeneous matrix \f$M\f$ used to initialize the twist
  transformation matrix.

*/
vpTwistMatrix
vpTwistMatrix::buildFrom(const vpHomogeneousMatrix &M)
{
    vpTranslationVector t ;
    vpRotationMatrix R ;
    M.extract(R) ;
    M.extract(t) ;

    buildFrom(t, R) ;
    return (*this) ;
}
/****************************************************************

           Deprecated functions

*****************************************************************/

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!

  \deprecated Build a twist transformation matrix from a translation vector
  \e t and a euler vector.

  \param t : Translation vector.
  
  \param euler : Euler vector.

*/
vpTwistMatrix
vpTwistMatrix::buildFrom(const vpTranslationVector &t,
			 const vpEulerVector &euler)
{
  vpRotationMatrix R ;
  vpRzyzVector rzyz;
  for (int i=0; i < 3; i ++)
    rzyz[i] = euler[i];
  R.buildFrom(rzyz) ;
  buildFrom(t,R) ;
  return (*this) ;
}

#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
