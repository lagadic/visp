/****************************************************************************
 *
 * $Id: vpRotationMatrix.cpp,v 1.17 2008-11-18 21:55:41 fspindle Exp $
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
 * Rotation matrix.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpRotationMatrix.cpp
  \brief Class that consider
  the particular case of rotation matrix
*/

#include <math.h>
#include <visp/vpMath.h>
#include <visp/vpMatrix.h>

// Rotation classes
#include <visp/vpRotationMatrix.h>


// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>

const double vpRotationMatrix::threshold = 1e-6;
const double vpRotationMatrix::minimum = 0.00001;

#define vpDEBUG_LEVEL1 0


/*!
  \brief initializes a 3x3 matrix as identity
*/
void
vpRotationMatrix::init()
{
  int i,j ;

  try {
    resize(3,3) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  for (i=0 ; i < 3 ; i++)
    for (j=0 ; j < 3; j++)
      if (i==j)
	(*this)[i][j] = 1.0 ;
      else
	(*this)[i][j] = 0.0;

}

//! Basic initialisation (identity)
void
vpRotationMatrix::setIdentity()
{
  init() ;
}

/*!
  Affectation of two rotation matrix.

  \param m : *this = m
*/
vpRotationMatrix &
vpRotationMatrix::operator=(const vpRotationMatrix &m)
{
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
      rowPtrs[i][j] = m.rowPtrs[i][j];
    }
  }

  return *this;
}

/*!
  Affectation of two rotation matrix

  \param m : *this = m
*/
vpRotationMatrix &
vpRotationMatrix::operator=(const vpMatrix &m)
{

  if ((m.getCols() !=3) &&(m.getRows() !=3))
    {
      vpERROR_TRACE("m is not a rotation matrix !!!!! ") ;
      throw(vpMatrixException(vpMatrixException::forbiddenOperatorError,
			  "m is not a rotation matrix !!!!!"));
    }

  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
    {
      (*this)[i][j] = m[i][j];
    }
  }

  if (isARotationMatrix() == false)
  {
    vpERROR_TRACE("m is not a rotation matrix !!!!! ") ;
      throw(vpMatrixException(vpMatrixException::forbiddenOperatorError,
			  "m is not a rotation matrix !!!!!"));
  }

  return *this;
}

//! operation C = A * B (A is unchanged)
vpRotationMatrix
vpRotationMatrix::operator*(const vpRotationMatrix &B) const
{
  vpRotationMatrix p ;

  for (int i=0;i<3;i++)
    for (int j=0;j<3;j++)
    {
      double s =0 ;
      for (int k=0;k<3;k++)
	s +=rowPtrs[i][k] * B.rowPtrs[k][j];
      p[i][j] = s ;
    }
  return p;
}

/*!

  Operator that allows to multiply a rotation matrix by a 3 dimension
  column vector.

  \param v : Three dimension column vector.

  \return The product of the rotation matrix by the column vector

  \exception vpMatrixException::incorrectMatrixSizeError If the column
  vector \e v is not a 3 dimension vector.

  The code below shows how to use this operator.
\code
#include <visp/vpRotationMatrix.h>
#include <visp/vpColVector.h>

int main()
{
  vpColVector p1(3), p2(3);
  vpRotationMatrix R;

  p2 = R * p1;
  
  return 0;
}
\endcode

*/
vpColVector
vpRotationMatrix::operator*(const vpColVector &v) const
{
  if (v.getRows() != 3) {
    vpERROR_TRACE("The column vector is not a 3 dimension vector") ;
    throw (vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			     "The column vector is not a 3 dimension vector"));
  }
  vpColVector c;
  vpMatrix::multMatrixVector(*this, v, c);
  return c;
}


/*! overload + operator (to say it forbidden operation, throw exception)


  \exception Cannot add two rotation matrices !!!!!
  vpMatrixException::forbiddenOperatorError
 */
vpRotationMatrix
vpRotationMatrix::operator+(const vpRotationMatrix &/* B*/) const
{
 vpERROR_TRACE("Cannot add two rotation matrices !!!!! ") ;
  throw(vpMatrixException(vpMatrixException::forbiddenOperatorError,
			  "Cannot add two rotation matrices !!!!!"));
}

/*! overload - operator (to say it forbidden operation, throw exception)


  \exception Cannot substract two rotation matrices !!!!!
  vpMatrixException::forbiddenOperatorError
 */
vpRotationMatrix
vpRotationMatrix::operator-(const vpRotationMatrix &/* B */) const
{
  vpERROR_TRACE("Cannot substract two rotation matrices !!!!! ") ;
  throw(vpMatrixException(vpMatrixException::forbiddenOperatorError,
			  "Cannot substract two rotation matrices !!!!!"));
}

//! operation c = A * b (A is unchanged)
vpTranslationVector
vpRotationMatrix::operator*(const vpTranslationVector &mat) const
{
  vpTranslationVector p ;

  for (int j=0;j<3;j++)p[j]=0 ;

  for (int j=0;j<3;j++) {
    for (int i=0;i<3;i++) {
      p[i]+=rowPtrs[i][j] * mat[j];
    }
  }

  return p;
}

/*********************************************************************/

/*!
  \brief  test if the 3x3 rotational part of the  rotation matrix is really a   rotation matrix
*/

bool
vpRotationMatrix::isARotationMatrix() const
{
  int i,j ;
  bool isRotation = true ;

  // test R^TR = Id ;
  vpRotationMatrix RtR = (*this).t()*(*this) ;
  for (i=0 ; i < 3 ; i++)
    for (j=0 ; j < 3 ; j++)
      if (i==j)
      {
	if (fabs(RtR[i][j]-1) > threshold)  isRotation = false ;
      }
      else
      {
	if (fabs(RtR[i][j]) > threshold)  isRotation = false ;
      }

  // test if it is a basis
  // test || Ci || = 1
  for (i=0 ; i < 3 ; i++)
    if ((sqrt(vpMath::sqr(RtR[0][i]) +
	      vpMath::sqr(RtR[1][i]) +
	      vpMath::sqr(RtR[2][i])) - 1) > threshold)  isRotation = false ;

  // test || Ri || = 1
  for (i=0 ; i < 3 ; i++)
    if ((sqrt(vpMath::sqr(RtR[i][0]) +
	      vpMath::sqr(RtR[i][1]) +
	      vpMath::sqr(RtR[i][2])) - 1) > threshold)  isRotation = false ;

  //  test if the basis is orthogonal
  return isRotation ;
}


/*!
  \brief initialize a rotation matrix as Identity
*/
vpRotationMatrix::vpRotationMatrix() : vpMatrix()
{
  init() ;
}


/*!
  \brief initialize a rotation matrix from another rotation matrix
*/

vpRotationMatrix::vpRotationMatrix(const vpRotationMatrix &M) : vpMatrix()
{
  init() ;
  (*this) = M ;
}

//! Construction from  rotation (Theta U parameterization)
vpRotationMatrix::vpRotationMatrix(const vpThetaUVector &tu) : vpMatrix()
{
  init() ;
  buildFrom(tu) ;
}


//! Construction from  rotation (Euler parameterization, ie Rzyz parameterization)
vpRotationMatrix::vpRotationMatrix(const vpRzyzVector &euler) : vpMatrix()
{
  init() ;
  buildFrom(euler) ;
}



//! Construction from  rotation Rxyz
vpRotationMatrix::vpRotationMatrix(const vpRxyzVector &Rxyz) : vpMatrix()
{
  init() ;
  buildFrom(Rxyz) ;
}

//! Construction from  rotation Rzyx
vpRotationMatrix::vpRotationMatrix(const vpRzyxVector &Rzyx) : vpMatrix()
{
  init() ;
  buildFrom(Rzyx) ;
}

//! Construction from  rotation (Theta U parameterization)
vpRotationMatrix::vpRotationMatrix(const double tux,
				   const double tuy,
				   const double tuz) : vpMatrix()
{
  init() ;
  buildFrom(tux, tuy, tuz) ;
}



/*!
  \brief transpose
  R^T
*/
vpRotationMatrix
vpRotationMatrix::t() const
{
  vpRotationMatrix Rt ;

  int i,j;
  for (i=0;i<3;i++)
    for (j=0;j<3;j++)
      Rt[j][i] = (*this)[i][j];

  return Rt;

}

/*!
  \brief inverse the rotation matrix

  \f$ R^-1 = R^T \f$
*/
vpRotationMatrix vpRotationMatrix::inverse() const
{
  vpRotationMatrix Ri = (*this).t() ;

  return Ri ;
}

/*!
  \brief inverse the rotation matrix

  \f$ R^-1 = R^T \f$
*/
void
vpRotationMatrix::inverse(vpRotationMatrix &M) const
{
  M = inverse() ;
}


//! std::cout an rotation matrix [thetaU]
std::ostream &operator <<(std::ostream &s,const vpRotationMatrix &R)
{
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
      std::cout << R[i][j] << "  " ;
    std::cout << std::endl ;
  }

  return (s);
}

//! Print the matrix as a vector [thetaU]
void
vpRotationMatrix::printVector()
{
  vpThetaUVector tu(*this) ;

  for (int i=0; i<3; i++)
    std::cout << tu[i] << "  " ;

  std::cout << std::endl ;
}


/*
  \relates vpRotationMatrix
  \brief   Transform a vector vpThetaUVector into an rotation matrix

  representation Utheta (axes and angle of the rotation) is considered for
  the rotation vector

  The rotation is computed using :
  \f[
  R = \cos{ \theta} \; {I}_{3} + (1 - \cos{ \theta}) \; v v^{T} + \sin{ \theta} \; [v]_\times
  \f]
*/
vpRotationMatrix
vpRotationMatrix::buildFrom(const vpThetaUVector &v)
{
  int i,j;
  double theta, si, co, sinc, mcosc;
  vpRotationMatrix R;

  theta = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  si = sin(theta);
  co = cos(theta);
  sinc = vpMath::sinc(si,theta);
  mcosc = vpMath::mcosc(co,theta);

  R[0][0] = co + mcosc*v[0]*v[0];
  R[0][1] = -sinc*v[2] + mcosc*v[0]*v[1];
  R[0][2] = sinc*v[1] + mcosc*v[0]*v[2];
  R[1][0] = sinc*v[2] + mcosc*v[1]*v[0];
  R[1][1] = co + mcosc*v[1]*v[1];
  R[1][2] = -sinc*v[0] + mcosc*v[1]*v[2];
  R[2][0] = -sinc*v[1] + mcosc*v[2]*v[0];
  R[2][1] = sinc*v[0] + mcosc*v[2]*v[1];
  R[2][2] = co + mcosc*v[2]*v[2];

  for (i=0;i<3;i++) for (j=0;j<3;j++) (*this)[i][j] = R[i][j];

  if (vpDEBUG_LEVEL1)  // test new version wrt old version
  {
    // old version
    vpRotationMatrix R_old; // has to be replaced by (*this) if good version
    double sinu,cosi,mcosi,u[3],ang;

    ang = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
    if (ang > minimum)
    {
      for (i=0;i<3;i++) u[i] = v[i]/ang;
      sinu = sin(ang);
      cosi = cos(ang);
      mcosi = 1-cosi;
      R_old[0][0] = cosi + mcosi*u[0]*u[0];
      R_old[0][1] = -sinu*u[2] + mcosi*u[0]*u[1];
      R_old[0][2] = sinu*u[1] + mcosi*u[0]*u[2];
      R_old[1][0] = sinu*u[2] + mcosi*u[1]*u[0];
      R_old[1][1] = cosi + mcosi*u[1]*u[1];
      R_old[1][2] = -sinu*u[0] + mcosi*u[1]*u[2];
      R_old[2][0] = -sinu*u[1] + mcosi*u[2]*u[0];
      R_old[2][1] = sinu*u[0] + mcosi*u[2]*u[1];
      R_old[2][2] = cosi + mcosi*u[2]*u[2];
    }
    else
    {
      for (i=0;i<3;i++)
      {
        for(j=0;j<3;j++) R_old[i][j] = 0.0;
        R_old[i][i] = 1.0;
      }
    }
    // end old version
    // test the new version

    int pb = 0;
    for (i=0;i<3;i++)
    {
      for(j=0;j<3;j++)
        if (fabs(R_old[i][j] - R[i][j]) > 1.e-4) pb = 1;
    }
    if (pb == 1)
    {
      printf("vpRotationMatrix::buildFrom(const vpThetaUVector &v)\n");
      std::cout << " theta " << theta << std::endl;
      std::cout << " R : " << std::endl << R << std::endl;
      std::cout << " R_old : " << std::endl << R_old << std::endl;
    }

  }
  // end test
  return *this ;
}

/*!
  \brief   Transform a vector reprensenting the euler angle
  into an rotation matrix
  Rzyz =  Rot(\f$ z,\phi \f$) Rot(\f$ y,\theta \f$) Rot(\f$ z,\psi \f$)

*/
vpRotationMatrix
vpRotationMatrix::buildFrom(const vpRzyzVector &v)
{
  double c0,c1,c2,s0,s1,s2;

  c0 = cos(v[0]);
  c1 = cos(v[1]);
  c2 = cos(v[2]);
  s0 = sin(v[0]);
  s1 = sin(v[1]);
  s2 = sin(v[2]);

  (*this)[0][0] = c0*c1*c2 - s0*s2;
  (*this)[0][1] = -c0*c1*s2 - s0*c2;
  (*this)[0][2] = c0*s1;
  (*this)[1][0] = s0*c1*c2+c0*s2 ;
  (*this)[1][1] = -s0*c1*s2 + c0*c2 ;
  (*this)[1][2] = s0*s1;
  (*this)[2][0] = -s1*c2;
  (*this)[2][1] = s1*s2;
  (*this)[2][2] = c1;

  return (*this) ;
}


/*!

  \brief Transform a vector reprensenting the Rxyz angle into an rotation
  matrix

  Rxyz(\f$ \phi,\theta, \psi \f$) = Rot(\f$ x, \psi \f$) Rot(\f$ y, \theta \f$
  ) Rot(\f$ z,\phi \f$)

*/
vpRotationMatrix
vpRotationMatrix::buildFrom(const vpRxyzVector &v)
{
  double c0,c1,c2,s0,s1,s2;

  c0 = cos(v[0]);
  c1 = cos(v[1]);
  c2 = cos(v[2]);
  s0 = sin(v[0]);
  s1 = sin(v[1]);
  s2 = sin(v[2]);

  (*this)[0][0] = c1*c2;
  (*this)[0][1] = -c1*s2;
  (*this)[0][2] = s1;
  (*this)[1][0] = c0*s2+s0*s1*c2;
  (*this)[1][1] = c0*c2-s0*s1*s2;
  (*this)[1][2] = -s0*c1;
  (*this)[2][0] = -c0*s1*c2+s0*s2;
  (*this)[2][1] = c0*s1*s2+c2*s0;
  (*this)[2][2] = c0*c1;

  return (*this) ;
}



/*!
  \brief   Transform a vector reprensenting the Rzyx angle
  into an rotation matrix

   Rxyz(\f$ \phi, \theta , \psi \f$)
   Rot(\f$ z, \psi \f$) Rot(\f$ y, \theta \f$)Rot(\f$ x, \phi \f$)
*/
vpRotationMatrix
vpRotationMatrix::buildFrom(const vpRzyxVector &v)
{
  double c0,c1,c2,s0,s1,s2;

  c0 = cos(v[0]);
  c1 = cos(v[1]);
  c2 = cos(v[2]);
  s0 = sin(v[0]);
  s1 = sin(v[1]);
  s2 = sin(v[2]);

  (*this)[0][0] = c0*c1 ;
  (*this)[0][1] = c0*s1*s2 - s0*c2 ;
  (*this)[0][2] = c0*s1*c2 + s0*s2 ;

  (*this)[1][0] = s0*c1 ;
  (*this)[1][1] = s0*s1*s2 + c0*c2 ;
  (*this)[1][2] = s0*s1*c2 - c0*s2 ;

  (*this)[2][0] = -s1 ;
  (*this)[2][1] = c1*s2 ;
  (*this)[2][2] = c1*c2 ;

  return (*this) ;
}



//! Construction from  rotation (theta U parameterization)
vpRotationMatrix
vpRotationMatrix::buildFrom(const double tux,
			    const double tuy,
			    const double tuz)
{
  vpThetaUVector tu(tux, tuy, tuz) ;
  buildFrom(tu) ;
  return *this ;
}

/****************************************************************

           Deprecated functions

*****************************************************************/

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  \deprecated This constructor is deprecated.

  Construction from rotation (Euler parameterization)

*/
vpRotationMatrix::vpRotationMatrix(const vpEulerVector &euler) : vpMatrix()
{
  init() ;
  buildFrom(euler) ;
}

/*!
  \deprecated This method is deprecated. You should use
  vpRotationMatrix::buildFrom(const vpRzyzVector &) instead.

  Transform a vector representing the euler angle
  into an rotation matrix.

  Rzyz = Rot(\f$ z,\phi \f$) Rot(\f$ y,\theta \f$) Rot(\f$ z,\psi \f$)
*/
vpRotationMatrix
vpRotationMatrix::buildFrom(const vpEulerVector &v)
{
  double c0,c1,c2,s0,s1,s2;

  c0 = cos(v[0]);
  c1 = cos(v[1]);
  c2 = cos(v[2]);
  s0 = sin(v[0]);
  s1 = sin(v[1]);
  s2 = sin(v[2]);

  (*this)[0][0] = c0*c1*c2 - s0*s2;
  (*this)[0][1] = -c0*c1*s2 - s0*c2;
  (*this)[0][2] = c0*s1;
  (*this)[1][0] = s0*c1*c2+c0*s2 ;
  (*this)[1][1] = -s0*c1*s2 + c0*c2 ;
  (*this)[1][2] = s0*s1;
  (*this)[2][0] = -s1*c2;
  (*this)[2][1] = s1*s2;
  (*this)[2][2] = c1;

  return (*this) ;
}

#endif // ifdef VISP_BUILD_DEPRECATED_FUNCTIONS


#undef vpDEBUG_LEVEL1

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
