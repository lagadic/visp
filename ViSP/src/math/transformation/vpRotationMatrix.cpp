


/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRotationMatrix.cpp
 * Project:   ViSP2
 *
 * Version control
 * ===============
 *
 *  $Id: vpRotationMatrix.cpp,v 1.5 2005-07-21 14:55:04 obourqua Exp $
 *
 * Description
 * ============
 *     Class that consider the particular case of rotation matrix
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \file vpRotationMatrix.cpp
  \brief Class that consider
  the particular case of rotation matrix
*/

#include <math.h>
#include <visp/vpMath.h>
#include <visp/vpMatrix.h>

// Rotation classes
#include <visp/vpRxyzVector.h>
#include <visp/vpRzyxVector.h>
#include <visp/vpEulerVector.h>
#include <visp/vpRotationMatrix.h>


// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>


/*!
  \class vpRotationMatrix

  \brief the vpRotationMatrix is derived from vpMatrix.
  It considers the particular case of rotation matrix

  \author  Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes

*/



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
    ERROR_TRACE("Error caught") ;
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
  \brief  affectation of two rotation matrix

  \param vpRotationMatrix &m : *this = m
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
  \brief  affectation of two rotation matrix

  \param vpRotationMatrix &m : *this = m
*/
vpRotationMatrix &
vpRotationMatrix::operator=(const vpMatrix &m)
{

  if ((m.getCols() !=3) &&(m.getRows() !=3))
    {
      ERROR_TRACE("m is not a rotation matrix !!!!! ") ;
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
    ERROR_TRACE("m is not a rotation matrix !!!!! ") ;
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

/*! overload + operator (to say it forbidden operation, throw exception)


  \exception Cannot add two rotation matrices !!!!!
  vpMatrixException::forbiddenOperatorError
 */
vpRotationMatrix
vpRotationMatrix::operator+(const vpRotationMatrix &B) const
{
 ERROR_TRACE("Cannot add two rotation matrices !!!!! ") ;
  throw(vpMatrixException(vpMatrixException::forbiddenOperatorError,
			  "Cannot add two rotation matrices !!!!!"));

  return vpRotationMatrix();
}

/*! overload - operator (to say it forbidden operation, throw exception)


  \exception Cannot substract two rotation matrices !!!!!
  vpMatrixException::forbiddenOperatorError
 */
vpRotationMatrix
vpRotationMatrix::operator-(const vpRotationMatrix &B) const
{
  ERROR_TRACE("Cannot substract two rotation matrices !!!!! ") ;
  throw(vpMatrixException(vpMatrixException::forbiddenOperatorError,
			  "Cannot substract two rotation matrices !!!!!"));

  return vpRotationMatrix();
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
#ifdef THRESHOLD
#undef THRESHOLD
#endif
#define THRESHOLD 1e-6

bool
vpRotationMatrix::isARotationMatrix() const
{
  int i,j ;
  int isRotation = true ;

  // test R^TR = Id ;
  vpRotationMatrix RtR = (*this).t()*(*this) ;
  for (i=0 ; i < 3 ; i++)
    for (j=0 ; j < 3 ; j++)
      if (i==j)
      {
	if (fabs(RtR[i][j]-1) > THRESHOLD)  isRotation = false ;
      }
      else
      {
	if (fabs(RtR[i][j]) > THRESHOLD)  isRotation = false ;
      }

  // test if it is a basis
  // test || Ci || = 1
  for (i=0 ; i < 3 ; i++)
    if ((sqrt(vpMath::sqr(RtR[0][i]) +
	      vpMath::sqr(RtR[1][i]) +
	      vpMath::sqr(RtR[2][i])) - 1) > THRESHOLD)  isRotation = false ;

  // test || Ri || = 1
  for (i=0 ; i < 3 ; i++)
    if ((sqrt(vpMath::sqr(RtR[i][0]) +
	      vpMath::sqr(RtR[i][1]) +
	      vpMath::sqr(RtR[i][2])) - 1) > THRESHOLD)  isRotation = false ;

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

//! Construction from  rotation (Euler parameterization)
vpRotationMatrix::vpRotationMatrix(const vpEulerVector &euler) : vpMatrix()
{
  init() ;
  buildFrom(euler) ;
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

  R^-1 = R^T
*/
vpRotationMatrix vpRotationMatrix::inverse() const
{
  vpRotationMatrix Ri = (*this).t() ;

  return Ri ;
}

/*!
  \brief inverse the rotation matrix

  R^-1 = R^T
*/
void
vpRotationMatrix::inverse(vpRotationMatrix &M) const
{
  M = inverse() ;
}


//! cout an rotation matrix [thetaU]
ostream &operator <<(ostream &s,const vpRotationMatrix &R)
{
  for (int i=0; i<3; i++)
  {
    for (int j=0; j<3; j++)
      cout << R[i][j] << "  " ;
    cout << endl ;
  }

  return (s);
}

//! Print the matrix as a vector [thetaU]
void
vpRotationMatrix::printVector()
{
  vpThetaUVector tu(*this) ;

  for (int i=0; i<3; i++)
    cout << tu[i] << "  " ;

  cout << endl ;
}

#define MINIMUM 0.0001

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
  double sinu,cosi,mcosi,u[3],ang;

  ang = sqrt(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);
  if (ang > MINIMUM)
  {
    for (i=0;i<3;i++) u[i] = v[i]/ang;
    sinu = sin(ang);
    cosi = cos(ang);
    mcosi = 1-cosi;
    (*this)[0][0] = cosi + mcosi*u[0]*u[0];
    (*this)[0][1] = -sinu*u[2] + mcosi*u[0]*u[1];
    (*this)[0][2] = sinu*u[1] + mcosi*u[0]*u[2];
    (*this)[1][0] = sinu*u[2] + mcosi*u[1]*u[0];
    (*this)[1][1] = cosi + mcosi*u[1]*u[1];
    (*this)[1][2] = -sinu*u[0] + mcosi*u[1]*u[2];
    (*this)[2][0] = -sinu*u[1] + mcosi*u[2]*u[0];
    (*this)[2][1] = sinu*u[0] + mcosi*u[2]*u[1];
    (*this)[2][2] = cosi + mcosi*u[2]*u[2];
  }
  else
  {
    for (i=0;i<3;i++)
    {
      for(j=0;j<3;j++) (*this)[i][j] = 0.0;
      (*this)[i][i] = 1.0;
    }
  }
  return *this ;
}
#undef MINIMUM

/*!
  \brief   Transform a vector reprensenting the euler angle
  into an rotation matrix

  Rzyz = Rot(z,\Phi)Rot(y,\theta)Rot(z,\psi)
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

/*!
  \brief   Transform a vector reprensenting the euler angle
  into an rotation matrix
  Rzyz =  Rot(z,\Phi)Rot(y,\theta)Rot(z,\psi)
*/
vpRotationMatrix
vpRotationMatrix::buildFrom(const vpRzyzVector &v)
{
  buildFrom((vpEulerVector)v) ;
  return (*this) ;
}


/*!
  \brief   Transform a vector reprensenting the Rxyz angle
  into an rotation matrix

   Rxyz(\phi,\theta, \psy) = Rot(x,\Psy)Rot(y,\theta)Rot(z,\phi)
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

   Rxyz(\phi,\theta, \psy)
   Rot(z,\Psy)Rot(y,\theta)Rot(x,\phi)
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



#undef DEBUG_LEVEL1

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
