
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpHomogeneousMatrix.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      vpHomogeneousMatrix.cpp, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpHomogeneousMatrix.cpp,v 1.2 2005-06-28 08:31:20 marchand Exp $
 *
 * Description
 * ============
 *     Class that consider the particular case of homogeneous matrix
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \file vpHomogeneousMatrix.cpp
  \brief Définition de la classe vpHomogeneousMatrix. Class that consider
  the particular case of homogeneous matrix
*/

#include <visp/vpDebug.h>
#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>



/*!
  \brief initialiaze a 4x4 matrix as identity
*/

void
vpHomogeneousMatrix::init()
{
  int i,j ;

  try {
    resize(4,4) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE("Error caught") ;
    throw ;
  }


  for (i=0 ; i < 4 ; i++)
    for (j=0 ; j < 4; j++)
      if (i==j)
	(*this)[i][j] = 1.0 ;
      else
	(*this)[i][j] = 0.0;

}

/*!
  \brief initialize an homogeneous matrix as Identity
*/
vpHomogeneousMatrix::vpHomogeneousMatrix() : vpMatrix()
{
  init() ;
}


/*!
  \brief initialize an homogeneous matrix from another homogeneous matrix
*/

vpHomogeneousMatrix::vpHomogeneousMatrix(const vpHomogeneousMatrix &M) : vpMatrix()
{
  init() ;
  *this = M ;
}

vpHomogeneousMatrix::vpHomogeneousMatrix(const vpThetaUVector &tu,
					 const vpTranslationVector &T) : vpMatrix()
{
  init() ;
  buildFrom(tu,T) ;
}

vpHomogeneousMatrix::vpHomogeneousMatrix(const vpRotationMatrix &R,
					 const vpTranslationVector &T) : vpMatrix()
{
  init() ;
  insert(R) ;
  insert(T) ;
}

vpHomogeneousMatrix::vpHomogeneousMatrix(const vpPoseVector &p) : vpMatrix()
{

  init() ;
  buildFrom(p[0],p[1],p[2],p[3],p[4],p[5]) ;
}

vpHomogeneousMatrix::vpHomogeneousMatrix(const double Tx,
					 const double Ty,
					 const double Tz,
					 const double tux,
					 const double tuy,
					 const double tuz) : vpMatrix()
{
  init() ;
  buildFrom(Tx, Ty, Tz,tux, tuy, tuz) ;
}



void
vpHomogeneousMatrix::buildFrom(const vpThetaUVector &tu,
			       const vpTranslationVector &T)
{


  insert(tu) ;
  insert(T) ;
}

void
vpHomogeneousMatrix::buildFrom(const vpRotationMatrix &R,
			       const vpTranslationVector &T)
{
  init() ;
  insert(R) ;
  insert(T) ;
}

void
vpHomogeneousMatrix::buildFrom(const vpPoseVector &p)
{

  vpTranslationVector T(p[0],p[1],p[2]) ;
  vpThetaUVector tu(p[3],p[4],p[5]) ;

  insert(tu) ;
  insert(T) ;
}

void
vpHomogeneousMatrix::buildFrom(const double Tx,
			       const double Ty,
			       const double Tz,
			       const double tux,
			       const double tuy,
			       const double tuz)
{
  vpRotationMatrix R(tux, tuy, tuz) ;
  vpTranslationVector T(Tx, Ty, Tz) ;

  insert(R) ;
  insert(T) ;
}

/*!
  \brief  affectation of two homogeneous matrix

  \param vpHomogeneousMatrix &m : *this = m
*/
vpHomogeneousMatrix &
vpHomogeneousMatrix::operator=(const vpHomogeneousMatrix &m)
{

  if (rowPtrs != m.rowPtrs) init() ;

  for (int i=0; i<4; i++)
  {
    for (int j=0; j<4; j++)
    {
      rowPtrs[i][j] = m.rowPtrs[i][j];
    }
  }
  return *this;
}

/*!
  \brief Homogeneous matrix multiplication

  aMb = aMc*cMb

*/
vpHomogeneousMatrix
vpHomogeneousMatrix::operator*(const vpHomogeneousMatrix &mat) const
{
  vpHomogeneousMatrix p,p1 ;

  vpRotationMatrix R1, R2, R ;
  vpTranslationVector T1, T2 , T;


  extract(T1) ;
  mat.extract(T2) ;

  extract (R1) ;
  mat.extract (R2) ;

  R = R1*R2 ;

  T = R1*T2 + T1 ;

  p.insert(T) ;
  p.insert(R) ;

  return p;
}

vpColVector
vpHomogeneousMatrix::operator*(vpColVector &v) const
{
  vpColVector p(rowNum);

  p = 0.0;

  for (int j=0;j<4;j++) {
    for (int i=0;i<4;i++) {
      p[i]+=rowPtrs[i][j] * v[j];
    }
  }

  return p;
}


/*********************************************************************/

/*!
  \brief  test if the 3x3 rotational part of the  homogeneous matrix is really a   rotation matrix
*/

bool
vpHomogeneousMatrix::isAnHomogeneousMatrix() const
{
  vpRotationMatrix R ;
  extract(R) ;

  return  R.isARotationMatrix() ;
}

/*!
  \brief extract the rotational component of the homogeneous matrix
  \param R : rotational component
*/
void
vpHomogeneousMatrix::extract(vpRotationMatrix &R) const
{
  int i,j ;

  for (i=0 ; i < 3 ; i++)
    for (j=0 ; j < 3; j++)
      R[i][j] = (*this)[i][j] ;
}

/*!
  \brief extract the translational component of the homogeneous matrix
*/
void
vpHomogeneousMatrix::extract(vpTranslationVector &T) const
{

  T[0] = (*this)[0][3] ;
  T[1] = (*this)[1][3] ;
  T[2] = (*this)[2][3] ;
}


/*!
  \brief insert the rotational component of the homogeneous matrix
*/
void
vpHomogeneousMatrix::insert(const vpRotationMatrix &R)
{
  int i,j ;

  for (i=0 ; i < 3 ; i++)
    for (j=0 ; j < 3; j++)
      (*this)[i][j] = R[i][j] ;
}


/*!  \brief insert the rotational component of the homogeneous matrix, insert a
  theta u vector (transformation into a rotation matrix)

*/
void
vpHomogeneousMatrix::insert(const vpThetaUVector &tu)
{
  vpRotationMatrix R(tu) ;
  insert(R) ;
}


/*!
  \brief  insert the translational component in a homogeneous matrix
*/
void
vpHomogeneousMatrix::insert(const vpTranslationVector &T)
{
  (*this)[0][3] = T[0] ;
  (*this)[1][3] = T[1] ;
  (*this)[2][3] = T[2] ;
}


/*!
  \relates  vpHomogeneousMatrix
  \brief invert the homogeneous matrix

  [R T]^-1 = [R^T  -R^T T]

  \return   [R T]^-1
*/
vpHomogeneousMatrix
vpHomogeneousMatrix::inverse() const
{
  vpHomogeneousMatrix Mi ;


  vpRotationMatrix R ;      extract(R) ;
  vpTranslationVector T ;   extract(T) ;

  vpTranslationVector RtT ; RtT = -(R.t()*T) ;


  Mi.insert(R.t()) ;
  Mi.insert(RtT) ;

  return Mi ;
}

/*!
  \relates  vpHomogeneousMatrix
  \brief invert the homogeneous matrix

  [R T]^-1 = [R^T  -R^T T]

  \param Mi  [R T]^-1
*/
void
vpHomogeneousMatrix::inverse(vpHomogeneousMatrix &M) const
{
  M = inverse() ;
}


void
vpHomogeneousMatrix::save(ofstream &f) const
{
  if (f != NULL)
  {
    f << *this ;
  }
  else
  {
    ERROR_TRACE("\t\t file not open " );
    throw(vpException(vpException::ioERR, "\t\t file not open")) ;
  }
}


/*!
  Read an homogeneous matrix in a file, verify if it is really an homogeneous
  matrix

  \param ifstream &f : the file
*/
void
vpHomogeneousMatrix::load(ifstream &f)
{
  if (f != NULL)
  {
    for (int i=0 ; i < 4 ; i++)
      for (int j=0 ; j < 4 ; j++)
      {
	f>>   (*this)[i][j] ;
      }
  }
  else
  {
    ERROR_TRACE("\t\t file not open " );
    throw(vpException(vpException::ioERR, "\t\t file not open")) ;
  }
}

/*!
  Read an homogeneous matrix in a file, verify if it is really an homogeneous
  matrix

  \param ifstream &f : the file
*/
void
vpHomogeneousMatrix::loadMatrix34(ifstream &f)
{
  if (f != NULL)
  {
    for (int i=0 ; i < 3 ; i++)
      for (int j=0 ; j < 4 ; j++)
      {
	f>>   (*this)[i][j] ;
      }
  }
  else
  {
    ERROR_TRACE("\t\t file not open " );
    throw(vpException(vpException::ioERR, "\t\t file not open")) ;
  }
}



//! Print the matrix as a vector [T thetaU]
void
vpHomogeneousMatrix::print()
{
  vpPoseVector r(*this) ;
  cout << r.t() ;
}
//! Basic initialisation (identity)
void
vpHomogeneousMatrix::setIdentity()
{
  init() ;
}


vpHomogeneousMatrix
vpHomogeneousMatrix::expMap(const vpColVector &dx) const
{

  vpHomogeneousMatrix mati ;

  mati = *this ;

  int i,j;

  double sinu,cosi,mcosi,u[3],s;
  vpRotationMatrix rd ;
  vpTranslationVector dt ;

  s = sqrt(dx[3]*dx[3] + dx[4]*dx[4] + dx[5]*dx[5]);
  if (s > 1.e-15)
  {
    for (i=0;i<3;i++) u[i] = dx[3+i]/s;
    sinu = sin(s);
    cosi = cos(s);
    mcosi = 1-cosi;
    rd[0][0] = cosi + mcosi*u[0]*u[0];
    rd[0][1] = -sinu*u[2] + mcosi*u[0]*u[1];
    rd[0][2] = sinu*u[1] + mcosi*u[0]*u[2];
    rd[1][0] = sinu*u[2] + mcosi*u[1]*u[0];
    rd[1][1] = cosi + mcosi*u[1]*u[1];
    rd[1][2] = -sinu*u[0] + mcosi*u[1]*u[2];
    rd[2][0] = -sinu*u[1] + mcosi*u[2]*u[0];
    rd[2][1] = sinu*u[0] + mcosi*u[2]*u[1];
    rd[2][2] = cosi + mcosi*u[2]*u[2];

    dt[0] =   dx[0]*(sinu/s + u[0]*u[0]*(1-sinu/s))
      + dx[1]*(u[0]*u[1]*(1-sinu/s)-u[2]*mcosi/s)
      + dx[2]*(u[0]*u[2]*(1-sinu/s)+u[1]*mcosi/s);

    dt[1] =	  dx[0]*(u[0]*u[1]*(1-sinu/s)+u[2]*mcosi/s)
      + dx[1]*(sinu/s + u[1]*u[1]*(1-sinu/s))
      + dx[2]*(u[1]*u[2]*(1-sinu/s)-u[0]*mcosi/s);

    dt[2] =   dx[0]*(u[0]*u[2]*(1-sinu/s)-u[1]*mcosi/s)
      + dx[1]*(u[1]*u[2]*(1-sinu/s)+u[0]*mcosi/s)
      + dx[2]*(sinu/s + u[2]*u[2]*(1-sinu/s));
  }
  else
  {

    for (i=0;i<3;i++)
    {
      for(j=0;j<3;j++) rd[i][j] = 0.0;
      rd[i][i] = 1.0;
      dt[i] = dx[i];
    }
  }

  vpHomogeneousMatrix Delta ;
  Delta.insert(rd) ;
  Delta.insert(dt) ;

  mati = Delta.inverse() * mati ;

  return mati ;
}

#undef DEBUG_LEVEL1

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
