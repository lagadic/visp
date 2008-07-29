/****************************************************************************
 *
 * $Id: vpHomogeneousMatrix.cpp,v 1.13 2008-07-29 14:10:41 fspindle Exp $
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
 * Homogeneous matrix.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


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
    vpERROR_TRACE("Error caught") ;
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


vpHomogeneousMatrix::vpHomogeneousMatrix(const vpTranslationVector &T,
                                         const vpThetaUVector &tu) : vpMatrix()
{
  init() ;
  buildFrom(T,tu) ;
}

vpHomogeneousMatrix::vpHomogeneousMatrix(const vpTranslationVector &T,
                                         const vpRotationMatrix &R) : vpMatrix()
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
/*! 

  \deprecated Construction from translation and rotation.

  This function is deprecated: use vpHomogeneousMatrix(const
  vpTranslationVector&,const vpThetaUVector&) instead.

 */
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpThetaUVector &tu,
                                         const vpTranslationVector &T) : vpMatrix()
{
  init() ;
  buildFrom(T,tu) ;
  vpTRACE("Warning : This function is deprecated: \
      use vpHomogeneousMatrix(vpTranslationVector&,vpThetaUVector&) instead.");
}

/*! 

  \deprecated Construction from translation and rotation.

  This function is deprecated: use vpHomogeneousMatrix(const
  vpTranslationVector&,const vpRotationMatrix&) instead.

 */
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpRotationMatrix &R,
                                         const vpTranslationVector &T) : vpMatrix()
{
  init() ;
  insert(R) ;
  insert(T) ;
  vpTRACE("Warning : This function is deprecated: use \
      vpHomogeneousMatrix(vpTranslationVector&,vpRotationMatrix&).instead");
}

void
vpHomogeneousMatrix::buildFrom(const vpTranslationVector &T,
                               const vpThetaUVector &tu)
{
  insert(tu) ;
  insert(T) ;
}

void
vpHomogeneousMatrix::buildFrom(const vpTranslationVector &T,
                               const vpRotationMatrix &R)
{
  init() ;
  insert(R) ;
  insert(T) ;
}

/*! 

  \deprecated Construction from translation and rotation.

  This function is deprecated: use vpHomogeneousMatrix(const
  vpTranslationVector&,const vpThetaUVector&) instead.

 */
void
    vpHomogeneousMatrix::buildFrom(const vpThetaUVector &tu,
                                   const vpTranslationVector &T)
{
  insert(tu) ;
  insert(T) ;
  vpTRACE("Warning : This function is deprecated: \
      use buildFrom(vpTranslationVector&,vpThetaUVector&) instead.");
}

/*! 

  \deprecated Construction from translation and rotation.

  This function is deprecated: use vpHomogeneousMatrix(const
  vpTranslationVector&,const vpRotationMatrix&) instead.
 */
void
    vpHomogeneousMatrix::buildFrom(const vpRotationMatrix &R,
                                   const vpTranslationVector &T)
{
  init() ;
  insert(R) ;
  insert(T) ;
  vpTRACE("Warning : This function is deprecated:  \
      use buildFrom(vpTranslationVector&,vpRotationMatrix&) instead.");
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
  \brief affectation of two homogeneous matrix

  \param m : *this = m
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

  \param M : [R T]^-1
*/
void
vpHomogeneousMatrix::inverse(vpHomogeneousMatrix &M) const
{
  M = inverse() ;
}


/*!
  Write an homogeneous matrix in an output file stream. 

  \param f : Output file stream. The homogeneous matrix is saved as a
  4 by 4 matrix.

  The code below shows how to save an homogenous matrix in a file.

  \code
  // Contruct an homogeneous matrix
  vpTranslationVector t(1,2,3);
  vpRxyzVector r(M_PI, 0, -M_PI/4.);
  vpRotationMatrix R(r);
  vpHomogeneousMatrix M(t, R);
  
  // Save the content of the matrix in "homogeneous.dat"
  std::ofstream f("homogeneous.dat");  
  M.save(f);
  \endcode

  \sa load()
*/
void
vpHomogeneousMatrix::save(std::ofstream &f) const
{
  if (f != NULL)
  {
    f << *this ;
  }
  else
  {
    vpERROR_TRACE("\t\t file not open " );
    throw(vpException(vpException::ioError, "\t\t file not open")) ;
  }
}


/*!

  Read an homogeneous matrix from an input file stream. The
  homogeneous matrix is considered as a 4 by 4 matrix.

  \param f : Input file stream. 

  The code below shows how to get an homogenous matrix from a file.

  \code
  vpHomogeneousMatrix M;

  std::ifstream f("homogeneous.dat");
  M.load(f);
  \endcode

  \sa save()
*/
void
vpHomogeneousMatrix::load(std::ifstream &f)
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
    vpERROR_TRACE("\t\t file not open " );
    throw(vpException(vpException::ioError, "\t\t file not open")) ;
  }
}

/*!

  \deprecated Read an homogeneous matrix from an input file stream. The
  homogeneous matrix is considered here as a 3 by 4 matrix.

  This methos is deprecated and only proposed for compatibilty issue.

  \param f : Input file stream. 

*/
void
vpHomogeneousMatrix::loadMatrix34(std::ifstream &f)
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
    vpERROR_TRACE("\t\t file not open " );
    throw(vpException(vpException::ioError, "\t\t file not open")) ;
  }
}



//! Print the matrix as a vector [T thetaU]
void
vpHomogeneousMatrix::print()
{
  vpPoseVector r(*this) ;
  std::cout << r.t() ;
}
//! Basic initialisation (identity)
void
vpHomogeneousMatrix::setIdentity()
{
  init() ;
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
