/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
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
 * Homogeneous matrix.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpHomogeneousMatrix.cpp
  \brief Defines vpHomogeneousMatrix class. Class that consider
  the particular case of an homogeneous matrix.
*/

#include <visp/vpDebug.h>
#include <visp/vpMatrix.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpQuaternionVector.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>


/*!
  Initialize a 4x4 momogeneous matrix as identity.
*/
void
vpHomogeneousMatrix::init()
{
  unsigned int i,j ;

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

vpHomogeneousMatrix::vpHomogeneousMatrix(const vpTranslationVector &t, 
					 const vpQuaternionVector &q) 
{
  init();
  buildFrom(t,q);
}

/*!
  Initialize an homogeneous matrix as identity.
*/
vpHomogeneousMatrix::vpHomogeneousMatrix() : vpMatrix()
{
  init() ;
}


/*!
  Initialize an homogeneous matrix from another homogeneous matrix.
*/
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpHomogeneousMatrix &M) : vpMatrix()
{
  init() ;
  *this = M ;
}

vpHomogeneousMatrix::vpHomogeneousMatrix(const vpTranslationVector &t,
                                         const vpThetaUVector &tu) : vpMatrix()
{
  init() ;
  buildFrom(t,tu) ;
}

vpHomogeneousMatrix::vpHomogeneousMatrix(const vpTranslationVector &t,
                                         const vpRotationMatrix &R) : vpMatrix()
{
  init() ;
  insert(R) ;
  insert(t) ;
}

vpHomogeneousMatrix::vpHomogeneousMatrix(const vpPoseVector &p) : vpMatrix()
{

  init() ;
  buildFrom(p[0],p[1],p[2],p[3],p[4],p[5]) ;
}

vpHomogeneousMatrix::vpHomogeneousMatrix(const double tx,
					 const double ty,
					 const double tz,
					 const double tux,
					 const double tuy,
					 const double tuz) : vpMatrix()
{
  init() ;
  buildFrom(tx, ty, tz,tux, tuy, tuz) ;
}

void
vpHomogeneousMatrix::buildFrom(const vpTranslationVector &t,
                               const vpThetaUVector &tu)
{
  insert(tu) ;
  insert(t) ;
}

void
vpHomogeneousMatrix::buildFrom(const vpTranslationVector &t,
                               const vpRotationMatrix &R)
{
  init() ;
  insert(R) ;
  insert(t) ;
}


void
vpHomogeneousMatrix::buildFrom(const vpPoseVector &p)
{

  vpTranslationVector t(p[0],p[1],p[2]) ;
  vpThetaUVector tu(p[3],p[4],p[5]) ;

  insert(tu) ;
  insert(t) ;
}

void vpHomogeneousMatrix::buildFrom(const vpTranslationVector &t, 
				    const vpQuaternionVector &q) 
{
  insert(t);
  insert(q);
}

void
vpHomogeneousMatrix::buildFrom(const double tx,
			       const double ty,
			       const double tz,
			       const double tux,
			       const double tuy,
			       const double tuz)
{
  vpRotationMatrix R(tux, tuy, tuz) ;
  vpTranslationVector t(tx, ty, tz) ;

  insert(R) ;
  insert(t) ;
}

/*!
  Affectation of two homogeneous matrices.

  \param M : *this = M
*/
vpHomogeneousMatrix &
vpHomogeneousMatrix::operator=(const vpHomogeneousMatrix &M)
{

  if (rowPtrs != M.rowPtrs) init() ;

  for (int i=0; i<4; i++)
  {
    for (int j=0; j<4; j++)
    {
      rowPtrs[i][j] = M.rowPtrs[i][j];
    }
  }
  return *this;
}

/*!
  Allow homogeneous matrix multiplication.

  \code
#include <visp/vpHomogeneousMatrix.h>

int main()
{
  vpHomogeneousMatrix aMb, bMc;
  // Initialize aMb and bMc...

  // Compute aMc * bMc
  vpHomogeneousMatrix aMc = aMb * bMc;  
}
  \endcode

*/
vpHomogeneousMatrix
vpHomogeneousMatrix::operator*(const vpHomogeneousMatrix &M) const
{
  vpHomogeneousMatrix p,p1 ;

  vpRotationMatrix R1, R2, R ;
  vpTranslationVector T1, T2 , T;


  extract(T1) ;
  M.extract(T2) ;

  extract (R1) ;
  M.extract (R2) ;

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

  for (unsigned int j=0;j<4;j++) {
    for (unsigned int i=0;i<4;i++) {
      p[i]+=rowPtrs[i][j] * v[j];
    }
  }

  return p;
}


/*********************************************************************/

/*!
  Test if the 3x3 rotational part of the homogeneous matrix is really
  a rotation matrix.
*/
bool
vpHomogeneousMatrix::isAnHomogeneousMatrix() const
{
  vpRotationMatrix R ;
  extract(R) ;

  return  R.isARotationMatrix() ;
}

/*!
  Extract the rotational matrix from the homogeneous matrix.
  \param R : rotational component as a rotation matrix.
*/
void
vpHomogeneousMatrix::extract(vpRotationMatrix &R) const
{
  unsigned int i,j ;

  for (i=0 ; i < 3 ; i++)
    for (j=0 ; j < 3; j++)
      R[i][j] = (*this)[i][j] ;
}

/*!
  Extract the translation vector from the homogeneous matrix. 
*/
void
vpHomogeneousMatrix::extract(vpTranslationVector &t) const
{
  t[0] = (*this)[0][3] ;
  t[1] = (*this)[1][3] ;
  t[2] = (*this)[2][3] ;
}
/*!
  Extract the rotation as a Theta U vector.
*/
void
vpHomogeneousMatrix::extract(vpThetaUVector &tu) const
{
  
  vpRotationMatrix R;
  (*this).extract(R);
  tu.buildFrom(R);
}

/*!
  Extract the rotation as a quaternion.
*/
void
vpHomogeneousMatrix::extract(vpQuaternionVector &q) const
{
  vpRotationMatrix R;
  (*this).extract(R);
  q.buildFrom(R);
}

/*!
  Insert the rotational component of the homogeneous matrix.
*/
void
vpHomogeneousMatrix::insert(const vpRotationMatrix &R)
{
  unsigned int i,j ;

  for (i=0 ; i < 3 ; i++)
    for (j=0 ; j < 3; j++)
      (*this)[i][j] = R[i][j] ;
}

/*!  

  Insert the rotational component of the homogeneous matrix from a
  theta u rotation vector.

*/
void
vpHomogeneousMatrix::insert(const vpThetaUVector &tu)
{
  vpRotationMatrix R(tu) ;
  insert(R) ;
}

/*!
  Insert the translational component in a homogeneous matrix.
*/
void
vpHomogeneousMatrix::insert(const vpTranslationVector &T)
{
  (*this)[0][3] = T[0] ;
  (*this)[1][3] = T[1] ;
  (*this)[2][3] = T[2] ;
}

/*!  

  Insert the rotational component of the homogeneous matrix from a
  quaternion rotation vector.

*/
void
vpHomogeneousMatrix::insert(const vpQuaternionVector &q){
  insert(vpRotationMatrix(q));
}

/*!
  Invert the homogeneous matrix

  \return \f$\left[\begin{array}{cc}
  {\bf R} & {\bf t} \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right]^{-1} = \left[\begin{array}{cc}
  {\bf R}^T & -{\bf R}^T {\bf t} \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right]\f$
  
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
  Set transformation to identity.
*/
void vpHomogeneousMatrix::eye()
{
  (*this)[0][0] = 1 ;
  (*this)[1][1] = 1 ;
  (*this)[2][2] = 1 ;

  (*this)[0][1] = (*this)[0][2] = 0 ;
  (*this)[1][0] = (*this)[1][2] = 0 ;
  (*this)[2][0] = (*this)[2][1] = 0 ;

  (*this)[0][3] = 0 ;
  (*this)[1][3] = 0 ;
  (*this)[2][3] = 0 ;
}

/*!
  Invert the homogeneous matrix.

  \param M : The inverted homogenous matrix: \f$\left[\begin{array}{cc}
  {\bf R} & {\bf t} \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right]^{-1} = \left[\begin{array}{cc}
  {\bf R}^T & -{\bf R}^T {\bf t} \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right]\f$

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
  if (! f.fail())
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
  if (! f.fail())
  {
    for (unsigned int i=0 ; i < 4 ; i++)
      for (unsigned int j=0 ; j < 4 ; j++)
      {
        f >> (*this)[i][j] ;
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
