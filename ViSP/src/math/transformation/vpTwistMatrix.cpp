/****************************************************************************
 *
 * $Id: vpTwistMatrix.cpp,v 1.5 2006-06-23 14:45:06 brenier Exp $
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
  \class vpTwistMatrix

  \brief the vpTwistMatrix is derived from vpMatrix.  Class that consider the
    particular case of twist transformation matrix

  \author  Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes

  An twist transformation matrix is 6x6 matrix defines as
  \f[
  ^a{\bf V}_b = \left(\begin{array}{cc}
  ^a{\bf R}_b & [^a{\bf T}_b]_\times ^a{\bf R}_b\\
  {\bf 0}_{1\times 3} & ^a{\bf R}_b
  \end{array}
  \right)
  \f]
  that expressed a velocity in frame <em>a</em> knowing velocity in <em>b</em>

  \f$ ^a{\bf R}_b \f$ is a rotation matrix and
  \f$ ^a{\bf T}_b \f$ is a translation vector.
*/

/*!
  \brief  affectation of two homogeneous matrix

  \param vpTwistMatrix &m : *this = m
*/
vpTwistMatrix &
vpTwistMatrix::operator=(const vpTwistMatrix &m)
{
  init() ;

  for (int i=0; i<6; i++)
  {
    for (int j=0; j<6; j++)
    {
      rowPtrs[i][j] = m.rowPtrs[i][j];
    }
  }

  return *this;
}


/*!
  \brief initialiaze a 6x6 matrix as identity
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
  \brief initialize an homogeneous matrix as Identity
*/
vpTwistMatrix::vpTwistMatrix() : vpMatrix()
{
  init() ;
}

/*!
  \brief initialize an homogeneous matrix from another twist matrix
*/
vpTwistMatrix::vpTwistMatrix(const vpTwistMatrix &M) : vpMatrix()
{
  init() ;
  *this = M ;
}

/*!
  \brief initialize an homogeneous matrix from another twist matrix
*/
vpTwistMatrix::vpTwistMatrix(const vpHomogeneousMatrix &M) : vpMatrix()
{
  init() ;
  buildFrom(M);
}
//! Construction from Translation and rotation (ThetaU parameterization)
vpTwistMatrix::vpTwistMatrix(const vpTranslationVector &T,
			     const vpThetaUVector &R) : vpMatrix()
{
  init() ;
  buildFrom(T,R) ;
}

//! Construction from Translation and rotation (matrix parameterization)
vpTwistMatrix::vpTwistMatrix(const vpTranslationVector &T,
			     const vpRotationMatrix &R)
{
  init() ;
  buildFrom(T,R) ;
}


vpTwistMatrix::vpTwistMatrix(const double Rx,
			     const double Ry,
			     const double Rz,
			     const double Tx,
			     const double Ty,
			     const double Tz) : vpMatrix()
{
  init() ;

  vpERROR_TRACE("function not implemented") ;
  throw(vpException(vpException::functionNotImplementedError)) ;

}

//! Basic initialisation (identity)
void
vpTwistMatrix::setIdentity()
{
  init() ;
}


/*!
  \brief Skew transformation matrix multiplication
*/
vpTwistMatrix
vpTwistMatrix::operator*(const vpTwistMatrix &mat) const
{
  vpTwistMatrix p ;

  for (int i=0;i<6;i++)
    for (int j=0;j<6;j++)
    {
      double s =0 ;
      for (int k=0;k<6;k++)
	s +=rowPtrs[i][k] * mat.rowPtrs[k][j];
      p[i][j] = s ;
    }
  return p;
}

/*!
  operation c = A * v (A is unchanged, c and b are 6 dimension vectors )

  \exception vpMatrixException::incorrectMatrixSizeError if v is not a 6
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

vpTwistMatrix
vpTwistMatrix::buildFrom(const vpTranslationVector &t,
			 const vpEulerVector &e)
{
    vpRotationMatrix R ;
    R.buildFrom(e) ;
    buildFrom(t,R) ;
    return (*this) ;
}

vpTwistMatrix
vpTwistMatrix::buildFrom(const vpTranslationVector &t,
			 const vpThetaUVector &tu)
{
    vpRotationMatrix R ;
    R.buildFrom(tu) ;
    buildFrom(t,R) ;
    return (*this) ;

}


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

#undef MINI
#undef MINIMUM
#undef DEBUG_LEVEL1

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
