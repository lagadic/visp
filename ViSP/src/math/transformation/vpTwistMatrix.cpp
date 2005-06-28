
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpTwistMatrix.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpTwistMatrix.cpp,v 1.2 2005-06-28 13:25:13 marchand Exp $
 *
 * Description
 * ============
 *     Class that consider the particular case of twist transformation matrix
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

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
    ERROR_TRACE("Error caught") ;
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

vpTwistMatrix::vpTwistMatrix(const double Rx,
			     const double Ry,
			     const double Rz,
			     const double Tx,
			     const double Ty,
			     const double Tz) : vpMatrix()
{
  init() ;

  ERROR_TRACE("function not implemented") ;
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
