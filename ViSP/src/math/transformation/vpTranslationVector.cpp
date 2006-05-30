/****************************************************************************
 *
 * $Id: vpTranslationVector.cpp,v 1.3 2006-05-30 08:40:44 fspindle Exp $
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
 * Translation vector.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


#include <visp/vpTranslationVector.h>

/*!
  \file vpTranslationVector.h
  \brief class that consider the case of a translation vector
*/

/*!
  \class vpTranslationVector
  \brief class that consider the case of a translation vector
*/
//! initialize a size 3 vector
void vpTranslationVector::init()
{
    resize(3) ;
}

//! constructor from double in meter
vpTranslationVector::vpTranslationVector(const double tx,
					 const double ty,
					 const double tz)
{
    init() ;
    (*this)[0] = tx ;
    (*this)[1] = ty ;
    (*this)[2] = tz ;
}

//! operator addition of two translation vectors
vpTranslationVector
vpTranslationVector::operator+(const vpTranslationVector &_v) const
{
    vpTranslationVector v ;

    for (int i=0;i<3;i++)  v[i] = (*this)[i]+_v[i] ;

    return v;
}


//! negate t = -a  (t is unchanged)
vpTranslationVector vpTranslationVector::operator-() const //negate
{
    vpTranslationVector t ;
    for (int i=0;i<dsize;i++)
    {
	*(t.data + i) = -*(data + i) ;
    }

    return t;
}

//! initialisation each element of the vector is x
vpTranslationVector & vpTranslationVector::operator=(double x)
{

  double *d = data ;

  for (int i=0;i<3;i++)
    *(d++)=  x ;

  return *this;
}

/*!
  \relates vpMatrix

  \brief Compute the skew symmetric matrix of vector v (matrice de pre-produit
  vectoriel)

  \f[ \mbox{if} \quad  {\bf V} =  \left( \begin{array}{c} x \\ y \\  z
  \end{array}\right), \quad \mbox{then} \qquad
  skew(\bf V) = \left( \begin{array}{ccc}
  0 & -z & y \\
  z & 0 & -x \\
  -y & x & 0
  \end{array}\right)
  \f]

  \param
*/
void
vpTranslationVector::skew(const vpTranslationVector &v,vpMatrix &M)
{
    M.resize(3,3) ;
    M[0][0] = 0 ;     M[0][1] = -v[2] ;  M[0][2] = v[1] ;
    M[1][0] = v[2] ;  M[1][1] = 0 ;      M[1][2] = -v[0] ;
    M[2][0] = -v[1] ; M[2][1] = v[0] ;   M[2][2] = 0 ;
}

/*!
  \relates vpTranslationVector

  \brief Compute the skew symmetric matrix of vector v (matrice de pre-produit
  vectoriel)

  \sa skew(const vpColVector &v,vpMatrix &M) for more details

  \return the skew symmetric matrix
*/
vpMatrix
vpTranslationVector::skew(const vpTranslationVector &v)
{
    vpMatrix M(3, 3);
    skew(v,M);
    return M;
}

/*!
  \relates vpTranslationVector

  \brief Compute the skew symmetric matrix of vector v (matrice de pre-produit
  vectoriel)

  \sa skew(const vpColVector &v,vpMatrix &M) for more details

  \return the skew symmetric matrix
*/
vpMatrix
vpTranslationVector::skew() const
{
    vpMatrix M(3, 3);
    skew(*this,M);
    return M;
}


/*!
  \relates vpTranslationVector
  \brief Compute the cross product of two vectors C = a x b
*/
vpTranslationVector
vpTranslationVector::cross(const vpTranslationVector &a,
			   const vpTranslationVector &b)
{
  vpMatrix skew_a = vpTranslationVector::skew(a) ;
  return (vpTranslationVector)(skew_a * b);
}

/*
 * Local variables:
 * c-basic-offset: 4
 * End:
 */
