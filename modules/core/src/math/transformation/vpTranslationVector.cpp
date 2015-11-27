/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See http://visp.inria.fr for more information.
 *
 * This software was developed at:
 * Inria Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 *
 * If you have questions regarding the use of this file, please contact
 * Inria at visp@inria.fr
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Description:
 * Translation vector.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/


#include <stdio.h>
#include <string.h>

#include <visp3/core/vpTranslationVector.h>

/*!
  \file vpTranslationVector.cpp
  \brief Class that consider the case of a translation vector.
*/

/*!
  Construct a translation vector \f$ \bf t \f$ from 3 doubles.

  \param tx,ty,tz : Translation respectively along x, y and z axis. Values are in meters.

*/
vpTranslationVector::vpTranslationVector(const double tx, const double ty, const double tz)
  : vpArray2D<double>(3, 1)
{
  (*this)[0] = tx ;
  (*this)[1] = ty ;
  (*this)[2] = tz ;
}

/*!
  Copy constructor.

  \param t : Translation vector to copy.

  \code
  vpTranslationVector t1(1,2,3); // Create and initialize a translation vector

  vpTranslationVector t2(t1);    // t2 is now a copy of t1
  \endcode

*/
vpTranslationVector::vpTranslationVector (const vpTranslationVector &t)
  : vpArray2D<double>(t)
{
}

/*!
  Initialize a translation vector from 3 doubles.

  \param tx,ty,tz : Translation respectively along x, y and z axis.

*/
void
vpTranslationVector::set(const double tx, const double ty, const double tz)
{
  (*this)[0] = tx ;
  (*this)[1] = ty ;
  (*this)[2] = tz ;
}

/*!
  Operator that allows to add two translation vectors.

  \param t : Translation  vector to add.

  \return The sum of the current translation vector (*this) and the one to add.
  \code
  vpTranslationVector t1(1,2,3); 
  vpTranslationVector t2(4,5,6); 
  vpTranslationVector t3; 

  t3 = t2 + t1; 
  // t1 and t2 leave unchanged
  // t3 is now equal to : 5, 7, 9
  \endcode

*/
vpTranslationVector
vpTranslationVector::operator+(const vpTranslationVector &t) const
{
  vpTranslationVector s;

  for (unsigned int i=0;i<3;i++)  s[i] = (*this)[i]+t[i] ;

  return s;
}

/*!
  Operator that allows to substract two translation vectors.

  \param t : Translation  vector to substract.

  \return The substraction of the current translation vector (*this) and the one to substract.
  \code
  vpTranslationVector t1(1,2,3); 
  vpTranslationVector t2(4,5,6); 
  vpTranslationVector t3; 

  t3 = t2 - t1; 
  // t1 and t2 leave unchanged
  // t3 is now equal to : 3, 3, 3
  \endcode

*/
vpTranslationVector
vpTranslationVector::operator-(const vpTranslationVector &t) const
{
  vpTranslationVector sub ;

  for (unsigned int i=0;i<3;i++)  sub[i] = (*this)[i]-t[i] ;

  return sub;
}


/*!
  Operator that allows to negate a translation vector.

  \return The negate translation. The current translation vector
  (*this) is unchanged.

  \code
  vpTranslationVector t1(1,2,3); 
  vpTranslationVector t2; 
  t2 = -t1;
  // t1 is unchanged 
  // t2 is now equal to : [-1, -2, -3]
  \endcode
*/
vpTranslationVector vpTranslationVector::operator-() const //negate
{
  vpTranslationVector t;
  for (unsigned int i=0;i<dsize;i++)
  {
    *(t.data + i) = -*(data + i) ;
  }

  return t;
}

/*!
  Operator that allows to multiply each element of a translation vector by a scalar.

  \param x : The scalar.

  \return The translation vector multiplied by the scalar. The current
  translation vector (*this) is unchanged.

  \code
  vpTranslationVector t1(1,2,3); 
  t2 = t1 * 3;
  // t1 is unchanged 
  // t2 is now equal to : [3, 6, 9]
  \endcode
*/
vpTranslationVector vpTranslationVector::operator*(const double x) const 
{
  vpTranslationVector t;
  for (unsigned int i=0;i<dsize;i++) {
    *(t.data + i) = (*(data + i)) * x ;
  }

  return t;
}

/*!

  Multiply a 3-by-1 dimension translation vector by a 1-by-n row vector.

  \param v : Row vector.

  \return The resulting matrix that is 3-by-n dimension.

*/
vpMatrix vpTranslationVector::operator*(const vpRowVector &v) const
{
  vpMatrix M(rowNum, v.getCols());
  for (unsigned int i=0; i<rowNum; i++) {
    for (unsigned int j=0; j<v.getCols(); j++) {
      M[i][j] = (*this)[i] * v[j];
    }
  }
  return M;
}

/*!
  Operator that allows to multiply each element of a translation vector by a scalar.

  \param x : The scalar.

  \return The translation vector multiplied by the scalar.
*/
vpTranslationVector &vpTranslationVector::operator*=(double x)
{
  for (unsigned int i=0;i<rowNum;i++)
    (*this)[i] *= x;
  return (*this);
}
/*!
  Operator that allows to divide each element of a translation vector by a scalar.

  \param x : The scalar.

  \return The column vector divided by the scalar.
*/
vpTranslationVector &vpTranslationVector::operator/=(double x)
{
  for (unsigned int i=0;i<rowNum;i++)
    (*this)[i] /= x;
  return (*this);
}

/*!
  Operator that allows to divide each element of a translation vector by a scalar.

  \param x : The scalar.

  \return The translation vector divided by the scalar. The current
  translation vector (*this) is unchanged.

  \code
  vpTranslationVector t1(8,4,2);
  t2 = t1 / 2;
  // t1 is unchanged
  // t2 is now equal to : [4, 2, 1]
  \endcode
*/
vpTranslationVector vpTranslationVector::operator/(const double x) const
{
  vpTranslationVector t;
  for (unsigned int i=0;i<dsize;i++) {
    *(t.data + i) = (*(data + i)) / x ;
  }

  return t;
}

/*!
  Copy operator.  
  \param t : Translation vector to copy
  \return A copy of t.

  \code
  vpTranslationVector t1(1,2,3); 
  vpTranslationVector t2; 
  t2 = t1;
  // t1 is unchanged 
  // t2 is now equal to t1 : 1, 2, 3 
  \endcode
*/
vpTranslationVector &vpTranslationVector::operator=(const vpTranslationVector &t)
{
  unsigned int k = t.rowNum ;
  if (rowNum != k){
    try {
      resize(k, 1);
    }
    catch(...)
    {
      throw ;
    }
  }

  memcpy(data, t.data, rowNum*sizeof(double)) ;

  return *this;
}

/*! 
  Initialize each element of a translation vector to the same value x.

  \param x : Value to set for each element of the translation vector.

  \code
  vpTranslationVector t;
  t = 3;
  // Here t is set to 3,3,3
  \endcode
*/
vpTranslationVector & vpTranslationVector::operator=(double x)
{
  double *d = data ;

  for (int i=0;i<3;i++)
    *(d++)=  x ;

  return *this;
}

/*!
  Compute the skew symmetric matrix \f$M\f$ of translation vector \f$t\f$
  (matrice de pre-produit vectoriel).

  \f[ \mbox{if} \quad  {\bf t} =  \left( \begin{array}{c} t_x \\ t_y \\ t_z
  \end{array}\right), \quad \mbox{then} \qquad
  M = \left( \begin{array}{ccc}
  0 & -t_z & t_y \\
  t_z & 0 & -t_x \\
  -t_y & t_x & 0
  \end{array}\right)
  \f]

  \param t : Translation vector in input used to compute the skew symmetric
  matrix M.

  \param M : Skew symmetric matrix of translation vector \f$t\f$.
*/
void
vpTranslationVector::skew(const vpTranslationVector &t, vpMatrix &M)
{
  M.resize(3,3) ;
  M[0][0] = 0 ;     M[0][1] = -t[2] ;  M[0][2] = t[1] ;
  M[1][0] = t[2] ;  M[1][1] = 0 ;      M[1][2] = -t[0] ;
  M[2][0] = -t[1] ; M[2][1] = t[0] ;   M[2][2] = 0 ;
}

/*!

  Compute the skew symmetric matrix \f$M\f$ of translation vector
  \f$t\f$.

  \f[ \mbox{if} \quad  {\bf t} =  \left( \begin{array}{c} t_x \\ t_y \\ t_z
  \end{array}\right), \quad \mbox{then} \qquad
  M = \left( \begin{array}{ccc}
  0 & -t_z & t_y \\
  t_z & 0 & -t_x \\
  -t_y & t_x & 0
  \end{array}\right)
  \f]

  \param t : Translation vector in input.

  \return Skew symmetric matrix \f$M\f$ of translation vector \f$t\f$.

*/
vpMatrix
vpTranslationVector::skew(const vpTranslationVector &t)
{
  vpMatrix M(3, 3);
  skew(t,M);
  return M;
}

/*!

  Compute the skew symmetric matrix \f$M\f$ of the translation vector (matrice
  de pre-produit vectoriel), where

  \f[ M = \left( \begin{array}{ccc}
  0 & -t_z & t_y \\
  t_z & 0 & -t_x \\
  -t_y & t_x & 0
  \end{array}\right)
  \f] 

  and where \f$(t_x,t_y,t_z)\f$ are the coordinates of the translation
  vector.

  \return Skew symmetric matrix \f$M\f$ of the translation vector.

*/
vpMatrix
vpTranslationVector::skew() const
{
  vpMatrix M(3, 3);
  skew(*this,M);
  return M;
}


/*!

  Return the cross product of two translation vectors \f$a \times b\f$.

  \param a,b : Translation vectors in input. 

  \return The cross product of two translation vectors \f$a \times
  b\f$.
*/
vpTranslationVector
vpTranslationVector::cross(const vpTranslationVector &a,
                           const vpTranslationVector &b)
{
  vpMatrix skew_a = vpTranslationVector::skew(a) ;
  return (vpTranslationVector)(skew_a * b);
}

/*!
  Transpose the translation vector. The resulting vector becomes a row vector.
*/
vpRowVector vpTranslationVector::t() const
{
  vpRowVector v(rowNum);
  memcpy(v.data, data, rowNum*sizeof(double)) ;
  return v;
}
