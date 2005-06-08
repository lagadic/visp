
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpRowVector.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      CRowVector.cpp, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpRowVector.cpp,v 1.2 2005-06-08 08:57:51 fspindle Exp $
 *
 * Description
 * ============
 *    provide some simple operation on row vectors
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/


/*!
  \file vpRowVector.cpp
  \brief Definition of vpRowVector class member
*/

#include <string.h>
#include <stdlib.h>
#include <visp/vpMatrix.h>
#include <visp/vpRowVector.h>
#include <visp/vpColVector.h>

//! Copy operator.   Allow operation such as A = v
vpRowVector & vpRowVector::operator=(const vpRowVector &v)
{
  if (colNum==0)
    resize(v.getCols());

  for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
      rowPtrs[i][j] = v.rowPtrs[i][j];
    }
  }
  return *this;
}

/*!
  \brief copy from a matrix
  \warning  Handled with care m should be a 1 column matrix
*/
vpRowVector & vpRowVector::operator=(const vpMatrix &m)
{
  if (m.getCols() != colNum)
    resize(m.getCols());

  memcpy(data, m.data, colNum*sizeof(double)) ;
  /*
  for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
      rowPtrs[i][j] = m.rowPtrs[i][j];
    }
    }*/
  return *this;
}

//! initialisation each element of the vector is x
vpRowVector & vpRowVector::operator=(double x)
{
  for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
      rowPtrs[i][j] = x;
    }
  }
  return *this;
}
/*
  \brief Transpose the column vector A

  A is defined inside the class

  \return  A^T
*/
vpColVector vpRowVector::t() const
{
  vpColVector tmp(colNum);
  memcpy(tmp.data, data, rowNum*sizeof(double)) ;
  /*
  for (int i=0;i<colNum;i++)
      tmp[i] = (*this)[i];
  */
  return tmp;
}

//! copy constructor
vpRowVector::vpRowVector (const vpRowVector &v) : vpMatrix(v)
{
}

//! Constructor  (Take line i of matrix m)
vpRowVector::vpRowVector (vpMatrix &m, int i) : vpMatrix(m, i, 0, 1, m.getCols())
{
}

/*!
  \relates vpRowVector
  \brief normalise the vector

  \f[
  {\bf x}_i = \frac{{\bf x}_i}{\sqrt{\sum_{i=1}^{n}x^2_i}}
  \f]
*/
vpRowVector &vpRowVector::normalize(vpRowVector &x) const
{
  x = x/sqrt(x.sumSquare());

  return x;
}


/*!
  \relates vpRowVector
  \brief normalise the vector

  \f[
  {\bf x}_i = \frac{{\bf x}_i}{\sqrt{\sum_{i=1}^{n}x^2_i}}
  \f]
*/
vpRowVector &vpRowVector::normalize()
{

  double sum = sumSquare() ;
  *this /= sum ;

  return *this;
}

