/****************************************************************************
 *
 * $Id$
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2013 by INRIA. All rights reserved.
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
 * Provide some simple operation on column vectors.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpColVector.cpp
  \brief  Class that provides a data structure for the column vectors as well
  as a set of operations on these vectors
*/

#include <visp/vpColVector.h>
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>
#include <visp/vpDebug.h>
#include <visp/vpRotationVector.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits
#include <string.h> //EM gcc 4.3
#include <math.h> //EM gcc 4.3

  //! operator addition of two vectors
vpColVector
vpColVector::operator+(const vpColVector &m) const
{
  vpColVector v ;

  try {
    v.resize(rowNum) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

  double *vd = v.data ;   double *d = data ;  double *md = m.data ;

  for (unsigned int i=0;i<rowNum;i++)
      *(vd++) = *(d++) + *(md++);

  return v;
}

//! operator dot product
double
vpColVector::operator*(const vpColVector &m) const
{
  double v = 0 ;

  for (unsigned int i=0;i<rowNum;i++)
      v += (*this)[i] * m[i];
  return v;
}

//! operator dot product
vpMatrix vpColVector::operator*(const vpRowVector &m) const
{

  return (vpMatrix)*this*(vpMatrix)m;
}

  //! operator substraction of two vectors V = A-v
vpColVector vpColVector::operator-(const vpColVector &m) const
{
  vpColVector v ;

  try {
    v.resize(rowNum) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

  for (unsigned int i=0;i<rowNum;i++)
      v[i] = (*this)[i] - m[i];
  return v;
}

vpColVector::vpColVector (vpColVector &m, unsigned int r, unsigned int nrows)
{
  if ( (r+nrows) > m.getRows() )
  {
    vpERROR_TRACE("\n\t\t SubvpMatrix larger than vpMatrix") ;
    throw(vpMatrixException(vpMatrixException::subMatrixError,
			    "\n\t\t SubvpMatrix larger than vpColVector")) ;
  }
  init(m, r, 0, nrows, 1);
}


vpColVector::vpColVector (const vpRotationVector &v){
    resize(v._size);
    memcpy(data, v.r, v._size*sizeof(double)) ;
}

  
 //! operator A = -A
vpColVector vpColVector::operator-()
{
  vpColVector A ;
  try {
    A.resize(rowNum)  ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

  double *vd = A.data ;   double *d = data ;

  for (unsigned int i=0; i<rowNum; i++)
    *(vd++)= - (*d++);

  return A;
}

//! operator multiplication by a scalar V =  A * x
vpColVector vpColVector::operator*(double x) const
{
  vpColVector v;
  try {
    v.resize(rowNum)  ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

  double *vd = v.data ;   double *d = data ;

  for (unsigned int i=0;i<rowNum;i++)
     *(vd++)=  (*d++) * x;
  return v;
}

/*!
  \brief copy from a matrix
  \warning  Handled with care m should be a 1 row matrix
*/
vpColVector &vpColVector::operator=(const vpMatrix &m)
{


  if (m.getCols() !=1)
  {
    vpTRACE(" m should be a 1 cols matrix ") ;
    throw (vpException(vpException::dimensionError,"m should be a 1 cols matrix "));
  }

  try {
    resize(m.getRows());
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

  memcpy(data, m.data, rowNum*sizeof(double)) ;

  /*
    double *md = m.data ;   double *d = data ;
    for (int i=0;i<rowNum;i++)
    *(d++)=  *(md++) ;
    */
  /*
  for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
      rowPtrs[i][j] = m.rowPtrs[i][j];
    }
    }*/
  return *this;
}

 //! Copy operator.   Allow operation such as A = v
vpColVector &vpColVector::operator=(const vpColVector &v)
{
  unsigned int k = v.rowNum ;
  if (rowNum != k){
    try {
      resize(k);
    }
    catch(vpException me)
    {
      vpERROR_TRACE("Error caught") ;
      throw ;
    }
  }
  //

  memcpy(data, v.data, rowNum*sizeof(double)) ;
  /*
    double *vd = m.data ;   double *d = data ;
    for (int i=0;i<rowNum;i++)
    *(d++)=  *(vd++) ;


    for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
    rowPtrs[i][j] = v.rowPtrs[i][j];
    }
    }
*/
  return *this;
}

//! Copy operator.   Allow operation such as A = v
vpColVector & vpColVector::operator<<(const vpColVector &v)
{
  try {
    resize(v.getRows());
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }

  for (unsigned int i=0; i<rowNum; i++) {
    for (unsigned int j=0; j<colNum; j++) {
      rowPtrs[i][j] = v.rowPtrs[i][j];
    }
  }
  return *this;
}

//! Assigment operator.   Allow operation such as A = *v
vpColVector & vpColVector::operator<<( double *x )
{
  for (unsigned int i=0; i<rowNum; i++) {
    for (unsigned int j=0; j<colNum; j++) {
      rowPtrs[i][j] = *x++;
    }
  }
  return *this;
}

//! initialisation each element of the vector is x
vpColVector & vpColVector::operator=(double x)
{
  double *d = data ;

  for (unsigned int i=0;i<rowNum;i++)
    *(d++)=  x ;
  /*
  for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
      rowPtrs[i][j] = x;
    }
    }*/
  return *this;
}

/*
  \brief Transpose the row vector A

  A is defined inside the class

  \return  A^T
*/
vpRowVector vpColVector::t() const
{
  vpRowVector v ;
  try {
    v.resize(rowNum);
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    throw ;
  }
  memcpy(v.data, data, rowNum*sizeof(double)) ;
  /*
    // premiere optimisation
    double *vd = m.data ;   double *d = data ;
    for (int i=0;i<rowNum;i++)
    *(d++)=  *(vd++) ;
    */

  /*
    // solution brute
    for (int i=0;i<rowNum;i++)
      v[i] = (*this)[i];
  */
  return v;
}
/*!
  \relates vpColVector
  \brief  multiplication by a scalar Ci = x*Bi
*/
vpColVector operator*(const double &x,const vpColVector &B)
{
  vpColVector v1 ;
  v1 = B*x ;
  return v1 ;
}

vpColVector::vpColVector (const vpColVector &v) : vpMatrix(v)
{
}

vpColVector::vpColVector (vpMatrix &m, unsigned int j) : vpMatrix(m, 0, j, m.getRows(), 1)
{
}


/*!
  \relates vpColVector
  \brief compute  the dot product of two vectors C = a.b
*/
double
vpColVector::dotProd(const vpColVector &a, const vpColVector &b)
{
  if (a.data==NULL)
  {
    vpERROR_TRACE("vpColVector a non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }
  if (b.data==NULL)
  {
    vpERROR_TRACE("vpColVector b non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }
  double *ad = a.data ;   double *bd = b.data ;

  double c = 0 ;
  for (unsigned int i=0 ; i < a.getRows() ; i++)
    c += *(ad++)* *(bd++) ;
  //  vpMatrix c = (a.t() * b);
  //  return c[0][0];
  return c ;
}

/*!
  \relates vpColVector
  \brief normalise the vector

  \f[
  {\bf x}_i = \frac{{\bf x}_i}{\sqrt{\sum_{i=1}^{n}x^2_i}}
  \f]
*/
// vpColVector &vpColVector::normalize(vpColVector &x) const
// {
//   x = x/sqrt(x.sumSquare());

//   return x;
// }


/*!
  \relates vpColVector
  \brief normalise the vector

  \f[
  {\bf x}_i = \frac{{\bf x}_i}{\sqrt{\sum_{i=1}^{n}x^2_i}}
  \f]
*/
vpColVector &vpColVector::normalize()
{

  double sum = sumSquare() ;

  //if (sum != 0.0)
  if (std::fabs(sum) > std::numeric_limits<double>::epsilon())
    *this /= sqrt(sum) ;

  // If sum = 0, we have a nul vector. So we return just.
  return *this;
}



vpColVector
vpColVector::invSort(const vpColVector &v)
{
 if (v.data==NULL)
  {
    vpERROR_TRACE("vpColVector v non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }
  vpColVector tab ;
  tab = v ;
  unsigned int nb_permutation = 1 ;
  unsigned int i = 0 ;
  while (nb_permutation !=0 )
  {
    nb_permutation = 0 ;
    for (unsigned int j = v.getRows()-1 ; j >= i+1 ; j--)
    {
      if ((tab[j]>tab[j-1]))
      {
	double tmp = tab[j] ;
	tab[j] = tab[j-1] ;
	tab[j-1] = tmp ;
	nb_permutation++ ;
      }
    }
    i++ ;
  }

  return tab ;
}

vpColVector
vpColVector::sort(const vpColVector &v)
{
  if (v.data==NULL) {
    vpERROR_TRACE("vpColVector a non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }
  vpColVector tab ;
  tab = v ;
  unsigned int nb_permutation = 1 ;
  unsigned int i = 0 ;
  while (nb_permutation !=0 )
  {
    nb_permutation = 0 ;
    for (unsigned int j = v.getRows()-1 ; j >= i+1 ; j--)
    {
      if ((tab[j]<tab[j-1]))
      {
	double tmp = tab[j] ;
	tab[j] = tab[j-1] ;
	tab[j-1] = tmp ;
	nb_permutation++ ;
      }
    }
    i++ ;
  }

  return tab ;
}

/*!
  Stack column vector with new element.

  \param b : Element to stack to the existing one.

  \code
  vpColVector A(3);
  double b = 3;
  A.stack(b); // A = [A b]T
  // A is now an 4 dimension column vector
  \endcode

  \sa stack(const vpColVector &, const vpColVector &)
  \sa stack(const vpColVector &, const vpColVector &, vpColVector &)

*/
void vpColVector::stack(const double &b)
{
  this->resize(rowNum+1,false);
  (*this)[rowNum-1] = b;
}

/*!
  Stack column vectors.

  \param B : Vector to stack to the existing one.

  \code
  vpColVector A(3);
  vpColVector B(5);
  A.stack(B); // A = [A B]T
  // A is now an 8 dimension column vector
  \endcode

  \sa stack(const vpColVector &, const double &)
  \sa stack(const vpColVector &, const vpColVector &)
  \sa stack(const vpColVector &, const vpColVector &, vpColVector &)

*/
void vpColVector::stack(const vpColVector &B)
{
  *this = vpColVector::stack(*this, B);
}

/*!
  Stack column vectors.

  \param A : Initial vector.
  \param B : Vector to stack at the end of A.
  \return Stacked vector \f$[A B]^T\f$.

  \code
  vpColVector A(3);
  vpColVector B(5);
  vpColVector C;
  C = vpColVector::stack(A, B); // C = [A B]T
  // C is now an 8 dimension column vector
  \endcode

  \sa stack(const vpColVector &)
  \sa stack(const vpColVector &, const vpColVector &, vpColVector &)
*/
vpColVector vpColVector::stack(const vpColVector &A, const vpColVector &B)
{
  vpColVector C;
  vpColVector::stack(A, B, C);
  return C;
}

/*!
  Stack column vectors.

  \param A : Initial vector.
  \param B : Vector to stack at the end of A.
  \param C : Resulting stacked vector \f$C = [A B]^T\f$.

  \code
  vpColVector A(3);
  vpColVector B(5);
  vpColVector C;
  vpColVector::stack(A, B, C); // C = [A B]T
  // C is now an 8 dimension column vector
  \endcode

  \sa stack(const vpColVector &)
  \sa stack(const vpColVector &, const vpColVector &)
*/
void vpColVector::stack(const vpColVector &A, const vpColVector &B, vpColVector &C)
{
  unsigned int nrA = A.getRows();
  unsigned int nrB = B.getRows();

  if (nrA == 0 && nrB == 0) {
    C.resize(0);
    return;
  }

  if (nrB == 0) {
    C = A;
    return;
  }

  if (nrA == 0) {
    C = B;
    return;
  }

  // General case
  C.resize(nrA + nrB);

  for (unsigned int i=0; i<nrA; i++)
    C[i] = A[i];

  for (unsigned int i=0; i<nrB; i++)
    C[nrA+i] = B[i];
}

/*!
  \brief Compute the mean value of all the element of the vector
*/
double vpColVector::mean(const vpColVector &v)
{
 if (v.data==NULL)
  {
    vpERROR_TRACE("vpColVector v non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }
 double mean = 0 ;
  double *vd = v.data ;
  for (unsigned int i=0 ; i < v.getRows() ; i++)
    mean += *(vd++) ;

  /*
  for (int i=0 ; i < v.getRows() ; i++)
  {
    mean += v[i] ;
  }
   mean /= v.rowNum ;
   return mean;
  */
  return mean/v.getRows();
}

/*!
  Compute the median value of all the element of the vector
*/
double
vpColVector::median(const vpColVector &v)
{
  if (v.data==NULL)
  {
    vpERROR_TRACE("vpColVector v non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }

  unsigned int i,j;
  unsigned int inf, sup;
  unsigned int n = v.getRows() ;
  vpColVector infsup(n) ;

  for (i=0;i<v.getRows();i++)
  {
    // We compute the number of element gretear (sup) than the current
    // value and the number of element smaller (inf) than the current
    // value
    inf = sup = 0;
    for (j=0;j<v.getRows();j++)
    {
      if (i != j)
      {
        if (v[i] <= v[j]) inf++;
        if (v[i] >= v[j]) sup++;
      }
    }
    // We compute then difference between inf and sup
    // the median should be for |inf-sup| = 0 (1 if an even number of element)
    // which means that there are the same number of element in the array
    // that are greater and smaller that this value.
    infsup[i] = fabs((double)(inf-sup)); //EM gcc 4.3
  }

  // seek for the smaller value of |inf-sup| (should be 0 or 1)
  unsigned int imin=0 ; // index of the median in the array
  // min cannot be greater than the number of element
  double  min = v.getRows();
  for (i=0;i<v.getRows();i++)
    if (infsup[i] < min)
    {
      min = infsup[i];
      imin = i ;
    }

  // return the median
  return(v[imin]);

}

/*!
  Compute the skew symmetric matrix \f$[{\bf v}]_\times\f$ of vector v.

  \f[ \mbox{if} \quad  {\bf V} =  \left( \begin{array}{c} x \\ y \\  z
  \end{array}\right), \quad \mbox{then} \qquad
  [{\bf v}]_\times = \left( \begin{array}{ccc}
  0 & -z & y \\
  z & 0 & -x \\
  -y & x & 0
  \end{array}\right)
  \f]

  \param v : Input vector used to compute the skew symmetric matrix.
*/
vpMatrix
vpColVector::skew(const vpColVector &v)
{

  vpMatrix M ;
  if (v.getRows() != 3)
  {
    vpERROR_TRACE("Cannot compute skew kinematic matrix,"
		"v has not 3 rows") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t Cannot compute skew kinematic matrix"
			    "v has not 3 rows")) ;
  }


  M.resize(3,3) ;
  M[0][0] = 0 ;     M[0][1] = -v[2] ;  M[0][2] = v[1] ;
  M[1][0] = v[2] ;  M[1][1] = 0 ;      M[1][2] = -v[0] ;
  M[2][0] = -v[1] ; M[2][1] = v[0] ;   M[2][2] = 0 ;


  return M ;
}

/*!
  \brief Compute the cross product of two vectors C = a x b

  \param a : vpColVector
  \param b : vpColVector
*/
vpColVector vpColVector::crossProd(const vpColVector &a, const vpColVector &b)
{

  if (a.getRows() != 3)
  {
    vpERROR_TRACE("Cannot compute cross product,"
		"a has not 3 rows") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t Cannot compute  cross product"
			    "a has not 3 rows")) ;
  }
  if (b.getRows() != 3)
  {

    vpERROR_TRACE("Cannot compute cross product,"
		"b has not 3 rows") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t Cannot compute  cross product"
			    "b has not 3 rows")) ;;
  }

  return vpColVector::skew(a) * b;
}


/*!
  \brief reshape the colvector in a matrix
  \param nrows : number of rows of the matrix
  \param ncols : number of columns of the matrix
  \return a vpMatrix
*/
vpMatrix vpColVector::reshape(const unsigned int &nrows,const unsigned int &ncols){
  vpMatrix m(nrows,ncols);
  reshape(m,nrows,ncols);
  return m;
}

/*!
  \brief reshape the colvector in a matrix
  \param m : the reshaped Matrix
  \param nrows : number of rows of the matrix
  \param ncols : number of columns of the matrix
*/
void vpColVector::reshape(vpMatrix & m,const unsigned int &nrows,const unsigned int &ncols){
  if(dsize!=nrows*ncols)
  {
    vpERROR_TRACE("\n\t\t vpColVector mismatch size for reshape vpSubColVector in a vpMatrix") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpColVector mismatch size for reshape vpSubColVector in a vpMatrix")) ;
  }
  try 
  {
    if ((m.getRows() != nrows) || (m.getCols() != ncols)) m.resize(nrows,ncols);
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    std::cout << me << std::endl ;
    throw ;
  }

  for(unsigned int j =0; j< ncols; j++)
     for(unsigned int i =0; i< nrows; i++)
	  m[i][j]=data[j*ncols+i];
}

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
