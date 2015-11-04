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
 * Operation on row vectors.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/


/*!
  \file vpRowVector.cpp
  \brief Definition of vpRowVector class member
*/


#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpRowVector.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <string.h>
#include <stdlib.h>

//! Copy operator.   Allow operation such as A = v
vpRowVector & vpRowVector::operator=(const vpRowVector &v)
{
  unsigned int k = v.colNum ;
  if (colNum != k){
    try {
      resize(k);
    }
    catch(vpException &/*e*/)
    {
//      vpERROR_TRACE("Error caught");
      std::cerr << "Problem with resize(" << k << ") !" << std::endl;
      throw;
    }
  }

  memcpy(data, v.data, colNum*sizeof(double)) ;

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
  for (unsigned int i=0; i<rowNum; i++) {
    for (unsigned int j=0; j<colNum; j++) {
      rowPtrs[i][j] = x;
    }
  }
  return *this;
}

/*!

  Multiply a row vector by a column vector.

  \param x : Column vector.

  \warning The number of elements of the two vectors must be equal.

  \exception vpMatrixException::matrixError : If the number of elements of the
  two vectors is not the same.

  \return A scalar.

*/
double vpRowVector::operator*(const vpColVector &x) const
{
  unsigned int nelements = x.getRows();
  if (getCols() != nelements ) {
    throw(vpException(vpException::dimensionError,
                      "Bad size during vpRowVector (1x%d) by vpColVector (%dx1) multiplication",
                      colNum, x.getRows())) ;
  }

  double scalar = 0.0;

  for (unsigned int i=0; i<nelements; i++) {
    scalar += (*this)[i] * x[i];
  }
  return scalar;
}
/*!

  Multiply a row vector by a Matrix.

  \param A : Matrix.

  \warning The number of elements of the rowVector must be equal to the number
  of rows of the matrix.

  \exception vpMatrixException::matrixError : If the number of elements of the
  rowVector is not equal to the number of rows of the matrix.

  \return A vpRowVector.

*/
vpRowVector vpRowVector::operator*(const vpMatrix &A) const
{
  vpRowVector c(A.getCols());

  if (colNum != A.getRows())
  {
    vpERROR_TRACE("vpMatrix mismatch in vpRowVector/matrix multiply") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
                            "vpMatrix mismatch in vpRowVector/matrix multiply")) ;
  }

  c = 0.0;

  for (unsigned int i=0;i<colNum;i++) {
    {
      double bi = data[i] ; // optimization em 5/12/2006
      for (unsigned int j=0;j<A.getCols();j++) {
        c[j]+=bi*A[i][j];
      }
    }
  }

  return c ;
}

//! Operator A = -A
vpRowVector vpRowVector::operator-() const
{
 vpRowVector A ;
 try {
   A.resize(colNum)  ;
 }
 catch(vpException &/*e*/)
 {
   vpERROR_TRACE("Error caught") ;
   throw ;
 }

 double *vd = A.data ;   double *d = data ;

 for (unsigned int i=0; i<colNum; i++)
   *(vd++)= - (*d++);

 return A;
}

//! Substraction of two vectors V = A-v
vpRowVector vpRowVector::operator-(const vpRowVector &m) const
{
  if (getCols() != m.getCols() ) {
    throw(vpException(vpException::dimensionError,
                      "Bad size during vpRowVector (1x%d) and vpRowVector (1x%d) substraction",
                      getCols(), m.getCols())) ;
  }

  vpRowVector v(colNum) ;

  for (unsigned int i=0;i<colNum;i++)
    v[i] = (*this)[i] - m[i];
  return v;
}

//! Addition of two vectors V = A-v
vpRowVector vpRowVector::operator+(const vpRowVector &m) const
{
  if (getCols() != m.getCols() ) {
    throw(vpException(vpException::dimensionError,
                      "Bad size during vpRowVector (1x%d) and vpRowVector (1x%d) substraction",
                      getCols(), m.getCols())) ;
  }

  vpRowVector v(colNum) ;

  for (unsigned int i=0;i<colNum;i++)
    v[i] = (*this)[i] + m[i];
  return v;
}

/*!
  Copy operator.
  Allows operation such as A << v
  \code
#include <visp3/core/vpRowVector.h>

int main()
{
  vpRowVector A, B(5);
  for (unsigned int i=0; i<B.size(); i++)
    B[i] = i;
  A << B;
  std::cout << "A: \n" << A << std::endl;
}
  \endcode
  In row vector A we get:
  \code
A:
0  1  2  3  4
  \endcode

  */
vpRowVector & vpRowVector::operator<<(const vpRowVector &v)
{
  *this = v;
  return *this;
}

/*!
  \brief Transpose the row vector A

  A is defined inside the class

  \return  A^T
*/
vpColVector vpRowVector::t() const
{
  vpColVector tmp(colNum);
  memcpy(tmp.data, data, colNum*sizeof(double)) ;
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
vpRowVector::vpRowVector (vpMatrix &m, unsigned int i) : vpMatrix(m, i, 0, 1, m.getCols())
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
  double sum_square = sumSquare();
  if (std::fabs(sum_square) > std::numeric_limits<double>::epsilon()) {
    *this /= sqrt(sum_square) ;
  }

  // If sum = 0, we have a nul vector. So we return just.
  return *this;
}

/*!
  \brief reshape the row vector in a matrix
  \param nrows : number of rows of the matrix
  \param ncols : number of columns of the matrix
  \return a vpMatrix
*/
vpMatrix vpRowVector::reshape(const unsigned int &nrows,const unsigned int &ncols){
  vpMatrix m(nrows,ncols);
  reshape(m,nrows,ncols);
  return m;
}

/*!
  \brief reshape the row vector in a matrix
  \param m : the reshaped Matrix
  \param nrows : number of rows of the matrix
  \param ncols : number of columns of the matrix
*/
void vpRowVector::reshape(vpMatrix & m,const unsigned int &nrows,const unsigned int &ncols){
  if(dsize!=nrows*ncols)
  {
    vpERROR_TRACE("\n\t\t vpSubRowVector mismatch size for reshape vpSubColVector in a vpMatrix") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t \n\t\t vpSubRowVector mismatch size for reshape vpSubColVector in a vpMatrix")) ;
  }
  try 
  {
    if ((m.getRows() != nrows) || (m.getCols() != ncols)) m.resize(nrows,ncols);
  }
  catch(vpException &e)
  {
    vpERROR_TRACE("Error caught") ;
    std::cout << e << std::endl ;
    throw ;
  }
     for(unsigned int i =0; i< nrows; i++)
         for(unsigned int j =0; j< ncols; j++)
         	  m[i][j]=data[i*nrows+j];
}

/*!
  Insert a row vector.
  \param i : Index of the first element to introduce. This index starts from 0.
  \param v : Row vector to insert.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpRowVector.h>

int main()
{
  vpRowVector v(4);
  for (unsigned int i=0; i < v.size(); i++)
    v[i] = i;
  std::cout << "v:\n" << v << std::endl;

  vpRowVector w(2);
  for (unsigned int i=0; i < w.size(); i++)
    w[i] = i+10;
  std::cout << "w:\n" << w << std::endl;

  v.insert(1, w);
  std::cout << "v:\n" << v << std::endl;
}
  \endcode
  It produces the following output:
  \code
v:
0  1  2  3
w:
10  11
v:
0  10  11  3
  \endcode
 */
void vpRowVector::insert(unsigned int i, const vpRowVector &v)
{
  if (i+v.size() > this->size())
    throw(vpException(vpException::dimensionError, "Unable to insert a row vector"));
  for (unsigned int j=0; j < v.size(); j++)
    (*this)[i+j] = v[j];
}

/*!
  Stack row vector with new element.

  \param b : Element to stack to the existing one.

  \code
  vpRowVector A(3);
  double b = 3;
  A.stack(b); // A = [A b]
  // A is now a 4 dimension row vector
  \endcode

  \sa stack(const vpRowVector &, const vpRowVector &)
  \sa stack(const vpRowVector &, const vpRowVector &, vpRowVector &)

*/
void vpRowVector::stack(const double &b)
{
  this->resize(colNum+1,false);
  (*this)[colNum-1] = b;
}

/*!
  Stack row vectors.

  \param B : Vector to stack to the existing one.

  \code
  vpRowVector A(3);
  vpRowVector B(5);
  A.stack(B); // A = [A B]
  // A is now an 8 dimension row vector
  \endcode

  \sa stack(const vpRowVector &, const double &)
  \sa stack(const vpRowVector &, const vpRowVector &)
  \sa stack(const vpRowVector &, const vpRowVector &, vpRowVector &)

*/
void vpRowVector::stack(const vpRowVector &B)
{
  *this = vpRowVector::stack(*this, B);
}

/*!
  Stack row vectors.

  \param A : Initial vector.
  \param B : Vector to stack at the end of A.
  \return Stacked vector \f$[A B]\f$.

  \code
  vpRowVector A(3);
  vpRowVector B(5);
  vpRowVector C;
  C = vpRowVector::stack(A, B); // C = [A B]
  // C is now an 8 dimension row vector
  \endcode

  \sa stack(const vpRowVector &)
  \sa stack(const vpRowVector &, const vpRowVector &, vpRowVector &)
*/
vpRowVector vpRowVector::stack(const vpRowVector &A, const vpRowVector &B)
{
  vpRowVector C;
  vpRowVector::stack(A, B, C);
  return C;
}

/*!
  Stack row vectors.

  \param A : Initial vector.
  \param B : Vector to stack at the end of A.
  \param C : Resulting stacked vector \f$C = [A B]\f$.

  \code
  vpRowVector A(3);
  vpRowVector B(5);
  vpRowVector C;
  vpRowVector::stack(A, B, C); // C = [A B]
  // C is now an 8 dimension row vector
  \endcode

  \sa stack(const vpRowVector &)
  \sa stack(const vpRowVector &, const vpRowVector &)
*/
void vpRowVector::stack(const vpRowVector &A, const vpRowVector &B, vpRowVector &C)
{
  unsigned int nrA = A.getCols();
  unsigned int nrB = B.getCols();

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
  \brief Compute the mean value of all the elements of the vector
*/
double vpRowVector::mean(const vpRowVector &v)
{
  if (v.data == NULL) {
    vpERROR_TRACE("vpColVector v non initialized");
    throw(vpMatrixException(vpMatrixException::notInitializedError));
  }

  double mean = 0;
  double *vd = v.data;
  for (unsigned int i = 0; i < v.getCols(); i++)
    mean += *(vd++);

  return mean / v.getCols();
}

/*!
  Compute the median value of all the elements of the vector
*/
double
vpRowVector::median(const vpRowVector &v)
{
  if (v.data==NULL)
  {
    vpERROR_TRACE("vpColVector v non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }

  std::vector<double> vectorOfDoubles(v.size());
  for(unsigned int i = 0; i < v.size(); i++) {
    vectorOfDoubles[i] = v[i];
  }

  return vpMath::getMedian(vectorOfDoubles);
}

/*!
  Compute the standard deviation value of all the elements of the vector
*/
double
vpRowVector::stdev(const vpRowVector &v, const bool useBesselCorrection)
{
  if (v.data==NULL)
  {
    vpERROR_TRACE("vpColVector v non initialized") ;
    throw(vpMatrixException(vpMatrixException::notInitializedError)) ;
  }

  double mean_value = mean(v);
  double sum_squared_diff = 0.0;
  for(unsigned int i = 0; i < v.size(); i++) {
    sum_squared_diff += (v[i]-mean_value) * (v[i]-mean_value);
  }

  double divisor = (double) v.size();
  if(useBesselCorrection && v.size() > 1) {
    divisor = divisor-1;
  }

  return std::sqrt(sum_squared_diff / divisor);
}
