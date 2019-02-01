/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#include <assert.h>
#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <math.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <visp3/core/vpCPUFeatures.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRotationVector.h>

#if defined __SSE2__ || defined _M_X64 || (defined _M_IX86_FP && _M_IX86_FP >= 2)
#include <emmintrin.h>
#define VISP_HAVE_SSE2 1
#endif

//! Operator that allows to add two column vectors.
vpColVector vpColVector::operator+(const vpColVector &v) const
{
  if (getRows() != v.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot add (%dx1) column vector to (%dx1) column vector", getRows(),
                      v.getRows()));
  }
  vpColVector r(rowNum);

  for (unsigned int i = 0; i < rowNum; i++)
    r[i] = (*this)[i] + v[i];
  return r;
}
/*!
  Operator that allows to add a column vector to a translation vector.

  \param t : 3-dimension translation vector to add.

  \return The sum of the current columnn vector (*this) and the translation
  vector to add.
\code
  vpTranslationVector t1(1,2,3);
  vpColVector v(3);
  v[0] = 4;
  v[1] = 5;
  v[2] = 6;
  vpTranslationVector t2;

  t2 = v + t1;
  // t1 and v leave unchanged
  // t2 is now equal to : 5, 7, 9
  \endcode

*/
vpTranslationVector vpColVector::operator+(const vpTranslationVector &t) const
{
  if (getRows() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot add %d-dimension column vector to a translation vector",
                      getRows()));
  }
  vpTranslationVector s;

  for (unsigned int i = 0; i < 3; i++)
    s[i] = (*this)[i] + t[i];

  return s;
}

//! Operator that allows to add two column vectors.
vpColVector &vpColVector::operator+=(vpColVector v)
{
  if (getRows() != v.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot add (%dx1) column vector to (%dx1) column vector", getRows(),
                      v.getRows()));
  }

  for (unsigned int i = 0; i < rowNum; i++)
    (*this)[i] += v[i];
  return (*this);
}
//! Operator that allows to substract two column vectors.
vpColVector &vpColVector::operator-=(vpColVector v)
{
  if (getRows() != v.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot substract (%dx1) column vector to (%dx1) column vector",
                      getRows(), v.getRows()));
  }

  for (unsigned int i = 0; i < rowNum; i++)
    (*this)[i] -= v[i];
  return (*this);
}

/*!
   Operator that performs the dot product between two column vectors.

   \exception vpException::dimensionError If the vector dimension differ.

   \sa dotProd()
 */
double vpColVector::operator*(const vpColVector &v) const
{
  if (size() != v.size()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot compute the dot product between column vectors "
                      "with different dimensions (%d) and (%d)",
                      size(), v.size()));
  }
  double r = 0;

  for (unsigned int i = 0; i < rowNum; i++)
    r += (*this)[i] * v[i];
  return r;
}

/*!

  Multiply a column vector by a row vector.

  \param v : Row vector.

  \return The resulting matrix.

*/
vpMatrix vpColVector::operator*(const vpRowVector &v) const
{
  vpMatrix M(rowNum, v.getCols());
  for (unsigned int i = 0; i < rowNum; i++) {
    for (unsigned int j = 0; j < v.getCols(); j++) {
      M[i][j] = (*this)[i] * v[j];
    }
  }
  return M;
}

//! operator substraction of two vectors V = A-v
vpColVector vpColVector::operator-(const vpColVector &m) const
{
  if (getRows() != m.getRows()) {
    throw(vpException(vpException::dimensionError,
                      "Bad size during vpColVector (%dx1) and vpColVector "
                      "(%dx1) substraction",
                      getRows(), m.getRows()));
  }
  vpColVector v(rowNum);

  for (unsigned int i = 0; i < rowNum; i++)
    v[i] = (*this)[i] - m[i];
  return v;
}

/*!
  Construct a column vector from a part of an input column vector \e v.

  \param v : Input column vector used for initialization.
  \param r : row index in \e v that corresponds to the first element of the
  column vector to contruct. \param nrows : Number of rows of the constructed
  column vector.

  The sub-vector starting from v[r] element and ending on v[r+nrows-1] element
  is used to initialize the contructed column vector.

  \sa init()
*/
vpColVector::vpColVector(const vpColVector &v, unsigned int r, unsigned int nrows) : vpArray2D<double>(nrows, 1)
{
  init(v, r, nrows);
}

/*!
  Initialize the column vector from a part of an input column vector \e v.

  \param v : Input column vector used for initialization.
  \param r : row index in \e v that corresponds to the first element of the
  column vector to contruct.
  \param nrows : Number of rows of the constructed
  column vector.

  The sub-vector starting from v[r] element and ending on v[r+nrows-1] element
  is used to initialize the contructed column vector.

  The following code shows how to use this function:
\code
#include <visp3/core/vpColVector.h>

int main()
{
  vpColVector v(4);
  int val = 0;
  for(size_t i=0; i<v.getRows(); i++) {
    v[i] = val++;
  }
  std::cout << "v: " << v.t() << std::endl;

  vpColVector w;
  w.init(v, 0, 2);
  std::cout << "w: " << w.t() << std::endl;

}
\endcode
  It produces the following output:
  \code
v: 0 1 2 3
w: 1 2
  \endcode
 */
void vpColVector::init(const vpColVector &v, unsigned int r, unsigned int nrows)
{
  unsigned int rnrows = r + nrows;

  if (rnrows > v.getRows())
    throw(vpException(vpException::dimensionError, "Bad row dimension (%d > %d) used to initialize vpColVector", rnrows,
                      v.getRows()));
  resize(nrows, false);

  if (this->rowPtrs == NULL) // Fix coverity scan: explicit null dereferenced
    return;                  // Nothing to do
  for (unsigned int i = r; i < rnrows; i++)
    (*this)[i - r] = v[i];
}

vpColVector::vpColVector(const vpRotationVector &v) : vpArray2D<double>(v.size(), 1)
{
  for (unsigned int i = 0; i < v.size(); i++)
    (*this)[i] = v[i];
}

vpColVector::vpColVector(const vpPoseVector &p) : vpArray2D<double>(p.size(), 1)
{
  for (unsigned int i = 0; i < p.size(); i++)
    (*this)[i] = p[i];
}

vpColVector::vpColVector(const vpTranslationVector &v) : vpArray2D<double>(v.size(), 1)
{
  for (unsigned int i = 0; i < v.size(); i++)
    (*this)[i] = v[i];
}

//! Constructor that take column j of matrix M.
vpColVector::vpColVector(const vpMatrix &M, unsigned int j) : vpArray2D<double>(M.getRows(), 1)
{
  for (unsigned int i = 0; i < M.getCols(); i++)
    (*this)[i] = M[i][j];
}

/*!
   Constructor that creates a column vector from a m-by-1 matrix \e M.

   \exception vpException::dimensionError If the matrix is not a m-by-1
   matrix.
 */
vpColVector::vpColVector(const vpMatrix &M) : vpArray2D<double>(M.getRows(), 1)
{
  if (M.getCols() != 1) {
    throw(vpException(vpException::dimensionError, "Cannot construct a (%dx1) row vector from a (%dx%d) matrix",
                      M.getRows(), M.getRows(), M.getCols()));
  }

  for (unsigned int i = 0; i < M.getRows(); i++)
    (*this)[i] = M[i][0];
}

/*!
   Constructor that creates a column vector from a std vector of double.
 */
vpColVector::vpColVector(const std::vector<double> &v) : vpArray2D<double>((unsigned int)v.size(), 1)
{
  for (unsigned int i = 0; i < v.size(); i++)
    (*this)[i] = v[i];
}
/*!
   Constructor that creates a column vector from a std vector of float.
 */
vpColVector::vpColVector(const std::vector<float> &v) : vpArray2D<double>((unsigned int)v.size(), 1)
{
  for (unsigned int i = 0; i < v.size(); i++)
    (*this)[i] = (double)(v[i]);
}

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
vpColVector::vpColVector(vpColVector &&v) : vpArray2D<double>()
{
  rowNum = v.rowNum;
  colNum = v.colNum;
  rowPtrs = v.rowPtrs;
  dsize = v.dsize;
  data = v.data;

  v.rowNum = 0;
  v.colNum = 0;
  v.rowPtrs = NULL;
  v.dsize = 0;
  v.data = NULL;
}
#endif

/*!
   Operator that allows to negate all the column vector elements.

   \code
   vpColVector r(3, 1);
   // r contains [1 1 1]^T
   vpColVector v = -r;
   // v contains [-1 -1 -1]^T
   \endcode
 */
vpColVector vpColVector::operator-() const
{
  vpColVector A;
  A.resize(rowNum, false);

  double *vd = A.data;
  double *d = data;

  for (unsigned int i = 0; i < rowNum; i++)
    *(vd++) = -(*d++);

  return A;
}

/*!
  Operator that allows to multiply each element of a column vector by a
  scalar.

  \param x : The scalar.

  \return The column vector multiplied by the scalar. The current
  column vector (*this) is unchanged.

  \code
  vpColVector v(3);
  v[0] = 1;
  v[1] = 2;
  v[2] = 3;

  vpColVector w = v * 3;
  // v is unchanged
  // w is now equal to : [3, 6, 9]
  \endcode
*/
vpColVector vpColVector::operator*(double x) const
{
  vpColVector v(rowNum);

  double *vd = v.data;
  double *d = data;

  for (unsigned int i = 0; i < rowNum; i++)
    *(vd++) = (*d++) * x;
  return v;
}

/*!
  Operator that allows to multiply each element of a column vector by a
  scalar.

  \param x : The scalar.

  \return The column vector multiplied by the scalar.

  \code
  vpColVector v(3);
  v[0] = 1;
  v[1] = 2;
  v[2] = 3;

  v *= 3;
  // v is now equal to : [3, 6, 9]
  \endcode
*/
vpColVector &vpColVector::operator*=(double x)
{
  for (unsigned int i = 0; i < rowNum; i++)
    (*this)[i] *= x;
  return (*this);
}

/*!
  Operator that allows to divide each element of a column vector by a scalar.

  \param x : The scalar.

  \return The column vector divided by the scalar.

  \code
  vpColVector v(3);
  v[0] = 8;
  v[1] = 4;
  v[2] = 2;

  v /= 2;
  // v is now equal to : [4, 2, 1]
  \endcode
*/
vpColVector &vpColVector::operator/=(double x)
{
  for (unsigned int i = 0; i < rowNum; i++)
    (*this)[i] /= x;
  return (*this);
}

/*!
  Operator that allows to divide each element of a column vector by a scalar.

  \param x : The scalar.

  \return The column vector divided by the scalar. The current
  column vector (*this) is unchanged.

  \code
  vpColVector v(3);
  v[0] = 8;
  v[1] = 4;
  v[2] = 2;

  vpColVector w = v / 2;
  // v is unchanged
  // w is now equal to : [4, 2, 1]
  \endcode
*/
vpColVector vpColVector::operator/(double x) const
{
  vpColVector v(rowNum);

  double *vd = v.data;
  double *d = data;

  for (unsigned int i = 0; i < rowNum; i++)
    *(vd++) = (*d++) / x;
  return v;
}

/*!
  Transform a m-by-1 matrix into a column vector.
  \warning  Handled with care; M should be a 1 column matrix.
  \exception vpException::dimensionError If the matrix has more than 1 column.
*/
vpColVector &vpColVector::operator=(const vpMatrix &M)
{
  if (M.getCols() != 1) {
    throw(vpException(vpException::dimensionError, "Cannot transform a (%dx%d) matrix into a column vector",
                      M.getRows(), M.getCols()));
  }

  resize(M.getRows(), false);
  memcpy(data, M.data, rowNum * sizeof(double));

  return (*this);
}

/*!
  Initialize a row vector from a standard vector of double.
*/
vpColVector &vpColVector::operator=(const std::vector<double> &v)
{
  resize((unsigned int)v.size(), false);
  for (unsigned int i = 0; i < v.size(); i++)
    (*this)[i] = v[i];
  return *this;
}
/*!
  Initialize a row vector from a standard vector of double.
*/
vpColVector &vpColVector::operator=(const std::vector<float> &v)
{
  resize((unsigned int)v.size(), false);
  for (unsigned int i = 0; i < v.size(); i++)
    (*this)[i] = (float)v[i];
  return *this;
}

vpColVector &vpColVector::operator=(const vpColVector &v)
{
  unsigned int k = v.rowNum;
  if (rowNum != k) {
    resize(k, false);
  }

  memcpy(data, v.data, rowNum * sizeof(double));
  return *this;
}

/*!
   Operator that allows to convert a translation vector into a column vector.
 */
vpColVector &vpColVector::operator=(const vpTranslationVector &tv)
{
  unsigned int k = tv.getRows();
  if (rowNum != k) {
    resize(k, false);
  }

  memcpy(data, tv.data, rowNum * sizeof(double));
  return *this;
}
/*!
   Operator that allows to convert a rotation vector into a column vector.
 */
vpColVector &vpColVector::operator=(const vpRotationVector &rv)
{
  unsigned int k = rv.getRows();
  if (rowNum != k) {
    resize(k, false);
  }

  memcpy(data, rv.data, rowNum * sizeof(double));
  return *this;
}
/*!
   Operator that allows to convert a pose vector into a column vector.
 */
vpColVector &vpColVector::operator=(const vpPoseVector &p)
{
  unsigned int k = p.getRows();
  if (rowNum != k) {
    resize(k, false);
  }

  memcpy(data, p.data, rowNum * sizeof(double));
  return *this;
}

/*!
  Copy operator.
  Allows operation such as A << v
  \code
#include <visp3/core/vpColVector.h>

int main()
{
  vpColVector A, B(5);
  for (unsigned int i=0; i<B.size(); i++)
    B[i] = i;
  A << B;
  std::cout << "A: " << A.t() << std::endl;
}
  \endcode
  In column vector A we get:
  \code
A: 0 1 2 3 4
  \endcode
  */
vpColVector &vpColVector::operator<<(const vpColVector &v)
{
  *this = v;
  return *this;
}

/*!
  Assigment operator.   Allow operation such as A = *v

  The following example shows how to use this operator.
  \code
#include <visp3/core/vpColVector.h>

int main()
{
  size_t n = 5;
  vpColVector A(n);
  double *B = new double [n];
  for (unsigned int i = 0; i < n; i++)
    B[i] = i;
  A << B;
  std::cout << "A: " << A.t() << std::endl;
  delete [] B;
}
  \endcode
  It produces the following output:
  \code
A: 0 1 2 3 4
  \endcode
  */
vpColVector &vpColVector::operator<<(double *x)
{
  for (unsigned int i = 0; i < rowNum; i++) {
    for (unsigned int j = 0; j < colNum; j++) {
      rowPtrs[i][j] = *x++;
    }
  }
  return *this;
}

//! Set each element of the column vector to x.
vpColVector &vpColVector::operator=(double x)
{
  double *d = data;

  for (unsigned int i = 0; i < rowNum; i++)
    *(d++) = x;
  return *this;
}

/*!
 * Converts the vpColVector to a std::vector.
 * \return The corresponding std::vector<double>.
 */
std::vector<double> vpColVector::toStdVector()
{
  std::vector<double> v(this->size());

  for (unsigned int i = 0; i < this->size(); i++)
    v[i] = data[i];
  return v;
}

#ifdef VISP_HAVE_CPP11_COMPATIBILITY
vpColVector &vpColVector::operator=(vpColVector &&other)
{
  if (this != &other) {
    free(data);
    free(rowPtrs);

    rowNum = other.rowNum;
    colNum = other.colNum;
    rowPtrs = other.rowPtrs;
    dsize = other.dsize;
    data = other.data;

    other.rowNum = 0;
    other.colNum = 0;
    other.rowPtrs = NULL;
    other.dsize = 0;
    other.data = NULL;
  }

  return *this;
}
#endif

bool vpColVector::operator==(const vpColVector &v) const {
  if (rowNum != v.rowNum ||
      colNum != v.colNum /* should not happen */)
    return false;

  for (unsigned int i = 0; i < rowNum; i++) {
    if (!vpMath::equal(data[i], v.data[i], std::numeric_limits<double>::epsilon()))
      return false;
  }

  return true;
}

bool vpColVector::operator!=(const vpColVector &v) const {
  return !(*this == v);
}

/*!
  Transpose the column vector. The resulting vector becomes a row vector.
*/
vpRowVector vpColVector::t() const
{
  vpRowVector v(rowNum);
  memcpy(v.data, data, rowNum * sizeof(double));
  return v;
}

/*!
  Transpose the column vector. The resulting vector becomes a row vector.
  \sa t()
*/
vpRowVector vpColVector::transpose() const { return t(); }

/*!
  Transpose the column vector. The resulting vector \e v becomes a row vector.
  \sa t()
*/
void vpColVector::transpose(vpRowVector &v) const { v = t(); }

/*!
  \relates vpColVector
  Allows to multiply a scalar by a column vector.
*/
vpColVector operator*(const double &x, const vpColVector &v)
{
  vpColVector vout;
  vout = v * x;
  return vout;
}

/*!
  Compute end return the dot product of two column vectors:
  \f[ a \cdot b = \sum_{i=0}^n a_i * b_i\f] where \e n is the dimension of
  both vectors.

  \exception vpException::dimensionError If the vector dimension differ.
*/
double vpColVector::dotProd(const vpColVector &a, const vpColVector &b)
{
  if (a.data == NULL) {
    throw(vpException(vpException::fatalError, "Cannot compute the dot product: first vector empty"));
  }
  if (b.data == NULL) {
    throw(vpException(vpException::fatalError, "Cannot compute the dot product: second vector empty"));
  }
  if (a.size() != b.size()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot compute the dot product between column vectors "
                      "with different dimensions (%d) and (%d)",
                      a.size(), b.size()));
  }

  double *ad = a.data;
  double *bd = b.data;

  double c = 0;
  for (unsigned int i = 0; i < a.getRows(); i++)
    c += *(ad++) * *(bd++);
  //  vpMatrix c = (a.t() * b);
  //  return c[0][0];
  return c;
}

/*!
  Normalise the vector:

  \f[
  {\bf x}_i = \frac{{\bf x}_i}{\sqrt{\sum_{i=1}^{n}x^2_i}}
  \f]
*/
vpColVector &vpColVector::normalize(vpColVector &x) const
{
  x = x / sqrt(x.sumSquare());

  return x;
}

/*!
  Normalise the vector:

  \f[
  {\bf x}_i = \frac{{\bf x}_i}{\sqrt{\sum_{i=1}^{n}x^2_i}}
  \f]
*/
vpColVector &vpColVector::normalize()
{

  double sum_square = sumSquare();

  // if (sum != 0.0)
  if (std::fabs(sum_square) > std::numeric_limits<double>::epsilon())
    *this /= sqrt(sum_square);

  // If sum = 0, we have a nul vector. So we return just.
  return *this;
}

/*!
   Return a column vector with elements of \e v that are reverse sorted with
   values going from greatest to lowest.

   Example:
   \code
#include <visp/vpColVector.h>

int main()
{
  vpColVector v(10);
  v[0] = 5; v[1] = 7; v[2] = 4; v[3] = 2; v[4] = 8;
  v[5] = 6; v[6] = 1; v[7] = 9; v[8] = 0; v[9] = 3;

  std::cout << "v: " << v.t() << std::endl;

  vpColVector s = vpColVector::invSort(v);
  std::cout << "s: " << s.t() << std::endl;
}
   \endcode
   Output:
   \code
v: 5  7  4  2  8  6  1  9  0  3
s: 9  8  7  6  5  4  3  2  1  0
   \endcode

   \sa sort()
 */
vpColVector vpColVector::invSort(const vpColVector &v)
{
  if (v.data == NULL) {
    throw(vpException(vpException::fatalError, "Cannot sort content of column vector: vector empty"));
  }
  vpColVector tab;
  tab = v;
  unsigned int nb_permutation = 1;
  unsigned int i = 0;
  while (nb_permutation != 0) {
    nb_permutation = 0;
    for (unsigned int j = v.getRows() - 1; j >= i + 1; j--) {
      if ((tab[j] > tab[j - 1])) {
        double tmp = tab[j];
        tab[j] = tab[j - 1];
        tab[j - 1] = tmp;
        nb_permutation++;
      }
    }
    i++;
  }

  return tab;
}

/*!
   Return a column vector with elements of \e v that are sorted with values
   going from lowest to geatest.

   Example:
   \code
#include <visp/vpColVector.h>

int main()
{
  vpColVector v(10);
  v[0] = 5; v[1] = 7; v[2] = 4; v[3] = 2; v[4] = 8;
  v[5] = 6; v[6] = 1; v[7] = 9; v[8] = 0; v[9] = 3;

  std::cout << "v: " << v.t() << std::endl;

  vpColVector s = vpColVector::sort(v);
  std::cout << "s: " << s.t() << std::endl;
}
   \endcode
   Output:
   \code
v: 5  7  4  2  8  6  1  9  0  3
s: 0  1  2  3  4  5  6  7  8  9
   \endcode
   \sa invSort()
 */
vpColVector vpColVector::sort(const vpColVector &v)
{
  if (v.data == NULL) {
    throw(vpException(vpException::fatalError, "Cannot sort content of column vector: vector empty"));
  }
  vpColVector tab;
  tab = v;
  unsigned int nb_permutation = 1;
  unsigned int i = 0;
  while (nb_permutation != 0) {
    nb_permutation = 0;
    for (unsigned int j = v.getRows() - 1; j >= i + 1; j--) {
      if ((tab[j] < tab[j - 1])) {
        double tmp = tab[j];
        tab[j] = tab[j - 1];
        tab[j - 1] = tmp;
        nb_permutation++;
      }
    }
    i++;
  }

  return tab;
}

/*!
  Stack column vector with a new element at the end of the vector.

  \param d : Element to stack to the existing vector.

  \code
  vpColVector v(3, 1);
  // v is equal to [1 1 1]^T
  v.stack(-2);
  // v is equal to [1 1 1 -2]^T
  \endcode

  \sa stack(const vpColVector &, const vpColVector &)
  \sa stack(const vpColVector &, const vpColVector &, vpColVector &)

*/
void vpColVector::stack(double d)
{
  this->resize(rowNum + 1, false);
  (*this)[rowNum - 1] = d;
}

/*!
  Stack column vectors.

  \param v : Vector to stack to the existing one.

  \code
  vpColVector v1(3, 1);
  // v1 is equal to [1 1 1]^T
  vpColVector v2(2, 3);
  // v2 is equal to [3 3]^T
  v1.stack(v2);
  // v1 is equal to [1 1 1 3 3]^T
  \endcode

  \sa stack(const vpColVector &, const double &)
  \sa stack(const vpColVector &, const vpColVector &)
  \sa stack(const vpColVector &, const vpColVector &, vpColVector &)

*/
void vpColVector::stack(const vpColVector &v) { *this = vpColVector::stack(*this, v); }

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
  C.resize(nrA + nrB, false);

  for (unsigned int i = 0; i < nrA; i++)
    C[i] = A[i];

  for (unsigned int i = 0; i < nrB; i++)
    C[nrA + i] = B[i];
}

/*!
  Compute the mean value of all the elements of the vector.
*/
double vpColVector::mean(const vpColVector &v)
{
  if (v.data == NULL || v.size() == 0) {
    throw(vpException(vpException::dimensionError, "Cannot compute column vector mean: vector empty"));
  }

  // Use directly sum() function
  double mean = v.sum();

  // Old code used
  //  double *vd = v.data;
  //  for (unsigned int i=0 ; i < v.getRows() ; i++)
  //    mean += *(vd++);

  return mean / v.getRows();
}

/*!
  Compute the median value of all the elements of the vector.
*/
double vpColVector::median(const vpColVector &v)
{
  if (v.data == NULL || v.size() == 0) {
    throw(vpException(vpException::dimensionError, "Cannot compute column vector median: vector empty"));
  }

  std::vector<double> vectorOfDoubles(v.data, v.data + v.rowNum);

  return vpMath::getMedian(vectorOfDoubles);
}

/*!
  Compute the standard deviation value of all the elements of the vector.
*/
double vpColVector::stdev(const vpColVector &v, const bool useBesselCorrection)
{
  if (v.data == NULL || v.size() == 0) {
    throw(vpException(vpException::dimensionError, "Cannot compute column vector stdev: vector empty"));
  }

  double mean_value = mean(v);
  double sum_squared_diff = 0.0;
  unsigned int i = 0;

#if VISP_HAVE_SSE2
  if (vpCPUFeatures::checkSSE2()) {
    __m128d v_sub, v_mul, v_sum = _mm_setzero_pd();
    __m128d v_mean = _mm_set1_pd(mean_value);

    if (v.getRows() >= 4) {
      for (; i <= v.getRows() - 4; i += 4) {
        v_sub = _mm_sub_pd(_mm_loadu_pd(v.data + i), v_mean);
        v_mul = _mm_mul_pd(v_sub, v_sub);
        v_sum = _mm_add_pd(v_mul, v_sum);

        v_sub = _mm_sub_pd(_mm_loadu_pd(v.data + i + 2), v_mean);
        v_mul = _mm_mul_pd(v_sub, v_sub);
        v_sum = _mm_add_pd(v_mul, v_sum);
      }
    }

    double res[2];
    _mm_storeu_pd(res, v_sum);

    sum_squared_diff = res[0] + res[1];
  }
// Old code used before SSE
//#else
//  for(unsigned int i = 0; i < v.size(); i++) {
//    sum_squared_diff += (v[i]-mean_value) * (v[i]-mean_value);
//  }
#endif

  for (; i < v.getRows(); i++) {
    sum_squared_diff += (v[i] - mean_value) * (v[i] - mean_value);
  }

  double divisor = (double)v.size();
  if (useBesselCorrection && v.size() > 1) {
    divisor = divisor - 1;
  }

  return std::sqrt(sum_squared_diff / divisor);
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
vpMatrix vpColVector::skew(const vpColVector &v)
{
  vpMatrix M;
  if (v.getRows() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot compute skew vector of a non 3-dimention vector (%d)",
                      v.getRows()));
  }

  M.resize(3, 3, false, false);
  M[0][0] = 0;
  M[0][1] = -v[2];
  M[0][2] = v[1];
  M[1][0] = v[2];
  M[1][1] = 0;
  M[1][2] = -v[0];
  M[2][0] = -v[1];
  M[2][1] = v[0];
  M[2][2] = 0;

  return M;
}

/*!
  Compute and return the cross product of two vectors \f$a \times b\f$.

  \param a : 3-dimension column vector.
  \param b : 3-dimension column vector.
  \return The cross product \f$a \times b\f$.

  \exception vpException::dimensionError If the vectors dimension is not equal
  to 3.
*/
vpColVector vpColVector::crossProd(const vpColVector &a, const vpColVector &b)
{
  if (a.getRows() != 3 || b.getRows() != 3) {
    throw(vpException(vpException::dimensionError,
                      "Cannot compute the cross product between column "
                      "vector with dimension %d and %d",
                      a.getRows(), b.getRows()));
  }

  return vpColVector::skew(a) * b;
}

/*!
  Reshape the column vector in a matrix.
  \param nrows : number of rows of the matrix
  \param ncols : number of columns of the matrix
  \return The reshaped matrix.

  \sa reshape(vpMatrix &, const unsigned int &, const unsigned int &)
*/
vpMatrix vpColVector::reshape(const unsigned int &nrows, const unsigned int &ncols)
{
  vpMatrix M(nrows, ncols);
  reshape(M, nrows, ncols);
  return M;
}

/*!
  Reshape the column vector in a matrix.
  \param M : the reshaped matrix.
  \param nrows : number of rows of the matrix.
  \param ncols : number of columns of the matrix.

  \exception vpException::dimensionError If the matrix and the column vector
have not the same size.

  The following example shows how to use this method.
  \code
#include <visp/vpColVector.h>

int main()
{
  int var=0;
  vpMatrix mat(3, 4);
  for (int i = 0; i < 3; i++)
      for (int j = 0; j < 4; j++)
          mat[i][j] = ++var;
  std::cout << "mat: \n" << mat << std::endl;

  vpColVector col = mat.stackColumns();
  std::cout << "column vector: \n" << col << std::endl;

  vpMatrix remat = col.reshape(3, 4);
  std::cout << "remat: \n" << remat << std::endl;
}
  \endcode

  If you run the previous example, you get:
  \code
mat:
1  2  3  4
5  6  7  8
9  10  11  12
column vector:
1
5
9
2
6
10
3
7
11
4
8
12
remat:
1  2  3  4
5  6  7  8
9  10  11  12
  \endcode
*/
void vpColVector::reshape(vpMatrix &M, const unsigned int &nrows, const unsigned int &ncols)
{
  if (dsize != nrows * ncols) {
    throw(vpException(vpException::dimensionError, "Cannot reshape (%dx1) column vector in (%dx%d) matrix", rowNum,
                      M.getRows(), M.getCols()));
  }
  if ((M.getRows() != nrows) || (M.getCols() != ncols))
    M.resize(nrows, ncols, false, false);

  for (unsigned int j = 0; j < ncols; j++)
    for (unsigned int i = 0; i < nrows; i++)
      M[i][j] = data[j * nrows + i];
}

/*!
  Insert a column vector.
  \param i : Index of the first element to introduce. This index starts from
0. \param v : Column vector to insert.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpColVector.h>

int main()
{
  vpColVector v(4);
  for (unsigned int i=0; i < v.size(); i++)
    v[i] = i;
  std::cout << "v: " << v.t() << std::endl;

  vpColVector w(2);
  for (unsigned int i=0; i < w.size(); i++)
    w[i] = i+10;
  std::cout << "w: " << w.t() << std::endl;

  v.insert(1, w);
  std::cout << "v: " << v.t() << std::endl;
}
  \endcode
  It produces the following output:
  \code
v: 0 1 2 3
w: 10 11
v: 0 10 11 3
  \endcode
 */
void vpColVector::insert(unsigned int i, const vpColVector &v)
{
  if (i + v.size() > this->size())
    throw(vpException(vpException::dimensionError, "Unable to insert a column vector"));

  if (data != NULL && v.data != NULL && v.rowNum > 0) {
    memcpy(data + i, v.data, sizeof(double) * v.rowNum);
  }
}

/*!

  Pretty print a column vector. The data are tabulated.
  The common widths before and after the decimal point
  are set with respect to the parameter maxlen.

  \param s Stream used for the printing.

  \param length The suggested width of each vector element.
  The actual width grows in order to accomodate the whole integral part,
  and shrinks if the whole extent is not needed for all the numbers.
  \param intro The introduction which is printed before the vector.
  Can be set to zero (or omitted), in which case
  the introduction is not printed.

  \return Returns the common total width for all vector elements.

  \sa std::ostream &operator<<(std::ostream &s, const vpArray2D<Type> &A)
*/
int vpColVector::print(std::ostream &s, unsigned int length, char const *intro) const
{
  typedef std::string::size_type size_type;

  unsigned int m = getRows();
  unsigned int n = 1;

  std::vector<std::string> values(m * n);
  std::ostringstream oss;
  std::ostringstream ossFixed;
  std::ios_base::fmtflags original_flags = oss.flags();

  // ossFixed <<std::fixed;
  ossFixed.setf(std::ios::fixed, std::ios::floatfield);

  size_type maxBefore = 0; // the length of the integral part
  size_type maxAfter = 0;  // number of decimals plus
  // one place for the decimal point
  for (unsigned int i = 0; i < m; ++i) {
    oss.str("");
    oss << (*this)[i];
    if (oss.str().find("e") != std::string::npos) {
      ossFixed.str("");
      ossFixed << (*this)[i];
      oss.str(ossFixed.str());
    }

    values[i] = oss.str();
    size_type thislen = values[i].size();
    size_type p = values[i].find('.');

    if (p == std::string::npos) {
      maxBefore = vpMath::maximum(maxBefore, thislen);
      // maxAfter remains the same
    } else {
      maxBefore = vpMath::maximum(maxBefore, p);
      maxAfter = vpMath::maximum(maxAfter, thislen - p - 1);
    }
  }

  size_type totalLength = length;
  // increase totalLength according to maxBefore
  totalLength = vpMath::maximum(totalLength, maxBefore);
  // decrease maxAfter according to totalLength
  maxAfter = (std::min)(maxAfter, totalLength - maxBefore);
  if (maxAfter == 1)
    maxAfter = 0;

  // the following line is useful for debugging
  // std::cerr <<totalLength <<" " <<maxBefore <<" " <<maxAfter <<"\n";

  if (intro)
    s << intro;
  s << "[" << m << "," << n << "]=\n";

  for (unsigned int i = 0; i < m; i++) {
    s << "  ";
    size_type p = values[i].find('.');
    s.setf(std::ios::right, std::ios::adjustfield);
    s.width((std::streamsize)maxBefore);
    s << values[i].substr(0, p).c_str();

    if (maxAfter > 0) {
      s.setf(std::ios::left, std::ios::adjustfield);
      if (p != std::string::npos) {
        s.width((std::streamsize)maxAfter);
        s << values[i].substr(p, maxAfter).c_str();
      } else {
        assert(maxAfter > 1);
        s.width((std::streamsize)maxAfter);
        s << ".0";
      }
    }

    s << ' ';

    s << std::endl;
  }

  s.flags(original_flags); // restore s to standard state

  return (int)(maxBefore + maxAfter);
}

/*!
  Return the sum of all the elements \f$v_{i}\f$ of the column vector v(m).

  \return The value \f[\sum{i=0}^{m} v_i\f].
  */
double vpColVector::sum() const
{
  double sum = 0.0;
  unsigned int i = 0;

#if VISP_HAVE_SSE2
  if (vpCPUFeatures::checkSSE2()) {
    __m128d v_sum1 = _mm_setzero_pd(), v_sum2 = _mm_setzero_pd(), v_sum;

    if (rowNum >= 4) {
      for (; i <= rowNum - 4; i += 4) {
        v_sum1 = _mm_add_pd(_mm_loadu_pd(data + i), v_sum1);
        v_sum2 = _mm_add_pd(_mm_loadu_pd(data + i + 2), v_sum2);
      }
    }

    v_sum = _mm_add_pd(v_sum1, v_sum2);

    double res[2];
    _mm_storeu_pd(res, v_sum);

    sum = res[0] + res[1];
  }
// Old code used before SSE
//#else
//  for (unsigned int i=0;i<rowNum;i++) {
//    sum += rowPtrs[i][0];
//  }
#endif

  for (; i < rowNum; i++) {
    sum += (*this)[i];
  }

  return sum;
}

/*!
  Return the sum square of all the elements \f$v_{i}\f$ of the column vector
  v(m).

  \return The value \f[\sum{i=0}^{m} v_i^{2}\f].
  */
double vpColVector::sumSquare() const
{
  double sum_square = 0.0;
  unsigned int i = 0;

#if VISP_HAVE_SSE2
  if (vpCPUFeatures::checkSSE2()) {
    __m128d v_mul1, v_mul2;
    __m128d v_sum = _mm_setzero_pd();

    if (rowNum >= 4) {
      for (; i <= rowNum - 4; i += 4) {
        v_mul1 = _mm_mul_pd(_mm_loadu_pd(data + i), _mm_loadu_pd(data + i));
        v_mul2 = _mm_mul_pd(_mm_loadu_pd(data + i + 2), _mm_loadu_pd(data + i + 2));

        v_sum = _mm_add_pd(v_mul1, v_sum);
        v_sum = _mm_add_pd(v_mul2, v_sum);
      }
    }

    double res[2];
    _mm_storeu_pd(res, v_sum);

    sum_square = res[0] + res[1];
  }
// Old code used before SSE
//#else
//  for (unsigned int i=0;i<rowNum;i++) {
//    double x=rowPtrs[i][0];
//    sum_square += x*x;
//  }
#endif

  for (; i < rowNum; i++) {
    sum_square += (*this)[i] * (*this)[i];
  }

  return sum_square;
}

/*!
  \deprecated This function is deprecated. You should rather use frobeniusNorm().

  Compute and return the Euclidean norm also called Fronebius norm \f$ ||v|| = \sqrt{ \sum{v_{i}^2}} \f$.

  \return The Euclidean norm if the vector is initialized, 0 otherwise.

  \sa frobeniusNorm(), infinityNorm()

*/
double vpColVector::euclideanNorm() const
{
  return frobeniusNorm();
}

/*!
  Compute and return the Fronebius norm \f$ ||v|| = \sqrt{ \sum{v_{i}^2}} \f$.

  \return The Fronebius norm if the vector is initialized, 0 otherwise.

  \sa infinityNorm()

*/
double vpColVector::frobeniusNorm() const
{
  double norm = sumSquare();

  return sqrt(norm);
}

/*!
  Compute the Hadamard product (element wise vector multiplication).
  \param v : Second vector;
  \return v1.hadamard(v2) The kronecker product : \f$ v1 \circ v2 = (v1 \circ
  v2)_{i} = (v1)_{i} (v2)_{i} \f$
*/
vpColVector vpColVector::hadamard(const vpColVector &v) const
{
  if (v.getRows() != rowNum || v.getCols() != colNum) {
    throw(vpException(vpException::dimensionError, "Hadamard product: bad dimensions!"));
  }

  vpColVector out;
  out.resize(rowNum, false);

  unsigned int i = 0;

#if VISP_HAVE_SSE2
  if (vpCPUFeatures::checkSSE2() && dsize >= 2) {
    for (; i <= dsize - 2; i += 2) {
      __m128d vout = _mm_mul_pd(_mm_loadu_pd(data + i), _mm_loadu_pd(v.data + i));
      _mm_storeu_pd(out.data + i, vout);
    }
  }
#endif

  for (; i < dsize; i++) {
    out.data[i] = data[i] * v.data[i];
  }

  return out;
}

/*!

  Compute and return the infinity norm \f$ {||v||}_{\infty} =
  max\left({\mid v_{i} \mid}\right) \f$ with \f$i \in
  \{0, ..., m-1\}\f$ where \e m is the vector size and \f$v_i\f$ an element of
  the vector.

  \return The infinity norm if the matrix is initialized, 0 otherwise.

  \sa frobeniusNorm()
*/
double vpColVector::infinityNorm() const
{
  double norm = 0.0;
  for (unsigned int i = 0; i < rowNum; i++) {
    double x = fabs((*this)[i]);
    if (x > norm) {
      norm = x;
    }
  }
  return norm;
}

/*!
  Print to be used as part of a C++ code later.

  \param os : the stream to be printed in.
  \param matrixName : name of the column vector, "A" by default.
  \param octet : if false, print using double, if true, print byte per byte
  each bytes of the double array.

  The following code shows how to use this function:
\code
#include <visp3/core/vpColVector.h>

int main()
{
  vpColVector v(3);
  for (unsigned int i=0; i<v.size(); i++)
    v[i] = i;
  v.cppPrint(std::cout, "v");
}
\endcode
  It produces the following output that could be copy/paste in a C++ code:
  \code
vpColVector v (3);
v[0] = 0;
v[1] = 1;
v[2] = 2;
  \endcode
*/
std::ostream &vpColVector::cppPrint(std::ostream &os, const std::string &matrixName, bool octet) const
{
  os << "vpColVector " << matrixName << " (" << this->getRows() << "); " << std::endl;

  for (unsigned int i = 0; i < this->getRows(); ++i) {

    if (!octet) {
      os << matrixName << "[" << i << "] = " << (*this)[i] << "; " << std::endl;
    } else {
      for (unsigned int k = 0; k < sizeof(double); ++k) {
        os << "((unsigned char*)&(" << matrixName << "[" << i << "]) )[" << k << "] = 0x" << std::hex
           << (unsigned int)((unsigned char *)&((*this)[i]))[k] << "; " << std::endl;
      }
    }
  }
  std::cout << std::endl;
  return os;
};

/*!
  Print/save a column vector in csv format.

  The following code
  \code
#include <visp3/core/vpColVector.h>

int main()
{
  std::ofstream ofs("log.csv", std::ofstream::out);
  vpColVector v(3);
  for (unsigned int i=0; i<v.size(); i++)
    v[i] = i;

  v.csvPrint(ofs);

  ofs.close();
}
  \endcode
  produces log.csv file that contains:
  \code
0
1
2
  \endcode
*/
std::ostream &vpColVector::csvPrint(std::ostream &os) const
{
  for (unsigned int i = 0; i < this->getRows(); ++i) {
    os << (*this)[i];

    os << std::endl;
  }
  return os;
};

/*!
  Print using Maple syntax, to copy/paste in Maple later.

  The following code
  \code
#include <visp3/core/vpColVector.h>

int main()
{
  vpColVector v(3);
  for (unsigned int i=0; i<v.size(); i++)
    v[i] = i;
  std::cout << "v = "; v.maplePrint(std::cout);
}
  \endcode
  produces this output:
  \code
v = ([
[0, ],
[1, ],
[2, ],
])
  \endcode
  that could be copy/paste in Maple.
*/
std::ostream &vpColVector::maplePrint(std::ostream &os) const
{
  os << "([ " << std::endl;
  for (unsigned int i = 0; i < this->getRows(); ++i) {
    os << "[";
    os << (*this)[i] << ", ";
    os << "]," << std::endl;
  }
  os << "])" << std::endl;
  return os;
};

/*!
  Print using Matlab syntax, to copy/paste in Matlab later.

  The following code
  \code
#include <visp3/core/vpColVector.h>

int main()
{
  vpColVector v(3);
  for (unsigned int i=0; i<v.size(); i++)
    v[i] = i;
  std::cout << "v = "; v.matlabPrint(std::cout);
}
  \endcode
  produces this output:
  \code
v = [ 0, ;
1, ;
2, ]
  \endcode
  that could be copy/paste in Matlab:
  \code
>> v = [ 0, ;
1, ;
2, ]

v =

    0
    1
    2

>>
  \endcode
*/
std::ostream &vpColVector::matlabPrint(std::ostream &os) const
{
  os << "[ ";
  for (unsigned int i = 0; i < this->getRows(); ++i) {
    os << (*this)[i] << ", ";
    if (this->getRows() != i + 1) {
      os << ";" << std::endl;
    } else {
      os << "]" << std::endl;
    }
  }
  return os;
};

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
  \deprecated You should rather use insert(unsigned int, const vpColVector &).

  Insert column vector \e v at the given position \e r in the current column
  vector.

  \warning Throw vpMatrixException::incorrectMatrixSizeError if the
  dimensions of the matrices do not allow the operation.

  \param v : The column vector to insert.
  \param r : The index of the row to begin to insert data.
  \param c : Not used.

 */
void vpColVector::insert(const vpColVector &v, const unsigned int r, const unsigned int c)
{
  (void)c;
  insert(r, v);
}
#endif // defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
