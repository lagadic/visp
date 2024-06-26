/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
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
 * See https://visp.inria.fr for more information.
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
 */

/*!
  \file vpRowVector.cpp
  \brief Definition of vpRowVector class member
*/

#include <assert.h>
#include <cmath>
#include <sstream>
#include <stdlib.h>
#include <string.h>

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRowVector.h>

#if defined(VISP_HAVE_SIMDLIB)
#include <Simd/SimdLib.h>
#endif

BEGIN_VISP_NAMESPACE
//! Copy operator.   Allow operation such as A = v
vpRowVector &vpRowVector::operator=(const vpRowVector &v)
{
  unsigned int k = v.colNum;
  if (colNum != k) {
    try {
      resize(k);
    }
    catch (...) {
      throw;
    }
  }

  memcpy(data, v.data, colNum * sizeof(double));

  return *this;
}

/*!
  Initialize a row vector from a 1-by-n size matrix.
  \warning  Handled with care m should be a 1 column matrix.

  \exception vpException::dimensionError If the matrix is not a 1-by-n
  dimension matrix.
*/
vpRowVector &vpRowVector::operator=(const vpMatrix &M)
{
  if (M.getRows() != 1) {
    throw(vpException(vpException::dimensionError, "Cannot initialize a (1x%d) row vector from a (%dx%d) matrix",
                      M.getCols(), M.getRows(), M.getCols()));
  }

  if (M.getCols() != colNum) {
    resize(M.getCols());
  }

  memcpy(data, M.data, colNum * sizeof(double));
  return *this;
}

/*!
  Initialize a row vector from a standard vector of double.
*/
vpRowVector &vpRowVector::operator=(const std::vector<double> &v)
{
  size_t v_size = v.size();
  resize(v_size);
  for (size_t i = 0; i < v_size; ++i) {
    (*this)[i] = v[i];
  }
  return *this;
}
/*!
  Initialize a row vector from a standard vector of double.
*/
vpRowVector &vpRowVector::operator=(const std::vector<float> &v)
{
  size_t v_size = v.size();
  resize(v_size);
  for (size_t i = 0; i < v_size; ++i) {
    (*this)[i] = static_cast<float>(v[i]);
  }
  return *this;
}

//! Initialize each element of the vector with \e x.
vpRowVector &vpRowVector::operator=(double x)
{
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      rowPtrs[i][j] = x;
    }
  }
  return *this;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
vpRowVector &vpRowVector::operator=(vpRowVector &&other)
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
    other.rowPtrs = nullptr;
    other.dsize = 0;
    other.data = nullptr;
  }

  return *this;
}

/*!
  Set vector elements from a list of double.
  \code
  #include <visp3/core/vpRowVector.cpp>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRowVector r;
    r = {1, 2, 3};
    std::cout << "r: " << r << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  r: 1  2  3
  \endcode
  \sa operator<<()
*/
vpRowVector &vpRowVector::operator=(const std::initializer_list<double> &list)
{
  resize(1, static_cast<unsigned int>(list.size()), false);
  std::copy(list.begin(), list.end(), data);
  return *this;
}
#endif

bool vpRowVector::operator==(const vpRowVector &v) const
{
  if ((colNum != v.colNum) || (rowNum != v.rowNum) /* should not happen */) {
    return false;
  }

  for (unsigned int i = 0; i < colNum; ++i) {
    if (!vpMath::equal(data[i], v.data[i], std::numeric_limits<double>::epsilon())) {
      return false;
    }
  }

  return true;
}

bool vpRowVector::operator!=(const vpRowVector &v) const { return !(*this == v); }

/*!

  Multiply a row vector by a column vector.

  \param x : Column vector.

  \warning The number of elements of the two vectors must be equal.

  \exception vpException::dimensionError : If the number of elements of the
  two vectors is not the same.

  \return A scalar.

*/
double vpRowVector::operator*(const vpColVector &x) const
{
  unsigned int nelements = x.getRows();
  if (getCols() != nelements) {
    throw(vpException(vpException::dimensionError, "Cannot multiply (1x%d) row vector by (%dx1) column vector", colNum,
                      x.getRows()));
  }

  double scalar = 0.0;

  for (unsigned int i = 0; i < nelements; ++i) {
    scalar += (*this)[i] * x[i];
  }
  return scalar;
}
/*!

  Multiply a row vector by a matrix.

  \param M : Matrix.

  \warning The number of elements of the row vector must be equal to the
  number of rows of the matrix.

  \exception vpException::dimensionError If the number of elements of the
  row vector is not equal to the number of rows of the matrix.

  \return The resulting row vector.

*/
vpRowVector vpRowVector::operator*(const vpMatrix &M) const
{
  vpRowVector c(M.getCols());

  if (colNum != M.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot multiply (1x%d) row vector by (%dx%d) matrix", colNum,
                      M.getRows(), M.getCols()));
  }

  c = 0.0;

  for (unsigned int i = 0; i < colNum; ++i) {
    double bi = data[i]; // optimization em 5/12/2006
    unsigned int m_cols = M.getCols();
    for (unsigned int j = 0; j < m_cols; ++j) {
      c[j] += bi * M[i][j];
    }
  }

  return c;
}

/*!
  Operator that allows to multiply each element of a row vector by a scalar.

  \param x : The scalar.

  \return The row vector multiplied by the scalar. The current
  row vector (*this) is unchanged.

  \code
  vpRowVector v(3);
  v[0] = 1;
  v[1] = 2;
  v[2] = 3;

  vpRowVector w = v * 3;
  // v is unchanged
  // w is now equal to : [3 6 9]
  \endcode
*/
vpRowVector vpRowVector::operator*(double x) const
{
  vpRowVector v(colNum);

  double *vd = v.data;
  double *d = data;

  for (unsigned int i = 0; i < colNum; ++i) {
    *(vd++) = (*d++) * x;
  }
  return v;
}

/*!
  Operator that allows to multiply each element of a row vector by a scalar.

  \param x : The scalar.

  \return The row vector multiplied by the scalar.

  \code
  vpRowVector v(3);
  v[0] = 1;
  v[1] = 2;
  v[2] = 3;

  v *= 3;
  // v is now equal to : [3 6 9]
  \endcode
*/
vpRowVector &vpRowVector::operator*=(double x)
{
  for (unsigned int i = 0; i < colNum; ++i) {
    (*this)[i] *= x;
  }
  return (*this);
}

/*!
  Operator that allows to divide each element of a row vector by a scalar.

  \param x : The scalar.

  \return The row vector divided by the scalar. The current
  row vector (*this) is unchanged.

  \code
  vpRowVector v(3);
  v[0] = 8;
  v[1] = 4;
  v[2] = 2;

  vpRowVector w = v / 2;
  // v is equal to : [8 4 2]
  // w is equal to : [4 2 1]
  \endcode
*/
vpRowVector vpRowVector::operator/(double x) const
{
  vpRowVector v(colNum);

  double *vd = v.data;
  double *d = data;

  for (unsigned int i = 0; i < colNum; ++i) {
    *(vd++) = (*d++) / x;
  }
  return v;
}

/*!
  Operator that allows to divide each element of a row vector by a scalar.

  \param x : The scalar.

  \return The row vector divided by the scalar.

  \code
  vpRowVector v(3);
  v[0] = 8;
  v[1] = 4;
  v[2] = 2;
  // v is equal to : [8 4 2]

  v /= 2;
  // v is equal to : [4 2 1]
  \endcode
*/
vpRowVector &vpRowVector::operator/=(double x)
{
  for (unsigned int i = 0; i < colNum; ++i) {
    (*this)[i] /= x;
  }
  return (*this);
}

/*!
   Operator that allows to negate all the row vector elements.

   \code
   vpRowVector r(3, 1);
   // r contains [1 1 1]
   vpRowVector v = -r;
   // v contains [-1 -1 -1]
   \endcode
 */
vpRowVector vpRowVector::operator-() const
{
  vpRowVector A(colNum);

  double *vd = A.data;
  double *d = data;

  for (unsigned int i = 0; i < colNum; ++i) {
    *(vd++) = -(*d++);
  }

  return A;
}

/*!
   Operator that allows to subtract to row vectors that have the same size.
   \exception vpException::dimensionError If the vectors size differ.
 */
vpRowVector vpRowVector::operator-(const vpRowVector &m) const
{
  if (getCols() != m.getCols()) {
    throw(vpException(vpException::dimensionError, "Cannot subtract (1x%d) row vector to (1x%d) row vector", getCols(),
                      m.getCols()));
  }

  vpRowVector v(colNum);

  for (unsigned int i = 0; i < colNum; ++i) {
    v[i] = (*this)[i] - m[i];
  }
  return v;
}

/*!
   Operator that allows to add to row vectors that have the same size.
   \exception vpException::dimensionError If the vectors size differ.
 */
vpRowVector vpRowVector::operator+(const vpRowVector &v) const
{
  if (getCols() != v.getCols()) {
    throw(vpException(vpException::dimensionError, "Cannot add (1x%d) row vector to (1x%d) row vector", getCols(),
                      v.getCols()));
  }

  vpRowVector r(colNum);

  for (unsigned int i = 0; i < colNum; ++i) {
    r[i] = (*this)[i] + v[i];
  }
  return r;
}

/*!
   Operator that allows to add two row vectors that have the same size.
   \exception vpException::dimensionError If the size of the two vectors
   differ.
 */
vpRowVector &vpRowVector::operator+=(vpRowVector v)
{
  if (getCols() != v.getCols()) {
    throw(vpException(vpException::dimensionError, "Cannot add (1x%d) row vector to (1x%d) row vector", getCols(),
                      v.getCols()));
  }

  for (unsigned int i = 0; i < colNum; ++i) {
    (*this)[i] += v[i];
  }
  return (*this);
}

/*!
   Operator that allows to subtract two row vectors that have the same size.
   \exception vpException::dimensionError If the size of the two vectors
   differ.
 */
vpRowVector &vpRowVector::operator-=(vpRowVector v)
{
  if (getCols() != v.getCols()) {
    throw(vpException(vpException::dimensionError, "Cannot subtract (1x%d) row vector to (1x%d) row vector", getCols(),
                      v.getCols()));
  }

  for (unsigned int i = 0; i < colNum; ++i) {
    (*this)[i] -= v[i];
  }
  return (*this);
}

/*!
  Copy operator.
  Allows operation such as A << v
  \code
  #include <visp3/core/vpRowVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRowVector A, B(5);
    for (unsigned int i=0; i<B.size(); i++)
      B[i] = i;
    A << B;
    std::cout << "A: " << A << std::endl;
  }
  \endcode
  In row vector A we get:
  \code
A: 0  1  2  3  4
  \endcode

  */
vpRowVector &vpRowVector::operator<<(const vpRowVector &v)
{
  *this = v;
  return *this;
}

vpRowVector &vpRowVector::operator<<(double val)
{
  resize(1, false);
  data[0] = val;
  return *this;
}

vpRowVector &vpRowVector::operator,(double val)
{
  resize(colNum + 1, false);
  data[colNum - 1] = val;
  return *this;
}

/*!
  Transpose the row vector. The resulting vector becomes a column vector.
*/
vpColVector vpRowVector::t() const
{
  vpColVector v(colNum);
  memcpy(v.data, data, colNum * sizeof(double));
  return v;
}

/*!
  Transpose the row vector. The resulting vector becomes a column vector.
  \sa t()
*/
vpColVector vpRowVector::transpose() const { return t(); }
/*!
  Transpose the row vector. The resulting vector \e v becomes a column vector.
  \sa t()
*/
void vpRowVector::transpose(vpColVector &v) const { v = t(); }

/*!
   Constructor that creates a row vector corresponding to row \e i
   of matrix \e M.
 */
vpRowVector::vpRowVector(const vpMatrix &M, unsigned int i) : vpArray2D<double>(1, M.getCols())
{
  unsigned int m_cols = M.getCols();
  for (unsigned int j = 0; j < m_cols; ++j) {
    (*this)[j] = M[i][j];
  }
}

/*!
   Constructor that creates a row vector from a 1-by-n matrix \e M.

   \exception vpException::dimensionError If the matrix is not a 1-by-n
   matrix.
 */
vpRowVector::vpRowVector(const vpMatrix &M) : vpArray2D<double>(1, M.getCols())
{
  if (M.getRows() != 1) {
    throw(vpException(vpException::dimensionError, "Cannot construct a (1x%d) row vector from a (%dx%d) matrix",
                      M.getCols(), M.getRows(), M.getCols()));
  }
  unsigned int m_cols = M.getCols();
  for (unsigned int j = 0; j < m_cols; ++j) {
    (*this)[j] = M[0][j];
  }
}

/*!
   Constructor that creates a row vector from a std vector of double.
 */
vpRowVector::vpRowVector(const std::vector<double> &v) : vpArray2D<double>(1, static_cast<unsigned int>(v.size()))
{
  size_t v_size = v.size();
  for (size_t j = 0; j < v_size; ++j) {
    (*this)[j] = v[j];
  }
}
/*!
   Constructor that creates a row vector from a std vector of float.
 */
vpRowVector::vpRowVector(const std::vector<float> &v) : vpArray2D<double>(1, static_cast<unsigned int>(v.size()))
{
  size_t v_size = v.size();
  for (size_t j = 0; j < v_size; ++j) {
    (*this)[j] = static_cast<double>(v[j]);
  }
}

/*!
  Construct a row vector from a part of an input row vector \e v.

  \param v : Input row vector used for initialization.
  \param c : column index in \e v that corresponds to the first element of the
  row vector to construct.
  \param ncols : Number of columns of the constructed row vector.

  The sub-vector starting from v[c] element and ending on v[c+ncols-1] element
  is used to initialize the constructed row vector.

  \sa init()
*/
vpRowVector::vpRowVector(const vpRowVector &v, unsigned int c, unsigned int ncols) : vpArray2D<double>(1, ncols)
{
  init(v, c, ncols);
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
vpRowVector::vpRowVector(vpRowVector &&v) : vpArray2D<double>()
{
  rowNum = v.rowNum;
  colNum = v.colNum;
  rowPtrs = v.rowPtrs;
  dsize = v.dsize;
  data = v.data;

  v.rowNum = 0;
  v.colNum = 0;
  v.rowPtrs = nullptr;
  v.dsize = 0;
  v.data = nullptr;
}
#endif

/*!
  Normalize the vector given as input parameter and return the normalized
  vector:

  \f[
  {\bf x} = \frac{{\bf x}}{\sqrt{\sum_{i=1}^{n}x^2_i}}
  \f]
  where \f$x_i\f$ is an element of the row vector \f$\bf x\f$.
*/
vpRowVector &vpRowVector::normalize(vpRowVector &x) const
{
  x = x / sqrt(x.sumSquare());

  return x;
}

/*!
  Normalise the vector modifying the vector as:

  \f[
  {\bf x} = \frac{{\bf x}}{\sqrt{\sum_{i=1}^{n}x^2_i}}
  \f]
  where \f$x_i\f$ is an element of the row vector \f$\bf x\f$.
*/
vpRowVector &vpRowVector::normalize()
{
  double sum_square = sumSquare();
  if (std::fabs(sum_square) > std::numeric_limits<double>::epsilon()) {
    *this /= sqrt(sum_square);
  }

  // If sum = 0, we have a nul vector. So we return just.
  return *this;
}

/*!
  Reshape the row vector in a matrix.
  \param nrows : number of rows of the matrix.
  \param ncols : number of columns of the matrix.
  \return The resulting matrix.

  \exception vpException::dimensionError If the matrix and the row vector have
  not the same size.

  \sa reshape(vpMatrix &, const unsigned int &, const unsigned int &)
*/
vpMatrix vpRowVector::reshape(unsigned int nrows, unsigned int ncols)
{
  vpMatrix M(nrows, ncols);
  reshape(M, nrows, ncols);
  return M;
}

/*!
  Reshape the row vector in a matrix.
  \param M : the reshaped matrix.
  \param nrows : number of rows of the matrix.
  \param ncols : number of columns of the matrix.

  \exception vpException::dimensionError If the matrix and the row vector have
  not the same size.

  The following example shows how to use this method.
  \code
  #include <visp3/core/vpRowVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    int var=0;
    vpMatrix mat(3, 4);
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 4; j++)
            mat[i][j] = ++var;
    std::cout << "mat: \n" << mat << std::endl;

    vpRowVector row = mat.stackRows();
    std::cout << "row vector: " << row << std::endl;

    vpMatrix remat = row.reshape(3, 4);
    std::cout << "remat: \n" << remat << std::endl;
  }
  \endcode

  If you run the previous example, you get:
  \code
  mat:
  1  2  3  4
  5  6  7  8
  9  10  11  12
  row vector: 1  2  3  4  5  6  7  8  9  10  11  12
  remat:
  1  2  3  4
  5  6  7  8
  9  10  11  12
  \endcode
*/
void vpRowVector::reshape(vpMatrix &M, const unsigned int &nrows, const unsigned int &ncols)
{
  if (dsize != (nrows * ncols)) {
    throw(vpException(vpException::dimensionError, "Cannot reshape (1x%d) row vector in (%dx%d) matrix", colNum,
                      M.getRows(), M.getCols()));
  }
  try {
    if ((M.getRows() != nrows) || (M.getCols() != ncols)) {
      M.resize(nrows, ncols);
    }
  }
  catch (...) {
    throw;
  }
  for (unsigned int i = 0; i < nrows; ++i) {
    for (unsigned int j = 0; j < ncols; ++j) {
      M[i][j] = data[(i * ncols) + j];
    }
  }
}

/*!
  Insert a row vector.
  \param i : Index of the first element to introduce. This index starts from 0.
  \param v : Row vector to insert.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpRowVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRowVector v(4);
    for (unsigned int i=0; i < v.size(); i++)
      v[i] = i;
    std::cout << "v: " << v << std::endl;

    vpRowVector w(2);
    for (unsigned int i=0; i < w.size(); i++)
      w[i] = i+10;
    std::cout << "w: " << w << std::endl;

    v.insert(1, w);
    std::cout << "v: " << v << std::endl;
  }
  \endcode
  It produces the following output:
  \code
  v: 0  1  2  3
  w: 10  11
  v: 0  10  11  3
  \endcode
 */
void vpRowVector::insert(unsigned int i, const vpRowVector &v)
{
  if ((i + v.size()) > this->size()) {
    throw(vpException(vpException::dimensionError,
                      "Unable to insert (1x%d) row vector in (1x%d) row "
                      "vector at position (%d)",
                      v.getCols(), colNum, i));
  }
  unsigned int v_size = v.size();
  for (unsigned int j = 0; j < v_size; ++j) {
    (*this)[i + j] = v[j];
  }
}

/*!
 * Converts the vpRowVector to a std::vector.
 * \return The corresponding std::vector<double>.
 */
std::vector<double> vpRowVector::toStdVector() const
{
  std::vector<double> v(this->size());

  unsigned int this_size = this->size();
  for (unsigned int i = 0; i < this_size; ++i) {
    v[i] = data[i];
  }
  return v;
}

/*!
  Stack row vector with a new element at the end of the vector.

  \param d : Element to stack to the existing one.

  \code
  vpRowVector v(3, 1);
  // v is equal to [1 1 1]
  v.stack(-2);
  // v is equal to [1 1 1 -2]
  \endcode

  \sa stack(const vpRowVector &, const vpRowVector &)
  \sa stack(const vpRowVector &, const vpRowVector &, vpRowVector &)

*/
void vpRowVector::stack(double d)
{
  this->resize(colNum + 1, false);
  (*this)[colNum - 1] = d;
}

/*!
  Stack row vectors.

  \param v : Vector to stack to the existing one.

  \code
  vpRowVector v1(3, 1);
  // v1 is equal to [1 1 1]
  vpRowVector v2(2, 3);
  // v2 is equal to [3 3]
  v1.stack(v2);
  // v1 is equal to [1 1 1 3 3]
  \endcode

  \sa stack(const vpRowVector &, const double &)
  \sa stack(const vpRowVector &, const vpRowVector &)
  \sa stack(const vpRowVector &, const vpRowVector &, vpRowVector &)

*/
void vpRowVector::stack(const vpRowVector &v) { *this = vpRowVector::stack(*this, v); }

/*!
  Stack row vectors.

  \param A : Initial vector.
  \param B : Vector to stack at the end of A.
  \return Stacked vector \f$[A B]\f$.

  \code
  vpRowVector r1(3, 1);
  // r1 is equal to [1 1 1]
  vpRowVector r2(2, 3);
  // r2 is equal to [3 3]
  vpRowVector v;
  v = vpRowVector::stack(r1, r2);
  // v is equal to [1 1 1 3 3]
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
  vpRowVector r1(3, 1);
  // r1 is equal to [1 1 1]
  vpRowVector r2(2, 3);
  // r2 is equal to [3 3]
  vpRowVector v;
  vpRowVector::stack(r1, r2, v);
  // v is equal to [1 1 1 3 3]
  \endcode

  \sa stack(const vpRowVector &)
  \sa stack(const vpRowVector &, const vpRowVector &)
*/
void vpRowVector::stack(const vpRowVector &A, const vpRowVector &B, vpRowVector &C)
{
  unsigned int nrA = A.getCols();
  unsigned int nrB = B.getCols();

  if ((nrA == 0) && (nrB == 0)) {
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

  for (unsigned int i = 0; i < nrA; ++i) {
    C[i] = A[i];
  }

  for (unsigned int i = 0; i < nrB; ++i) {
    C[nrA + i] = B[i];
  }
}

/*!
  Compute the mean value of all the elements of the vector.
*/
double vpRowVector::mean(const vpRowVector &v)
{
  if ((v.data == nullptr) || (v.size() == 0)) {
    throw(vpException(vpException::dimensionError, "Cannot compute mean value of an empty row vector"));
  }

  double mean = 0;
  double *vd = v.data;
  unsigned int v_col = v.getCols();
  for (unsigned int i = 0; i < v_col; ++i) {
    mean += *(vd++);
  }

  return mean / v.getCols();
}

/*!
  Compute the median value of all the elements of the vector.
*/
double vpRowVector::median(const vpRowVector &v)
{
  if ((v.data == nullptr) || (v.size() == 0)) {
    throw(vpException(vpException::dimensionError, "Cannot compute mean value of an empty row vector"));
  }

  std::vector<double> vectorOfDoubles(v.data, v.data + v.colNum);

  return vpMath::getMedian(vectorOfDoubles);
}

/*!
  Compute the standard deviation value of all the elements of the vector.
*/
double vpRowVector::stdev(const vpRowVector &v, bool useBesselCorrection)
{
  if ((v.data == nullptr) || (v.size() == 0)) {
    throw(vpException(vpException::dimensionError, "Cannot compute mean value of an empty row vector"));
  }

  double mean_value = mean(v);
  double sum_squared_diff = 0.0;
  unsigned int v_size = v.size();
  for (unsigned int i = 0; i < v_size; ++i) {
    sum_squared_diff += (v[i] - mean_value) * (v[i] - mean_value);
  }

  double divisor = static_cast<double>(v.size());
  if (useBesselCorrection && (v.size() > 1)) {
    divisor = divisor - 1;
  }

  return std::sqrt(sum_squared_diff / divisor);
}

/*!

  Pretty print a row vector. The data are tabulated.
  The common widths before and after the decimal point
  are set with respect to the parameter maxlen.

  \param s Stream used for the printing.

  \param length The suggested width of each row vector element.
  The actual width grows in order to accommodate the whole integral part,
  and shrinks if the whole extent is not needed for all the numbers.
  \param intro The introduction which is printed before the vector.
  Can be set to zero (or omitted), in which case
  the introduction is not printed.

  \return Returns the common total width for all vector elements.

  \sa std::ostream &operator<<(std::ostream &s, const vpArray2D<Type> &A)
*/
int vpRowVector::print(std::ostream &s, unsigned int length, char const *intro) const
{
  typedef std::string::size_type size_type;

  unsigned int m = 1;
  unsigned int n = getCols();

  std::vector<std::string> values(m * n);
  std::ostringstream oss;
  std::ostringstream ossFixed;
  std::ios_base::fmtflags original_flags = oss.flags();

  // ossFixed <<std::fixed
  ossFixed.setf(std::ios::fixed, std::ios::floatfield);

  size_type maxBefore = 0; // the length of the integral part
  size_type maxAfter = 0;  // number of decimals plus
  // one place for the decimal point
  for (unsigned int j = 0; j < n; ++j) {
    oss.str("");
    oss << (*this)[j];
    if (oss.str().find("e") != std::string::npos) {
      ossFixed.str("");
      ossFixed << (*this)[j];
      oss.str(ossFixed.str());
    }

    values[j] = oss.str();
    size_type thislen = values[j].size();
    size_type p = values[j].find('.');

    if (p == std::string::npos) {
      maxBefore = vpMath::maximum(maxBefore, thislen);
      // maxAfter remains the same
    }
    else {
      maxBefore = vpMath::maximum(maxBefore, p);
      maxAfter = vpMath::maximum(maxAfter, thislen - p - 1);
    }
  }

  size_type totalLength = length;
  // increase totalLength according to maxBefore
  totalLength = vpMath::maximum(totalLength, maxBefore);
  // decrease maxAfter according to totalLength
  maxAfter = std::min<size_type>(maxAfter, totalLength - maxBefore);
  if (maxAfter == 1) {
    maxAfter = 0;
  }

  // the following line is useful for debugging
  // std::cerr <<totalLength <<" " <<maxBefore <<" " <<maxAfter <<"\n";

  if (intro) {
    s << intro;
  }
  s << "[" << m << "," << n << "]=\n";

  s << "  ";
  for (unsigned int j = 0; j < n; ++j) {
    size_type p = values[j].find('.');
    s.setf(std::ios::right, std::ios::adjustfield);
    s.width(static_cast<std::streamsize>(maxBefore));
    s << values[j].substr(0, p).c_str();

    if (maxAfter > 0) {
      s.setf(std::ios::left, std::ios::adjustfield);
      if (p != std::string::npos) {
        s.width(static_cast<std::streamsize>(maxAfter));
        s << values[j].substr(p, maxAfter).c_str();
      }
      else {
        assert(maxAfter > 1);
        s.width(static_cast<std::streamsize> (maxAfter));
        s << ".0";
      }
    }

    s << ' ';
  }
  s << std::endl;

  s.flags(original_flags); // restore s to standard state

  return static_cast<int>(maxBefore + maxAfter);
}

/*!
  Return the sum of all the elements \f$v_{i}\f$ of the row vector v(n).

  \return The sum square value: \f$\sum_{j=0}^{n} v_j\f$.
 */
double vpRowVector::sum() const
{
  double sum = 0.0;

  for (unsigned int j = 0; j < colNum; ++j) {
    sum += rowPtrs[0][j];
  }

  return sum;
}

/*!
  Return the sum square of all the elements \f$v_{i}\f$ of the row vector
  v(n).

  \return The sum square value: \f$\sum_{j=0}^{n} v_j^{2}\f$.
 */
double vpRowVector::sumSquare() const
{
  double sum_square = 0.0;

  for (unsigned int j = 0; j < colNum; ++j) {
    double x = rowPtrs[0][j];
    sum_square += x * x;
  }

  return sum_square;
}

/*!
  Compute and return the Frobenius norm \f$ ||v|| = \sqrt{ \sum{v_{i}^2}} \f$.

  \return The Frobenius norm if the vector is initialized, 0 otherwise.
 */
double vpRowVector::frobeniusNorm() const
{
  double norm = sumSquare();

  return sqrt(norm);
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
  \deprecated This function is deprecated. You should rather use frobeniusNorm().

  Compute and return the Euclidean norm also called Frobenius norm \f$ ||v|| = \sqrt{ \sum{v_{i}^2}} \f$.

  \return The Euclidean norm if the vector is initialized, 0 otherwise.

  \sa frobeniusNorm()
*/
double vpRowVector::euclideanNorm() const { return frobeniusNorm(); }
#endif

/*!
  Initialize the row vector from a part of an input row vector \e v.

  \param v : Input row vector used for initialization.
  \param c : column index in \e v that corresponds to the first element of the
  row vector to construct. \param ncols : Number of columns of the constructed
  row vector.

  The sub-vector starting from v[c] element and ending on v[c+ncols-1] element
  is used to initialize the constructed row vector.

  The following code shows how to use this function:
  \code
  #include <visp3/core/vpRowVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRowVector v(4);
    int val = 0;
    for(size_t i=0; i<v.getCols(); i++) {
      v[i] = val++;
    }
    std::cout << "v: " << v << std::endl;

    vpRowVector w;
    w.init(v, 1, 2);
    std::cout << "w: " << w << std::endl;
  }
  \endcode
  It produces the following output:
  \code
  v: 0 1 2 3
  w: 1 2
  \endcode
 */
void vpRowVector::init(const vpRowVector &v, unsigned int c, unsigned int ncols)
{
  unsigned int cncols = c + ncols;

  if (cncols > v.getCols()) {
    throw(vpException(vpException::dimensionError, "Bad column dimension (%d > %d) used to initialize vpRowVector",
                      cncols, v.getCols()));
  }
  resize(ncols);
  if (this->rowPtrs == nullptr) { // Fix coverity scan: explicit null dereferenced
    return;                  // Noting to do
  }
  for (unsigned int i = 0; i < ncols; ++i) {
    (*this)[i] = v[i + c];
  }
}

/*!
  Print to be used as part of a C++ code later.

  \param os : the stream to be printed in.
  \param matrixName : name of the row vector, "A" by default.
  \param octet : if false, print using double, if true, print byte per byte
  each bytes of the double array.

  The following code shows how to use this function:
  \code
  #include <visp3/core/vpRowVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRowVector r(3);
    for (unsigned int i=0; i<r.size(); i++)
      r[i] = i;

    r.cppPrint(std::cout, "r");
  }
  \endcode
  It produces the following output that could be copy/paste in a C++ code:
  \code
  vpRowVector r (3);
  r[0] = 0;
  r[1] = 1;
  r[2] = 2;
  \endcode
*/
std::ostream &vpRowVector::cppPrint(std::ostream &os, const std::string &matrixName, bool octet) const
{
  os << "vpRowVector " << matrixName << " (" << this->getCols() << "); " << std::endl;

  unsigned int this_cols = this->getCols();
  for (unsigned int j = 0; j < this_cols; ++j) {
    if (!octet) {
      os << matrixName << "[" << j << "] = " << (*this)[j] << "; " << std::endl;
    }
    else {
      for (unsigned int k = 0; k < sizeof(double); ++k) {
        os << "((unsigned char*)&(" << matrixName << "[" << j << "]) )[" << k << "] = 0x" << std::hex
          << static_cast<unsigned int>(((unsigned char *)&((*this)[j]))[k]) << "; " << std::endl;
      }
    }
  }
  std::cout << std::endl;
  return os;
}

/*!
  Print/save a row vector in csv format.

  The following code
  \code
  #include <visp3/core/vpRowVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    std::ofstream ofs("log.csv", std::ofstream::out);
    vpRowVector r(3);
    for (unsigned int i=0; i<r.size(); i++)
      r[i] = i;

    r.csvPrint(ofs);

    ofs.close();
  }
  \endcode
  produces log.csv file that contains:
  \code
  0, 1, 2
  \endcode
*/
std::ostream &vpRowVector::csvPrint(std::ostream &os) const
{
  unsigned int this_cols = this->getCols();
  for (unsigned int j = 0; j < this_cols; ++j) {
    os << (*this)[j];
    if (!(j == (this->getCols() - 1))) {
      os << ", ";
    }
  }
  os << std::endl;
  return os;
}

/*!
  Print using Maple syntax, to copy/paste in Maple later.

  The following code
  \code
  #include <visp3/core/vpRowVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRowVector r(3);
    for (unsigned int i=0; i<r.size(); i++)
      r[i] = i;
    std::cout << "r = "; r.maplePrint(std::cout);
  }
  \endcode
  produces this output:
  \code
  r = ([
  [0, 1, 2, ],
  ])
  \endcode
  that could be copy/paste in Maple.
*/
std::ostream &vpRowVector::maplePrint(std::ostream &os) const
{
  os << "([ " << std::endl;
  os << "[";
  unsigned int this_cols = this->getCols();
  for (unsigned int j = 0; j < this_cols; ++j) {
    os << (*this)[j] << ", ";
  }
  os << "]," << std::endl;
  os << "])" << std::endl;
  return os;
}

/*!
  Print using Matlab syntax, to copy/paste in Matlab later.

  The following code
  \code
  #include <visp3/core/vpRowVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRowVector r(3);
    for (unsigned int i=0; i<r.size(); i++)
      r[i] = i;
    std::cout << "r = "; r.matlabPrint(std::cout);
  }
  \endcode
  produces this output:
  \code
  r = [ 0, 1, 2, ]
  \endcode
  that could be copy/paste in Matlab:
  \code
  >> r = [ 0, 1, 2, ]

  r =

      0   1   2

  >>
  \endcode
*/
std::ostream &vpRowVector::matlabPrint(std::ostream &os) const
{
  os << "[ ";

  unsigned int this_cols = this->getCols();
  for (unsigned int j = 0; j < this_cols; ++j) {
    os << (*this)[j] << ", ";
  }
  os << "]" << std::endl;
  return os;
}

/*!
  Allows to multiply a scalar by row vector.
*/
vpRowVector operator*(const double &x, const vpRowVector &v)
{
  vpRowVector vout;
  vout = v * x;
  return vout;
}

vpRowVector vpRowVector::hadamard(const vpRowVector &v) const
{
  if ((v.getRows() != rowNum) || (v.getCols() != colNum)) {
    throw(vpException(vpException::dimensionError, "Hadamard product: bad dimensions!"));
  }

  vpRowVector out;
  out.resize(colNum, false);
#if defined(VISP_HAVE_SIMDLIB)
  SimdVectorHadamard(data, v.data, colNum, out.data);
#else
  for (unsigned int i = 0; i < dsize; ++i) {
    out.data[i] = data[i] * v.data[i];
  }
#endif
  return out;
}
END_VISP_NAMESPACE
