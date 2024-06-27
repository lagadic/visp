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
 * Matrix manipulation.
 */

/*!
  \file vpMatrix.cpp
  \brief Definition of the vpMatrix class
*/

#include <algorithm>
#include <assert.h>
#include <cmath> // std::fabs
#include <fstream>
#include <limits> // numeric_limits
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <string>
#include <vector>

#include <visp3/core/vpCPUFeatures.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpTranslationVector.h>

#if defined(VISP_HAVE_SIMDLIB)
#include <Simd/SimdLib.h>
#endif

#ifdef VISP_HAVE_LAPACK
#ifdef VISP_HAVE_GSL
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_math.h>
#elif defined(VISP_HAVE_MKL)
#include <mkl.h>
#endif
#endif

BEGIN_VISP_NAMESPACE

#ifdef VISP_HAVE_LAPACK
#ifdef VISP_HAVE_GSL
#elif defined(VISP_HAVE_MKL)
typedef MKL_INT integer;

void vpMatrix::blas_dsyev(char jobz, char uplo, unsigned int n_, double *a_data, unsigned int lda_, double *w_data,
                          double *work_data, int lwork_, int &info_)
{
  MKL_INT n = static_cast<MKL_INT>(n_);
  MKL_INT lda = static_cast<MKL_INT>(lda_);
  MKL_INT lwork = static_cast<MKL_INT>(lwork_);
  MKL_INT info = static_cast<MKL_INT>(info_);

  dsyev(&jobz, &uplo, &n, a_data, &lda, w_data, work_data, &lwork, &info);
}

#else
#if defined(VISP_HAVE_LAPACK_BUILT_IN)
typedef long int integer;
#else
typedef int integer;
#endif
extern "C" integer dsyev_(char *jobz, char *uplo, integer *n, double *a, integer *lda, double *w, double *WORK,
                          integer *lwork, integer *info);

void vpMatrix::blas_dsyev(char jobz, char uplo, unsigned int n_, double *a_data, unsigned int lda_, double *w_data,
                          double *work_data, int lwork_, int &info_)
{
  integer n = static_cast<integer>(n_);
  integer lda = static_cast<integer>(lda_);
  integer lwork = static_cast<integer>(lwork_);
  integer info = static_cast<integer>(info_);

  dsyev_(&jobz, &uplo, &n, a_data, &lda, w_data, work_data, &lwork, &info);

  lwork_ = static_cast<int>(lwork);
  info_ = static_cast<int>(info);
}
#endif
#endif

#if !defined(VISP_USE_MSVC) || (defined(VISP_USE_MSVC) && !defined(VISP_BUILD_SHARED_LIBS))
const unsigned int vpMatrix::m_lapack_min_size_default = 0;
unsigned int vpMatrix::m_lapack_min_size = vpMatrix::m_lapack_min_size_default;
#endif

// Prototypes of specific functions
vpMatrix subblock(const vpMatrix &M, unsigned int col, unsigned int row);

/*!
  Construct a matrix as a sub-matrix of the input matrix \e M.
  \sa init(const vpMatrix &M, unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols)
*/
vpMatrix::vpMatrix(const vpMatrix &M, unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols)
  : vpArray2D<double>()
{
  if (((r + nrows) > M.rowNum) || ((c + ncols) > M.colNum)) {
    throw(vpException(vpException::dimensionError,
                      "Cannot construct a sub matrix (%dx%d) starting at "
                      "position (%d,%d) that is not contained in the "
                      "original matrix (%dx%d)",
                      nrows, ncols, r, c, M.rowNum, M.colNum));
  }

  init(M, r, c, nrows, ncols);
}

/*!
 * Create a matrix from a homogeneous matrix.
 * @param M : Homogeneous matrix.
 */
vpMatrix::vpMatrix(const vpHomogeneousMatrix &M)
{
  *this = M;
}

/*!
 * Create a matrix from a velocity twist matrix.
 * @param V : Velocity twist matrix.
 */
vpMatrix::vpMatrix(const vpVelocityTwistMatrix &V)
{
  *this = V;
}

/*!
 * Create a matrix from a force twist matrix.
 * @param F : Force twist matrix.
 */
vpMatrix::vpMatrix(const vpForceTwistMatrix &F)
{
  *this = F;
}

/*!
 * Create a matrix from a row vector.
 * @param R : Rotation matrix.
 */
vpMatrix::vpMatrix(const vpRotationMatrix &R)
{
  *this = R;
}

/*!
 * Create a matrix from a column vector.
 * @param v : Column vector.
 */
vpMatrix::vpMatrix(const vpColVector &v)
{
  *this = v;
}

/*!
 * Create a matrix from a row vector.
 * @param v : Row vector.
 */
vpMatrix::vpMatrix(const vpRowVector &v)
{
  *this = v;
}

/*!
 * Create a matrix from a row vector.
 * @param t : Translation vector.
 */
vpMatrix::vpMatrix(const vpTranslationVector &t)
{
  *this = t;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
vpMatrix::vpMatrix(vpMatrix &&A) : vpArray2D<double>()
{
  rowNum = A.rowNum;
  colNum = A.colNum;
  rowPtrs = A.rowPtrs;
  dsize = A.dsize;
  data = A.data;

  A.rowNum = 0;
  A.colNum = 0;
  A.rowPtrs = nullptr;
  A.dsize = 0;
  A.data = nullptr;
}

/*!
  Construct a matrix from a list of double values.
  \param list : List of double.

  The following code shows how to use this constructor to initialize a 2-by-3 matrix using reshape() function:

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  vpMatrix M( {-1, -2, -3, 4, 5.5, 6.0f} );
  M.reshape(2, 3);
  std::cout << "M:\n" << M << std::endl;
  }
  \endcode
  It produces the following output:
  \code
  M:
  -1  -2  -3
  4  5.5  6
  \endcode
*/
vpMatrix::vpMatrix(const std::initializer_list<double> &list) : vpArray2D<double>(list) { }

/*!
  Construct a matrix from a list of double values.
  \param ncols, nrows : Matrix size.
  \param list : List of double.

  The following code shows how to use this constructor to initialize a 2-by-3 matrix:
  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  vpMatrix M(2, 3, {-1, -2, -3, 4, 5.5, 6});
  std::cout << "M:\n" << M << std::endl;
  }
  \endcode
  It produces the following output:
  \code
  M:
  -1  -2  -3
  4  5.5  6
  \endcode
 */
vpMatrix::vpMatrix(unsigned int nrows, unsigned int ncols, const std::initializer_list<double> &list)
  : vpArray2D<double>(nrows, ncols, list)
{ }

/*!
  Construct a matrix from a list of double values.
  \param lists : List of double.
  The following code shows how to use this constructor to initialize a 2-by-3 matrix function:
  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
  vpMatrix M( { {-1, -2, -3}, {4, 5.5, 6} } );
  std::cout << "M:\n" << M << std::endl;
  }
  \endcode
  It produces the following output:
  \code
  M:
  -1  -2  -3
  4  5.5  6
  \endcode
 */
vpMatrix::vpMatrix(const std::initializer_list<std::initializer_list<double> > &lists) : vpArray2D<double>(lists) { }
#endif

/*!
  Initialize the matrix from a part of an input matrix \e M.

  \param M : Input matrix used for initialization.
  \param r : row index in matrix M.
  \param c : column index in matrix M.
  \param nrows : Number of rows of the matrix that should be initialized.
  \param ncols : Number of columns of the matrix that should be initialized.

  The sub-matrix starting from M[r][c] element and ending on
  M[r+nrows-1][c+ncols-1] element is used to initialize the matrix.

  The following code shows how to use this function:
  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix M(4,5);
    int val = 0;
    for(size_t i=0; i<M.getRows(); i++) {
      for(size_t j=0; j<M.getCols(); j++) {
        M[i][j] = val++;
      }
    }
    M.print(std::cout, 4, "M ");

    vpMatrix N;
    N.init(M, 0, 1, 2, 3);
    N.print(std::cout, 4, "N ");
  }
  \endcode
  It produces the following output:
  \code
  M [4,5]=
    0  1  2  3  4
    5  6  7  8  9
    10 11 12 13 14
    15 16 17 18 19
  N [2,3]=
    1 2 3
    6 7 8
  \endcode

  \sa extract()
 */
void vpMatrix::init(const vpMatrix &M, unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols)
{
  unsigned int rnrows = r + nrows;
  unsigned int cncols = c + ncols;

  if (rnrows > M.getRows()) {
    throw(vpException(vpException::dimensionError, "Bad row dimension (%d > %d) used to initialize vpMatrix", rnrows,
                      M.getRows()));
  }
  if (cncols > M.getCols()) {
    throw(vpException(vpException::dimensionError, "Bad column dimension (%d > %d) used to initialize vpMatrix", cncols,
                      M.getCols()));
  }
  resize(nrows, ncols, false, false);

  if (this->rowPtrs == nullptr) { // Fix coverity scan: explicit null dereferenced
    return;                  // Noting to do
  }
  for (unsigned int i = 0; i < nrows; ++i) {
    memcpy((*this)[i], &M[i + r][c], ncols * sizeof(double));
  }
}

/*!
  Extract a sub matrix from a matrix \e M.

  \param r : row index in matrix \e M.
  \param c : column index in matrix \e M.
  \param nrows : Number of rows of the matrix that should be extracted.
  \param ncols : Number of columns of the matrix that should be extracted.

  The following code shows how to use this function:
  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix M(4,5);
    int val = 0;
    for(size_t i=0; i<M.getRows(); i++) {
      for(size_t j=0; j<M.getCols(); j++) {
        M[i][j] = val++;
      }
    }
    M.print(std::cout, 4, "M ");
    vpMatrix N = M.extract(0, 1, 2, 3);
    N.print(std::cout, 4, "N ");
  }
  \endcode
  It produces the following output:
  \code
  M [4,5]=
    0  1  2  3  4
    5  6  7  8  9
    10 11 12 13 14
    15 16 17 18 19
  N [2,3]=
    1 2 3
    6 7 8
  \endcode

  \sa init(const vpMatrix &, unsigned int, unsigned int, unsigned int, unsigned int)
 */
vpMatrix vpMatrix::extract(unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols) const
{
  unsigned int rnrows = r + nrows;
  unsigned int cncols = c + ncols;

  if (rnrows > getRows()) {
    throw(vpException(vpException::dimensionError, "Bad row dimension (%d > %d) used to initialize vpMatrix", rnrows,
                      getRows()));
  }
  if (cncols > getCols()) {
    throw(vpException(vpException::dimensionError, "Bad column dimension (%d > %d) used to initialize vpMatrix", cncols,
                      getCols()));
  }

  vpMatrix M;
  M.resize(nrows, ncols, false, false);
  for (unsigned int i = 0; i < nrows; ++i) {
    memcpy(M[i], &(*this)[i + r][c], ncols * sizeof(double));
  }

  return M;
}

/*!
  Extract a column vector from a matrix.
  \warning All the indexes start from 0 in this function.
  \param j : Index of the column to extract. If col=0, the first column is extracted.
  \param i_begin : Index of the row that gives the location of the first element
  of the column vector to extract.
  \param column_size : Size of the column vector to extract.
  \return The extracted column vector.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(4,4);

    for(unsigned int i=0; i < A.getRows(); i++)
      for(unsigned int j=0; j < A.getCols(); j++)
        A[i][j] = i*A.getCols()+j;

    A.print(std::cout, 4);

    vpColVector cv = A.getCol(1, 1, 3);
    std::cout << "Column vector: \n" << cv << std::endl;
  }
  \endcode
  It produces the following output :
  \code
  [4, 4] =
  0  1  2  3
  4  5  6  7
  8  9 10 11
  12 13 14 15
  column vector :
  5
  9
  13
  \endcode
*/
vpColVector vpMatrix::getCol(unsigned int j, unsigned int i_begin, unsigned int column_size) const
{
  if (((i_begin + column_size) > getRows()) || (j >= getCols())) {
    throw(vpException(vpException::dimensionError, "Unable to extract column %u from the %ux%u matrix", j, getRows(),
                      getCols()));
  }
  vpColVector c(column_size);
  for (unsigned int i = 0; i < column_size; ++i) {
    c[i] = (*this)[i_begin + i][j];
  }
  return c;
}

/*!
  Extract a column vector from a matrix.
  \warning All the indexes start from 0 in this function.
  \param j : Index of the column to extract. If j=0, the first column is extracted.
  \return The extracted column vector.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(4,4);

    for (unsigned int i = 0; i < A.getRows(); i++)
      for (unsigned int j = 0; j < A.getCols(); j++)
        A[i][j] = i*A.getCols()+j;

    A.print(std::cout, 4);

    vpColVector cv = A.getCol(1);
    std::cout << "Column vector: \n" << cv << std::endl;
  }
  \endcode
    It produces the following output :
  \code
  [4, 4] =
  0  1  2  3
  4  5  6  7
  8  9 10 11
  12 13 14 15
  column vector :
  1
  5
  9
  13
  \endcode
  */
vpColVector vpMatrix::getCol(unsigned int j) const { return getCol(j, 0, rowNum); }

/*!
  Extract a row vector from a matrix.
  \warning All the indexes start from 0 in this function.
  \param i : Index of the row to extract. If i=0, the first row is extracted.
  \return The extracted row vector.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpMatrix.h>
  #include <visp3/core/vpRowVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(4,4);

    for(unsigned int i=0; i < A.getRows(); i++)
      for(unsigned int j=0; j < A.getCols(); j++)
        A[i][j] = i*A.getCols()+j;

    A.print(std::cout, 4);

    vpRowVector rv = A.getRow(1);
    std::cout << "Row vector: \n" << rv << std::endl;
  }
  \endcode
  It produces the following output :
  \code
  [4, 4] =
  0  1  2  3
  4  5  6  7
  8  9 10 11
  12 13 14 15
  Row vector :
  4  5  6  7
  \endcode
*/
vpRowVector vpMatrix::getRow(unsigned int i) const { return getRow(i, 0, colNum); }

/*!
  Extract a row vector from a matrix.
  \warning All the indexes start from 0 in this function.
  \param i : Index of the row to extract.If i = 0, the first row is extracted.
  \param j_begin : Index of the column that gives the location of the first
  element of the row vector to extract.
  \param row_size : Size of the row vector to extract.
  \return The extracted row vector.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpMatrix.h>
  #include <visp3/core/vpRowVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(4, 4);

    for (unsigned int i = 0; i < A.getRows(); i++)
      for (unsigned int j = 0; j < A.getCols(); j++)
        A[i][j] = i*A.getCols()+j;

    A.print(std::cout, 4);

    vpRowVector rv = A.getRow(1, 1, 3);
    std::cout << "Row vector: \n" << rv << std::endl;
  }
  \endcode
  It produces the following output :
  \code
  [4, 4] =
  0  1  2  3
  4  5  6  7
  8  9 10 11
  12 13 14 15
  Row vector :
  5  6  7
  \endcode
*/
vpRowVector vpMatrix::getRow(unsigned int i, unsigned int j_begin, unsigned int row_size) const
{
  if (((j_begin + row_size) > colNum) || (i >= rowNum)) {
    throw(vpException(vpException::dimensionError, "Unable to extract a row vector from the matrix"));
  }
  vpRowVector r(row_size);
  if ((r.data != nullptr) && (data != nullptr)) {
    memcpy(r.data, (*this)[i] + j_begin, row_size * sizeof(double));
  }

  return r;
}

/*!
  Extract a diagonal vector from a matrix.

  \return The diagonal of the matrix.

  \warning An empty vector is returned if the matrix is empty.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(3, 4);

    for (unsigned int i = 0; i < A.getRows(); i++)
      for (unsigned int j = 0; j < A.getCols(); j++)
        A[i][j] = i*A.getCols()+j;

    A.print(std::cout, 4);

    vpColVector diag = A.getDiag();
    std::cout << "Diag vector: \n" << diag.t() << std::endl;
  }
  \endcode
  It produces the following output :
  \code
  [3, 4] =
  0  1  2  3
  4  5  6  7
  8  9 10 11
  Diag vector :
  0  5  10
  \endcode
*/
vpColVector vpMatrix::getDiag() const
{
  unsigned int min_size = std::min<unsigned int>(rowNum, colNum);
  vpColVector diag;

  if (min_size > 0) {
    diag.resize(min_size, false);

    for (unsigned int i = 0; i < min_size; ++i) {
      diag[i] = (*this)[i][i];
    }
  }

  return diag;
}

/*!
  Insert matrix B in matrix A at the given position.

  \param A : Main matrix.
  \param B : Matrix to insert.
  \param r : Index of the row where to add the matrix.
  \param c : Index of the column where to add the matrix.
  \return Matrix with B insert in A.

  \warning Throw exception if the sizes of the matrices do not allow the
  insertion.
*/
vpMatrix vpMatrix::insert(const vpMatrix &A, const vpMatrix &B, unsigned int r, unsigned int c)
{
  vpArray2D<double> C;

  vpArray2D<double>::insert(A, B, C, r, c);

  return vpMatrix(C);
}

/*!
  \relates vpMatrix
  Insert matrix B in matrix A at the given position.

  \param A : Main matrix.
  \param B : Matrix to insert.
  \param C : Result matrix.
  \param r : Index of the row where to insert matrix B.
  \param c : Index of the column where to insert matrix B.

  \warning Throw exception if the sizes of the matrices do not
  allow the insertion.
*/
void vpMatrix::insert(const vpMatrix &A, const vpMatrix &B, vpMatrix &C, unsigned int r, unsigned int c)
{
  vpArray2D<double> C_array;

  vpArray2D<double>::insert(A, B, C_array, r, c);

  C = C_array;
}

/*!
  Juxtapose to matrices C = [ A B ].

  \f$ C = \left( \begin{array}{cc} A & B \end{array}\right)    \f$

  \param A : Left matrix.
  \param B : Right matrix.
  \return Juxtaposed matrix C = [ A B ]

  \warning A and B must have the same number of rows.
*/
vpMatrix vpMatrix::juxtaposeMatrices(const vpMatrix &A, const vpMatrix &B)
{
  vpMatrix C;

  juxtaposeMatrices(A, B, C);

  return C;
}

/*!
  \relates vpMatrix
  Juxtapose to matrices C = [ A B ].

  \f$ C = \left( \begin{array}{cc} A & B \end{array}\right)    \f$

  \param A : Left matrix.
  \param B : Right matrix.
  \param C : Juxtaposed matrix C = [ A B ]

  \warning A and B must have the same number of rows.
*/
void vpMatrix::juxtaposeMatrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
  unsigned int nca = A.getCols();
  unsigned int ncb = B.getCols();

  if (nca != 0) {
    if (A.getRows() != B.getRows()) {
      throw(vpException(vpException::dimensionError, "Cannot juxtapose (%dx%d) matrix with (%dx%d) matrix", A.getRows(),
                        A.getCols(), B.getRows(), B.getCols()));
    }
  }

  if ((B.getRows() == 0) || ((nca + ncb) == 0)) {
    std::cerr << "B.getRows() == 0 || nca+ncb == 0" << std::endl;
    return;
  }

  C.resize(B.getRows(), nca + ncb, false, false);

  C.insert(A, 0, 0);
  C.insert(B, 0, nca);
}

//--------------------------------------------------------------------
// Output
//--------------------------------------------------------------------

/*!

  Pretty print a matrix. The data are tabulated.
  The common widths before and after the decimal point
  are set with respect to the parameter `length`.

  \param s : Stream used for the printing.

  \param length : The suggested width of each matrix element.
  If needed, the used `length` grows in order to accommodate the whole integral part,
  and shrinks the decimal part to print only `length` digits.
  \param intro : The introduction which is printed before the matrix.
  Can be set to zero (or omitted), in which case
  the introduction is not printed.

  \return Returns the common total width for all matrix elements.

  \sa std::ostream &operator<<(std::ostream &s, const vpArray2D<Type> &A)
*/
int vpMatrix::print(std::ostream &s, unsigned int length, const std::string &intro) const
{
  typedef std::string::size_type size_type;

  unsigned int m = getRows();
  unsigned int n = getCols();

  std::vector<std::string> values(m * n);
  std::ostringstream oss;
  std::ostringstream ossFixed;
  std::ios_base::fmtflags original_flags = oss.flags();

  ossFixed.setf(std::ios::fixed, std::ios::floatfield);

  size_type maxBefore = 0; // the length of the integral part
  size_type maxAfter = 0;  // number of decimals plus
  // one place for the decimal point
  for (unsigned int i = 0; i < m; ++i) {
    for (unsigned int j = 0; j < n; ++j) {
      oss.str("");
      oss << (*this)[i][j];
      if (oss.str().find("e") != std::string::npos) {
        ossFixed.str("");
        ossFixed << (*this)[i][j];
        oss.str(ossFixed.str());
      }

      values[(i * n) + j] = oss.str();
      size_type thislen = values[(i * n) + j].size();
      size_type p = values[(i * n) + j].find('.');

      if (p == std::string::npos) {
        maxBefore = vpMath::maximum(maxBefore, thislen);
        // maxAfter remains the same
      }
      else {
        maxBefore = vpMath::maximum(maxBefore, p);
        maxAfter = vpMath::maximum(maxAfter, thislen - p);
      }
    }
  }

  size_type totalLength = length;
  // increase totalLength according to maxBefore
  totalLength = vpMath::maximum(totalLength, maxBefore);
  // decrease maxAfter according to totalLength
  maxAfter = std::min<size_type>(maxAfter, totalLength - maxBefore);

  if (!intro.empty()) {
    s << intro;
  }
  s << "[" << m << "," << n << "]=\n";

  for (unsigned int i = 0; i < m; ++i) {
    s << "  ";
    for (unsigned int j = 0; j < n; ++j) {
      size_type p = values[(i * n) + j].find('.');
      s.setf(std::ios::right, std::ios::adjustfield);
      s.width(static_cast<std::streamsize>(maxBefore));
      s << values[(i * n) + j].substr(0, p).c_str();

      if (maxAfter > 0) {
        s.setf(std::ios::left, std::ios::adjustfield);
        if (p != std::string::npos) {
          s.width(static_cast<std::streamsize>(maxAfter));
          s << values[(i * n) + j].substr(p, maxAfter).c_str();
        }
        else {
          s.width(static_cast<std::streamsize>(maxAfter));
          s << ".0";
        }
      }

      s << ' ';
    }
    s << std::endl;
  }

  s.flags(original_flags); // restore s to standard state

  return static_cast<int>(maxBefore + maxAfter);
}

/*!
  Print using Matlab syntax, to copy/paste in Matlab later.

  The following code
  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix M(2,3);
    int cpt = 0;
    for (unsigned int i=0; i<M.getRows(); i++)
      for (unsigned int j=0; j<M.getCols(); j++)
        M[i][j] = cpt++;

    std::cout << "M = "; M.matlabPrint(std::cout);
  }
  \endcode
  produces this output:
  \code
  M = [ 0, 1, 2, ;
  3, 4, 5, ]
  \endcode
  that could be copy/paste in Matlab:
  \code
  >> M = [ 0, 1, 2, ;
  3, 4, 5, ]

  M =

      0    1    2
      3    4    5

  >>
  \endcode
*/
std::ostream &vpMatrix::matlabPrint(std::ostream &os) const
{
  unsigned int this_rows = this->getRows();
  unsigned int this_col = this->getCols();
  os << "[ ";
  for (unsigned int i = 0; i < this_rows; ++i) {
    for (unsigned int j = 0; j < this_col; ++j) {
      os << (*this)[i][j] << ", ";
    }
    if (this->getRows() != (i + 1)) {
      os << ";" << std::endl;
    }
    else {
      os << "]" << std::endl;
    }
  }
  return os;
}

/*!
  Print using Maple syntax, to copy/paste in Maple later.

  The following code
  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix M(2,3);
    int cpt = 0;
    for (unsigned int i=0; i<M.getRows(); i++)
      for (unsigned int j=0; j<M.getCols(); j++)
        M[i][j] = cpt++;

    std::cout << "M = "; M.maplePrint(std::cout);
  }
  \endcode
  produces this output:
  \code
  M = ([
  [0, 1, 2, ],
  [3, 4, 5, ],
  ])
  \endcode
  that could be copy/paste in Maple.

*/
std::ostream &vpMatrix::maplePrint(std::ostream &os) const
{
  unsigned int this_rows = this->getRows();
  unsigned int this_col = this->getCols();
  os << "([ " << std::endl;
  for (unsigned int i = 0; i < this_rows; ++i) {
    os << "[";
    for (unsigned int j = 0; j < this_col; ++j) {
      os << (*this)[i][j] << ", ";
    }
    os << "]," << std::endl;
  }
  os << "])" << std::endl;
  return os;
}

/*!
  Print/save a matrix in csv format.

  The following code
  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    std::ofstream ofs("log.csv", std::ofstream::out);
    vpMatrix M(2,3);
    int cpt = 0;
    for (unsigned int i=0; i<M.getRows(); i++)
      for (unsigned int j=0; j<M.getCols(); j++)
        M[i][j] = cpt++;

    M.csvPrint(ofs);

    ofs.close();
  }
  \endcode
  produces log.csv file that contains:
  \code
  0, 1, 2
  3, 4, 5
  \endcode
*/
std::ostream &vpMatrix::csvPrint(std::ostream &os) const
{
  unsigned int this_rows = this->getRows();
  unsigned int this_col = this->getCols();
  for (unsigned int i = 0; i < this_rows; ++i) {
    for (unsigned int j = 0; j < this_col; ++j) {
      os << (*this)[i][j];
      if (!(j == (this->getCols() - 1))) {
        os << ", ";
      }
    }
    os << std::endl;
  }
  return os;
}

/*!
  Print to be used as part of a C++ code later.

  \param os : the stream to be printed in.
  \param matrixName : name of the matrix, "A" by default.
  \param octet : if false, print using double, if true, print byte per byte
  each bytes of the double array.

  The following code shows how to use this function:
  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix M(2,3);
    int cpt = 0;
    for (unsigned int i=0; i<M.getRows(); i++)
      for (unsigned int j=0; j<M.getCols(); j++)
        M[i][j] = cpt++;

    M.cppPrint(std::cout, "M");
  }
  \endcode
  It produces the following output that could be copy/paste in a C++ code:
  \code
  vpMatrix M (2, 3);
  M[0][0] = 0;
  M[0][1] = 1;
  M[0][2] = 2;

  M[1][0] = 3;
  M[1][1] = 4;
  M[1][2] = 5;
  \endcode
*/
std::ostream &vpMatrix::cppPrint(std::ostream &os, const std::string &matrixName, bool octet) const
{
  os << "vpMatrix " << matrixName << " (" << this->getRows() << ", " << this->getCols() << "); " << std::endl;

  unsigned int this_rows = this->getRows();
  unsigned int this_col = this->getCols();
  for (unsigned int i = 0; i < this_rows; ++i) {
    for (unsigned int j = 0; j < this_col; ++j) {
      if (!octet) {
        os << matrixName << "[" << i << "][" << j << "] = " << (*this)[i][j] << "; " << std::endl;
      }
      else {
        for (unsigned int k = 0; k < sizeof(double); ++k) {
          os << "((unsigned char*)&(" << matrixName << "[" << i << "][" << j << "]) )[" << k << "] = 0x" << std::hex
            << static_cast<unsigned int>(((unsigned char *)&((*this)[i][j]))[k]) << "; " << std::endl;
        }
      }
    }
    os << std::endl;
  }
  return os;
}

/*!
  Insert matrix A at the given position in the current matrix.

  \warning Throw vpException::dimensionError if the
  dimensions of the matrices do not allow the operation.

  \param A : The matrix to insert.
  \param r : The index of the row to begin to insert data.
  \param c : The index of the column to begin to insert data.
*/
void vpMatrix::insert(const vpMatrix &A, unsigned int r, unsigned int c)
{
  if (((r + A.getRows()) <= rowNum) && ((c + A.getCols()) <= colNum)) {
    if ((A.colNum == colNum) && (data != nullptr) && (A.data != nullptr) && (A.data != data)) {
      memcpy(data + (r * colNum), A.data, sizeof(double) * A.size());
    }
    else if ((data != nullptr) && (A.data != nullptr) && (A.data != data)) {
      unsigned int a_rows = A.getRows();
      for (unsigned int i = r; i < (r + a_rows); ++i) {
        memcpy(data + (i * colNum) + c, A.data + ((i - r) * A.colNum), sizeof(double) * A.colNum);
      }
    }
  }
  else {
    throw vpException(vpException::dimensionError, "Cannot insert (%dx%d) matrix in (%dx%d) matrix at position (%d,%d)",
                      A.getRows(), A.getCols(), rowNum, colNum, r, c);
  }
}

/*!
  Compute the eigenvalues of a n-by-n real symmetric matrix using
  Lapack 3rd party.

  \return The eigenvalues of a n-by-n real symmetric matrix, sorted in ascending order.

  \exception vpException::dimensionError If the matrix is not square.
  \exception vpException::fatalError If the matrix is not symmetric.
  \exception vpException::functionNotImplementedError If the Lapack 3rd party
  is not detected.

  Here an example:
  \code
  #include <iostream>

  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(3,3); // A is a symmetric matrix
    A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.;
    A[1][0] = 1/2.; A[1][1] = 1/3.; A[1][2] = 1/4.;
    A[2][0] = 1/3.; A[2][1] = 1/4.; A[2][2] = 1/5.;
    std::cout << "Initial symmetric matrix: \n" << A << std::endl;

    // Compute the eigen values
    vpColVector evalue; // Eigenvalues
    evalue = A.eigenValues();
    std::cout << "Eigen values: \n" << evalue << std::endl;
  }
  \endcode

  \sa eigenValues(vpColVector &, vpMatrix &)

*/
vpColVector vpMatrix::eigenValues() const
{
  vpColVector evalue(rowNum); // Eigen values

  if (rowNum != colNum) {
    throw(vpException(vpException::dimensionError, "Cannot compute eigen values on a non square matrix (%dx%d)", rowNum,
                      colNum));
  }

  // Check if the matrix is symmetric: At - A = 0
  vpMatrix At_A = (*this).t() - (*this);
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < rowNum; ++j) {
      // in comment: if (At_A[i][j] != 0) {
      if (std::fabs(At_A[i][j]) > std::numeric_limits<double>::epsilon()) {
        throw(vpException(vpException::fatalError, "Cannot compute eigen values on a non symmetric matrix"));
      }
    }
  }

#if defined(VISP_HAVE_LAPACK)
#if defined(VISP_HAVE_GSL) /* be careful of the copy below */
  {
    gsl_vector *eval = gsl_vector_alloc(rowNum);
    gsl_matrix *evec = gsl_matrix_alloc(rowNum, colNum);

    gsl_eigen_symmv_workspace *w = gsl_eigen_symmv_alloc(rowNum);
    gsl_matrix *m = gsl_matrix_alloc(rowNum, colNum);

    unsigned int Atda = (unsigned int)m->tda;
    for (unsigned int i = 0; i < rowNum; i++) {
      unsigned int k = i * Atda;
      for (unsigned int j = 0; j < colNum; j++)
        m->data[k + j] = (*this)[i][j];
    }
    gsl_eigen_symmv(m, eval, evec, w);

    gsl_eigen_symmv_sort(eval, evec, GSL_EIGEN_SORT_ABS_ASC);

    for (unsigned int i = 0; i < rowNum; i++) {
      evalue[i] = gsl_vector_get(eval, i);
    }

    gsl_eigen_symmv_free(w);
    gsl_vector_free(eval);
    gsl_matrix_free(m);
    gsl_matrix_free(evec);
  }
#else
  {
    const char jobz = 'N';
    const char uplo = 'U';
    vpMatrix A = (*this);
    vpColVector WORK;
    int lwork = -1;
    int info = 0;
    double wkopt;
    vpMatrix::blas_dsyev(jobz, uplo, rowNum, A.data, colNum, evalue.data, &wkopt, lwork, info);
    lwork = static_cast<int>(wkopt);
    WORK.resize(lwork);
    vpMatrix::blas_dsyev(jobz, uplo, rowNum, A.data, colNum, evalue.data, WORK.data, lwork, info);
  }
#endif
#else
  {
    throw(vpException(vpException::functionNotImplementedError, "Eigen values computation is not implemented. "
                      "You should install Lapack 3rd party"));
  }
#endif
  return evalue;
}

/*!
  Compute the eigenvalues of a n-by-n real symmetric matrix using
  Lapack 3rd party.

  \param evalue : Eigenvalues of the matrix, sorted in ascending order.

  \param evector : Corresponding eigenvectors of the matrix.

  \exception vpException::dimensionError If the matrix is not square.
  \exception vpException::fatalError If the matrix is not symmetric.
  \exception vpException::functionNotImplementedError If Lapack 3rd party is
  not detected.

  Here an example:
  \code
  #include <iostream>

  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(4,4); // A is a symmetric matrix
    A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.; A[0][3] = 1/4.;
    A[1][0] = 1/2.; A[1][1] = 1/3.; A[1][2] = 1/4.; A[1][3] = 1/5.;
    A[2][0] = 1/3.; A[2][1] = 1/4.; A[2][2] = 1/5.; A[2][3] = 1/6.;
    A[3][0] = 1/4.; A[3][1] = 1/5.; A[3][2] = 1/6.; A[3][3] = 1/7.;
    std::cout << "Initial symmetric matrix: \n" << A << std::endl;

    vpColVector d; // Eigenvalues
    vpMatrix    V; // Eigenvectors

    // Compute the eigenvalues and eigenvectors
    A.eigenValues(d, V);
    std::cout << "Eigen values: \n" << d << std::endl;
    std::cout << "Eigen vectors: \n" << V << std::endl;

    vpMatrix D;
    D.diag(d); // Eigenvalues are on the diagonal

    std::cout << "D: " << D << std::endl;

    // Verification: A * V = V * D
    std::cout << "AV-VD = 0 ? \n" << (A*V) - (V*D) << std::endl;
  }
  \endcode

  \sa eigenValues()
*/
void vpMatrix::eigenValues(vpColVector &evalue, vpMatrix &evector) const
{
  if (rowNum != colNum) {
    throw(vpException(vpException::dimensionError, "Cannot compute eigen values on a non square matrix (%dx%d)", rowNum,
                      colNum));
  }

  // Check if the matrix is symmetric: At - A = 0
  vpMatrix At_A = (*this).t() - (*this);
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < rowNum; ++j) {
      // -- in comment: if (At_A[i][j] != 0) {
      if (std::fabs(At_A[i][j]) > std::numeric_limits<double>::epsilon()) {
        throw(vpException(vpException::fatalError, "Cannot compute eigen values on a non symmetric matrix"));
      }
    }
  }

  // Resize the output matrices
  evalue.resize(rowNum);
  evector.resize(rowNum, colNum);

#if defined(VISP_HAVE_LAPACK)
#if defined(VISP_HAVE_GSL) /* be careful of the copy below */
  {
    gsl_vector *eval = gsl_vector_alloc(rowNum);
    gsl_matrix *evec = gsl_matrix_alloc(rowNum, colNum);

    gsl_eigen_symmv_workspace *w = gsl_eigen_symmv_alloc(rowNum);
    gsl_matrix *m = gsl_matrix_alloc(rowNum, colNum);

    unsigned int Atda = (unsigned int)m->tda;
    for (unsigned int i = 0; i < rowNum; i++) {
      unsigned int k = i * Atda;
      for (unsigned int j = 0; j < colNum; j++)
        m->data[k + j] = (*this)[i][j];
    }
    gsl_eigen_symmv(m, eval, evec, w);

    gsl_eigen_symmv_sort(eval, evec, GSL_EIGEN_SORT_ABS_ASC);

    for (unsigned int i = 0; i < rowNum; i++) {
      evalue[i] = gsl_vector_get(eval, i);
    }
    Atda = (unsigned int)evec->tda;
    for (unsigned int i = 0; i < rowNum; i++) {
      unsigned int k = i * Atda;
      for (unsigned int j = 0; j < rowNum; j++) {
        evector[i][j] = evec->data[k + j];
      }
    }

    gsl_eigen_symmv_free(w);
    gsl_vector_free(eval);
    gsl_matrix_free(m);
    gsl_matrix_free(evec);
  }
#else  // defined(VISP_HAVE_GSL)
  {
    const char jobz = 'V';
    const char uplo = 'U';
    vpMatrix A = (*this);
    vpColVector WORK;
    int lwork = -1;
    int info = 0;
    double wkopt;
    vpMatrix::blas_dsyev(jobz, uplo, rowNum, A.data, colNum, evalue.data, &wkopt, lwork, info);
    lwork = static_cast<int>(wkopt);
    WORK.resize(lwork);
    vpMatrix::blas_dsyev(jobz, uplo, rowNum, A.data, colNum, evalue.data, WORK.data, lwork, info);
    evector = A.t();
  }
#endif // defined(VISP_HAVE_GSL)
#else
  {
    throw(vpException(vpException::functionNotImplementedError, "Eigen values computation is not implemented. "
                      "You should install Lapack 3rd party"));
  }
#endif
}

/*!
  Function to compute the null space (the kernel) of a m-by-n matrix \f$\bf
  A\f$.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param kerAt: The matrix that contains the null space (kernel) of \f$\bf
  A\f$ defined by the matrix \f${\bf X}^T\f$. If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is (0, n), otherwise the dimension is (n-r,
  n). This matrix is thus the transpose of \f$\mbox{Ker}({\bf A})\f$.

  \param svThreshold: Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The rank of the matrix.
*/
unsigned int vpMatrix::kernel(vpMatrix &kerAt, double svThreshold) const
{
  unsigned int nbline = getRows();
  unsigned int nbcol = getCols();

  vpMatrix U, V; // Copy of the matrix, SVD function is destructive
  vpColVector sv;
  sv.resize(nbcol, false);       // singular values
  V.resize(nbcol, nbcol, false); // V matrix of singular value decomposition

  // Copy and resize matrix to have at least as many rows as columns
  // kernel is computed in svd method only if the matrix has more rows than
  // columns

  if (nbline < nbcol) {
    U.resize(nbcol, nbcol, true);
  }
  else {
    U.resize(nbline, nbcol, false);
  }

  U.insert(*this, 0, 0);

  U.svd(sv, V);

  // Compute the highest singular value and rank of the matrix
  double maxsv = 0;
  for (unsigned int i = 0; i < nbcol; ++i) {
    if (sv[i] > maxsv) {
      maxsv = sv[i];
    }
  }

  unsigned int rank = 0;
  for (unsigned int i = 0; i < nbcol; ++i) {
    if (sv[i] >(maxsv * svThreshold)) {
      ++rank;
    }
  }

  kerAt.resize(nbcol - rank, nbcol);
  if (rank != nbcol) {
    for (unsigned int j = 0, k = 0; j < nbcol; ++j) {
      // if( v.col(j) in kernel and non zero )
      if ((sv[j] <= (maxsv * svThreshold)) &&
          (std::fabs(V.getCol(j).sumSquare()) > std::numeric_limits<double>::epsilon())) {
        unsigned int v_rows = V.getRows();
        for (unsigned int i = 0; i < v_rows; ++i) {
          kerAt[k][i] = V[i][j];
        }
        ++k;
      }
    }
  }

  return rank;
}

/*!
  Function to compute the null space (the kernel) of a m-by-n matrix \f$\bf
  A\f$.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param kerA: The matrix that contains the null space (kernel) of \f$\bf
  A\f$. If matrix \f$\bf A\f$ is full rank, the dimension of \c kerA is (n, 0),
  otherwise its dimension is (n, n-r).

  \param svThreshold: Threshold used to test the singular values. The dimension
  of kerA corresponds to the number of singular values lower than this threshold

  \return The dimension of the nullspace, that is \f$ n - r \f$.
*/
unsigned int vpMatrix::nullSpace(vpMatrix &kerA, double svThreshold) const
{
  unsigned int nbrow = getRows();
  unsigned int nbcol = getCols();

  vpMatrix U, V; // Copy of the matrix, SVD function is destructive
  vpColVector sv;
  sv.resize(nbcol, false);       // singular values
  V.resize(nbcol, nbcol, false); // V matrix of singular value decomposition

  // Copy and resize matrix to have at least as many rows as columns
  // kernel is computed in svd method only if the matrix has more rows than
  // columns

  if (nbrow < nbcol) {
    U.resize(nbcol, nbcol, true);
  }
  else {
    U.resize(nbrow, nbcol, false);
  }

  U.insert(*this, 0, 0);

  U.svd(sv, V);

  // Compute the highest singular value and rank of the matrix
  double maxsv = sv[0];

  unsigned int rank = 0;
  for (unsigned int i = 0; i < nbcol; ++i) {
    if (sv[i] >(maxsv * svThreshold)) {
      ++rank;
    }
  }

  kerA.resize(nbcol, nbcol - rank);
  if (rank != nbcol) {
    for (unsigned int j = 0, k = 0; j < nbcol; ++j) {
      // if( v.col(j) in kernel and non zero )
      if (sv[j] <= (maxsv * svThreshold)) {
        for (unsigned int i = 0; i < nbcol; ++i) {
          kerA[i][k] = V[i][j];
        }
        ++k;
      }
    }
  }

  return (nbcol - rank);
}

/*!
  Function to compute the null space (the kernel) of a m-by-n matrix \f$\bf
  A\f$.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param kerA: The matrix that contains the null space (kernel) of \f$\bf
  A\f$. If matrix \f$\bf A\f$ is full rank, the dimension of \c kerA is (n, 0),
  otherwise its dimension is (n, n-r).

  \param dim: the dimension of the null space when it is known a priori

  \return The estimated dimension of the nullspace, that is \f$ n - r \f$, by
  using 1e-6 as threshold for the sigular values.
*/
unsigned int vpMatrix::nullSpace(vpMatrix &kerA, int dim) const
{
  unsigned int nbrow = getRows();
  unsigned int nbcol = getCols();
  unsigned int dim_ = static_cast<unsigned int>(dim);

  vpMatrix U, V; // Copy of the matrix, SVD function is destructive
  vpColVector sv;
  sv.resize(nbcol, false);       // singular values
  V.resize(nbcol, nbcol, false); // V matrix of singular value decomposition

  // Copy and resize matrix to have at least as many rows as columns
  // kernel is computed in svd method only if the matrix has more rows than
  // columns

  if (nbrow < nbcol) {
    U.resize(nbcol, nbcol, true);
  }
  else {
    U.resize(nbrow, nbcol, false);
  }

  U.insert(*this, 0, 0);

  U.svd(sv, V);

  kerA.resize(nbcol, dim_);
  if (dim_ != 0) {
    unsigned int rank = nbcol - dim_;
    for (unsigned int k = 0; k < dim_; ++k) {
      unsigned int j = k + rank;
      for (unsigned int i = 0; i < nbcol; ++i) {
        kerA[i][k] = V[i][j];
      }
    }
  }

  double maxsv = sv[0];
  unsigned int rank = 0;
  for (unsigned int i = 0; i < nbcol; ++i) {
    if (sv[i] >(maxsv * 1e-6)) {
      ++rank;
    }
  }
  return (nbcol - rank);
}

/*!
  Compute the determinant of a n-by-n matrix.

  \param method : Method used to compute the determinant. Default LU
  decomposition method is faster than the method based on Gaussian
  elimination.

  \return Determinant of the matrix.

  \code
  #include <iostream>

  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(3,3);
    A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.;
    A[1][0] = 1/3.; A[1][1] = 1/4.; A[1][2] = 1/5.;
    A[2][0] = 1/6.; A[2][1] = 1/7.; A[2][2] = 1/8.;
    std::cout << "Initial matrix: \n" << A << std::endl;

    // Compute the determinant
    std:: cout << "Determinant by default method           : " << A.det() << std::endl;
    std:: cout << "Determinant by LU decomposition         : " << A.detByLU() << std::endl;
    std:: cout << "Determinant by LU decomposition (Lapack): " << A.detByLULapack() << std::endl;
    std:: cout << "Determinant by LU decomposition (OpenCV): " << A.detByLUOpenCV() << std::endl;
    std:: cout << "Determinant by LU decomposition (Eigen3): " << A.detByLUEigen3() << std::endl;
  }
  \endcode
*/
double vpMatrix::det(vpDetMethod method) const
{
  double det = 0.;

  if (method == LU_DECOMPOSITION) {
    det = this->detByLU();
  }

  return det;
}

vpMatrix vpMatrix::cholesky() const
{
#if defined(VISP_HAVE_EIGEN3)
  return choleskyByEigen3();
#elif defined(VISP_HAVE_LAPACK)
  return choleskyByLapack();
#elif defined(VISP_HAVE_OPENCV)
  return choleskyByOpenCV();
#else
  throw(vpException(vpException::fatalError, "Cannot compute matrix Chloesky decomposition. "
                    "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!

  Compute the exponential matrix of a square matrix.

  \return Return the exponential matrix.

*/
vpMatrix vpMatrix::expm() const
{
  if (colNum != rowNum) {
    throw(vpException(vpException::dimensionError, "Cannot compute the exponential of a non square (%dx%d) matrix",
                      rowNum, colNum));
  }
  else {
#ifdef VISP_HAVE_GSL
    size_t size_ = rowNum * colNum;
    double *b = new double[size_];
    for (size_t i = 0; i < size_; i++)
      b[i] = 0.;
    gsl_matrix_view m = gsl_matrix_view_array(this->data, rowNum, colNum);
    gsl_matrix_view em = gsl_matrix_view_array(b, rowNum, colNum);
    gsl_linalg_exponential_ss(&m.matrix, &em.matrix, 0);
    // gsl_matrix_fprintf(stdout, &em.matrix, "%g");
    vpMatrix expA;
    expA.resize(rowNum, colNum, false);
    memcpy(expA.data, b, size_ * sizeof(double));

    delete[] b;
    return expA;
#else
    vpMatrix v_expE(rowNum, colNum, false);
    vpMatrix v_expD(rowNum, colNum, false);
    vpMatrix v_expX(rowNum, colNum, false);
    vpMatrix v_expcX(rowNum, colNum, false);
    vpMatrix v_eye(rowNum, colNum, false);

    v_eye.eye();
    vpMatrix exp(*this);

    //   -- in comment:   double f;
    int e;
    double c = 0.5;
    int q = 6;
    int p = 1;

    double nA = 0;
    for (unsigned int i = 0; i < rowNum; ++i) {
      double sum = 0;
      for (unsigned int j = 0; j < colNum; ++j) {
        sum += fabs((*this)[i][j]);
      }
      if ((sum > nA) || (i == 0)) {
        nA = sum;
      }
    }

    /* f = */ frexp(nA, &e);
    // -- in comment: double s = (0 > e+1)?0:e+1;
    double s = e + 1;

    double sca = 1.0 / pow(2.0, s);
    exp = sca * exp;
    v_expX = *this;
    v_expE = (c * exp) + v_eye;
    v_expD = (-c * exp) + v_eye;
    for (int k = 2; k <= q; ++k) {
      c = (c * (static_cast<double>((q - k) + 1))) / (static_cast<double>(k * (((2 * q) - k) + 1)));
      v_expcX = exp * v_expX;
      v_expX = v_expcX;
      v_expcX = c * v_expX;
      v_expE = v_expE + v_expcX;
      if (p) {
        v_expD = v_expD + v_expcX;
      }
      else {
        v_expD = v_expD - v_expcX;
      }
      p = !p;
    }
    v_expX = v_expD.inverseByLU();
    exp = v_expX * v_expE;
    for (int k = 1; k <= s; ++k) {
      v_expE = exp * exp;
      exp = v_expE;
    }
    return exp;
#endif
  }
}

/**************************************************************************************************************/
/**************************************************************************************************************/

// Specific functions

/*
  input:: matrix M(nCols,nRows), nCols > 3, nRows > 3 , nCols == nRows.

  output:: the complement matrix of the element (rowNo,colNo).
  This is the matrix obtained from M after elimenating the row rowNo and column
  colNo

  example:
  1 2 3
  M = 4 5 6
  7 8 9
  1 3
  subblock(M, 1, 1) give the matrix 7 9
*/
vpMatrix subblock(const vpMatrix &M, unsigned int col, unsigned int row)
{
  vpMatrix M_comp;
  M_comp.resize(M.getRows() - 1, M.getCols() - 1, false);

  unsigned int m_rows = M.getRows();
  unsigned int m_col = M.getCols();
  for (unsigned int i = 0; i < col; ++i) {
    for (unsigned int j = 0; j < row; ++j) {
      M_comp[i][j] = M[i][j];
    }
    for (unsigned int j = row + 1; j < m_rows; ++j) {
      M_comp[i][j - 1] = M[i][j];
    }
  }
  for (unsigned int i = col + 1; i < m_col; ++i) {
    for (unsigned int j = 0; j < row; ++j) {
      M_comp[i - 1][j] = M[i][j];
    }
    for (unsigned int j = row + 1; j < m_rows; ++j) {
      M_comp[i - 1][j - 1] = M[i][j];
    }
  }
  return M_comp;
}

/*!
  \return The condition number, the ratio of the largest singular value of
  the matrix to the smallest.

  \param svThreshold: Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

 */
double vpMatrix::cond(double svThreshold) const
{
  unsigned int nbline = getRows();
  unsigned int nbcol = getCols();

  vpMatrix U, V; // Copy of the matrix, SVD function is destructive
  vpColVector sv;
  sv.resize(nbcol);              // singular values
  V.resize(nbcol, nbcol, false); // V matrix of singular value decomposition

  // Copy and resize matrix to have at least as many rows as columns
  // kernel is computed in svd method only if the matrix has more rows than
  // columns

  if (nbline < nbcol) {
    U.resize(nbcol, nbcol, true);
  }
  else {
    U.resize(nbline, nbcol, false);
  }

  U.insert(*this, 0, 0);

  U.svd(sv, V);

  // Compute the highest singular value
  double maxsv = 0;
  for (unsigned int i = 0; i < nbcol; ++i) {
    if (sv[i] > maxsv) {
      maxsv = sv[i];
    }
  }

  // Compute the rank of the matrix
  unsigned int rank = 0;
  for (unsigned int i = 0; i < nbcol; ++i) {
    if (sv[i] >(maxsv * svThreshold)) {
      ++rank;
    }
  }

  // Compute the lowest singular value
  double minsv = maxsv;
  for (unsigned int i = 0; i < rank; ++i) {
    if (sv[i] < minsv) {
      minsv = sv[i];
    }
  }

  if (std::fabs(minsv) > std::numeric_limits<double>::epsilon()) {
    return maxsv / minsv;
  }
  else {
    return std::numeric_limits<double>::infinity();
  }
}

/*!
  Compute \f${\bf H} + \alpha * diag({\bf H})\f$
  \param H : input Matrix \f${\bf H}\f$. This matrix should be square.
  \param alpha : Scalar \f$\alpha\f$
  \param HLM : Resulting operation.
 */
void vpMatrix::computeHLM(const vpMatrix &H, const double &alpha, vpMatrix &HLM)
{
  if (H.getCols() != H.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot compute HLM on a non square matrix (%dx%d)", H.getRows(),
                      H.getCols()));
  }

  HLM = H;
  unsigned int h_col = H.getCols();
  for (unsigned int i = 0; i < h_col; ++i) {
    HLM[i][i] += alpha * H[i][i];
  }
}

/*!
  Compute and return the Frobenius norm (also called Euclidean norm) \f$||A|| = \sqrt{ \sum{A_{ij}^2}}\f$.

  \return The Frobenius norm (also called Euclidean norm) if the matrix is initialized, 0 otherwise.

  \sa infinityNorm(), inducedL2Norm()
*/
double vpMatrix::frobeniusNorm() const
{
  double norm = 0.0;
  for (unsigned int i = 0; i < dsize; ++i) {
    double x = *(data + i);
    norm += x * x;
  }

  return sqrt(norm);
}

/*!
  Compute and return the induced L2 norm \f$||A|| = \Sigma_{max}(A)\f$ which is equal to
  the maximum singular value of the matrix.

  \return The induced L2 norm if the matrix is initialized, 0 otherwise.

  \sa infinityNorm(), frobeniusNorm()
*/
double vpMatrix::inducedL2Norm() const
{
  if (this->dsize != 0) {
    vpMatrix v;
    vpColVector w;

    vpMatrix M = *this;

    M.svd(w, v);

    double max = w[0];
    unsigned int maxRank = std::min<unsigned int>(this->getCols(), this->getRows());
    // The maximum reachable rank is either the number of columns or the number of rows
    // of the matrix.
    unsigned int boundary = std::min<unsigned int>(maxRank, w.size());
    // boundary is here to ensure that the number of singular values used for the com-
    // putation of the euclidean norm of the matrix is not greater than the maximum
    // reachable rank. Indeed, some svd library pad the singular values vector with 0s
    // if the input matrix is non-square.
    for (unsigned int i = 0; i < boundary; ++i) {
      if (max < w[i]) {
        max = w[i];
      }
    }
    return max;
  }
  else {
    return 0.;
  }
}

/*!

  Compute and return the infinity norm \f$ {||A||}_{\infty} =
  max\left(\sum_{j=0}^{n}{\mid A_{ij} \mid}\right) \f$ with \f$i \in
  \{0, ..., m\}\f$ where \f$(m,n)\f$ is the matrix size.

  \return The infinity norm if the matrix is initialized, 0 otherwise.

  \sa frobeniusNorm(), inducedL2Norm()
*/
double vpMatrix::infinityNorm() const
{
  double norm = 0.0;
  for (unsigned int i = 0; i < rowNum; ++i) {
    double x = 0;
    for (unsigned int j = 0; j < colNum; ++j) {
      x += fabs(*(*(rowPtrs + i) + j));
    }
    if (x > norm) {
      norm = x;
    }
  }
  return norm;
}

/*!
  Return the sum square of all the \f$A_{ij}\f$ elements of the matrix \f$A(m,
  n)\f$.

  \return The value \f$\sum A_{ij}^{2}\f$.
*/
double vpMatrix::sumSquare() const
{
  double sum_square = 0.0;
  double x;

  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      x = rowPtrs[i][j];
      sum_square += x * x;
    }
  }

  return sum_square;
}
#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
  \deprecated This function is deprecated. You should rather use frobeniusNorm().

  Compute and return the Euclidean norm (also called Frobenius norm) \f$||A|| = \sqrt{ \sum{A_{ij}^2}}\f$.

  \return The Euclidean norm (also called Frobenius norm) if the matrix is initialized, 0 otherwise.

  \sa frobeniusNorm(), infinityNorm(), inducedL2Norm()
*/
double vpMatrix::euclideanNorm() const { return frobeniusNorm(); }

/*!
  \deprecated This method is deprecated. You should rather use getRow().
  More precisely, the following code:
  \code
  vpMatrix L;
  unsigned int row_index = ...;
  ... = L.row(row_index);
  \endcode
  should be replaced with:
  \code
  ... = L.getRow(row_index - 1);
  \endcode

  \warning Notice row(1) is the 0th row.
  This function returns the i-th row of the matrix.
  \param i : Index of the row to extract noting that row index start at 1 to get the first row.

*/
vpRowVector vpMatrix::row(unsigned int i)
{
  vpRowVector c(getCols());

  for (unsigned int j = 0; j < getCols(); ++j) {
    c[j] = (*this)[i - 1][j];
  }
  return c;
}

/*!
  \deprecated This method is deprecated. You should rather use getCol().
  More precisely, the following code:
  \code
  vpMatrix L;
  unsigned int column_index = ...;
  ... = L.column(column_index);
  \endcode
  should be replaced with:
  \code
  ... = L.getCol(column_index - 1);
  \endcode

  \warning Notice column(1) is the 0-th column.
  This function returns the j-th columns of the matrix.
  \param j : Index of the column to extract noting that column index start at 1 to get the first column.
*/
vpColVector vpMatrix::column(unsigned int j)
{
  vpColVector c(getRows());

  for (unsigned int i = 0; i < getRows(); ++i) {
    c[i] = (*this)[i][j - 1];
  }
  return c;
}

/*!
  \deprecated You should rather use diag(const double &)

  Set the matrix diagonal elements to \e val.
  More generally set M[i][i] = val.
*/
void vpMatrix::setIdentity(const double &val)
{
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      if (i == j) {
        (*this)[i][j] = val;
      }
      else {
        (*this)[i][j] = 0;
      }
    }
  }
}

#endif //#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

END_VISP_NAMESPACE
