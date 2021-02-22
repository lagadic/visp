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
 * Matrix manipulation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/
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

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpCPUFeatures.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpTranslationVector.h>

#include <Simd/SimdLib.hpp>

#ifdef VISP_HAVE_LAPACK
#  ifdef VISP_HAVE_GSL
#    include <gsl/gsl_eigen.h>
#    include <gsl/gsl_math.h>
#    include <gsl/gsl_linalg.h>
#  elif defined(VISP_HAVE_MKL)
#    include <mkl.h>
typedef MKL_INT integer;

void vpMatrix::blas_dsyev(char jobz, char uplo, unsigned int n_, double *a_data, unsigned int lda_,
                          double *w_data, double *work_data, int lwork_, int &info_)
{
  MKL_INT n = static_cast<MKL_INT>(n_);
  MKL_INT lda = static_cast<MKL_INT>(lda_);
  MKL_INT lwork = static_cast<MKL_INT>(lwork_);
  MKL_INT info = static_cast<MKL_INT>(info_);

  dsyev(&jobz, &uplo, &n, a_data, &lda, w_data, work_data, &lwork, &info);
}

#  else
#    if defined(VISP_HAVE_LAPACK_BUILT_IN)
typedef long int integer;
#    else
typedef int integer;
#    endif
extern "C" integer dsyev_(char *jobz, char *uplo, integer *n, double *a, integer *lda,
                      double *w, double *WORK, integer *lwork, integer *info);

void vpMatrix::blas_dsyev(char jobz, char uplo, unsigned int n_, double *a_data, unsigned int lda_,
                          double *w_data, double *work_data, int lwork_, int &info_)
{
  integer n = static_cast<integer>(n_);
  integer lda = static_cast<integer>(lda_);
  integer lwork = static_cast<integer>(lwork_);
  integer info = static_cast<integer>(info_);

  dsyev_(&jobz, &uplo, &n, a_data, &lda, w_data, work_data, &lwork, &info);

  lwork_ = static_cast<int>(lwork);
  info_ = static_cast<int>(info);
}
#  endif
#endif

#if !defined(VISP_USE_MSVC) || (defined(VISP_USE_MSVC) && !defined(VISP_BUILD_SHARED_LIBS))
const unsigned int vpMatrix::m_lapack_min_size_default = 0;
unsigned int vpMatrix::m_lapack_min_size = vpMatrix::m_lapack_min_size_default;
#endif

// Prototypes of specific functions
vpMatrix subblock(const vpMatrix &, unsigned int, unsigned int);

void compute_pseudo_inverse(const vpMatrix &U, const vpColVector &sv, const vpMatrix &V, unsigned int nrows,
                            unsigned int ncols, double svThreshold, vpMatrix &Ap, int &rank_out,
                            int *rank_in, vpMatrix *imA, vpMatrix *imAt, vpMatrix *kerAt)
{
  Ap.resize(ncols, nrows, true, false);

  // compute the highest singular value and the rank of h
  double maxsv = sv[0];

  rank_out = 0;

  for (unsigned int i = 0; i < ncols; i++) {
    if (sv[i] > maxsv * svThreshold) {
      rank_out++;
    }
  }

  unsigned int rank = static_cast<unsigned int>(rank_out);
  if (rank_in) {
    rank = static_cast<unsigned int>(*rank_in);
  }

  for (unsigned int i = 0; i < ncols; i++) {
    for (unsigned int j = 0; j < nrows; j++) {
      for (unsigned int k = 0; k < rank; k++) {
        Ap[i][j] += V[i][k] * U[j][k] / sv[k];
      }
    }
  }

  // Compute im(A)
  if (imA) {
    imA->resize(nrows, rank);

    for (unsigned int i = 0; i < nrows; i++) {
      for (unsigned int j = 0; j < rank; j++) {
        (*imA)[i][j] = U[i][j];
      }
    }
  }

  // Compute im(At)
  if (imAt) {
    imAt->resize(ncols, rank);
    for (unsigned int i = 0; i < ncols; i++) {
      for (unsigned int j = 0; j < rank; j++) {
        (*imAt)[i][j] = V[i][j];
      }
    }
  }

  // Compute ker(At)
  if (kerAt) {
    kerAt->resize(ncols - rank, ncols);
    if (rank != ncols) {
      for (unsigned int k = 0; k < (ncols - rank); k++) {
        unsigned j = k + rank;
        for (unsigned int i = 0; i < V.getRows(); i++) {
          (*kerAt)[k][i] = V[i][j];
        }
      }
    }
  }
}

/*!
  Construct a matrix as a sub-matrix of the input matrix \e M.
  \sa init(const vpMatrix &M, unsigned int r, unsigned int c, unsigned int
  nrows, unsigned int ncols)
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
  A.rowPtrs = NULL;
  A.dsize = 0;
  A.data = NULL;
}

/*!
   Construct a matrix from a list of double values.
   \param list : List of double.

   The following code shows how to use this constructor to initialize a 2-by-3 matrix using reshape() function:

   \code
#include <visp3/core/vpMatrix.h>

int main()
{
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
vpMatrix M( {-1, -2, -3, 4, 5.5, 6.0f} );
M.reshape(2, 3);
std::cout << "M:\n" << M << std::endl;
#endif
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

int main()
{
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
vpMatrix M(2, 3, {-1, -2, -3, 4, 5.5, 6});
std::cout << "M:\n" << M << std::endl;
#endif
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
  : vpArray2D<double>(nrows, ncols, list) {}

/*!
   Construct a matrix from a list of double values.
   \param lists : List of double.
   The following code shows how to use this constructor to initialize a 2-by-3 matrix function:
   \code
#include <visp3/core/vpMatrix.h>

int main()
{
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
vpMatrix M( { {-1, -2, -3}, {4, 5.5, 6} } );
std::cout << "M:\n" << M << std::endl;
#endif
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

int main()
{
  vpMatrix M(4,5);
  int val = 0;
  for(size_t i=0; i<M.getRows(); i++) {
    for(size_t j=0; j<M.getCols(); j++) {
      M[i][j] = val++;
    }
  }
  M.print (std::cout, 4, "M ");

  vpMatrix N;
  N.init(M, 0, 1, 2, 3);
  N.print (std::cout, 4, "N ");
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

  if (rnrows > M.getRows())
    throw(vpException(vpException::dimensionError, "Bad row dimension (%d > %d) used to initialize vpMatrix", rnrows,
                      M.getRows()));
  if (cncols > M.getCols())
    throw(vpException(vpException::dimensionError, "Bad column dimension (%d > %d) used to initialize vpMatrix", cncols,
                      M.getCols()));
  resize(nrows, ncols, false, false);

  if (this->rowPtrs == NULL) // Fix coverity scan: explicit null dereferenced
    return;                  // Noting to do
  for (unsigned int i = 0; i < nrows; i++) {
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

int main()
{
  vpMatrix M(4,5);
  int val = 0;
  for(size_t i=0; i<M.getRows(); i++) {
    for(size_t j=0; j<M.getCols(); j++) {
      M[i][j] = val++;
    }
  }
  M.print (std::cout, 4, "M ");
  vpMatrix N = M.extract(0, 1, 2, 3);
  N.print (std::cout, 4, "N ");
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

  \sa init(const vpMatrix &, unsigned int, unsigned int, unsigned int,
unsigned int)
 */
vpMatrix vpMatrix::extract(unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols) const
{
  unsigned int rnrows = r + nrows;
  unsigned int cncols = c + ncols;

  if (rnrows > getRows())
    throw(vpException(vpException::dimensionError, "Bad row dimension (%d > %d) used to initialize vpMatrix", rnrows,
                      getRows()));
  if (cncols > getCols())
    throw(vpException(vpException::dimensionError, "Bad column dimension (%d > %d) used to initialize vpMatrix", cncols,
                      getCols()));

  vpMatrix M;
  M.resize(nrows, ncols, false, false);
  for (unsigned int i = 0; i < nrows; i++) {
    memcpy(M[i], &(*this)[i + r][c], ncols * sizeof(double));
  }

  return M;
}

/*!
  Set an n-by-n matrix to identity with ones on the diagonal and zeros
  else where.
*/
void vpMatrix::eye(unsigned int n) { eye(n, n); }

/*!
  Set an m-by-n matrix to identity with ones on the diagonal and zeros
  else where.
*/
void vpMatrix::eye(unsigned int m, unsigned int n)
{
  resize(m, n);

  eye();
}

/*!
  Set an m-by-n matrix to identity with ones on the diagonal and zeros
  else where.
*/
void vpMatrix::eye()
{
  for (unsigned int i = 0; i < rowNum; i++) {
    for (unsigned int j = 0; j < colNum; j++) {
      if (i == j)
        (*this)[i][j] = 1.0;
      else
        (*this)[i][j] = 0;
    }
  }
}

/*!
  Compute and return the transpose of the matrix.
*/
vpMatrix vpMatrix::t() const
{
  return transpose();
}

/*!
  Compute and return the transpose of the matrix.

  \sa t()
*/
vpMatrix vpMatrix::transpose() const
{
  vpMatrix At;
  transpose(At);
  return At;
}

/*!
  Compute \e At the transpose of the matrix.
  \param At (output) : Resulting transpose matrix.
  \sa t()
*/
void vpMatrix::transpose(vpMatrix &At) const
{
  At.resize(colNum, rowNum, false, false);

  if (rowNum <= 16 || colNum <= 16) {
    for (unsigned int i = 0; i < rowNum; i++) {
      for (unsigned int j = 0; j < colNum; j++) {
        At[j][i] = (*this)[i][j];
      }
    }
  }
  else {
    SimdMatTranspose(data, rowNum, colNum, At.data);
  }
}

/*!
  Computes the \f$AA^T\f$ operation \f$B = A*A^T\f$
  \return  \f$A*A^T\f$
  \sa AAt(vpMatrix &) const
*/
vpMatrix vpMatrix::AAt() const
{
  vpMatrix B;

  AAt(B);

  return B;
}

/*!
  Compute the AAt operation such as \f$B = A*A^T\f$.

  The result is placed in the parameter \e B and not returned.

  A new matrix won't be allocated for every use of the function. This
  results in a speed gain if used many times with the same result
  matrix size.

  \sa AAt()
*/
void vpMatrix::AAt(vpMatrix &B) const
{
  if ((B.rowNum != rowNum) || (B.colNum != rowNum))
    B.resize(rowNum, rowNum, false, false);

  // If available use Lapack only for large matrices
  bool useLapack = (rowNum > vpMatrix::m_lapack_min_size || colNum > vpMatrix::m_lapack_min_size);
#if !(defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL))
  useLapack = false;
#endif

  if (useLapack) {
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL)
    const double alpha = 1.0;
    const double beta = 0.0;
    const char transa = 't';
    const char transb = 'n';

    vpMatrix::blas_dgemm(transa, transb, rowNum, rowNum, colNum, alpha, data, colNum, data, colNum, beta, B.data, rowNum);
#endif
  }
  else {
    // compute A*A^T
    for (unsigned int i = 0; i < rowNum; i++) {
      for (unsigned int j = i; j < rowNum; j++) {
        double *pi = rowPtrs[i]; // row i
        double *pj = rowPtrs[j]; // row j

        // sum (row i .* row j)
        double ssum = 0;
        for (unsigned int k = 0; k < colNum; k++)
          ssum += *(pi++) * *(pj++);

        B[i][j] = ssum; // upper triangle
        if (i != j)
          B[j][i] = ssum; // lower triangle
      }
    }
  }
}

/*!
  Compute the AtA operation such as \f$B = A^T*A\f$.

  The result is placed in the parameter \e B and not returned.

  A new matrix won't be allocated for every use of the function. This
  results in a speed gain if used many times with the same result matrix
  size.

  \sa AtA()
*/
void vpMatrix::AtA(vpMatrix &B) const
{
  if ((B.rowNum != colNum) || (B.colNum != colNum))
    B.resize(colNum, colNum, false, false);

  // If available use Lapack only for large matrices
  bool useLapack = (rowNum > vpMatrix::m_lapack_min_size || colNum > vpMatrix::m_lapack_min_size);
#if !(defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL))
  useLapack = false;
#endif

  if (useLapack) {
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL)
    const double alpha = 1.0;
    const double beta = 0.0;
    const char transa = 'n';
    const char transb = 't';

    vpMatrix::blas_dgemm(transa, transb, colNum, colNum, rowNum, alpha, data, colNum, data, colNum, beta, B.data, colNum);
#endif
  }
  else {
    for (unsigned int i = 0; i < colNum; i++) {
      double *Bi = B[i];
      for (unsigned int j = 0; j < i; j++) {
        double *ptr = data;
        double s = 0;
        for (unsigned int k = 0; k < rowNum; k++) {
          s += (*(ptr + i)) * (*(ptr + j));
          ptr += colNum;
        }
        *Bi++ = s;
        B[j][i] = s;
      }
      double *ptr = data;
      double s = 0;
      for (unsigned int k = 0; k < rowNum; k++) {
        s += (*(ptr + i)) * (*(ptr + i));
        ptr += colNum;
      }
      *Bi = s;
    }
  }
}

/*!
  Compute the AtA operation such as \f$B = A^T*A\f$
  \return  \f$A^T*A\f$
  \sa AtA(vpMatrix &) const
*/
vpMatrix vpMatrix::AtA() const
{
  vpMatrix B;

  AtA(B);

  return B;
}

/*!
  Copy operator that allows to convert on of the following container that
  inherit from vpArray2D such as vpMatrix, vpRotationMatrix,
  vpHomogeneousMatrix, vpPoseVector, vpColVector, vpRowVector... into a
  vpMatrix.

  \param A : 2D array to be copied.

  The following example shows how to create a matrix from an homogeneous
  matrix:
\code
vpRotationMatrix R;
vpMatrix M = R;
\endcode

*/
vpMatrix &vpMatrix::operator=(const vpArray2D<double> &A)
{
  resize(A.getRows(), A.getCols(), false, false);

  if (data != NULL && A.data != NULL && data != A.data) {
    memcpy(data, A.data, dsize * sizeof(double));
  }

  return *this;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
vpMatrix &vpMatrix::operator=(const vpMatrix &A)
{
  resize(A.getRows(), A.getCols(), false, false);

  if (data != NULL && A.data != NULL && data != A.data) {
    memcpy(data, A.data, dsize * sizeof(double));
  }

  return *this;
}

vpMatrix &vpMatrix::operator=(vpMatrix &&other)
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

/*!
  Set matrix elements from a list of values.
  \param list : List of double. Matrix size (number of columns multiplied by number of columns) should match the number of elements.
  \return The modified Matrix.
  The following example shows how to set each element of a 2-by-3 matrix.
  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix M;
  M = { -1, -2, -3, -4, -5, -6 };
  M.reshape(2, 3);
  std::cout << "M:\n" << M << std::endl;
}
  \endcode
  It produces the following printings:
  \code
M:
-1  -2  -3
-4  -5  -6
  \endcode
  \sa operator<<()
 */
vpMatrix& vpMatrix::operator=(const std::initializer_list<double> &list)
{
  if (dsize != static_cast<unsigned int>(list.size())) {
    resize(1, static_cast<unsigned int>(list.size()), false, false);
  }

  std::copy(list.begin(), list.end(), data);

  return *this;
}

/*!
  Set matrix elements from a list of values.
  \param lists : List of double.
  \return The modified Matrix.
  The following example shows how to set each element of a 2-by-3 matrix.
  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix M;
  M = { {-1, -2, -3}, {-4, -5, -6} };
  std::cout << "M:\n" << M << std::endl;
}
  \endcode
  It produces the following printings:
  \code
M:
-1  -2  -3
-4  -5  -6
  \endcode
  \sa operator<<()
 */
vpMatrix& vpMatrix::operator=(const std::initializer_list<std::initializer_list<double> > &lists)
{
  unsigned int nrows = static_cast<unsigned int>(lists.size()), ncols = 0;
  for (auto& l : lists) {
    if (static_cast<unsigned int>(l.size()) > ncols) {
      ncols = static_cast<unsigned int>(l.size());
    }
  }

  resize(nrows, ncols, false, false);
  auto it = lists.begin();
  for (unsigned int i = 0; i < rowNum; i++, ++it) {
    std::copy(it->begin(), it->end(), rowPtrs[i]);
  }

  return *this;
}
#endif

//! Set all the element of the matrix A to \e x.
vpMatrix &vpMatrix::operator=(double x)
{
  std::fill(data, data + rowNum*colNum, x);
  return *this;
}

/*!
  Assigment from an array of double. This method has to be used carefully
  since the array allocated behind \e x pointer should have the same dimension
  than the matrix.
*/
vpMatrix &vpMatrix::operator<<(double *x)
{
  for (unsigned int i = 0; i < rowNum; i++) {
    for (unsigned int j = 0; j < colNum; j++) {
      rowPtrs[i][j] = *x++;
    }
  }
  return *this;
}

vpMatrix& vpMatrix::operator<<(double val)
{
  resize(1, 1, false, false);
  rowPtrs[0][0] = val;
  return *this;
}

vpMatrix& vpMatrix::operator,(double val)
{
  resize(1, colNum + 1, false, false);
  rowPtrs[0][colNum - 1] = val;
  return *this;
}

/*!
  Create a diagonal matrix with the element of a vector.

  \param  A : Vector which element will be put in the diagonal.

  \sa createDiagonalMatrix()

\code
#include <iostream>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A;
  vpColVector v(3);

  v[0] = 1;
  v[1] = 2;
  v[2] = 3;

  A.diag(v);

  std::cout << "A:\n" << A << std::endl;
}
\endcode

  Matrix A is now equal to:
\code
1 0 0
0 2 0
0 0 3
\endcode
*/
void vpMatrix::diag(const vpColVector &A)
{
  unsigned int rows = A.getRows();
  this->resize(rows, rows);

  (*this) = 0;
  for (unsigned int i = 0; i < rows; i++)
    (*this)[i][i] = A[i];
}

/*!
  Set the matrix as a diagonal matrix where each element on the diagonal is
set to \e val. Elements that are not on the diagonal are set to 0.

  \param val : Value to set.

  \sa eye()

\code
#include <iostream>

#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(3, 4);

  A.diag(2);

  std::cout << "A:\n" << A << std::endl;
}
\endcode

  Matrix A is now equal to:
\code
2 0 0 0
0 2 0 0
0 0 2 0
\endcode
*/
void vpMatrix::diag(const double &val)
{
  (*this) = 0;
  unsigned int min_ = (rowNum < colNum) ? rowNum : colNum;
  for (unsigned int i = 0; i < min_; i++)
    (*this)[i][i] = val;
}

/*!

  Create a diagonal matrix with the element of a vector \f$ DA_{ii} = A_i \f$.

  \param  A : Vector which element will be put in the diagonal.

  \param  DA : Diagonal matrix DA[i][i] = A[i]

\sa diag()
*/

void vpMatrix::createDiagonalMatrix(const vpColVector &A, vpMatrix &DA)
{
  unsigned int rows = A.getRows();
  DA.resize(rows, rows, true);

  for (unsigned int i = 0; i < rows; i++)
    DA[i][i] = A[i];
}

/*!
  Operator that allows to multiply a matrix by a translation vector.
  The matrix should be of dimension (3x3)
  */
vpTranslationVector vpMatrix::operator*(const vpTranslationVector &tv) const
{
  vpTranslationVector t_out;

  if (rowNum != 3 || colNum != 3) {
    throw(vpException(vpException::dimensionError, "Cannot multiply a (%dx%d) matrix by a (%dx%d) translation vector",
                      rowNum, colNum, tv.getRows(), tv.getCols()));
  }

  for (unsigned int j = 0; j < 3; j++)
    t_out[j] = 0;

  for (unsigned int j = 0; j < 3; j++) {
    double tj = tv[j]; // optimization em 5/12/2006
    for (unsigned int i = 0; i < 3; i++) {
      t_out[i] += rowPtrs[i][j] * tj;
    }
  }
  return t_out;
}

/*!
  Operation w = A * v (matrix A is unchanged, v and w are column vectors).
  \sa multMatrixVector() to avoid matrix allocation for each use.
*/
vpColVector vpMatrix::operator*(const vpColVector &v) const
{
  vpColVector v_out;
  vpMatrix::multMatrixVector(*this, v, v_out);
  return v_out;
}

/*!
  Operation w = A * v (v and w are vectors).

  A new matrix won't be allocated for every use of the function
  (Speed gain if used many times with the same result matrix size).

  \sa operator*(const vpColVector &v) const
*/
void vpMatrix::multMatrixVector(const vpMatrix &A, const vpColVector &v, vpColVector &w)
{
  if (A.colNum != v.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot multiply a (%dx%d) matrix by a (%d) column vector",
                      A.getRows(), A.getCols(), v.getRows()));
  }

  if (A.rowNum != w.rowNum)
    w.resize(A.rowNum, false);

  // If available use Lapack only for large matrices
  bool useLapack = (A.rowNum > vpMatrix::m_lapack_min_size || A.colNum > vpMatrix::m_lapack_min_size);
#if !(defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL))
  useLapack = false;
#endif

  if (useLapack) {
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL)
    double alpha = 1.0;
    double beta = 0.0;
    char trans = 't';
    int incr = 1;

    vpMatrix::blas_dgemv(trans, A.colNum, A.rowNum, alpha, A.data, A.colNum, v.data, incr, beta, w.data, incr);
#endif
  }
  else {
    w = 0.0;
    for (unsigned int j = 0; j < A.colNum; j++) {
      double vj = v[j]; // optimization em 5/12/2006
      for (unsigned int i = 0; i < A.rowNum; i++) {
        w[i] += A.rowPtrs[i][j] * vj;
      }
    }
  }
}

//---------------------------------
// Matrix operations.
//---------------------------------

/*!
  Operation C = A * B.

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \sa operator*()
*/
void vpMatrix::mult2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
  if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum))
    C.resize(A.rowNum, B.colNum, false, false);

  if (A.colNum != B.rowNum) {
    throw(vpException(vpException::dimensionError, "Cannot multiply (%dx%d) matrix by (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  // If available use Lapack only for large matrices
  bool useLapack = (A.rowNum > vpMatrix::m_lapack_min_size || A.colNum > vpMatrix::m_lapack_min_size || B.colNum > vpMatrix::m_lapack_min_size);
#if !(defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL))
  useLapack = false;
#endif

  if (useLapack) {
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL)
    const double alpha = 1.0;
    const double beta = 0.0;
    const char trans = 'n';
    vpMatrix::blas_dgemm(trans, trans, B.colNum, A.rowNum, A.colNum, alpha, B.data, B.colNum, A.data, A.colNum, beta,
                         C.data, B.colNum);
#endif
  }
  else {
    // 5/12/06 some "very" simple optimization to avoid indexation
    const unsigned int BcolNum = B.colNum;
    const unsigned int BrowNum = B.rowNum;
    double **BrowPtrs = B.rowPtrs;
    for (unsigned int i = 0; i < A.rowNum; i++) {
      const double *rowptri = A.rowPtrs[i];
      double *ci = C[i];
      for (unsigned int j = 0; j < BcolNum; j++) {
        double s = 0;
        for (unsigned int k = 0; k < BrowNum; k++)
          s += rowptri[k] * BrowPtrs[k][j];
        ci[j] = s;
      }
    }
  }
}

/*!
  \warning This function is provided for compat with previous releases. You
  should rather use the functionalities provided in vpRotationMatrix class.

  Operation C = A * B.

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \exception vpException::dimensionError If matrices are not 3-by-3 dimension.

*/
void vpMatrix::mult2Matrices(const vpMatrix &A, const vpMatrix &B, vpRotationMatrix &C)
{
  if (A.colNum != 3 || A.rowNum != 3 || B.colNum != 3 || B.rowNum != 3) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply (%dx%d) matrix by (%dx%d) matrix as a "
                      "rotation matrix",
                      A.getRows(), A.getCols(), B.getRows(), B.getCols()));
  }
  // 5/12/06 some "very" simple optimization to avoid indexation
  const unsigned int BcolNum = B.colNum;
  const unsigned int BrowNum = B.rowNum;
  double **BrowPtrs = B.rowPtrs;
  for (unsigned int i = 0; i < A.rowNum; i++) {
    const double *rowptri = A.rowPtrs[i];
    double *ci = C[i];
    for (unsigned int j = 0; j < BcolNum; j++) {
      double s = 0;
      for (unsigned int k = 0; k < BrowNum; k++)
        s += rowptri[k] * BrowPtrs[k][j];
      ci[j] = s;
    }
  }
}

/*!
  \warning This function is provided for compat with previous releases. You
  should rather use the functionalities provided in vpHomogeneousMatrix class.

  Operation C = A * B.

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \exception vpException::dimensionError If matrices are not 4-by-4 dimension.

*/
void vpMatrix::mult2Matrices(const vpMatrix &A, const vpMatrix &B, vpHomogeneousMatrix &C)
{
  if (A.colNum != 4 || A.rowNum != 4 || B.colNum != 4 || B.rowNum != 4) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply (%dx%d) matrix by (%dx%d) matrix as a "
                      "rotation matrix",
                      A.getRows(), A.getCols(), B.getRows(), B.getCols()));
  }
  // Considering perfMatrixMultiplication.cpp benchmark,
  // using either MKL, OpenBLAS, or Netlib can slow down this function with respect to the naive code.
  // Lapack usage needs to be validated again.
  // If available use Lapack only for large matrices.
  // Using SSE2 doesn't speed up.
  bool useLapack = (A.rowNum > vpMatrix::m_lapack_min_size || A.colNum > vpMatrix::m_lapack_min_size || B.colNum > vpMatrix::m_lapack_min_size);
#if !(defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL))
  useLapack = false;
#endif

  if (useLapack) {
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL)
    const double alpha = 1.0;
    const double beta = 0.0;
    const char trans = 'n';
    vpMatrix::blas_dgemm(trans, trans, B.colNum, A.rowNum, A.colNum, alpha, B.data, B.colNum, A.data, A.colNum, beta,
                         C.data, B.colNum);
#endif
  }
  else {
    // 5/12/06 some "very" simple optimization to avoid indexation
    const unsigned int BcolNum = B.colNum;
    const unsigned int BrowNum = B.rowNum;
    double **BrowPtrs = B.rowPtrs;
    for (unsigned int i = 0; i < A.rowNum; i++) {
      const double *rowptri = A.rowPtrs[i];
      double *ci = C[i];
      for (unsigned int j = 0; j < BcolNum; j++) {
        double s = 0;
        for (unsigned int k = 0; k < BrowNum; k++)
          s += rowptri[k] * BrowPtrs[k][j];
        ci[j] = s;
      }
    }
  }
}

/*!
  \warning This function is provided for compat with previous releases. You
  should rather use multMatrixVector() that is more explicit.

  Operation C = A * B.

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \sa multMatrixVector()
*/
void vpMatrix::mult2Matrices(const vpMatrix &A, const vpColVector &B, vpColVector &C)
{
  vpMatrix::multMatrixVector(A, B, C);
}

/*!
  Operation C = A * B (A is unchanged).
  \sa mult2Matrices() to avoid matrix allocation for each use.
*/
vpMatrix vpMatrix::operator*(const vpMatrix &B) const
{
  vpMatrix C;

  vpMatrix::mult2Matrices(*this, B, C);

  return C;
}

/*!
  Operator that allow to multiply a matrix by a rotation matrix.
  The matrix should be of dimension m-by-3.
*/
vpMatrix vpMatrix::operator*(const vpRotationMatrix &R) const
{
  if (colNum != R.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot multiply (%dx%d) matrix by (3x3) rotation matrix", rowNum,
                      colNum));
  }
  vpMatrix C;
  C.resize(rowNum, 3, false, false);

  unsigned int RcolNum = R.getCols();
  unsigned int RrowNum = R.getRows();
  for (unsigned int i = 0; i < rowNum; i++) {
    double *rowptri = rowPtrs[i];
    double *ci = C[i];
    for (unsigned int j = 0; j < RcolNum; j++) {
      double s = 0;
      for (unsigned int k = 0; k < RrowNum; k++)
        s += rowptri[k] * R[k][j];
      ci[j] = s;
    }
  }

  return C;
}

/*!
  Operator that allow to multiply a matrix by a homogeneous matrix.
  The matrix should be of dimension m-by-4.
*/
vpMatrix vpMatrix::operator*(const vpHomogeneousMatrix &M) const
{
  if (colNum != M.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot multiply (%dx%d) matrix by (3x3) rotation matrix", rowNum,
                      colNum));
  }
  vpMatrix C;
  C.resize(rowNum, 4, false, false);

  const unsigned int McolNum = M.getCols();
  const unsigned int MrowNum = M.getRows();
  for (unsigned int i = 0; i < rowNum; i++) {
    const double *rowptri = rowPtrs[i];
    double *ci = C[i];
    for (unsigned int j = 0; j < McolNum; j++) {
      double s = 0;
      for (unsigned int k = 0; k < MrowNum; k++)
        s += rowptri[k] * M[k][j];
      ci[j] = s;
    }
  }

  return C;
}

/*!
  Operator that allow to multiply a matrix by a velocity twist matrix.
  The matrix should be of dimension m-by-6.
*/
vpMatrix vpMatrix::operator*(const vpVelocityTwistMatrix &V) const
{
  if (colNum != V.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot multiply (%dx%d) matrix by (6x6) velocity twist matrix",
                      rowNum, colNum));
  }
  vpMatrix M;
  M.resize(rowNum, 6, false, false);

  // Considering perfMatrixMultiplication.cpp benchmark,
  // using either MKL, OpenBLAS, or Netlib can slow down this function with respect to the naive code.
  // Lapack usage needs to be validated again.
  // If available use Lapack only for large matrices.
  // Speed up obtained using SSE2.
  bool useLapack = (rowNum > vpMatrix::m_lapack_min_size || colNum > vpMatrix::m_lapack_min_size || V.colNum > vpMatrix::m_lapack_min_size);
#if !(defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL))
  useLapack = false;
#endif

  if (useLapack) {
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL)
    const double alpha = 1.0;
    const double beta = 0.0;
    const char trans = 'n';
    vpMatrix::blas_dgemm(trans, trans, V.colNum, rowNum, colNum, alpha, V.data, V.colNum, data, colNum, beta,
                         M.data, M.colNum);
#endif
  }
  else {
    SimdMatMulTwist(data, rowNum, V.data, M.data);
  }

  return M;
}

/*!
  Operator that allow to multiply a matrix by a force/torque twist matrix.
  The matrix should be of dimension m-by-6.
*/
vpMatrix vpMatrix::operator*(const vpForceTwistMatrix &V) const
{
  if (colNum != V.getRows()) {
    throw(vpException(vpException::dimensionError, "Cannot multiply (%dx%d) matrix by (6x6) force/torque twist matrix",
                      rowNum, colNum));
  }
  vpMatrix M;
  M.resize(rowNum, 6, false, false);

  // Considering perfMatrixMultiplication.cpp benchmark,
  // using either MKL, OpenBLAS, or Netlib can slow down this function with respect to the naive code.
  // Lapack usage needs to be validated again.
  // If available use Lapack only for large matrices.
  // Speed up obtained using SSE2.
  bool useLapack = (rowNum > vpMatrix::m_lapack_min_size || colNum > vpMatrix::m_lapack_min_size || V.getCols() > vpMatrix::m_lapack_min_size);
#if !(defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL))
  useLapack = false;
#endif

  if (useLapack) {
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL)
    const double alpha = 1.0;
    const double beta = 0.0;
    const char trans = 'n';
    vpMatrix::blas_dgemm(trans, trans, V.getCols(), rowNum, colNum, alpha, V.data, V.getCols(), data, colNum, beta,
                         M.data, M.colNum);
#endif
  }
  else {
    SimdMatMulTwist(data, rowNum, V.data, M.data);
  }

  return M;
}

/*!
Operation C = A*wA + B*wB

The result is placed in the third parameter C and not returned.
A new matrix won't be allocated for every use of the function
(Speed gain if used many times with the same result matrix size).

\sa operator+()
*/

void vpMatrix::add2WeightedMatrices(const vpMatrix &A, const double &wA, const vpMatrix &B, const double &wB,
                                    vpMatrix &C)
{
  if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum))
    C.resize(A.rowNum, B.colNum, false, false);

  if ((A.colNum != B.getCols()) || (A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot add (%dx%d) matrix with (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  double **ArowPtrs = A.rowPtrs;
  double **BrowPtrs = B.rowPtrs;
  double **CrowPtrs = C.rowPtrs;

  for (unsigned int i = 0; i < A.rowNum; i++)
    for (unsigned int j = 0; j < A.colNum; j++)
      CrowPtrs[i][j] = wB * BrowPtrs[i][j] + wA * ArowPtrs[i][j];
}

/*!
  Operation C = A + B.

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \sa operator+()
*/
void vpMatrix::add2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
  if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum))
    C.resize(A.rowNum, B.colNum, false, false);

  if ((A.colNum != B.getCols()) || (A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot add (%dx%d) matrix with (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  double **ArowPtrs = A.rowPtrs;
  double **BrowPtrs = B.rowPtrs;
  double **CrowPtrs = C.rowPtrs;

  for (unsigned int i = 0; i < A.rowNum; i++) {
    for (unsigned int j = 0; j < A.colNum; j++) {
      CrowPtrs[i][j] = BrowPtrs[i][j] + ArowPtrs[i][j];
    }
  }
}

/*!
  \warning This function is provided for compat with previous releases. You
  should rather use the functionalities provided in vpColVector class.

  Operation C = A + B.

  The result is placed in the third parameter C and not returned.
  A new vector won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \sa vpColVector::operator+()
*/
void vpMatrix::add2Matrices(const vpColVector &A, const vpColVector &B, vpColVector &C)
{
  if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum))
    C.resize(A.rowNum);

  if ((A.colNum != B.getCols()) || (A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot add (%dx%d) matrix with (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  double **ArowPtrs = A.rowPtrs;
  double **BrowPtrs = B.rowPtrs;
  double **CrowPtrs = C.rowPtrs;

  for (unsigned int i = 0; i < A.rowNum; i++) {
    for (unsigned int j = 0; j < A.colNum; j++) {
      CrowPtrs[i][j] = BrowPtrs[i][j] + ArowPtrs[i][j];
    }
  }
}

/*!
  Operation C = A + B (A is unchanged).
  \sa add2Matrices() to avoid matrix allocation for each use.
*/
vpMatrix vpMatrix::operator+(const vpMatrix &B) const
{
  vpMatrix C;
  vpMatrix::add2Matrices(*this, B, C);
  return C;
}

/*!
  \warning This function is provided for compat with previous releases. You
  should rather use the functionalities provided in vpColVector class.

  Operation C = A - B on column vectors.

  The result is placed in the third parameter C and not returned.
  A new vector won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \exception vpException::dimensionError If A and B vectors have not the same
  size.

  \sa vpColVector::operator-()
*/
void vpMatrix::sub2Matrices(const vpColVector &A, const vpColVector &B, vpColVector &C)
{
  if ((A.rowNum != C.rowNum) || (A.colNum != C.colNum))
    C.resize(A.rowNum);

  if ((A.colNum != B.getCols()) || (A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot substract (%dx%d) matrix to (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  double **ArowPtrs = A.rowPtrs;
  double **BrowPtrs = B.rowPtrs;
  double **CrowPtrs = C.rowPtrs;

  for (unsigned int i = 0; i < A.rowNum; i++) {
    for (unsigned int j = 0; j < A.colNum; j++) {
      CrowPtrs[i][j] = ArowPtrs[i][j] - BrowPtrs[i][j];
    }
  }
}

/*!
  Operation C = A - B.

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \exception vpException::dimensionError If A and B matrices have not the same
  size.

  \sa operator-()
*/
void vpMatrix::sub2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
  if ((A.rowNum != C.rowNum) || (A.colNum != C.colNum))
    C.resize(A.rowNum, A.colNum, false, false);

  if ((A.colNum != B.getCols()) || (A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot substract (%dx%d) matrix to (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  double **ArowPtrs = A.rowPtrs;
  double **BrowPtrs = B.rowPtrs;
  double **CrowPtrs = C.rowPtrs;

  for (unsigned int i = 0; i < A.rowNum; i++) {
    for (unsigned int j = 0; j < A.colNum; j++) {
      CrowPtrs[i][j] = ArowPtrs[i][j] - BrowPtrs[i][j];
    }
  }
}

/*!
  Operation C = A - B (A is unchanged).
  \sa sub2Matrices() to avoid matrix allocation for each use.
*/
vpMatrix vpMatrix::operator-(const vpMatrix &B) const
{
  vpMatrix C;
  vpMatrix::sub2Matrices(*this, B, C);
  return C;
}

//! Operation A = A + B

vpMatrix &vpMatrix::operator+=(const vpMatrix &B)
{
  if ((colNum != B.getCols()) || (rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot add (%dx%d) matrix to (%dx%d) matrix", rowNum, colNum,
                      B.getRows(), B.getCols()));
  }

  double **BrowPtrs = B.rowPtrs;

  for (unsigned int i = 0; i < rowNum; i++)
    for (unsigned int j = 0; j < colNum; j++)
      rowPtrs[i][j] += BrowPtrs[i][j];

  return *this;
}

//! Operation A = A - B
vpMatrix &vpMatrix::operator-=(const vpMatrix &B)
{
  if ((colNum != B.getCols()) || (rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot substract (%dx%d) matrix to (%dx%d) matrix", rowNum, colNum,
                      B.getRows(), B.getCols()));
  }

  double **BrowPtrs = B.rowPtrs;
  for (unsigned int i = 0; i < rowNum; i++)
    for (unsigned int j = 0; j < colNum; j++)
      rowPtrs[i][j] -= BrowPtrs[i][j];

  return *this;
}

/*!
  Operation C = -A.

  The result is placed in the second parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (Speed gain if used many times with the same result matrix size).

  \sa operator-(void)
*/
void vpMatrix::negateMatrix(const vpMatrix &A, vpMatrix &C)
{
  if ((A.rowNum != C.rowNum) || (A.colNum != C.colNum))
    C.resize(A.rowNum, A.colNum, false, false);

  double **ArowPtrs = A.rowPtrs;
  double **CrowPtrs = C.rowPtrs;

  // 	t0=vpTime::measureTimeMicros();
  for (unsigned int i = 0; i < A.rowNum; i++)
    for (unsigned int j = 0; j < A.colNum; j++)
      CrowPtrs[i][j] = -ArowPtrs[i][j];
}

/*!
  Operation C = -A (A is unchanged).
  \sa negateMatrix() to avoid matrix allocation for each use.
*/
vpMatrix vpMatrix::operator-() const // negate
{
  vpMatrix C;
  vpMatrix::negateMatrix(*this, C);
  return C;
}

double vpMatrix::sum() const
{
  double s = 0.0;
  for (unsigned int i = 0; i < rowNum; i++) {
    for (unsigned int j = 0; j < colNum; j++) {
      s += rowPtrs[i][j];
    }
  }

  return s;
}

//---------------------------------
// Matrix/vector operations.
//---------------------------------

//---------------------------------
// Matrix/real operations.
//---------------------------------

/*!
  \relates vpMatrix
  Allow to multiply a scalar by a matrix.
*/
vpMatrix operator*(const double &x, const vpMatrix &B)
{
  if (std::fabs(x - 1.) < std::numeric_limits<double>::epsilon()) {
    return B;
  }

  unsigned int Brow = B.getRows();
  unsigned int Bcol = B.getCols();

  vpMatrix C;
  C.resize(Brow, Bcol, false, false);

  for (unsigned int i = 0; i < Brow; i++)
    for (unsigned int j = 0; j < Bcol; j++)
      C[i][j] = B[i][j] * x;

  return C;
}

/*!
   Operator that allows to multiply all the elements of a matrix
   by a scalar.
 */
vpMatrix vpMatrix::operator*(double x) const
{
  if (std::fabs(x - 1.) < std::numeric_limits<double>::epsilon()) {
    return (*this);
  }

  vpMatrix M;
  M.resize(rowNum, colNum, false, false);

  for (unsigned int i = 0; i < rowNum; i++)
    for (unsigned int j = 0; j < colNum; j++)
      M[i][j] = rowPtrs[i][j] * x;

  return M;
}

//! Cij = Aij / x (A is unchanged)
vpMatrix vpMatrix::operator/(double x) const
{
  if (std::fabs(x - 1.) < std::numeric_limits<double>::epsilon()) {
    return (*this);
  }

  if (std::fabs(x) < std::numeric_limits<double>::epsilon()) {
    throw vpException(vpException::divideByZeroError, "Divide matrix by zero scalar");
  }

  vpMatrix C;
  C.resize(rowNum, colNum, false, false);

  double xinv = 1 / x;

  for (unsigned int i = 0; i < rowNum; i++)
    for (unsigned int j = 0; j < colNum; j++)
      C[i][j] = rowPtrs[i][j] * xinv;

  return C;
}

//! Add x to all the element of the matrix : Aij = Aij + x
vpMatrix &vpMatrix::operator+=(double x)
{
  for (unsigned int i = 0; i < rowNum; i++)
    for (unsigned int j = 0; j < colNum; j++)
      rowPtrs[i][j] += x;

  return *this;
}

//! Substract x to all the element of the matrix : Aij = Aij - x
vpMatrix &vpMatrix::operator-=(double x)
{
  for (unsigned int i = 0; i < rowNum; i++)
    for (unsigned int j = 0; j < colNum; j++)
      rowPtrs[i][j] -= x;

  return *this;
}

/*!
   Operator that allows to multiply all the elements of a matrix
   by a scalar.
 */
vpMatrix &vpMatrix::operator*=(double x)
{
  if (std::fabs(x - 1.) < std::numeric_limits<double>::epsilon()) {
    return *this;
  }

  for (unsigned int i = 0; i < rowNum; i++)
    for (unsigned int j = 0; j < colNum; j++)
      rowPtrs[i][j] *= x;

  return *this;
}

//! Divide  all the element of the matrix by x : Aij = Aij / x
vpMatrix &vpMatrix::operator/=(double x)
{
  if (std::fabs(x - 1.) < std::numeric_limits<double>::epsilon()) {
    return *this;
  }

  if (std::fabs(x) < std::numeric_limits<double>::epsilon())
    throw vpException(vpException::divideByZeroError, "Divide matrix by zero scalar");

  double xinv = 1 / x;

  for (unsigned int i = 0; i < rowNum; i++)
    for (unsigned int j = 0; j < colNum; j++)
      rowPtrs[i][j] *= xinv;

  return *this;
}

//----------------------------------------------------------------
// Matrix Operation
//----------------------------------------------------------------

/*!
  Stacks columns of a matrix in a vector.
  \param out : a vpColVector.
*/
void vpMatrix::stackColumns(vpColVector &out)
{
  if ((out.rowNum != colNum * rowNum) || (out.colNum != 1))
    out.resize(colNum * rowNum, false, false);

  double *optr = out.data;
  for (unsigned int j = 0; j < colNum; j++) {
    for (unsigned int i = 0; i < rowNum; i++) {
      *(optr++) = rowPtrs[i][j];
    }
  }
}

/*!
  Stacks columns of a matrix in a vector.
  \return a vpColVector.
*/
vpColVector vpMatrix::stackColumns()
{
  vpColVector out(colNum * rowNum);
  stackColumns(out);
  return out;
}

/*!
  Stacks rows of a matrix in a vector
  \param out : a vpRowVector.
*/
void vpMatrix::stackRows(vpRowVector &out)
{
  if ((out.getRows() != 1) || (out.getCols() != colNum * rowNum))
    out.resize(colNum * rowNum, false, false);

  memcpy(out.data, data, sizeof(double)*out.getCols());
}
/*!
  Stacks rows of a matrix in a vector.
 \return a vpRowVector.
*/
vpRowVector vpMatrix::stackRows()
{
  vpRowVector out(colNum * rowNum);
  stackRows(out);
  return out;
}

/*!
  Compute the Hadamard product (element wise matrix multiplication).
  \param m : Second matrix;
  \return m1.hadamard(m2) The Hadamard product :
  \f$ m1 \circ m2 = (m1 \circ m2)_{i,j} = (m1)_{i,j} (m2)_{i,j} \f$
*/
vpMatrix vpMatrix::hadamard(const vpMatrix &m) const
{
  if (m.getRows() != rowNum || m.getCols() != colNum) {
    throw(vpException(vpException::dimensionError, "In Hadamard product: bad dimension of input matrix"));
  }

  vpMatrix out;
  out.resize(rowNum, colNum, false, false);

  SimdVectorHadamard(data, m.data, dsize, out.data);

  return out;
}

/*!
  Compute Kronecker product matrix.
  \param m1 : vpMatrix;
  \param m2 : vpMatrix;
  \param out : The kronecker product : \f$ m1 \otimes m2 \f$
*/
void vpMatrix::kron(const vpMatrix &m1, const vpMatrix &m2, vpMatrix &out)
{
  unsigned int r1 = m1.getRows();
  unsigned int c1 = m1.getCols();
  unsigned int r2 = m2.getRows();
  unsigned int c2 = m2.getCols();

  out.resize(r1*r2, c1*c2, false, false);

  for (unsigned int r = 0; r < r1; r++) {
    for (unsigned int c = 0; c < c1; c++) {
      double alpha = m1[r][c];
      double *m2ptr = m2[0];
      unsigned int roffset = r * r2;
      unsigned int coffset = c * c2;
      for (unsigned int rr = 0; rr < r2; rr++) {
        for (unsigned int cc = 0; cc < c2; cc++) {
          out[roffset + rr][coffset + cc] = alpha * *(m2ptr++);
        }
      }
    }
  }
}

/*!
  Compute Kronecker product matrix.
  \param m : vpMatrix.
  \param out : If m1.kron(m2) out contains the kronecker product's result :
  \f$ m1 \otimes m2 \f$.
*/
void vpMatrix::kron(const vpMatrix &m, vpMatrix &out) const { kron(*this, m, out); }

/*!
  Compute Kronecker product matrix.
  \param m1 : vpMatrix;
  \param m2 : vpMatrix;
  \return The kronecker product : \f$ m1 \otimes m2 \f$
*/
vpMatrix vpMatrix::kron(const vpMatrix &m1, const vpMatrix &m2)
{
  unsigned int r1 = m1.getRows();
  unsigned int c1 = m1.getCols();
  unsigned int r2 = m2.getRows();
  unsigned int c2 = m2.getCols();

  vpMatrix out;
  out.resize(r1 * r2, c1 * c2, false, false);

  for (unsigned int r = 0; r < r1; r++) {
    for (unsigned int c = 0; c < c1; c++) {
      double alpha = m1[r][c];
      double *m2ptr = m2[0];
      unsigned int roffset = r * r2;
      unsigned int coffset = c * c2;
      for (unsigned int rr = 0; rr < r2; rr++) {
        for (unsigned int cc = 0; cc < c2; cc++) {
          out[roffset + rr][coffset + cc] = alpha * *(m2ptr++);
        }
      }
    }
  }
  return out;
}

/*!
  Compute Kronecker product matrix.
  \param m : vpMatrix;
  \return m1.kron(m2) The kronecker product : \f$ m1 \otimes m2 \f$
*/
vpMatrix vpMatrix::kron(const vpMatrix &m) const { return kron(*this, m); }

/*!

  Solve a linear system \f$ A X = B \f$ using Singular Value
  Decomposition (SVD).

  Non destructive wrt. A and B.

  \param b : Vector\f$ B \f$.

  \param x : Vector \f$ X \f$.

  Here an example:
\code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

int main()
{
vpMatrix A(3,3);

A[0][0] = 4.64;
A[0][1] = 0.288;
A[0][2] = -0.384;

A[1][0] = 0.288;
A[1][1] = 7.3296;
A[1][2] = 2.2272;

A[2][0] = -0.384;
A[2][1] = 2.2272;
A[2][2] = 6.0304;

vpColVector X(3), B(3);
B[0] = 1;
B[1] = 2;
B[2] = 3;

A.solveBySVD(B, X);

// Obtained values of X
// X[0] = 0.2468;
// X[1] = 0.120782;
// X[2] = 0.468587;

std::cout << "X:\n" << X << std::endl;
}
\endcode

\sa solveBySVD(const vpColVector &)
*/
void vpMatrix::solveBySVD(const vpColVector &b, vpColVector &x) const { x = pseudoInverse(1e-6) * b; }

/*!

  Solve a linear system \f$ A X = B \f$ using Singular Value
  Decomposition (SVD).

  Non destructive wrt. A and B.

  \param B : Vector\f$ B \f$.

  \return Vector \f$ X \f$.

  Here an example:
\code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

int main()
{
vpMatrix A(3,3);

A[0][0] = 4.64;
A[0][1] = 0.288;
A[0][2] = -0.384;

A[1][0] = 0.288;
A[1][1] = 7.3296;
A[1][2] = 2.2272;

A[2][0] = -0.384;
A[2][1] = 2.2272;
A[2][2] = 6.0304;

vpColVector X(3), B(3);
B[0] = 1;
B[1] = 2;
B[2] = 3;

X = A.solveBySVD(B);
// Obtained values of X
// X[0] = 0.2468;
// X[1] = 0.120782;
// X[2] = 0.468587;

std::cout << "X:\n" << X << std::endl;
}
\endcode

\sa solveBySVD(const vpColVector &, vpColVector &)
*/
vpColVector vpMatrix::solveBySVD(const vpColVector &B) const
{
  vpColVector X(colNum);

  solveBySVD(B, X);
  return X;
}

/*!

  Matrix singular value decomposition (SVD).

  This function calls the first following function that is available:
  - svdLapack() if Lapack 3rd party is installed
  - svdEigen3() if Eigen3 3rd party is installed
  - svdOpenCV() if OpenCV 3rd party is installed.

  If none of these previous 3rd parties is installed, we use by default
  svdLapack() with a Lapack built-in version.

  Given matrix \f$M\f$, this function computes it singular value decomposition
  such as

  \f[ M = U \Sigma V^{\top} \f]

  \warning This method is destructive wrt. to the matrix \f$ M \f$ to
  decompose. You should make a COPY of that matrix if needed.

  \param w : Vector of singular values: \f$ \Sigma = diag(w) \f$.

  \param V : Matrix \f$ V \f$.

  \return Matrix \f$ U \f$.

  \note The singular values are ordered in decreasing
  fashion in \e w. It means that the highest singular value is in \e w[0].

  Here an example of SVD decomposition of a non square Matrix M.

\code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix M(3,2);
  M[0][0] = 1;   M[0][1] = 6;
  M[1][0] = 2;   M[1][1] = 8;
  M[2][0] = 0.5; M[2][1] = 9;

  vpColVector w;
  vpMatrix V, Sigma, U = M;

  U.svd(w, V);

  // Construct the diagonal matrix from the singular values
  Sigma.diag(w);

  // Reconstruct the initial matrix using the decomposition
  vpMatrix Mrec =  U * Sigma * V.t();

  // Here, Mrec is obtained equal to the initial value of M
  // Mrec[0][0] = 1;   Mrec[0][1] = 6;
  // Mrec[1][0] = 2;   Mrec[1][1] = 8;
  // Mrec[2][0] = 0.5; Mrec[2][1] = 9;

  std::cout << "Reconstructed M matrix: \n" << Mrec << std::endl;
}
  \endcode

  \sa svdLapack(), svdEigen3(), svdOpenCV()
*/
void vpMatrix::svd(vpColVector &w, vpMatrix &V)
{
#if defined(VISP_HAVE_LAPACK)
  svdLapack(w, V);
#elif defined(VISP_HAVE_EIGEN3)
  svdEigen3(w, V);
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
  svdOpenCV(w, V);
#else
  (void)w;
  (void)V;
  throw(vpException(vpException::fatalError, "Cannot compute SVD. Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ and return the rank of the matrix.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The rank of the matrix.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  unsigned int rank = A.pseudoInverse(A_p);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank: " << rank << std::endl;
}
  \endcode

  Once build, the previous example produces the following output:
  \code
A: [2,3]=
   2  3  5
  -4  2  3
A^+ (pseudo-inverse): [3,2]=
   0.117899 -0.190782
   0.065380  0.039657
   0.113612  0.052518
Rank: 2
  \endcode
*/
unsigned int vpMatrix::pseudoInverse(vpMatrix &Ap, double svThreshold) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(Ap, svThreshold);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(Ap, svThreshold);
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(Ap, svThreshold);
#else
  (void)Ap;
  (void)svThreshold;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                                             "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ and return the rank of the matrix.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param[in] rank_in : Known rank of the matrix.

  \return The rank of the matrix.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  // This matrix rank is 2
  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  int rank_in = 2;
  int rank_out = A.pseudoInverse(A_p, rank_in);
  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_out << std::endl;
}
  \endcode

  Once build, the previous example produces the following output:
  \code
A: [2,3]=
   2  3  5
  -4  2  3
A^+ (pseudo-inverse): [3,2]=
   0.117899 -0.190782
   0.065380  0.039657
   0.113612  0.052518
Rank in : 2
Rank out: 2
  \endcode
*/
int vpMatrix::pseudoInverse(vpMatrix &Ap, int rank_in) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(Ap, rank_in);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(Ap, rank_in);
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(Ap, rank_in);
#else
  (void)Ap;
  (void)svThreshold;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                                             "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!
  Compute and return the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n
  matrix \f$\bf A\f$.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The Moore-Penros pseudo inverse \f$ A^+ \f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p = A.pseudoInverse();

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
}
  \endcode

  Once build, the previous example produces the following output:
  \code
A: [2,3]=
   2  3  5
  -4  2  3
A^+ (pseudo-inverse): [3,2]=
   0.117899 -0.190782
   0.065380  0.039657
   0.113612  0.052518
  \endcode

*/
vpMatrix vpMatrix::pseudoInverse(double svThreshold) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(svThreshold);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(svThreshold);
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(svThreshold);
#else
  (void)svThreshold;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                                             "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!
  Compute and return the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n
  matrix \f$\bf A\f$.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param[in] rank_in : Known rank of the matrix.

  \return The Moore-Penros pseudo inverse \f$ A^+ \f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  // This matrix rank is 2
  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  int rank_in = 2;
  vpMatrix A_p = A.pseudoInverseLapack(rank_in);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
}
  \endcode

  Once build, the previous example produces the following output:
  \code
A: [2,3]=
   2  3  5
  -4  2  3
A^+ (pseudo-inverse): [3,2]=
   0.117899 -0.190782
   0.065380  0.039657
   0.113612  0.052518
  \endcode

*/
vpMatrix vpMatrix::pseudoInverse(int rank_in) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(rank_in);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(rank_in);
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(rank_in);
#else
  (void)svThreshold;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                                             "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#if defined(VISP_HAVE_LAPACK)
/*!
  Compute and return the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n
  matrix \f$\bf A\f$ using Lapack 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The Moore-Penros pseudo inverse \f$ A^+ \f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p = A.pseudoInverseLapack();

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
}
  \endcode

  \sa pseudoInverse(double) const
*/
vpMatrix vpMatrix::pseudoInverseLapack(double svThreshold) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;

  vpMatrix U, V, Ap;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdLapack(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, NULL, NULL, NULL, NULL);

  return Ap;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ and return the rank of the matrix using Lapack 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The rank of the matrix.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  unsigned int rank = A.pseudoInverseLapack(A_p);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank: " << rank << std::endl;
}
  \endcode

  \sa pseudoInverse(vpMatrix &, double) const
*/
unsigned int vpMatrix::pseudoInverseLapack(vpMatrix &Ap, double svThreshold) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;

  vpMatrix U, V;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdLapack(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, NULL, NULL, NULL, NULL);

  return static_cast<unsigned int>(rank_out);
}
/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values and return the rank of the matrix using
  Lapack 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  vpColVector sv;
  unsigned int rank = A.pseudoInverseLapack(A_p, sv);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");

  std::cout << "Rank: " << rank << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
}
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, double) const
*/
unsigned int vpMatrix::pseudoInverseLapack(vpMatrix &Ap, vpColVector &sv, double svThreshold) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;

  vpMatrix U, V;
  vpColVector sv_;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdLapack(sv_, V);

  compute_pseudo_inverse(U, sv_, V, nrows, ncols, svThreshold,  Ap, rank_out, NULL, NULL, NULL, NULL);

  // Remove singular values equal to the one that corresponds to the lines of 0
  // introduced when m < n
  memcpy(sv.data, sv_.data, sv.size() * sizeof(double));

  return static_cast<unsigned int>(rank_out);
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$, \f$\mbox{Im}(A^T)\f$ and
  \f$\mbox{Ker}(A)\f$ and return the rank of the matrix using Lapack 3rd
  party.

  \warning To inverse a square n-by-n matrix, you have to use rather
  inverseByLU(), inverseByCholesky(), or inverseByQR() that are kwown as faster.

  Using singular value decomposition, we have:

  \f[
  {\bf A}_{m\times n} = {\bf U}_{m\times m} \; {\bf S}_{m\times n} \; {\bf
  V^\top}_{n\times n} \f] \f[
  {\bf A}_{m\times n} = \left[\begin{array}{ccc}\mbox{Im} ({\bf A}) & | &
  \mbox{Ker} ({\bf A}^\top) \end{array} \right] {\bf S}_{m\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{m\times n}\f$ corresponds to the matrix
  \f$A\f$ singular values.

  This equation could be reformulated in a minimal way:
  \f[
  {\bf A}_{m\times n} = \mbox{Im} ({\bf A}) \; {\bf S}_{r\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{r\times n}\f$ corresponds to the matrix
  \f$A\f$ first r singular values.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param Ap: The Moore-Penros pseudo inverse \f$ {\bf A}^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param svThreshold: Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \param kerAt: The matrix that contains the null space (kernel) of \f$\bf
  A\f$ defined by the matrix \f${\bf X}^T\f$. If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is (0, n), otherwise the dimension is (n-r,
  n). This matrix is thus the transpose of \f$\mbox{Ker}({\bf A})\f$.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpColVector sv;
  vpMatrix A_p, imA, imAt, kerAt;
  unsigned int rank = A.pseudoInverseLapack(A_p, sv, 1e-6, imA, imAt, kerAt);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank: " << rank << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  imA.print(std::cout, 10, "Im(A): ");
  imAt.print(std::cout, 10, "Im(A^T): ");

  if (kerAt.size()) {
    kerAt.t().print(std::cout, 10, "Ker(A): ");
  }
  else {
    std::cout << "Ker(A) empty " << std::endl;
  }

  // Reconstruct matrix A from ImA, ImAt, KerAt
  vpMatrix S(rank, A.getCols());
  for(unsigned int i = 0; i< rank; i++)
    S[i][i] = sv[i];
  vpMatrix Vt(A.getCols(), A.getCols());
  Vt.insert(imAt.t(), 0, 0);
  Vt.insert(kerAt, rank, 0);
  (imA * S * Vt).print(std::cout, 10, "Im(A) * S * [Im(A^T) | Ker(A)]^T:");
}
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, double, vpMatrix &, vpMatrix &, vpMatrix &) const
*/
unsigned int vpMatrix::pseudoInverseLapack(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA,
                                           vpMatrix &imAt, vpMatrix &kerAt) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  vpMatrix U, V;
  vpColVector sv_;

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdLapack(sv_, V);

  compute_pseudo_inverse(U, sv_, V, nrows, ncols, svThreshold,  Ap, rank_out, NULL, &imA, &imAt, &kerAt);

  // Remove singular values equal to the one that corresponds to the lines of 0
  // introduced when m < n
  memcpy(sv.data, sv_.data, sv.size() * sizeof(double));

  return static_cast<unsigned int>(rank_out);
}

/*!
  Compute and return the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n
  matrix \f$\bf A\f$ using Lapack 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param[in] rank_in : Known rank of the matrix.
  \return The Moore-Penros pseudo inverse \f$ A^+ \f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  // This matrix rank is 2
  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  int rank_in = 2;
  vpMatrix A_p = A.pseudoInverseLapack(rank_in);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  return 0;
}
  \endcode

  \sa pseudoInverse(int) const
*/
vpMatrix vpMatrix::pseudoInverseLapack(int rank_in) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;

  vpMatrix U, V, Ap;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdLapack(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, NULL, NULL, NULL);

  return Ap;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ and return the rank of the matrix using Lapack 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param[out] Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param[in] rank_in : Known rank of the matrix.

  \return The computed rank of the matrix.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  // This matrix rank is 2
  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  int rank_in = 2;
  int rank_out = A.pseudoInverseLapack(A_p, rank_in);
  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_out << std::endl;
  return 0;
}
  \endcode

  \sa pseudoInverse(vpMatrix &, int) const
*/
int vpMatrix::pseudoInverseLapack(vpMatrix &Ap, int rank_in) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;

  vpMatrix U, V;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdLapack(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, NULL, NULL, NULL);

  return rank_out;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values and return the rank of the matrix using
  Lapack 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  // This matrix rank is 2
  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  vpColVector sv;
  int rank_in = 2;

  int rank_out = A.pseudoInverseLapack(A_p, sv, rank_in);
  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_out << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  return 0;
}
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, int) const
*/
int vpMatrix::pseudoInverseLapack(vpMatrix &Ap, vpColVector &sv, int rank_in) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;

  vpMatrix U, V;
  vpColVector sv_;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdLapack(sv_, V);

  compute_pseudo_inverse(U, sv_, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, NULL, NULL, NULL);

  // Remove singular values equal to the one that corresponds to the lines of 0
  // introduced when m < n
  memcpy(sv.data, sv_.data, sv.size() * sizeof(double));

  return rank_out;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$, \f$\mbox{Im}(A^T)\f$ and
  \f$\mbox{Ker}(A)\f$ and return the rank of the matrix using Lapack 3rd
  party.

  \warning To inverse a square n-by-n matrix, you have to use rather
  inverseByLU(), inverseByCholesky(), or inverseByQR() that are kwown as faster.

  Using singular value decomposition, we have:

  \f[
  {\bf A}_{m\times n} = {\bf U}_{m\times m} \; {\bf S}_{m\times n} \; {\bf
  V^\top}_{n\times n} \f] \f[
  {\bf A}_{m\times n} = \left[\begin{array}{ccc}\mbox{Im} ({\bf A}) & | &
  \mbox{Ker} ({\bf A}^\top) \end{array} \right] {\bf S}_{m\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{m\times n}\f$ corresponds to the matrix
  \f$A\f$ singular values.

  This equation could be reformulated in a minimal way:
  \f[
  {\bf A}_{m\times n} = \mbox{Im} ({\bf A}) \; {\bf S}_{r\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{r\times n}\f$ corresponds to the matrix
  \f$A\f$ first r singular values.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param Ap: The Moore-Penros pseudo inverse \f$ {\bf A}^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \param kerAt: The matrix that contains the null space (kernel) of \f$\bf
  A\f$ defined by the matrix \f${\bf X}^T\f$. If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is (0, n), otherwise the dimension is (n-r,
  n). This matrix is thus the transpose of \f$\mbox{Ker}({\bf A})\f$.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpColVector sv;
  vpMatrix A_p, imA, imAt, kerAt;
  int rank_in = 2;
  int rank_out = A.pseudoInverseLapack(A_p, sv, rank_in, imA, imAt, kerAt);

  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_out << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  imA.print(std::cout, 10, "Im(A): ");
  imAt.print(std::cout, 10, "Im(A^T): ");

  if (kerAt.size()) {
    kerAt.t().print(std::cout, 10, "Ker(A): ");
  }
  else {
    std::cout << "Ker(A) empty " << std::endl;
  }

  // Reconstruct matrix A from ImA, ImAt, KerAt
  vpMatrix S(rank_in, A.getCols());
  for(unsigned int i = 0; i< rank_in; i++)
    S[i][i] = sv[i];
  vpMatrix Vt(A.getCols(), A.getCols());
  Vt.insert(imAt.t(), 0, 0);
  Vt.insert(kerAt, rank_in, 0);
  (imA * S * Vt).print(std::cout, 10, "Im(A) * S * [Im(A^T) | Ker(A)]^T:");
}
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, int, vpMatrix &, vpMatrix &, vpMatrix &) const
*/
int vpMatrix::pseudoInverseLapack(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA,
                                  vpMatrix &imAt, vpMatrix &kerAt) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;
  vpMatrix U, V;
  vpColVector sv_;

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdLapack(sv_, V);

  compute_pseudo_inverse(U, sv_, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, &imA, &imAt, &kerAt);

  // Remove singular values equal to the one that corresponds to the lines of 0
  // introduced when m < n
  memcpy(sv.data, sv_.data, sv.size() * sizeof(double));

  return rank_out;
}

#endif
#if defined(VISP_HAVE_EIGEN3)
/*!
  Compute and return the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n
  matrix \f$\bf A\f$ using Eigen3 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The Moore-Penros pseudo inverse \f$ A^+ \f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p = A.pseudoInverseEigen3();

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
}
  \endcode

  \sa pseudoInverse(double)
*/
vpMatrix vpMatrix::pseudoInverseEigen3(double svThreshold) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;

  vpMatrix U, V, Ap;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdEigen3(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, NULL, NULL, NULL, NULL);

  return Ap;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ and return the rank of the matrix using Eigen3 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The rank of the matrix.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  unsigned int rank = A.pseudoInverseEigen3(A_p);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank: " << rank << std::endl;
}
  \endcode

  \sa pseudoInverse(vpMatrix &, double) const
*/
unsigned int vpMatrix::pseudoInverseEigen3(vpMatrix &Ap, double svThreshold) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;

  vpMatrix U, V;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdEigen3(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, NULL, NULL, NULL, NULL);

  return static_cast<unsigned int>(rank_out);
}
/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values and return the rank of the matrix using
  Eigen3 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  vpColVector sv;
  unsigned int rank = A.pseudoInverseEigen3(A_p, sv);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");

  std::cout << "Rank: " << rank << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
}
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, double) const
*/
unsigned int vpMatrix::pseudoInverseEigen3(vpMatrix &Ap, vpColVector &sv, double svThreshold) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;

  vpMatrix U, V;
  vpColVector sv_;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdEigen3(sv_, V);

  compute_pseudo_inverse(U, sv_, V, nrows, ncols, svThreshold,  Ap, rank_out, NULL, NULL, NULL, NULL);

  // Remove singular values equal to the one that corresponds to the lines of 0
  // introduced when m < n
  memcpy(sv.data, sv_.data, sv.size() * sizeof(double));

  return static_cast<unsigned int>(rank_out);
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$, \f$\mbox{Im}(A^T)\f$ and
  \f$\mbox{Ker}(A)\f$ and return the rank of the matrix using Eigen3 3rd
  party.

  \warning To inverse a square n-by-n matrix, you have to use rather
  inverseByLU(), inverseByCholesky(), or inverseByQR() that are kwown as faster.

  Using singular value decomposition, we have:

  \f[
  {\bf A}_{m\times n} = {\bf U}_{m\times m} \; {\bf S}_{m\times n} \; {\bf
  V^\top}_{n\times n} \f] \f[
  {\bf A}_{m\times n} = \left[\begin{array}{ccc}\mbox{Im} ({\bf A}) & | &
  \mbox{Ker} ({\bf A}^\top) \end{array} \right] {\bf S}_{m\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{m\times n}\f$ corresponds to the matrix
  \f$A\f$ singular values.

  This equation could be reformulated in a minimal way:
  \f[
  {\bf A}_{m\times n} = \mbox{Im} ({\bf A}) \; {\bf S}_{r\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{r\times n}\f$ corresponds to the matrix
  \f$A\f$ first r singular values.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param Ap: The Moore-Penros pseudo inverse \f$ {\bf A}^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param svThreshold: Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \param kerAt: The matrix that contains the null space (kernel) of \f$\bf
  A\f$ defined by the matrix \f${\bf X}^T\f$. If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is (0, n), otherwise the dimension is (n-r,
  n). This matrix is thus the transpose of \f$\mbox{Ker}({\bf A})\f$.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpColVector sv;
  vpMatrix A_p, imA, imAt, kerAt;
  unsigned int rank = A.pseudoInverseEigen3(A_p, sv, 1e-6, imA, imAt, kerAt);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank: " << rank << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  imA.print(std::cout, 10, "Im(A): ");
  imAt.print(std::cout, 10, "Im(A^T): ");

  if (kerAt.size()) {
    kerAt.t().print(std::cout, 10, "Ker(A): ");
  }
  else {
    std::cout << "Ker(A) empty " << std::endl;
  }

  // Reconstruct matrix A from ImA, ImAt, KerAt
  vpMatrix S(rank, A.getCols());
  for(unsigned int i = 0; i< rank; i++)
    S[i][i] = sv[i];
  vpMatrix Vt(A.getCols(), A.getCols());
  Vt.insert(imAt.t(), 0, 0);
  Vt.insert(kerAt, rank, 0);
  (imA * S * Vt).print(std::cout, 10, "Im(A) * S * [Im(A^T) | Ker(A)]^T:");
}
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, double, vpMatrix &, vpMatrix &, vpMatrix &) const
*/
unsigned int vpMatrix::pseudoInverseEigen3(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA,
                                           vpMatrix &imAt, vpMatrix &kerAt) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  vpMatrix U, V;
  vpColVector sv_;

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdEigen3(sv_, V);

  compute_pseudo_inverse(U, sv_, V, nrows, ncols, svThreshold,  Ap, rank_out, NULL, &imA, &imAt, &kerAt);

  // Remove singular values equal to the one that corresponds to the lines of 0
  // introduced when m < n
  memcpy(sv.data, sv_.data, sv.size() * sizeof(double));

  return static_cast<unsigned int>(rank_out);
}

/*!
  Compute and return the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n
  matrix \f$\bf A\f$ using Eigen3 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param[in] rank_in : Known rank of the matrix.
  \return The Moore-Penros pseudo inverse \f$ A^+ \f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  // This matrix rank is 2
  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  int rank_in = 2;
  vpMatrix A_p = A.pseudoInverseEigen3(rank_in);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  return 0;
}
  \endcode

  \sa pseudoInverse(int) const
*/
vpMatrix vpMatrix::pseudoInverseEigen3(int rank_in) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;

  vpMatrix U, V, Ap;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdEigen3(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, NULL, NULL, NULL);

  return Ap;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ and return the rank of the matrix using Eigen3 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param[out] Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param[in] rank_in : Known rank of the matrix.

  \return The computed rank of the matrix.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  // This matrix rank is 2
  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  int rank_in = 2;
  int rank_out = A.pseudoInverseEigen3(A_p, rank_in);
  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_out << std::endl;
  return 0;
}
  \endcode

  \sa pseudoInverse(vpMatrix &, int) const
*/
int vpMatrix::pseudoInverseEigen3(vpMatrix &Ap, int rank_in) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;

  vpMatrix U, V;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdEigen3(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, NULL, NULL, NULL);

  return rank_out;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values and return the rank of the matrix using
  Eigen3 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  // This matrix rank is 2
  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  vpColVector sv;
  int rank_in = 2;

  int rank_out = A.pseudoInverseEigen3(A_p, sv, rank_in);
  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_out << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  return 0;
}
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, int) const
*/
int vpMatrix::pseudoInverseEigen3(vpMatrix &Ap, vpColVector &sv, int rank_in) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;

  vpMatrix U, V;
  vpColVector sv_;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdEigen3(sv_, V);

  compute_pseudo_inverse(U, sv_, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, NULL, NULL, NULL);

  // Remove singular values equal to the one that corresponds to the lines of 0
  // introduced when m < n
  memcpy(sv.data, sv_.data, sv.size() * sizeof(double));

  return rank_out;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$, \f$\mbox{Im}(A^T)\f$ and
  \f$\mbox{Ker}(A)\f$ and return the rank of the matrix using Eigen3 3rd
  party.

  \warning To inverse a square n-by-n matrix, you have to use rather
  inverseByLU(), inverseByCholesky(), or inverseByQR() that are kwown as faster.

  Using singular value decomposition, we have:

  \f[
  {\bf A}_{m\times n} = {\bf U}_{m\times m} \; {\bf S}_{m\times n} \; {\bf
  V^\top}_{n\times n} \f] \f[
  {\bf A}_{m\times n} = \left[\begin{array}{ccc}\mbox{Im} ({\bf A}) & | &
  \mbox{Ker} ({\bf A}^\top) \end{array} \right] {\bf S}_{m\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{m\times n}\f$ corresponds to the matrix
  \f$A\f$ singular values.

  This equation could be reformulated in a minimal way:
  \f[
  {\bf A}_{m\times n} = \mbox{Im} ({\bf A}) \; {\bf S}_{r\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{r\times n}\f$ corresponds to the matrix
  \f$A\f$ first r singular values.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param Ap: The Moore-Penros pseudo inverse \f$ {\bf A}^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \param kerAt: The matrix that contains the null space (kernel) of \f$\bf
  A\f$ defined by the matrix \f${\bf X}^T\f$. If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is (0, n), otherwise the dimension is (n-r,
  n). This matrix is thus the transpose of \f$\mbox{Ker}({\bf A})\f$.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpColVector sv;
  vpMatrix A_p, imA, imAt, kerAt;
  int rank_in = 2;
  int rank_out = A.pseudoInverseEigen3(A_p, sv, rank_in, imA, imAt, kerAt);

  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_out << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  imA.print(std::cout, 10, "Im(A): ");
  imAt.print(std::cout, 10, "Im(A^T): ");

  if (kerAt.size()) {
    kerAt.t().print(std::cout, 10, "Ker(A): ");
  }
  else {
    std::cout << "Ker(A) empty " << std::endl;
  }

  // Reconstruct matrix A from ImA, ImAt, KerAt
  vpMatrix S(rank_in, A.getCols());
  for(unsigned int i = 0; i< rank_in; i++)
    S[i][i] = sv[i];
  vpMatrix Vt(A.getCols(), A.getCols());
  Vt.insert(imAt.t(), 0, 0);
  Vt.insert(kerAt, rank_in, 0);
  (imA * S * Vt).print(std::cout, 10, "Im(A) * S * [Im(A^T) | Ker(A)]^T:");
}
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, int, vpMatrix &, vpMatrix &, vpMatrix &) const
*/
int vpMatrix::pseudoInverseEigen3(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA,
                                  vpMatrix &imAt, vpMatrix &kerAt) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;
  vpMatrix U, V;
  vpColVector sv_;

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdEigen3(sv_, V);

  compute_pseudo_inverse(U, sv_, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, &imA, &imAt, &kerAt);

  // Remove singular values equal to the one that corresponds to the lines of 0
  // introduced when m < n
  memcpy(sv.data, sv_.data, sv.size() * sizeof(double));

  return rank_out;
}

#endif
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
/*!
  Compute and return the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n
  matrix \f$\bf A\f$ using OpenCV 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The Moore-Penros pseudo inverse \f$ A^+ \f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p = A.pseudoInverseEigen3();

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
}
  \endcode

  \sa pseudoInverse(double) const
*/
vpMatrix vpMatrix::pseudoInverseOpenCV(double svThreshold) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;

  vpMatrix U, V, Ap;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdOpenCV(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, NULL, NULL, NULL, NULL);

  return Ap;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ and return the rank of the matrix using OpenCV 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The rank of the matrix.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  unsigned int rank = A.pseudoInverseOpenCV(A_p);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank: " << rank << std::endl;
}
  \endcode

  \sa pseudoInverse(vpMatrix &, double) const
*/
unsigned int vpMatrix::pseudoInverseOpenCV(vpMatrix &Ap, double svThreshold) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;

  vpMatrix U, V;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdOpenCV(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, NULL, NULL, NULL, NULL);

  return static_cast<unsigned int>(rank_out);
}
/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values and return the rank of the matrix using
  OpenCV 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  vpColVector sv;
  unsigned int rank = A.pseudoInverseOpenCV(A_p, sv);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");

  std::cout << "Rank: " << rank << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
}
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, double) const
*/
unsigned int vpMatrix::pseudoInverseOpenCV(vpMatrix &Ap, vpColVector &sv, double svThreshold) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;

  vpMatrix U, V;
  vpColVector sv_;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdOpenCV(sv_, V);

  compute_pseudo_inverse(U, sv_, V, nrows, ncols, svThreshold,  Ap, rank_out, NULL, NULL, NULL, NULL);

  // Remove singular values equal to the one that corresponds to the lines of 0
  // introduced when m < n
  memcpy(sv.data, sv_.data, sv.size() * sizeof(double));

  return static_cast<unsigned int>(rank_out);
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$, \f$\mbox{Im}(A^T)\f$ and
  \f$\mbox{Ker}(A)\f$ and return the rank of the matrix using OpenCV 3rd
  party.

  \warning To inverse a square n-by-n matrix, you have to use rather
  inverseByLU(), inverseByCholesky(), or inverseByQR() that are kwown as faster.

  Using singular value decomposition, we have:

  \f[
  {\bf A}_{m\times n} = {\bf U}_{m\times m} \; {\bf S}_{m\times n} \; {\bf
  V^\top}_{n\times n} \f] \f[
  {\bf A}_{m\times n} = \left[\begin{array}{ccc}\mbox{Im} ({\bf A}) & | &
  \mbox{Ker} ({\bf A}^\top) \end{array} \right] {\bf S}_{m\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{m\times n}\f$ corresponds to the matrix
  \f$A\f$ singular values.

  This equation could be reformulated in a minimal way:
  \f[
  {\bf A}_{m\times n} = \mbox{Im} ({\bf A}) \; {\bf S}_{r\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{r\times n}\f$ corresponds to the matrix
  \f$A\f$ first r singular values.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param Ap: The Moore-Penros pseudo inverse \f$ {\bf A}^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param svThreshold: Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \param kerAt: The matrix that contains the null space (kernel) of \f$\bf
  A\f$ defined by the matrix \f${\bf X}^T\f$. If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is (0, n), otherwise the dimension is (n-r,
  n). This matrix is thus the transpose of \f$\mbox{Ker}({\bf A})\f$.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpColVector sv;
  vpMatrix A_p, imA, imAt, kerAt;
  unsigned int rank = A.pseudoInverseOpenCV(A_p, sv, 1e-6, imA, imAt, kerAt);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank: " << rank << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  imA.print(std::cout, 10, "Im(A): ");
  imAt.print(std::cout, 10, "Im(A^T): ");

  if (kerAt.size()) {
    kerAt.t().print(std::cout, 10, "Ker(A): ");
  }
  else {
    std::cout << "Ker(A) empty " << std::endl;
  }

  // Reconstruct matrix A from ImA, ImAt, KerAt
  vpMatrix S(rank, A.getCols());
  for(unsigned int i = 0; i< rank; i++)
    S[i][i] = sv[i];
  vpMatrix Vt(A.getCols(), A.getCols());
  Vt.insert(imAt.t(), 0, 0);
  Vt.insert(kerAt, rank, 0);
  (imA * S * Vt).print(std::cout, 10, "Im(A) * S * [Im(A^T) | Ker(A)]^T:");
}
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, double, vpMatrix &, vpMatrix &, vpMatrix &) const
*/
unsigned int vpMatrix::pseudoInverseOpenCV(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA,
                                           vpMatrix &imAt, vpMatrix &kerAt) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  vpMatrix U, V;
  vpColVector sv_;

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdOpenCV(sv_, V);

  compute_pseudo_inverse(U, sv_, V, nrows, ncols, svThreshold,  Ap, rank_out, NULL, &imA, &imAt, &kerAt);

  // Remove singular values equal to the one that corresponds to the lines of 0
  // introduced when m < n
  memcpy(sv.data, sv_.data, sv.size() * sizeof(double));

  return static_cast<unsigned int>(rank_out);
}

/*!
  Compute and return the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n
  matrix \f$\bf A\f$ using OpenCV 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param[in] rank_in : Known rank of the matrix.
  \return The Moore-Penros pseudo inverse \f$ A^+ \f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  // This matrix rank is 2
  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  int rank_in = 2;
  vpMatrix A_p = A.pseudoInverseOpenCV(rank_in);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  return 0;
}
  \endcode

  \sa pseudoInverse(int) const
*/
vpMatrix vpMatrix::pseudoInverseOpenCV(int rank_in) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;

  vpMatrix U, V, Ap;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdOpenCV(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, NULL, NULL, NULL);

  return Ap;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ and return the rank of the matrix using OpenCV 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param[out] Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param[in] rank_in : Known rank of the matrix.

  \return The computed rank of the matrix.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  // This matrix rank is 2
  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  int rank_in = 2;
  int rank_out = A.pseudoInverseOpenCV(A_p, rank_in);
  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_out << std::endl;
  return 0;
}
  \endcode

  \sa pseudoInverse(vpMatrix &, int) const
*/
int vpMatrix::pseudoInverseOpenCV(vpMatrix &Ap, int rank_in) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;

  vpMatrix U, V;
  vpColVector sv;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdOpenCV(sv, V);

  compute_pseudo_inverse(U, sv, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, NULL, NULL, NULL);

  return rank_out;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values and return the rank of the matrix using
  OpenCV 3rd party.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  // This matrix rank is 2
  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  vpColVector sv;
  int rank_in = 2;

  int rank_out = A.pseudoInverseOpenCV(A_p, sv, rank_in);
  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_out << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  return 0;
}
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, int) const
*/
int vpMatrix::pseudoInverseOpenCV(vpMatrix &Ap, vpColVector &sv, int rank_in) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;

  vpMatrix U, V;
  vpColVector sv_;

  Ap.resize(ncols, nrows, false, false);

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdOpenCV(sv_, V);

  compute_pseudo_inverse(U, sv_, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, NULL, NULL, NULL);

  // Remove singular values equal to the one that corresponds to the lines of 0
  // introduced when m < n
  memcpy(sv.data, sv_.data, sv.size() * sizeof(double));

  return rank_out;
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$, \f$\mbox{Im}(A^T)\f$ and
  \f$\mbox{Ker}(A)\f$ and return the rank of the matrix using OpenCV 3rd
  party.

  \warning To inverse a square n-by-n matrix, you have to use rather
  inverseByLU(), inverseByCholesky(), or inverseByQR() that are kwown as faster.

  Using singular value decomposition, we have:

  \f[
  {\bf A}_{m\times n} = {\bf U}_{m\times m} \; {\bf S}_{m\times n} \; {\bf
  V^\top}_{n\times n} \f] \f[
  {\bf A}_{m\times n} = \left[\begin{array}{ccc}\mbox{Im} ({\bf A}) & | &
  \mbox{Ker} ({\bf A}^\top) \end{array} \right] {\bf S}_{m\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{m\times n}\f$ corresponds to the matrix
  \f$A\f$ singular values.

  This equation could be reformulated in a minimal way:
  \f[
  {\bf A}_{m\times n} = \mbox{Im} ({\bf A}) \; {\bf S}_{r\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{r\times n}\f$ corresponds to the matrix
  \f$A\f$ first r singular values.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param Ap: The Moore-Penros pseudo inverse \f$ {\bf A}^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \param kerAt: The matrix that contains the null space (kernel) of \f$\bf
  A\f$ defined by the matrix \f${\bf X}^T\f$. If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is (0, n), otherwise the dimension is (n-r,
  n). This matrix is thus the transpose of \f$\mbox{Ker}({\bf A})\f$.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpColVector sv;
  vpMatrix A_p, imA, imAt, kerAt;
  int rank_in = 2;
  int rank_out = A.pseudoInverseOpenCV(A_p, sv, rank_in, imA, imAt, kerAt);

  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_out << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  imA.print(std::cout, 10, "Im(A): ");
  imAt.print(std::cout, 10, "Im(A^T): ");

  if (kerAt.size()) {
    kerAt.t().print(std::cout, 10, "Ker(A): ");
  }
  else {
    std::cout << "Ker(A) empty " << std::endl;
  }

  // Reconstruct matrix A from ImA, ImAt, KerAt
  vpMatrix S(rank_in, A.getCols());
  for(unsigned int i = 0; i< rank_in; i++)
    S[i][i] = sv[i];
  vpMatrix Vt(A.getCols(), A.getCols());
  Vt.insert(imAt.t(), 0, 0);
  Vt.insert(kerAt, rank_in, 0);
  (imA * S * Vt).print(std::cout, 10, "Im(A) * S * [Im(A^T) | Ker(A)]^T:");
}
  \endcode

  \sa pseudoInverse(vpMatrix &, vpColVector &, int, vpMatrix &, vpMatrix &, vpMatrix &) const
*/
int vpMatrix::pseudoInverseOpenCV(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA,
                                  vpMatrix &imAt, vpMatrix &kerAt) const
{
  unsigned int nrows = getRows();
  unsigned int ncols = getCols();
  int rank_out;
  double svThreshold = 1e-26;
  vpMatrix U, V;
  vpColVector sv_;

  if (nrows < ncols) {
    U.resize(ncols, ncols, true);
    sv.resize(nrows, false);
  } else {
    U.resize(nrows, ncols, false);
    sv.resize(ncols, false);
  }

  U.insert(*this, 0, 0);
  U.svdOpenCV(sv_, V);

  compute_pseudo_inverse(U, sv_, V, nrows, ncols, svThreshold, Ap, rank_out, &rank_in, &imA, &imAt, &kerAt);

  // Remove singular values equal to the one that corresponds to the lines of 0
  // introduced when m < n
  memcpy(sv.data, sv_.data, sv.size() * sizeof(double));

  return rank_out;
}

#endif
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values and return the rank of the matrix.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  vpColVector sv;
  unsigned int rank = A.pseudoInverse(A_p, sv);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");

  std::cout << "Rank: " << rank << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
}
  \endcode

  Once build, the previous example produces the following output:
  \code
A: [2,3]=
   2  3  5
  -4  2  3
A^+ (pseudo-inverse): [3,2]=
   0.117899 -0.190782
   0.065380  0.039657
   0.113612  0.052518
Rank: 2
Singular values: 6.874359351  4.443330227
  \endcode
*/
unsigned int vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(Ap, sv, svThreshold);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(Ap, sv, svThreshold);
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(Ap, sv, svThreshold);
#else
  (void)Ap;
  (void)sv;
  (void)svThreshold;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                                             "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values and return the rank of the matrix.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  vpColVector sv;
  int rank_in = 2;
  int rank_out = A.pseudoInverse(A_p, sv, rank_in);
  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");

  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_out << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
}
  \endcode

  Once build, the previous example produces the following output:
  \code
A: [2,3]=
   2  3  5
  -4  2  3
A^+ (pseudo-inverse): [3,2]=
   0.117899 -0.190782
   0.065380  0.039657
   0.113612  0.052518
Rank in : 2
Rank out: 2
Singular values: 6.874359351  4.443330227
  \endcode
*/
int vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, int rank_in) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(Ap, sv, rank_in);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(Ap, sv, rank_in);
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(Ap, sv, rank_in);
#else
  (void)Ap;
  (void)sv;
  (void)svThreshold;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                                             "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}
/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$ and \f$\mbox{Im}(A^T)\f$
  and return the rank of the matrix.

  See pseudoInverse(vpMatrix &, vpColVector &, double, vpMatrix &, vpMatrix &, vpMatrix &) const
  for a complete description of this function.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param svThreshold : Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  vpColVector sv;
  vpMatrix imA, imAt;
  unsigned int rank = A.pseudoInverse(A_p, sv, 1e-6, imA, imAt);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank: " << rank << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  imA.print(std::cout, 10, "Im(A): ");
  imAt.print(std::cout, 10, "Im(A^T): ");
}
  \endcode

  Once build, the previous example produces the following output:
  \code
A: [2,3]=
   2  3  5
  -4  2  3
A^+ (pseudo-inverse): [3,2]=
   0.117899 -0.190782
   0.065380  0.039657
   0.113612  0.052518
Rank: 2
Singular values: 6.874359351  4.443330227
Im(A): [2,2]=
   0.81458 -0.58003
   0.58003  0.81458
Im(A^T): [3,2]=
  -0.100515 -0.994397
   0.524244 -0.024967
   0.845615 -0.102722
  \endcode
*/
unsigned int vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA, vpMatrix &imAt) const
{
  vpMatrix kerAt;
  return pseudoInverse(Ap, sv, svThreshold, imA, imAt, kerAt);
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$ and \f$\mbox{Im}(A^T)\f$
  and return the rank of the matrix.

  See pseudoInverse(vpMatrix &, vpColVector &, double, vpMatrix &, vpMatrix &, vpMatrix &) const
  for a complete description of this function.

  \warning To inverse a square n-by-n matrix, you have to use rather one of
  the following functions inverseByLU(), inverseByQR(), inverseByCholesky() that
  are kwown as faster.

  \param Ap : The Moore-Penros pseudo inverse \f$ A^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpMatrix A_p;
  vpColVector sv;
  vpMatrix imA, imAt;
  int rank_in = 2;
  int rank_out = A.pseudoInverse(A_p, sv, rank_in, imA, imAt);
  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_in << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  imA.print(std::cout, 10, "Im(A): ");
  imAt.print(std::cout, 10, "Im(A^T): ");
}
  \endcode

  Once build, the previous example produces the following output:
  \code
A: [2,3]=
   2  3  5
  -4  2  3
A^+ (pseudo-inverse): [3,2]=
   0.117899 -0.190782
   0.065380  0.039657
   0.113612  0.052518
Rank: 2
Singular values: 6.874359351  4.443330227
Im(A): [2,2]=
   0.81458 -0.58003
   0.58003  0.81458
Im(A^T): [3,2]=
  -0.100515 -0.994397
   0.524244 -0.024967
   0.845615 -0.102722
  \endcode
*/
int vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA, vpMatrix &imAt) const
{
  vpMatrix kerAt;
  return pseudoInverse(Ap, sv, rank_in, imA, imAt, kerAt);
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$, \f$\mbox{Im}(A^T)\f$ and
  \f$\mbox{Ker}(A)\f$ and return the rank of the matrix.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather
  inverseByLU(), inverseByCholesky(), or inverseByQR() that are kwown as faster.

  Using singular value decomposition, we have:

  \f[
  {\bf A}_{m\times n} = {\bf U}_{m\times m} \; {\bf S}_{m\times n} \; {\bf
  V^\top}_{n\times n} \f] \f[
  {\bf A}_{m\times n} = \left[\begin{array}{ccc}\mbox{Im} ({\bf A}) & | &
  \mbox{Ker} ({\bf A}^\top) \end{array} \right] {\bf S}_{m\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{m\times n}\f$ corresponds to the matrix
  \f$A\f$ singular values.

  This equation could be reformulated in a minimal way:
  \f[
  {\bf A}_{m\times n} = \mbox{Im} ({\bf A}) \; {\bf S}_{r\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{r\times n}\f$ corresponds to the matrix
  \f$A\f$ first r singular values.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param Ap: The Moore-Penros pseudo inverse \f$ {\bf A}^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param svThreshold: Threshold used to test the singular values. If
  a singular value is lower than this threshold we consider that the
  matrix is not full rank.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \param kerAt: The matrix that contains the null space (kernel) of \f$\bf
  A\f$ defined by the matrix \f${\bf X}^T\f$. If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is (0, n), otherwise the dimension is (n-r,
  n). This matrix is thus the transpose of \f$\mbox{Ker}({\bf A})\f$.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpColVector sv;
  vpMatrix A_p, imA, imAt, kerAt;
  unsigned int rank = A.pseudoInverse(A_p, sv, 1e-6, imA, imAt, kerAt);

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank: " << rank << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  imA.print(std::cout, 10, "Im(A): ");
  imAt.print(std::cout, 10, "Im(A^T): ");

  if (kerAt.size()) {
    kerAt.t().print(std::cout, 10, "Ker(A): ");
  }
  else {
    std::cout << "Ker(A) empty " << std::endl;
  }

  // Reconstruct matrix A from ImA, ImAt, KerAt
  vpMatrix S(rank, A.getCols());
  for(unsigned int i = 0; i< rank; i++)
    S[i][i] = sv[i];
  vpMatrix Vt(A.getCols(), A.getCols());
  Vt.insert(imAt.t(), 0, 0);
  Vt.insert(kerAt, rank, 0);
  (imA * S * Vt).print(std::cout, 10, "Im(A) * S * [Im(A^T) | Ker(A)]^T:");
}
  \endcode

  Once build, the previous example produces the following output:
  \code
A: [2,3]=
   2  3  5
  -4  2  3
A^+ (pseudo-inverse): [3,2]=
   0.117899 -0.190782
   0.065380  0.039657
   0.113612  0.052518
Rank: 2
Singular values: 6.874359351  4.443330227
Im(A): [2,2]=
   0.81458 -0.58003
   0.58003  0.81458
Im(A^T): [3,2]=
  -0.100515 -0.994397
   0.524244 -0.024967
   0.845615 -0.102722
Ker(A): [3,1]=
  -0.032738
  -0.851202
   0.523816
Im(A) * S * [Im(A^T) | Ker(A)]^T:[2,3]=
   2  3  5
  -4  2  3
  \endcode
*/
unsigned int vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA, vpMatrix &imAt, vpMatrix &kerAt) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(Ap, sv, svThreshold, imA, imAt, kerAt);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(Ap, sv, svThreshold, imA, imAt, kerAt);
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(Ap, sv, svThreshold, imA, imAt, kerAt);
#else
  (void)Ap;
  (void)sv;
  (void)svThreshold;
  (void)imA;
  (void)imAt;
  (void)kerAt;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                                             "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

/*!
  Compute the Moore-Penros pseudo inverse \f$A^+\f$ of a m-by-n matrix \f$\bf
  A\f$ along with singular values, \f$\mbox{Im}(A)\f$, \f$\mbox{Im}(A^T)\f$ and
  \f$\mbox{Ker}(A)\f$ and return the rank of the matrix.

  \note By default, this function uses Lapack 3rd party. It is also possible
  to use a specific 3rd party suffixing this function name with one of the
  following 3rd party names (Lapack, Eigen3 or OpenCV).

  \warning To inverse a square n-by-n matrix, you have to use rather
  inverseByLU(), inverseByCholesky(), or inverseByQR() that are kwown as faster.

  Using singular value decomposition, we have:

  \f[
  {\bf A}_{m\times n} = {\bf U}_{m\times m} \; {\bf S}_{m\times n} \; {\bf
  V^\top}_{n\times n} \f] \f[
  {\bf A}_{m\times n} = \left[\begin{array}{ccc}\mbox{Im} ({\bf A}) & | &
  \mbox{Ker} ({\bf A}^\top) \end{array} \right] {\bf S}_{m\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{m\times n}\f$ corresponds to the matrix
  \f$A\f$ singular values.

  This equation could be reformulated in a minimal way:
  \f[
  {\bf A}_{m\times n} = \mbox{Im} ({\bf A}) \; {\bf S}_{r\times n}
  \left[
  \begin{array}{c} \left[\mbox{Im} ({\bf A}^\top)\right]^\top \\
  \\
  \hline \\
  \left[\mbox{Ker}({\bf A})\right]^\top \end{array}\right]
  \f]

  where the diagonal of \f${\bf S}_{r\times n}\f$ corresponds to the matrix
  \f$A\f$ first r singular values.

  The null space of a matrix \f$\bf A\f$ is defined as \f$\mbox{Ker}({\bf A})
  = { {\bf X} : {\bf A}*{\bf X} = {\bf 0}}\f$.

  \param Ap: The Moore-Penros pseudo inverse \f$ {\bf A}^+ \f$.

  \param sv: Vector corresponding to matrix \f$A\f$ singular values. The size
  of this vector is equal to min(m, n).

  \param[in] rank_in : Known rank of the matrix.

  \param imA: \f$\mbox{Im}({\bf A})\f$ that is a m-by-r matrix.

  \param imAt: \f$\mbox{Im}({\bf A}^T)\f$ that is n-by-r matrix.

  \param kerAt: The matrix that contains the null space (kernel) of \f$\bf
  A\f$ defined by the matrix \f${\bf X}^T\f$. If matrix \f$\bf A\f$ is full
  rank, the dimension of \c kerAt is (0, n), otherwise the dimension is (n-r,
  n). This matrix is thus the transpose of \f$\mbox{Ker}({\bf A})\f$.

  \return The rank of the matrix \f$\bf A\f$.

  Here an example to compute the pseudo-inverse of a 2-by-3 matrix that is rank 2.

  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(2, 3);

  A[0][0] = 2; A[0][1] = 3; A[0][2] = 5;
  A[1][0] = -4; A[1][1] = 2; A[1][2] = 3;

  A.print(std::cout, 10, "A: ");

  vpColVector sv;
  vpMatrix A_p, imA, imAt, kerAt;
  int rank_in = 2;
  int rank_out = A.pseudoInverse(A_p, sv, rank_in, imA, imAt, kerAt);
  if (rank_out != rank_in) {
    std::cout << "There is a possibility that the pseudo-inverse in wrong." << std::endl;
    std::cout << "Are you sure that the matrix rank is " << rank_in << std::endl;
  }

  A_p.print(std::cout, 10, "A^+ (pseudo-inverse): ");
  std::cout << "Rank in : " << rank_in << std::endl;
  std::cout << "Rank out: " << rank_out << std::endl;
  std::cout << "Singular values: " << sv.t() << std::endl;
  imA.print(std::cout, 10, "Im(A): ");
  imAt.print(std::cout, 10, "Im(A^T): ");

  if (kerAt.size()) {
    kerAt.t().print(std::cout, 10, "Ker(A): ");
  }
  else {
    std::cout << "Ker(A) empty " << std::endl;
  }

  // Reconstruct matrix A from ImA, ImAt, KerAt
  vpMatrix S(rank, A.getCols());
  for(unsigned int i = 0; i< rank_in; i++)
    S[i][i] = sv[i];
  vpMatrix Vt(A.getCols(), A.getCols());
  Vt.insert(imAt.t(), 0, 0);
  Vt.insert(kerAt, rank, 0);
  (imA * S * Vt).print(std::cout, 10, "Im(A) * S * [Im(A^T) | Ker(A)]^T:");
}
  \endcode

  Once build, the previous example produces the following output:
  \code
A: [2,3]=
   2  3  5
  -4  2  3
A^+ (pseudo-inverse): [3,2]=
   0.117899 -0.190782
   0.065380  0.039657
   0.113612  0.052518
Rank in : 2
Rank out: 2
Singular values: 6.874359351  4.443330227
Im(A): [2,2]=
   0.81458 -0.58003
   0.58003  0.81458
Im(A^T): [3,2]=
  -0.100515 -0.994397
   0.524244 -0.024967
   0.845615 -0.102722
Ker(A): [3,1]=
  -0.032738
  -0.851202
   0.523816
Im(A) * S * [Im(A^T) | Ker(A)]^T:[2,3]=
   2  3  5
  -4  2  3
  \endcode
*/
int vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA, vpMatrix &imAt, vpMatrix &kerAt) const
{
#if defined(VISP_HAVE_LAPACK)
  return pseudoInverseLapack(Ap, sv, rank_in, imA, imAt, kerAt);
#elif defined(VISP_HAVE_EIGEN3)
  return pseudoInverseEigen3(Ap, sv, rank_in, imA, imAt, kerAt);
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
  return pseudoInverseOpenCV(Ap, sv, rank_in, imA, imAt, kerAt);
#else
  (void)Ap;
  (void)sv;
  (void)svThreshold;
  (void)imA;
  (void)imAt;
  (void)kerAt;
  throw(vpException(vpException::fatalError, "Cannot compute pseudo-inverse. "
                                             "Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
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
It produces the following output:
  \code
[4,4]=
   0  1  2  3
   4  5  6  7
   8  9 10 11
  12 13 14 15
column vector:
5
9
13
  \endcode
 */
vpColVector vpMatrix::getCol(unsigned int j, unsigned int i_begin, unsigned int column_size) const
{
  if (i_begin + column_size > getRows() || j >= getCols())
    throw(vpException(vpException::dimensionError, "Unable to extract column %u from the %ux%u matrix", j, getRows(), getCols()));
  vpColVector c(column_size);
  for (unsigned int i = 0; i < column_size; i++)
    c[i] = (*this)[i_begin + i][j];
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

int main()
{
  vpMatrix A(4,4);

  for(unsigned int i=0; i < A.getRows(); i++)
    for(unsigned int j=0; j < A.getCols(); j++)
      A[i][j] = i*A.getCols()+j;

  A.print(std::cout, 4);

  vpColVector cv = A.getCol(1);
  std::cout << "Column vector: \n" << cv << std::endl;
}
  \endcode
It produces the following output:
  \code
[4,4]=
   0  1  2  3
   4  5  6  7
   8  9 10 11
  12 13 14 15
column vector:
1
5
9
13
  \endcode
 */
vpColVector vpMatrix::getCol(unsigned int j) const
{
  return getCol(j, 0, rowNum);
}

/*!
  Extract a row vector from a matrix.
  \warning All the indexes start from 0 in this function.
  \param i : Index of the row to extract. If i=0, the first row is extracted.
  \return The extracted row vector.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRowVector.h>

int main()
{
  vpMatrix A(4,4);

  for(unsigned int i=0; i < A.getRows(); i++)
    for(unsigned int j=0; j < A.getCols(); j++)
      A[i][j] = i*A.getCols()+j;

  A.print(std::cout, 4);

  vpRowVector rv = A.getRow(1);
  std::cout << "Row vector: \n" << rv << std::endl;
}  \endcode
It produces the following output:
  \code
[4,4]=
   0  1  2  3
   4  5  6  7
   8  9 10 11
  12 13 14 15
Row vector:
4  5  6  7
  \endcode
 */
vpRowVector vpMatrix::getRow(unsigned int i) const
{
  return getRow(i, 0, colNum);
}

/*!
  Extract a row vector from a matrix.
  \warning All the indexes start from 0 in this function.
  \param i : Index of the row to extract. If i=0, the first row is extracted.
  \param j_begin : Index of the column that gives the location of the first
element of the row vector to extract.
  \param row_size : Size of the row vector to extract.
  \return The extracted row vector.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRowVector.h>

int main()
{
  vpMatrix A(4,4);

  for(unsigned int i=0; i < A.getRows(); i++)
    for(unsigned int j=0; j < A.getCols(); j++)
      A[i][j] = i*A.getCols()+j;

  A.print(std::cout, 4);

  vpRowVector rv = A.getRow(1, 1, 3);
  std::cout << "Row vector: \n" << rv << std::endl;
}
  \endcode
It produces the following output:
  \code
[4,4]=
   0  1  2  3
   4  5  6  7
   8  9 10 11
  12 13 14 15
Row vector:
5  6  7
  \endcode
 */
vpRowVector vpMatrix::getRow(unsigned int i, unsigned int j_begin, unsigned int row_size) const
{
  if (j_begin + row_size > colNum || i >= rowNum)
    throw(vpException(vpException::dimensionError, "Unable to extract a row vector from the matrix"));

  vpRowVector r(row_size);
  if (r.data != NULL && data != NULL) {
    memcpy(r.data, (*this)[i] + j_begin, row_size*sizeof(double));
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

int main()
{
  vpMatrix A(3,4);

  for(unsigned int i=0; i < A.getRows(); i++)
    for(unsigned int j=0; j < A.getCols(); j++)
      A[i][j] = i*A.getCols()+j;

  A.print(std::cout, 4);

  vpColVector diag = A.getDiag();
  std::cout << "Diag vector: \n" << diag.t() << std::endl;
}
  \endcode
It produces the following output:
  \code
[3,4]=
   0  1  2  3
   4  5  6  7
   8  9 10 11
Diag vector:
0  5  10
  \endcode
 */
vpColVector vpMatrix::getDiag() const
{
  unsigned int min_size = std::min(rowNum, colNum);
  vpColVector diag;

  if (min_size > 0) {
    diag.resize(min_size, false);

    for (unsigned int i = 0; i < min_size; i++) {
      diag[i] = (*this)[i][i];
    }
  }

  return diag;
}

/*!
  Stack matrix \e B to the end of matrix \e A and return the resulting matrix
  [ A B ]^T

  \param A : Upper matrix.
  \param B : Lower matrix.
  \return Stacked matrix [ A B ]^T

  \warning A and B must have the same number of columns.
*/
vpMatrix vpMatrix::stack(const vpMatrix &A, const vpMatrix &B)
{
  vpMatrix C;

  vpMatrix::stack(A, B, C);

  return C;
}

/*!
  Stack matrix \e B to the end of matrix \e A and return the resulting matrix
  in \e C.

  \param  A : Upper matrix.
  \param  B : Lower matrix.
  \param  C : Stacked matrix C = [ A B ]^T

  \warning A and B must have the same number of columns. A and C, B and C must
  be two different objects.
*/
void vpMatrix::stack(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
  unsigned int nra = A.getRows();
  unsigned int nrb = B.getRows();

  if (nra != 0) {
    if (A.getCols() != B.getCols()) {
      throw(vpException(vpException::dimensionError, "Cannot stack (%dx%d) matrix with (%dx%d) matrix", A.getRows(),
                        A.getCols(), B.getRows(), B.getCols()));
    }
  }

  if (A.data != NULL && A.data == C.data) {
    std::cerr << "A and C must be two different objects!" << std::endl;
    return;
  }

  if (B.data != NULL && B.data == C.data) {
    std::cerr << "B and C must be two different objects!" << std::endl;
    return;
  }

  C.resize(nra + nrb, B.getCols(), false, false);

  if (C.data != NULL && A.data != NULL && A.size() > 0) {
    // Copy A in C
    memcpy(C.data, A.data, sizeof(double) * A.size());
  }

  if (C.data != NULL && B.data != NULL && B.size() > 0) {
    // Copy B in C
    memcpy(C.data + A.size(), B.data, sizeof(double) * B.size());
  }
}

/*!
  Stack row vector \e r to matrix \e A and return the resulting matrix [ A r ]^T

  \param A : Upper matrix.
  \param r : Lower row vector.
  \return Stacked matrix [ A r ]^T

  \warning \e A and \e r must have the same number of columns.
*/
vpMatrix vpMatrix::stack(const vpMatrix &A, const vpRowVector &r)
{
  vpMatrix C;
  vpMatrix::stack(A, r, C);

  return C;
}

/*!
  Stack row vector \e r to the end of matrix \e A and return the resulting
  matrix in \e C.

  \param  A : Upper matrix.
  \param  r : Lower row vector.
  \param  C : Stacked matrix C = [ A r ]^T

  \warning A and r must have the same number of columns. A and C must be two
  different objects.
*/
void vpMatrix::stack(const vpMatrix &A, const vpRowVector &r, vpMatrix &C)
{
  if (A.data != NULL && A.data == C.data) {
    std::cerr << "A and C must be two different objects!" << std::endl;
    return;
  }

  C = A;
  C.stack(r);
}

/*!
  Stack column vector \e c to matrix \e A and return the resulting matrix [ A c ]

  \param A : Left matrix.
  \param c : Right column vector.
  \return Stacked matrix [ A c ]

  \warning \e A and \e c must have the same number of rows.
*/
vpMatrix vpMatrix::stack(const vpMatrix &A, const vpColVector &c)
{
  vpMatrix C;
  vpMatrix::stack(A, c, C);

  return C;
}

/*!
  Stack column vector \e c to the end of matrix \e A and return the resulting
  matrix in \e C.

  \param  A : Left matrix.
  \param  c : Right column vector.
  \param  C : Stacked matrix C = [ A c ]

  \warning A and c must have the same number of rows. A and C must be two
  different objects.
*/
void vpMatrix::stack(const vpMatrix &A, const vpColVector &c, vpMatrix &C)
{
  if (A.data != NULL && A.data == C.data) {
    std::cerr << "A and C must be two different objects!" << std::endl;
    return;
  }

  C = A;
  C.stack(c);
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
  vpMatrix C;

  insert(A, B, C, r, c);

  return C;
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
  if (((r + B.getRows()) <= A.getRows()) && ((c + B.getCols()) <= A.getCols())) {
    C.resize(A.getRows(), A.getCols(), false, false);

    for (unsigned int i = 0; i < A.getRows(); i++) {
      for (unsigned int j = 0; j < A.getCols(); j++) {
        if (i >= r && i < (r + B.getRows()) && j >= c && j < (c + B.getCols())) {
          C[i][j] = B[i - r][j - c];
        } else {
          C[i][j] = A[i][j];
        }
      }
    }
  } else {
    throw vpException(vpException::dimensionError, "Cannot insert (%dx%d) matrix in (%dx%d) matrix at position (%d,%d)",
                      B.getRows(), B.getCols(), A.getCols(), A.getRows(), r, c);
  }
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

  if (B.getRows() == 0 || nca + ncb == 0) {
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
  are set with respect to the parameter maxlen.

  \param s Stream used for the printing.

  \param length The suggested width of each matrix element.
  The actual width grows in order to accomodate the whole integral part,
  and shrinks if the whole extent is not needed for all the numbers.
  \param intro The introduction which is printed before the matrix.
  Can be set to zero (or omitted), in which case
  the introduction is not printed.

  \return Returns the common total width for all matrix elements

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

  // ossFixed <<std::fixed;
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

      values[i * n + j] = oss.str();
      size_type thislen = values[i * n + j].size();
      size_type p = values[i * n + j].find('.');

      if (p == std::string::npos) {
        maxBefore = vpMath::maximum(maxBefore, thislen);
        // maxAfter remains the same
      } else {
        maxBefore = vpMath::maximum(maxBefore, p);
        maxAfter = vpMath::maximum(maxAfter, thislen - p - 1);
      }
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

  if (! intro.empty())
    s << intro;
  s << "[" << m << "," << n << "]=\n";

  for (unsigned int i = 0; i < m; i++) {
    s << "  ";
    for (unsigned int j = 0; j < n; j++) {
      size_type p = values[i * n + j].find('.');
      s.setf(std::ios::right, std::ios::adjustfield);
      s.width((std::streamsize)maxBefore);
      s << values[i * n + j].substr(0, p).c_str();

      if (maxAfter > 0) {
        s.setf(std::ios::left, std::ios::adjustfield);
        if (p != std::string::npos) {
          s.width((std::streamsize)maxAfter);
          s << values[i * n + j].substr(p, maxAfter).c_str();
        } else {
          assert(maxAfter > 1);
          s.width((std::streamsize)maxAfter);
          s << ".0";
        }
      }

      s << ' ';
    }
    s << std::endl;
  }

  s.flags(original_flags); // restore s to standard state

  return (int)(maxBefore + maxAfter);
}

/*!
  Print using Matlab syntax, to copy/paste in Matlab later.

  The following code
  \code
#include <visp3/core/vpMatrix.h>

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
  os << "[ ";
  for (unsigned int i = 0; i < this->getRows(); ++i) {
    for (unsigned int j = 0; j < this->getCols(); ++j) {
      os << (*this)[i][j] << ", ";
    }
    if (this->getRows() != i + 1) {
      os << ";" << std::endl;
    } else {
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
  os << "([ " << std::endl;
  for (unsigned int i = 0; i < this->getRows(); ++i) {
    os << "[";
    for (unsigned int j = 0; j < this->getCols(); ++j) {
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
  for (unsigned int i = 0; i < this->getRows(); ++i) {
    for (unsigned int j = 0; j < this->getCols(); ++j) {
      os << (*this)[i][j];
      if (!(j == (this->getCols() - 1)))
        os << ", ";
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

  for (unsigned int i = 0; i < this->getRows(); ++i) {
    for (unsigned int j = 0; j < this->getCols(); ++j) {
      if (!octet) {
        os << matrixName << "[" << i << "][" << j << "] = " << (*this)[i][j] << "; " << std::endl;
      } else {
        for (unsigned int k = 0; k < sizeof(double); ++k) {
          os << "((unsigned char*)&(" << matrixName << "[" << i << "][" << j << "]) )[" << k << "] = 0x" << std::hex
             << (unsigned int)((unsigned char *)&((*this)[i][j]))[k] << "; " << std::endl;
        }
      }
    }
    os << std::endl;
  }
  return os;
}

/*!
  Stack A at the end of the current matrix, or copy if the matrix has no
  dimensions : this = [ this A ]^T.
*/
void vpMatrix::stack(const vpMatrix &A)
{
  if (rowNum == 0) {
    *this = A;
  } else if (A.getRows() > 0) {
    if (colNum != A.getCols()) {
      throw(vpException(vpException::dimensionError, "Cannot stack (%dx%d) matrix with (%dx%d) matrix", rowNum, colNum,
                        A.getRows(), A.getCols()));
    }

    unsigned int rowNumOld = rowNum;
    resize(rowNum + A.getRows(), colNum, false, false);
    insert(A, rowNumOld, 0);
  }
}

/*!
  Stack row vector \e r at the end of the current matrix, or copy if the
matrix has no dimensions: this = [ this r ]^T.

  Here an example for a robot velocity log :
\code
vpMatrix A;
vpColVector v(6);
for(unsigned int i = 0;i<100;i++)
{
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, v);
  Velocities.stack(v.t());
}
\endcode
*/
void vpMatrix::stack(const vpRowVector &r)
{
  if (rowNum == 0) {
    *this = r;
  } else {
    if (colNum != r.getCols()) {
      throw(vpException(vpException::dimensionError, "Cannot stack (%dx%d) matrix with (1x%d) row vector", rowNum,
                        colNum, r.getCols()));
    }

    if (r.size() == 0) {
      return;
    }

    unsigned int oldSize = size();
    resize(rowNum + 1, colNum, false, false);

    if (data != NULL && r.data != NULL && data != r.data) {
      // Copy r in data
      memcpy(data + oldSize, r.data, sizeof(double) * r.size());
    }
  }
}

/*!
  Stack column vector \e c at the right of the current matrix, or copy if the
matrix has no dimensions: this = [ this c ].

  Here an example for a robot velocity log matrix:
\code
vpMatrix log;
vpColVector v(6);
for(unsigned int i = 0; i<100;i++)
{
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, v);
  log.stack(v);
}
\endcode
Here the log matrix has size 6 rows by 100 columns.
*/
void vpMatrix::stack(const vpColVector &c)
{
  if (colNum == 0) {
    *this = c;
  } else {
    if (rowNum != c.getRows()) {
      throw(vpException(vpException::dimensionError, "Cannot stack (%dx%d) matrix with (%dx1) column vector", rowNum,
                        colNum, c.getRows()));
    }

    if (c.size() == 0) {
      return;
    }

    vpMatrix tmp = *this;
    unsigned int oldColNum = colNum;
    resize(rowNum, colNum + 1, false, false);

    if (data != NULL && tmp.data != NULL && data != tmp.data) {
      // Copy c in data
      for (unsigned int i = 0; i < rowNum; i++) {
        memcpy(data + i*colNum, tmp.data + i*oldColNum, sizeof(double) * oldColNum);
        rowPtrs[i][oldColNum] = c[i];
      }
    }
  }
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
  if ((r + A.getRows()) <= rowNum && (c + A.getCols()) <= colNum) {
    if (A.colNum == colNum && data != NULL && A.data != NULL && A.data != data) {
      memcpy(data + r * colNum, A.data, sizeof(double) * A.size());
    } else if (data != NULL && A.data != NULL && A.data != data) {
      for (unsigned int i = r; i < (r + A.getRows()); i++) {
        memcpy(data + i * colNum + c, A.data + (i - r) * A.colNum, sizeof(double) * A.colNum);
      }
    }
  } else {
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
    throw(vpException(vpException::dimensionError,
                      "Cannot compute eigen values on a non square matrix (%dx%d)",
                      rowNum, colNum));
  }

  // Check if the matrix is symetric: At - A = 0
  vpMatrix At_A = (*this).t() - (*this);
  for (unsigned int i = 0; i < rowNum; i++) {
    for (unsigned int j = 0; j < rowNum; j++) {
      // if (At_A[i][j] != 0) {
      if (std::fabs(At_A[i][j]) > std::numeric_limits<double>::epsilon()) {
        throw(vpException(vpException::fatalError, "Cannot compute eigen values on a non symetric matrix"));
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
    int info;
    double wkopt;
    vpMatrix::blas_dsyev(jobz, uplo, rowNum, A.data, colNum, evalue.data, &wkopt, lwork, info);
    lwork = static_cast<int>(wkopt);
    WORK.resize(lwork);
    vpMatrix::blas_dsyev(jobz, uplo, rowNum, A.data, colNum, evalue.data, WORK.data, lwork, info);
  }
#endif
#else
  {
    throw(vpException(vpException::functionNotImplementedError,
                      "Eigen values computation is not implemented. "
                      "You should install Lapack 3rd party"));
  }
#endif
  return evalue;
}

/*!
  Compute the eigenvalues of a n-by-n real symmetric matrix using
  Lapack 3rd party.
  \return The eigenvalues of a n-by-n real symmetric matrix.

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
    throw(vpException(vpException::dimensionError,
                      "Cannot compute eigen values on a non square matrix (%dx%d)",
                      rowNum, colNum));
  }

  // Check if the matrix is symetric: At - A = 0
  vpMatrix At_A = (*this).t() - (*this);
  for (unsigned int i = 0; i < rowNum; i++) {
    for (unsigned int j = 0; j < rowNum; j++) {
      // if (At_A[i][j] != 0) {
      if (std::fabs(At_A[i][j]) > std::numeric_limits<double>::epsilon()) {
        throw(vpException(vpException::fatalError, "Cannot compute eigen values on a non symetric matrix"));
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
#else // defined(VISP_HAVE_GSL)
  {
    const char jobz = 'V';
    const char uplo = 'U';
    vpMatrix A = (*this);
    vpColVector WORK;
    int lwork = -1;
    int info;
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
    throw(vpException(vpException::functionNotImplementedError,
                      "Eigen values computation is not implemented. "
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

  vpMatrix U, V;               // Copy of the matrix, SVD function is destructive
  vpColVector sv;
  sv.resize(nbcol, false);    // singular values
  V.resize(nbcol, nbcol, false); // V matrix of singular value decomposition

  // Copy and resize matrix to have at least as many rows as columns
  // kernel is computed in svd method only if the matrix has more rows than
  // columns

  if (nbline < nbcol)
    U.resize(nbcol, nbcol, true);
  else
    U.resize(nbline, nbcol, false);

  U.insert(*this, 0, 0);

  U.svd(sv, V);

  // Compute the highest singular value and rank of the matrix
  double maxsv = 0;
  for (unsigned int i = 0; i < nbcol; i++) {
    if (sv[i] > maxsv) {
      maxsv = sv[i];
    }
  }

  unsigned int rank = 0;
  for (unsigned int i = 0; i < nbcol; i++) {
    if (sv[i] > maxsv * svThreshold) {
      rank++;
    }
  }

  kerAt.resize(nbcol - rank, nbcol);
  if (rank != nbcol) {
    for (unsigned int j = 0, k = 0; j < nbcol; j++) {
      // if( v.col(j) in kernel and non zero )
      if ((sv[j] <= maxsv * svThreshold) &&
          (std::fabs(V.getCol(j).sumSquare()) > std::numeric_limits<double>::epsilon())) {
        for (unsigned int i = 0; i < V.getRows(); i++) {
          kerAt[k][i] = V[i][j];
        }
        k++;
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

  vpMatrix U, V;               // Copy of the matrix, SVD function is destructive
  vpColVector sv;
  sv.resize(nbcol, false);    // singular values
  V.resize(nbcol, nbcol, false); // V matrix of singular value decomposition

  // Copy and resize matrix to have at least as many rows as columns
  // kernel is computed in svd method only if the matrix has more rows than
  // columns

  if (nbrow < nbcol)
    U.resize(nbcol, nbcol, true);
  else
    U.resize(nbrow, nbcol, false);

  U.insert(*this, 0, 0);

  U.svd(sv, V);

  // Compute the highest singular value and rank of the matrix
  double maxsv = sv[0];

  unsigned int rank = 0;
  for (unsigned int i = 0; i < nbcol; i++) {
    if (sv[i] > maxsv * svThreshold) {
      rank++;
    }
  }

  kerA.resize(nbcol, nbcol - rank);
  if (rank != nbcol) {
    for (unsigned int j = 0, k = 0; j < nbcol; j++) {
      // if( v.col(j) in kernel and non zero )
      if (sv[j] <= maxsv * svThreshold) {
        for (unsigned int i = 0; i < nbcol; i++) {
          kerA[i][k] = V[i][j];
        }
        k++;
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

  vpMatrix U, V;               // Copy of the matrix, SVD function is destructive
  vpColVector sv;
  sv.resize(nbcol, false);    // singular values
  V.resize(nbcol, nbcol, false); // V matrix of singular value decomposition

  // Copy and resize matrix to have at least as many rows as columns
  // kernel is computed in svd method only if the matrix has more rows than
  // columns

  if (nbrow < nbcol)
    U.resize(nbcol, nbcol, true);
  else
    U.resize(nbrow, nbcol, false);

  U.insert(*this, 0, 0);

  U.svd(sv, V);

  kerA.resize(nbcol, dim_);
  if (dim_ != 0) {
    unsigned int rank = nbcol - dim_;
    for (unsigned int k = 0; k < dim_; k++) {
      unsigned int j = k + rank;
      for (unsigned int i = 0; i < nbcol; i++) {
        kerA[i][k] = V[i][j];
      }
    }
  }

  double maxsv = sv[0];
  unsigned int rank = 0;
  for (unsigned int i = 0; i < nbcol; i++) {
    if (sv[i] > maxsv * 1e-6) {
      rank++;
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

  return (det);
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
  } else {
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
    vpMatrix _expE(rowNum, colNum, false);
    vpMatrix _expD(rowNum, colNum, false);
    vpMatrix _expX(rowNum, colNum, false);
    vpMatrix _expcX(rowNum, colNum, false);
    vpMatrix _eye(rowNum, colNum, false);

    _eye.eye();
    vpMatrix exp(*this);

    //      double f;
    int e;
    double c = 0.5;
    int q = 6;
    int p = 1;

    double nA = 0;
    for (unsigned int i = 0; i < rowNum; i++) {
      double sum = 0;
      for (unsigned int j = 0; j < colNum; j++) {
        sum += fabs((*this)[i][j]);
      }
      if (sum > nA || i == 0) {
        nA = sum;
      }
    }

    /* f = */ frexp(nA, &e);
    // double s = (0 > e+1)?0:e+1;
    double s = e + 1;

    double sca = 1.0 / pow(2.0, s);
    exp = sca * exp;
    _expX = *this;
    _expE = c * exp + _eye;
    _expD = -c * exp + _eye;
    for (int k = 2; k <= q; k++) {
      c = c * ((double)(q - k + 1)) / ((double)(k * (2 * q - k + 1)));
      _expcX = exp * _expX;
      _expX = _expcX;
      _expcX = c * _expX;
      _expE = _expE + _expcX;
      if (p)
        _expD = _expD + _expcX;
      else
        _expD = _expD - _expcX;
      p = !p;
    }
    _expX = _expD.inverseByLU();
    exp = _expX * _expE;
    for (int k = 1; k <= s; k++) {
      _expE = exp * exp;
      exp = _expE;
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

  for (unsigned int i = 0; i < col; i++) {
    for (unsigned int j = 0; j < row; j++)
      M_comp[i][j] = M[i][j];
    for (unsigned int j = row + 1; j < M.getRows(); j++)
      M_comp[i][j - 1] = M[i][j];
  }
  for (unsigned int i = col + 1; i < M.getCols(); i++) {
    for (unsigned int j = 0; j < row; j++)
      M_comp[i - 1][j] = M[i][j];
    for (unsigned int j = row + 1; j < M.getRows(); j++)
      M_comp[i - 1][j - 1] = M[i][j];
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

  vpMatrix U, V;               // Copy of the matrix, SVD function is destructive
  vpColVector sv;
  sv.resize(nbcol);    // singular values
  V.resize(nbcol, nbcol, false); // V matrix of singular value decomposition

  // Copy and resize matrix to have at least as many rows as columns
  // kernel is computed in svd method only if the matrix has more rows than
  // columns

  if (nbline < nbcol)
    U.resize(nbcol, nbcol, true);
  else
    U.resize(nbline, nbcol, false);

  U.insert(*this, 0, 0);

  U.svd(sv, V);

  // Compute the highest singular value
  double maxsv = 0;
  for (unsigned int i = 0; i < nbcol; i++) {
    if (sv[i] > maxsv) {
      maxsv = sv[i];
    }
  }

  // Compute the rank of the matrix
  unsigned int rank = 0;
  for (unsigned int i = 0; i < nbcol; i++) {
    if (sv[i] > maxsv * svThreshold) {
      rank++;
    }
  }

  // Compute the lowest singular value
  double minsv = maxsv;
  for (unsigned int i = 0; i < rank; i++) {
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
  for (unsigned int i = 0; i < H.getCols(); i++) {
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
  for (unsigned int i = 0; i < dsize; i++) {
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
  if(this->dsize != 0){
    vpMatrix v;
    vpColVector w;

    vpMatrix M = *this;

    M.svd(w, v);

    double max = w[0];
    unsigned int maxRank = std::min(this->getCols(), this->getRows());
    // The maximum reachable rank is either the number of columns or the number of rows
    // of the matrix.
    unsigned int boundary = std::min(maxRank, w.size());
    // boundary is here to ensure that the number of singular values used for the com-
    // putation of the euclidean norm of the matrix is not greater than the maximum
    // reachable rank. Indeed, some svd library pad the singular values vector with 0s
    // if the input matrix is non-square.
    for (unsigned int i = 0; i < boundary; i++) {
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
  for (unsigned int i = 0; i < rowNum; i++) {
    double x = 0;
    for (unsigned int j = 0; j < colNum; j++) {
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

  for (unsigned int i = 0; i < rowNum; i++) {
    for (unsigned int j = 0; j < colNum; j++) {
      x = rowPtrs[i][j];
      sum_square += x * x;
    }
  }

  return sum_square;
}

/*!
  Perform a 2D convolution similar to Matlab conv2 function: \f$ M \star kernel \f$.

  \param M : First matrix.
  \param kernel : Second matrix.
  \param mode : Convolution mode: "full" (default), "same", "valid".

  \image html vpMatrix-conv2-mode.jpg "Convolution mode: full, same, valid (image credit: Theano doc)."

  \note This is a very basic implementation that does not use FFT.
 */
vpMatrix vpMatrix::conv2(const vpMatrix &M, const vpMatrix &kernel, const std::string &mode)
{
  vpMatrix res;
  conv2(M, kernel, res, mode);
  return res;
}

/*!
  Perform a 2D convolution similar to Matlab conv2 function: \f$ M \star kernel \f$.

  \param M : First matrix.
  \param kernel : Second matrix.
  \param res : Result.
  \param mode : Convolution mode: "full" (default), "same", "valid".

  \image html vpMatrix-conv2-mode.jpg "Convolution mode: full, same, valid (image credit: Theano doc)."

  \note This is a very basic implementation that does not use FFT.
 */
void vpMatrix::conv2(const vpMatrix &M, const vpMatrix &kernel, vpMatrix &res, const std::string &mode)
{
  if (M.getRows()*M.getCols() == 0 || kernel.getRows()*kernel.getCols() == 0)
    return;

  if (mode == "valid") {
    if (kernel.getRows() > M.getRows() || kernel.getCols() > M.getCols())
      return;
  }

  vpMatrix M_padded, res_same;

  if (mode == "full" || mode == "same") {
    const unsigned int pad_x = kernel.getCols()-1;
    const unsigned int pad_y = kernel.getRows()-1;
    M_padded.resize(M.getRows() + 2*pad_y, M.getCols() + 2*pad_x, true, false);
    M_padded.insert(M, pad_y, pad_x);

    if (mode == "same") {
      res.resize(M.getRows(), M.getCols(), false, false);
      res_same.resize(M.getRows() + pad_y, M.getCols() + pad_x, true, false);
    } else {
      res.resize(M.getRows() + pad_y, M.getCols() + pad_x, true, false);
    }
  } else if (mode == "valid") {
    M_padded = M;
    res.resize(M.getRows()-kernel.getRows()+1, M.getCols()-kernel.getCols()+1);
  } else {
    return;
  }

  if (mode == "same") {
    for (unsigned int i = 0; i < res_same.getRows(); i++) {
      for (unsigned int j = 0; j < res_same.getCols(); j++) {
        for (unsigned int k = 0; k < kernel.getRows(); k++) {
          for (unsigned int l = 0; l < kernel.getCols(); l++) {
            res_same[i][j] += M_padded[i+k][j+l] * kernel[kernel.getRows()-k-1][kernel.getCols()-l-1];
          }
        }
      }
    }

    const unsigned int start_i = kernel.getRows()/2;
    const unsigned int start_j = kernel.getCols()/2;
    for (unsigned int i = 0; i < M.getRows(); i++) {
      memcpy(res.data + i*M.getCols(), res_same.data + (i+start_i)*res_same.getCols() + start_j, sizeof(double)*M.getCols());
    }
  } else {
    for (unsigned int i = 0; i < res.getRows(); i++) {
      for (unsigned int j = 0; j < res.getCols(); j++) {
        for (unsigned int k = 0; k < kernel.getRows(); k++) {
          for (unsigned int l = 0; l < kernel.getCols(); l++) {
            res[i][j] += M_padded[i+k][j+l] * kernel[kernel.getRows()-k-1][kernel.getCols()-l-1];
          }
        }
      }
    }
  }
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
  \deprecated This function is deprecated. You should rather use frobeniusNorm().

  Compute and return the Euclidean norm (also called Frobenius norm) \f$||A|| = \sqrt{ \sum{A_{ij}^2}}\f$.

  \return The Euclidean norm (also called Frobenius norm) if the matrix is initialized, 0 otherwise.

  \sa frobeniusNorm(), infinityNorm(), inducedL2Norm()
*/
vp_deprecated double vpMatrix::euclideanNorm() const
{
  return frobeniusNorm();
}

vpMatrix vpMatrix::stackMatrices(const vpColVector &A, const vpColVector &B)
{
  return (vpMatrix)(vpColVector::stack(A, B));
}

void vpMatrix::stackMatrices(const vpColVector &A, const vpColVector &B, vpColVector &C)
{
  vpColVector::stack(A, B, C);
}

vpMatrix vpMatrix::stackMatrices(const vpMatrix &A, const vpRowVector &B) { return vpMatrix::stack(A, B); }

void vpMatrix::stackMatrices(const vpMatrix &A, const vpRowVector &B, vpMatrix &C) { vpMatrix::stack(A, B, C); }

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

  for (unsigned int j = 0; j < getCols(); j++)
    c[j] = (*this)[i - 1][j];
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

  for (unsigned int i = 0; i < getRows(); i++)
    c[i] = (*this)[i][j - 1];
  return c;
}

/*!
  \deprecated You should rather use diag(const double &)

  Set the matrix diagonal elements to \e val.
  More generally set M[i][i] = val.
*/
void vpMatrix::setIdentity(const double &val)
{
  for (unsigned int i = 0; i < rowNum; i++)
    for (unsigned int j = 0; j < colNum; j++)
      if (i == j)
        (*this)[i][j] = val;
      else
        (*this)[i][j] = 0;
}

#endif //#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
