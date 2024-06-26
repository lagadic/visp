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
 * Stack matrix.
 */

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>

#if defined(VISP_HAVE_SIMDLIB)
#include <Simd/SimdLib.h>
#endif

BEGIN_VISP_NAMESPACE

/*!
  Compute and return the transpose of the matrix.
*/
vpMatrix vpMatrix::t() const { return transpose(); }

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

  if ((rowNum <= 16) || (colNum <= 16)) {
    for (unsigned int i = 0; i < rowNum; ++i) {
      for (unsigned int j = 0; j < colNum; ++j) {
        At[j][i] = (*this)[i][j];
      }
    }
  }
  else {
#if defined(VISP_HAVE_SIMDLIB)
    SimdMatTranspose(data, rowNum, colNum, At.data);
#else
    // https://stackoverflow.com/a/21548079
    const int tileSize = 32;
    for (unsigned int i = 0; i < rowNum; i += tileSize) {
      for (unsigned int j = 0; j < colNum; ++j) {
        for (unsigned int b = 0; ((b < static_cast<unsigned int>(tileSize)) && ((i + b) < rowNum)); ++b) {
          At[j][i + b] = (*this)[i + b][j];
        }
      }
    }
#endif
  }
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

  if (A.rowNum != w.rowNum) {
    w.resize(A.rowNum, false);
  }

  // If available use Lapack only for large matrices
  bool useLapack = ((A.rowNum > vpMatrix::m_lapack_min_size) || (A.colNum > vpMatrix::m_lapack_min_size));
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
    for (unsigned int j = 0; j < A.colNum; ++j) {
      double vj = v[j]; // optimization em 5/12/2006
      for (unsigned int i = 0; i < A.rowNum; ++i) {
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
  if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum)) {
    C.resize(A.rowNum, B.colNum, false, false);
  }

  if (A.colNum != B.rowNum) {
    throw(vpException(vpException::dimensionError, "Cannot multiply (%dx%d) matrix by (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  // If available use Lapack only for large matrices
  bool useLapack = ((A.rowNum > vpMatrix::m_lapack_min_size) || (A.colNum > vpMatrix::m_lapack_min_size) ||
                    (B.colNum > vpMatrix::m_lapack_min_size));
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
    for (unsigned int i = 0; i < A.rowNum; ++i) {
      const double *rowptri = A.rowPtrs[i];
      double *ci = C[i];
      for (unsigned int j = 0; j < BcolNum; ++j) {
        double s = 0;
        for (unsigned int k = 0; k < BrowNum; ++k) {
          s += rowptri[k] * BrowPtrs[k][j];
        }
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
  if ((A.colNum != 3) || (A.rowNum != 3) || (B.colNum != 3) || (B.rowNum != 3)) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply (%dx%d) matrix by (%dx%d) matrix as a "
                      "rotation matrix",
                      A.getRows(), A.getCols(), B.getRows(), B.getCols()));
  }
  // 5/12/06 some "very" simple optimization to avoid indexation
  const unsigned int BcolNum = B.colNum;
  const unsigned int BrowNum = B.rowNum;
  double **BrowPtrs = B.rowPtrs;
  for (unsigned int i = 0; i < A.rowNum; ++i) {
    const double *rowptri = A.rowPtrs[i];
    double *ci = C[i];
    for (unsigned int j = 0; j < BcolNum; ++j) {
      double s = 0;
      for (unsigned int k = 0; k < BrowNum; ++k) {
        s += rowptri[k] * BrowPtrs[k][j];
      }
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
  if ((A.colNum != 4) || (A.rowNum != 4) || (B.colNum != 4) || (B.rowNum != 4)) {
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
  bool useLapack = ((A.rowNum > vpMatrix::m_lapack_min_size) || (A.colNum > vpMatrix::m_lapack_min_size) ||
                    (B.colNum > vpMatrix::m_lapack_min_size));
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
    for (unsigned int i = 0; i < A.rowNum; ++i) {
      const double *rowptri = A.rowPtrs[i];
      double *ci = C[i];
      for (unsigned int j = 0; j < BcolNum; ++j) {
        double s = 0;
        for (unsigned int k = 0; k < BrowNum; ++k) {
          s += rowptri[k] * BrowPtrs[k][j];
        }
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
  Operation C = A*wA + B*wB

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (Speed gain if used many times with the same result matrix size).

  \sa operator+()
*/

void vpMatrix::add2WeightedMatrices(const vpMatrix &A, const double &wA, const vpMatrix &B, const double &wB,
                                    vpMatrix &C)
{
  if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum)) {
    C.resize(A.rowNum, B.colNum, false, false);
  }

  if ((A.colNum != B.getCols()) || (A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot add (%dx%d) matrix with (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  double **ArowPtrs = A.rowPtrs;
  double **BrowPtrs = B.rowPtrs;
  double **CrowPtrs = C.rowPtrs;

  for (unsigned int i = 0; i < A.rowNum; ++i) {
    for (unsigned int j = 0; j < A.colNum; ++j) {
      CrowPtrs[i][j] = (wB * BrowPtrs[i][j]) + (wA * ArowPtrs[i][j]);
    }
  }
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
  if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum)) {
    C.resize(A.rowNum, B.colNum, false, false);
  }

  if ((A.colNum != B.getCols()) || (A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot add (%dx%d) matrix with (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  double **ArowPtrs = A.rowPtrs;
  double **BrowPtrs = B.rowPtrs;
  double **CrowPtrs = C.rowPtrs;

  for (unsigned int i = 0; i < A.rowNum; ++i) {
    for (unsigned int j = 0; j < A.colNum; ++j) {
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
  if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum)) {
    C.resize(A.rowNum);
  }

  if ((A.colNum != B.getCols()) || (A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot add (%dx%d) matrix with (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  double **ArowPtrs = A.rowPtrs;
  double **BrowPtrs = B.rowPtrs;
  double **CrowPtrs = C.rowPtrs;

  for (unsigned int i = 0; i < A.rowNum; ++i) {
    for (unsigned int j = 0; j < A.colNum; ++j) {
      CrowPtrs[i][j] = BrowPtrs[i][j] + ArowPtrs[i][j];
    }
  }
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
  if ((A.rowNum != C.rowNum) || (A.colNum != C.colNum)) {
    C.resize(A.rowNum);
  }

  if ((A.colNum != B.getCols()) || (A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot subtract (%dx%d) matrix to (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  double **ArowPtrs = A.rowPtrs;
  double **BrowPtrs = B.rowPtrs;
  double **CrowPtrs = C.rowPtrs;

  for (unsigned int i = 0; i < A.rowNum; ++i) {
    for (unsigned int j = 0; j < A.colNum; ++j) {
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
  if ((A.rowNum != C.rowNum) || (A.colNum != C.colNum)) {
    C.resize(A.rowNum, A.colNum, false, false);
  }

  if ((A.colNum != B.getCols()) || (A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot subtract (%dx%d) matrix to (%dx%d) matrix", A.getRows(),
                      A.getCols(), B.getRows(), B.getCols()));
  }

  double **ArowPtrs = A.rowPtrs;
  double **BrowPtrs = B.rowPtrs;
  double **CrowPtrs = C.rowPtrs;

  for (unsigned int i = 0; i < A.rowNum; ++i) {
    for (unsigned int j = 0; j < A.colNum; ++j) {
      CrowPtrs[i][j] = ArowPtrs[i][j] - BrowPtrs[i][j];
    }
  }
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
  if ((A.rowNum != C.rowNum) || (A.colNum != C.colNum)) {
    C.resize(A.rowNum, A.colNum, false, false);
  }

  double **ArowPtrs = A.rowPtrs;
  double **CrowPtrs = C.rowPtrs;

  for (unsigned int i = 0; i < A.rowNum; ++i) {
    for (unsigned int j = 0; j < A.colNum; ++j) {
      CrowPtrs[i][j] = -ArowPtrs[i][j];
    }
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
  if ((B.rowNum != rowNum) || (B.colNum != rowNum)) {
    B.resize(rowNum, rowNum, false, false);
  }

  // If available use Lapack only for large matrices
  bool useLapack = ((rowNum > vpMatrix::m_lapack_min_size) || (colNum > vpMatrix::m_lapack_min_size));
#if !(defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL))
  useLapack = false;
#endif

  if (useLapack) {
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL)
    const double alpha = 1.0;
    const double beta = 0.0;
    const char transa = 't';
    const char transb = 'n';

    vpMatrix::blas_dgemm(transa, transb, rowNum, rowNum, colNum, alpha, data, colNum, data, colNum, beta, B.data,
                         rowNum);
#endif
  }
  else {
    // compute A*A^T
    for (unsigned int i = 0; i < rowNum; ++i) {
      for (unsigned int j = i; j < rowNum; ++j) {
        double *pi = rowPtrs[i]; // row i
        double *pj = rowPtrs[j]; // row j

        // sum (row i .* row j)
        double ssum = 0;
        for (unsigned int k = 0; k < colNum; ++k) {
          ssum += *(pi++) * *(pj++);
        }

        B[i][j] = ssum; // upper triangle
        if (i != j) {
          B[j][i] = ssum; // lower triangle
        }
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
  if ((B.rowNum != colNum) || (B.colNum != colNum)) {
    B.resize(colNum, colNum, false, false);
  }

  // If available use Lapack only for large matrices
  bool useLapack = ((rowNum > vpMatrix::m_lapack_min_size) || (colNum > vpMatrix::m_lapack_min_size));
#if !(defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL))
  useLapack = false;
#endif

  if (useLapack) {
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL)
    const double alpha = 1.0;
    const double beta = 0.0;
    const char transa = 'n';
    const char transb = 't';

    vpMatrix::blas_dgemm(transa, transb, colNum, colNum, rowNum, alpha, data, colNum, data, colNum, beta, B.data,
                         colNum);
#endif
  }
  else {
    for (unsigned int i = 0; i < colNum; ++i) {
      double *Bi = B[i];
      for (unsigned int j = 0; j < i; ++j) {
        double *ptr = data;
        double s = 0;
        for (unsigned int k = 0; k < rowNum; ++k) {
          s += (*(ptr + i)) * (*(ptr + j));
          ptr += colNum;
        }
        *Bi++ = s;
        B[j][i] = s;
      }
      double *ptr = data;
      double s = 0;
      for (unsigned int k = 0; k < rowNum; ++k) {
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
  Create a diagonal matrix with the element of a vector.

  \param  A : Vector which element will be put in the diagonal.

  \sa createDiagonalMatrix()

  \code
  #include <iostream>

  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
  for (unsigned int i = 0; i < rows; ++i) {
    (*this)[i][i] = A[i];
  }
}

/*!
  Set the matrix as a diagonal matrix where each element on the diagonal is
  set to \e val. Elements that are not on the diagonal are set to 0.

  \param val : Value to set.

  \sa eye()

  \code
  #include <iostream>

  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
  for (unsigned int i = 0; i < min_; ++i) {
    (*this)[i][i] = val;
  }
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

  for (unsigned int i = 0; i < rows; ++i) {
    DA[i][i] = A[i];
  }
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
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      if (i == j) {
        (*this)[i][j] = 1.0;
      }
      else {
        (*this)[i][j] = 0;
      }
    }
  }
}

/*!
  Compute the Hadamard product (element wise matrix multiplication).
  \param m : Second matrix;
  \return m1.hadamard(m2) The Hadamard product :
  \f$ m1 \circ m2 = (m1 \circ m2)_{i,j} = (m1)_{i,j} (m2)_{i,j} \f$
*/
vpMatrix vpMatrix::hadamard(const vpMatrix &m) const
{
  if ((m.getRows() != rowNum) || (m.getCols() != colNum)) {
    throw(vpException(vpException::dimensionError, "In Hadamard product: bad dimension of input matrix"));
  }

  vpMatrix out;
  out.resize(rowNum, colNum, false, false);

#if defined(VISP_HAVE_SIMDLIB)
  SimdVectorHadamard(data, m.data, dsize, out.data);
#else
  for (unsigned int i = 0; i < dsize; ++i) {
    out.data[i] = data[i] * m.data[i];
  }
#endif

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

  out.resize(r1 * r2, c1 * c2, false, false);

  for (unsigned int r = 0; r < r1; ++r) {
    for (unsigned int c = 0; c < c1; ++c) {
      double alpha = m1[r][c];
      double *m2ptr = m2[0];
      unsigned int roffset = r * r2;
      unsigned int coffset = c * c2;
      for (unsigned int rr = 0; rr < r2; ++rr) {
        for (unsigned int cc = 0; cc < c2; ++cc) {
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

  for (unsigned int r = 0; r < r1; ++r) {
    for (unsigned int c = 0; c < c1; ++c) {
      double alpha = m1[r][c];
      double *m2ptr = m2[0];
      unsigned int roffset = r * r2;
      unsigned int coffset = c * c2;
      for (unsigned int rr = 0; rr < r2; ++rr) {
        for (unsigned int cc = 0; cc < c2; ++cc) {
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

vpMatrix vpMatrix::conv2(const vpMatrix &M, const vpMatrix &kernel, const std::string &mode)
{
  vpMatrix res;
  conv2(M, kernel, res, mode);
  return res;
}

void vpMatrix::conv2(const vpMatrix &M, const vpMatrix &kernel, vpMatrix &res, const std::string &mode)
{
  if (((M.getRows() * M.getCols()) == 0) || ((kernel.getRows() * kernel.getCols()) == 0)) {
    return;
  }

  if (mode == "valid") {
    if ((kernel.getRows() > M.getRows()) || (kernel.getCols() > M.getCols())) {
      return;
    }
  }

  vpMatrix M_padded, res_same;

  if ((mode == "full") || (mode == "same")) {
    const unsigned int pad_x = kernel.getCols() - 1;
    const unsigned int pad_y = kernel.getRows() - 1;
    const unsigned int pad = 2;
    M_padded.resize(M.getRows() + (pad * pad_y), M.getCols() + (pad * pad_x), true, false);
    M_padded.insert(M, pad_y, pad_x);

    if (mode == "same") {
      res.resize(M.getRows(), M.getCols(), false, false);
      res_same.resize(M.getRows() + pad_y, M.getCols() + pad_x, true, false);
    }
    else {
      res.resize(M.getRows() + pad_y, M.getCols() + pad_x, true, false);
    }
  }
  else if (mode == "valid") {
    M_padded = M;
    res.resize((M.getRows() - kernel.getRows()) + 1, (M.getCols() - kernel.getCols()) + 1);
  }
  else {
    return;
  }

  if (mode == "same") {
    unsigned int res_same_rows = res_same.getRows();
    unsigned int res_same_cols = res_same.getCols();
    unsigned int kernel_rows = kernel.getRows();
    unsigned int kernel_cols = kernel.getCols();
    for (unsigned int i = 0; i < res_same_rows; ++i) {
      for (unsigned int j = 0; j < res_same_cols; ++j) {
        for (unsigned int k = 0; k < kernel_rows; ++k) {
          for (unsigned int l = 0; l < kernel_cols; ++l) {
            res_same[i][j] += M_padded[i + k][j + l] * kernel[kernel.getRows() - k - 1][kernel.getCols() - l - 1];
          }
        }
      }
    }

    const unsigned int start_i = kernel.getRows() / 2;
    const unsigned int start_j = kernel.getCols() / 2;
    unsigned int m_rows = M.getRows();
    for (unsigned int i = 0; i < m_rows; ++i) {
      memcpy(res.data + (i * M.getCols()), res_same.data + ((i + start_i) * res_same.getCols()) + start_j,
             sizeof(double) * M.getCols());
    }
  }
  else {
    unsigned int res_rows = res.getRows();
    unsigned int res_cols = res.getCols();
    unsigned int kernel_rows = kernel.getRows();
    unsigned int kernel_cols = kernel.getCols();
    for (unsigned int i = 0; i < res_rows; ++i) {
      for (unsigned int j = 0; j < res_cols; ++j) {
        for (unsigned int k = 0; k < kernel_rows; ++k) {
          for (unsigned int l = 0; l < kernel_cols; ++l) {
            res[i][j] += M_padded[i + k][j + l] * kernel[kernel.getRows() - k - 1][kernel.getCols() - l - 1];
          }
        }
      }
    }
  }
}
END_VISP_NAMESPACE
