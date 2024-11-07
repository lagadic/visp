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
  Copy operator that allows to convert on of the following container that
  inherit from vpArray2D such as vpMatrix, vpRotationMatrix,
  vpHomogeneousMatrix, vpPoseVector, vpColVector, vpRowVector... into a
  vpMatrix.

  \param A : 2D array to be copied.

  The following example shows how to create a matrix from an homogeneous
  matrix:
  \code
  vpRotationMatrix R;
  vpMatrix M;
  M = R;
  \endcode
*/
vpMatrix &vpMatrix::operator=(const vpArray2D<double> &A)
{
  resize(A.getRows(), A.getCols(), false, false);

  if ((data != nullptr) && (A.data != nullptr) && (data != A.data)) {
    memcpy(data, A.data, dsize * sizeof(double));
  }

  return *this;
}

/*!
  Copy operator that allows to convert a homogenous matrix to a matrix.

  \param M : Homogeneous matrix.

  The following example shows how to create a matrix from a homogenous matrix:
  \code
  vpHomogeneousMatrix H;
  vpMatrix M;
  M = H;
  \endcode
*/
vpMatrix &vpMatrix::operator=(const vpHomogeneousMatrix &M)
{
  resize(M.getRows(), M.getCols(), false, false);

  if ((data != nullptr) && (M.data != nullptr) && (data != M.data)) {
    memcpy(data, M.data, dsize * sizeof(double));
  }

  return *this;
}

/*!
  Copy operator that allows to convert a rotation matrix to a matrix.

  \param R : Rotation matrix.

  The following example shows how to create a matrix from a rotation matrix:
  \code
  vpRotationMatrix R;
  vpMatrix M;
  M = R;
  \endcode
*/
vpMatrix &vpMatrix::operator=(const vpRotationMatrix &R)
{
  resize(R.getRows(), R.getCols(), false, false);

  if ((data != nullptr) && (R.data != nullptr) && (data != R.data)) {
    memcpy(data, R.data, dsize * sizeof(double));
  }

  return *this;
}

/*!
  Copy operator that allows to convert a velocity twist matrix to a matrix.

  \param V : Velocity twist matrix.

  The following example shows how to create a matrix from a velocity twist matrix:
  \code
  vpVelocityTwistMatrix V;
  vpMatrix M;
  M = V;
  \endcode
*/
vpMatrix &vpMatrix::operator=(const vpVelocityTwistMatrix &V)
{
  resize(V.getRows(), V.getCols(), false, false);

  if ((data != nullptr) && (V.data != nullptr) && (data != V.data)) {
    memcpy(data, V.data, dsize * sizeof(double));
  }

  return *this;
}

/*!
  Copy operator that allows to convert a force twist matrix to a matrix.

  \param F : Force twist matrix.

  The following example shows how to create a matrix from a force twist matrix:
  \code
  vpForceTwistMatrix F;
  vpMatrix M;
  M = F;
  \endcode
*/
vpMatrix &vpMatrix::operator=(const vpForceTwistMatrix &F)
{
  resize(F.getRows(), F.getCols(), false, false);

  if ((data != nullptr) && (F.data != nullptr) && (data != F.data)) {
    memcpy(data, F.data, dsize * sizeof(double));
  }

  return *this;
}

/*!
  Copy operator that allows to convert a column vector to a matrix.

  \param v : Column vector.

  The following example shows how to create a matrix from a column vector:
  \code
  vpColVector v(3);
  vpMatrix M;
  M = v;
  \endcode
*/
vpMatrix &vpMatrix::operator=(const vpColVector &v)
{
  resize(v.getRows(), v.getCols(), false, false);

  if ((data != nullptr) && (v.data != nullptr) && (data != v.data)) {
    memcpy(data, v.data, dsize * sizeof(double));
  }

  return *this;
}

/*!
  Copy operator that allows to convert a row vector to a matrix.

  \param v : Column vector.

  The following example shows how to create a matrix from a row vector:
  \code
  vpRowVector v(3);
  vpMatrix M;
  M = v;
  \endcode
*/
vpMatrix &vpMatrix::operator=(const vpRowVector &v)
{
  resize(v.getRows(), v.getCols(), false, false);

  if ((data != nullptr) && (v.data != nullptr) && (data != v.data)) {
    memcpy(data, v.data, dsize * sizeof(double));
  }

  return *this;
}

/*!
  Copy operator that allows to convert a translation vector to a matrix.

  \param t : Translation vector.

  The following example shows how to create a matrix from a translation vector:
  \code
  vpTranslationVector t;
  vpMatrix M;
  M = t;
  \endcode
*/
vpMatrix &vpMatrix::operator=(const vpTranslationVector &t)
{
  resize(t.getRows(), t.getCols(), false, false);

  if ((data != nullptr) && (t.data != nullptr) && (data != t.data)) {
    memcpy(data, t.data, dsize * sizeof(double));
  }

  return *this;
}

vpMatrix &vpMatrix::operator=(const vpMatrix &A)
{
  resize(A.getRows(), A.getCols(), false, false);

  if ((data != nullptr) && (A.data != nullptr) && (data != A.data)) {
    memcpy(data, A.data, dsize * sizeof(double));
  }

  return *this;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
vpMatrix &vpMatrix::operator=(vpMatrix &&other)
{
  if (this != &other) {
    if (data) {
      free(data);
    }
    if (rowPtrs) {
      free(rowPtrs);
    }

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
  Set matrix elements from a list of values.
  \param list : List of double. Matrix size (number of columns multiplied by number of columns) should match the number
  of elements.

  \return The modified Matrix. The following example shows how to set each element of a 2-by-3 matrix.

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
vpMatrix &vpMatrix::operator=(const std::initializer_list<double> &list)
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

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
vpMatrix &vpMatrix::operator=(const std::initializer_list<std::initializer_list<double> > &lists)
{
  unsigned int nrows = static_cast<unsigned int>(lists.size()), ncols = 0;
  for (auto &l : lists) {
    if (static_cast<unsigned int>(l.size()) > ncols) {
      ncols = static_cast<unsigned int>(l.size());
    }
  }

  resize(nrows, ncols, false, false);
  auto it = lists.begin();
  for (unsigned int i = 0; i < rowNum; ++i, ++it) {
    std::copy(it->begin(), it->end(), rowPtrs[i]);
  }

  return *this;
}
#endif

//! Set all the element of the matrix A to \e x.
vpMatrix &vpMatrix::operator=(double x)
{
  std::fill(data, data + (rowNum * colNum), x);
  return *this;
}

/*!
  Assignment from an array of double. This method has to be used carefully
  since the array allocated behind \e x pointer should have the same dimension
  than the matrix.
*/
vpMatrix &vpMatrix::operator<<(double *x)
{
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      rowPtrs[i][j] = *x++;
    }
  }
  return *this;
}

vpMatrix &vpMatrix::operator<<(double val)
{
  resize(1, 1, false, false);
  rowPtrs[0][0] = val;
  return *this;
}

vpMatrix &vpMatrix::operator,(double val)
{
  resize(1, colNum + 1, false, false);
  rowPtrs[0][colNum - 1] = val;
  return *this;
}

/*!
  Operator that allows to multiply a matrix by a translation vector.
  The matrix should be of dimension (3x3)
*/
vpTranslationVector vpMatrix::operator*(const vpTranslationVector &tv) const
{
  vpTranslationVector t_out;

  if ((rowNum != 3) || (colNum != 3)) {
    throw(vpException(vpException::dimensionError, "Cannot multiply a (%dx%d) matrix by a (%dx%d) translation vector",
                      rowNum, colNum, tv.getRows(), tv.getCols()));
  }

  const unsigned int val_3 = 3;
  for (unsigned int j = 0; j < val_3; ++j) {
    t_out[j] = 0;
  }

  for (unsigned int j = 0; j < val_3; ++j) {
    double tj = tv[j]; // optimization em 5/12/2006
    for (unsigned int i = 0; i < val_3; ++i) {
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
  for (unsigned int i = 0; i < rowNum; ++i) {
    double *rowptri = rowPtrs[i];
    double *ci = C[i];
    for (unsigned int j = 0; j < RcolNum; ++j) {
      double s = 0;
      for (unsigned int k = 0; k < RrowNum; ++k) {
        s += rowptri[k] * R[k][j];
      }
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
  for (unsigned int i = 0; i < rowNum; ++i) {
    const double *rowptri = rowPtrs[i];
    double *ci = C[i];
    for (unsigned int j = 0; j < McolNum; ++j) {
      double s = 0;
      for (unsigned int k = 0; k < MrowNum; ++k) {
        s += rowptri[k] * M[k][j];
      }
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
  bool useLapack = ((rowNum > vpMatrix::m_lapack_min_size) || (colNum > vpMatrix::m_lapack_min_size) ||
                    (V.colNum > vpMatrix::m_lapack_min_size));
#if !(defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL))
  useLapack = false;
#endif

  if (useLapack) {
#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN) && !defined(VISP_HAVE_GSL)
    const double alpha = 1.0;
    const double beta = 0.0;
    const char trans = 'n';
    vpMatrix::blas_dgemm(trans, trans, V.colNum, rowNum, colNum, alpha, V.data, V.colNum, data, colNum, beta, M.data,
                         M.colNum);
#endif
  }
  else {
#if defined(VISP_HAVE_SIMDLIB)
    SimdMatMulTwist(data, rowNum, V.data, M.data);
#else
    unsigned int VcolNum = V.getCols();
    unsigned int VrowNum = V.getRows();
    for (unsigned int i = 0; i < rowNum; ++i) {
      double *rowptri = rowPtrs[i];
      double *ci = M[i];
      for (unsigned int j = 0; j < VcolNum; ++j) {
        double s = 0;
        for (unsigned int k = 0; k < VrowNum; ++k) {
          s += rowptri[k] * V[k][j];
        }
        ci[j] = s;
      }
    }
#endif
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
  bool useLapack = ((rowNum > vpMatrix::m_lapack_min_size) || (colNum > vpMatrix::m_lapack_min_size) ||
                    (V.getCols() > vpMatrix::m_lapack_min_size));
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
#if defined(VISP_HAVE_SIMDLIB)
    SimdMatMulTwist(data, rowNum, V.data, M.data);
#else
    unsigned int VcolNum = V.getCols();
    unsigned int VrowNum = V.getRows();
    for (unsigned int i = 0; i < rowNum; ++i) {
      double *rowptri = rowPtrs[i];
      double *ci = M[i];
      for (unsigned int j = 0; j < VcolNum; ++j) {
        double s = 0;
        for (unsigned int k = 0; k < VrowNum; ++k) {
          s += rowptri[k] * V[k][j];
        }
        ci[j] = s;
      }
    }
#endif
  }

  return M;
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

  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      rowPtrs[i][j] += BrowPtrs[i][j];
    }
  }

  return *this;
}

//! Operation A = A - B
vpMatrix &vpMatrix::operator-=(const vpMatrix &B)
{
  if ((colNum != B.getCols()) || (rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError, "Cannot subtract (%dx%d) matrix to (%dx%d) matrix", rowNum, colNum,
                      B.getRows(), B.getCols()));
  }

  double **BrowPtrs = B.rowPtrs;
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      rowPtrs[i][j] -= BrowPtrs[i][j];
    }
  }

  return *this;
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
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      s += rowPtrs[i][j];
    }
  }

  return s;
}

/*!
   Operator that allows to multiply all the elements of a matrix
   by a scalar.
 */
vpMatrix vpMatrix::operator*(double x) const
{
  if (std::fabs(x - 1.) < std::numeric_limits<double>::epsilon()) {
    return *this;
  }

  vpMatrix M;
  M.resize(rowNum, colNum, false, false);

  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      M[i][j] = rowPtrs[i][j] * x;
    }
  }

  return M;
}


//! Cij = Aij / x (A is unchanged)
vpMatrix vpMatrix::operator/(double x) const
{
  if (std::fabs(x - 1.) < std::numeric_limits<double>::epsilon()) {
    return *this;
  }

  if (std::fabs(x) < std::numeric_limits<double>::epsilon()) {
    throw vpException(vpException::divideByZeroError, "Divide matrix by zero scalar");
  }

  vpMatrix C;
  C.resize(rowNum, colNum, false, false);

  double xinv = 1 / x;

  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      C[i][j] = rowPtrs[i][j] * xinv;
    }
  }

  return C;
}

//! Add x to all the element of the matrix : Aij = Aij + x
vpMatrix &vpMatrix::operator+=(double x)
{
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      rowPtrs[i][j] += x;
    }
  }

  return *this;
}

//! subtract x to all the element of the matrix : Aij = Aij - x
vpMatrix &vpMatrix::operator-=(double x)
{
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      rowPtrs[i][j] -= x;
    }
  }

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

  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      rowPtrs[i][j] *= x;
    }
  }

  return *this;
}

//! Divide  all the element of the matrix by x : Aij = Aij / x
vpMatrix &vpMatrix::operator/=(double x)
{
  if (std::fabs(x - 1.) < std::numeric_limits<double>::epsilon()) {
    return *this;
  }

  if (std::fabs(x) < std::numeric_limits<double>::epsilon()) {
    throw vpException(vpException::divideByZeroError, "Divide matrix by zero scalar");
  }

  double xinv = 1 / x;

  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      rowPtrs[i][j] *= xinv;
    }
  }

  return *this;
}

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

  VISP_NAMESPACE_ADDRESSING vpMatrix C;
  C.resize(Brow, Bcol, false, false);

  for (unsigned int i = 0; i < Brow; ++i) {
    for (unsigned int j = 0; j < Bcol; ++j) {
      C[i][j] = B[i][j] * x;
    }
  }

  return C;
}

END_VISP_NAMESPACE
