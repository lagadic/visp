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
 * Matrix Cholesky decomposition.
 */

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrixException.h>

#if defined(VISP_HAVE_OPENCV) // Require opencv >= 2.1.1
#include <opencv2/core/core.hpp>
#endif

#ifdef VISP_HAVE_LAPACK
#ifdef VISP_HAVE_GSL
#include <gsl/gsl_linalg.h>
#endif
#ifdef VISP_HAVE_MKL
#include <mkl.h>
typedef MKL_INT integer;
#elif !defined(VISP_HAVE_GSL)
#ifdef VISP_HAVE_LAPACK_BUILT_IN
typedef long int integer;
#else
typedef int integer;
#endif
extern "C" void dpotrf_(char *uplo, integer *n, double *a, integer *lda, integer *info);
extern "C" int dpotri_(char *uplo, integer *n, double *a, integer *lda, integer *info);
#endif
#endif

#if defined(VISP_HAVE_EIGEN3)
#include <Eigen/Dense>
#endif

BEGIN_VISP_NAMESPACE
/*!
  Compute the inverse of a n-by-n matrix using the Cholesky decomposition.
  The matrix must be real symmetric positive defined.

  This function calls the first following function that is available:
  - inverseByCholeskyLapack() if Lapack 3rd party is installed
  - inverseByLUOpenCV() if OpenCV 3rd party is installed.

  If none of these 3rd parties is installed we use a Lapack built-in version.

  \return The inverse matrix.

  Here an example:
  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix A(4,4);

    // Symmetric matrix
    A[0][0] = 1/1.; A[0][1] = 1/5.; A[0][2] = 1/6.; A[0][3] = 1/7.;
    A[1][0] = 1/5.; A[1][1] = 1/3.; A[1][2] = 1/3.; A[1][3] = 1/5.;
    A[2][0] = 1/6.; A[2][1] = 1/3.; A[2][2] = 1/2.; A[2][3] = 1/6.;
    A[3][0] = 1/7.; A[3][1] = 1/5.; A[3][2] = 1/6.; A[3][3] = 1/7.;

    // Compute the inverse
    vpMatrix A_1; // A^-1
    A_1 = A.inverseByCholesky();
    std::cout << "Inverse by Cholesky: \n" << A_1 << std::endl;

    std::cout << "A*A^-1: \n" << A * A_1 << std::endl;
  }
  \endcode

  \sa pseudoInverse()
*/
vpMatrix vpMatrix::inverseByCholesky() const
{
#if defined(VISP_HAVE_LAPACK)
  return inverseByCholeskyLapack();
#elif defined(VISP_HAVE_OPENCV)
  return inverseByCholeskyOpenCV();
#else
  throw(vpException(vpException::fatalError, "Cannot inverse matrix by Cholesky. Install Lapack or OpenCV 3rd party"));
#endif
}

#if defined(VISP_HAVE_LAPACK)
/*!
  Compute the inverse of a n-by-n matrix using the Cholesky decomposition with
  Lapack 3rd party. The matrix must be real symmetric positive defined.

  \return The inverse matrix.

  Here an example:
  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    unsigned int n = 4;
    vpMatrix A(n, n);
    vpMatrix I;
    I.eye(4);

    A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.; A[0][3] = 1/4.;
    A[1][0] = 1/5.; A[1][1] = 1/3.; A[1][2] = 1/3.; A[1][3] = 1/5.;
    A[2][0] = 1/6.; A[2][1] = 1/4.; A[2][2] = 1/2.; A[2][3] = 1/6.;
    A[3][0] = 1/7.; A[3][1] = 1/5.; A[3][2] = 1/6.; A[3][3] = 1/7.;

    // Make matrix symmetric positive
    A = 0.5*(A+A.t());
    A = A + n*I;

    // Compute the inverse
    vpMatrix A_1 = A.inverseByCholeskyLapack();
    std::cout << "Inverse by Cholesky (Lapack): \n" << A_1 << std::endl;

    std::cout << "A*A^-1: \n" << A * A_1 << std::endl;
  }
  \endcode

  \sa inverseByCholesky(), inverseByCholeskyOpenCV()
*/
vpMatrix vpMatrix::inverseByCholeskyLapack() const
{
#if defined(VISP_HAVE_GSL)
  {
    vpMatrix invA = *this;

    gsl_matrix cholesky;
    cholesky.size1 = rowNum;
    cholesky.size2 = colNum;
    cholesky.tda = cholesky.size2;
    cholesky.data = invA.data;
    cholesky.owner = 0;
    cholesky.block = 0;

#if (GSL_MAJOR_VERSION >= 2 && GSL_MINOR_VERSION >= 3)
    gsl_linalg_cholesky_decomp1(&cholesky);
#else
    gsl_linalg_cholesky_decomp(&cholesky);
#endif
    gsl_linalg_cholesky_invert(&cholesky);
    return invA;
  }
#else
  {
    if (rowNum != colNum) {
      throw(vpMatrixException(vpMatrixException::matrixError, "Cannot inverse a non-square matrix (%ux%u) by Cholesky",
                              rowNum, colNum));
    }

    integer rowNum_ = (integer)this->getRows();
    integer lda = (integer)rowNum_; // lda is the number of rows because we don't use a submatrix
    integer info;

    vpMatrix A = *this;
    dpotrf_((char *)"L", &rowNum_, A.data, &lda, &info);

    if (info != 0) {
      std::stringstream errMsg;
      errMsg << "Cannot inverse by Cholesky with Lapack: error "<< info << " in dpotrf_()";
      throw(vpException(vpException::fatalError, errMsg.str()));
    }

    dpotri_((char *)"L", &rowNum_, A.data, &lda, &info);
    if (info != 0) {
      std::cout << "cholesky:dpotri_:error" << std::endl;
      throw vpMatrixException::badValue;
    }

    for (unsigned int i = 0; i < A.getRows(); ++i)
      for (unsigned int j = 0; j < A.getCols(); ++j)
        if (i > j)
          A[i][j] = A[j][i];

    return A;
  }
#endif
}

/*!
 * \brief Compute the Cholesky decomposition of a Hermitian positive-definite matrix
 * using Lapack library.
 *
 * \return vpMatrix The lower triangular matrix resulting from the Cholesky decomposition
 */
vpMatrix vpMatrix::choleskyByLapack()const
{
  vpMatrix L = *this;
#if defined(VISP_HAVE_GSL)
  gsl_matrix cholesky;
  cholesky.size1 = rowNum;
  cholesky.size2 = colNum;
  cholesky.tda = cholesky.size2;
  cholesky.data = L.data;
  cholesky.owner = 0;
  cholesky.block = 0;

#if (GSL_MAJOR_VERSION >= 2 && GSL_MINOR_VERSION >= 3)
  gsl_linalg_cholesky_decomp1(&cholesky);
#else
  gsl_linalg_cholesky_decomp(&cholesky);
#endif
#else
  if (rowNum != colNum) {
    throw(vpMatrixException(vpMatrixException::matrixError, "Cannot inverse a non-square matrix (%ux%u) by Cholesky",
                            rowNum, colNum));
  }

  integer rowNum_ = (integer)this->getRows();
  integer lda = (integer)rowNum_; // lda is the number of rows because we don't use a submatrix
  integer info;
  dpotrf_((char *)"L", &rowNum_, L.data, &lda, &info);
  L = L.transpose(); // For an unknown reason, dpotrf seems to return the transpose of L
  if (info < 0) {
    std::stringstream errMsg;
    errMsg << "The " << -info << "th argument has an illegal value" << std::endl;
    throw(vpMatrixException(vpMatrixException::forbiddenOperatorError, errMsg.str()));
  }
  else if (info > 0) {
    std::stringstream errMsg;
    errMsg << "The leading minor of order" << info << "is not positive definite." << std::endl;
    throw(vpMatrixException(vpMatrixException::forbiddenOperatorError, errMsg.str()));
  }
#endif
  // Make sure that the upper part of L is null
  unsigned int nbRows = this->getRows();
  unsigned int nbCols = this->getCols();
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = r + 1; c < nbCols; ++c) {
      L[r][c] = 0.;
    }
  }
  return L;
}
#endif // VISP_HAVE_LAPACK

#if defined(VISP_HAVE_OPENCV)
/*!
  Compute the inverse of a n-by-n matrix using the Cholesky decomposition with
  OpenCV 3rd party. The matrix must be real symmetric positive defined.

  \return The inverse matrix.

  Here an example:
  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    unsigned int n = 4;
    vpMatrix A(n, n);
    vpMatrix I;
    I.eye(4);

    A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.; A[0][3] = 1/4.;
    A[1][0] = 1/5.; A[1][1] = 1/3.; A[1][2] = 1/3.; A[1][3] = 1/5.;
    A[2][0] = 1/6.; A[2][1] = 1/4.; A[2][2] = 1/2.; A[2][3] = 1/6.;
    A[3][0] = 1/7.; A[3][1] = 1/5.; A[3][2] = 1/6.; A[3][3] = 1/7.;

    // Make matrix symmetric positive
    A = 0.5*(A+A.t());
    A = A + n*I;

    // Compute the inverse
    vpMatrix A_1 = A.inverseByCholeskyOpenCV();
    std::cout << "Inverse by Cholesky (OpenCV): \n" << A_1 << std::endl;

    std::cout << "A*A^-1: \n" << A * A_1 << std::endl;
  }
  \endcode

  \sa inverseByCholesky(), inverseByCholeskyLapack()
*/
vpMatrix vpMatrix::inverseByCholeskyOpenCV() const
{
  if (rowNum != colNum) {
    throw(
        vpException(vpException::fatalError, "Cannot inverse a non square matrix (%ux%u) by Cholesky", rowNum, colNum));
  }

  cv::Mat M(rowNum, colNum, CV_64F, this->data);
  cv::Mat Minv = M.inv(cv::DECOMP_CHOLESKY);

  vpMatrix A(rowNum, colNum);
  memcpy(A.data, Minv.data, (size_t)(8 * Minv.rows * Minv.cols));

  return A;
}

/*!
 * \brief Compute the Cholesky decomposition of a Hermitian positive-definite matrix
 * using OpenCV library.
 *
 * \return vpMatrix The lower triangular matrix resulting from the Cholesky decomposition
 */
vpMatrix vpMatrix::choleskyByOpenCV() const
{
  cv::Mat M(rowNum, colNum, CV_64F);
  memcpy(M.data, this->data, (size_t)(8 * M.rows * M.cols));
  std::size_t bytes_per_row = sizeof(double)*rowNum;
  bool result = cv::Cholesky(M.ptr<double>(), bytes_per_row, rowNum, nullptr, bytes_per_row, rowNum);
  if (!result) {
    throw(vpMatrixException(vpMatrixException::fatalError, "Could not compute the Cholesky's decomposition of the input matrix."));
  }
  vpMatrix L(rowNum, colNum);
  memcpy(L.data, M.data, (size_t)(8 * M.rows * M.cols));
  // Make sure that the upper part of L is null
  unsigned int nbRows = this->getRows();
  unsigned int nbCols = this->getCols();
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = r + 1; c < nbCols; ++c) {
      L[r][c] = 0.;
    }
  }
  return L;
}
#endif // VISP_HAVE_OPENCV

#if defined(VISP_HAVE_EIGEN3)
/*!
 * \brief Compute the Cholesky decomposition of a Hermitian positive-definite matrix
 * using Eigen3 library.
 *
 * \return vpMatrix The lower triangular matrix resulting from the Cholesky decomposition
 */
vpMatrix vpMatrix::choleskyByEigen3() const
{
  unsigned int nbRows = this->getRows();
  unsigned int nbCols = this->getCols();
  Eigen::MatrixXd A(nbRows, nbCols);
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c < nbCols; ++c) {
      A(r, c) = (*this)[r][c];
    }
  }
  Eigen::MatrixXd L = A.llt().matrixL();
  vpMatrix Lvisp(static_cast<unsigned int>(L.rows()), static_cast<unsigned int>(L.cols()), 0.);
  for (unsigned int r = 0; r < nbRows; ++r) {
    for (unsigned int c = 0; c <= r; ++c) {
      Lvisp[r][c] = L(r, c);
    }
  }
  return Lvisp;
}
#endif // VISP_HAVE_EIGEN3

END_VISP_NAMESPACE
