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
 * Matrix LU decomposition.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

#ifdef VISP_HAVE_EIGEN3
#include <Eigen/LU>
#endif

#ifdef VISP_HAVE_GSL
#include <gsl/gsl_linalg.h>
#include <gsl/gsl_permutation.h>
#endif

#ifdef VISP_HAVE_LAPACK
#  ifdef VISP_HAVE_MKL
#include <mkl.h>
typedef MKL_INT integer;
#  else
#    ifdef VISP_HAVE_LAPACK_BUILT_IN
typedef long int integer;
#    else
typedef int integer;
#    endif
extern "C" int dgetrf_(integer *m, integer *n, double *a, integer *lda, integer *ipiv, integer *info);
extern "C" void dgetri_(integer *n, double *a, integer *lda, integer *ipiv, double *work, integer *lwork,
                        integer *info);
#  endif
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
#include <opencv2/core/core.hpp>
#endif

// Exception
#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrixException.h>

#include <cmath>  // std::fabs
#include <limits> // numeric_limits

/*--------------------------------------------------------------------
  LU Decomposition  related functions
-------------------------------------------------------------------- */

/*!
  Compute the inverse of a n-by-n matrix using the LU decomposition.

  This function calls the first following function that is available:
  - inverseByLULapack() if Lapack 3rd party is installed
  - inverseByLUEigen3() if Eigen3 3rd party is installed
  - inverseByLUOpenCV() if OpenCV 3rd party is installed
  - inverseByLUGsl() if GSL 3rd party is installed.

  If none of these previous 3rd parties is installed, we use by default
inverseByLULapack() with a Lapack built-in version.

  \return The inverse matrix.

  Here an example:
  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(4,4);

  A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.; A[0][3] = 1/4.;
  A[1][0] = 1/5.; A[1][1] = 1/3.; A[1][2] = 1/3.; A[1][3] = 1/5.;
  A[2][0] = 1/6.; A[2][1] = 1/4.; A[2][2] = 1/2.; A[2][3] = 1/6.;
  A[3][0] = 1/7.; A[3][1] = 1/5.; A[3][2] = 1/6.; A[3][3] = 1/7.;

  // Compute the inverse
  vpMatrix A_1 = A.inverseByLU();

  std::cout << "Inverse by LU ";
#if defined(VISP_HAVE_LAPACK)
  std::cout << "(using Lapack)";
#elif defined(VISP_HAVE_EIGEN3)
  std::cout << "(using Eigen3)";
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  std::cout << "(using OpenCV)";
#elif defined (VISP_HAVE_GSL)
  std::cout << "(using GSL)";
#endif
  std::cout << ": \n" << A_1 << std::endl;

  std::cout << "A*A^-1: \n" << A * A_1 << std::endl;
}
  \endcode

  \sa inverseByLULapack(), inverseByLUEigen3(), inverseByLUOpenCV(),
inverseByLUGsl(), pseudoInverse()
*/
vpMatrix vpMatrix::inverseByLU() const
{
#if defined(VISP_HAVE_LAPACK)
  return inverseByLULapack();
#elif defined(VISP_HAVE_EIGEN3)
  return inverseByLUEigen3();
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101)
  return inverseByLUOpenCV();
#elif defined(VISP_HAVE_GSL)
  return inverseByLUGsl();
#else
  throw(vpException(vpException::fatalError, "Cannot compute matrix determinant. Install Eigen3, "
                                             "Lapack, OpenCV or GSL 3rd party"));
#endif
}

/*!
  Compute the determinant of a square matrix using the LU decomposition.

  This function calls the first following function that is available:
  - detByLULapack() if Lapack 3rd party is installed
  - detByLUEigen3() if Eigen3 3rd party is installed
  - detByLUOpenCV() if OpenCV 3rd party is installed
  - detByLUGsl() if GSL 3rd party is installed.

  If none of these previous 3rd parties is installed, we use by default
detByLULapack() with a Lapack built-in version.

  \return The determinant of the matrix if the matrix is square.

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
}
  \endcode
  \sa detByLULapack(), detByLUEigen3(), detByLUOpenCV(), detByLUGsl()
*/
double vpMatrix::detByLU() const
{
  if (rowNum == 2 && colNum == 2) {
    return ((*this)[0][0] * (*this)[1][1] - (*this)[0][1] * (*this)[1][0]);
  } else if (rowNum == 3 && colNum == 3) {
    return ((*this)[0][0] * ((*this)[1][1] * (*this)[2][2] - (*this)[1][2] * (*this)[2][1]) -
            (*this)[0][1] * ((*this)[1][0] * (*this)[2][2] - (*this)[1][2] * (*this)[2][0]) +
            (*this)[0][2] * ((*this)[1][0] * (*this)[2][1] - (*this)[1][1] * (*this)[2][0]));
  } else {
#if defined(VISP_HAVE_LAPACK)
    return detByLULapack();
#elif defined(VISP_HAVE_EIGEN3)
    return detByLUEigen3();
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101)
    return detByLUOpenCV();
#elif defined(VISP_HAVE_GSL)
    return detByLUGsl();
#else
    throw(vpException(vpException::fatalError, "Cannot compute matrix determinant. Install Lapack, "
                                               "Eigen3, OpenCV or GSL 3rd party"));
#endif
  }
}


#if defined(VISP_HAVE_GSL)
/*!
  Compute the inverse of a n-by-n matrix using the LU decomposition with GSL
3rd party.

  \return The inverse matrix.

  Here an example:
  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(4,4);

  A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.; A[0][3] = 1/4.;
  A[1][0] = 1/5.; A[1][1] = 1/3.; A[1][2] = 1/3.; A[1][3] = 1/5.;
  A[2][0] = 1/6.; A[2][1] = 1/4.; A[2][2] = 1/2.; A[2][3] = 1/6.;
  A[3][0] = 1/7.; A[3][1] = 1/5.; A[3][2] = 1/6.; A[3][3] = 1/7.;

  // Compute the inverse
  vpMatrix A_1; // A^-1
  A_1 = A.inverseByLUGsl();
  std::cout << "Inverse by LU (GSL): \n" << A_1 << std::endl;

  std::cout << "A*A^-1: \n" << A * A_1 << std::endl;
}
  \endcode

  \sa inverseByLU(), inverseByLUEigen3(), inverseByLULapack(),
inverseByLUOpenCV()
*/
vpMatrix vpMatrix::inverseByLUGsl() const
{
  if (rowNum != colNum) {
    throw(vpException(vpException::fatalError, "Cannot inverse a non square matrix (%ux%u) by LU", rowNum, colNum));
  }

  gsl_matrix *A = gsl_matrix_alloc(rowNum, colNum);

  // copy the input matrix to ensure the function doesn't modify its content
  unsigned int tda = (unsigned int)A->tda;
  for (unsigned int i = 0; i < rowNum; i++) {
    unsigned int k = i * tda;
    for (unsigned int j = 0; j < colNum; j++)
      A->data[k + j] = (*this)[i][j];
  }

  vpMatrix Ainv(rowNum, colNum);

  gsl_matrix inverse;
  inverse.size1 = rowNum;
  inverse.size2 = colNum;
  inverse.tda = inverse.size2;
  inverse.data = Ainv.data;
  inverse.owner = 0;
  inverse.block = 0;

  gsl_permutation *p = gsl_permutation_alloc(rowNum);
  int s;

  // Do the LU decomposition on A and use it to solve the system
  gsl_linalg_LU_decomp(A, p, &s);
  gsl_linalg_LU_invert(A, p, &inverse);

  gsl_permutation_free(p);
  gsl_matrix_free(A);

  return Ainv;
}

/*!
  Compute the determinant of a square matrix using the LU decomposition with
GSL 3rd party.

  \return The determinant of the matrix if the matrix is square.

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
  std:: cout << "Determinant by LU decomposition (GSL): " << A.detByLUGsl() << std::endl;
}
  \endcode
  \sa detByLU(), detByLUEigen3(), detByLUOpenCV(), detByLULapack()
*/
double vpMatrix::detByLUGsl() const
{
  double det = 0.;

  if (rowNum != colNum) {
    throw(vpException(vpException::fatalError, "Cannot compute matrix determinant of a non square matrix (%ux%u)",
                      rowNum, colNum));
  }

  gsl_matrix *A = gsl_matrix_alloc(rowNum, colNum);

  // copy the input matrix to ensure the function doesn't modify its content
  unsigned int tda = (unsigned int)A->tda;
  for (unsigned int i = 0; i < rowNum; i++) {
    unsigned int k = i * tda;
    for (unsigned int j = 0; j < colNum; j++)
      A->data[k + j] = (*this)[i][j];
  }

  gsl_permutation *p = gsl_permutation_alloc(rowNum);
  int s;

  // Do the LU decomposition on A and use it to solve the system
  gsl_linalg_LU_decomp(A, p, &s);
  det = gsl_linalg_LU_det(A, s);

  gsl_permutation_free(p);
  gsl_matrix_free(A);

  return det;
}
#endif

#if defined(VISP_HAVE_LAPACK)
/*!
  Compute the inverse of a n-by-n matrix using the LU decomposition with
Lapack 3rd party.

  \return The inverse matrix.

  Here an example:
  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(4,4);

  A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.; A[0][3] = 1/4.;
  A[1][0] = 1/5.; A[1][1] = 1/3.; A[1][2] = 1/3.; A[1][3] = 1/5.;
  A[2][0] = 1/6.; A[2][1] = 1/4.; A[2][2] = 1/2.; A[2][3] = 1/6.;
  A[3][0] = 1/7.; A[3][1] = 1/5.; A[3][2] = 1/6.; A[3][3] = 1/7.;

  // Compute the inverse
  vpMatrix A_1; // A^-1
  A_1 = A.inverseByLULapack();
  std::cout << "Inverse by LU (Lapack): \n" << A_1 << std::endl;

  std::cout << "A*A^-1: \n" << A * A_1 << std::endl;
}
  \endcode

  \sa inverseByLU(), inverseByLUEigen3(), inverseByLUGsl(),
inverseByLUOpenCV()
*/
vpMatrix vpMatrix::inverseByLULapack() const
{
  if (rowNum != colNum) {
    throw(vpException(vpException::fatalError, "Cannot inverse a non square matrix (%ux%u) by LU", rowNum, colNum));
  }

  integer dim = (integer)rowNum;
  integer lda = dim;
  integer info;
  integer lwork = dim * dim;
  integer *ipiv = new integer[dim + 1];
  double *work = new double[lwork];

  vpMatrix A = *this;

  dgetrf_(&dim, &dim, A.data, &lda, &ipiv[1], &info);
  if (info) {
    delete[] ipiv;
    delete[] work;
    throw(vpException(vpException::fatalError, "Lapack LU decomposition failed; info=%d", info));
  }

  dgetri_(&dim, A.data, &dim, &ipiv[1], work, &lwork, &info);

  delete[] ipiv;
  delete[] work;

  return A;
}

/*!
  Compute the determinant of a square matrix using the LU decomposition with
GSL 3rd party.

  \return The determinant of the matrix if the matrix is square.

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
  std:: cout << "Determinant by LU decomposition (Lapack): " << A.detByLULapack() << std::endl;
}
  \endcode
  \sa detByLU(), detByLUEigen3(), detByLUGsl(), detByLUOpenCV()
*/
double vpMatrix::detByLULapack() const
{
  if (rowNum != colNum) {
    throw(vpException(vpException::fatalError, "Cannot compute matrix determinant of a non square matrix (%ux%u)",
                      rowNum, colNum));
  }

  integer dim = (integer)rowNum;
  integer lda = dim;
  integer info;
  integer *ipiv = new integer[dim + 1];

  vpMatrix A = *this;

  dgetrf_(&dim, &dim, A.data, &lda, &ipiv[1], &info);
  if (info < 0) {
    delete[] ipiv;
    throw(vpException(vpException::fatalError, "Lapack LU decomposition failed; info=%d", -info));
  }

  double det = A[0][0];
  for (unsigned int i = 1; i < rowNum; i++) {
    det *= A[i][i];
  }

  double sign = 1.;
  for (int i = 1; i <= dim; i++) {
    if (ipiv[i] != i)
      sign = -sign;
  }

  det *= sign;

  delete[] ipiv;

  return det;
}
#endif

#if (VISP_HAVE_OPENCV_VERSION >= 0x020101)
/*!
  Compute the inverse of a n-by-n matrix using the LU decomposition with
OpenCV 3rd party.

  \return The inverse matrix.

  Here an example:
  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(4,4);

  A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.; A[0][3] = 1/4.;
  A[1][0] = 1/5.; A[1][1] = 1/3.; A[1][2] = 1/3.; A[1][3] = 1/5.;
  A[2][0] = 1/6.; A[2][1] = 1/4.; A[2][2] = 1/2.; A[2][3] = 1/6.;
  A[3][0] = 1/7.; A[3][1] = 1/5.; A[3][2] = 1/6.; A[3][3] = 1/7.;

  // Compute the inverse
  vpMatrix A_1; // A^-1
  A_1 = A.inverseByLUOpenCV();
  std::cout << "Inverse by LU (OpenCV): \n" << A_1 << std::endl;

  std::cout << "A*A^-1: \n" << A * A_1 << std::endl;
}
  \endcode

  \sa inverseByLU(), inverseByLUEigen3(), inverseByLUGsl(),
inverseByLULapack()
*/
vpMatrix vpMatrix::inverseByLUOpenCV() const
{
  if (rowNum != colNum) {
    throw(vpException(vpException::fatalError, "Cannot inverse a non square matrix (%ux%u) by LU", rowNum, colNum));
  }

  cv::Mat M(rowNum, colNum, CV_64F, this->data);

  cv::Mat Minv = M.inv(cv::DECOMP_LU);

  vpMatrix A(rowNum, colNum);
  memcpy(A.data, Minv.data, (size_t)(8 * Minv.rows * Minv.cols));

  return A;
}

/*!
  Compute the determinant of a n-by-n matrix using the LU decomposition with
OpenCV 3rd party.

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
  std:: cout << "Determinant by LU decomposition (OpenCV): " << A.detByLUOpenCV() << std::endl;
}
  \endcode
  \sa detByLU(), detByLUEigen3(), detByLUGsl(), detByLULapack()
*/
double vpMatrix::detByLUOpenCV() const
{
  double det = 0.;

  if (rowNum != colNum) {
    throw(vpException(vpException::fatalError, "Cannot compute matrix determinant of a non square matrix (%ux%u)",
                      rowNum, colNum));
  }

  cv::Mat M(rowNum, colNum, CV_64F, this->data);
  det = cv::determinant(M);

  return (det);
}
#endif

#if defined(VISP_HAVE_EIGEN3)

/*!
  Compute the inverse of a n-by-n matrix using the LU decomposition with
Eigen3 3rd party.

  \return The inverse matrix.

  Here an example:
  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(4,4);

  A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.; A[0][3] = 1/4.;
  A[1][0] = 1/5.; A[1][1] = 1/3.; A[1][2] = 1/3.; A[1][3] = 1/5.;
  A[2][0] = 1/6.; A[2][1] = 1/4.; A[2][2] = 1/2.; A[2][3] = 1/6.;
  A[3][0] = 1/7.; A[3][1] = 1/5.; A[3][2] = 1/6.; A[3][3] = 1/7.;

  // Compute the inverse
  vpMatrix A_1; // A^-1
  A_1 = A.inverseByLUEigen3();
  std::cout << "Inverse by LU (Eigen3): \n" << A_1 << std::endl;

  std::cout << "A*A^-1: \n" << A * A_1 << std::endl;
}
  \endcode

  \sa inverseByLU(), inverseByLULapack(), inverseByLUOpenCV(),
inverseByLUGsl()
*/
vpMatrix vpMatrix::inverseByLUEigen3() const
{
  if (rowNum != colNum) {
    throw(vpException(vpException::fatalError, "Cannot inverse a non square matrix (%ux%u) by LU", rowNum, colNum));
  }
  vpMatrix A(this->getRows(), this->getCols());

  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > M(this->data, this->getRows(),
                                                                                        this->getCols());
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > A_(A.data, this->getRows(),
                                                                                         this->getCols());

  A_ = M.inverse();

  return A;
}

/*!
  Compute the determinant of a square matrix using the LU decomposition with
Eigen3 3rd party.

  \return The determinant of the matrix if the matrix is square.

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
  std:: cout << "Determinant by LU decomposition (Eigen3): " << A.detByLUEigen3() << std::endl;
}
  \endcode
  \sa detByLU(), detByLUOpenCV(), detByLULapack()
*/
double vpMatrix::detByLUEigen3() const
{
  if (rowNum != colNum) {
    throw(vpException(vpException::fatalError, "Cannot compute matrix determinant of a non square matrix (%ux%u)",
                      rowNum, colNum));
  }

  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > M(this->data, this->getRows(),
                                                                                        this->getCols());

  return M.determinant();
}
#endif
