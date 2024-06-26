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
 * Matrix SVD decomposition.
 */

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpMatrixException.h>

#include <cmath> // std::fabs
#include <iostream>

#ifdef VISP_HAVE_EIGEN3
#include <Eigen/SVD>
#endif

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
#else
#if defined(VISP_HAVE_LAPACK_BUILT_IN)
typedef long int integer;
#else
typedef int integer;
#endif
extern "C" int dgesdd_(char *jobz, integer *m, integer *n, double *a, integer *lda, double *s, double *u, integer *ldu,
                       double *vt, integer *ldvt, double *work, integer *lwork, integer *iwork, integer *info);

#include <stdio.h>
#include <string.h>
#endif
#endif

BEGIN_VISP_NAMESPACE

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

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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

  The matrix object `(*this) is updated with \f$ U \f$.

  \note The singular values are ordered in decreasing
  fashion in \e w. It means that the highest singular value is in \e w[0].

  Here an example of SVD decomposition of a non square Matrix M.

  \code
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
#elif defined(VISP_HAVE_OPENCV) // Require opencv >= 2.1.1
  svdOpenCV(w, V);
#else
  (void)w;
  (void)V;
  throw(vpException(vpException::fatalError, "Cannot compute SVD. Install Lapack, Eigen3 or OpenCV 3rd party"));
#endif
}

#if defined(VISP_HAVE_OPENCV) // Require opencv >= 2.1.1

/*!

  Singular value decomposition (SVD) using OpenCV 3rd party.

  Given matrix \f$M\f$, this function computes it singular value decomposition such as

  \f[ M = U \Sigma V^{\top} \f]

  \warning This method is destructive wrt. to the matrix \f$ M \f$ to
  decompose. You should make a COPY of that matrix if needed.

  \param w : Vector of singular values: \f$ \Sigma = diag(w) \f$.

  \param V : Matrix \f$ V \f$.

  The matrix object `(*this)` is updated with \f$ U \f$.

  \note The singular values are ordered in decreasing
  fashion in \e w. It means that the highest singular value is in \e w[0].

  Here an example of SVD decomposition of a non square Matrix M.

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix M(3,2);
    M[0][0] = 1;
    M[1][0] = 2;
    M[2][0] = 0.5;

    M[0][1] = 6;
    M[1][1] = 8 ;
    M[2][1] = 9 ;

    vpMatrix V;
    vpColVector w;
    vpMatrix Mrec;
    vpMatrix Sigma;

    M.svdOpenCV(w, V);
    // Here M is modified and is now equal to U

    // Construct the diagonal matrix from the singular values
    Sigma.diag(w);

    // Reconstruct the initial matrix M using the decomposition
    Mrec =  M * Sigma * V.t();

    // Here, Mrec is obtained equal to the initial value of M
    // Mrec[0][0] = 1;
    // Mrec[1][0] = 2;
    // Mrec[2][0] = 0.5;
    // Mrec[0][1] = 6;
    // Mrec[1][1] = 8 ;
    // Mrec[2][1] = 9 ;

    std::cout << "Reconstructed M matrix: \n" << Mrec << std::endl;
  }
  \endcode

  \sa svd(), svdEigen3(), svdLapack()
*/
void vpMatrix::svdOpenCV(vpColVector &w, vpMatrix &V)
{
  int rows = (int)this->getRows();
  int cols = (int)this->getCols();
  cv::Mat m(rows, cols, CV_64F, this->data);
  cv::SVD opencvSVD(m);
  cv::Mat opencvV = opencvSVD.vt;
  cv::Mat opencvW = opencvSVD.w;
  V.resize((unsigned int)opencvV.rows, (unsigned int)opencvV.cols);
  w.resize((unsigned int)(opencvW.rows * opencvW.cols));

  memcpy(V.data, opencvV.data, (size_t)(8 * opencvV.rows * opencvV.cols));
  V = V.transpose();
  memcpy(w.data, opencvW.data, (size_t)(8 * opencvW.rows * opencvW.cols));
  this->resize((unsigned int)opencvSVD.u.rows, (unsigned int)opencvSVD.u.cols);
  memcpy(this->data, opencvSVD.u.data, (size_t)(8 * opencvSVD.u.rows * opencvSVD.u.cols));
}

#endif

#if defined(VISP_HAVE_LAPACK)
/*!

  Singular value decomposition (SVD) using Lapack 3rd party.

  Given matrix \f$M\f$, this function computes it singular value decomposition
  such as

  \f[ M = U \Sigma V^{\top} \f]

  \warning This method is destructive wrt. to the matrix \f$ M \f$ to
  decompose. You should make a COPY of that matrix if needed.

  \param w : Vector of singular values: \f$ \Sigma = diag(w) \f$.

  \param V : Matrix \f$ V \f$.

  The matrix object `(*this)` is updated with \f$ U \f$.

  \note The singular values are ordered in decreasing
  fashion in \e w. It means that the highest singular value is in \e w[0].

  Here an example of SVD decomposition of a non square Matrix M.

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix M(3,2);
    M[0][0] = 1;
    M[1][0] = 2;
    M[2][0] = 0.5;

    M[0][1] = 6;
    M[1][1] = 8 ;
    M[2][1] = 9 ;

    vpMatrix V;
    vpColVector w;
    vpMatrix Mrec;
    vpMatrix Sigma;

    M.svdLapack(w, V);
    // Here M is modified and is now equal to U

    // Construct the diagonal matrix from the singular values
    Sigma.diag(w);

    // Reconstruct the initial matrix M using the decomposition
    Mrec =  M * Sigma * V.t();

    // Here, Mrec is obtained equal to the initial value of M
    // Mrec[0][0] = 1;
    // Mrec[1][0] = 2;
    // Mrec[2][0] = 0.5;
    // Mrec[0][1] = 6;
    // Mrec[1][1] = 8 ;
    // Mrec[2][1] = 9 ;

    std::cout << "Reconstructed M matrix: \n" << Mrec << std::endl;
  }
  \endcode

  \sa svd(), svdEigen3(), svdOpenCV()
*/
void vpMatrix::svdLapack(vpColVector &w, vpMatrix &V)
{
#ifdef VISP_HAVE_GSL
  {
    // GSL cannot consider M < N. In that case we transpose input matrix
    vpMatrix U;
    unsigned int nc = getCols();
    unsigned int nr = getRows();

    if (rowNum < colNum) {
      U = this->transpose();
      nc = getRows();
      nr = getCols();
    }
    else {
      nc = getCols();
      nr = getRows();
    }

    w.resize(nc);
    V.resize(nc, nc);

    gsl_vector *work = gsl_vector_alloc(nc);

    gsl_matrix A;
    A.size1 = nr;
    A.size2 = nc;
    A.tda = A.size2;
    if (rowNum < colNum) {
      A.data = U.data;
    }
    else {
      A.data = this->data;
    }
    A.owner = 0;
    A.block = 0;

    gsl_matrix V_;
    V_.size1 = nc;
    V_.size2 = nc;
    V_.tda = V_.size2;
    V_.data = V.data;
    V_.owner = 0;
    V_.block = 0;

    gsl_vector S;
    S.size = nc;
    S.stride = 1;
    S.data = w.data;
    S.owner = 0;
    S.block = 0;

    gsl_linalg_SV_decomp(&A, &V_, &S, work);

    if (rowNum < colNum) {
      (*this) = V;
      V = U;
    }

    gsl_vector_free(work);
  }
#else
  {
    vpMatrix U;
    unsigned int nc = getCols();
    unsigned int nr = getRows();

    if (rowNum < colNum) {
      U = this->transpose();
      nc = getRows();
      nr = getCols();
    }
    else {
      nc = getCols();
      nr = getRows();
    }

    w.resize(nc);
    V.resize(nc, nc);

    double *a = new double[static_cast<unsigned int>(nr * nc)];
    if (rowNum < colNum) {
      memcpy(a, U.data, this->getRows() * this->getCols() * sizeof(double));
    }
    else {
      memcpy(a, this->data, this->getRows() * this->getCols() * sizeof(double));
    }

    integer m = (integer)(nc);
    integer n = (integer)(nr);
    integer lda = nc;
    integer ldu = nc;
    integer ldvt = std::min<integer>(nr, nc);
    integer info, lwork;

    double wkopt;
    double *work;

    integer *iwork = new integer[8 * static_cast<integer>(std::min<integer>(nr, nc))];

    double *s = w.data;
    double *u = V.data;
    double *vt;
    if (rowNum < colNum) {
      vt = U.data;
    }
    else {
      vt = this->data;
    }

    lwork = -1;
    dgesdd_((char *)"S", &m, &n, a, &lda, s, u, &ldu, vt, &ldvt, &wkopt, &lwork, iwork, &info);
    lwork = (int)wkopt;
    work = new double[static_cast<unsigned int>(lwork)];

    dgesdd_((char *)"S", &m, &n, a, &lda, s, u, &ldu, vt, &ldvt, work, &lwork, iwork, &info);

    if (info > 0) {
      throw(vpMatrixException(vpMatrixException::fatalError, "The algorithm computing SVD failed to converge."));
    }

    if (rowNum < colNum) {
      (*this) = V.transpose();
      V = U;
    }
    else {
      V = V.transpose();
    }
    delete[] work;
    delete[] iwork;
    delete[] a;
  }
#endif
}
#endif

#ifdef VISP_HAVE_EIGEN3
/*!

  Singular value decomposition (SVD) using Eigen3 3rd party.

  Given matrix \f$M\f$, this function computes it singular value decomposition
  such as

  \f[ M = U \Sigma V^{\top} \f]

  \warning This method is destructive wrt. to the matrix \f$ M \f$ to
  decompose. You should make a COPY of that matrix if needed.

  \param w : Vector of singular values: \f$ \Sigma = diag(w) \f$.

  \param V : Matrix \f$ V \f$.

  The matrix object `(*this)` is updated with \f$ U \f$.

  \note The singular values are ordered in decreasing
  fashion in \e w. It means that the highest singular value is in \e w[0].

  Here an example of SVD decomposition of a non square Matrix M.

  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix M(3,2);
    M[0][0] = 1;
    M[1][0] = 2;
    M[2][0] = 0.5;

    M[0][1] = 6;
    M[1][1] = 8 ;
    M[2][1] = 9 ;

    vpMatrix V;
    vpColVector w;
    vpMatrix Mrec;
    vpMatrix Sigma;

    M.svdEigen3(w, V);
    // Here M is modified and is now equal to U

    // Construct the diagonal matrix from the singular values
    Sigma.diag(w);

    // Reconstruct the initial matrix M using the decomposition
    Mrec =  M * Sigma * V.t();

    // Here, Mrec is obtained equal to the initial value of M
    // Mrec[0][0] = 1;
    // Mrec[1][0] = 2;
    // Mrec[2][0] = 0.5;
    // Mrec[0][1] = 6;
    // Mrec[1][1] = 8 ;
    // Mrec[2][1] = 9 ;

    std::cout << "Reconstructed M matrix: \n" << Mrec << std::endl;
  }
  \endcode

  \sa svd(), svdLapack(), svdOpenCV()
*/
void vpMatrix::svdEigen3(vpColVector &w, vpMatrix &V)
{
  if (rowNum < colNum) {
    w.resize(rowNum);
    V.resize(colNum, rowNum);
  }
  else {
    w.resize(colNum);
    V.resize(colNum, colNum);
  }

  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > M(this->data, this->getRows(),
                                                                                        this->getCols());
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(M, Eigen::ComputeThinU | Eigen::ComputeThinV);

  Eigen::Map<Eigen::VectorXd> w_(w.data, w.size());

  if (rowNum < colNum) {
    this->resize(rowNum, rowNum);
  }
  else {
    this->resize(rowNum, colNum);
  }
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > V_(V.data, V.getRows(), V.getCols());
  Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> > U_(this->data, rowNum, colNum);
  w_ = svd.singularValues();
  V_ = svd.matrixV();
  U_ = svd.matrixU();
}
#endif
END_VISP_NAMESPACE
