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
 * Matrix QR decomposition.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <cmath>     // For std::abs() on iOS
#include <cstdlib>   // For std::abs() on iOS
#include <algorithm> // for (std::min) and (std::max)
#include <visp3/core/vpConfig.h>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrixException.h>

// Debug trace
#include <visp3/core/vpDebug.h>

#ifdef VISP_HAVE_LAPACK
#  ifdef VISP_HAVE_MKL
#include <mkl.h>
typedef MKL_INT integer;

integer allocate_work(double **work)
{
  integer dimWork = (integer)((*work)[0]);
  delete[] * work;
  *work = new double[dimWork];
  return (integer)dimWork;
}
#  else
#    ifdef VISP_HAVE_LAPACK_BUILT_IN
typedef long int integer;
#    else
typedef int integer;
#    endif
extern "C" int dgeqrf_(integer *m, integer *n, double *a, integer *lda, double *tau, double *work, integer *lwork,
  integer *info);
extern "C" int dormqr_(char *side, char *trans, integer *m, integer *n, integer *k, double *a, integer *lda,
  double *tau, double *c__, integer *ldc, double *work, integer *lwork, integer *info);
extern "C" int dorgqr_(integer *, integer *, integer *, double *, integer *, double *, double *, integer *, integer *);
extern "C" int dtrtri_(char *uplo, char *diag, integer *n, double *a, integer *lda, integer *info);
extern "C" int dgeqp3_(integer *m, integer *n, double*a, integer *lda, integer *p,
  double *tau, double *work, integer* lwork, integer *info);

int allocate_work(double **work);

int allocate_work(double **work)
{
  unsigned int dimWork = (unsigned int)((*work)[0]);
  delete[] * work;
  *work = new double[dimWork];
  return (int)dimWork;
}
#  endif
#endif

#if defined(VISP_HAVE_LAPACK)
/*!
  Compute the inverse of a n-by-n matrix using the QR decomposition with
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
  vpMatrix A_1 = A.inverseByQRLapack();
  std::cout << "Inverse by QR: \n" << A_1 << std::endl;

  std::cout << "A*A^-1: \n" << A * A_1 << std::endl;
}
  \endcode

  \sa inverseByQR()
*/
vpMatrix vpMatrix::inverseByQRLapack() const
{
  if (rowNum != colNum) {
    throw(vpMatrixException(vpMatrixException::matrixError, "Cannot inverse a non-square matrix (%ux%u) by QR", rowNum,
                            colNum));
  }

  integer rowNum_ = (integer)this->getRows();
  integer colNum_ = (integer)this->getCols();
  integer lda = (integer)rowNum_; // lda is the number of rows because we don't use a submatrix
  integer dimTau = (std::min)(rowNum_, colNum_);
  integer dimWork = -1;
  double *tau = new double[dimTau];
  double *work = new double[1];
  integer info;
  vpMatrix C;
  vpMatrix A = *this;

  try {
    // 1) Extract householder reflections (useful to compute Q) and R
    dgeqrf_(&rowNum_, // The number of rows of the matrix A.  M >= 0.
            &colNum_, // The number of columns of the matrix A.  N >= 0.
            A.data,   /*On entry, the M-by-N matrix A.
                                    On exit, the elements on and above the diagonal of
                                 the array   contain the min(M,N)-by-N upper trapezoidal
                                 matrix R (R is   upper triangular if m >= n); the
                                 elements below the diagonal,   with the array TAU,
                                 represent the orthogonal matrix Q as a   product of
                                 min(m,n) elementary reflectors.
                                  */
            &lda,     // The leading dimension of the array A.  LDA >= max(1,M).
            tau,      /*Dimension (min(M,N))
                                The scalar factors of the elementary reflectors
                              */
            work,     // Internal working array. dimension (MAX(1,LWORK))
            &dimWork, // The dimension of the array WORK.  LWORK >= max(1,N).
            &info     // status
            );

    if (info != 0) {
      std::cout << "dgeqrf_:Preparation:" << -info << "th element had an illegal value" << std::endl;
      throw vpMatrixException::badValue;
    }
    dimWork = allocate_work(&work);

    dgeqrf_(&rowNum_, // The number of rows of the matrix A.  M >= 0.
            &colNum_, // The number of columns of the matrix A.  N >= 0.
            A.data,   /*On entry, the M-by-N matrix A.
                                    On exit, the elements on and above the diagonal of
                                 the array   contain the min(M,N)-by-N upper trapezoidal
                                 matrix R (R is   upper triangular if m >= n); the
                                 elements below the diagonal,   with the array TAU,
                                 represent the orthogonal matrix Q as a   product of
                                 min(m,n) elementary reflectors.
                                  */
            &lda,     // The leading dimension of the array A.  LDA >= max(1,M).
            tau,      /*Dimension (min(M,N))
                                The scalar factors of the elementary reflectors
                              */
            work,     // Internal working array. dimension (MAX(1,LWORK))
            &dimWork, // The dimension of the array WORK.  LWORK >= max(1,N).
            &info     // status
            );

    if (info != 0) {
      std::cout << "dgeqrf_:" << -info << "th element had an illegal value" << std::endl;
      throw vpMatrixException::badValue;
    }

    // A now contains the R matrix in its upper triangular (in lapack
    // convention)
    C = A;

    // 2) Invert R
    dtrtri_((char *)"U", (char *)"N", &dimTau, C.data, &lda, &info);
    if (info != 0) {
      if (info < 0)
        std::cout << "dtrtri_:" << -info << "th element had an illegal value" << std::endl;
      else if (info > 0) {
        std::cout << "dtrtri_:R(" << info << "," << info << ")"
                  << " is exactly zero.  The triangular matrix is singular "
                     "and its inverse can not be computed."
                  << std::endl;
        std::cout << "R=" << std::endl << C << std::endl;
      }
      throw vpMatrixException::badValue;
    }

    // 3) Zero-fill R^-1
    // the matrix is upper triangular for lapack but lower triangular for visp
    // we fill it with zeros above the diagonal (where we don't need the
    // values)
    for (unsigned int i = 0; i < C.getRows(); i++)
      for (unsigned int j = 0; j < C.getRows(); j++)
        if (j > i)
          C[i][j] = 0.;

    dimWork = -1;
    integer ldc = lda;

    // 4) Transpose Q and left-multiply it by R^-1
    // get R^-1*tQ
    // C contains R^-1
    // A contains Q
    dormqr_((char *)"R", (char *)"T", &rowNum_, &colNum_, &dimTau, A.data, &lda, tau, C.data, &ldc, work, &dimWork,
            &info);
    if (info != 0) {
      std::cout << "dormqr_:Preparation" << -info << "th element had an illegal value" << std::endl;
      throw vpMatrixException::badValue;
    }
    dimWork = allocate_work(&work);

    dormqr_((char *)"R", (char *)"T", &rowNum_, &colNum_, &dimTau, A.data, &lda, tau, C.data, &ldc, work, &dimWork,
            &info);

    if (info != 0) {
      std::cout << "dormqr_:" << -info << "th element had an illegal value" << std::endl;
      throw vpMatrixException::badValue;
    }
    delete[] tau;
    delete[] work;
  } catch (vpMatrixException &) {
    delete[] tau;
    delete[] work;
    throw;
  }

  return C;
}
#endif

/*!
  Compute the inverse of a n-by-n matrix using the QR decomposition.
  Only available if Lapack 3rd party is installed. If Lapack is not installed
  we use a Lapack built-in version.

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
  vpMatrix A_1 = A.inverseByQR();
  std::cout << "Inverse by QR: \n" << A_1 << std::endl;

  std::cout << "A*A^-1: \n" << A * A_1 << std::endl;
}
  \endcode

  \sa inverseByLU(), inverseByCholesky()
*/
vpMatrix vpMatrix::inverseByQR() const
{
#if defined(VISP_HAVE_LAPACK)
  return inverseByQRLapack();
#else
  throw(vpException(vpException::fatalError, "Cannot inverse matrix by QR. Install Lapack 3rd party"));
#endif
}


/*!
  Compute the QR decomposition of a (m x n) matrix of rank r.
  Only available if Lapack 3rd party is installed. If Lapack is not installed
  we use a Lapack built-in version.

  \param Q : orthogonal matrix (will be modified).
  \param R : upper-triangular matrix (will be modified).
  \param full : whether or not we want full decomposition.
  \param squareR : will return only the square (min(m,n) x min(m,n)) part of R.
  \param tol : tolerance to test the rank of R.

  \return The rank r of the matrix.

  If full is false (default) then Q is (m x min(n,m)) and R is (min(n,m) x n).
  We then have this = QR.

  If full is true and m > n then Q is (m x m) and R is (n x n).
  In this case this = Q (R, 0)^T

  If squareR is true and n > m then R is (m x m). If r = m then R is invertible.

  Here an example:
  \code
#include <visp3/core/vpMatrix.h>

double residual(vpMatrix M1, vpMatrix M2)
{
    return (M1 - M2).frobeniusNorm();
}

int main()
{
  vpMatrix A(4,3);

  A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.;
  A[1][0] = 1/5.; A[1][1] = 1/3.; A[1][2] = 1/3.;
  A[2][0] = 1/6.; A[2][1] = 1/4.; A[2][2] = 1/2.;
  A[3][0] = 1/7.; A[3][1] = 1/5.; A[3][2] = 1/6.;

  // Economic QR (Q 4x3, R 3x3)
  vpMatrix Q, R;
  int r = A.qr(A, R);
  std::cout << "QR Residual: "
            << residual(A, Q*R) << std::endl;

  // Full QR (Q 4x4, R 3x3)
  r = A.qr(Q, R, true);
  std::cout << "Full QR Residual: "
            << residual(A, Q.extract(0, 0, 4, 3)*R) << std::endl;
}
  \endcode

  \sa qrPivot()
*/
unsigned int vpMatrix::qr(vpMatrix &Q, vpMatrix &R, bool full, bool squareR, double tol) const
{
#if defined(VISP_HAVE_LAPACK)
  integer m = (integer)rowNum;     // also rows of Q
  integer n = (integer)colNum;     // also columns of R
  integer r = std::min(n, m);  // a priori non-null rows of R = rank of R
  integer q = r;              // columns of Q and rows of R
  integer na = n;             // columns of A

  // cannot be full decomposition if m < n
  if(full && m > n)
  {
    q = m;              // Q is square
    na = m;             // A is square
  }

  // prepare matrices and deal with r = 0
  Q.resize(static_cast<unsigned int>(m), static_cast<unsigned int>(q));
  if(squareR)
    R.resize(static_cast<unsigned int>(r), static_cast<unsigned int>(r));
  else
    R.resize(static_cast<unsigned int>(r), static_cast<unsigned int>(n));
  if(r == 0)
    return 0;

  integer dimWork = -1;
  double * qrdata = new double[m*na];
  double *tau = new double[std::min(m,q)];
  double *work = new double[1];
  integer info;

  // copy this to qrdata in Lapack convention
  for (integer i = 0; i < m; ++i)
  {
    for (integer j = 0; j < n; ++j)
      qrdata[i + m * j] = data[j + n * i];
    for (integer j = n; j < na; ++j)
      qrdata[i + m * j] = 0;
  }

  //   work = new double[1];
  //1) Extract householder reflections (useful to compute Q) and R
  dgeqrf_(
        &m,        //The number of rows of the matrix A.  M >= 0.
        &na,        //The number of columns of the matrix A.  N >= 0.
        qrdata,
        &m,
        tau,
        work,           //Internal working array. dimension (MAX(1,LWORK))
        &dimWork,       //The dimension of the array WORK.  LWORK >= max(1,N).
        &info           //status
        );

  if(info != 0){
    std::cout << "dgeqrf_:Preparation:" << -info << "th element had an illegal value" << std::endl;
    delete[] qrdata;
    delete[] work;
    delete[] tau;
    throw vpMatrixException::badValue;
  }
  dimWork = allocate_work(&work);

  dgeqrf_(
        &m,        //The number of rows of the matrix A.  M >= 0.
        &na,        //The number of columns of the matrix A.  N >= 0.
        qrdata,
        &m,            //The leading dimension of the array A.  LDA >= max(1,M).
        tau,
        work,           //Internal working array. dimension (MAX(1,LWORK))
        &dimWork,       //The dimension of the array WORK.  LWORK >= max(1,N).
        &info           //status
        );

  if(info != 0){
    std::cout << "dgeqrf_:" << -info << "th element had an illegal value" << std::endl;
    delete[] qrdata;
    delete[] work;
    delete[] tau;
    throw vpMatrixException::badValue;
  }

  // data now contains the R matrix in its upper triangular (in lapack convention)

  // copy useful part of R from Q and update rank
  na = std::min(m,n);
  if(squareR)
  {
    for(int i=0;i<na;i++)
    {
      for(int j=i;j<na;j++)
        R[i][j] = qrdata[i+m*j];
      if(std::abs(qrdata[i+m*i]) < tol)
        r--;
    }
  }
  else
  {
    for(int i=0;i<na;i++)
    {
      for(int j=i;j<n;j++)
        R[i][j] = qrdata[i+m*j];
      if(std::abs(qrdata[i+m*i]) < tol)
        r--;
    }
  }

  // extract Q
  dorgqr_(&m, // The number of rows of the matrix Q. M >= 0.
          &q, // The number of columns of the matrix Q. M >= N >= 0.
          &q,
          qrdata,
          &m,            //The leading dimension of the array A.  LDA >= max(1,M).
          tau,
          work,           //Internal working array. dimension (MAX(1,LWORK))
          &dimWork,       //The dimension of the array WORK.  LWORK >= max(1,N).
          &info           //status
          );

  // write qrdata into Q
  for(int i = 0; i < m; ++i)
    for(int j = 0; j < q; ++j)
      Q[i][j] = qrdata[i+m*j];

  delete[] qrdata;
  delete[] work;
  delete[] tau;
  return (unsigned int) r;
#else
  (void)Q;
  (void)R;
  (void)full;
  (void)squareR;
  (void)tol;
  throw(vpException(vpException::fatalError, "Cannot perform QR decomposition. Install Lapack 3rd party"));
#endif
}


/*!
  Compute the QR pivot decomposition of a (m x n) matrix of rank r.
  Only available if Lapack 3rd party is installed. If Lapack is not installed
  we use a Lapack built-in version.

  \param Q : orthogonal matrix (will be modified).
  \param R : upper-triangular matrix (will be modified).
  \param P : the (n x n) permutation matrix.
  \param full : whether or not we want full decomposition.
  \param squareR : will return only the (r x r) part of R and the (r x n) part of P.
  \param tol : tolerance to test the rank of R.

  \return The rank r of the matrix.

  If full is false (default) then Q is (m x min(n,m)) and R is (min(n,m) x n).
  We then have this.P = Q.R.

  If full is true and m > n then Q is (m x m) and R is (n x n).
  In this case this.P = Q (R, 0)^T

  If squareR is true then R is (r x r) invertible.

  Here an example:
  \code
#include <visp3/core/vpMatrix.h>

double residual(vpMatrix M1, vpMatrix M2)
{
    return (M1 - M2).frobeniusNorm();
}

int main()
{
  vpMatrix A(4,3);

  A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/2.;
  A[1][0] = 1/5.; A[1][1] = 1/3.; A[1][2] = 1/3.;
  A[2][0] = 1/6.; A[2][1] = 1/4.; A[2][2] = 1/4.;
  A[3][0] = 1/7.; A[3][1] = 1/5.; A[3][2] = 1/5.;
  // A is rank (4x3) but rank 2

  // Economic QR (Q 4x3, R 3x3)
  vpMatrix Q, R, P;
  int r = A.qrPivot(Q, R, P);
  std::cout << "A rank: " << r << std::endl;
  std::cout << "Residual: " << residual(A*P, Q*R) << std::endl;

  // Full QR (Q 4x4, R 3x3)
  r = A.qrPivot(Q, R, P, true);
  std::cout << "QRPivot Residual: " <<
  residual(A*P, Q.extract(0, 0, 4, 3)*R) << std::endl;

  // Using permutation matrix: keep only non-null part of R
  Q.resize(4, r, false);            // Q is 4 x 2
  R = R.extract(0, 0, r, 3)*P.t();  // R is 2 x 3
  std::cout << "Full QRPivot Residual: " <<
  residual(A, Q*R) << std::endl;
}
  \endcode

  \sa qrPivot()
*/
unsigned int vpMatrix::qrPivot(vpMatrix &Q, vpMatrix &R, vpMatrix &P, bool full, bool squareR, double tol) const
{
#if defined(VISP_HAVE_LAPACK)
  integer m = (integer) rowNum;     // also rows of Q
  integer n = (integer) colNum;     // also columns of R
  integer r = std::min(n,m);             // a priori non-null rows of R = rank of R
  integer q = r;                         // columns of Q and rows of R
  integer na = n;

  // cannot be full decomposition if m < n
  if(full && m > n)
  {
    q = m;              // Q is square
    na = m;
  }

  // prepare Q and deal with r = 0
  Q.resize(static_cast<unsigned int>(m), static_cast<unsigned int>(q));
  if(r == 0)
  {
    if(squareR)
    {
      R.resize(0, 0);
      P.resize(0, static_cast<unsigned int>(n));
    }
    else
    {
      R.resize(static_cast<unsigned int>(r), static_cast<unsigned int>(n));
      P.resize(static_cast<unsigned int>(n), static_cast<unsigned int>(n));
    }
    return 0;
  }

  integer dimWork = -1;
  double* qrdata = new double[m*na];
  double* tau = new double[std::min(q,m)];
  double* work = new double[1];
  integer* p = new integer[na];
  for(int i = 0; i < na; ++i)
    p[i] = 0;

  integer info;

  // copy this to qrdata in Lapack convention
  for(integer i = 0; i < m; ++i)
  {
    for(integer j = 0; j < n; ++j)
      qrdata[i+m*j] = data[j + n*i];
    for(integer j = n; j < na; ++j)
      qrdata[i+m*j] = 0;
  }

  //1) Extract householder reflections (useful to compute Q) and R
  dgeqp3_(
        &m,        //The number of rows of the matrix A.  M >= 0.
        &na,        //The number of columns of the matrix A.  N >= 0.
        qrdata,    /*On entry, the M-by-N matrix A.        */
        &m,      //The leading dimension of the array A.  LDA >= max(1,M).
        p,         // Dimension N
        tau,        /*Dimension (min(M,N))        */
        work,       //Internal working array. dimension (3*N)

        &dimWork,
        &info       //status
        );

  if(info != 0){
    std::cout << "dgeqp3_:Preparation:" << -info << "th element had an illegal value" << std::endl;
    delete[] qrdata;
    delete[] work;
    delete[] tau;
    delete[] p;
    throw vpMatrixException::badValue;
  }

  dimWork = allocate_work(&work);

  dgeqp3_(
        &m,        //The number of rows of the matrix A.  M >= 0.
        &na,        //The number of columns of the matrix A.  N >= 0.
        qrdata,    /*On entry, the M-by-N matrix A.        */
        &m,      //The leading dimension of the array A.  LDA >= max(1,M).
        p,         // Dimension N
        tau,        /*Dimension (min(M,N))        */
        work,       //Internal working array. dimension (3*N)

        &dimWork,
        &info       //status
        );

  if(info != 0){
    std::cout << "dgeqp3_:" << -info << " th element had an illegal value" << std::endl;
    delete[] qrdata;
    delete[] work;
    delete[] tau;
    delete[] p;
    throw vpMatrixException::badValue;
  }


  // data now contains the R matrix in its upper triangular (in lapack convention)
  // get rank of R in r
  na = std::min(n,m);
  for(int i = 0; i < na; ++i)
    if(std::abs(qrdata[i+m*i]) < tol)
      r--;

  // write R
  if(squareR) // R r x r
  {
    R.resize(static_cast<unsigned int>(r), static_cast<unsigned int>(r));
    for(int i=0;i<r;i++)
      for(int j=i;j<r;j++)
        R[i][j] = qrdata[i+m*j];

    // write P
    P.resize(static_cast<unsigned int>(r), static_cast<unsigned int>(n));
    for(int i = 0; i < r; ++i)
      P[i][p[i]-1] = 1;
  }
  else        // R is min(m,n) x n of rank r
  {
    R.resize(static_cast<unsigned int>(na), static_cast<unsigned int>(n));
    for(int i=0;i<na;i++)
      for(int j=i;j<n;j++)
        R[i][j] = qrdata[i+m*j];
    // write P
    P.resize(static_cast<unsigned int>(n), static_cast<unsigned int>(n));
    for(int i = 0; i < n; ++i)
      P[i][p[i]-1] = 1;
  }

  // extract Q
  dorgqr_(&m, // The number of rows of the matrix Q. M >= 0.
          &q, // The number of columns of the matrix Q. M >= N >= 0.
          &q,
          qrdata,
          &m,            //The leading dimension of the array A.  LDA >= max(1,M).
          tau,
          work,           //Internal working array. dimension (MAX(1,LWORK))
          &dimWork,       //The dimension of the array WORK.  LWORK >= max(1,N).
          &info           //status
          );

  // write qrdata into Q
  for(int i = 0; i < m; ++i)
    for(int j = 0; j < q; ++j)
      Q[i][j] = qrdata[i+m*j];

  delete[] qrdata;
  delete[] work;
  delete[] tau;
  delete[] p;
  return (unsigned int) r;
#else
  (void)Q;
  (void)R;
  (void)P;
  (void)full;
  (void)squareR;
  (void)tol;
  throw(vpException(vpException::fatalError, "Cannot perform QR decomposition. Install Lapack 3rd party"));
#endif
}

/*!
  Compute the inverse of a full-rank n-by-n triangular matrix.
  Only available if Lapack 3rd party is installed. If Lapack is not installed
  we use a Lapack built-in version.

  \param upper : if it is an upper triangular matrix

  The function does not check if the matrix is actually upper or lower triangular.

  \return The inverse matrix
*/
vpMatrix vpMatrix::inverseTriangular(bool upper) const
{
#if defined(VISP_HAVE_LAPACK)
  if(rowNum != colNum || rowNum == 0)
    throw vpMatrixException::dimensionError;

  integer n = (integer) rowNum; // lda is the number of rows because we don't use a submatrix

  vpMatrix R = *this;
  integer info;

  // we just use the other (upper / lower) method from Lapack
  if(rowNum > 1 && upper)  // upper triangular
    dtrtri_((char *)"L", (char *)"N", &n, R.data, &n, &info);
  else
    dtrtri_((char *)"U", (char *)"N", &n, R.data, &n, &info);

  if (info != 0) {
    if (info < 0)
      std::cout << "dtrtri_:" << -info << "th element had an illegal value" << std::endl;
    else if (info > 0) {
      std::cout << "dtrtri_:R(" << info << "," << info << ")"
                << " is exactly zero.  The triangular matrix is singular "
                   "and its inverse can not be computed."
                << std::endl;
      throw vpMatrixException::rankDeficient;
    }
    throw vpMatrixException::badValue;
  }
  return R;
#else
  (void)upper;
  throw(vpException(vpException::fatalError, "Cannot perform triangular inverse. Install Lapack 3rd party"));
#endif
}


/*!
  Solve a linear system Ax = b using QR Decomposition.

  Non destructive wrt. A and b.

  \param b : Vector  b
  \param x : Vector  x

  \warning If Ax = b does not have a solution, this method does not return the least-square
  minimizer. Use solveBySVD() to get this vector.

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
      A.solveByQR(B, X);
      // Obtained values of X
      // X[0] = 0.2468;
      // X[1] = 0.120782;
      // X[2] = 0.468587;
      std::cout << "X:\n" << X << std::endl;
    }
  \endcode

  \sa qrPivot()
*/
void vpMatrix::solveByQR(const vpColVector &b, vpColVector &x) const
{
  vpMatrix Q, R, P;
  unsigned int r = t().qrPivot(Q, R, P, false, true);
  x = Q.extract(0, 0, colNum, r)
      * R.inverseTriangular().t()
      * P * b;
}

/*!
  Solve a linear system Ax = b using QR Decomposition.

  Non destructive wrt. A and B.

  \param b : Vector b

  \return Vector x

  \warning If Ax = b does not have a solution, this method does not return the least-square
  minimizer. Use solveBySVD() to get this vector.

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
      X = A.solveByQR(B);
      // Obtained values of X
      // X[0] = 0.2468;
      // X[1] = 0.120782;
      // X[2] = 0.468587;
      std::cout << "X:\n" << X << std::endl;
    }
  \endcode

  \sa qrPivot()
*/
vpColVector vpMatrix::solveByQR(const vpColVector &b) const
{
  vpColVector x(colNum);
  solveByQR(b, x);
  return x;
}
