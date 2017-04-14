/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
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

#include <algorithm> // for (std::min) and (std::max)
#include <visp3/core/vpConfig.h>

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpColVector.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrixException.h>

// Debug trace
#include <visp3/core/vpDebug.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS
#ifdef VISP_HAVE_LAPACK
#  ifdef VISP_HAVE_LAPACK_BUILT_IN
typedef long int integer;
#  else
typedef int integer;
#  endif

extern "C" int dgeqrf_(integer *m, integer *n, double *a, integer *lda, double *tau, double *work, integer *lwork, integer *info);
extern "C" int dormqr_(char *side, char *trans, integer *m, integer *n,
                       integer *k, double *a, integer *lda, double *tau, double *c__,
                       integer *ldc, double *work, integer *lwork, integer *info);
extern "C" int dorgqr_(integer *, integer *, integer *, double *, integer *,
                       double *, double *, integer *, integer *);
extern "C" int dtrtri_(char *uplo, char *diag, integer *n, double *a, integer *lda, integer *info);

int allocate_work(double** work);

int allocate_work(double** work)
{
  unsigned int dimWork = (unsigned int)((*work)[0]);
  delete[] *work;
  *work = new double[dimWork];
  return (int)dimWork;
}
#endif

#ifdef VISP_HAVE_LAPACK
/*!
  Compute the inverse of a n-by-n matrix using the QR decomposition with Lapack 3rd party.

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
  if ( rowNum != colNum) {
    throw(vpMatrixException(vpMatrixException::matrixError,
                            "Cannot inverse a non-square matrix (%ux%u) by QR", rowNum, colNum)) ;
  }

  integer rowNum_ = (integer)this->getRows();
  integer colNum_ = (integer)this->getCols();
  integer lda = (integer)rowNum_; //lda is the number of rows because we don't use a submatrix
  integer dimTau = (std::min)(rowNum_,colNum_);
  integer dimWork = -1;
  double *tau = new double[dimTau];
  double *work = new double[1];
  integer info;
  vpMatrix C;
  vpMatrix A = *this;

  try{
    //1) Extract householder reflections (useful to compute Q) and R
    dgeqrf_(
            &rowNum_,        //The number of rows of the matrix A.  M >= 0.
            &colNum_,        //The number of columns of the matrix A.  N >= 0.
            A.data,     /*On entry, the M-by-N matrix A.
                              On exit, the elements on and above the diagonal of the array
                              contain the min(M,N)-by-N upper trapezoidal matrix R (R is
                              upper triangular if m >= n); the elements below the diagonal,
                              with the array TAU, represent the orthogonal matrix Q as a
                              product of min(m,n) elementary reflectors.
                            */
            &lda,            //The leading dimension of the array A.  LDA >= max(1,M).
            tau,            /*Dimension (min(M,N))
                              The scalar factors of the elementary reflectors
                            */
            work,           //Internal working array. dimension (MAX(1,LWORK))
            &dimWork,       //The dimension of the array WORK.  LWORK >= max(1,N).
            &info           //status
          );

    if(info != 0){
      std::cout << "dgeqrf_:Preparation:" << -info << "th element had an illegal value" << std::endl;
      throw vpMatrixException::badValue;
    }
    dimWork = allocate_work(&work);

    dgeqrf_(
          &rowNum_,        //The number of rows of the matrix A.  M >= 0.
          &colNum_,        //The number of columns of the matrix A.  N >= 0.
          A.data,     /*On entry, the M-by-N matrix A.
                            On exit, the elements on and above the diagonal of the array
                            contain the min(M,N)-by-N upper trapezoidal matrix R (R is
                            upper triangular if m >= n); the elements below the diagonal,
                            with the array TAU, represent the orthogonal matrix Q as a
                            product of min(m,n) elementary reflectors.
                          */
          &lda,            //The leading dimension of the array A.  LDA >= max(1,M).
          tau,            /*Dimension (min(M,N))
                            The scalar factors of the elementary reflectors
                          */
          work,           //Internal working array. dimension (MAX(1,LWORK))
          &dimWork,       //The dimension of the array WORK.  LWORK >= max(1,N).
          &info           //status
        );


    if(info != 0){
      std::cout << "dgeqrf_:" << -info << "th element had an illegal value" << std::endl;
      throw vpMatrixException::badValue;
    }

    //A now contains the R matrix in its upper triangular (in lapack convention)
    C = A;

    //2) Invert R
    dtrtri_((char*)"U",(char*)"N",&dimTau,C.data,&lda,&info);
    if(info!=0){
      if(info < 0)
        std::cout << "dtrtri_:"<< -info << "th element had an illegal value" << std::endl;
      else if(info > 0){
        std::cout << "dtrtri_:R("<< info << "," <<info << ")"<< " is exactly zero.  The triangular matrix is singular and its inverse can not be computed." << std::endl;
        std::cout << "R=" << std::endl << C << std::endl;
      }
      throw vpMatrixException::badValue;
    }

    //3) Zero-fill R^-1
    //the matrix is upper triangular for lapack but lower triangular for visp
    //we fill it with zeros above the diagonal (where we don't need the values)
    for(unsigned int i=0;i<C.getRows();i++)
      for(unsigned int j=0;j<C.getRows();j++)
        if(j>i) C[i][j] = 0.;

    dimWork = -1;
    integer ldc = lda;

    //4) Transpose Q and left-multiply it by R^-1
    //get R^-1*tQ
    //C contains R^-1
    //A contains Q
    dormqr_((char*)"R", (char*)"T", &rowNum_, &colNum_, &dimTau, A.data, &lda, tau, C.data, &ldc, work, &dimWork, &info);
    if(info != 0){
      std::cout << "dormqr_:Preparation"<< -info << "th element had an illegal value" << std::endl;
      throw vpMatrixException::badValue;
    }
    dimWork = allocate_work(&work);

    dormqr_((char*)"R", (char*)"T", &rowNum_, &colNum_, &dimTau, A.data, &lda, tau, C.data, &ldc, work, &dimWork, &info);

    if(info != 0){
      std::cout << "dormqr_:"<< -info << "th element had an illegal value" << std::endl;
      throw vpMatrixException::badValue;
    }
    delete[] tau;
    delete[] work;
  } catch(vpMatrixException&) {
    delete[] tau;
    delete[] work;
    throw;
  }

  return C;
}
#endif
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
  Compute the inverse of a n-by-n matrix using the QR decomposition.
  Only available if Lapack 3rd party is installed. If Lapack is not installed we use a
  Lapack built-in version.

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
vpMatrix
vpMatrix::inverseByQR() const
{
#ifdef VISP_HAVE_LAPACK
  return inverseByQRLapack();
#else
  throw(vpException(vpException::fatalError, "Cannot inverse matrix by QR. Install Lapack 3rd party"));
#endif
}
