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
 * BLAS subroutines.
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#if defined(VISP_HAVE_LAPACK) && !defined(VISP_HAVE_LAPACK_BUILT_IN)
#  ifdef VISP_HAVE_MKL
#include <mkl.h>

void vpMatrix::blas_dgemm(char trans_a, char trans_b, const int M_, const int N_, const int K_, double alpha,
                          double *a_data, const int lda_, double *b_data, const int ldb_, double beta, double *c_data,
                          const int ldc_)
{
  MKL_INT M = (MKL_INT)M_, K = (MKL_INT)K_, N = (MKL_INT)N_;
  MKL_INT lda = (MKL_INT)lda_, ldb = (MKL_INT)ldb_, ldc = (MKL_INT)ldc_;

  dgemm(&trans_a, &trans_b, &M, &N, &K, &alpha, a_data, &lda, b_data, &ldb, &beta, c_data, &ldc);
}

void vpMatrix::blas_dgemv(char trans, const int M_, const int N_, double alpha, double *a_data, const int lda_,
                          double *x_data, const int incx_, double beta, double *y_data, const int incy_)
{
  MKL_INT M = (MKL_INT)M_, N = (MKL_INT)N_;
  MKL_INT lda = (MKL_INT)lda_, incx = (MKL_INT)incx_, incy = (MKL_INT)incy_;

  dgemv(&trans, &M, &N, &alpha, a_data, &lda, x_data, &incx, &beta, y_data, &incy);
}
#  else
typedef int integer;

extern "C" void dgemm_(char *transa, char *transb, integer *M, integer *N, integer *K, double *alpha, double *a,
                       integer *lda, double *b, integer *ldb, double *beta, double *c, integer *ldc);

extern "C" void dgemv_(char *trans, integer *M, integer *N, double *alpha, double *a, integer *lda, double *x,
                       integer *incx, double *beta, double *y, integer *incy);

void vpMatrix::blas_dgemm(char trans_a, char trans_b, const int M_, const int N_, const int K_, double alpha,
                          double *a_data, const int lda_, double *b_data, const int ldb_, double beta, double *c_data,
                          const int ldc_)
{
  integer M = (integer)M_, K = (integer)K_, N = (integer)N_;
  integer lda = (integer)lda_, ldb = (integer)ldb_, ldc = (integer)ldc_;

  dgemm_(&trans_a, &trans_b, &M, &N, &K, &alpha, a_data, &lda, b_data, &ldb, &beta, c_data, &ldc);
}

void vpMatrix::blas_dgemv(char trans, const int M_, const int N_, double alpha, double *a_data, const int lda_,
                          double *x_data, const int incx_, double beta, double *y_data, const int incy_)
{
  integer M = (integer)M_, N = (integer)N_;
  integer lda = (integer)lda_, incx = (integer)incx_, incy = (integer)incy_;

  dgemv_(&trans, &M, &N, &alpha, a_data, &lda, x_data, &incx, &beta, y_data, &incy);
}
#  endif
#else
// Work arround to avoid warning LNK4221: This object file does not define any
// previously undefined public symbols
void dummy_vpMatrix_blas() {};
#endif

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
