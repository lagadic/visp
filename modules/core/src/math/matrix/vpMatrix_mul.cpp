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
 * BLAS subroutines.
 */

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpMatrix.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

// Since GSL doesn't provide Fortran interface for Lapack we should use
// gsl_blas_dgemm() and gsl_blas_dgemv() instead of dgemm() and dgemv().
// As a side effect, it means that we have to allocate and copy the matrix
// or vector content in a gsl_matrix or gsl_vector. This is not acceptable here.
// that's why we prefer use naive code when VISP_HAVE_GSL is defined.
#if defined(VISP_HAVE_LAPACK)
BEGIN_VISP_NAMESPACE
#ifdef VISP_HAVE_MKL
#include <mkl.h>
typedef MKL_INT integer;

void vpMatrix::blas_dgemm(char trans_a, char trans_b, unsigned int M_, unsigned int N_, unsigned int K_, double alpha,
                          double *a_data, unsigned int lda_, double *b_data, unsigned int ldb_, double beta,
                          double *c_data, unsigned int ldc_)
{
  MKL_INT M = static_cast<MKL_INT>(M_);
  MKL_INT N = static_cast<MKL_INT>(N_);
  MKL_INT K = static_cast<MKL_INT>(K_);
  MKL_INT lda = static_cast<MKL_INT>(lda_);
  MKL_INT ldb = static_cast<MKL_INT>(ldb_);
  MKL_INT ldc = static_cast<MKL_INT>(ldc_);

  dgemm(&trans_a, &trans_b, &M, &N, &K, &alpha, a_data, &lda, b_data, &ldb, &beta, c_data, &ldc);
}

void vpMatrix::blas_dgemv(char trans, unsigned int M_, unsigned int N_, double alpha, double *a_data, unsigned int lda_,
                          double *x_data, int incx_, double beta, double *y_data, int incy_)
{
  MKL_INT M = static_cast<MKL_INT>(M_);
  MKL_INT N = static_cast<MKL_INT>(N_);
  MKL_INT lda = static_cast<MKL_INT>(lda_);
  MKL_INT incx = static_cast<MKL_INT>(incx_);
  MKL_INT incy = static_cast<MKL_INT>(incy_);

  dgemv(&trans, &M, &N, &alpha, a_data, &lda, x_data, &incx, &beta, y_data, &incy);
}
#elif !defined(VISP_HAVE_GSL)
#ifdef VISP_HAVE_LAPACK_BUILT_IN
typedef long int integer;
#else
typedef int integer;
#endif

extern "C" void dgemm_(char *transa, char *transb, integer *M, integer *N, integer *K, double *alpha, double *a,
                       integer *lda, double *b, integer *ldb, double *beta, double *c, integer *ldc);

extern "C" void dgemv_(char *trans, integer *M, integer *N, double *alpha, double *a, integer *lda, double *x,
                       integer *incx, double *beta, double *y, integer *incy);

void vpMatrix::blas_dgemm(char trans_a, char trans_b, unsigned int M_, unsigned int N_, unsigned int K_, double alpha,
                          double *a_data, unsigned int lda_, double *b_data, unsigned int ldb_, double beta,
                          double *c_data, unsigned int ldc_)
{
  integer M = static_cast<integer>(M_);
  integer K = static_cast<integer>(K_);
  integer N = static_cast<integer>(N_);
  integer lda = static_cast<integer>(lda_);
  integer ldb = static_cast<integer>(ldb_);
  integer ldc = static_cast<integer>(ldc_);

  dgemm_(&trans_a, &trans_b, &M, &N, &K, &alpha, a_data, &lda, b_data, &ldb, &beta, c_data, &ldc);
}

void vpMatrix::blas_dgemv(char trans, unsigned int M_, unsigned int N_, double alpha, double *a_data, unsigned int lda_,
                          double *x_data, int incx_, double beta, double *y_data, int incy_)
{
  integer M = static_cast<integer>(M_);
  integer N = static_cast<integer>(N_);
  integer lda = static_cast<integer>(lda_);
  integer incx = static_cast<integer>(incx_);
  integer incy = static_cast<integer>(incy_);

  dgemv_(&trans, &M, &N, &alpha, a_data, &lda, x_data, &incx, &beta, y_data, &incy);
}
#endif
END_VISP_NAMESPACE
#else
// Work around to avoid warning LNK4221: This object file does not define any
// previously undefined public symbols
void dummy_vpMatrix_blas() { };
#endif

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
