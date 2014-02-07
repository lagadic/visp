/****************************************************************************
 *
 * $Id: vpMatrix_lu.cpp 3530 2012-01-03 10:52:12Z fspindle $
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2014 by INRIA. All rights reserved.
 * 
 * This software is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * ("GPL") version 2 as published by the Free Software Foundation.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact INRIA about acquiring a ViSP Professional 
 * Edition License.
 *
 * See http://www.irisa.fr/lagadic/visp/visp.html for more information.
 * 
 * This software was developed at:
 * INRIA Rennes - Bretagne Atlantique
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * France
 * http://www.irisa.fr/lagadic
 *
 * If you have questions regarding the use of this file, please contact
 * INRIA at visp@inria.fr
 * 
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 *
 * Description:
 * Matrix Cholesky decomposition.
 *
 * Authors:
 * Filip Novotny
 *
 *****************************************************************************/

#include <visp/vpConfig.h>

#include <visp/vpMatrix.h>
#include <visp/vpMath.h>
#include <visp/vpColVector.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>

#ifdef VISP_HAVE_LAPACK
extern "C" void dpotrf_ (char *uplo, int *n, double *a, int *lda, int *info);
extern "C" int dpotri_(char *uplo, int *n, double *a, int *lda, int *info);

#endif

#ifdef VISP_HAVE_LAPACK
vpMatrix vpMatrix::inverseByCholeskyLapack() const{
  int rowNum_ = (int)this->getRows();
  int lda = (int)rowNum_; //lda is the number of rows because we don't use a submatrix
  int info;

  vpMatrix A = *this;
  dpotrf_((char*)"L",&rowNum_,A.data,&lda,&info);

  if(info!=0)
    std::cout << "cholesky:dpotrf_:error" << std::endl;

  dpotri_((char*)"L",&rowNum_,A.data,&lda,&info);
  if(info!=0){
    std::cout << "cholesky:dpotri_:error" << std::endl;
    throw vpMatrixException::badValue;
  }

  for(unsigned int i=0;i<A.getRows();i++)
    for(unsigned int j=0;j<A.getCols();j++)
      if(i>j) A[i][j] = A[j][i];

  return A;
}
#endif

/*!
  Compute the inverse of a n-by-n matrix using the Cholesky decomposition.
  The matrix must be real and symmetric.
  Only available if lapack is installed.

  \return The inverse matrix.

  Here an example:
  \code
#include <visp/vpMatrix.h>

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

#if defined(VISP_HAVE_LAPACK)
vpMatrix
vpMatrix::inverseByCholesky() const
{

  if ( rowNum != colNum)
  {
    vpERROR_TRACE("\n\t\tCannot invert a non-square vpMatrix") ;
    throw(vpMatrixException(vpMatrixException::matrixError,
                            "Cannot invert a non-square vpMatrix")) ;
  }
#ifdef VISP_HAVE_LAPACK
  return inverseByCholeskyLapack();
#endif
}

#endif
