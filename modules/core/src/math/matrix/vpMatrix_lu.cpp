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
 * Matrix LU decomposition.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#include <visp3/core/vpConfig.h>

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpColVector.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/core/vpMatrixException.h>

// Debug trace
#include <visp3/core/vpDebug.h>

#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

#define TINY 1.0e-20;


/*--------------------------------------------------------------------
  LU Decomposition  related functions
-------------------------------------------------------------------- */

/*!
  \brief Performed the LU decomposition

  Given a matrix A (n x n), this routine replaces it by the LU decomposition of
  a rowwise permutation of itself.  A is output, arranged as in equation
  (2.3.14) of the NRC ; perm (n) is an output vector that records the row
  permutation effected by the partial pivoting; d is output as 1 depending on
  whether the number of row interchanges was even or odd, respectively.

  \warning Destructive wrt. A

  \sa This routine is used in combination with LUDksb to solve linear equations
  or invert a matrix.

  This function is extracted from the NRC

 */

void
vpMatrix::LUDcmp(unsigned int *perm, int& d)
{
  unsigned int n = rowNum;

  unsigned int i,imax=0,j,k;
  double big,dum,sum_,temp;
  vpColVector vv(n);

  d=1;
  for (i=0;i<n;i++) {
    big=0.0;
    for (j=0;j<n;j++)
      if ((temp=fabs(rowPtrs[i][j])) > big) big=temp;
    //if (big == 0.0)
    if (std::fabs(big) <= std::numeric_limits<double>::epsilon())
    {
      //vpERROR_TRACE("Singular vpMatrix in  LUDcmp") ;
      throw(vpMatrixException(vpMatrixException::matrixError,
                              "Singular vpMatrix in  LUDcmp")) ;
    }
    vv[i]=1.0/big;
  }
  for (j=0;j<n;j++) {
    for (i=0;i<j;i++) {
      sum_=rowPtrs[i][j];
      for (k=0;k<i;k++) sum_ -= rowPtrs[i][k]*rowPtrs[k][j];
      rowPtrs[i][j]=sum_;
    }
    big=0.0;
    for (i=j;i<n;i++) {
      sum_=rowPtrs[i][j];
      for (k=0;k<j;k++)
        sum_ -= rowPtrs[i][k]*rowPtrs[k][j];
      rowPtrs[i][j]=sum_;
      if ( (dum=vv[i]*fabs(sum_)) >= big) {
        big=dum;
        imax=i;
      }
    }
    if (j != imax) {
      for (k=0;k<n;k++) {
        dum=rowPtrs[imax][k];
        rowPtrs[imax][k]=rowPtrs[j][k];
        rowPtrs[j][k]=dum;
      }
      d *= -1;
      vv[imax]=vv[j];
    }
    perm[j]=imax;
    //if (rowPtrs[j][j] == 0.0)
    if (std::fabs(rowPtrs[j][j]) <= std::numeric_limits<double>::epsilon())
      rowPtrs[j][j]=TINY;
    if (j != n) {
      dum=1.0/(rowPtrs[j][j]);
      for (i=j+1;i<n;i++) rowPtrs[i][j] *= dum;
    }
  }
}

#undef TINY

/*!
  \brief Solve linear system AX = B using LU decomposition

  Solves the set of n linear equations AX = B. Here A (n x n) is input, not
  as the matrix A but rather as its LU decomposition, determined by the routine
  ludcmp. perm (n) is input as the permutation vector returned by
  ludcmp. b (n) is input as the right-hand side vector B, and returns with
  the solution vector X. A and perm are not modified by this routine and can
  be left in place for successive calls with different right-hand sides b. This
  routine takes into account the possibility that b will begin with many zero
  elements, so it is efficient for use in matrix inversion.

  \sa This function must be used with LUDcmp

  \sa LUDsolve and solveByLUD are more intuitive and direct to use

  This function is extracted from the NRC

*/
void vpMatrix::LUBksb(unsigned int *perm, vpColVector& b)
{
  unsigned int n = rowNum;

  unsigned int ii=0;
  double sum_;
  bool flag = false;
  unsigned int i;

  for (i=0;i<n;i++) {
    unsigned int ip=perm[i];
    sum_=b[ip];
    b[ip]=b[i];
    if (flag) {
      for (unsigned int j=ii;j<=i-1;j++) sum_ -= rowPtrs[i][j]*b[j];
	}
    //else if (sum_) {
    else if (std::fabs(sum_) > std::numeric_limits<double>::epsilon()) {
      ii=i;
      flag = true;
    }
    b[i]=sum_;
  }
  // for (int i=n-1;i>=0;i--) {
  //   sum_=b[i];
  //   for (int j=i+1;j<n;j++) sum_ -= rowPtrs[i][j]*b[j];
  //   b[i]=sum_/rowPtrs[i][i];
  // }
  i=n;
  do {
    i --;

    sum_=b[i];
    for (unsigned int j=i+1;j<n;j++) sum_ -= rowPtrs[i][j]*b[j];
    b[i]=sum_/rowPtrs[i][i];
  } while(i != 0);
}
#endif // doxygen should skip this

/*!
  Compute the inverse of a n-by-n matrix using the LU decomposition.

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
  A_1 = A.inverseByLU();
  std::cout << "Inverse by LU: \n" << A_1 << std::endl;

  std::cout << "A*A^-1: \n" << A * A_1 << std::endl;
}
  \endcode

  \sa pseudoInverse()
*/
vpMatrix
vpMatrix::inverseByLU() const
{
  unsigned int i,j;

  if ( rowNum != colNum)
  {
    vpERROR_TRACE("\n\t\tCannot invert a non-square vpMatrix") ;
    throw(vpMatrixException(vpMatrixException::matrixError,
			    "Cannot invert a non-square vpMatrix")) ;
  }

  vpMatrix B(rowNum, rowNum), X(rowNum, rowNum);
  vpMatrix V(rowNum, rowNum);
  vpColVector W(rowNum);

  for (i=0; i<rowNum; i++) {
    for (j=0; j<rowNum; j++) {
      B[i][j] = (i == j) ? 1 : 0;
    }
  }

  vpMatrix A(rowNum, rowNum);
  A = *this;

  unsigned int *perm = new unsigned int[rowNum];

  try {
    int p;
    A.LUDcmp(perm, p);
  }
  catch(vpException &e) {
    delete [] perm;
    throw(e);
  }

  vpColVector c_tmp(rowNum)  ;
  for (j=1; j<=rowNum; j++)
  {
    c_tmp =0 ;  c_tmp[j-1] = 1 ;
    A.LUBksb(perm, c_tmp);
    for (unsigned int k=0 ; k < c_tmp.getRows() ; k++)
      B[k][j-1] = c_tmp[k] ;
  }
  delete [] perm;
  return B;
}

