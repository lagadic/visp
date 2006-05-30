/****************************************************************************
 *
 * $Id: vpMatrix_lu.cpp,v 1.3 2006-05-30 08:40:43 fspindle Exp $
 *
 * Copyright (C) 1998-2006 Inria. All rights reserved.
 *
 * This software was developed at:
 * IRISA/INRIA Rennes
 * Projet Lagadic
 * Campus Universitaire de Beaulieu
 * 35042 Rennes Cedex
 * http://www.irisa.fr/lagadic
 *
 * This file is part of the ViSP toolkit
 *
 * This file may be distributed under the terms of the Q Public License
 * as defined by Trolltech AS of Norway and appearing in the file
 * LICENSE included in the packaging of this file.
 *
 * Licensees holding valid ViSP Professional Edition licenses may
 * use this file in accordance with the ViSP Commercial License
 * Agreement provided with the Software.
 *
 * This file is provided AS IS with NO WARRANTY OF ANY KIND, INCLUDING THE
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE.
 *
 * Contact visp@irisa.fr if any conditions of this licensing are
 * not clear to you.
 *
 * Description:
 * Matrix LU decomposition.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpColVector.h>

// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>

#include <visp/vpConfig.h>
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
vpMatrix::LUDcmp(int *perm, int& d)
{
  int n = rowNum;

  int i,imax=0,j,k;
  double big,dum,sum,temp;
  vpColVector vv(n);

  d=1;
  for (i=0;i<n;i++) {
    big=0.0;
    for (j=0;j<n;j++)
      if ((temp=fabs(rowPtrs[i][j])) > big) big=temp;
    if (big == 0.0)
    {
      ERROR_TRACE("Singular vpMatrix in  LUDcmp") ;
        throw(vpMatrixException(vpMatrixException::matrixError,
			    "\n\t\tSingular vpMatrix in  LUDcmp")) ;
    }
    vv[i]=1.0/big;
  }
  for (j=0;j<n;j++) {
    for (i=0;i<j;i++) {
      sum=rowPtrs[i][j];
      for (k=0;k<i;k++) sum -= rowPtrs[i][k]*rowPtrs[k][j];
      rowPtrs[i][j]=sum;
    }
    big=0.0;
    for (i=j;i<n;i++) {
      sum=rowPtrs[i][j];
      for (k=0;k<j;k++)
	sum -= rowPtrs[i][k]*rowPtrs[k][j];
      rowPtrs[i][j]=sum;
      if ( (dum=vv[i]*fabs(sum)) >= big) {
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
    if (rowPtrs[j][j] == 0.0) rowPtrs[j][j]=TINY;
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
void vpMatrix::LUBksb(int *perm, vpColVector& b)
{
  int n = rowNum;

  int i,ii=-1,ip,j;
  double sum;

  for (i=0;i<n;i++) {
    ip=perm[i];
    sum=b[ip];
    b[ip]=b[i];
    if (ii != -1)
      for (j=ii;j<=i-1;j++) sum -= rowPtrs[i][j]*b[j];
    else if (sum) ii=i;
    b[i]=sum;
  }
  for (i=n-1;i>=0;i--) {
    sum=b[i];
    for (j=i+1;j<n;j++) sum -= rowPtrs[i][j]*b[j];
    b[i]=sum/rowPtrs[i][i];
  }
}

/*!
  \brief inverse matrix A using the LU  (C = A^-1)

  \sa InverseBySVD()
*/
vpMatrix
vpMatrix::inverseByLU() const
{
  int i,j;

  if ( rowNum != colNum)
  {
    ERROR_TRACE("\n\t\tCannot invert a non-square vpMatrix") ;
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

  int *perm = new int[rowNum];
  int p;

  A.LUDcmp(perm, p);

  vpColVector c_tmp(rowNum)  ;
  for (j=1; j<=rowNum; j++)
  {
    c_tmp =0 ;  c_tmp[j-1] = 1 ;
    A.LUBksb(perm, c_tmp);
    for (int k=0 ; k < c_tmp.getRows() ; k++)
      B[k][j-1] = c_tmp[k] ;
  }
  delete [] perm;
  return B;
}

#endif // doxygen should skip this
