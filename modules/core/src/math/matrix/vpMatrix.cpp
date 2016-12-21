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
 * Matrix manipulation.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/



/*!
\file vpMatrix.cpp
\brief Definition of the vpMatrix class
*/

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <sstream>
#include <algorithm>
#include <assert.h>
#include <fstream>
#include <string>
#include <cmath>    // std::fabs
#include <limits>   // numeric_limits

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_GSL
#include <gsl/gsl_linalg.h>
#endif

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpDebug.h>

//Prototypes of specific functions
vpMatrix subblock(const vpMatrix &, unsigned int, unsigned int);


/*!
  Construct a matrix as a sub-matrix of the input matrix \e M.
  \sa init(const vpMatrix &M, unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols)
*/
vpMatrix::vpMatrix(const vpMatrix &M,
                   unsigned int r, unsigned int c, 
                   unsigned int nrows, unsigned int ncols)
  : vpArray2D<double>()
{
  if (((r + nrows) > M.rowNum) || ((c + ncols) > M.colNum)) {
    throw(vpException(vpException::dimensionError,
                      "Cannot construct a sub matrix (%dx%d) starting at position (%d,%d) that is not contained in the original matrix (%dx%d)",
                      nrows, ncols, r, c, M.rowNum, M.colNum)) ;
  }

  init(M, r, c, nrows, ncols);
}

/*!
  Initialize the matrix from a part of an input matrix \e M.

  \param M : Input matrix used for initialization.
  \param r : row index in matrix M.
  \param c : column index in matrix M.
  \param nrows : Number of rows of the matrix that should be initialized.
  \param ncols : Number of columns of the matrix that should be initialized.

  The sub-matrix starting from M[r][c] element and ending on M[r+nrows-1][c+ncols-1] element
  is used to initialize the matrix.

  The following code shows how to use this function:
\code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix M(4,5);
  int val = 0;
  for(size_t i=0; i<M.getRows(); i++) {
    for(size_t j=0; j<M.getCols(); j++) {
      M[i][j] = val++;
    }
  }
  M.print (std::cout, 4, "M ");

  vpMatrix N;
  N.init(M, 0, 1, 2, 3);
  N.print (std::cout, 4, "N ");
}
\endcode
  It produces the following output:
  \code
M [4,5]=
   0  1  2  3  4
   5  6  7  8  9
  10 11 12 13 14
  15 16 17 18 19
N [2,3]=
  1 2 3
  6 7 8
  \endcode
 */
void
vpMatrix::init(const vpMatrix &M, unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols)
{
  unsigned int rnrows = r+nrows ;
  unsigned int cncols = c+ncols ;

  if (rnrows > M.getRows())
    throw(vpException(vpException::dimensionError,
                      "Bad row dimension (%d > %d) used to initialize vpMatrix", rnrows, M.getRows()));
  if (cncols > M.getCols())
    throw(vpException(vpException::dimensionError,
                      "Bad column dimension (%d > %d) used to initialize vpMatrix", cncols, M.getCols()));
  resize(nrows, ncols);

  if (this->rowPtrs == NULL) // Fix coverity scan: explicit null dereferenced
    return; // Noting to do
  for (unsigned int i=r ; i < rnrows; i++)
    for (unsigned int j=c ; j < cncols; j++)
      (*this)[i-r][j-c] = M[i][j] ;
}

/*!
  Set an n-by-n matrix to identity with ones on the diagonal and zeros
  else where.
*/
void
vpMatrix::eye(unsigned int n)
{
  try {
    eye(n, n);
  }
  catch(...) {
    throw ;
  }
}

/*!
  Set an m-by-n matrix to identity with ones on the diagonal and zeros
  else where.
*/
void
vpMatrix::eye(unsigned int m, unsigned int n)
{
  try {
    resize(m,n) ;
  }
  catch(...) {
    throw ;
  }

  eye();
}

/*!
  Set an m-by-n matrix to identity with ones on the diagonal and zeros
  else where.
*/
void
vpMatrix::eye()
{
  for (unsigned int i=0; i<rowNum; i++) {
    for (unsigned int j=0; j<colNum; j++) {
      if (i == j) (*this)[i][j] = 1.0;
      else        (*this)[i][j] = 0;
    }
  }
}

/*!
  Compute and return the transpose of the matrix.
*/
vpMatrix vpMatrix::t() const
{
  vpMatrix At ;

  try {
    At.resize(colNum, rowNum);
  }
  catch(...)
  {
    throw ;
  }

  for (unsigned int i=0;i<rowNum;i++) {
    double *coli = (*this)[i] ;
    for (unsigned int j=0;j<colNum;j++)
      At[j][i] = coli[j];
  }
  return At;
}


/*!
  Compute and return the transpose of the matrix.

  \sa t()
*/
vpMatrix vpMatrix::transpose()const
{
  vpMatrix At ;
  transpose(At);
  return At;
}

/*!
  Compute \e At the transpose of the matrix.
  \param At (output) : Resulting transpose matrix.
  \sa t()
*/
void vpMatrix::transpose(vpMatrix & At ) const
{
  try {
    At.resize(colNum,rowNum);
  }
  catch(...)
  {
    throw ;
  }

  size_t A_step = colNum;
  double ** AtRowPtrs = At.rowPtrs;

  for( unsigned int i = 0; i < colNum; i++ ) {
    double * row_ = AtRowPtrs[i];
    double * col = rowPtrs[0]+i;
    for( unsigned int j = 0; j < rowNum; j++, col+=A_step )
      *(row_++)=*col;
  }
}


/*!
  Computes the \f$AA^T\f$ operation \f$B = A*A^T\f$
  \return  \f$A*A^T\f$
  \sa AAt(vpMatrix &) const
*/
vpMatrix vpMatrix::AAt() const
{
  vpMatrix B;

  AAt(B);

  return B;
}

/*!
  Compute the AAt operation such as \f$B = A*A^T\f$.

  The result is placed in the parameter \e B and not returned.

  A new matrix won't be allocated for every use of the function. This
  results in a speed gain if used many times with the same result
  matrix size.

  \sa AAt()
*/
void vpMatrix::AAt(vpMatrix &B) const
{
  try {
    if ((B.rowNum != rowNum) || (B.colNum != rowNum)) B.resize(rowNum,rowNum);
  }
  catch(...)
  {
    throw ;
  }

  // compute A*A^T
  for(unsigned int i=0;i<rowNum;i++){
    for(unsigned int j=i;j<rowNum;j++){
      double *pi = rowPtrs[i];// row i
      double *pj = rowPtrs[j];// row j

      // sum (row i .* row j)
      double ssum=0;
      for(unsigned int k=0; k < colNum ;k++)
        ssum += *(pi++)* *(pj++);

      B[i][j]=ssum; //upper triangle
      if(i!=j)
        B[j][i]=ssum; //lower triangle
    }
  }
}

/*!
  Compute the AtA operation such as \f$B = A^T*A\f$.

  The result is placed in the parameter \e B and not returned.

  A new matrix won't be allocated for every use of the function. This
  results in a speed gain if used many times with the same result matrix
  size.

  \sa AtA()
*/
void vpMatrix::AtA(vpMatrix &B) const
{
  try {
    if ((B.rowNum != colNum) || (B.colNum != colNum)) B.resize(colNum,colNum);
  }
  catch(...)
  {
    throw ;
  }

  unsigned int i,j,k;
  double s;
  double *ptr;
  for (i=0;i<colNum;i++)
  {
    double *Bi = B[i] ;
    for (j=0;j<i;j++)
    {
      ptr=data;
      s = 0 ;
      for (k=0;k<rowNum;k++)
      {
        s +=(*(ptr+i)) * (*(ptr+j));
        ptr+=colNum;
      }
      *Bi++ = s ;
      B[j][i] = s;
    }
    ptr=data;
    s = 0 ;
    for (k=0;k<rowNum;k++)
    {
      s +=(*(ptr+i)) * (*(ptr+i));
      ptr+=colNum;
    }
    *Bi = s;
  }
}


/*!
  Compute the AtA operation such as \f$B = A^T*A\f$
  \return  \f$A^T*A\f$
  \sa AtA(vpMatrix &) const
*/
vpMatrix vpMatrix::AtA() const
{
  vpMatrix B;

  AtA(B);

  return B;
}

/*!
  Copy operator that allows to convert on of the following container that
  inherit from vpArray2D such as vpMatrix, vpRotationMatrix, vpHomogeneousMatrix,
  vpPoseVector, vpColVector, vpRowVector... into a vpMatrix.

  \param A : 2D array to be copied.

  The following example shows how to create a matrix from an homogeneous matrix:
  \code
  vpRotationMatrix R;
  vpMatrix M = R;
  \endcode

*/
vpMatrix &
vpMatrix::operator=(const vpArray2D<double> &A)
{
  try {
    resize(A.getRows(), A.getCols()) ;
  }
  catch(...) {
    throw ;
  }

  memcpy(data, A.data, dsize*sizeof(double));

  return *this;
}

//! Set all the element of the matrix A to \e x.
vpMatrix &
vpMatrix::operator=(double x)
{
  for (unsigned int i=0;i<rowNum;i++)
    for(unsigned int j=0;j<colNum;j++)
      rowPtrs[i][j] = x;

  return *this;
}


/*!
  Assigment from an array of double. This method has to be used carefully since
  the array allocated behind \e x pointer should have the same dimension than the matrix.
*/
vpMatrix &
vpMatrix::operator<<( double *x )
{
  for (unsigned int i=0; i<rowNum; i++) {
    for (unsigned int j=0; j<colNum; j++) {
      rowPtrs[i][j] = *x++;
    }
  }
  return *this;
}

/*!

  Create a diagonal matrix with the element of a vector.

  \param  A : Vector which element will be put in the diagonal.

  \sa createDiagonalMatrix()

\code
#include <iostream>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A;
  vpColVector v(3);

  v[0] = 1;
  v[1] = 2;
  v[2] = 3;

  A.diag(v);

  std::cout << "A:\n" << A << std::endl;
}
\endcode

  Matrix A is now equal to:
\code
1 0 0
0 2 0
0 0 3
\endcode
*/
void
vpMatrix::diag(const vpColVector &A)
{
  unsigned int rows = A.getRows() ;
  try {
    this->resize(rows,rows) ;
  }
  catch(...) {
    throw ;
  }
  (*this) = 0 ;
  for (unsigned int i=0 ; i< rows ; i++ )
    (* this)[i][i] = A[i] ;
}

/*!

  Set the matrix as a diagonal matrix where each element on the diagonal is set to \e val.
  Elements that are not on the diagonal are set to 0.

  \param val : Value to set.

  \sa eye()

\code
#include <iostream>

#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(3, 4);

  A.diag(2);

  std::cout << "A:\n" << A << std::endl;
}
\endcode

  Matrix A is now equal to:
\code
2 0 0 0
0 2 0 0
0 0 2 0
\endcode
*/
void
vpMatrix::diag(const double &val)
{
  (*this) = 0;
  unsigned int min_ = (rowNum < colNum) ? rowNum : colNum;
  for (unsigned int i=0 ; i< min_ ; i++ )
    (* this)[i][i] = val;
}


/*!

  Create a diagonal matrix with the element of a vector \f$ DA_{ii} = A_i \f$.

  \param  A : Vector which element will be put in the diagonal.

  \param  DA : Diagonal matrix DA[i][i] = A[i]

\sa diag()
*/

void
vpMatrix::createDiagonalMatrix(const vpColVector &A, vpMatrix &DA)
{
  unsigned int rows = A.getRows() ;
  try {
    DA.resize(rows,rows) ;
  }
  catch(...)
  {
    throw ;
  }
  DA =0 ;
  for (unsigned int i=0 ; i< rows ; i++ )
    DA[i][i] = A[i] ;
}

/*!
  Operator that allows to multiply a matrix by a translation vector.
  The matrix should be of dimension (3x3)
  */
vpTranslationVector
vpMatrix::operator*(const vpTranslationVector &tv) const
{
  vpTranslationVector t_out;

  if (rowNum != 3 || colNum != 3) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply a (%dx%d) matrix by a (%dx%d) translation vector",
                      rowNum, colNum, tv.getRows(), tv.getCols())) ;
  }

  for (unsigned int j=0;j<3;j++) t_out[j]=0 ;

  for (unsigned int j=0;j<3;j++) {
    double tj = tv[j] ; // optimization em 5/12/2006
    for (unsigned int i=0;i<3;i++) {
      t_out[i]+=rowPtrs[i][j] * tj;
    }
  }
  return t_out;
}

/*!
  Operation w = A * v (matrix A is unchanged, v and w are column vectors).
  \sa multMatrixVector() to avoid matrix allocation for each use.
*/
vpColVector
vpMatrix::operator*(const vpColVector &v) const
{
  vpColVector v_out;
  vpMatrix::multMatrixVector(*this, v, v_out);
  return v_out;
}

/*!
  Operation w = A * v (v and w are vectors).

  A new matrix won't be allocated for every use of the function
  (Speed gain if used many times with the same result matrix size).

  \sa operator*(const vpColVector &v) const
*/
void vpMatrix::multMatrixVector(const vpMatrix &A, const vpColVector &v, vpColVector &w)
{
  if (A.colNum != v.getRows()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply a (%dx%d) matrix by a (%d) column vector",
                      A.getRows(), A.getCols(), v.getRows())) ;
  }

  try {
    if (A.rowNum != w.rowNum) w.resize(A.rowNum);
  }
  catch(...) {
    throw ;
  }

  w = 0.0;
  for (unsigned int j=0;j<A.colNum;j++) {
    double vj = v[j] ; // optimization em 5/12/2006
    for (unsigned int i=0;i<A.rowNum;i++) {
      w[i]+=A.rowPtrs[i][j] * vj;
    }
  }
}

//---------------------------------
// Matrix operations.
//---------------------------------

/*!
  Operation C = A * B.

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \sa operator*()
*/
void vpMatrix::mult2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
  try {
    if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum)) C.resize(A.rowNum,B.colNum);
  }
  catch(...) {
    throw ;
  }

  if (A.colNum != B.rowNum) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply (%dx%d) matrix by (%dx%d) matrix",
                      A.getRows(), A.getCols(), B.getRows(), B.getCols())) ;
  }

  // 5/12/06 some "very" simple optimization to avoid indexation
  unsigned int BcolNum = B.colNum;
  unsigned int BrowNum = B.rowNum;
  unsigned int i,j,k;
  double **BrowPtrs = B.rowPtrs;
  for (i=0;i<A.rowNum;i++)
  {
    double *rowptri = A.rowPtrs[i];
    double *ci = C[i];
    for (j=0;j<BcolNum;j++)
    {
      double s = 0;
      for (k=0;k<BrowNum;k++) s += rowptri[k] * BrowPtrs[k][j];
      ci[j] = s;
    }
  }
}

/*!
  \warning This function is provided for compat with previous releases. You should
  rather use the functionalities provided in vpRotationMatrix class.

  Operation C = A * B.

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \exception vpException::dimensionError If matrices are not 3-by-3 dimension.

*/
void vpMatrix::mult2Matrices(const vpMatrix &A, const vpMatrix &B, vpRotationMatrix &C)
{
  if (A.colNum != 3 || A.rowNum != 3 || B.colNum != 3 || B.rowNum != 3) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply (%dx%d) matrix by (%dx%d) matrix as a rotation matrix",
                      A.getRows(), A.getCols(), B.getRows(), B.getCols())) ;
  }

  // 5/12/06 some "very" simple optimization to avoid indexation
  unsigned int BcolNum = B.colNum;
  unsigned int BrowNum = B.rowNum;
  unsigned int i,j,k;
  double **BrowPtrs = B.rowPtrs;
  for (i=0;i<A.rowNum;i++)
  {
    double *rowptri = A.rowPtrs[i];
    double *ci = C[i];
    for (j=0;j<BcolNum;j++)
    {
      double s = 0;
      for (k=0;k<BrowNum;k++) s += rowptri[k] * BrowPtrs[k][j];
      ci[j] = s;
    }
  }
}

/*!
  \warning This function is provided for compat with previous releases. You should
  rather use the functionalities provided in vpHomogeneousMatrix class.

  Operation C = A * B.

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \exception vpException::dimensionError If matrices are not 4-by-4 dimension.

*/
void vpMatrix::mult2Matrices(const vpMatrix &A, const vpMatrix &B, vpHomogeneousMatrix &C)
{
  if (A.colNum != 4 || A.rowNum != 4 || B.colNum != 4 || B.rowNum != 4) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply (%dx%d) matrix by (%dx%d) matrix as a rotation matrix",
                      A.getRows(), A.getCols(), B.getRows(), B.getCols())) ;
  }

  // 5/12/06 some "very" simple optimization to avoid indexation
  unsigned int BcolNum = B.colNum;
  unsigned int BrowNum = B.rowNum;
  unsigned int i,j,k;
  double **BrowPtrs = B.rowPtrs;
  for (i=0;i<A.rowNum;i++)
  {
    double *rowptri = A.rowPtrs[i];
    double *ci = C[i];
    for (j=0;j<BcolNum;j++)
    {
      double s = 0;
      for (k=0;k<BrowNum;k++) s += rowptri[k] * BrowPtrs[k][j];
      ci[j] = s;
    }
  }
}

/*!
  \warning This function is provided for compat with previous releases. You should
  rather use multMatrixVector() that is more explicit.

  Operation C = A * B.

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \sa multMatrixVector()
*/
void vpMatrix::mult2Matrices(const vpMatrix &A, const vpColVector &B, vpColVector &C)
{
  vpMatrix::multMatrixVector(A, B, C);
}

/*!
  Operation C = A * B (A is unchanged).
  \sa mult2Matrices() to avoid matrix allocation for each use.
*/
vpMatrix vpMatrix::operator*(const vpMatrix &B) const
{
  vpMatrix C;

  vpMatrix::mult2Matrices(*this,B,C);

  return C;
}

/*!
  Operator that allow to multiply a matrix by a rotation matrix.
  The matrix should be of dimension m-by-3.
*/
vpMatrix vpMatrix::operator*(const vpRotationMatrix &R) const
{
  if (colNum != R.getRows()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply (%dx%d) matrix by (3x3) rotation matrix",
                      rowNum, colNum)) ;
  }
  vpMatrix C(rowNum, 3);

  unsigned int RcolNum = R.getCols();
  unsigned int RrowNum = R.getRows();
  for (unsigned int i=0;i<rowNum;i++)
  {
    double *rowptri = rowPtrs[i];
    double *ci = C[i];
    for (unsigned int j=0;j<RcolNum;j++)
    {
      double s = 0;
      for (unsigned int k=0;k<RrowNum;k++) s += rowptri[k] * R[k][j];
      ci[j] = s;
    }
  }

  return C;
}
/*!
  Operator that allow to multiply a matrix by a velocity twist matrix.
  The matrix should be of dimension m-by-6.
*/
vpMatrix vpMatrix::operator*(const vpVelocityTwistMatrix &V) const
{
  if (colNum != V.getRows()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply (%dx%d) matrix by (3x3) velocity twist matrix",
                      rowNum, colNum)) ;
  }
  vpMatrix M(rowNum, 6);

  unsigned int VcolNum = V.getCols();
  unsigned int VrowNum = V.getRows();
  for (unsigned int i=0;i<rowNum;i++)
  {
    double *rowptri = rowPtrs[i];
    double *ci = M[i];
    for (unsigned int j=0;j<VcolNum;j++)
    {
      double s = 0;
      for (unsigned int k=0;k<VrowNum;k++) s += rowptri[k] * V[k][j];
      ci[j] = s;
    }
  }

  return M;
}
/*!
  Operator that allow to multiply a matrix by a force/torque twist matrix.
  The matrix should be of dimension m-by-6.
*/
vpMatrix vpMatrix::operator*(const vpForceTwistMatrix &V) const
{
  if (colNum != V.getRows()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply (%dx%d) matrix by (3x3) force/torque twist matrix",
                      rowNum, colNum)) ;
  }
  vpMatrix M(rowNum, 6);

  unsigned int VcolNum = V.getCols();
  unsigned int VrowNum = V.getRows();
  for (unsigned int i=0;i<rowNum;i++)
  {
    double *rowptri = rowPtrs[i];
    double *ci = M[i];
    for (unsigned int j=0;j<VcolNum;j++)
    {
      double s = 0;
      for (unsigned int k=0;k<VrowNum;k++) s += rowptri[k] * V[k][j];
      ci[j] = s;
    }
  }

  return M;
}

/*!
Operation C = A*wA + B*wB 

The result is placed in the third parameter C and not returned.
A new matrix won't be allocated for every use of the function 
(Speed gain if used many times with the same result matrix size).

\sa operator+()
*/

void vpMatrix::add2WeightedMatrices(const vpMatrix &A, const double &wA, const vpMatrix &B,const double &wB, vpMatrix &C){
  try 
  {
    if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum)) C.resize(A.rowNum,B.colNum);
  }
  catch(...) {
    throw ;
  }

  if ((A.colNum != B.getCols())||(A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError,
                      "Cannot add (%dx%d) matrix with (%dx%d) matrix",
                      A.getRows(), A.getCols(), B.getRows(), B.getCols())) ;
  }

  double ** ArowPtrs=A.rowPtrs;
  double ** BrowPtrs=B.rowPtrs;
  double ** CrowPtrs=C.rowPtrs;

  for (unsigned int i=0;i<A.rowNum;i++)
    for(unsigned int j=0;j<A.colNum;j++)	 
      CrowPtrs[i][j] = wB*BrowPtrs[i][j]+wA*ArowPtrs[i][j];
}

/*!
  Operation C = A + B.

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \sa operator+()
*/
void vpMatrix::add2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{  
  try  {
    if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum)) C.resize(A.rowNum,B.colNum);
  }
  catch(...) {
    throw ;
  }

  if ((A.colNum != B.getCols())||(A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError,
                      "Cannot add (%dx%d) matrix with (%dx%d) matrix",
                      A.getRows(), A.getCols(), B.getRows(), B.getCols())) ;
  }

  double ** ArowPtrs=A.rowPtrs;
  double ** BrowPtrs=B.rowPtrs;
  double ** CrowPtrs=C.rowPtrs;

  for (unsigned int i=0;i<A.rowNum;i++) {
    for(unsigned int j=0;j<A.colNum;j++) {
      CrowPtrs[i][j] = BrowPtrs[i][j]+ArowPtrs[i][j];
    }
  }
}

/*!
  \warning This function is provided for compat with previous releases. You should
  rather use the functionalities provided in vpColVector class.

  Operation C = A + B.

  The result is placed in the third parameter C and not returned.
  A new vector won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \sa vpColVector::operator+()
*/
void vpMatrix::add2Matrices(const vpColVector &A, const vpColVector &B, vpColVector &C)
{
  try  {
    if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum)) C.resize(A.rowNum);
  }
  catch(...) {
    throw ;
  }

  if ((A.colNum != B.getCols())||(A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError,
                      "Cannot add (%dx%d) matrix with (%dx%d) matrix",
                      A.getRows(), A.getCols(), B.getRows(), B.getCols())) ;
  }

  double ** ArowPtrs=A.rowPtrs;
  double ** BrowPtrs=B.rowPtrs;
  double ** CrowPtrs=C.rowPtrs;

  for (unsigned int i=0;i<A.rowNum;i++) {
    for(unsigned int j=0;j<A.colNum;j++) {
      CrowPtrs[i][j] = BrowPtrs[i][j]+ArowPtrs[i][j];
    }
  }
}

/*!
  Operation C = A + B (A is unchanged).
  \sa add2Matrices() to avoid matrix allocation for each use.
*/
vpMatrix vpMatrix::operator+(const vpMatrix &B) const
{
  vpMatrix C;
  vpMatrix::add2Matrices(*this,B,C);
  return C;
}


/*!
  \warning This function is provided for compat with previous releases. You should
  rather use the functionalities provided in vpColVector class.

  Operation C = A - B on column vectors.

  The result is placed in the third parameter C and not returned.
  A new vector won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \exception vpException::dimensionError If A and B vectors have not the same size.

  \sa vpColVector::operator-()
*/
void vpMatrix::sub2Matrices(const vpColVector &A, const vpColVector &B, vpColVector &C)
{
  try {
    if ((A.rowNum != C.rowNum) || (A.colNum != C.colNum)) C.resize(A.rowNum);
  }
  catch(...) {
    throw ;
  }

  if ( (A.colNum != B.getCols())||(A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError,
                      "Cannot substract (%dx%d) matrix to (%dx%d) matrix",
                      A.getRows(), A.getCols(), B.getRows(), B.getCols())) ;
  }

  double ** ArowPtrs=A.rowPtrs;
  double ** BrowPtrs=B.rowPtrs;
  double ** CrowPtrs=C.rowPtrs;

  for (unsigned int i=0;i<A.rowNum;i++) {
    for(unsigned int j=0;j<A.colNum;j++) {
      CrowPtrs[i][j] = ArowPtrs[i][j]-BrowPtrs[i][j];
    }
  }
}

/*!
  Operation C = A - B.

  The result is placed in the third parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (speed gain if used many times with the same result matrix size).

  \exception vpException::dimensionError If A and B matrices have not the same size.

  \sa operator-()
*/
void vpMatrix::sub2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
  try {
    if ((A.rowNum != C.rowNum) || (A.colNum != C.colNum)) C.resize(A.rowNum,A.colNum);
  }
  catch(...) {
    throw ;
  }

  if ( (A.colNum != B.getCols())||(A.rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError,
                      "Cannot substract (%dx%d) matrix to (%dx%d) matrix",
                      A.getRows(), A.getCols(), B.getRows(), B.getCols())) ;
  }

  double ** ArowPtrs=A.rowPtrs;
  double ** BrowPtrs=B.rowPtrs;
  double ** CrowPtrs=C.rowPtrs;

  for (unsigned int i=0;i<A.rowNum;i++) {
    for(unsigned int j=0;j<A.colNum;j++) {
      CrowPtrs[i][j] = ArowPtrs[i][j]-BrowPtrs[i][j];
    }
  }
}

/*!
  Operation C = A - B (A is unchanged).
  \sa sub2Matrices() to avoid matrix allocation for each use.
*/
vpMatrix vpMatrix::operator-(const vpMatrix &B) const
{
  vpMatrix C;
  vpMatrix::sub2Matrices(*this,B,C);
  return C;
}

//! Operation A = A + B

vpMatrix &vpMatrix::operator+=(const vpMatrix &B)
{
  if ( (colNum != B.getCols())||(rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError,
                      "Cannot add (%dx%d) matrix to (%dx%d) matrix",
                      rowNum, colNum, B.getRows(), B.getCols())) ;
  }

  double ** BrowPtrs=B.rowPtrs;

  for (unsigned int i=0;i<rowNum;i++)
    for(unsigned int j=0;j<colNum;j++)	
      rowPtrs[i][j] += BrowPtrs[i][j];

  return *this;
}

//! Operation A = A - B
vpMatrix & vpMatrix::operator-=(const vpMatrix &B)
{
  if ( (colNum != B.getCols())||(rowNum != B.getRows())) {
    throw(vpException(vpException::dimensionError,
                      "Cannot substract (%dx%d) matrix to (%dx%d) matrix",
                      rowNum, colNum, B.getRows(), B.getCols())) ;
  }

  double ** BrowPtrs=B.rowPtrs;
  for (unsigned int i=0;i<rowNum;i++)
    for(unsigned int j=0;j<colNum;j++)
      rowPtrs[i][j] -= BrowPtrs[i][j];

  return *this;
}

/*!
  Operation C = -A.

  The result is placed in the second parameter C and not returned.
  A new matrix won't be allocated for every use of the function
  (Speed gain if used many times with the same result matrix size).

  \sa operator-(void)
*/
void vpMatrix::negateMatrix(const vpMatrix &A, vpMatrix &C)
{
  try {
    if ((A.rowNum != C.rowNum) || (A.colNum != C.colNum)) C.resize(A.rowNum,A.colNum);
  }
  catch(...) {
    throw ;
  }

  double ** ArowPtrs=A.rowPtrs;
  double ** CrowPtrs=C.rowPtrs;

  // 	t0=vpTime::measureTimeMicros();
  for (unsigned int i=0;i<A.rowNum;i++)
    for(unsigned int j=0;j<A.colNum;j++)
      CrowPtrs[i][j]= -ArowPtrs[i][j];
}

/*!
  Operation C = -A (A is unchanged).
  \sa negateMatrix() to avoid matrix allocation for each use.
*/
vpMatrix vpMatrix::operator-() const //negate
{
  vpMatrix C;
  vpMatrix::negateMatrix(*this,C);
  return C;
}


double
vpMatrix::sum() const
{
  double s=0.0;
  for (unsigned int i=0;i<rowNum;i++)
  {
    for(unsigned int j=0;j<colNum;j++)
    {
      s += rowPtrs[i][j];
    }
  }

  return s;
}


//---------------------------------
// Matrix/vector operations.
//---------------------------------




//---------------------------------
// Matrix/real operations.
//---------------------------------

/*!
  \relates vpMatrix
  Allow to multiply a scalar by a matrix.
*/
vpMatrix operator*(const double &x,const vpMatrix &B)
{
  vpMatrix C(B.getRows(), B.getCols());

  unsigned int Brow = B.getRows() ;
  unsigned int Bcol = B.getCols() ;

  for (unsigned int i=0;i<Brow;i++)
    for(unsigned int j=0;j<Bcol;j++)
      C[i][j]= B[i][j]*x;

  return C ;
}

/*!
   Operator that allows to multiply all the elements of a matrix
   by a scalar.
 */
vpMatrix vpMatrix::operator*(double x) const
{
  vpMatrix M(rowNum,colNum);

  for (unsigned int i=0;i<rowNum;i++)
    for(unsigned int j=0;j<colNum;j++)
      M[i][j]= rowPtrs[i][j]*x;

  return M;
}

//! Cij = Aij / x (A is unchanged)
vpMatrix  vpMatrix::operator/(double x) const
{
  vpMatrix C;

  try {
    C.resize(rowNum,colNum);
  }
  catch(...) {
    throw ;
  }

  //if (x == 0) {
  if (std::fabs(x) <= std::numeric_limits<double>::epsilon()) {
    throw vpException(vpException::divideByZeroError, "Divide matrix by zero scalar");
  }

  double  xinv = 1/x ;

  for (unsigned int i=0;i<rowNum;i++)
    for(unsigned int j=0;j<colNum;j++)
      C[i][j]=rowPtrs[i][j]*xinv;

  return C;
}


//! Add x to all the element of the matrix : Aij = Aij + x
vpMatrix & vpMatrix::operator+=(double x)
{
  for (unsigned int i=0;i<rowNum;i++)
    for(unsigned int j=0;j<colNum;j++)
      rowPtrs[i][j]+=x;

  return *this;
}


//! Substract x to all the element of the matrix : Aij = Aij - x
vpMatrix & vpMatrix::operator-=(double x)
{
  for (unsigned int i=0;i<rowNum;i++)
    for(unsigned int j=0;j<colNum;j++)
      rowPtrs[i][j]-=x;

  return *this;
}

/*!
   Operator that allows to multiply all the elements of a matrix
   by a scalar.
 */
vpMatrix & vpMatrix::operator*=(double x)
{
  for (unsigned int i=0;i<rowNum;i++)
    for(unsigned int j=0;j<colNum;j++)
      rowPtrs[i][j]*=x;

  return *this;
}

//! Divide  all the element of the matrix by x : Aij = Aij / x
vpMatrix & vpMatrix::operator/=(double x)
{
  //if (x == 0)
  if (std::fabs(x) <= std::numeric_limits<double>::epsilon()) 
    throw vpException(vpException::divideByZeroError, "Divide matrix by zero scalar");

  double xinv = 1/x ;

  for (unsigned int i=0;i<rowNum;i++)
    for(unsigned int j=0;j<colNum;j++)
      rowPtrs[i][j]*=xinv;

  return *this;
}

//----------------------------------------------------------------
// Matrix Operation
//----------------------------------------------------------------







/*! 
  Stacks columns of a matrix in a vector.
  \param out : a vpColVector.
*/
void vpMatrix::stackColumns(vpColVector  &out ){

  try {
    if ((out.rowNum != colNum*rowNum) || (out.colNum != 1)) out.resize(rowNum);
  }
  catch(...) {
    throw ;
  }

  double *optr=out.data;
  for(unsigned int j =0;j<colNum ; j++){
    for(unsigned int i =0;i<rowNum ; i++){
      *(optr++)=rowPtrs[i][j];
    }
  }
}

/*!
  Stacks columns of a matrix in a vector.
  \return a vpColVector. 
*/
vpColVector vpMatrix::stackColumns()
{
  vpColVector out(colNum*rowNum);
  stackColumns(out);
  return out;
}

/*! 
  Stacks rows of a matrix in a vector
  \param out : a vpRowVector.
*/
void vpMatrix::stackRows(vpRowVector &out)
{
  try {
    if ((out.getRows() != 1) || (out.getCols() != colNum*rowNum)) out.resize(rowNum);
  }
  catch(...) {
    throw ;
  }

  double *mdata=data;
  double *optr=out.data;
  for(unsigned int i =0;i<dsize ; i++){
    *(optr++)=*(mdata++);
  }
}
/*! 
  Stacks rows of a matrix in a vector.
 \return a vpRowVector.
*/
vpRowVector vpMatrix::stackRows()
{
  vpRowVector out(colNum*rowNum);
  stackRows(out );
  return out; 
}

/*!
  Compute Kronecker product matrix.
  \param m1 : vpMatrix;
  \param m2 : vpMatrix;
  \param out : The kronecker product : \f$ m1 \otimes m2 \f$
*/
void vpMatrix::kron(const vpMatrix &m1, const vpMatrix &m2 , vpMatrix &out)
{
  unsigned int r1= m1.getRows();
  unsigned int c1= m1.getCols();
  unsigned int r2= m2.getRows();
  unsigned int c2= m2.getCols();

  if (r1*r2 !=out.rowNum || c1*c2!= out.colNum )
  {
    vpERROR_TRACE("Kronecker prodect bad dimension of output vpMatrix") ;
    throw(vpException(vpException::dimensionError,
                      "In Kronecker product bad dimension of output matrix"));
  }

  for(unsigned int r =0;r<r1 ; r++){
    for(unsigned int c =0;c<c1 ; c++){
      double alpha = m1[r][c];
      double *m2ptr = m2[0];
      unsigned int roffset= r*r2;
      unsigned int coffset= c*c2;
      for(unsigned int rr =0;rr<r2 ; rr++){
        for(unsigned int cc =0;cc<c2 ;cc++){
          out[roffset+rr][coffset+cc]= alpha* *(m2ptr++);
        }
      }
    }
  }

}

/*!
  Compute Kronecker product matrix.
  \param m : vpMatrix.
  \param out : If m1.kron(m2) out contains the kronecker product's result : \f$ m1 \otimes m2 \f$.
*/
void vpMatrix::kron(const vpMatrix  &m , vpMatrix  &out) const
{
  kron(*this,m,out);
}

/*!
  Compute Kronecker product matrix.
  \param m1 : vpMatrix;
  \param m2 : vpMatrix;
  \return The kronecker product : \f$ m1 \otimes m2 \f$
*/
vpMatrix vpMatrix::kron(const vpMatrix &m1, const vpMatrix &m2)
{
  unsigned int r1= m1.getRows();
  unsigned int c1= m1.getCols();
  unsigned int r2= m2.getRows();
  unsigned int c2= m2.getCols();

  vpMatrix out(r1*r2,c1*c2);

  for(unsigned int r =0;r<r1 ; r++){
    for(unsigned int c =0;c<c1 ; c++){
      double alpha = m1[r][c];
      double *m2ptr = m2[0];
      unsigned int roffset= r*r2;
      unsigned int coffset= c*c2;
      for(unsigned int rr =0;rr<r2 ; rr++){
        for(unsigned int cc =0;cc<c2 ;cc++){
          out[roffset+rr ][coffset+cc]= alpha* *(m2ptr++);
        }
      }
    }
  }
  return out;
}


/*!
  Compute Kronecker product matrix.
  \param m : vpMatrix;
  \return m1.kron(m2) The kronecker product : \f$ m1 \otimes m2 \f$
*/
vpMatrix vpMatrix::kron(const vpMatrix  &m) const
{
  return kron(*this,m);
}

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
void
vpMatrix::solveBySVD(const vpColVector &b, vpColVector &x) const
{
  x = pseudoInverse(1e-6)*b ;
}


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

  Singular value decomposition (SVD).

  \f[ M = U \Sigma V^{\top} \f]

  \warning Destructive method wrt. to the matrix \f$ M \f$ to
  decompose. You should make a COPY of that matrix if needed not to
  CHANGE.

  \param w : Vector of singular values. \f$ \Sigma = diag(w) \f$.

  \param v : Matrix \f$ V \f$.

  \return Matrix \f$ U \f$.

  \warning If the GNU Scientific Library (GSL) third party library is used to compute the SVD
  decomposition, the singular values \f$ \Sigma_{i,i} \f$ are ordered in decreasing
  fashion in \e w. This is not the case, if the GSL is not detected by ViSP.

  Here an example of SVD decomposition of a non square Matrix M.

\code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix M(3,2);
  M[0][0] = 1;
  M[1][0] = 2;
  M[2][0] = 0.5;

  M[0][1] = 6;
  M[1][1] = 8 ;
  M[2][1] = 9 ;

  vpMatrix v;
  vpColVector w;
  vpMatrix Mrec;
  vpMatrix Sigma;

  M.svd(w, v);
  // Here M is modified and is now equal to U

  // Construct the diagonal matrix from the singular values
  Sigma.diag(w);

  // Reconstruct the initial matrix M using the decomposition
  Mrec =  M * Sigma * v.t();

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

*/
void
vpMatrix::svd(vpColVector& w, vpMatrix& v)
{
#if 1 /* no verification */
  {
    w.resize( this->getCols() );
    v.resize( this->getCols(), this->getCols() );

#if defined (VISP_HAVE_LAPACK_C)
    svdLapack(w,v);
#elif (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
    svdOpenCV(w,v);
#elif defined (VISP_HAVE_GSL)  /* be careful of the copy below */
    svdGsl(w,v) ;
#else
    svdNr(w,v) ;
#endif

    //svdNr(w,v) ;
  }
#else  /* verification of the SVD */
  {
    int pb = 0;
    unsigned int i,j,k,nrows,ncols;
    vpMatrix A, Asvd;

    A = (*this);        /* copy because svd is destructive */

    w.resize( this->getCols() );
    v.resize( this->getCols(), this->getCols() );
#ifdef VISP_HAVE_GSL  /* be careful of the copy above */
    svdGsl(w,v) ;
#else
    svdNr(w,v) ;
#endif
    //svdNr(w,v) ;

    nrows = A.getRows();
    ncols = A.getCols();
    Asvd.resize(nrows,ncols);

    for (i = 0 ; i < nrows ; i++)
    {
      for (j = 0 ; j < ncols ; j++)
      {
        Asvd[i][j] = 0.0;
        for (k=0 ; k < ncols ; k++) Asvd[i][j] += (*this)[i][k]*w[k]*v[j][k];
      }
    }
    for (i=0;i<nrows;i++)
    {
      for (j=0;j<ncols;j++) if (fabs(A[i][j]-Asvd[i][j]) > 1e-6) pb = 1;
    }
    if (pb == 1)
    {
      printf("pb in SVD\n");
      std::cout << " A : " << std::endl << A << std::endl;
      std::cout << " Asvd : " << std::endl << Asvd << std::endl;
    }
    //    else printf("SVD ok ;-)\n");  /* It's so good... */
  }
#endif
}
/*!
  Compute the pseudo inverse of the matrix \f$Ap = A^+\f$
  \param Ap : The pseudo inverse \f$ A^+ \f$.
  \param svThreshold : Threshold used to test the singular values.
  \return Return the rank of the matrix A
*/

unsigned int
vpMatrix::pseudoInverse(vpMatrix &Ap, double svThreshold) const
{
  vpColVector sv ;
  return  pseudoInverse(Ap, sv, svThreshold) ;
}

/*!
  Compute and return the pseudo inverse of a n-by-m matrix : \f$ A^+ \f$
  \param svThreshold : Threshold used to test the singular values.

  \return Pseudo inverse of the matrix.

  Here an example to compute the inverse of a n-by-n matrix. If the
  matrix is n-by-n it is also possible to use inverseByLU().

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
  A_1 = A.pseudoInverse();
  std::cout << "Inverse by pseudo inverse: \n" << A_1 << std::endl;

  std::cout << "A*A^-1: \n" << A * A_1 << std::endl;
}
\endcode

  \sa inverseByLU()

*/
vpMatrix
vpMatrix::pseudoInverse(double svThreshold) const
{
  vpMatrix Ap ;
  vpColVector sv ;
  pseudoInverse(Ap, sv, svThreshold) ;
  return   Ap ;
}

/*!
  Compute the pseudo inverse of the matrix \f$Ap = A^+\f$
  \param Ap : The pseudo inverse \f$ A^+ \f$.
  \param sv : Singular values.
  \param svThreshold : Threshold used to test the singular values.
  \return Return the rank of the matrix A
*/
unsigned int
vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold) const
{
  vpMatrix imA, imAt ;
  return pseudoInverse(Ap, sv, svThreshold, imA, imAt) ;
}

/*!
  Compute the pseudo inverse of the matrix \f$Ap = A^+\f$ along with Ker A, Ker \f$A^T\f$, Im A and Im \f$A^T\f$

  Pseudo inverse, kernel and image are computed using the SVD decomposition.

  A is an m x n matrix,
  if m >=n the svd works on A other wise it works on \f$A^T\f$.

  Therefore if m>=n we have

  \f[
  {\bf A}_{m\times n} = {\bf U}_{m\times m} {\bf S}_{m\times n} {\bf V^\top}_{n\times n}
  \f]
  \f[
  {\bf A}_{m\times n} = \left[\begin{array}{ccc}\mbox{Im} {\bf A} & | &
  \mbox{Ker} {\bf A^\top} \end{array} \right] {\bf S}
  \left[
  \begin{array}{c} (\mbox{Im} {\bf A^\top})^\top \\   (\mbox{Ker}{\bf A})^\top \end{array}\right]
  \f]
  where
  Im(A) is an m x r matrix (r is the rank of A) and
  Im(A^T) is an r x n matrix

  \param Ap : The pseudo inverse \f$ A^+ \f$.
  \param sv : Singular values.
  \param svThreshold : Threshold used to test the singular values.
  \param imAt : Image A^T
  \param imA: Image  A
  \return Return the rank of the matrix A

*/
unsigned int 
vpMatrix::pseudoInverse(vpMatrix &Ap,
                        vpColVector &sv, double svThreshold,
                        vpMatrix &imA,
                        vpMatrix &imAt) const
{

  unsigned int i, j, k ;

  unsigned int nrows, ncols;
  unsigned int nrows_orig = getRows() ;
  unsigned int ncols_orig = getCols() ;
  Ap.resize(ncols_orig,nrows_orig) ;

  if (nrows_orig >=  ncols_orig)
  {
    nrows = nrows_orig;
    ncols = ncols_orig;
  }
  else
  {
    nrows = ncols_orig;
    ncols = nrows_orig;
  }

  vpMatrix a(nrows,ncols) ;
  vpMatrix a1(ncols,nrows);
  vpMatrix v(ncols,ncols) ;
  sv.resize(ncols) ;

  if (nrows_orig >=  ncols_orig) a = *this;
  else a = (*this).t();

  a.svd(sv,v);

  // compute the highest singular value and the rank of h
  double maxsv = 0 ;
  for (i=0 ; i < ncols ; i++)
    if (fabs(sv[i]) > maxsv) maxsv = fabs(sv[i]) ;

  unsigned int rank = 0 ;
  for (i=0 ; i < ncols ; i++)
    if (fabs(sv[i]) > maxsv*svThreshold) rank++ ;

  /*------------------------------------------------------- */
  for (i = 0 ; i < ncols ; i++)
  {
    for (j = 0 ; j < nrows ; j++)
    {
      a1[i][j] = 0.0;

      for (k=0 ; k < ncols ; k++)
        if (fabs(sv[k]) > maxsv*svThreshold)
        {
          a1[i][j] += v[i][k]*a[j][k]/sv[k];
        }
    }
  }
  if (nrows_orig >=  ncols_orig) Ap = a1;
  else Ap = a1.t();

  if (nrows_orig >=  ncols_orig)
  {
    //  compute dim At
    imAt.resize(ncols_orig,rank) ;
    for (i=0 ; i  < ncols_orig ; i++)
      for (j=0 ; j < rank ; j++)
        imAt[i][j] = v[i][j] ;

    //  compute dim A
    imA.resize(nrows_orig,rank) ;
    for (i=0 ; i  < nrows_orig ; i++)
      for (j=0 ; j < rank ; j++)
        imA[i][j] = a[i][j] ;
  }
  else
  {
    //  compute dim At
    imAt.resize(ncols_orig,rank) ;
    for (i=0 ; i  < ncols_orig ; i++)
      for (j=0 ; j < rank ; j++)
        imAt[i][j] = a[i][j] ;

    imA.resize(nrows_orig,rank) ;
    for (i=0 ; i  < nrows_orig ; i++)
      for (j=0 ; j < rank ; j++)
        imA[i][j] = v[i][j] ;

  }

#if 0 // debug
  {
    int pb = 0;
    vpMatrix A, ApA, AAp, AApA, ApAAp ;

    nrows = nrows_orig;
    ncols = ncols_orig;

    A.resize(nrows,ncols) ;
    A = *this ;

    ApA = Ap * A;
    AApA = A * ApA;
    ApAAp = ApA * Ap;
    AAp = A * Ap;

    for (i=0;i<nrows;i++)
    {
      for (j=0;j<ncols;j++) if (fabs(AApA[i][j]-A[i][j]) > 1e-6) pb = 1;
    }
    for (i=0;i<ncols;i++)
    {
      for (j=0;j<nrows;j++) if (fabs(ApAAp[i][j]-Ap[i][j]) > 1e-6) pb = 1;
    }
    for (i=0;i<nrows;i++)
    {
      for (j=0;j<nrows;j++) if (fabs(AAp[i][j]-AAp[j][i]) > 1e-6) pb = 1;
    }
    for (i=0;i<ncols;i++)
    {
      for (j=0;j<ncols;j++) if (fabs(ApA[i][j]-ApA[j][i]) > 1e-6) pb = 1;
    }
    if (pb == 1)
    {
      printf("pb in pseudo inverse\n");
      std::cout << " A : " << std::endl << A << std::endl;
      std::cout << " Ap : " << std::endl << Ap << std::endl;
      std::cout << " A - AApA : " << std::endl << A - AApA << std::endl;
      std::cout << " Ap - ApAAp : " << std::endl << Ap - ApAAp << std::endl;
      std::cout << " AAp - (AAp)^T : " << std::endl << AAp - AAp.t() << std::endl;
      std::cout << " ApA - (ApA)^T : " << std::endl << ApA - ApA.t() << std::endl;
    }
    //    else printf("Ap OK ;-) \n");

  }
#endif

  // std::cout << v << std::endl ;
  return rank ;
}



/*!
  Compute the pseudo inverse of the matrix \f$Ap = A^+\f$ along with Ker A, Ker \f$A^T\f$, Im A and Im \f$A^T\f$

  Pseudo inverse, kernel and image are computed using the SVD decomposition.

  A is an m x n matrix,
  if m >=n the svd works on A other wise it works on \f$A^T\f$.

  Therefore if m>=n we have

\f[
{\bf A}_{m\times n} = {\bf U}_{m\times m} {\bf S}_{m\times n} {\bf V^\top}_{n\times n}
\f]
\f[
{\bf A}_{m\times n} = \left[\begin{array}{ccc}\mbox{Im} {\bf A} & | &
\mbox{Ker} {\bf A^\top} \end{array} \right] {\bf S}
\left[
\begin{array}{c} (\mbox{Im} {\bf A^\top})^\top \\   (\mbox{Ker}{\bf A})^\top \end{array}\right]
\f]
where
Im(A) is an m x r matrix (r is the rank of A) and
Im(A^T) is an r x n matrix


  \param Ap : The pseudo inverse \f$ A^+ \f$.
  \param sv : Singular values.
  \param svThreshold : Threshold used to test the singular values.
  \param imA: Image  A
  \param imAt : Image A^T
  \param kerA : null space of A
  \return Return the rank of the matrix A

*/
unsigned int 
vpMatrix::pseudoInverse(vpMatrix &Ap,
                        vpColVector &sv, double svThreshold,
                        vpMatrix &imA,
                        vpMatrix &imAt,
                        vpMatrix &kerA) const
{

  unsigned int i, j, k ;

  unsigned int nrows, ncols;
  unsigned int nrows_orig = getRows() ;
  unsigned int ncols_orig = getCols() ;
  Ap.resize(ncols_orig,nrows_orig) ;

  if (nrows_orig >=  ncols_orig)
  {
    nrows = nrows_orig;
    ncols = ncols_orig;
  }
  else
  {
    nrows = ncols_orig;
    ncols = nrows_orig;
  }

  vpMatrix a(nrows,ncols) ;
  vpMatrix a1(ncols,nrows);
  vpMatrix v(ncols,ncols) ;
  sv.resize(ncols) ;

  if (nrows_orig >=  ncols_orig) a = *this;
  else a = (*this).t();

  a.svd(sv,v);

  // compute the highest singular value and the rank of h
  double maxsv = 0 ;
  for (i=0 ; i < ncols ; i++)
    if (fabs(sv[i]) > maxsv) maxsv = fabs(sv[i]) ;

  unsigned int rank = 0 ;
  for (i=0 ; i < ncols ; i++)
    if (fabs(sv[i]) > maxsv*svThreshold) rank++ ;



  /*------------------------------------------------------- */
  for (i = 0 ; i < ncols ; i++)
  {
    for (j = 0 ; j < nrows ; j++)
    {
      a1[i][j] = 0.0;

      for (k=0 ; k < ncols ; k++)
        if (fabs(sv[k]) > maxsv*svThreshold)
        {
          a1[i][j] += v[i][k]*a[j][k]/sv[k];
        }
    }
  }
  if (nrows_orig >=  ncols_orig) Ap = a1;
  else Ap = a1.t();

  if (nrows_orig >=  ncols_orig)
  {
    //  compute dim At
    imAt.resize(ncols_orig,rank) ;
    for (i=0 ; i  < ncols_orig ; i++)
      for (j=0 ; j < rank ; j++)
        imAt[i][j] = v[i][j] ;

    //  compute dim A
    imA.resize(nrows_orig,rank) ;
    for (i=0 ; i  < nrows_orig ; i++)
      for (j=0 ; j < rank ; j++)
        imA[i][j] = a[i][j] ;
  }
  else
  {
    //  compute dim At
    imAt.resize(ncols_orig,rank) ;
    for (i=0 ; i  < ncols_orig ; i++)
      for (j=0 ; j < rank ; j++)
        imAt[i][j] = a[i][j] ;

    imA.resize(nrows_orig,rank) ;
    for (i=0 ; i  < nrows_orig ; i++)
      for (j=0 ; j < rank ; j++)
        imA[i][j] = v[i][j] ;

  }

  vpMatrix cons(ncols_orig, ncols_orig);
  cons = 0;

  for (j = 0; j < ncols_orig; j++)
  {
    for (i = 0; i < ncols_orig; i++)
    {
      if (fabs(sv[i]) <= maxsv*svThreshold)
      {
        cons[i][j] = v[j][i];
      }
    }
  }

  vpMatrix Ker (ncols_orig-rank, ncols_orig);
  k = 0;
  for (j = 0; j < ncols_orig ; j++)
  {
    //if ( cons.row(j+1).sumSquare() != 0)
    if ( std::fabs(cons.getRow(j).sumSquare()) > std::numeric_limits<double>::epsilon())
    {
      for (i = 0; i < cons.getCols(); i++)
        Ker[k][i] = cons[j][i];

      k++;
    }
  }
  kerA = Ker;

#if 0 // debug
  {
    int pb = 0;
    vpMatrix A, ApA, AAp, AApA, ApAAp ;

    nrows = nrows_orig;
    ncols = ncols_orig;

    A.resize(nrows,ncols) ;
    A = *this ;

    ApA = Ap * A;
    AApA = A * ApA;
    ApAAp = ApA * Ap;
    AAp = A * Ap;

    for (i=0;i<nrows;i++)
    {
      for (j=0;j<ncols;j++) if (fabs(AApA[i][j]-A[i][j]) > 1e-6) pb = 1;
    }
    for (i=0;i<ncols;i++)
    {
      for (j=0;j<nrows;j++) if (fabs(ApAAp[i][j]-Ap[i][j]) > 1e-6) pb = 1;
    }
    for (i=0;i<nrows;i++)
    {
      for (j=0;j<nrows;j++) if (fabs(AAp[i][j]-AAp[j][i]) > 1e-6) pb = 1;
    }
    for (i=0;i<ncols;i++)
    {
      for (j=0;j<ncols;j++) if (fabs(ApA[i][j]-ApA[j][i]) > 1e-6) pb = 1;
    }
    if (pb == 1)
    {
      printf("pb in pseudo inverse\n");
      std::cout << " A : " << std::endl << A << std::endl;
      std::cout << " Ap : " << std::endl << Ap << std::endl;
      std::cout << " A - AApA : " << std::endl << A - AApA << std::endl;
      std::cout << " Ap - ApAAp : " << std::endl << Ap - ApAAp << std::endl;
      std::cout << " AAp - (AAp)^T : " << std::endl << AAp - AAp.t() << std::endl;
      std::cout << " ApA - (ApA)^T : " << std::endl << ApA - ApA.t() << std::endl;
      std::cout << " KerA : " << std::endl << kerA << std::endl;
    }
    //    else printf("Ap OK ;-) \n");

  }
#endif

  // std::cout << v << std::endl ;
  return rank ;
}

/*!
  Extract a column vector from a matrix.
  \warning All the indexes start from 0 in this function.
  \param j : Index of the column to extract. If col=0, the first column is extracted.
  \param i_begin : Index of the row that gives the location of the first element of the column vector to extract.
  \param column_size : Size of the column vector to extract.
  \return The extracted column vector.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(4,4);

  for(unsigned int i=0; i < A.getRows(); i++)
    for(unsigned int j=0; j < A.getCols(); j++)
      A[i][j] = i*A.getCols()+j;

  A.print(std::cout, 4);

  vpColVector cv = A.getCol(1, 1, 3);
  std::cout << "Column vector: \n" << cv << std::endl;
}
  \endcode
It produces the following output:
  \code
[4,4]=
   0  1  2  3
   4  5  6  7
   8  9 10 11
  12 13 14 15
column vector:
5
9
13
  \endcode
 */
vpColVector
vpMatrix::getCol(const unsigned int j, const unsigned int i_begin, const unsigned int column_size) const
{
  if (i_begin + column_size > getRows() || j >= getCols())
    throw(vpException(vpException::dimensionError, "Unable to extract a column vector from the matrix"));
  vpColVector c(column_size);
  for (unsigned int i=0 ; i < column_size ; i++)
    c[i] = (*this)[i_begin+i][j];
  return c;
}

/*!
  Extract a column vector from a matrix.
  \warning All the indexes start from 0 in this function.
  \param j : Index of the column to extract. If j=0, the first column is extracted.
  \return The extracted column vector.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(4,4);

  for(unsigned int i=0; i < A.getRows(); i++)
    for(unsigned int j=0; j < A.getCols(); j++)
      A[i][j] = i*A.getCols()+j;

  A.print(std::cout, 4);

  vpColVector cv = A.getCol(1);
  std::cout << "Column vector: \n" << cv << std::endl;
}
  \endcode
It produces the following output:
  \code
[4,4]=
   0  1  2  3
   4  5  6  7
   8  9 10 11
  12 13 14 15
column vector:
1
5
9
13
  \endcode
 */
vpColVector
vpMatrix::getCol(const unsigned int j) const
{
  if (j >= getCols())
    throw(vpException(vpException::dimensionError, "Unable to extract a column vector from the matrix"));
  unsigned int nb_rows = getRows();
  vpColVector c(nb_rows);
  for (unsigned int i=0 ; i < nb_rows ; i++)
    c[i] = (*this)[i][j];
  return c;
}

/*!
  Extract a row vector from a matrix.
  \warning All the indexes start from 0 in this function.
  \param i : Index of the row to extract. If i=0, the first row is extracted.
  \return The extracted row vector.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRowVector.h>

int main()
{
  vpMatrix A(4,4);

  for(unsigned int i=0; i < A.getRows(); i++)
    for(unsigned int j=0; j < A.getCols(); j++)
      A[i][j] = i*A.getCols()+j;

  A.print(std::cout, 4);

  vpRowVector rv = A.getRow(1);
  std::cout << "Row vector: \n" << rv << std::endl;
}  \endcode
It produces the following output:
  \code
[4,4]=
   0  1  2  3
   4  5  6  7
   8  9 10 11
  12 13 14 15
Row vector:
4  5  6  7
  \endcode
 */
vpRowVector
vpMatrix::getRow(const unsigned int i) const
{
  if (i >= getRows())
    throw(vpException(vpException::dimensionError, "Unable to extract a row vector from the matrix"));
  unsigned int nb_cols = getCols();
  vpRowVector r( nb_cols );
  for (unsigned int j=0 ; j < nb_cols ; j++)
    r[j] = (*this)[i][j];
  return r;
}

/*!
  Extract a row vector from a matrix.
  \warning All the indexes start from 0 in this function.
  \param i : Index of the row to extract. If i=0, the first row is extracted.
  \param j_begin : Index of the column that gives the location of the first element of the row vector to extract.
  \param row_size : Size of the row vector to extract.
  \return The extracted row vector.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRowVector.h>

int main()
{
  vpMatrix A(4,4);

  for(unsigned int i=0; i < A.getRows(); i++)
    for(unsigned int j=0; j < A.getCols(); j++)
      A[i][j] = i*A.getCols()+j;

  A.print(std::cout, 4);

  vpRowVector rv = A.getRow(1, 1, 3);
  std::cout << "Row vector: \n" << rv << std::endl;
}  \endcode
It produces the following output:
  \code
[4,4]=
   0  1  2  3
   4  5  6  7
   8  9 10 11
  12 13 14 15
Row vector:
5  6  7
  \endcode
 */
vpRowVector
vpMatrix::getRow(const unsigned int i, const unsigned int j_begin, const unsigned int row_size) const
{
  if (j_begin + row_size > getCols() || i >= getRows())
    throw(vpException(vpException::dimensionError, "Unable to extract a row vector from the matrix"));
  vpRowVector r(row_size);
  for (unsigned int j=0 ; j < row_size ; j++)
    r[j] = (*this)[i][j_begin+i];
  return r;
}

/*!
  Stack matrix \e B to the end of matrix \e A and return the resulting matrix  [ A B ]^T

  \param A : Upper matrix.
  \param B : Lower matrix.
  \return Stacked matrix [ A B ]^T

  \warning A and B must have the same number of columns.
*/
vpMatrix
vpMatrix::stack(const vpMatrix &A, const vpMatrix &B)
{
  vpMatrix C ;

  try{
    vpMatrix::stack(A, B, C) ;
  }
  catch(...) {
    throw ;
  }

  return C ;
}

/*!
  Stack row vector \e r to matrix \e A and return the resulting matrix [ A r ]^T

  \param A : Upper matrix.
  \param r : Lower matrix.
  \return Stacked matrix [ A r ]^T

  \warning \e A and \e r must have the same number of columns.
*/
vpMatrix
vpMatrix::stack(const vpMatrix &A, const vpRowVector &r)
{
  vpMatrix C ;

  try{
    vpMatrix::stack(A, r, C) ;
  }
  catch(...) {
    throw ;
  }

  return C ;
}

/*!
  Stack matrix \e B to the end of matrix \e A and return the resulting matrix in \e C.

  \param  A : Upper matrix.
  \param  B : Lower matrix.
  \param  C : Stacked matrix C = [ A B ]^T

  \warning A and B must have the same number of columns.
*/
void
vpMatrix::stack(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
  unsigned int nra = A.getRows() ;
  unsigned int nrb = B.getRows() ;

  if (nra !=0)
    if (A.getCols() != B.getCols()) {
      throw(vpException(vpException::dimensionError,
                        "Cannot stack (%dx%d) matrix with (%dx%d) matrix",
                        A.getRows(), A.getCols(), B.getRows(), B.getCols())) ;
    }

  try {
    C.resize(nra+nrb,B.getCols()  ) ;
  }
  catch(...) {
    throw ;
  }

  unsigned int i,j ;
  for (i=0 ; i < nra ; i++) {
    for (j=0 ; j < A.getCols() ; j++)
      C[i][j] = A[i][j] ;
  }

  for (i=0 ; i < nrb ; i++) {
    for (j=0 ; j < B.getCols() ; j++) {
      C[i+nra][j] = B[i][j] ;
    }
  }
}

/*!
  Stack row vector \e v to the end of matrix \e A and return the resulting matrix in \e C.

  \param  A : Upper matrix.
  \param  r : Lower row vector.
  \param  C : Stacked matrix C = [ A r ]^T

  \warning A and r must have the same number of columns.
*/
void
vpMatrix::stack(const vpMatrix &A, const vpRowVector &r, vpMatrix &C)
{
  unsigned int nra = A.getRows() ;

  if (nra !=0)
    if (A.getCols() != r.getCols()) {
      throw(vpException(vpException::dimensionError,
                        "Cannot stack (%dx%d) matrix with (1x%d) row vector",
                        A.getRows(), A.getCols(), r.getCols())) ;
    }

  try {
    C.resize(nra+1,r.getCols()  ) ;
  }
  catch(...) {
    throw ;
  }

  unsigned int i,j ;
  for (i=0 ; i < nra ; i++) {
    for (j=0 ; j < A.getCols() ; j++)
      C[i][j] = A[i][j] ;
  }

  for (j=0 ; j < r.getCols() ; j++) {
    C[nra][j] = r[j] ;
  }
}

/*!
  Insert matrix B in matrix A at the given position.

  \param A : Main matrix.
  \param B : Matrix to insert.
  \param r : Index of the row where to add the matrix.
  \param c : Index of the column where to add the matrix.
  \return Matrix with B insert in A.

  \warning Throw exception if the sizes of the matrices do not allow the insertion.
*/
vpMatrix
vpMatrix::insert(const vpMatrix &A, const vpMatrix &B, 
                 const unsigned int r, const unsigned int c)
{
  vpMatrix C ;

  try{
    insert(A,B, C, r, c) ;
  }
  catch(...) {
    throw;
  }

  return C ;
}

/*!
  \relates vpMatrix
  Insert matrix B in matrix A at the given position.

  \param A : Main matrix.
  \param B : Matrix to insert.
  \param C : Result matrix.
  \param r : Index of the row where to insert matrix B.
  \param c : Index of the column where to insert matrix B.

  \warning Throw exception if the sizes of the matrices do not
  allow the insertion.
*/
void
vpMatrix::insert(const vpMatrix &A, const vpMatrix &B, vpMatrix &C, 
                 const unsigned int r, const unsigned int c)
{
  if( ( (r + B.getRows()) <= A.getRows() ) && 
    ( (c + B.getCols()) <= A.getCols() ) ){
      try {
        C.resize(A.getRows(),A.getCols()  ) ;
      }
      catch(...)  {
        throw ;
      }
      for(unsigned int i=0; i<A.getRows(); i++){
        for(unsigned int j=0; j<A.getCols(); j++){
          if(i >= r && i < (r + B.getRows()) && j >= c && j < (c+B.getCols())){
            C[i][j] = B[i-r][j-c];
          }
          else{
            C[i][j] = A[i][j];
          }
        }
      }
  }
  else{
    throw vpException(vpException::dimensionError,
                      "Cannot insert (%dx%d) matrix in (%dx%d) matrix at position (%d,%d)",
                      B.getRows(), B.getCols(), A.getCols(), A.getRows(), r, c);
  }
}

/*!
  Juxtapose to matrices C = [ A B ].

  \f$ C = \left( \begin{array}{cc} A & B \end{array}\right)    \f$

  \param A : Left matrix.
  \param B : Right matrix.
  \return Juxtaposed matrix C = [ A B ]

  \warning A and B must have the same number of column
*/
vpMatrix
vpMatrix::juxtaposeMatrices(const vpMatrix &A, const vpMatrix &B)
{
  vpMatrix C ;

  try{
    juxtaposeMatrices(A,B, C) ;
  }
  catch(...) {
    throw ;
  }

  return C ;
}

/*!
  \relates vpMatrix
  Juxtapose to matrices C = [ A B ].

  \f$ C = \left( \begin{array}{cc} A & B \end{array}\right)    \f$

  \param A : Left matrix.
  \param B : Right matrix.
  \param C : Juxtaposed matrix C = [ A B ]

  \warning A and B must have the same number of rows.
*/
void
vpMatrix::juxtaposeMatrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
  unsigned int nca = A.getCols() ;
  unsigned int ncb = B.getCols() ;

  if (nca !=0)
    if (A.getRows() != B.getRows()) {
      throw(vpException(vpException::dimensionError,
                        "Cannot juxtapose (%dx%d) matrix with (%dx%d) matrix",
                        A.getRows(), A.getCols(), B.getRows(), B.getCols())) ;
    }

    try {
      C.resize(B.getRows(),nca+ncb) ;
    }
    catch(...) {
      throw ;
    }

    unsigned int i,j ;
    for (i=0 ; i < C.getRows(); i++)
      for (j=0 ; j < nca ; j++)
        C[i][j] = A[i][j] ;

    for (i=0 ; i < C.getRows() ; i++)
      for (j=0 ; j < ncb ; j++){
        C[i][nca+j] = B[i][j] ;
      }
}


//--------------------------------------------------------------------
// Output
//--------------------------------------------------------------------

/*!

  Pretty print a matrix. The data are tabulated.
  The common widths before and after the decimal point
  are set with respect to the parameter maxlen.

  \param s Stream used for the printing.

  \param length The suggested width of each matrix element.
  The actual width grows in order to accomodate the whole integral part,
  and shrinks if the whole extent is not needed for all the numbers.
  \param intro The introduction which is printed before the matrix.
  Can be set to zero (or omitted), in which case
  the introduction is not printed.

  \return Returns the common total width for all matrix elements

  \sa std::ostream &operator<<(std::ostream &s, const vpArray2D<Type> &A)
*/
int
vpMatrix::print(std::ostream& s, unsigned int length, char const* intro) const
{
  typedef std::string::size_type size_type;

  unsigned int m = getRows();
  unsigned int n = getCols();

  std::vector<std::string> values(m*n);
  std::ostringstream oss;
  std::ostringstream ossFixed;
  std::ios_base::fmtflags original_flags = oss.flags();

  // ossFixed <<std::fixed;
  ossFixed.setf ( std::ios::fixed, std::ios::floatfield );

  size_type maxBefore=0;  // the length of the integral part
  size_type maxAfter=0;   // number of decimals plus
  // one place for the decimal point
  for (unsigned int i=0;i<m;++i) {
    for (unsigned int j=0;j<n;++j){
      oss.str("");
      oss << (*this)[i][j];
      if (oss.str().find("e")!=std::string::npos){
        ossFixed.str("");
        ossFixed << (*this)[i][j];
        oss.str(ossFixed.str());
      }

      values[i*n+j]=oss.str();
      size_type thislen=values[i*n+j].size();
      size_type p=values[i*n+j].find('.');

      if (p==std::string::npos){
        maxBefore=vpMath::maximum(maxBefore, thislen);
        // maxAfter remains the same
      } else{
        maxBefore=vpMath::maximum(maxBefore, p);
        maxAfter=vpMath::maximum(maxAfter, thislen-p-1);
      }
    }
  }

  size_type totalLength=length;
  // increase totalLength according to maxBefore
  totalLength=vpMath::maximum(totalLength,maxBefore);
  // decrease maxAfter according to totalLength
  maxAfter=std::min(maxAfter, totalLength-maxBefore);
  if (maxAfter==1) maxAfter=0;

  // the following line is useful for debugging
  //std::cerr <<totalLength <<" " <<maxBefore <<" " <<maxAfter <<"\n";

  if (intro) s <<intro;
  s <<"["<<m<<","<<n<<"]=\n";

  for (unsigned int i=0;i<m;i++) {
    s <<"  ";
    for (unsigned int j=0;j<n;j++){
      size_type p=values[i*n+j].find('.');
      s.setf(std::ios::right, std::ios::adjustfield);
      s.width((std::streamsize)maxBefore);
      s <<values[i*n+j].substr(0,p).c_str();

      if (maxAfter>0){
        s.setf(std::ios::left, std::ios::adjustfield);
        if (p!=std::string::npos){
          s.width((std::streamsize)maxAfter);
          s <<values[i*n+j].substr(p,maxAfter).c_str();
        } else{
          assert(maxAfter>1);
          s.width((std::streamsize)maxAfter);
          s <<".0";
        }
      }

      s <<' ';
    }
    s <<std::endl;
  }

  s.flags(original_flags); // restore s to standard state

  return (int)(maxBefore+maxAfter);
}


/*!
  Print using Matlab syntax, to copy/paste in Matlab later.

  The following code
  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix M(2,3);
  int cpt = 0;
  for (unsigned int i=0; i<M.getRows(); i++)
    for (unsigned int j=0; j<M.getCols(); j++)
      M[i][j] = cpt++;

  std::cout << "M = "; M.matlabPrint(std::cout);
}
  \endcode
  produces this output:
  \code
M = [ 0, 1, 2, ;
3, 4, 5, ]
  \endcode
  that could be copy/paste in Matlab:
  \code
>> M = [ 0, 1, 2, ;
3, 4, 5, ]

M =

    0    1    2
    3    4    5

>>
  \endcode
*/
std::ostream & vpMatrix::matlabPrint(std::ostream & os) const
{
  os << "[ ";
  for (unsigned int i=0; i < this->getRows(); ++ i) {
    for (unsigned int j=0; j < this ->getCols(); ++ j) {
      os <<  (*this)[i][j] << ", ";
    }
    if (this ->getRows() != i+1) { os << ";" << std::endl; }
    else { os << "]" << std::endl; }
  }
  return os;
};

/*!
  Print using Maple syntax, to copy/paste in Maple later.

  The following code
  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix M(2,3);
  int cpt = 0;
  for (unsigned int i=0; i<M.getRows(); i++)
    for (unsigned int j=0; j<M.getCols(); j++)
      M[i][j] = cpt++;

  std::cout << "M = "; M.maplePrint(std::cout);
}
  \endcode
  produces this output:
  \code
M = ([
[0, 1, 2, ],
[3, 4, 5, ],
])
  \endcode
  that could be copy/paste in Maple.

*/
std::ostream & vpMatrix::maplePrint(std::ostream & os) const
{
  os << "([ " << std::endl;
  for (unsigned int i=0; i < this->getRows(); ++ i) {
    os << "[";
    for (unsigned int j=0; j < this->getCols(); ++ j) {
      os <<  (*this)[i][j] << ", ";
    }
    os << "]," << std::endl;
  }
  os << "])" << std::endl;
  return os;
};

/*!
  Print/save a matrix in csv format.

  The following code
  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  std::ofstream ofs("log.csv", std::ofstream::out);
  vpMatrix M(2,3);
  int cpt = 0;
  for (unsigned int i=0; i<M.getRows(); i++)
    for (unsigned int j=0; j<M.getCols(); j++)
      M[i][j] = cpt++;

  M.csvPrint(ofs);

  ofs.close();
}
  \endcode
  produces log.csv file that contains:
  \code
0, 1, 2
3, 4, 5
  \endcode
*/
std::ostream & vpMatrix::csvPrint(std::ostream & os) const
{
  for (unsigned int i=0; i < this->getRows(); ++ i) {
    for (unsigned int j=0; j < this->getCols(); ++ j) {
      os <<  (*this)[i][j];
      if (!(j==(this->getCols()-1)))
        os << ", ";
    }
    os << std::endl;
  }
  return os;
};


/*!
  Print to be used as part of a C++ code later.

  \param os : the stream to be printed in.
  \param matrixName : name of the matrix, "A" by default.
  \param octet : if false, print using double, if true, print byte per byte
  each bytes of the double array.

  The following code shows how to use this function:
  \code
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix M(2,3);
  int cpt = 0;
  for (unsigned int i=0; i<M.getRows(); i++)
    for (unsigned int j=0; j<M.getCols(); j++)
      M[i][j] = cpt++;

  M.cppPrint(std::cout, "M");
}
  \endcode
  It produces the following output that could be copy/paste in a C++ code:
  \code
vpMatrix M (2, 3);
M[0][0] = 0;
M[0][1] = 1;
M[0][2] = 2;

M[1][0] = 3;
M[1][1] = 4;
M[1][2] = 5;

  \endcode
*/
std::ostream & vpMatrix::cppPrint(std::ostream & os, const std::string &matrixName, bool octet) const
{
  os << "vpMatrix " << matrixName
     << " (" << this ->getRows ()
     << ", " << this ->getCols () << "); " <<std::endl;

  for (unsigned int i=0; i < this->getRows(); ++ i)
  {
    for (unsigned int j=0; j < this ->getCols(); ++ j)
    {
      if (! octet)
      {
        os << matrixName << "[" << i << "][" << j
           << "] = " << (*this)[i][j] << "; " << std::endl;
      }
      else
      {
        for (unsigned int k = 0; k < sizeof(double); ++ k)
        {
          os << "((unsigned char*)&(" << matrixName
             << "[" << i << "][" << j << "]) )[" << k
             << "] = 0x" << std::hex
             << (unsigned int)((unsigned char*)& ((*this)[i][j])) [k]
             << "; " << std::endl;
        }
      }
    }
    os << std::endl;
  }
  return os;
};

/*!
  Compute the determinant of the matrix using the LU Decomposition.

  \return The determinant of the matrix if the matrix is square, 0 otherwise.

  See the Numerical Recipes in C page 43 for further explanations.
*/

double vpMatrix::detByLU() const
{
  double det_ = 0;

  // Test wether the matrix is squred
  if (rowNum == colNum)
  {
    // create a temporary matrix that will be modified by LUDcmp
    vpMatrix tmp(*this);

    // using th LUdcmp based on NR codes
    // it modified the tmp matrix in a special structure of type :
    //  b11 b12 b13 b14
    //  a21 b22 b23 b24
    //  a21 a32 b33 b34
    //  a31 a42 a43 b44 

    unsigned int  * perm = new unsigned int[rowNum];  // stores the permutations
    int d;   // +- 1 fi the number of column interchange is even or odd
    tmp.LUDcmp(perm,  d);
    delete[]perm;

    // compute the determinant that is the product of the eigen values
    det_ = (double) d;
    for(unsigned int i=0;i<rowNum;i++)
    {
      det_*=tmp[i][i];
    }
  }
  else {
    throw(vpException(vpException::fatalError,
                      "Cannot compute LU decomposition on a non square matrix (%dx%d)",
                      rowNum, colNum)) ;
  }
  return det_ ;
}



/*!
  Stack A at the end of the current matrix, or copy if the matrix has no dimensions : this = [ this A ]^T.

  Here an example for a robot velocity log :
\code
vpMatrix A;
vpColVector v(6);
for(unsigned int i = 0;i<100;i++)
{
  robot.getVelocity(vpRobot::ARTICULAR_FRAME, v);
  Velocities.stackMatrices(v.t());
}
\endcode
*/
void vpMatrix::stack(const vpMatrix &A)
{
  if(rowNum == 0)
    *this = A;
  else
    *this = vpMatrix::stack(*this, A);
}

/*!
  Stack row vector \e r at the end of the current matrix, or copy if the matrix has no dimensions : this = [ this r ]^T.
*/
void vpMatrix::stack(const vpRowVector &r)
{
  if(rowNum == 0)
    *this = r;
  else
    *this = vpMatrix::stack(*this, r);
}


/*!
  Insert matrix A at the given position in the current matrix.

  \warning Throw vpException::dimensionError if the
  dimensions of the matrices do not allow the operation.

  \param A : The matrix to insert.
  \param r : The index of the row to begin to insert data.
  \param c : The index of the column to begin to insert data.
*/
void vpMatrix::insert(const vpMatrix&A, const unsigned int r, 
                      const unsigned int c)
{
  if( (r + A.getRows() ) <= rowNum && (c + A.getCols() ) <= colNum ){
    // recopy matrix A in the current one, does not call static function to avoid initialisation and recopy of matrix
    for(unsigned int i=r; i<(r+A.getRows()); i++){
      for(unsigned int j=c; j<(c+A.getCols()); j++){
        (*this)[i][j] = A[i-r][j-c];
      }
    }
  }
  else{
    throw vpException(vpException::dimensionError,
                      "Cannot insert (%dx%d) matrix in (%dx%d) matrix at position (%d,%d)",
                      A.getRows(), A.getCols(), rowNum, colNum, r, c);
  }
}


/*!
  Compute the eigenvalues of a n-by-n real symmetric matrix.

  \return The eigenvalues of a n-by-n real symmetric matrix.

  \warning This method is only available if the Gnu Scientific Library
  (GSL) is detected as a third party library.

  \exception vpException::dimensionError If the matrix is not square.
  \exception vpException::fatalError If the matrix is not symmetric.
  \exception vpException::functionNotImplementedError If the GSL library is not detected.

  Here an example:
\code
#include <iostream>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(3,3); // A is a symmetric matrix
  A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.;
  A[1][0] = 1/2.; A[1][1] = 1/3.; A[1][2] = 1/4.;
  A[2][0] = 1/3.; A[2][1] = 1/4.; A[2][2] = 1/5.;
  std::cout << "Initial symmetric matrix: \n" << A << std::endl;

  // Compute the eigen values
  vpColVector evalue; // Eigenvalues
  evalue = A.eigenValues();
  std::cout << "Eigen values: \n" << evalue << std::endl;
}
\endcode

  \sa eigenValues(vpColVector &, vpMatrix &)

*/ 
vpColVector vpMatrix::eigenValues() const
{
  if (rowNum != colNum) {
    throw(vpException(vpException::dimensionError,
                      "Cannot compute eigen values on a non square matrix (%dx%d)",
                      rowNum, colNum)) ;
  }

#ifdef VISP_HAVE_GSL  /* be careful of the copy below */
  {
    // Check if the matrix is symetric: At - A = 0
    vpMatrix At_A = (*this).t() - (*this);
    for (unsigned int i=0; i < rowNum; i++) {
      for (unsigned int j=0; j < rowNum; j++) {
        //if (At_A[i][j] != 0) {
        if (std::fabs(At_A[i][j]) > std::numeric_limits<double>::epsilon()) {
          throw(vpException(vpException::fatalError,
                            "Cannot compute eigen values on a non symetric matrix")) ;
        }
      }
    }

    vpColVector evalue(rowNum); // Eigen values

    gsl_vector *eval = gsl_vector_alloc (rowNum);
    gsl_matrix *evec = gsl_matrix_alloc (rowNum, colNum);

    gsl_eigen_symmv_workspace * w =  gsl_eigen_symmv_alloc (rowNum);
    gsl_matrix *m = gsl_matrix_alloc(rowNum, colNum);

    unsigned int Atda = (unsigned int)m->tda ;
    for (unsigned int i=0 ; i < rowNum ; i++){
      unsigned int k = i*Atda ;
      for (unsigned int j=0 ; j < colNum ; j++)
        m->data[k+j] = (*this)[i][j] ;
    }
    gsl_eigen_symmv (m, eval, evec, w);

    gsl_eigen_symmv_sort (eval, evec, GSL_EIGEN_SORT_ABS_ASC);

    for (unsigned int i=0; i < rowNum; i++) {
      evalue[i] = gsl_vector_get (eval, i);
    }

    gsl_eigen_symmv_free (w);
    gsl_vector_free (eval);
    gsl_matrix_free (m);
    gsl_matrix_free (evec);

    return evalue;
  }
#else
  {
    throw(vpException(vpException::functionNotImplementedError,
                      "Eigen values computation is not implemented. You should install GSL rd party")) ;
  }
#endif  
}

/*!
  Compute the eigenvalues of a n-by-n real symmetric matrix.
  \return The eigenvalues of a n-by-n real symmetric matrix.

  \warning This method is only available if the Gnu Scientific Library
  (GSL) is detected as a third party library.

  \param evalue : Eigenvalues of the matrix.

  \param evector : Eigenvector of the matrix.

  \exception vpException::dimensionError If the matrix is not square.
  \exception vpException::fatalError If the matrix is not symmetric.
  \exception vpException::functionNotImplementedError If the GSL library is not detected.

  Here an example:
\code
#include <iostream>

#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMatrix.h>

int main()
{
  vpMatrix A(4,4); // A is a symmetric matrix
  A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.; A[0][3] = 1/4.;
  A[1][0] = 1/2.; A[1][1] = 1/3.; A[1][2] = 1/4.; A[1][3] = 1/5.;
  A[2][0] = 1/3.; A[2][1] = 1/4.; A[2][2] = 1/5.; A[2][3] = 1/6.;
  A[3][0] = 1/4.; A[3][1] = 1/5.; A[3][2] = 1/6.; A[3][3] = 1/7.;
  std::cout << "Initial symmetric matrix: \n" << A << std::endl;

  vpColVector d; // Eigenvalues
  vpMatrix    V; // Eigenvectors

  // Compute the eigenvalues and eigenvectors
  A.eigenValues(d, V);
  std::cout << "Eigen values: \n" << d << std::endl;
  std::cout << "Eigen vectors: \n" << V << std::endl;

  vpMatrix D;
  D.diag(d); // Eigenvalues are on the diagonal

  std::cout << "D: " << D << std::endl;

  // Verification: A * V = V * D
  std::cout << "AV-VD = 0 ? \n" << (A*V) - (V*D) << std::endl;
}
\endcode

\sa eigenValues()

*/
#ifdef VISP_HAVE_GSL  /* be careful of the copy below */
void vpMatrix::eigenValues(vpColVector &evalue, vpMatrix &evector) const
#else
void vpMatrix::eigenValues(vpColVector & /* evalue */, vpMatrix & /* evector */) const
#endif
{
  if (rowNum != colNum) {
    throw(vpException(vpException::dimensionError,
                      "Cannot compute eigen values on a non square matrix (%dx%d)",
                      rowNum, colNum)) ;
  }

#ifdef VISP_HAVE_GSL  /* be careful of the copy below */
  {
    // Check if the matrix is symetric: At - A = 0
    vpMatrix At_A = (*this).t() - (*this);
    for (unsigned int i=0; i < rowNum; i++) {
      for (unsigned int j=0; j < rowNum; j++) {
        //if (At_A[i][j] != 0) {
        if (std::fabs(At_A[i][j]) > std::numeric_limits<double>::epsilon()) {
          throw(vpException(vpException::fatalError,
                            "Cannot compute eigen values on a non symetric matrix")) ;
        }
      }
    }

    // Resize the output matrices
    evalue.resize(rowNum);
    evector.resize(rowNum, colNum);

    gsl_vector *eval = gsl_vector_alloc (rowNum);
    gsl_matrix *evec = gsl_matrix_alloc (rowNum, colNum);

    gsl_eigen_symmv_workspace * w =  gsl_eigen_symmv_alloc (rowNum);
    gsl_matrix *m = gsl_matrix_alloc(rowNum, colNum);

    unsigned int Atda = (unsigned int)m->tda ;
    for (unsigned int i=0 ; i < rowNum ; i++){
      unsigned int k = i*Atda ;
      for (unsigned int j=0 ; j < colNum ; j++)
        m->data[k+j] = (*this)[i][j] ;
    }
    gsl_eigen_symmv (m, eval, evec, w);

    gsl_eigen_symmv_sort (eval, evec, GSL_EIGEN_SORT_ABS_ASC);

    for (unsigned int i=0; i < rowNum; i++) {
      evalue[i] = gsl_vector_get (eval, i);
    }
    Atda = (unsigned int)evec->tda ;
    for (unsigned int i=0; i < rowNum; i++) {
      unsigned int k = i*Atda ;
      for (unsigned int j=0; j < rowNum; j++) {
        evector[i][j] = evec->data[k+j];
      }
    }

    gsl_eigen_symmv_free (w);
    gsl_vector_free (eval);
    gsl_matrix_free (m);
    gsl_matrix_free (evec);
  }
#else
  {
    throw(vpException(vpException::functionNotImplementedError,
                      "Eigen values computation is not implemented. You should install GSL rd party")) ;
  }
#endif  
}


/*!
  Function to compute the null space (the kernel) of the interaction matrix A which is not full rank.
  The null space ( the kernel ) of a matrix A is defined as Null(A) = Ker(A) = {X : A*X =0}.

  \param kerA : The matrix to contain the null space (kernel) of A defined by the row vectors (A*KerA.t()=0)
  \param svThreshold : Specify the used threshold in the svd(...) function (a function to compute the singular value decomposition)

  \return the rank of the matrix.
*/

unsigned int 
vpMatrix::kernel(vpMatrix &kerA, double svThreshold) const
{
  unsigned int i, j ;
  unsigned int nbline = getRows() ;
  unsigned int nbcol = getCols() ;

  vpMatrix A ; // Copy of the matrix, SVD function is destructive
  vpColVector sv(nbcol) ;   // singular values
  vpMatrix v(nbcol,nbcol) ; // V matrix of singular value decomposition

  // Copy and resize matrix to have at least as many rows as columns
  // kernel is computed in svd method only if the matrix has more rows than columns

  if (nbline < nbcol) A.resize(nbcol,nbcol) ;
  else A.resize(nbline,nbcol) ;

  for (i=0 ; i < nbline ; i++)
  {
    for (j=0 ; j < nbcol ; j++)
    {
      A[i][j] = (*this)[i][j] ;
    }
  }

  A.svd(sv,v);

  // Compute the highest singular value and rank of the matrix
  double maxsv = 0 ;
  for (i=0 ; i < nbcol ; i++)
    if (fabs(sv[i]) > maxsv) maxsv = fabs(sv[i]) ;

  unsigned int rank = 0 ;
  for (i=0 ; i < nbcol ; i++)
    if (fabs(sv[i]) > maxsv*svThreshold) rank++ ;

  if (rank != nbcol)
  {
    vpMatrix Ker(nbcol-rank,nbcol) ;
    unsigned int k = 0 ;
    for (j = 0 ; j < nbcol ; j++)
    {
      //if( v.col(j) in kernel and non zero )
      if ( (fabs(sv[j]) <= maxsv*svThreshold) && (std::fabs(v.getCol(j).sumSquare()) > std::numeric_limits<double>::epsilon()))
      {
        //  Ker.Row(k) = v.Col(j) ;
        for (i=0 ; i < v.getRows() ; i++)
        {
          Ker[k][i] = v[i][j];
        }
        k++;
      }
    }
    kerA = Ker ;
  }
  else
  {
    kerA.resize(0,0);
  }

  return rank ;
}

/*!
  Compute the determinant of a n-by-n matrix.

  \param method : Method used to compute the determinant. Default LU
  decomposition method is faster than the method based on Gaussian
  elimination.

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
  std:: cout << "Determinant by default method  : " <<
  A.det() << std::endl;
  std:: cout << "Determinant by LU decomposition: " <<
  A.det(vpMatrix::LU_DECOMPOSITION ) << std::endl;
}
\endcode
*/
double vpMatrix::det(vpDetMethod method) const
{
  double det_ = 0;

  if ( method == LU_DECOMPOSITION )
  {
    det_ = this->detByLU();
  }

  return (det_);
}

/*!

  Compute the exponential matrix of a square matrix.

  \return Return the exponential matrix.

*/ 
vpMatrix
vpMatrix::expm() const
{
  if(colNum != rowNum) {
    throw(vpException(vpException::dimensionError,
                      "Cannot compute the exponential of a non square (%dx%d) matrix",
                      rowNum, colNum ));
  }
  else
  {
#ifdef VISP_HAVE_GSL
    size_t size_ = rowNum * colNum;
    double *b = new double [size_];
    for (size_t i=0; i< size_; i++)
      b[i] = 0.;
    gsl_matrix_view m  = gsl_matrix_view_array(this->data, rowNum, colNum);
    gsl_matrix_view em = gsl_matrix_view_array(b, rowNum, colNum);
    gsl_linalg_exponential_ss(&m.matrix, &em.matrix, 0);
    //gsl_matrix_fprintf(stdout, &em.matrix, "%g");
    vpMatrix expA(rowNum, colNum);
    memcpy(expA.data, b, size_ * sizeof(double));

    delete [] b;
    return expA;
#else
    vpMatrix _expE(rowNum, colNum);
    vpMatrix _expD(rowNum, colNum);
    vpMatrix _expX(rowNum, colNum);
    vpMatrix _expcX(rowNum, colNum);
    vpMatrix _eye(rowNum, colNum);

    _eye.eye();
    vpMatrix exp(*this);

    //      double f;
    int e;
    double c = 0.5;
    int q = 6;
    int p = 1;

    double nA = 0;
    for (unsigned int i = 0; i < rowNum;i++)
    {
      double sum = 0;
      for (unsigned int j=0; j < colNum; j++)
      {
        sum += fabs((*this)[i][j]);
      }
      if (sum>nA||i==0)
      {
        nA=sum;
      }
    }

    /* f = */ frexp(nA, &e);
    //double s = (0 > e+1)?0:e+1;
    double s = e+1;

    double sca = 1.0 / pow(2.0,s);
    exp=sca*exp;
    _expX=*this;
    _expE=c*exp+_eye;
    _expD=-c*exp+_eye;
    for (int k=2;k<=q;k++)
    {
      c = c * ((double)(q-k+1)) / ((double)(k*(2*q-k+1)));
      _expcX=exp*_expX;
      _expX=_expcX;
      _expcX=c*_expX;
      _expE=_expE+_expcX;
      if (p) _expD=_expD+_expcX;
      else _expD=_expD- _expcX;
      p = !p;
    }
    _expX=_expD.inverseByLU();
    exp=_expX*_expE;
    for (int k=1;k<=s;k++)
    {
      _expE=exp*exp;
      exp=_expE;
    }
    return exp;
#endif
  }
}

/**************************************************************************************************************/
/**************************************************************************************************************/


//Specific functions

/*
input:: matrix M(nCols,nRows), nCols > 3, nRows > 3 , nCols == nRows.

output:: the complement matrix of the element (rowNo,colNo).
This is the matrix obtained from M after elimenating the row rowNo and column colNo 

example:
1 2 3
M = 4 5 6
7 8 9
1 3
subblock(M, 1, 1) give the matrix 7 9
*/
vpMatrix subblock(const vpMatrix &M, unsigned int col, unsigned int row)
{
  vpMatrix M_comp(M.getRows()-1,M.getCols()-1);

  for ( unsigned int i = 0 ; i < col ; i++)
  {
    for ( unsigned int j = 0 ; j < row ; j++)
      M_comp[i][j]=M[i][j];
    for ( unsigned int j = row+1 ; j < M.getRows() ; j++)
      M_comp[i][j-1]=M[i][j];
  }
  for ( unsigned int i = col+1 ; i < M.getCols(); i++)
  {
    for ( unsigned int j = 0 ; j < row ; j++)
      M_comp[i-1][j]=M[i][j];
    for ( unsigned int j = row+1 ; j < M.getRows() ; j++)
      M_comp[i-1][j-1]=M[i][j];
  }
  return M_comp;
}

/*!
   \return The condition number, the ratio of the largest singular value of the matrix to the smallest.
 */
double vpMatrix::cond() const
{
  vpMatrix v;
  vpColVector w;

  vpMatrix M;
  M = *this;

  M.svd(w, v);
  double min=w[0];
  double max=w[0];
  for(unsigned int i=0;i<M.getCols();i++)
  {
    if(min>w[i])min=w[i];
    if(max<w[i])max=w[i];
  }
  return max/min;
}

/*!
  Compute \f${\bf H} + \alpha * diag({\bf H})\f$
  \param H : input Matrix \f${\bf H}\f$. This matrix should be square.
  \param alpha : Scalar \f$\alpha\f$
  \param HLM : Resulting operation.
 */
void vpMatrix::computeHLM(const vpMatrix &H, const double &alpha, vpMatrix &HLM)
{
  if(H.getCols() != H.getRows()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot compute HLM on a non square matrix (%dx%d)",
                      H.getRows(), H.getCols() ));
  }
  HLM.resize(H.getRows(), H.getCols());

  for(unsigned int i=0;i<H.getCols();i++)
  {
    for(unsigned int j=0;j<H.getCols();j++)
    {
      HLM[i][j]=H[i][j];
      if(i==j)
        HLM[i][j]+= alpha*H[i][j];
    }
  }
}

/*!
  Compute and return the Euclidean norm \f$ ||x|| = \sqrt{ \sum{A_{ij}^2}} \f$.

  \return The Euclidean norm if the matrix is initialized, 0 otherwise.

  \sa infinityNorm()
*/
double vpMatrix::euclideanNorm() const
{
  double norm=0.0;
  for (unsigned int i=0;i<dsize;i++) {
    double x = *(data +i);
    norm += x*x;
  }

  return sqrt(norm);
}

/*!

  Compute and return the infinity norm \f$ {||x||}_{\infty} =
  max\left(\sum_{j=0}^{n}{\mid x_{ij} \mid}\right) \f$ with \f$i \in
  \{0, ..., m\}\f$ where \f$(m,n)\f$ is the matrix size.

  \return The infinity norm if the matrix is initialized, 0 otherwise.

  \sa euclideanNorm()
*/
double vpMatrix::infinityNorm() const
{
  double norm=0.0;
  for (unsigned int i=0;i<rowNum;i++){
    double x = 0;
    for (unsigned int j=0; j<colNum;j++){
      x += fabs (*(*(rowPtrs + i)+j)) ;
    }
    if (x > norm) {
      norm = x;
    }
  }
  return norm;
}

/*!
  Return the sum square of all the \f$A_{ij}\f$ elements of the matrix \f$A(m, n)\f$.

  \return The value \f$\sum A_{ij}^{2}\f$.
  */
double vpMatrix::sumSquare() const
{
  double sum_square=0.0;
  double x ;

  for (unsigned int i=0;i<rowNum;i++) {
    for(unsigned int j=0;j<colNum;j++) {
      x=rowPtrs[i][j];
      sum_square += x*x;
    }
  }

  return sum_square;
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
vpMatrix vpMatrix::stackMatrices(const vpColVector &A, const vpColVector &B)
{
  return (vpMatrix)(vpColVector::stack(A, B));
}

void vpMatrix::stackMatrices(const vpColVector &A, const vpColVector &B, vpColVector &C)
{
  vpColVector::stack(A, B, C);
}

vpMatrix vpMatrix::stackMatrices(const vpMatrix &A, const vpRowVector &B)
{
  return vpMatrix::stack(A, B);
};

void vpMatrix::stackMatrices(const vpMatrix &A, const vpRowVector &B, vpMatrix &C)
{
  vpMatrix::stack(A, B, C);
};

/*!
  \deprecated This method is deprecated. You should use getRow().

  Return the i-th row of the matrix.
  \warning notice row(1) is the 0th row.
*/
vpRowVector
vpMatrix::row(const unsigned int i)
{
  vpRowVector c(getCols()) ;

  for (unsigned int j =0 ; j < getCols() ; j++)  c[j] = (*this)[i-1][j] ;
  return c ;
}

/*!
  \deprecated This method is deprecated. You should use getCol().

  Return the j-th columns of the matrix.
  \warning notice column(1) is the 0-th column.
  \param j : Index of the column to extract.
*/
vpColVector
vpMatrix::column(const unsigned int j)
{
  vpColVector c(getRows()) ;

  for (unsigned int i =0 ; i < getRows() ; i++)     c[i] = (*this)[i][j-1] ;
  return c ;
}

/*!
  \deprecated You should rather use diag(const double &)

  Set the matrix diagonal elements to \e val.
  More generally set M[i][i] = val.
*/
void
vpMatrix::setIdentity(const double & val)
{
  for (unsigned int i=0;i<rowNum;i++)
    for (unsigned int j=0;j<colNum;j++)
      if (i==j) (*this)[i][j] = val ;
      else      (*this)[i][j] = 0;
}

#endif //#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

