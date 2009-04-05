/****************************************************************************
 *
 * $Id: vpMatrix.cpp,v 1.48 2009-01-08 16:17:07 fspindle Exp $
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

/*
  \class vpMatrix

  \brief Provide simple Matrices computation

  \author Eric Marchand   (Eric.Marchand@irisa.fr) Irisa / Inria Rennes
*/


#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <vector>
#include <sstream>
#include <algorithm>
#include <assert.h>

#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpTranslationVector.h>


// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>

#define DEBUG_LEVEL1 0
#define DEBUG_LEVEL2 0


//Prototypes of specific functions
vpMatrix subblock(const vpMatrix &, int, int);

/*!
  \brief initialization of the object matrix.
  Number of columns and rows are zero.
*/

void
vpMatrix::init()
{

  rowNum = 0  ;
  colNum = 0 ;

  data = NULL ;
  rowPtrs = NULL ;

  dsize = 0 ;
  trsize =0 ;
}

/*!
  \brief basic constructor of the matrix class
  Construction of the object matrix.
  Number of columns and rows are zero.
*/
vpMatrix::vpMatrix()
{
  init() ;
}


/*!
  Constructor.

  Initialize a matrix with 0.

  \param r : Matrix number of rows.
  \param c : Matrix number of columns.
*/
vpMatrix::vpMatrix(int r,int c)
{
  init() ;
  resize(r, c);
}

/*!
  \brief submatrix constructor
*/
vpMatrix::vpMatrix(const vpMatrix &m,
		   int r,
		   int c, int nrows, int ncols)
{
  init() ;

  if ( (r<0) || (c<0) )
  {
    vpERROR_TRACE("\n\t\t Illegal subMatrix operation") ;
    throw(vpMatrixException(vpMatrixException::subMatrixError,
			    "\n\t\t Illegal subMatrix operation")) ;
  }

  if (((r + nrows) > m.rowNum) || ((c + ncols) > m.colNum))
  {
    vpERROR_TRACE("\n\t\t SubvpMatrix larger than vpMatrix") ;
    throw(vpMatrixException(vpMatrixException::subMatrixError,
			    "\n\t\t SubvpMatrix larger than vpMatrix")) ;
  }

  init(m,r,c,nrows,ncols);
}

//! copie constructor
vpMatrix::vpMatrix(const vpMatrix& m)
{
  init() ;

  resize(m.rowNum,m.colNum);

  memcpy(data,m.data,rowNum*colNum*sizeof(double)) ;

  // MODIF EM 13/6/03
  /*for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
    rowPtrs[i][j] = m.rowPtrs[i][j];
    }
    }
  */
}


/*!
  \brief set the size of the matrix, initialization with a zero matrix

  \param nrows : number of rows
  \param ncols : number of column
  \param flagNullify : if true, then the matrix is re-initialized to 0
  afet resize. If false, the initial values from the common part of the
  matrix (comon part between old and new version of the matrix) are kept.
  Default value is true.

  \return OK or MEMORY_FAULT if memory cannot be allocated
*/

void vpMatrix::resize(const int nrows, const int ncols, const bool flagNullify)
{

  if ((nrows == rowNum) && (ncols == colNum))
  {
    if (flagNullify)
      { memset(this->data,0,this->dsize*sizeof(double)) ;}
  }
  else
  {
    const bool recopyNeeded = (ncols != this ->colNum);
    double * copyTmp = NULL;
    int rowTmp = 0, colTmp=0;

    vpDEBUG_TRACE (25, "Recopy case per case is required iff number of "
		 "cols has changed (structure of double array is not "
		 "the same in this case.");
    if (recopyNeeded)
      {
	copyTmp = new double[this->dsize];
	memcpy (copyTmp, this ->data, sizeof(double)*this->dsize);
	rowTmp=this->rowNum; colTmp=this->colNum;
      }

    vpDEBUG_TRACE (25, "Reallocation of this->data array.");
    this->dsize = nrows*ncols;
    this->data = (double*)realloc(this->data, this->dsize*sizeof(double));
    if ((NULL == this->data) && (0 != this->dsize))
    {
      vpERROR_TRACE("\n\t\tMemory allocation error when allocating data") ;
      throw(vpException(vpException::memoryAllocationError,
			"\n\t\t Memory allocation error when "
			"allocating data")) ;
    }

    vpDEBUG_TRACE (25, "Reallocation of this->trsize array.");
    this->trsize = nrows;
    this->rowPtrs = (double**)realloc (this->rowPtrs, this->trsize*sizeof(double*));
    if ((NULL == this->rowPtrs) && (0 != this->dsize))
    {
      vpERROR_TRACE("\n\t\tMemory allocation error when allocating rowPtrs") ;
      throw(vpException(vpException::memoryAllocationError,
			"\n\t\t Memory allocation error when "
			"allocating rowPtrs")) ;
    }

    vpDEBUG_TRACE (25, "Recomputation this->trsize array values.");
    {
      double **t= rowPtrs;
      for (int i=0; i<dsize; i+=ncols)  { *t++ = this->data + i; }
    }

    this->rowNum = nrows; this->colNum = ncols;

    vpDEBUG_TRACE (25, "Recopy of this->data array values or nullify.");
    if (flagNullify)
      { memset(this->data,0,this->dsize*sizeof(double)) ;}
    else
      {
	if (recopyNeeded)
	  {
	    vpDEBUG_TRACE (25, "Recopy...");
	    const int minRow = (this->rowNum<rowTmp)?this->rowNum:rowTmp;
	    const int minCol = (this->colNum<colTmp)?this->colNum:colTmp;
	    for (int i=0; i<this->rowNum; ++i)
	      for (int j=0; j<this->colNum; ++j)
		{
		  if ((minRow > i) && (minCol > j))
		    {
		      (*this)[i][j] = copyTmp [i*colTmp+j];
		      vpCDEBUG (25) << i << "x" << j << "<- " << i*colTmp+j
				  << "=" << copyTmp [i*colTmp+j] << std::endl;
		    }
		  else {(*this)[i][j] = 0;}
		}
	  }
	else { vpDEBUG_TRACE (25,"Nothing to do: already done by realloc.");}
      }

    if (copyTmp != NULL) delete [] copyTmp;
  }

}


void
vpMatrix::init(const vpMatrix &m,int r, int c, int nrows, int ncols)
{
  try {
    resize(nrows, ncols) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    std::cout << me << std::endl ;
    throw ;
  }

  int rnrows = r+nrows ;
  int cncols = c+ncols ;
  for (int i=r ; i < rnrows; i++)
    for (int j=c ; j < cncols; j++)
      (*this)[i-r][j-c] = m[i][j] ;
}

/*!
  \brief Destruction of the matrix  (Memory de-allocation)
*/
void
vpMatrix::kill()
{
  if (data != NULL )
  {
    free(data);
    data=NULL;
  }

  if (rowPtrs!=NULL)
  {
    free(rowPtrs);
    rowPtrs=NULL ;
  }
}
/*!
  \brief Destructor (Memory de-allocation)
*/
vpMatrix::~vpMatrix()
{
  kill() ;
}


/*!
  \brief Copy operator.
  Allow operation such as A = B

  \param B : matrix to be copied.
*/
vpMatrix &
vpMatrix::operator=(const vpMatrix &B)
{
  try {
    resize(B.rowNum, B.colNum) ;
    // suppress by em 5/12/06
    //    *this = 0;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    std::cout << me << std::endl ;
    throw ;
  }

  memcpy(data,B.data,dsize*sizeof(double)) ;

  return *this;
}

//! set all the element of the matrix A to x
vpMatrix &
vpMatrix::operator=(double x)
{
  for (int i=0; i<dsize; i++)
  {
    *(data+i) = x;
  }
  return *this;
}

/*!
  \brief Assigment from an array of double
*/
vpMatrix &
vpMatrix::operator<<( double *x )
{

  for (int i=0; i<rowNum; i++) {
    for (int j=0; j<colNum; j++) {
      rowPtrs[i][j] = *x++;
    }
  }
  return *this;
}


//---------------------------------
// Matrix operations.
//---------------------------------

/*!
	operation C = A * B. 
	
	The Result is placed in the third parameter C and not returned.
	A new matrix won't be allocated for every use of the function 
	(Speed gain if used many times with the same result matrix size).

	\sa operator*()
*/
void vpMatrix::mult2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
	try 
	{
		if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum)) C.resize(A.rowNum,B.colNum);
	}
	catch(vpException me)
	{
		vpERROR_TRACE("Error caught") ;
		std::cout << me << std::endl ;
		throw ;
	}

	if (A.colNum != B.rowNum)
	{
		vpERROR_TRACE("\n\t\tvpMatrix mismatch in vpMatrix/vpMatrix multiply") ;
		throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\tvpMatrix mismatch in "
			    "vpMatrix/vpMatrix multiply")) ;
	}

	// 5/12/06 some "very" simple optimization to avoid indexation
	int BcolNum = B.colNum;
	int BrowNum = B.rowNum;
	int i,j,k;
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
	operation C = A * B (A is unchanged).
	\sa mult2Matrices() to avoid matrix allocation for each use.
*/
vpMatrix vpMatrix::operator*(const vpMatrix &B) const
{
  vpMatrix C;

  vpMatrix::mult2Matrices(*this,B,C);

  return C;
}

/*!
	operation C = A + B. 
	
	The Result is placed in the third parameter C and not returned.
	A new matrix won't be allocated for every use of the function 
	(Speed gain if used many times with the same result matrix size).

	\sa operator+()
*/
void vpMatrix::add2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{  
	try 
	{
		if ((A.rowNum != C.rowNum) || (B.colNum != C.colNum)) C.resize(A.rowNum,B.colNum);
	}
	catch(vpException me)
	{
		vpERROR_TRACE("Error caught") ;
		std::cout << me << std::endl ;
		throw ;
	}

	if ((A.colNum != B.getCols())||(A.rowNum != B.getRows()))
	{
		vpERROR_TRACE("\n\t\t vpMatrix mismatch in vpMatrix/vpMatrix addition") ;
		throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t vpMatrix mismatch in "
			    "vpMatrix/vpMatrix addition")) ;
	}

	int i;
	// MODIF EM 16/6/03
	/*int j;
    for (i=0;i<rowNum;i++)
    for(j=0;j<colNum;j++)
    {
    v.rowPtrs[i][j] = B.rowPtrs[i][j]+rowPtrs[i][j];
    }
	*/

	for (i=0;i<A.dsize;i++)
	{
		*(C.data + i) = *(B.data + i) + *(A.data + i) ;
	}
}

/*!
	operation C = A + B (A is unchanged).
	\sa add2Matrices() to avoid matrix allocation for each use.
*/
vpMatrix vpMatrix::operator+(const vpMatrix &B) const
{
	vpMatrix C;
	vpMatrix::add2Matrices(*this,B,C);
	return C;
}

/*!
	operation C = A - B. 
	
	The Result is placed in the third parameter C and not returned.
	A new matrix won't be allocated for every use of the function 
	(Speed gain if used many times with the same result matrix size).

	\sa operator-()
*/
void vpMatrix::sub2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
	try 
	{
		if ((A.rowNum != C.rowNum) || (A.colNum != C.colNum)) C.resize(A.rowNum,A.colNum);
	}
	catch(vpException me)
	{
		vpERROR_TRACE("Error caught") ;
		std::cout << me << std::endl ;
		throw ;
	}

	if ( (A.colNum != B.getCols())||(A.rowNum != B.getRows()))
	{
		vpERROR_TRACE("\n\t\t vpMatrix mismatch in vpMatrix/vpMatrix substraction") ;
		throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t vpMatrix mismatch in "
			    "vpMatrix/vpMatrix substraction")) ;
	}

	int i;
	// MODIF EM 16/6/03
	for (i=0;i<A.dsize;i++)
	{
		*(C.data + i) = *(A.data + i) - *(B.data + i) ;
	}
}

/*!
	operation C = A - B (A is unchanged).
	\sa sub2Matrices() to avoid matrix allocation for each use.
*/
vpMatrix vpMatrix::operator-(const vpMatrix &B) const
{
	vpMatrix C;
	vpMatrix::sub2Matrices(*this,B,C);
	return C;
}

//! operation A = A + B

vpMatrix &vpMatrix::operator+=(const vpMatrix &B)
{
  if ( (colNum != B.getCols())||(rowNum != B.getRows()))
  {
    vpERROR_TRACE("\n\t\t vpMatrix mismatch in vpMatrix +=  addition") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t vpMatrix mismatch in "
			    "vpMatrix += addition")) ;

  }

  int i;

  // MODIF EM 16/6/03
  for (i=0;i<dsize;i++)
  {
    *(data + i) += *(B.data + i) ;
  }
  /* int j;
     for (i=0;i<rowNum;i++)
     for(j=0;j<colNum;j++)
     rowPtrs[i][j] += B.rowPtrs[i][j];
  */
  return *this;
}

//! operation A = A - B

vpMatrix & vpMatrix::operator-=(const vpMatrix &B)
{
  if ( (colNum != B.getCols())||(rowNum != B.getRows()))
  {
    vpERROR_TRACE("\n\t\t vpMatrix mismatch in vpMatrix -= substraction") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t vpMatrix mismatch in "
			    "vpMatrix -= substraction")) ;

  }




  int i;
  // MODIF EM 16/6/03
  for (i=0;i<dsize;i++)
  {
    *(data + i) -= *(B.data + i) ;
  }
  /*
    int j; for (i=0;i<rowNum;i++)
    for(j=0;j<colNum;j++)
    rowPtrs[i][j] -= B.rowPtrs[i][j];
  */
  return *this;
}

/*!
	operation C = -A. 
	
	The Result is placed in the second parameter C and not returned.
	A new matrix won't be allocated for every use of the function 
	(Speed gain if used many times with the same result matrix size).

	\sa operator-(void)
*/
void vpMatrix::negateMatrix(const vpMatrix &A, vpMatrix &C)
{
	try 
	{
		if ((A.rowNum != C.rowNum) || (A.colNum != C.colNum)) C.resize(A.rowNum,A.colNum);
	}
	catch(vpException me)
	{
		vpERROR_TRACE("Error caught") ;
		std::cout << me << std::endl ;
		throw ;
	}

	for (int i=0;i<A.dsize;i++)
	{
		*(C.data + i) = -*(A.data + i) ;
	}
}

/*!
	operation C = -A (A is unchanged).
	\sa negateMatrix() to avoid matrix allocation for each use.
*/
vpMatrix vpMatrix::operator-() const //negate
{
	vpMatrix C;
	vpMatrix::negateMatrix(*this,C);
	return C;
}

//!return sum of the Aij^2 (for all i, for all j)
double
vpMatrix::sumSquare() const
{
  double sum=0.0;
  double x ;


  double *d = data ;
  double *n = data+dsize ;
  while (d < n )
  {
    x = *d++ ;
    sum += x*x ;
  }
/*
  for (int i=0;i<dsize;i++)
  {
    x = *(data + i) ;
    sum += x*x ;
  }

    for (int i=0; i<rowNum; i++)
    for (int j=0; j<colNum; j++)
    sum += rowPtrs[i][j]*rowPtrs[i][j];
  */

  return sum;
}

//---------------------------------
// Matrix/vector operations.
//---------------------------------

/*!
	operation c = A * b (c and b are vectors). 
	
	The Result is placed in the second parameter C and not returned.
	A new matrix won't be allocated for every use of the function 
	(Speed gain if used many times with the same result matrix size).

	\sa operator*(const vpColVector &b) const
*/
void vpMatrix::multMatrixVector(const vpMatrix &A, const vpColVector &b, vpColVector &c)
{
	if (A.colNum != b.getRows())
	{
		vpERROR_TRACE("vpMatrix mismatch in vpMatrix/vector multiply") ;
		throw(vpMatrixException::incorrectMatrixSizeError) ;
	}

	try 
	{
		if (A.rowNum != c.rowNum) c.resize(A.rowNum);
	}
	catch(vpException me)
	{
		vpERROR_TRACE("Error caught") ;
		std::cout << me << std::endl ;
		throw ;
	}

	c = 0.0;
	for (int j=0;j<A.colNum;j++) 
	{
		double bj = b[j] ; // optimization em 5/12/2006
		for (int i=0;i<A.rowNum;i++) 
		{
			c[i]+=A.rowPtrs[i][j] * bj;
		}
    }
}

/*!
	operation c = A * b (A is unchanged, c and b are vectors).
	\sa multMatrixVector() to avoid matrix allocation for each use.
*/
vpColVector
vpMatrix::operator*(const vpColVector &b) const
{
	vpColVector c;
	vpMatrix::multMatrixVector(*this,b,c);
	return c;
}

//! operation c = A * b (A is unchanged, c and b are translation vectors)
vpTranslationVector
vpMatrix::operator*(const vpTranslationVector &b) const
{
  vpTranslationVector c;

  if (rowNum != 3 || colNum != 3)
  {
    vpERROR_TRACE("vpMatrix mismatch in vpMatrix::operator*(const vpTranslationVector)") ;
    throw(vpMatrixException::incorrectMatrixSizeError) ;
  }

  for (int j=0;j<3;j++) c[j]=0 ;

  for (int j=0;j<3;j++) {
    {
      double bj = b[j] ; // optimization em 5/12/2006
      for (int i=0;i<3;i++) {
	c[i]+=rowPtrs[i][j] * bj;
      }
    }
  }
  return c ;
}

//---------------------------------
// Matrix/real operations.
//---------------------------------

/*!
  \relates vpMatrix
  \brief multiplication by a scalar  Cij = x*Bij
*/
vpMatrix operator*(const double &x,const vpMatrix &B)
{
  // Modif EM 13/6/03
  vpMatrix v ;

  try {
    v.resize(B.getRows(),B.getCols());
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    std::cout << me << std::endl ;
    throw ;
  }

  int Brow = B.getRows() ;
  int Bcol = B.getCols() ;
  for (int i=0;i<Brow; i++)
    {
      double *vi = v[i] ;
      double *Bi = B[i] ;
      for (int j=0 ; j < Bcol;j++)
	vi[j] = Bi[j]*x;
    }
  return v ;
}

//! Cij = Aij * x (A is unchanged)
vpMatrix vpMatrix::operator*(double x) const
{
  vpMatrix v;

  try {
    v.resize(rowNum,colNum);
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    std::cout << me << std::endl ;
    throw ;
  }



  for (int i=0;i<dsize;i++)
    *(v.data+i) = *(data+i)*x;

  // Modif EM 13/6/03
  /*
    int i;
    int j;
    for (i=0;i<rowNum;i++)
    for(j=0;j<colNum;j++)  v.rowPtrs[i][j] = rowPtrs[i][j]*x;
  */
  return v;
}

//! Cij = Aij / x (A is unchanged)
vpMatrix  vpMatrix::operator/(double x) const
{
  vpMatrix v;

  try {
    v.resize(rowNum,colNum);
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    vpCERROR << me << std::endl ;
    throw ;
  }

  if (x == 0)
    throw vpMatrixException(vpMatrixException::divideByZeroError, "Divide by zero in method /(double x)");

  double  xinv = 1/x ;

  for (int i=0;i<dsize;i++)
    *(v.data+i) = *(data+i)*xinv ;

  return v;
}


//! Add x to all the element of the matrix : Aij = Aij + x
vpMatrix & vpMatrix::operator+=(double x)
{


  for (int i=0;i<dsize;i++)
    *(data+i) += x;

  return *this;
}


//! Substract x to all the element of the matrix : Aij = Aij - x
vpMatrix & vpMatrix::operator-=(double x)
{


  for (int i=0;i<dsize;i++)
    *(data+i) -= x;
  /*
    int i;int j;
    for (i=0;i<rowNum;i++)
    for(j=0;j<colNum;j++)
    rowPtrs[i][j] -= x;
  */
  return *this;
}

//! Multiply  all the element of the matrix by x : Aij = Aij * x
vpMatrix & vpMatrix::operator*=(double x)
{


  for (int i=0;i<dsize;i++)
    *(data+i) *= x;
  /*
    int i;int j;
    for (i=0;i<rowNum;i++)
    for(j=0;j<colNum;j++)  rowPtrs[i][j] *= x;
  */
  return *this;
}

//! Divide  all the element of the matrix by x : Aij = Aij / x
vpMatrix & vpMatrix::operator/=(double x)
{
  if (x == 0)
    throw vpMatrixException(vpMatrixException::divideByZeroError, "Divide by zero in method /=(double x)");

  double xinv = 1/x ;
  for (int i=0;i<dsize;i++)
    *(data+i) *= xinv;
  /*
    int i;int j;
    for (i=0;i<rowNum;i++)
    for(j=0;j<colNum;j++)  rowPtrs[i][j] /= x;
  */
  return *this;
}


//----------------------------------------------------------------
// Matrix Operation
//----------------------------------------------------------------

/*!
  \brief set the matrix to identity
*/
void
vpMatrix::setIdentity()
{

  if (rowNum != colNum)
  {
    vpERROR_TRACE("non square matrix") ;
    throw(vpMatrixException(vpMatrixException::matrixError)) ;
  }



  int i,j;
  for (i=0;i<rowNum;i++)
    for (j=0;j<colNum;j++)
      if (i==j) (*this)[i][j] = 1 ; else (*this)[i][j] = 0;
}

/*!
  \brief set the matrix to identity

  eye(n) is an n-by-n matrix with ones on the diagonal and zeros
  elsewhere

  \sa eye(n) is also a matlab function
*/
void
vpMatrix::eye(int n)
{
  try {
    eye(n,n) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    vpCERROR << me << std::endl ;
    throw ;
  }
}
/*!
  \brief eye(m,n) is an m-by-n matrix with ones on the diagonal and zeros
  elsewhere

  \sa eye(m,n) is also a matlab function
*/
void
vpMatrix::eye(int m, int n)
{
  try {
     resize(m,n) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    vpCERROR << me << std::endl ;
    throw ;
  }


  for (int i=0; i<rowNum; i++)
    for (int j=0; j<colNum; j++)
      if (i == j) (*this)[i][j] = 1;
      else        (*this)[i][j] = 0;

}


/*!
  \brief Transpose the matrix C = A^T
  \return  A^T
*/
vpMatrix vpMatrix::t() const
{
  vpMatrix At ;

  try {
    At.resize(colNum,rowNum);
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    vpCERROR << me << std::endl ;
    throw ;
  }



  int i,j;
  for (i=0;i<rowNum;i++)
  {
    double *coli = (*this)[i] ;
    for (j=0;j<colNum;j++)
      At[j][i] = coli[j];
  }
  return At;
}


/*!
 \brief Compute the AtA operation B = A^T*A
  The Result is placed in the parameter B and not returned.
   A new matrix won't be allocated for every use of the function
   (Speed gain if used many times with the same result matrix size).

   \sa AtA()
*/
void vpMatrix::AtA(vpMatrix &B) const
{
   try {
       if ((B.rowNum != colNum) || (B.colNum != colNum)) B.resize(colNum,colNum);
   }
   catch(vpException me)
   {
       vpERROR_TRACE("Error caught") ;
       vpCERROR << me << std::endl ;
       throw ;
   }

   int i,j,k;
   double s;
   double *ptr;
   double *Bi;
        for (i=0;i<colNum;i++)
   {
       Bi = B[i] ;
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
 \brief Compute the AtA operation B = A^T*A
 \return  A^T*A
 \sa AtA(vpMatrix &) const
*/
vpMatrix vpMatrix::AtA() const
{
 vpMatrix B;

 AtA(B);

 return B;
}

/*!
  Solve a linear system \f$ A X = B \f$ using Singular Value Decomposition (SVD).

  Non destructive wrt. A and B.

  \param b : Vector\f$ B \f$.

  \param x : Vector \f$ X \f$.

  Here an example:
  \code
#include <visp/vpColVector.h>
#include <visp/vpMatrix.h>

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

  \sa SVDsolve()
*/
void
vpMatrix::solveBySVD(const vpColVector& b, vpColVector& x) const
{
  x = pseudoInverse(1e-6)*b ;
}


/*!
  Solve a linear system \f$ A X = B \f$ using Singular Value Decomposition (SVD).

  Non destructive wrt. A and B.

  \param B : Vector\f$ B \f$.

  \return Vector \f$ X \f$.

  Here an example:
  \code
#include <visp/vpColVector.h>
#include <visp/vpMatrix.h>

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

  X = A.SVDsolve(B);
  // Obtained values of X
  // X[0] = 0.2468; 
  // X[1] = 0.120782; 
  // X[2] = 0.468587; 

  std::cout << "X:\n" << X << std::endl;
}
  \endcode

  \sa solveBySVD()
*/
vpColVector vpMatrix::SVDsolve(const vpColVector& B) const
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
#include <visp/vpColVector.h>
#include <visp/vpMatrix.h>

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
  if (DEBUG_LEVEL1 == 0) /* no verification */
  {
    w.resize( this->getCols() );
    v.resize( this->getCols(), this->getCols() );
#ifdef VISP_HAVE_GSL  /* be careful of the copy below */
    svdGsl(w,v) ;
#else
    svdNr(w,v) ;
#endif
  //svdNr(w,v) ;
  }
  else  /* verification of the SVD */
  {
    int pb = 0;
    int i,j,k,nrows,ncols;
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
}
/*!
  \brief Compute the pseudo inverse of the matrix Ap = A^+
  \param Ap : The pseudo inverse \f$ A^+ \f$.
  \param svThreshold : Threshold used to test the singular values.
  \return Return the rank of the matrix A
*/

int
vpMatrix::pseudoInverse(vpMatrix &Ap, double svThreshold) const
{
  vpColVector sv ;
  return   pseudoInverse(Ap, sv, svThreshold) ;
}

/*!
  \brief Compute and return the pseudo inverse of a n-by-m matrix : \f$ A^+ \f$
  \param svThreshold : Threshold used to test the singular values.

  \return Pseudo inverse of the matrix.

  Here an example to compute the inverse of a n-by-n matrix. If the
  matrix is n-by-n it is also possible to use inverseByLU().

  \code
#include <visp/vpMatrix.h>

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
  \brief Compute the pseudo inverse of the matrix Ap = A^+
  \param Ap : The pseudo inverse \f$ A^+ \f$.
  \param sv : Singular values.
  \param svThreshold : Threshold used to test the singular values.
  \return Return the rank of the matrix A
*/
int
vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold) const
{
  vpMatrix imA, imAt ;
  return pseudoInverse(Ap, sv, svThreshold, imA, imAt) ;
}

/*!
  \brief Compute the pseudo inverse of the matrix Ap = A^+ along with Ker A, Ker A^T, Im A and Im A^T

  Pseudo Inverse, Kernel and Image are computed using the SVD decomposition

  A is an m x n matrix,
  if m >=n the svd works on A other wise it works on A^T

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
int
vpMatrix::pseudoInverse(vpMatrix &Ap,
			vpColVector &sv, double svThreshold,
			vpMatrix &imA,
			vpMatrix &imAt) const
{

  int i, j, k ;

  int nrows, ncols;
  int nrows_orig = getRows() ;
  int ncols_orig = getCols() ;
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

  int rank = 0 ;
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

  if (DEBUG_LEVEL1)
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


  // std::cout << v << std::endl ;
  return rank ;
}



/*!
  \brief Compute the pseudo inverse of the matrix Ap = A^+ along with Ker A, Ker A^T, Im A and Im A^T

  Pseudo Inverse, Kernel and Image are computed using the SVD decomposition

  A is an m x n matrix,
  if m >=n the svd works on A other wise it works on A^T

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
int vpMatrix::pseudoInverse(vpMatrix &Ap,
		    vpColVector &sv, double svThreshold,
		    vpMatrix &imA,
		    vpMatrix &imAt,
		    vpMatrix &kerA) const
{

  int i, j, k ;

  int nrows, ncols;
  int nrows_orig = getRows() ;
  int ncols_orig = getCols() ;
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

  int rank = 0 ;
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
    if ( cons.row(j+1).sumSquare() != 0)
    {
      for (i = 0; i < cons.getCols(); i++)
        Ker[k][i] = cons[j][i];

      k++;
    }
  }
  kerA = Ker;

  if (DEBUG_LEVEL1)
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


  // std::cout << v << std::endl ;
  return rank ;
}


/*!
  \brief  Return the ith rows of the matrix
  \warning notice row(1) is the 0th row.
*/

vpRowVector
vpMatrix::row(const int j)
{
  vpRowVector c(getCols()) ;

  for (int i =0 ; i < getCols() ; i++)  c[i] = (*this)[j-1][i] ;
  return c ;
}


/*!
  \brief  Return the ith columns of the matrix
  \warning notice column(1) is the 0th column.
*/

vpColVector
vpMatrix::column(const int j)
{
  vpColVector c(getRows()) ;

  for (int i =0 ; i < getRows() ; i++)     c[i] = (*this)[i][j-1] ;
  return c ;
}




/*!
  \brief Stack matrices. "Stack" two matrices  C = [ A B ]^T

  \f$ C = \left( \begin{array}{c} A \\ B \end{array}\right)    \f$

  \param A : Upper matrix.
  \param B : Lower matrix.
  \return Stacked matrix C = [ A B ]^T

  \warning A and B must have the same number of column.
*/
vpMatrix
vpMatrix::stackMatrices(const vpMatrix &A, const vpMatrix &B)
{
  vpMatrix C ;

  try{
  	stackMatrices(A,B, C) ;
  }
  catch(vpMatrixException me)
  {
    vpCERROR << me << std::endl;
    throw ;
  }

  return C ;
}

/*!
  \relates vpMatrix
  \brief stackMatrices. "stack" two matrices  C = [ A B ]^T

  \f$ C = \left( \begin{array}{c} A \\ B \end{array}\right)    \f$

  \param  A : Upper matrix.
  \param  B : Lower matrix.
  \param  C : Stacked matrix C = [ A B ]^T

  \warning A and B must have the same number of column
*/
void
vpMatrix::stackMatrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
  int nra = A.getRows() ;
  int nrb = B.getRows() ;

  if (nra !=0)
    if (A.getCols() != B.getCols())
    {
      vpERROR_TRACE("\n\t\t incorrect matrices size") ;
      throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			      "\n\t\t incorrect matrices size")) ;
    }

  try {
    C.resize(nra+nrb,B.getCols()  ) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    vpCERROR << me << std::endl ;
    throw ;
  }

  int i,j ;
  for (i=0 ; i < nra ; i++)
    for (j=0 ; j < A.getCols() ; j++)
      C[i][j] = A[i][j] ;


  for (i=0 ; i < nrb ; i++)
    for (j=0 ; j < B.getCols() ; j++)
    {
      C[i+nra][j] = B[i][j] ;

    }


}
/*!
  \brief Juxtapose matrices. "juxtapos" two matrices  C = [ A B ]

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
  catch(vpMatrixException me)
  {
    vpCERROR << me << std::endl ;
    throw ;
  }

  return C ;
}

/*!
  \relates vpMatrix
  \brief juxtaposeMatrices. "juxtapose" two matrices  C = [ A B ]

  \f$ C = \left( \begin{array}{cc} A & B \end{array}\right)    \f$

  \param A : Left matrix.
  \param B : Right matrix.
  \param C : Juxtaposed matrix C = [ A B ]

  \warning A and B must have the same number of column
*/
void
vpMatrix::juxtaposeMatrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
{
  int nca = A.getCols() ;
  int ncb = B.getCols() ;

  if (nca !=0)
    if (A.getRows() != B.getRows())
    {
      vpERROR_TRACE("\n\t\t incorrect matrices size") ;
      throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			      "\n\t\t incorrect matrices size")) ;
    }

  try {
    C.resize(B.getRows(),nca+ncb) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    vpCERROR << me << std::endl ;
    throw ;
  }

  int i,j ;
  for (i=0 ; i < C.getRows(); i++)
    for (j=0 ; j < nca ; j++)
      C[i][j] = A[i][j] ;


  for (i=0 ; i < C.getRows() ; i++)
    for (j=0 ; j < ncb ; j++)
    {
      C[i][nca+j] = B[i][j] ;
    }
}

/*!

  Create a diagonal matrix with the element of a vector.

  \param  A : Vector which element will be put in the diagonal.

  \sa createDiagonalMatrix()

  \code
#include <iostream>

#include <visp/vpColVector.h>
#include <visp/vpMatrix.h>

int main()
{
  vpMatrix A;
  vpColVector v(3);

  v[0] = 1;
  v[1] = 2;
  v[2] = 3;

  A.diag(v);

  std::cout << "A:\n" << A << std::endl;

  // A is now equal to:
  // 1 0 0
  // 0 2 0
  // 0 0 3
}
  \endcode
*/

void
vpMatrix::diag(const vpColVector &A)
{
  int rows = A.getRows() ;
  try {
    this->resize(rows,rows) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    vpCERROR << me << std::endl ;
    throw ;
  }
  (*this) = 0 ;
  for (int i=0 ; i< rows ; i++ )
    (* this)[i][i] = A[i] ;
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
  int rows = A.getRows() ;
  try {
    DA.resize(rows,rows) ;
  }
  catch(vpException me)
  {
    vpERROR_TRACE("Error caught") ;
    vpCERROR << me << std::endl ;
    throw ;
  }
  DA =0 ;
  for (int i=0 ; i< rows ; i++ )
    DA[i][i] = A[i] ;
}

//--------------------------------------------------------------------
// Output
//--------------------------------------------------------------------


/*!
  \brief std::cout a matrix
*/
std::ostream &operator <<(std::ostream &s,const vpMatrix &m)
{
  s.precision(10) ;
  for (int i=0;i<m.getRows();i++) {
    for (int j=0;j<m.getCols();j++){
      s <<  m[i][j] << "  ";
    }
    s <<std::endl;
  }

  return s;
}

/*!

  Pretty print a matrix. The data are tabulated.
  The common widths before and after the decimal point
  are set with respect to the parameter maxlen.

  \param s
    Stream used for the printing.

  \param length
    The suggested width of each matrix element.
    The actual width grows in order to accomodate the whole integral part,
    and shrinks if the whole extent is not needed for all the numbers.
  \param intro
    The introduction which is printed before the matrix.
    Can be set to zero (or omitted), in which case
    the introduction is not printed.

    \return
    Returns the common total width for all matrix elements

  \sa std::ostream &operator <<(ostream &s,const vpMatrix &m)
*/
int
vpMatrix::print(std::ostream& s, unsigned length, char const* intro)
{
  typedef std::string::size_type size_type;

  int m = getRows();
  int n = getCols();

  std::vector<std::string> values(m*n);
  std::ostringstream oss;
  std::ostringstream ossFixed;
  // ossFixed <<std::fixed;
  ossFixed.setf ( std::ios::fixed, std::ios::floatfield );

  size_type maxBefore=0;  // the length of the integral part
  size_type maxAfter=0;   // number of decimals plus
                          // one place for the decimal point
  for (int i=0;i<m;++i) {
    for (int j=0;j<n;++j){
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
        maxBefore=std::max(maxBefore, thislen);
        // maxAfter remains the same
      } else{
        maxBefore=std::max(maxBefore, p);
        maxAfter=std::max(maxAfter, thislen-p-1);
      }
    }
  }

  size_type totalLength=length;
  // increase totalLength according to maxBefore
  totalLength=std::max(totalLength,maxBefore);
  // decrease maxAfter according to totalLength
  maxAfter=std::min(maxAfter, totalLength-maxBefore);
  if (maxAfter==1) maxAfter=0;

  // the following line is useful for debugging
  std::cerr <<totalLength <<" " <<maxBefore <<" " <<maxAfter <<"\n";

  if (intro) s <<intro;
  s <<"["<<m<<","<<n<<"]=\n";

  for (int i=0;i<m;i++) {
    s <<"  ";
    for (int j=0;j<n;j++){
      size_type p=values[i*n+j].find('.');
      s.setf(std::ios::right, std::ios::adjustfield);
      s.width(maxBefore);
      s <<values[i*n+j].substr(0,p).c_str();

      if (maxAfter>0){
        s.setf(std::ios::left, std::ios::adjustfield);
        if (p!=std::string::npos){
          s.width(maxAfter);
          s <<values[i*n+j].substr(p,maxAfter).c_str();
        } else{
          assert(maxAfter>1);
          s.width(maxAfter);
          s <<".0";
        }
      }

      s <<' ';
    }
    s <<std::endl;
  }

  return (int)(maxBefore+maxAfter);
}


/*!
  \brief Print using matlab syntax, to be put in matlab later.

  Print using the following form:
     [ a,b,c;
       d,e,f;
       g,h,i]
*/
std::ostream & vpMatrix::
matlabPrint(std::ostream & os)
{

  int i,j;

  os << "[ ";
  for (i=0; i < this->getRows(); ++ i)
    {
      for (j=0; j < this ->getCols(); ++ j)
	{
	  os <<  (*this)[i][j] << ", ";
	}
      if (this ->getRows() != i+1) { os << ";" << std::endl; }
      else { os << "]" << std::endl; }
    }
  return os;
};

/*!
  \brief Print to be used as part of a C++ code later.

  Print under the following form:
    vpMatrix A(6,4);
    A[0][0]  = 1.4;
    A[0][1] = 0.6; ...

  \param os: the stream to be printed in.
  \param matrixName: name of the matrix, "A" by default, to be used for
  the line vpMatrix A(6,7) (see example).
  \param octet: if false, print using double, if true, print byte per byte
  each bytes of the double array.
*/
std::ostream & vpMatrix::
cppPrint(std::ostream & os, const char * matrixName, bool octet)
{

  int i,j;
  const char defaultName [] = "A";
  if (NULL == matrixName)
    {
      matrixName = defaultName;
    }
  os << "vpMatrix " << defaultName
     << " (" << this ->getRows ()
     << ", " << this ->getCols () << "); " <<std::endl;

  for (i=0; i < this->getRows(); ++ i)
    {
      for (j=0; j < this ->getCols(); ++ j)
	{
 	  if (! octet)
	    {
	      os << defaultName << "[" << i << "][" << j
		 << "] = " << (*this)[i][j] << "; " << std::endl;
	    }
	  else
	    {
	      for (unsigned int k = 0; k < sizeof(double); ++ k)
		{
		  os << "((unsigned char*)&(" << defaultName
		     << "[" << i << "][" << j << "]) )[" << k
		     <<"] = 0x" <<std::hex<<
		    (unsigned int)((unsigned char*)& ((*this)[i][j])) [k]
		     << "; " << std::endl;
		}
	    }
	}
      os << std::endl;
    }
  return os;
};


/*!
  \brief Compute determinant of a 3x3 matrix.

  \param M : the matrix used to compute determinant.
  \return determinant of the matrix.

  \warning M must be a 3x3 matrix.
  \deprecated Use det() instead. 
*/

double
vpMatrix::det33(const vpMatrix &M)
{

  if ((M.getCols() !=3 ) || (M.getRows() !=3))
  {
    vpTRACE("matrix is not of size 3x3 ") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\tmatrix is not of size 3x3"
			    )) ;
  }
  double detint ;

  detint = 0.0 ;
  detint =          M[0][0]*M[1][1]*M[2][2] ;
  detint = detint + M[2][0]*M[0][1]*M[1][2] ;
  detint = detint + M[0][2]*M[2][1]*M[1][0] ;
  detint = detint - M[0][2]*M[1][1]*M[2][0] ;
  detint = detint - M[0][0]*M[2][1]*M[1][2] ;
  detint = detint - M[2][2]*M[1][0]*M[0][1] ;
  return(detint);

}







/*!
  Euclidian norm ||x||=sqrt(sum(x_i^2))
  \return the norm if the matrix is initialized, 0 otherwise
  \sa infinityNorm
*/
double
vpMatrix::euclidianNorm () const
{
  double norm=0.0;
  double x ;
  for (int i=0;i<dsize;i++)
    { x = *(data +i); norm += x*x;  }
    
  return sqrt(norm);
}



/*!
  Infinity norm ||x||=max(sum(fabs(x_i)))
  \return the norm if the matrix is initialized, 0 otherwise
  \sa euclidianNorm
*/
double
vpMatrix::infinityNorm () const
{
  double norm=0.0;
  double x ;
  for (int i=0;i<rowNum;i++){
    x = 0;
    for (int j=0; j<colNum;j++){
      x += fabs (*(*(rowPtrs + i)+j)) ;
    }
      if (x > norm) { norm = x; }
  }
  return norm;
}

/*!
  Determinant of the Matrix
  \return the determinant of the matrix if the matrix is squared, 0 otherwise
  based on the LU decomposition
  see the Numerical Recipes in C page 43 for further explanations.
  \deprecated Use det() instead.
 */
 
double vpMatrix::detByLU() const
{
  double det(0);

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
      
      int  * perm = new int[rowNum];  // stores the permutations
      int d;   // +- 1 fi the number of column interchange is even or odd
      tmp.LUDcmp(perm,  d);
      delete[]perm;

      // compute the determinant that is the product of the eigen values
      det = (double) d;
      for(int i=0;i<rowNum;i++)
        {
          det*=tmp[i][i];
        }
  }

  else {
    vpERROR_TRACE("Determinant Computation : ERR Matrix not squared") ;
      throw(vpMatrixException(vpMatrixException::matrixError,
            "\n\t\tDeterminant Computation : ERR Matrix not squared")) ;
    

  }
 return det ;
}

/*!
  Compute the eigenvalues of a n-by-n real symmetric matrix.

  \return The eigenvalues of a n-by-n real symmetric matrix.

  \warning This method is only available if the Gnu Scientific Library
  (GSL) is detected as a third party library.

  \exception vpMatrixException::matrixError If the matrix is not
  square or if the matrix is not symmetric.

  \exception vpMatrixException::notImplementedError If the GSL library
  is not detected

  Here an example:
  \code
#include <iostream>

#include <visp/vpColVector.h>
#include <visp/vpMatrix.h>

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
vpColVector vpMatrix::eigenValues()
{
  if (rowNum != colNum) {
    vpERROR_TRACE("Eigen values computation: ERR Matrix not square") ;
    throw(vpMatrixException(vpMatrixException::matrixError,
          "\n\t\tEigen values computation: ERR Matrix not square")) ;
  }

#ifdef VISP_HAVE_GSL  /* be careful of the copy below */
  {
    // Check if the matrix is symetric: At - A = 0
    vpMatrix At_A = (*this).t() - (*this);
    for (int i=0; i < rowNum; i++) {
      for (int j=0; j < rowNum; j++) {
	if (At_A[i][j] != 0) {
	  vpERROR_TRACE("Eigen values computation: ERR Matrix not symmetric") ;
	  throw(vpMatrixException(vpMatrixException::matrixError,
				  "\n\t\tEigen values computation: "
				  "ERR Matrix not symmetric")) ;
	}
      }
    }
    

    vpColVector evalue(rowNum); // Eigen values

    gsl_vector *eval = gsl_vector_alloc (rowNum);
    gsl_matrix *evec = gsl_matrix_alloc (rowNum, colNum);

    gsl_eigen_symmv_workspace * w =  gsl_eigen_symmv_alloc (rowNum);
    gsl_matrix *m = gsl_matrix_alloc(rowNum, colNum);
       
    int Atda = m->tda ;
    for (int i=0 ; i < rowNum ; i++){
      int k = i*Atda ;
      for (int j=0 ; j < colNum ; j++)
	m->data[k+j] = (*this)[i][j] ;
    }
    gsl_eigen_symmv (m, eval, evec, w);
     
    gsl_eigen_symmv_sort (eval, evec, GSL_EIGEN_SORT_ABS_ASC);

    for (int i=0; i < rowNum; i++) {
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
    vpERROR_TRACE("Not implemented since the GSL library is not detected.") ;
    throw(vpMatrixException(vpMatrixException::notImplementedError,
			    "\n\t\tEigen values Computation: Not implemented "
			    "since the GSL library is not detected")) ;
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

  \exception vpMatrixException::matrixError If the matrix is not
  square or if the matrix is not symmetric.

  \exception vpMatrixException::notImplementedError If the GSL library
  is not detected

  Here an example:
  \code
#include <iostream>

#include <visp/vpColVector.h>
#include <visp/vpMatrix.h>

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
void vpMatrix::eigenValues(vpColVector &evalue, vpMatrix &evector)
{
  if (rowNum != colNum) {
    vpERROR_TRACE("Eigen values computation: ERR Matrix not square") ;
    throw(vpMatrixException(vpMatrixException::matrixError,
          "\n\t\tEigen values computation: ERR Matrix not square")) ;
  }

#ifdef VISP_HAVE_GSL  /* be careful of the copy below */
  {
    // Check if the matrix is symetric: At - A = 0
    vpMatrix At_A = (*this).t() - (*this);
    for (int i=0; i < rowNum; i++) {
      for (int j=0; j < rowNum; j++) {
	if (At_A[i][j] != 0) {
	  vpERROR_TRACE("Eigen values computation: ERR Matrix not symmetric") ;
	  throw(vpMatrixException(vpMatrixException::matrixError,
				  "\n\t\tEigen values computation: "
				  "ERR Matrix not symmetric")) ;
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
       
    int Atda = m->tda ;
    for (int i=0 ; i < rowNum ; i++){
      int k = i*Atda ;
      for (int j=0 ; j < colNum ; j++)
	m->data[k+j] = (*this)[i][j] ;
    }
    gsl_eigen_symmv (m, eval, evec, w);
     
    gsl_eigen_symmv_sort (eval, evec, GSL_EIGEN_SORT_ABS_ASC);

    for (int i=0; i < rowNum; i++) {
      evalue[i] = gsl_vector_get (eval, i);
    }
    Atda = evec->tda ;
    for (int i=0; i < rowNum; i++) {
      int k = i*Atda ;
      for (int j=0; j < rowNum; j++) {
	evector[i][j] = evec->data[k+j];
      }
    }


//        {
//          int i;
     
//          for (i = 0; i < rowNum; i++)
//            {
//              double eval_i 
//                 = gsl_vector_get (eval, i);
//              gsl_vector_view evec_i 
//                 = gsl_matrix_column (evec, i);
     
//              printf ("eigenvalue = %g\n", eval_i);
//              printf ("eigenvector = \n");
//              gsl_vector_fprintf (stdout, 
//                                  &evec_i.vector, "%g");
//            }
//        }
    
    gsl_eigen_symmv_free (w);
    gsl_vector_free (eval);
    gsl_matrix_free (m);
    gsl_matrix_free (evec);
  }
#else
  {
    vpERROR_TRACE("Not implemented since the GSL library is not detected.") ;
    throw(vpMatrixException(vpMatrixException::notImplementedError,
			    "\n\t\tEigen values Computation: Not implemented "
			    "since the GSL library is not detected")) ;
  }
#endif  
}


/*!
  Function to compute the null space (the kernel) of the interaction matrix A which is not full rank.
  The null space ( the kernel ) of a matrix A is defined as Null(A) = Ker(M) ={KerA : A*KerA =0}.

  \param kerA : The matrix to contain the null space (kernel) of A (A*KerA.t()=0) 
  \param svThreshold : Specify the used threshold in the svd(...) function (a function to compute the singular value decomposition)

  \return the rank of the matrix.
*/
int vpMatrix::kernel(vpMatrix &kerA, double svThreshold)
{
 if (DEBUG_LEVEL1)
   std::cout << "begin Kernel" << std::endl ;
 int i, j ;
 int ncaptor = getRows() ;
 int ddl = getCols() ;
 vpMatrix C ;

 if (ncaptor == 0)
   std::cout << "Erreur Matrice  non initialise" << std::endl ;

 if (DEBUG_LEVEL2)
 {
   std::cout << "Interaction matrix L" << std::endl ;
   std::cout << *this   ;
   std::cout << "signaux capteurs : " << ncaptor << std::endl ;
 }

 C.resize(ddl,ncaptor) ;
 int min ;



 if (ncaptor > ddl) min = ddl ; else min = ncaptor ;

 // ! the SVDcmp function inthe matrix lib is destructive

 vpMatrix a1 ;
 vpMatrix a2 ;

 vpColVector sv(ddl) ;   // singular values
 vpMatrix v(ddl,ddl) ;

 if (ncaptor < ddl)
 {
   a1.resize(ddl,ddl) ;
 }
 else
 {
   a1.resize(ncaptor,ddl) ;
 }

 a2.resize(ncaptor,ddl) ;

 for (i=0 ; i < ncaptor ; i++)
   for (j=0 ; j < ddl ; j++)
   {
     a1[i][j] = (*this)[i][j] ;
     a2[i][j] = (*this)[i][j] ;
   }


 if (ncaptor < ddl)
 {
   for (i=ncaptor ; i < ddl ; i++)
     for (j=0 ; j < ddl ; j++)
     {
       a1[i][j] = 0 ;
     }
   a1.svd(sv,v);
 }
 else
 {
   a1.svd(sv,v);
 }

 // compute the highest singular value and the rank of h

 double maxsv = 0 ;
 for (i=0 ; i < ddl ; i++)
   if (fabs(sv[i]) > maxsv) maxsv = fabs(sv[i]) ;

 if (DEBUG_LEVEL2)
 {
   std::cout << "Singular Value : (" ;
   for (i=0 ; i < ddl ; i++) std::cout << sv[i] << " , " ;
   std::cout << ")" << std::endl ;
 }

 int rank = 0 ;
 for (i=0 ; i < ddl ; i++)
   if (fabs(sv[i]) > maxsv*svThreshold) rank++ ;

 if (DEBUG_LEVEL2)
 /*------------------------------------------------------- */

 for (i = 0 ; i < ddl ; i++)
 {
   for (j = 0 ; j < ncaptor ; j++)
   {
     int k=0 ;
     C[i][j] = 0.0;

     // modif le 25 janvier 1999 0.001 <-- maxsv*1.e-ndof
     // sinon on peut observer une perte de range de la matrice
     // ( d'ou venait ce 0.001 ??? )
     for (k=0 ; k < ddl ; k++)  if (fabs(sv[k]) > maxsv*svThreshold)
     {
       C[i][j] += v[i][k]*a1[j][k]/sv[k];
     }
   }
 }

 // cout << v << endl ;
 if (DEBUG_LEVEL2)
 {
   std::cout << C << std::endl ;
 }

 /*------------------------------------------------------- */
 if (rank != ddl)
 {
   // Compute the kernel if wanted
   if (min < ddl)
   {
     vpMatrix ch(ddl,ddl) ;
     ch = C*a2 ;
     ch.svd(sv,v) ;

   }
   //   if (noyau == 1)
   {
     double maxsv = 0 ;
     for (i=0 ; i < ddl ; i++)
     if (fabs(sv[i]) > maxsv) maxsv = fabs(sv[i]) ;
       int rank = 0 ;
     for (i=0 ; i < ddl ; i++)
       if (fabs(sv[i]) > maxsv*svThreshold) rank++ ;
         vpMatrix cons(ddl,ddl) ;

     cons =0 ;
     for (j = 0 ; j < ddl ; j++)
     {
       for (i = 0 ; i < ddl ; i++)
       // Change Nicolas Mansard 23/4/04
       // was         if (fabs(sv[i]) < maxsv*seuilvp)
         if (fabs(sv[i]) <= maxsv*svThreshold)
         {
           cons[i][j] = v[j][i];
         }
     }

     vpMatrix Ker(ddl-rank,ddl) ;
     int k =0 ;
     for (j = 0 ; j < ddl ; j++)
     {
   //    cout << cons.Row(j+1) << " = " << cons.Row(j+1).SumSquare() << endl ;

       if (cons.row(j+1).sumSquare() !=0)
       {
         for (i=0 ; i < cons.getCols() ; i++)
           Ker[k][i] = cons[j][i] ;
     //  Ker.Row(k+1) = cons.Row(j+1) ;
         k++;
       }
     }
     kerA = Ker ;
   }
 }
 if (DEBUG_LEVEL1) std::cout << "end Kernel" << std::endl ;
 return rank ;
}

/*!
  Compute the determinant of a n-by-n matrix.

  \param method : Method used to compute the determinant. Default LU
  decomposition methos is faster than the method based on Gaussian
  elimination.

  \return Determinant of the matrix.

  \code
#include <iostream>

#include <visp/vpMatrix.h>

int main()
{
  vpMatrix A(3,3);
  A[0][0] = 1/1.; A[0][1] = 1/2.; A[0][2] = 1/3.;
  A[1][0] = 1/3.; A[1][1] = 1/4.; A[1][2] = 1/5.;
  A[2][0] = 1/6.; A[2][1] = 1/7.; A[2][2] = 1/8.;
  std::cout << "Initial matrix: \n" << A << std::endl;

  // Compute the determinant
  std:: cout << "By defaukt determinant by LU decomposition    : " << 
    A.det() << std::endl;
  std:: cout << "Determinant by LU decomposition    : " << 
    A.det(vpMatrix::LU_DECOMPOSITION ) << std::endl;
}
  \endcode
*/
double vpMatrix::det(vpDetMethod method) const
{
  double det = 0;

  if ( method == LU_DECOMPOSITION )
    {
      det = this->detByLU();
    }

  return (det);
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
vpMatrix subblock(const vpMatrix &M, int col, int row)
{
vpMatrix M_comp(M.getRows()-1,M.getCols()-1);

for ( int i = 0 ; i < col ; i++)
    {
    for ( int j = 0 ; j < row ; j++)
        M_comp[i][j]=M[i][j];
     for ( int j = row+1 ; j < M.getRows() ; j++)
         M_comp[i][j-1]=M[i][j];
    }
for ( int i = col+1 ; i < M.getCols(); i++)
    {
    for ( int j = 0 ; j < row ; j++)
        M_comp[i-1][j]=M[i][j];
     for ( int j = row+1 ; j < M.getRows() ; j++)
         M_comp[i-1][j-1]=M[i][j];
    }
   return M_comp;
}


#undef DEBUG_LEVEL1
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
