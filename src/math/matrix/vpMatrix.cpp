
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpMatrix.cpp
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 * From:      vpMatrix.cpp, ViSP 1.6.8 (author:Eric Marchand)
 *
 * Version control
 * ===============
 *
 *  $Id: vpMatrix.cpp,v 1.14 2005-11-04 14:52:27 nmansard Exp $
 *
 * Description
 * ============
 *     Class that consider matrices operation
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/




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
#include <vector>
#include <sstream>
#include <algorithm>

#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpTranslationVector.h>


// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>

#define DEBUG_LEVEL1 1

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
  \brief   constructor
  initialize a rr x cc matrix with 0

  \param int rr : number of rows
  \param int cc number of columns
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
    ERROR_TRACE("\n\t\t Illegal subMatrix operation") ;
    throw(vpMatrixException(vpMatrixException::subMatrixError,
			    "\n\t\t Illegal subMatrix operation")) ;
  }

  if (((r + nrows) > m.rowNum) || ((c + ncols) > m.colNum))
  {
    ERROR_TRACE("\n\t\t SubvpMatrix larger than vpMatrix") ;
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

void
vpMatrix::resize(const int nrows, const int ncols, const bool flagNullify)
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

    DEBUG_TRACE (25, "Recopy case per case is required iff number of "
		 "cols has changed (structure of double array is not "
		 "the same in this case.");
    if (recopyNeeded) 
      { 
	copyTmp = new double[this->dsize];
	memcpy (copyTmp, this ->data, sizeof(double)*this->dsize);
	rowTmp=this->rowNum; colTmp=this->colNum;
      }

    DEBUG_TRACE (25, "Reallocation of this->data array.");
    this->dsize = nrows*ncols;
    this->data = (double*)realloc(this->data, this->dsize*sizeof(double));
    if ((NULL == this->data) && (0 != this->dsize))
    {
      ERROR_TRACE("\n\t\tMemory allocation error when allocating data") ;
      throw(vpException(vpException::memoryAllocationError,
			"\n\t\t Memory allocation error when "
			"allocating data")) ;
    }

    DEBUG_TRACE (25, "Reallocation of this->trsize array.");
    this->trsize = nrows;
    this->rowPtrs = (double**)realloc (this->rowPtrs, this->trsize*sizeof(double*));
    if ((NULL == this->rowPtrs) && (0 != this->dsize))
    {
      ERROR_TRACE("\n\t\tMemory allocation error when allocating rowPtrs") ;
      throw(vpException(vpException::memoryAllocationError,
			"\n\t\t Memory allocation error when "
			"allocating rowPtrs")) ;
    }

    DEBUG_TRACE (25, "Recomputation this->trsize array values.");
    {
      double **t= rowPtrs;
      for (int i=0; i<dsize; i+=ncols)  { *t++ = this->data + i; }
    }

    this->rowNum = nrows; this->colNum = ncols;

    DEBUG_TRACE (25, "Recopy of this->data array values or nullify.");
    if (flagNullify) 
      { memset(this->data,0,this->dsize*sizeof(double)) ;}
    else 
      {
	if (recopyNeeded)
	  {
	    DEBUG_TRACE (25, "Recopy...");
	    const int minRow = (this->rowNum<rowTmp)?this->rowNum:rowTmp;
	    const int minCol = (this->colNum<colTmp)?this->colNum:colTmp;
	    for (int i=0; i<this->rowNum; ++i) 
	      for (int j=0; j<this->colNum; ++j) 
		{
		  if ((minRow > i) && (minCol > j))
		    {
		      (*this)[i][j] = copyTmp [i*colTmp+j];
		      CDEBUG (25) << i << "x" << j << "<- " << i*colTmp+j 
				  << "=" << copyTmp [i*colTmp+j] << endl;
		    }
		  else {(*this)[i][j] = 0;}
		}
	  }
	else { DEBUG_TRACE (25,"Nothing to do: already done by realloc.");}
      }

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
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
    throw ;
  }

  for (int i=r ; i < r+nrows; i++)
    for (int j=c ; j < c+ncols; j++)
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
    delete []data;
    data=NULL;
  }

  if (rowPtrs!=NULL)
  {
    delete []rowPtrs;
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

  \param m : matrix to be copied.
*/
vpMatrix &
vpMatrix::operator=(const vpMatrix &B)
{
  try {
    resize(B.rowNum, B.colNum) ;
    *this = 0;
  }
  catch(vpException me)
  {
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
    throw ;
  }



  // Modif EM 16/6/03
    memcpy(data,B.data,dsize*sizeof(double)) ;
    /*  for (int i=0; i<rowNum; i++) {
      for (int j=0; j<colNum; j++) {
      rowPtrs[i][j] = B.rowPtrs[i][j];
      }
      }
  */

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

//! operation C = A * B (A is unchanged)
vpMatrix
vpMatrix::operator*(const vpMatrix &B) const
{
  vpMatrix p ;


  try {
    p.resize(rowNum,B.colNum) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
    throw ;
  }

  if (colNum != B.rowNum)
  {
    ERROR_TRACE("\n\t\tvpMatrix mismatch in vpMatrix/vpMatrix multiply") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\tvpMatrix mismatch in "
			    "vpMatrix/vpMatrix multiply")) ;
  }
  for (int i=0;i<rowNum;i++)
    for (int j=0;j<B.colNum;j++)
    {
      double s =0 ;
      for (int k=0;k<B.rowNum;k++)
	s +=rowPtrs[i][k] * B.rowPtrs[k][j];
      p[i][j] = s ;
    }
  return p;
}

//! operation C = A + B (A is unchanged)
vpMatrix
vpMatrix::operator+(const vpMatrix &B) const
{
  vpMatrix v ;


  try {
    v.resize(rowNum,colNum) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
    throw ;
  }

  if ( (colNum != B.getCols())||(rowNum != B.getRows()))
  {
    ERROR_TRACE("\n\t\t vpMatrix mismatch in vpMatrix/vpMatrix addition") ;
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
  for (i=0;i<dsize;i++)
  {
    *(v.data + i) = *(B.data + i) + *(data + i) ;
  }
  return v;
}

//! operation C = A - B (A is unchanged)
vpMatrix
vpMatrix::operator-(const vpMatrix &B) const
{
  vpMatrix v ;
  try {
    v.resize(rowNum,colNum) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
    throw ;
  }

  if ( (colNum != B.getCols())||(rowNum != B.getRows()))
  {
    ERROR_TRACE("\n\t\t vpMatrix mismatch in vpMatrix/vpMatrix substraction") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\t vpMatrix mismatch in "
			    "vpMatrix/vpMatrix substraction")) ;

  }

  int i;

  // MODIF EM 16/6/03
  for (i=0;i<dsize;i++)
  {
    *(v.data + i) = *(data + i) - *(B.data + i) ;
  }
  /*
    int j;
    for (i=0;i<rowNum;i++)
    for(j=0;j<colNum;j++)
    v.rowPtrs[i][j] = rowPtrs[i][j]-B.rowPtrs[i][j];
  */
  return v;
}

//! operation A = A + B

vpMatrix &vpMatrix::operator+=(const vpMatrix &B)
{
  if ( (colNum != B.getCols())||(rowNum != B.getRows()))
  {
    ERROR_TRACE("\n\t\t vpMatrix mismatch in vpMatrix +=  addition") ;
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
    ERROR_TRACE("\n\t\t vpMatrix mismatch in vpMatrix -= substraction") ;
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

//! C = -A  (A is unchanged)

vpMatrix vpMatrix::operator-() const //negate
{
  vpMatrix C ;



  try {
    C.resize(rowNum, colNum) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
    throw ;
  }


  for (int i=0;i<dsize;i++)
  {
    *(C.data + i) = -*(data + i) ;
  }
  /*
    for (int i=0; i<rowNum; i++)
    for (int j=0; j<colNum; j++)
    C[i][j] = - rowPtrs[i][j];
  */
  return C;
}

//!return sum of the Aij^2 (for all i, for all j)
double
vpMatrix::sumSquare() const
{
  double sum=0.0;
  double x ;



  for (int i=0;i<dsize;i++)
  {
    x = *(data + i) ;
    sum += x*x ;
  }
  /*
    for (int i=0; i<rowNum; i++)
    for (int j=0; j<colNum; j++)
    sum += rowPtrs[i][j]*rowPtrs[i][j];
  */

  return sum;
}

//---------------------------------
// Matrix/vector operations.
//---------------------------------

//! operation c = A * b (A is unchanged, c and b are vectors)
vpColVector
vpMatrix::operator*(const vpColVector &b) const
{

  vpColVector c(rowNum);


  if (colNum != b.getRows())
  {
    ERROR_TRACE("vpMatrix mismatch in vpMatrix/vector multiply") ;
    throw(vpMatrixException::incorrectMatrixSizeError) ;
  }



  c = 0.0;

  for (int j=0;j<colNum;j++) {
    {
      for (int i=0;i<rowNum;i++) {
	c[i]+=rowPtrs[i][j] * b[j];
      }
    }
  }

  return c ;
}

//! operation c = A * b (A is unchanged, c and b are translation vectors)
vpTranslationVector
vpMatrix::operator*(const vpTranslationVector &b) const
{
  vpTranslationVector c;

  for (int j=0;j<3;j++) c[j]=0 ;

  for (int j=0;j<3;j++) {
    {
      for (int i=0;i<3;i++) {
	c[i]+=rowPtrs[i][j] * b[j];
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
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
    throw ;
  }

  for (int i=0;i<B.getRows(); i++)
    for (int j=0 ; j < B.getCols();j++)
      v[i][j] = B[i][j]*x;
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
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
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
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
    throw ;
  }



  for (int i=0;i<dsize;i++)
    *(v.data+i) = *(data+i)/x;
  /*
    int i;int j;
    for (i=0;i<rowNum;i++)
    for(j=0;j<colNum;j++)
    v.rowPtrs[i][j] = rowPtrs[i][j]/x;
  */
  return v;
}


//! Add x to all the element of the matrix : Aij = Aij + x
vpMatrix & vpMatrix::operator+=(double x)
{


  for (int i=0;i<dsize;i++)
    *(data+i) += x;
  /*
    int i;int j;
    for (i=0;i<rowNum;i++)
    for(j=0;j<colNum;j++)
    rowPtrs[i][j] += x;
  */
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


  for (int i=0;i<dsize;i++)
    *(data+i) /= x;
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
    ERROR_TRACE("non square matrix") ;
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
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
    throw ;
  }
}
/*
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
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
    throw ;
  }


  for (int i=0; i<rowNum; i++)
    for (int j=0; j<colNum; j++)
      if (i == j) (*this)[i][j] = 1;
      else        (*this)[i][j] = 0;

}


/*
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
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
    throw ;
  }



  int i,j;
  for (i=0;i<rowNum;i++)
    for (j=0;j<colNum;j++)
      At[j][i] = (*this)[i][j];

  return At;
}

/*!
  \brief solve a linear system AX = B using an SVD decomposition

  non destructive wrt. A and B

  \sa this function SVDcmp and SVDksb for solving the system
*/
void
vpMatrix::solveBySVD(const vpColVector& b, vpColVector& x) const
{
  x = pseudoInverse(1e-6)*b ;
}


/*!
  \brief solve a linear system AX = B using an SVD decomposition

  non destructive wrt. A and B

  \sa SVDcmp and SVDksb
*/
vpColVector vpMatrix::SVDsolve(const vpColVector& B) const
{
  vpColVector X(colNum);

  solveBySVD(B, X);
  return X;
}



void
vpMatrix::svd(vpColVector& w, vpMatrix& v)
{
  if (DEBUG_LEVEL1 == 0) /* no verification */
  {
#ifdef HAVE_LIBGSL  /* be careful of the copy below */
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

#ifdef HAVE_LIBGSL  /* be careful of the copy above */
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
      cout << " A : " << endl << A << endl;
      cout << " Asvd : " << endl << Asvd << endl;
    }
    //    else printf("SVD ok ;-)\n");  /* It's so good... */
  }
}
/*!
  \brief Compute the pseudo inverse of the matrix Ap = A^+
  \param Ap = A^+ the pseudo inverse
  \param th threshold used to test the singular values
  \return Return the rank of the matrix A
*/

int
vpMatrix::pseudoInverse(vpMatrix &Ap, double th) const
{
  vpColVector sv ;
  return   pseudoInverse(Ap,sv,th) ;
}

/*!
  \brief retourne la pseudo d'un matrice C = A^+
  \param th threshold used to test the singular values
*/
vpMatrix
vpMatrix::pseudoInverse(double seuilvp) const
{
  vpMatrix Ap ;
  vpColVector sv ;
  pseudoInverse(Ap,sv,seuilvp) ;
  return   Ap ;
}

/*!
  \brief Compute the pseudo inverse of the matrix Ap = A^+
  \param Ap = A^+ the pseudo inverse
  \param sv singular values
  \param th threshold used to test the singular values
  \return Return the rank of the matrix A
*/
int
vpMatrix::pseudoInverse(vpMatrix &Ap, vpColVector &sv, double seuilvp) const
{
  vpMatrix imA, imAt ;
  return pseudoInverse(Ap,sv,seuilvp, imA, imAt) ;
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



  \param Ap = A^+ the pseudo inverse
  \param sv singular values
  \param th threshold used to test the singular values
  \param ImAt : Image A^T
  \param ImA: Image  A
  \return Return the rank of the matrix A

*/
int
vpMatrix::pseudoInverse(vpMatrix &Ap,
			vpColVector &sv, double seuilvp,
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
    if (fabs(sv[i]) > maxsv*seuilvp) rank++ ;



  /*------------------------------------------------------- */
  for (i = 0 ; i < ncols ; i++)
  {
    for (j = 0 ; j < nrows ; j++)
    {
      a1[i][j] = 0.0;

      for (k=0 ; k < ncols ; k++)
    	if (fabs(sv[k]) > maxsv*seuilvp)
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
      cout << " A : " << endl << A << endl;
      cout << " Ap : " << endl << Ap << endl;
      cout << " A - AApA : " << endl << A - AApA << endl;
      cout << " Ap - ApAAp : " << endl << Ap - ApAAp << endl;
      cout << " AAp - (AAp)^T : " << endl << AAp - AAp.t() << endl;
      cout << " ApA - (ApA)^T : " << endl << ApA - ApA.t() << endl;
    }
    //    else printf("Ap OK ;-) \n");

  }


  // cout << v << endl ;
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
  \relates vpMatrix
  \brief StackMatrices. "Stack" two matrices  C = [ A B ]^T

  \f$ C = \left( \begin{array}{c} A \\ B \end{array}\right)    \f$

  \param vpMatrix A
  \param vpMatrix B
  \return  vpMatrix C = [ A B ]^T

  \warning A and B must have the same number of column
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
    CERROR ;
    throw ;
  }

  return C ;
}

/*!
  \relates vpMatrix
  \brief stackMatrices. "stack" two matrices  C = [ A B ]^T

  \f$ C = \left( \begin{array}{c} A \\ B \end{array}\right)    \f$

  \param  A
  \param  B
  \param  C = [ A B ]^T

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
      ERROR_TRACE("\n\t\t incorrect matrices size") ;
      throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			      "\n\t\t incorrect matrices size")) ;
    }

  try {
    C.resize(nra+nrb,B.getCols()  ) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
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
  \relates vpMatrix
  \brief Create a diagonal matrix with the element of a vector DAii = Ai

  \param  DA  = Ai
  \param  A
*/
//! Create a diagonal matrix with the element of a vector DAii = Ai
void
vpMatrix::createDiagonalMatrix(const vpColVector &A, vpMatrix &DA)
{
  int rows = A.getRows() ;
  try {
    DA.resize(rows,rows) ;
  }
  catch(vpException me)
  {
    ERROR_TRACE("Error caught") ;
    cout << me << endl ;
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
  \brief cout a matrix
*/
ostream &operator <<(ostream &s,const vpMatrix &m)
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

  \param length
    The suggested width of each matrix element.
    The actual width grows in order to accomodate the whole integral part,
    and shrinks if the whole extent is not needed for all the numbers.
  \return
    Returns the common total width for all matrix elements

  \sa vpMatrix::print(), ostream &operator <<(ostream &s,const vpMatrix &m)
*/
int
vpMatrix::print(std::ostream& s, unsigned maxlen)
{
  int m = getRows();
  int n = getCols();

  std::vector<std::string> values(m*n);
  std::ostringstream oss;

  unsigned maxBefore=0;
  unsigned maxAfter=0;

  for (int i=0;i<m;++i) {
    for (int j=0;j<n;++j){
      oss.str("");
      oss << (*this)[i][j];

      values[i*n+j]=oss.str();
      unsigned len=values[i*n+j].size();
      unsigned p=values[i*n+j].find('.');

      if (p==std::string::npos){
        maxBefore=std::max(maxBefore, len);
        // maxAfter remains the same
      } else{
	maxBefore=std::max(maxBefore, p);
	maxAfter=std::max(maxAfter, len-p-1);
      }
    }
  }

  // increase maxlen if needed to accomodate
  // the whole integral part
  maxlen=std::max(maxlen,maxBefore);
  // decrease the number of decimals,
  // according to maxlen
  maxAfter=std::min(maxAfter, maxlen-maxBefore-1);

  for (int i=0;i<m;i++) {
    for (int j=0;j<n;j++){
      unsigned p=values[i*n+j].find('.');
      s.setf(ios::right, ios::adjustfield);
      s.width(maxBefore);
      s <<values[i*n+j].substr(0,p);

      if (maxAfter>0){
        s.setf(ios::left, ios::adjustfield);
        if (p!=std::string::npos){
          s <<'.';
          s.width(maxAfter);
          s <<values[i*n+j].substr(p+1,maxAfter);
        } else{
          s.width(maxAfter+1);
          s <<".0";
        }
      }

      s <<' ';
    }
    s <<std::endl;
  }

  int rv=maxBefore;
  if (maxAfter>0)
    rv+=(1+maxAfter);
  return rv;
}


/*!
  \brief Print using matlab syntax, to be put in matlab later.

  Print using the following form:
     [ a,b,c;
       d,e,f;
       g,h,i]
*/
ostream & vpMatrix::
matlabPrint(ostream & os)
{
  
  int i,j;

  os << "[ ";
  for (i=0; i < this->getRows(); ++ i) 
    {
      for (j=0; j < this ->getCols(); ++ j)
	{
	  os <<  (*this)[i][j] << ", ";
	}
      if (this ->getRows() != i+1) { os << ";" << endl; }
      else { os << "]" << endl; }
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
ostream & vpMatrix::
cppPrint(ostream & os, const char * matrixName, bool octet)
{
  
  int i,j;
  const char defaultName [] = "A";
  if (NULL == matrixName)
    {
      matrixName = defaultName;
    }
  os << "vpMatrix " << defaultName 
     << " (" << this ->getRows ()
     << ", " << this ->getCols () << "); " <<endl;

  for (i=0; i < this->getRows(); ++ i) 
    {
      for (j=0; j < this ->getCols(); ++ j)
	{
 	  if (! octet)
	    {
	      os << defaultName << "[" << i << "][" << j 
		 << "] = " << (*this)[i][j] << "; " << endl;
	    }
	  else
	    {
	      for (unsigned int k = 0; k < sizeof(double); ++ k)
		{
		  os << "((unsigned char*)&(" << defaultName 
		     << "[" << i << "][" << j << "]) )[" << k 
		     <<"] = 0x" <<hex<< 
		    (unsigned int)((unsigned char*)& ((*this)[i][j])) [k] 
		     << "; " << endl;
		}
	    }
	}
      os << endl; 
    }
  return os;
};





double
vpMatrix::det33(const vpMatrix &M)
{

  if ((M.getCols() !=3 ) || (M.getRows() !=3))
  {
    TRACE("matrix is not of size 3x3 ") ;
    throw(vpMatrixException(vpMatrixException::incorrectMatrixSizeError,
			    "\n\t\tmatrix is not of size 3x3"
			    )) ;
  }
  double detint ;

  detint = 0.0 ;
  detint =          M[0][0]*M[1][1]*M[2][2]/2 ;
  detint = detint + M[2][0]*M[0][1]*M[1][2]/2 ;
  detint = detint + M[0][2]*M[2][1]*M[1][0]/2 ;
  detint = detint - M[0][2]*M[1][1]*M[2][0]/2 ;
  detint = detint - M[0][0]*M[2][1]*M[1][2]/2 ;
  detint = detint - M[2][2]*M[1][0]*M[0][1]/2 ;
  return(detint);

}







/*!
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
  
  return norm;
}



/*!
  \return the norm if the matrix is initialized, 0 otherwise
  \sa euclidianNorm
*/
double  
vpMatrix::infinityNorm () const
{
  double norm=0.0;
  double x ;
  for (int i=0;i<dsize;i++)
    {
      x = fabs (*(data + i)) ;
      if (x > norm) { norm = x; }
    }
  
  return norm;
}




#undef DEBUG_LEVEL1
/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
