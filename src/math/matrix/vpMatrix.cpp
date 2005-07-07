
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
 *  $Id: vpMatrix.cpp,v 1.4 2005-07-07 08:29:37 fspindle Exp $
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
#include <visp/vpMath.h>
#include <visp/vpMatrix.h>
#include <visp/vpTranslationVector.h>


// Exception
#include <visp/vpException.h>
#include <visp/vpMatrixException.h>

// Debug trace
#include <visp/vpDebug.h>

// Debug trace
#include <visp/vpConfig.h>


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

  \return OK or MEMORY_FAULT if memory cannot be allocated
*/

void
vpMatrix::resize(int nrows, int ncols)
{

  //  ERROR_TRACE("In resize") ; cout << rowNum <<"  " << colNum <<endl ;
  // If things are the same, do nothing and return
  if ((nrows == rowNum) && (ncols == colNum))
  {
    memset(data,0,dsize*sizeof(double)) ;
  }
  else
  {
    rowNum = nrows; colNum = ncols;
    // Check data room

    dsize = nrows*ncols;
    if ( data != NULL ) { delete [] data; data = NULL ; }
    data = new double[dsize];
    if (data == NULL)
    {
      ERROR_TRACE("\n\t\tMemory allocation error") ;
      throw(vpException(vpException::memoryAllocationError,
			"\n\t\t Memory allocation error")) ;
    }

    trsize = nrows;
    if ( rowPtrs != NULL  ) delete [] rowPtrs;
    rowPtrs = new double*[trsize];
    if (rowPtrs == NULL)
    {
      ERROR_TRACE("\n\t\tMemory allocation error") ;
      throw(vpException(vpException::memoryAllocationError,
			"\n\t\t Memory allocation error")) ;
    }

    double **t;

    t = rowPtrs;
    for (int i=0; i<dsize; i+=ncols)  *t++ = data + i;

    memset(data,0,dsize*sizeof(double)) ;

  }  //ERROR_TRACE("out resize") ;

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
#ifdef HAVE_LIBGSL
  svdGsl(w,v) ;
#else
  svdNr(w,v) ;
#endif

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

  int i, j ;
  int nrows = getRows() ;



  int ncols = getCols() ;
  Ap.resize(ncols,nrows) ;
  int min ;

  if (nrows > ncols) min = ncols ; else min = nrows ;

  // ! the SVDcmp function inthe matrix lib is destructive

  vpMatrix a1 ;
  vpMatrix a2 ;

  vpMatrix v(ncols,ncols) ;

  sv.resize(ncols) ;
  if (nrows < ncols)
  {
    a1.resize(ncols,ncols) ;
  }
  else
  {
    a1.resize(nrows,ncols) ;
  }
  a2.resize(nrows,ncols) ;

  for (i=0 ; i < nrows ; i++)
    for (j=0 ; j < ncols ; j++)
    {
      a1[i][j] = (*this)[i][j] ;
    }

  a2 = *this ;

  if (nrows < ncols)
  {
    for (i=nrows ; i < ncols ; i++)
      for (j=0 ; j < ncols ; j++)
	a1[i][j] = 0 ;
    a1.svd(sv,v);

  }
  else
  {
    a1.svd(sv,v);
  }
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
      int k=0 ;
      Ap[i][j] = 0.0;

      // modif le 25 janvier 1999 0.001 <-- maxsv*1.e-6
      // sinon on peut observer une perte de range de la matrice
      // ( d'ou venait ce 0.001 ??? )
      for (k=0 ; k < ncols ; k++)
	if (fabs(sv[k]) > maxsv*seuilvp)
	{
	  Ap[i][j] += v[i][k]*a1[j][k]/sv[k];
	}
    }
  }

  // cout << v << endl ;
  return rank ;
}


//////////////////////////////////////////////////////////////////////////

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
    ctrace ;
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
vpMatrix::CreateDiagonalMatrix(const vpColVector &A, vpMatrix &DA)
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

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
