
/*+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 * Copyright Projet Lagadic / IRISA-INRIA Rennes, 2005
 * www  : http://www.irisa.fr/lagadic
 *+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 *
 * File:      vpMatrix.h
 * Project:   ViSP 2.0
 * Author:    Eric Marchand
 *
 * Version control
 * ===============
 *
 *  $Id: vpMatrix.h,v 1.12 2006-04-19 09:01:21 fspindle Exp $
 *
 * Description
 * ============
 *     class that defines what is a matrix
 *
 * ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++*/



#ifndef vpMatrix_H
#define vpMatrix_H


#include <iostream>
#include <math.h>

#include <visp/vpConfig.h>

class vpRowVector;
class vpColVector;
class vpTranslationVector;


class vpColVector;
class vpTranslationVector;
class vpRowVector;

using namespace std ;

/*!
  \file vpMatrix.h
  \brief definition of matrix class as well
  as a set of operations on these matrices
*/


/*!
  \class vpMatrix

  \brief Definition of the vpMatrix class.

  vpMatrix class provides a data structure for the matrices as well
  as a set of operations on these matrices

  \author Eric Marchand (IRISA - INRIA Rennes)

  \warning Note the matrix in the class (*this) will be noted A in the comment

  \ingroup libmath

  \sa vpRowVector, vpColVector, vpHomogeneousMatrix, vpRotationMatrix,
  vpTwistMatrix, vpHomography
*/
class vpMatrix
{
  int k ;
protected:
  //! number of rows
  int rowNum;
  //! number of columns
  int colNum;

public:
  //! address of the first element of the data array
  double *data;
protected:
  //! address of the first element of each rows
  double **rowPtrs;

  //! Current size (rowNum * colNum)
  int dsize;
  //! Total row space
  int trsize;

public:
  //! Initialization of the object matrix
   void  init() ;
  //! Basic constructor
  vpMatrix() ;
  //! Constructor. Initialization of A as an r x c matrix with 0.
  vpMatrix(int r,int c) ;

  //! sub vpMatrix constructor
  vpMatrix(const vpMatrix &m, int r, int c, int nrows, int ncols) ;

  //! Destruction of the matrix  (Memory de-allocation)
  void kill() ;
  //! Destructor (Memory de-allocation)
  virtual ~vpMatrix();

  //---------------------------------
  // Set/get Matrix size
  //---------------------------------

  //! Return the number of rows of the matrix
  inline int getRows() const { return rowNum ;}
  //! Return the number of columns of the matrix
  inline int getCols() const { return colNum; }

  //! Set the size of the matrix A, initialization with a zero matrix
  virtual void resize(const int nrows, const int ncols, const bool nullify = true);

  //---------------------------------
  // Printing
  //---------------------------------
  friend ostream &operator << (ostream &s,const vpMatrix &m);

  int print(std::ostream& s, unsigned lenght, char const* intro=0);
  //! Affichage pour reinsertion dans matlab
  ostream & matlabPrint(ostream & os);
  //! Affichage pour reinsertion dans ViSP
  ostream & cppPrint(ostream & os, const char * matrixName = NULL, bool octet = false);

  //---------------------------------
  // Copy / assigment
  //---------------------------------

  //! Copy constructor
  vpMatrix (const vpMatrix& m);

  //! Assignment from an array
  vpMatrix &operator<<(double*);

  //! Copy operator.   Allow operation such as A = B
  vpMatrix &operator=(const vpMatrix &B);
  //! Set all the element of the matrix A to x
  vpMatrix &operator=(const double x);

  //---------------------------------
  // Access/modification operators
  //---------------------------------

  //! write elements Aij (usage : A[i][j] = x )
  inline double *operator[](int n) { return rowPtrs[n]; }
  //! read elements Aij  (usage : x = A[i][j] )
  inline double *operator[](int n) const {return rowPtrs[n];}

  //---------------------------------
  // Matrix operations.
  //---------------------------------

  //! operation A = A + B
   vpMatrix &operator+=(const vpMatrix &B);
  //! operation A = A - B
   vpMatrix &operator-=(const vpMatrix &B);
  //! operation C = A * B (A is unchanged)
   vpMatrix operator*(const vpMatrix &B) const  ;
  //! operation C = A + B (A is unchanged)
   vpMatrix operator+(const vpMatrix &B) const;
  //! operation C = A - B (A is unchanged)
   vpMatrix operator-(const vpMatrix &B) const;
  //! C = -A  (A is unchanged)
   vpMatrix operator-() const;

  //!return sum of the Aij^2 (for all i, for all j)
   double sumSquare() const;

  //---------------------------------
  // Matrix/vector operations.
  //---------------------------------

  //! operation c = A * b (A is unchanged, c and b are vectors)
    vpColVector operator*(const vpColVector &b) const;
    //! operation c = A * b (A is unchanged, c and b are translation vectors)
    vpTranslationVector operator*(const vpTranslationVector  &b) const;
  //---------------------------------
  // Matrix/real operations.
  //---------------------------------

  //! Add x to all the element of the matrix : Aij = Aij + x
  vpMatrix &operator+=(const double x);
  //! Substract x to all the element of the matrix : Aij = Aij - x
  vpMatrix &operator-=(const double x);
  //! Multiply  all the element of the matrix by x : Aij = Aij * x
  vpMatrix &operator*=(const double x);
  //! Divide  all the element of the matrix by x : Aij = Aij / x
  vpMatrix &operator/=(double x);

  // Cij = Aij * x (A is unchanged)
  vpMatrix operator*(const double x) const;
  // Cij = Aij / x (A is unchanged)
  vpMatrix operator/(const double x) const;

  //-------------------------------------------------
  // Columns, Rows extraction, SubMatrix
  //-------------------------------------------------

  //! Row extraction
  vpRowVector row(const int i);
  //! Column extraction
  vpColVector column(const int j);
  //! subvpMatrix extraction
  void init(const vpMatrix &m, int r, int c, int nrows, int ncols);

  //-------------------------------------------------
  // transpose, identity
  //-------------------------------------------------

  //! Compute the transpose C = A^T
  vpMatrix t() const;

  //! Set the matrix to identity
  void setIdentity() ;

  //! Initialize an identity matrix n-by-n
  void eye(int n) ;
  //! Initialize an identity matrix m-by-n
   void eye(int m, int n) ;


  //-------------------------------------------------
  // LU decomposition
  //-------------------------------------------------

  //! LU Decomposition
  void LUDcmp(int* perm, int& d);
  //! solve AX = B using the LU Decomposition
  void LUBksb(int* perm, vpColVector& b);

  //! solve Ax=B using the LU decomposition (usage A = solveByLUD(B,x) )
  //  void solveByLUD(const CColVector& B, CColVector& x) const ;

  //! solve Ax=B using the LU decomposition (usage  x=A.LUDsolve(B))
  //  CColVector LUDsolve(const CColVector& B) const ;

  //! inverse matrix A using the SVD  (C = A^-1)
 vpMatrix inverseByLU() const;

  //-------------------------------------------------
  // SVD decomposition
  //-------------------------------------------------

#ifndef DOXYGEN_SHOULD_SKIP_THIS
  void svdFlake(vpColVector& w, vpMatrix& v);
  void svdNr(vpColVector& w, vpMatrix& v);
#ifdef VISP_HAVE_GSL
  void svdGsl(vpColVector& w, vpMatrix& v);
#endif
 //! solve AX=B using the SVD decomposition
    void SVBksb(const vpColVector& w,
  	      const vpMatrix& v,
  	      const vpColVector& b, vpColVector& x);
#endif

  //! singular value decomposition
  void svd(vpColVector& w, vpMatrix& v);


  //! solve Ax=B using the SVD decomposition (usage A = solveBySVD(B,x) )
  void solveBySVD(const vpColVector& B, vpColVector& x) const ;
  //! solve Ax=B using the SVD decomposition (usage  x=A.SVDsolve(B))
  vpColVector SVDsolve(const vpColVector& B) const ;

  //! Compute the pseudo inverse of the matrix using the SVD.
  //! return the rank
  int pseudoInverse(vpMatrix &Ap, double seuil=1e-6)  const;
  //! Compute the pseudo inverse of the matrix using the SVD.
  //! return the rank and the singular value
  int pseudoInverse(vpMatrix &Ap, vpColVector &sv, double seuil=1e-6) const ;
  //! Compute the pseudo inverse of the matrix using the SVD.
  //! return the rank and the singular value, image
  int pseudoInverse(vpMatrix &Ap,
		    vpColVector &sv, double seuilvp,
		    vpMatrix &ImAt,
		    vpMatrix &ImA) const ;
  //! Compute the pseudo inverse of the matrix using the SVD.
  vpMatrix pseudoInverse(double seuil=1e-6)  const;

  //! test if the matrix is a rotation matrix ( R^T R = Id 3x3 )
  //  int isARotationMatrix() const ;
  void printSize() { cout << getRows() <<" x " << getCols() <<"  " ; }


  //! Stack two Matrices C = [ A B ]^T
  static vpMatrix stackMatrices(const vpMatrix &A,const  vpMatrix &B) ;
  //! Stack two Matrices C = [ A B ]^T
  static void stackMatrices(const vpMatrix &A,const  vpMatrix &B, vpMatrix &C) ;
  //! Juxtapose to matrices C = [ A B ]
  static vpMatrix juxtaposeMatrices(const vpMatrix &A,const  vpMatrix &B) ;
  //! Juxtapose to matrices C = [ A B ]
  static void juxtaposeMatrices(const vpMatrix &A,const  vpMatrix &B, vpMatrix &C) ;

  //! Create a diagonal matrix with the element of a vector DAii = Ai
  static void createDiagonalMatrix(const vpColVector &A, vpMatrix &DA)  ;

  static double det33(const vpMatrix &P) ;



  // -------------------------
  // Norms
  // -------------------------

  //! Euclidian norm ||x||=sqrt(sum(x_i))
  double euclidianNorm () const;
  //! Infinity norm ||x||=max(fabs(x_i))
  double infinityNorm () const;

};


//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////


//! multiplication by a scalar C = x*A
vpMatrix operator*(const double &x, const vpMatrix &A) ;

  //! multiplication by a scalar C = x*A
vpColVector operator*(const double &x, const vpColVector &A) ;



#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
