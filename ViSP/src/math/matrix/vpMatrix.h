/****************************************************************************
 *
 * $Id: vpMatrix.h,v 1.28 2009-01-08 16:17:07 fspindle Exp $
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



#ifndef vpMatrix_H
#define vpMatrix_H


#include <iostream>
#include <math.h>

#include <visp/vpTime.h>
#include <visp/vpConfig.h>

#ifdef VISP_HAVE_GSL
#  include <gsl/gsl_math.h>
#  include <gsl/gsl_eigen.h>
#endif

class vpRowVector;
class vpColVector;
class vpTranslationVector;


class vpColVector;
class vpTranslationVector;
class vpRowVector;



/*!
  \file vpMatrix.h

  \brief Definition of matrix class as well as a set of operations on
  these matrices.
*/



/*!
  \class vpMatrix
  \ingroup Matrix

  \brief Definition of the vpMatrix class.

  vpMatrix class provides a data structure for the matrices as well
  as a set of operations on these matrices

  \author Eric Marchand (IRISA - INRIA Rennes)

  \warning Note the matrix in the class (*this) will be noted A in the comment

  \ingroup libmath

  \sa vpRowVector, vpColVector, vpHomogeneousMatrix, vpRotationMatrix,
  vpTwistMatrix, vpHomography
*/
class VISP_EXPORT vpMatrix
{
  int k ;

 public:
  /*!
    Method used to compute the determinant of a square matrix.
    \sa det()
  */
  typedef enum {
    LU_DECOMPOSITION,     /*!< LU decomposition method. */
  } vpDetMethod;


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
  //! Basic constructor
  vpMatrix() ;
  //! Constructor. Initialization of A as an r x c matrix with 0.
  vpMatrix(int r,int c) ;

  //! sub vpMatrix constructor
  vpMatrix(const vpMatrix &m, int r, int c, int nrows, int ncols) ;

  //! Destructor (Memory de-allocation)
  virtual ~vpMatrix();

  //! Initialization of the object matrix
   void  init() ;

  //! Destruction of the matrix  (Memory de-allocation)
  void kill() ;


  //---------------------------------
  // Set/get Matrix size
  //---------------------------------
  /** @name Set/get Matrix size  */
  //@{
  //! Return the number of rows of the matrix
  inline int getRows() const { return rowNum ;}
  //! Return the number of columns of the matrix
  inline int getCols() const { return colNum; }

  //! Set the size of the matrix A, initialization with a zero matrix
  virtual void resize(const int nrows, const int ncols, const bool nullify = true);
  //@}

  //---------------------------------
  // Printing
  //---------------------------------

  friend VISP_EXPORT std::ostream &operator << (std::ostream &s,const vpMatrix &m);
  /** @name Printing  */
  //@{

  int print(std::ostream& s, unsigned lenght, char const* intro=0);
  //! Affichage pour reinsertion dans matlab
  std::ostream & matlabPrint(std::ostream & os);
  //! Affichage pour reinsertion dans ViSP
  std::ostream & cppPrint(std::ostream & os, const char * matrixName = NULL, bool octet = false);

  void printSize() { std::cout << getRows() <<" x " << getCols() <<"  " ; }
  //@}
  
  static bool saveMatrix(const char *filename, const vpMatrix &M, const bool binary = false, const char *Header = "");
  static bool loadMatrix(const char *filename, vpMatrix &M, const bool binary = false, char *Header = NULL);

  //---------------------------------
  // Copy / assignment
  //---------------------------------
  /** @name Copy / assignment  */
  //@{
  //! Copy constructor
  vpMatrix (const vpMatrix& m);

  //! Assignment from an array
  vpMatrix &operator<<(double*);

  //! Copy operator.   Allow operation such as A = B
  vpMatrix &operator=(const vpMatrix &B);
  //! Set all the element of the matrix A to x
  vpMatrix &operator=(const double x);
  void diag(const vpColVector &A);
  //@}

  //---------------------------------
  // Access/modification operators
  //---------------------------------
  /** @name Access/modification operators  */
  //@{
  //! write elements Aij (usage : A[i][j] = x )
  inline double *operator[](int n) { return rowPtrs[n]; }
  //! read elements Aij  (usage : x = A[i][j] )
  inline double *operator[](int n) const {return rowPtrs[n];}
  //@}

  //---------------------------------
  // Matrix operations (Static).
  //---------------------------------

  static void mult2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C);
  static void add2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C);
  static void sub2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C);
  static void negateMatrix(const vpMatrix &A, vpMatrix &C);
  static void multMatrixVector(const vpMatrix &A, const vpColVector &b, vpColVector &c);

  //---------------------------------
  // Matrix operations.
  //---------------------------------
  /** @name Matrix operations  */
  //@{
  //! operation A = A + B
   vpMatrix &operator+=(const vpMatrix &B);
  //! operation A = A - B
   vpMatrix &operator-=(const vpMatrix &B);

   vpMatrix operator*(const vpMatrix &B) const;
   vpMatrix operator+(const vpMatrix &B) const;
   vpMatrix operator-(const vpMatrix &B) const;
   vpMatrix operator-() const;

  //---------------------------------
  // Matrix/vector operations.
  //---------------------------------

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

  //!return sum of the Aij^2 (for all i, for all j)
   double sumSquare() const;

  // return the determinant of the matrix.
  double det(vpDetMethod method = LU_DECOMPOSITION) const;
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /** @name Deprecated functions */
  //@{
  /*!
    \deprecated This method is deprecated. You should use vpMatrix::det() 
    instead.
  */
  double detByLU() const;
  //@}
#else
 private:
  double detByLU() const;
 public:
#endif

  //-------------------------------------------------
  // Columns, Rows extraction, SubMatrix
  //-------------------------------------------------
  /** @name Columns, Rows extraction, Submatrix  */
  //@{
  //! Row extraction
  vpRowVector row(const int i);
  //! Column extraction
  vpColVector column(const int j);
  //! subvpMatrix extraction
  void init(const vpMatrix &m, int r, int c, int nrows, int ncols);
  //@}

  //-------------------------------------------------
  // transpose, identity
  //-------------------------------------------------
  /** @name Transpose, Identity  */
  //@{
  //! Compute the transpose C = A^T
  vpMatrix t() const;

  //! Set the matrix to identity
  void setIdentity() ;

  //! Initialize an identity matrix n-by-n
  void eye(int n) ;
  //! Initialize an identity matrix m-by-n
   void eye(int m, int n) ;

  //! Compute the AtA operation B = A^T*A
  vpMatrix AtA() const;
  void AtA(vpMatrix &B) const;
  //@}

  //-------------------------------------------------
  // LU decomposition
  //-------------------------------------------------
  /** @name LU decomposition  */
  //@{
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //! LU Decomposition
  void LUDcmp(int* perm, int& d);
  //! solve AX = B using the LU Decomposition
  void LUBksb(int* perm, vpColVector& b);
#endif // doxygen should skip this

  // inverse matrix A using the LU decomposition 
  vpMatrix inverseByLU() const;
  //@}

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

  /** @name SVD decomposition  */
  //@{
  // singular value decomposition SVD

  void svd(vpColVector& w, vpMatrix& v);

  // solve Ax=B using the SVD decomposition (usage A = solveBySVD(B,x) )
  void solveBySVD(const vpColVector &B, vpColVector &x) const ;
  // solve Ax=B using the SVD decomposition (usage  x=A.solveBySVD(B))
  vpColVector solveBySVD(const vpColVector &B) const ;

  //! Compute the pseudo inverse of the matrix using the SVD.
  vpMatrix pseudoInverse(double svThreshold=1e-6)  const;
  //! Compute the pseudo inverse of the matrix using the SVD.
  //! return the rank
  int pseudoInverse(vpMatrix &Ap, double svThreshold=1e-6)  const;
  //! Compute the pseudo inverse of the matrix using the SVD.
  //! return the rank and the singular value
  int pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold=1e-6) const ;
  //! Compute the pseudo inverse of the matrix using the SVD.
  //! return the rank and the singular value, image
  int pseudoInverse(vpMatrix &Ap,
		    vpColVector &sv, double svThreshold,
		    vpMatrix &ImA,
		    vpMatrix &ImAt) const ;
  //! Compute the pseudo inverse of the matrix using the SVD.
  //! return the rank and the singular value, image, kernel.
  int pseudoInverse(vpMatrix &Ap,
		    vpColVector &sv, double svThreshold,
		    vpMatrix &ImA,
		    vpMatrix &ImAt,
		    vpMatrix &kerA) const ;

  int kernel(vpMatrix &KerA, double svThreshold=1e-6);
  //@}

  //-------------------------------------------------
  // Eigen values and vectors
  //-------------------------------------------------

  /** @name Eigen values  */

  //@{
  // compute the eigen values using the Gnu Scientific library
  vpColVector eigenValues();
  void eigenValues(vpColVector &evalue, vpMatrix &evector);
  //@}

  //! Stack two Matrices C = [ A B ]^T
  static vpMatrix stackMatrices(const vpMatrix &A,const  vpMatrix &B) ;
  //! Stack two Matrices C = [ A B ]^T
  static void stackMatrices(const vpMatrix &A,const  vpMatrix &B, vpMatrix &C) ;
  //! Juxtapose to matrices C = [ A B ]
  static vpMatrix juxtaposeMatrices(const vpMatrix &A,const  vpMatrix &B) ;
  //! Juxtapose to matrices C = [ A B ]
  static void juxtaposeMatrices(const vpMatrix &A,const  vpMatrix &B, vpMatrix &C) ;

  // Create a diagonal matrix with the element of a vector DAii = Ai
  static void createDiagonalMatrix(const vpColVector &A, vpMatrix &DA)  ;
  
  // Stack the matrix A below the current one, copy if not initialized this = [ this A ]^T
  void stackMatrices(const vpMatrix &A);


  // -------------------------
  // Norms
  // -------------------------
  /** @name Norms  */
  //@{
  // Euclidean norm ||x||=sqrt(sum(x_i^2))
  double euclideanNorm () const;
  // Infinity norm ||x||=max(sum(fabs(x_i)))
  double infinityNorm () const;
  //@}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  /** @name Deprecated functions */
  //@{
  static double det33(const vpMatrix &P) ;
  double euclidianNorm () const;
  // solve Ax=B using the SVD decomposition (usage  x=A.SVDsolve(B))
  vpColVector SVDsolve(const vpColVector& B) const ;
  //@}
#endif

};


//////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////


//! multiplication by a scalar C = x*A
VISP_EXPORT vpMatrix operator*(const double &x, const vpMatrix &A) ;

  //! multiplication by a scalar C = x*A
VISP_EXPORT vpColVector operator*(const double &x, const vpColVector &A) ;



#endif


/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
