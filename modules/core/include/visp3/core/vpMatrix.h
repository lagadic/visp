/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2015 by Inria. All rights reserved.
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

#ifndef vpMatrix_H
#define vpMatrix_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpVelocityTwistMatrix.h>
#include <visp3/core/vpForceTwistMatrix.h>

#ifdef VISP_HAVE_GSL
#  include <gsl/gsl_math.h>
#  include <gsl/gsl_eigen.h>
#endif

#include <iostream>
#include <math.h>

class vpRowVector;
class vpColVector;
class vpTranslationVector;
class vpHomogeneousMatrix;
class vpVelocityTwistMatrix;
class vpForceTwistMatrix;

/*!
  \file vpMatrix.h

  \brief Definition of matrix class as well as a set of operations on
  these matrices.
*/



/*!
  \class vpMatrix
  \ingroup group_core_matrices

  \brief Implementation of a matrix and operations on matrices.

  vpMatrix class provides a data structure for the matrices as well
  as a set of operations on these matrices.

  The vpMatrix class is derived from vpArray2D<double>.

  \warning Note the matrix in the class (*this) will be noted A in the comment

  \ingroup libmath

  \sa vpRowVector, vpColVector, vpHomogeneousMatrix, vpRotationMatrix,
  vpTwistMatrix, vpHomography
*/
class VISP_EXPORT vpMatrix : public vpArray2D<double>
{
 public:
  /*!
    Method used to compute the determinant of a square matrix.
    \sa det()
  */
  typedef enum {
    LU_DECOMPOSITION     /*!< LU decomposition method. */
  } vpDetMethod;

 public:
  /*!
    Basic constructor of a matrix of double. Number of columns and rows are zero.
  */
  vpMatrix() : vpArray2D<double>(0, 0) {};
  /*!
    Constructor that initialize a matrix of double with 0.

    \param r : Matrix number of rows.
    \param c : Matrix number of columns.
  */
  vpMatrix(unsigned int r, unsigned int c) : vpArray2D<double>(r, c) {};
  /*!
    Constructor that initialize a matrix of double with \e val.

    \param r : Matrix number of rows.
    \param c : Matrix number of columns.
    \param val : Each element of the matrix is set to \e val.
  */
  vpMatrix(unsigned int r, unsigned int c, double val) : vpArray2D<double>(r, c, val) {};
  vpMatrix(const vpMatrix &M, unsigned int r, unsigned int c,
           unsigned int nrows, unsigned int ncols) ;
  /*!
     Create a matrix from a 2D array that could be one of the following container that
     inherit from vpArray2D such as vpMatrix, vpRotationMatrix, vpHomogeneousMatrix,
     vpPoseVector, vpColVector, vpRowVector...

     The following example shows how to create a matrix from an homogeneous matrix:
     \code
     vpRotationMatrix R;
     vpMatrix M(R);
     \endcode
   */
  vpMatrix(const vpArray2D<double>& A) : vpArray2D<double>(A) {};

  //! Destructor (Memory de-allocation)
  virtual ~vpMatrix() {};

  /*!
    Removes all elements from the matrix (which are destroyed),
    leaving the container with a size of 0.
  */
  void clear()
  {
    if (data != NULL ) {
      free(data);
      data=NULL;
    }

    if (rowPtrs!=NULL) {
      free(rowPtrs);
      rowPtrs=NULL ;
    }
    rowNum = colNum = dsize = 0;
  }

  //-------------------------------------------------
  // Setting a diagonal matrix
  //-------------------------------------------------
  /** @name Setting a diagonal matrix  */
  //@{
  void diag(const double &val = 1.0);
  void diag(const vpColVector &A);
  // Initialize an identity matrix n-by-n
  void eye();
  void eye(unsigned int n) ;
  // Initialize an identity matrix m-by-n
  void eye(unsigned int m, unsigned int n) ;
  //@}
  
  //---------------------------------
  // Assignment
  //---------------------------------
  /** @name Assignment operators */
  //@{
  vpMatrix &operator<<(double*);
  vpMatrix &operator=(const vpArray2D<double> &A);
  vpMatrix &operator=(const double x);
  //@}

  //-------------------------------------------------
  // Stacking
  //-------------------------------------------------
  /** @name Stacking  */
  //@{
  // Stack the matrix A below the current one, copy if not initialized this = [ this A ]^T
  void stack(const vpMatrix &A);
  void stack(const vpRowVector &r);
  // Stacks columns of a matrix in a vector
  void stackColumns(vpColVector &out );

  // Stacks columns of a matrix in a vector
  vpColVector stackColumns();

  // Stacks columns of a matrix in a vector
  void stackRows(vpRowVector  &out );

  // Stacks columns of a matrix in a vector
  vpRowVector stackRows();
  //@}

  //---------------------------------
  // Matrix insertion with Static Public Member Functions
  //---------------------------------
  /** @name Matrix insertion */
  //@{
  // Insert matrix A in the current matrix at the given position (r, c).
  void insert(const vpMatrix&A, const unsigned int r, const unsigned int c);
  //@}

  //-------------------------------------------------
  // Columns, Rows extraction, SubMatrix
  //-------------------------------------------------
  /** @name Columns, rows, sub-matrices extraction */
  //@{
  vpRowVector getRow(const unsigned int i) const;
  vpRowVector getRow(const unsigned int i, const unsigned int j_begin, const unsigned int size) const;
  vpColVector getCol(const unsigned int j) const;
  vpColVector getCol(const unsigned int j, const unsigned int i_begin, const unsigned int size) const;
  void init(const vpMatrix &M, unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols);
  //@}

  //---------------------------------
  // Matrix operations.
  //---------------------------------
  /** @name Matrix operations  */
  //@{
  // operation A = A + B
  vpMatrix &operator+=(const vpMatrix &B);
  // operation A = A - B
  vpMatrix &operator-=(const vpMatrix &B);
  vpMatrix operator*(const vpMatrix &B) const;
  vpMatrix operator*(const vpRotationMatrix &R) const;
  vpMatrix operator*(const vpVelocityTwistMatrix &V) const;
  vpMatrix operator*(const vpForceTwistMatrix &V) const;
  // operation t_out = A * t (A is unchanged, t and t_out are translation vectors)
  vpTranslationVector operator*(const vpTranslationVector &tv) const;
  vpColVector operator*(const vpColVector &v) const;
  vpMatrix operator+(const vpMatrix &B) const;
  vpMatrix operator-(const vpMatrix &B) const;
  vpMatrix operator-() const;

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

  /*!
    Return the sum of all the \f$a_{ij}\f$ elements of the matrix.

    \return \f$\sum a_{ij}\f$
    */
  double sum() const;
  double sumSquare() const;
  // return the determinant of the matrix.
  double det(vpDetMethod method = LU_DECOMPOSITION) const;
  
  //Compute the exponential matrix of a square matrix
  vpMatrix expm() const;

  //-------------------------------------------------
  // Kronecker product
  //-------------------------------------------------
  /** @name Kronecker product  */
  //@{  
  // Compute Kronecker product matrix 
  void kron(const vpMatrix  &m1, vpMatrix  &out) const;
  
  // Compute Kronecker product matrix 
  vpMatrix kron(const vpMatrix  &m1) const;
  //@}
  
  //-------------------------------------------------
  // Transpose
  //-------------------------------------------------
  /** @name Transpose  */
  //@{
  // Compute the transpose C = A^T
  vpMatrix t() const;

  // Compute the transpose C = A^T
  vpMatrix transpose()const;
  void  transpose(vpMatrix & C )const;

  vpMatrix AAt() const;
  void AAt(vpMatrix &B) const;

  vpMatrix AtA() const;
  void AtA(vpMatrix &B) const;
  //@}

  //-------------------------------------------------
  // Matrix inversion
  //-------------------------------------------------
  /** @name Matrix inversion  */
  //@{
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  //! LU Decomposition
  void LUDcmp(unsigned int* perm, int& d);
  //! solve AX = B using the LU Decomposition
  void LUBksb(unsigned int* perm, vpColVector& b);

#endif // doxygen should skip this
  // inverse matrix A using the LU decomposition 
  vpMatrix inverseByLU() const;
#if defined(VISP_HAVE_LAPACK_C)
  // inverse matrix A using the Cholesky decomposition (only for real symmetric matrices)
  vpMatrix inverseByCholesky() const;
  //lapack implementation of inverse by Cholesky
  vpMatrix inverseByCholeskyLapack() const;
  // inverse matrix A using the QR decomposition
  vpMatrix inverseByQR() const;
  //lapack implementation of inverse by QR
  vpMatrix inverseByQRLapack() const;
#endif
  //! Compute the pseudo inverse of the matrix using the SVD.
  vpMatrix pseudoInverse(double svThreshold=1e-6)  const;
  //! Compute the pseudo inverse of the matrix using the SVD.
  //! return the rank
  unsigned int pseudoInverse(vpMatrix &Ap, double svThreshold=1e-6)  const;
  //! Compute the pseudo inverse of the matrix using the SVD.
  //! return the rank and the singular value
  unsigned int pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold=1e-6) const ;
  //! Compute the pseudo inverse of the matrix using the SVD.
  //! return the rank and the singular value, image
  unsigned int pseudoInverse(vpMatrix &Ap,
                             vpColVector &sv, double svThreshold,
                             vpMatrix &ImA,
                             vpMatrix &ImAt) const ;
  //! Compute the pseudo inverse of the matrix using the SVD.
  //! return the rank and the singular value, image, kernel.
  unsigned int pseudoInverse(vpMatrix &Ap,
                             vpColVector &sv, double svThreshold,
                             vpMatrix &ImA,
                             vpMatrix &ImAt,
                             vpMatrix &kerA) const ;
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
#if (VISP_HAVE_OPENCV_VERSION >= 0x020101) // Require opencv >= 2.1.1
  void svdOpenCV(vpColVector& w, vpMatrix& v);
#endif
#ifdef VISP_HAVE_LAPACK_C
  void svdLapack(vpColVector& W, vpMatrix& V);
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

  unsigned int kernel(vpMatrix &KerA, double svThreshold=1e-6) const;
  double cond() const;
  //@}

  //-------------------------------------------------
  // Eigen values and vectors
  //-------------------------------------------------
  /** @name Eigen values  */

  //@{
  // compute the eigen values using the Gnu Scientific library
  vpColVector eigenValues() const;
  void eigenValues(vpColVector &evalue, vpMatrix &evector) const;
  //@}

  //-------------------------------------------------
  // Norms
  //-------------------------------------------------
  /** @name Norms  */
  //@{
  double euclideanNorm() const;
  double infinityNorm() const;
  //@}

  //---------------------------------
  // Printing
  //---------------------------------
  /** @name Printing  */
  //@{
  int print(std::ostream& s, unsigned int length, char const* intro=0) const;
  std::ostream & matlabPrint(std::ostream & os) const;
  std::ostream & maplePrint(std::ostream & os) const;
  std::ostream & csvPrint(std::ostream & os) const;
  std::ostream & cppPrint(std::ostream & os, const char * matrixName = NULL, bool octet = false) const;
  void printSize() const { std::cout << getRows() <<" x " << getCols() <<"  " ; }
  //@}

  //------------------------------------------------------------------
  // Static functionalities
  //------------------------------------------------------------------

  //---------------------------------
  // Setting a diagonal matrix with Static Public Member Functions
  //---------------------------------
  /** @name Setting a diagonal matrix with Static Public Member Functions  */
  //@{
  // Create a diagonal matrix with the element of a vector DAii = Ai
  static void createDiagonalMatrix(const vpColVector &A, vpMatrix &DA)  ;
  //@}

  //---------------------------------
  // Matrix insertion with Static Public Member Functions
  //---------------------------------
  /** @name Matrix insertion with Static Public Member Functions  */
  //@{
  // Insert matrix B in matrix A at the given position (r, c).
  static vpMatrix insert(const vpMatrix &A,const  vpMatrix &B, const unsigned int r, const unsigned int c) ;
  // Insert matrix B in matrix A (not modified) at the given position (r, c), the result is given in matrix C.
  static void insert(const vpMatrix &A, const vpMatrix &B, vpMatrix &C, const unsigned int r, const unsigned int c) ;

  //---------------------------------
  // Stacking with Static Public Member Functions
  //---------------------------------
  /** @name Stacking with Static Public Member Functions  */
  //@{
  // Juxtapose to matrices C = [ A B ]
  static vpMatrix juxtaposeMatrices(const vpMatrix &A,const  vpMatrix &B) ;
  // Juxtapose to matrices C = [ A B ]
  static void juxtaposeMatrices(const vpMatrix &A,const  vpMatrix &B, vpMatrix &C) ;
  // Stack two matrices C = [ A B ]^T
  static vpMatrix stack(const vpMatrix &A, const vpMatrix &B) ;
  static vpMatrix stack(const vpMatrix &A, const vpRowVector &r) ;

  // Stack two matrices C = [ A B ]^T
  static void stack(const vpMatrix &A, const vpMatrix &B, vpMatrix &C);
  static void stack(const vpMatrix &A, const vpRowVector &r, vpMatrix &C);
  //@}

  //---------------------------------
  // Matrix operations Static Public Member Functions
  //---------------------------------
  /** @name Matrix operations with Static Public Member Functions  */
  //@{
  static void add2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C);
  static void add2Matrices(const vpColVector &A, const vpColVector &B, vpColVector &C);
  static void add2WeightedMatrices(const vpMatrix &A, const double &wA, const vpMatrix &B,const double &wB, vpMatrix &C);
  static void computeHLM(const vpMatrix &H, const double &alpha, vpMatrix &HLM);
  static void mult2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C);
  static void mult2Matrices(const vpMatrix &A, const vpMatrix &B, vpRotationMatrix &C);
  static void mult2Matrices(const vpMatrix &A, const vpMatrix &B, vpHomogeneousMatrix &C);
  static void mult2Matrices(const vpMatrix &A, const vpColVector &B, vpColVector &C);
  static void multMatrixVector(const vpMatrix &A, const vpColVector &v, vpColVector &w);
  static void negateMatrix(const vpMatrix &A, vpMatrix &C);
  static void sub2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C);
  static void sub2Matrices(const vpColVector &A, const vpColVector &B, vpColVector &C);
  //@}

  //---------------------------------
  // Kronecker product Static Public Member Functions
  //---------------------------------
  /** @name Kronecker product with Static Public Member Functions  */
  //@{
  // Compute Kronecker product matrix
  static void kron(const vpMatrix  &m1, const vpMatrix  &m2 , vpMatrix  &out);

  // Compute Kronecker product matrix
  static vpMatrix kron(const vpMatrix  &m1, const vpMatrix  &m2 );
  //@}

  //---------------------------------
  // Covariance computation Static Public Member Functions
  //---------------------------------
  /** @name Covariance computation with Static Public Member Functions  */
  //@{
  static vpMatrix computeCovarianceMatrix(const vpMatrix &A, const vpColVector &x, const vpColVector &b);
  static vpMatrix computeCovarianceMatrix(const vpMatrix &A, const vpColVector &x, const vpColVector &b, const vpMatrix &w);
  static vpMatrix computeCovarianceMatrixVVS(const vpHomogeneousMatrix &cMo, const vpColVector &deltaS, const vpMatrix &Ls, const vpMatrix &W);
  static vpMatrix computeCovarianceMatrixVVS(const vpHomogeneousMatrix &cMo, const vpColVector &deltaS, const vpMatrix &Ls);
  //@}

  //---------------------------------
  // Matrix I/O  Static Public Member Functions
  //---------------------------------
  /** @name Matrix I/O with Static Public Member Functions  */
  //@{
  /*!
    Load a matrix from a file. This function overloads vpArray2D::load().

    \param filename : absolute file name.
    \param M : matrix to be loaded.
    \param binary :If true the matrix is loaded from a binary file, else from a text file.
    \param header : Header of the file is loaded in this parameter

    \return Returns true if no problem appends.
  */
  static inline bool loadMatrix(const std::string &filename, vpArray2D<double> &M,
                                const bool binary = false, char *header = NULL)
  {
    return vpArray2D<double>::load(filename, M, binary, header);
  }

  /*!
    Load a matrix from a YAML-formatted file. This function overloads vpArray2D::loadYAML().

    \param filename : absolute file name.
    \param M : matrix to be loaded from the file.
    \param header : Header of the file is loaded in this parameter.

    \return Returns true if no problem appends.
  */
  static inline bool loadMatrixYAML(const std::string &filename, vpArray2D<double> &M, char *header = NULL)
  {
    return vpArray2D<double>::loadYAML(filename, M, header);
  }

  /*!
    Save a matrix to a file. This function overloads vpArray2D::load().

    \param filename : absolute file name.
    \param M : matrix to be saved.
    \param binary : If true the matrix is save in a binary file, else a text file.
    \param header : optional line that will be saved at the beginning of the file.

    \return Returns true if no problem appends.

    Warning : If you save the matrix as in a text file the precision is less than if you save it in a binary file.
  */
  static inline bool saveMatrix(const std::string &filename, const vpArray2D<double> &M,
                                const bool binary = false,
                                const char *header = "")
  {
    return vpArray2D<double>::save(filename, M, binary, header);
  }

  /*!
    Save a matrix in a YAML-formatted file. This function overloads vpArray2D::saveYAML().

    \param filename : absolute file name.
    \param M : matrix to be saved in the file.
    \param header : optional lines that will be saved at the beginning of the file. Should be YAML-formatted and will adapt to the indentation if any.

    \return Returns true if success.

  */
  static inline bool saveMatrixYAML(const std::string &filename, const vpArray2D<double> &M, const char *header = "")
  {
    return vpArray2D<double>::saveYAML(filename, M, header);
  }
  //@}


#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
     \deprecated Only provided for compatibilty with ViSP previous releases. This function does nothing.
   */
  vp_deprecated void init() { };
  /*!
     \deprecated You should rather use stack(const vpMatrix &A)
   */
  vp_deprecated void stackMatrices(const vpMatrix &A) { stack(A); };
  /*!
     \deprecated You should rather use stack(const vpMatrix &A, const vpMatrix &B)
   */
  vp_deprecated static vpMatrix stackMatrices(const vpMatrix &A, const vpMatrix &B) { return vpMatrix::stack(A, B); };
  /*!
     \deprecated You should rather use stack(const vpMatrix &A, const vpMatrix &B, vpMatrix &C)
   */
  vp_deprecated static void stackMatrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C) { vpMatrix::stack(A, B, C); };
  /*!
     \deprecated You should rather use stack(const vpMatrix &A, const vpMatrix &B)
   */
  vp_deprecated static vpMatrix stackMatrices(const vpMatrix &A, const vpRowVector &B);
  /*!
     \deprecated You should rather use stack(const vpMatrix &A, const vpRowVector &B, vpMatrix &C)
   */
  vp_deprecated static void stackMatrices(const vpMatrix &A, const vpRowVector &B, vpMatrix &C);
  /*!
     \deprecated You should rather use vpColVector::stack(const vpColVector &A, const vpColVector &B)
   */
  vp_deprecated static vpMatrix stackMatrices(const vpColVector &A, const vpColVector &B);
  /*!
     \deprecated You should rather use vpColVector::stack(const vpColVector &A, const vpColVector &B, vpColVector &C)
   */
  vp_deprecated static void stackMatrices(const vpColVector &A, const vpColVector &B, vpColVector &C);

  /*!
     \deprecated You should rather use diag(const double &)
   */
  vp_deprecated void setIdentity(const double & val=1.0) ;
  //@}
#endif

 private:
  double detByLU() const;
  static void computeCovarianceMatrixVVS(const vpHomogeneousMatrix &cMo, const vpColVector &deltaS, const vpMatrix &Ls, vpMatrix &Js, vpColVector &deltaP);

};


//////////////////////////////////////////////////////////////////////////

#ifndef DOXYGEN_SHOULD_SKIP_THIS
VISP_EXPORT
#endif
vpMatrix operator*(const double &x, const vpMatrix &A) ;

#endif
