/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2024 by Inria. All rights reserved.
 *
 * This software is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * See the file LICENSE.txt at the root directory of this source
 * distribution for additional information about the GNU GPL.
 *
 * For using ViSP with software that can not be combined with the GNU
 * GPL, please contact Inria about acquiring a ViSP Professional
 * Edition License.
 *
 * See https://visp.inria.fr for more information.
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
 */

/*!
  \file vpMatrix.h

  \brief Definition of matrix class as well as a set of operations on
  these matrices.
*/

#ifndef VP_MATRIX_H
#define VP_MATRIX_H

#include <visp3/core/vpConfig.h>

BEGIN_VISP_NAMESPACE
class vpRowVector;
class vpColVector;
class vpTranslationVector;
class vpHomogeneousMatrix;
class vpVelocityTwistMatrix;
class vpForceTwistMatrix;
END_VISP_NAMESPACE

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpForceTwistMatrix.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTime.h>
#include <visp3/core/vpVelocityTwistMatrix.h>

#include <iostream>
#include <math.h>

BEGIN_VISP_NAMESPACE

/*!
  \class vpMatrix
  \ingroup group_core_matrices

  \brief Implementation of a matrix and operations on matrices.

  This class needs one of the following third-party to compute matrix inverse,
  pseudo-inverse, singular value decomposition, determinant:
  - If Lapack is installed and detected by ViSP, this 3rd party is used by
    vpMatrix. Installation instructions are provided here
    https://visp.inria.fr/3rd_lapack;
  - else if Eigen3 is installed and detected by ViSP, this 3rd party is used
    by vpMatrix. Installation instructions are provided here
    https://visp.inria.fr/3rd_eigen;
  - else if OpenCV is installed and detected by ViSP, this 3rd party is used,
    Installation instructions are provided here
    https://visp.inria.fr/3rd_opencv;
  - If none of these previous 3rd parties is installed, we use by default a
    Lapack built-in version.

  vpMatrix class provides a data structure for the matrices as well
  as a set of operations on these matrices.

  The vpMatrix class is derived from vpArray2D<double>.

  The code below shows how to create a 2-by-3 matrix of doubles, set the element values and access them:
  \code
  #include <visp3/code/vpMatrix.h

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix M(2, 3);
    M[0][0] = -1; M[0][1] =  -2; M[0][2] = -3;
    M[1][0] =  4; M[1][1] = 5.5; M[1][2] =  6.0f;

    std::cout << "M:" << std::endl;
    for (unsigned int i = 0; i < M.getRows(); ++i) {
      for (unsigned int j = 0; j < M.getCols(); ++j) {
        std::cout << M[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }
  \endcode
  Once build, this previous code produces the following output:
  \code
  M:
  -1 -2 -3
  4 5.5 6
  \endcode
  If ViSP is build with c++11 enabled, you can do the same using:
  \code
  #include <visp3/code/vpMatrix.h

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix M( {-1, -2, -3}, {4, 5.5, 6.0f} );
    std::cout << "M:\n" << M << std::endl;
  }
  \endcode
  You can also create and initialize a matrix this way:
  \code
  #include <visp3/code/vpMatrix.h

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix M(2, 3, {-1, -2, -3, 4, 5.5, 6.0f} );
  }
  \endcode

  The Matrix could also be initialized using operator=(const std::initializer_list< std::initializer_list< double > > &)
  \code
  #include <visp3/code/vpMatrix.h

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpMatrix M;
    M = { {-1, -2, -3}, {4, 5.5, 6.0f} };
  }
  \endcode

  \sa vpArray2D, vpRowVector, vpColVector, vpHomogeneousMatrix,
  vpRotationMatrix, vpVelocityTwistMatrix, vpForceTwistMatrix, vpHomography
*/
class VISP_EXPORT vpMatrix : public vpArray2D<double>
{
public:
  /*!
    Method used to compute the determinant of a square matrix.
    \sa det()
  */
  typedef enum
  {
    LU_DECOMPOSITION /*!< LU decomposition method. */
  } vpDetMethod;

public:
  /*!
    Basic constructor of a matrix of double. Number of columns and rows are
    zero.
  */
  vpMatrix() : vpArray2D<double>(0, 0) { }

  /*!
    Constructor that initialize a matrix of double with 0.

    \param r : Matrix number of rows.
    \param c : Matrix number of columns.
  */
  vpMatrix(unsigned int r, unsigned int c) : vpArray2D<double>(r, c) { }

  /*!
    Constructor that initialize a matrix of double with \e val.

    \param r : Matrix number of rows.
    \param c : Matrix number of columns.
    \param val : Each element of the matrix is set to \e val.
  */
  vpMatrix(unsigned int r, unsigned int c, double val) : vpArray2D<double>(r, c, val) { }
  vpMatrix(const vpMatrix &M, unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols);

  /*!
    Create a matrix from a 2D array that could be one of the following
    container that inherit from vpArray2D such as vpMatrix, vpRotationMatrix,
    vpHomogeneousMatrix, vpPoseVector, vpColVector, vpRowVector...

    The following example shows how to create a matrix from an homogeneous
    matrix:
    \code
    vpRotationMatrix R;
    vpMatrix M(R);
    \endcode
   */
  VP_EXPLICIT vpMatrix(const vpArray2D<double> &A) : vpArray2D<double>(A) { }
  vpMatrix(const vpMatrix &A) : vpArray2D<double>(A) { }
  VP_EXPLICIT vpMatrix(const vpHomogeneousMatrix &R);
  VP_EXPLICIT vpMatrix(const vpRotationMatrix &R);
  VP_EXPLICIT vpMatrix(const vpVelocityTwistMatrix &V);
  VP_EXPLICIT vpMatrix(const vpForceTwistMatrix &F);
  VP_EXPLICIT vpMatrix(const vpColVector &v);
  VP_EXPLICIT vpMatrix(const vpRowVector &v);
  VP_EXPLICIT vpMatrix(const vpTranslationVector &t);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpMatrix(vpMatrix &&A);
  VP_EXPLICIT vpMatrix(const std::initializer_list<double> &list);
  VP_EXPLICIT vpMatrix(unsigned int nrows, unsigned int ncols, const std::initializer_list<double> &list);
  VP_EXPLICIT vpMatrix(const std::initializer_list<std::initializer_list<double> > &lists);
#endif

  /*!
    Removes all elements from the matrix (which are destroyed),
    leaving the container with a size of 0.
  */
  void clear()
  {
    if (data != nullptr) {
      free(data);
      data = nullptr;
    }

    if (rowPtrs != nullptr) {
      free(rowPtrs);
      rowPtrs = nullptr;
    }
    rowNum = 0;
    colNum = 0;
    dsize = 0;
  }

  //-------------------------------------------------
  // Setting a diagonal matrix
  //-------------------------------------------------
  /** @name Linear algebra optimization  */
  //@{
  /*!
   * Return the minimum size of rows and columns required to enable Blas/Lapack
   * usage on matrices and vectors.
   *
   * To get more info see \ref tutorial-basic-linear-algebra.
   *
   * \sa setLapackMatrixMinSize()
   */
  static unsigned int getLapackMatrixMinSize() { return m_lapack_min_size; }

  /*!
   * Modify default size used to determine if Blas/Lapack basic linear algebra operations are enabled.
   *
   * To get more info see \ref tutorial-basic-linear-algebra.
   *
   * \param min_size : Minimum size of rows and columns required for a matrix or a vector to use
   * Blas/Lapack third parties like MKL, OpenBLAS, Netlib or Atlas. When matrix or vector size is
   * lower or equal to this parameter, Blas/Lapack is not used. In that case we prefer use naive code
   * that runs faster for small matrices.
   *
   * \sa getLapackMatrixMinSize()
   */
  static void setLapackMatrixMinSize(unsigned int min_size) { m_lapack_min_size = min_size; }
  //@}

  //-------------------------------------------------
  // Setting a diagonal matrix
  //-------------------------------------------------
  /** @name Setting a diagonal matrix  */
  //@{
  void diag(const double &val = 1.0);
  void diag(const vpColVector &A);
  // Initialize an identity matrix n-by-n
  void eye();
  void eye(unsigned int n);
  // Initialize an identity matrix m-by-n
  void eye(unsigned int m, unsigned int n);
  //@}

  //---------------------------------
  // Assignment
  //---------------------------------
  /** @name Assignment operators */
  //@{
  vpMatrix &operator<<(double *p);
  vpMatrix &operator<<(double val);
  vpMatrix &operator,(double val);
  vpMatrix &operator=(const vpArray2D<double> &A);
  vpMatrix &operator=(const vpMatrix &A);
  vpMatrix &operator=(const vpHomogeneousMatrix &M);
  vpMatrix &operator=(const vpRotationMatrix &R);
  vpMatrix &operator=(const vpVelocityTwistMatrix &V);
  vpMatrix &operator=(const vpForceTwistMatrix &F);
  vpMatrix &operator=(const vpColVector &v);
  vpMatrix &operator=(const vpRowVector &v);
  vpMatrix &operator=(const vpTranslationVector &t);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpMatrix &operator=(vpMatrix &&A);

  vpMatrix &operator=(const std::initializer_list<double> &list);
  vpMatrix &operator=(const std::initializer_list<std::initializer_list<double> > &lists);
#endif
  vpMatrix &operator=(double x);
  //@}

  //-------------------------------------------------
  // Stacking
  //-------------------------------------------------
  /** @name Stacking  */
  //@{
  // Stack the matrix A below the current one, copy if not initialized this =
  // [ this A ]^T
  void stack(const vpMatrix &A);
  void stack(const vpRowVector &r);
  void stack(const vpColVector &c);
  // Stacks columns of a matrix in a vector
  void stackColumns(vpColVector &out);

  // Stacks columns of a matrix in a vector
  vpColVector stackColumns();

  // Stacks columns of a matrix in a vector
  void stackRows(vpRowVector &out);

  // Stacks columns of a matrix in a vector
  vpRowVector stackRows();
  //@}

  //---------------------------------
  // Matrix insertion
  //---------------------------------
  /** @name Matrix insertion */
  //@{
  // Insert matrix A in the current matrix at the given position (r, c).
  void insert(const vpMatrix &A, unsigned int r, unsigned int c);
  //@}

  //-------------------------------------------------
  // Columns, Rows, Diag extraction, SubMatrix
  //-------------------------------------------------
  /** @name Columns, rows, sub-matrices extraction */
  //@{
  vpMatrix extract(unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols) const;
  vpColVector getCol(unsigned int j) const;
  vpColVector getCol(unsigned int j, unsigned int i_begin, unsigned int size) const;
  vpRowVector getRow(unsigned int i) const;
  vpRowVector getRow(unsigned int i, unsigned int j_begin, unsigned int size) const;
  vpColVector getDiag() const;
  void init(const vpMatrix &M, unsigned int r, unsigned int c, unsigned int nrows, unsigned int ncols);
  //@}

  //---------------------------------
  // Matrix operations.
  //---------------------------------
  /** @name Matrix operations  */
  //@{
  /*!
    Perform a 2D convolution similar to Matlab conv2 function: \f$ M \star kernel \f$.

    \param M : First matrix.
    \param kernel : Second matrix.
    \param mode : Convolution mode: "full" (default), "same", "valid".

    \image html vpMatrix-conv2-mode.jpg "Convolution mode: full, same, valid (image credit: Theano doc)."

    \note This is a very basic implementation that does not use FFT.
  */
  static vpMatrix conv2(const vpMatrix &M, const vpMatrix &kernel, const std::string &mode);

  /*!
    Perform a 2D convolution similar to Matlab conv2 function: \f$ M \star kernel \f$.

    \param M : First array.
    \param kernel : Second array.
    \param res : Result.
    \param mode : Convolution mode: "full" (default), "same", "valid".

    \image html vpMatrix-conv2-mode.jpg "Convolution mode: full, same, valid (image credit: Theano doc)."

    \note This is a very basic implementation that does not use FFT.
  */
  static void conv2(const vpMatrix &M, const vpMatrix &kernel, vpMatrix &res, const std::string &mode);

  // return the determinant of the matrix.
  double det(vpDetMethod method = LU_DECOMPOSITION) const;
  double detByLU() const;
#if defined(VISP_HAVE_EIGEN3)
  double detByLUEigen3() const;
#endif
#if defined(VISP_HAVE_LAPACK)
  double detByLULapack() const;
#endif
#if defined(VISP_HAVE_OPENCV)
  double detByLUOpenCV() const;
#endif
  vpMatrix cholesky() const;
#if defined(VISP_HAVE_EIGEN3)
  vpMatrix choleskyByEigen3() const;
#endif
#if defined(VISP_HAVE_LAPACK)
  vpMatrix choleskyByLapack() const;
#endif
#if defined(VISP_HAVE_OPENCV)
  vpMatrix choleskyByOpenCV() const;
#endif

  // Compute the exponential matrix of a square matrix
  vpMatrix expm() const;

  // operation A = A + B
  vpMatrix &operator+=(const vpMatrix &B);
  // operation A = A - B
  vpMatrix &operator-=(const vpMatrix &B);
  vpMatrix operator*(const vpMatrix &B) const;
  vpMatrix operator*(const vpRotationMatrix &R) const;
  vpMatrix operator*(const vpHomogeneousMatrix &R) const;
  vpMatrix operator*(const vpVelocityTwistMatrix &V) const;
  vpMatrix operator*(const vpForceTwistMatrix &V) const;
  // operation t_out = A * t (A is unchanged, t and t_out are translation
  // vectors)
  vpTranslationVector operator*(const vpTranslationVector &tv) const;
  vpColVector operator*(const vpColVector &v) const;
  vpMatrix operator+(const vpMatrix &B) const;
  vpMatrix operator-(const vpMatrix &B) const;
  vpMatrix operator-() const;

  //! Add x to all the element of the matrix : Aij = Aij + x
  vpMatrix &operator+=(double x);
  //! subtract x to all the element of the matrix : Aij = Aij - x
  vpMatrix &operator-=(double x);
  //! Multiply  all the element of the matrix by x : Aij = Aij * x
  vpMatrix &operator*=(double x);
  //! Divide  all the element of the matrix by x : Aij = Aij / x
  vpMatrix &operator/=(double x);

  // Cij = Aij * x (A is unchanged)
  vpMatrix operator*(double x) const;
  // Cij = Aij / x (A is unchanged)
  vpMatrix operator/(double x) const;

  /*!
    Return the sum of all the \f$a_{ij}\f$ elements of the matrix.

    \return Value of \f$\sum a_{ij}\f$
    */
  double sum() const;
  double sumSquare() const;

  //-------------------------------------------------
  // Hadamard product
  //-------------------------------------------------
  /** @name Hadamard product  */
  vpMatrix hadamard(const vpMatrix &m) const;

  //-------------------------------------------------
  // Kronecker product
  //-------------------------------------------------
  /** @name Kronecker product  */
  //@{
  // Compute Kronecker product matrix
  void kron(const vpMatrix &m1, vpMatrix &out) const;

  // Compute Kronecker product matrix
  vpMatrix kron(const vpMatrix &m1) const;
  //@}

  //-------------------------------------------------
  // Transpose
  //-------------------------------------------------
  /** @name Transpose  */
  //@{
  // Compute the transpose C = A^T
  vpMatrix t() const;

  // Compute the transpose C = A^T
  vpMatrix transpose() const;
  void transpose(vpMatrix &At) const;

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
  // inverse matrix A using the LU decomposition
  vpMatrix inverseByLU() const;

#if defined(VISP_HAVE_EIGEN3)
  vpMatrix inverseByLUEigen3() const;
#endif
#if defined(VISP_HAVE_LAPACK)
  vpMatrix inverseByLULapack() const;
#endif
#if defined(VISP_HAVE_OPENCV)
  vpMatrix inverseByLUOpenCV() const;
#endif

  // inverse matrix A using the Cholesky decomposition (only for real
  // symmetric matrices)
  vpMatrix inverseByCholesky() const;

#if defined(VISP_HAVE_LAPACK)
  vpMatrix inverseByCholeskyLapack() const;
#endif
#if defined(VISP_HAVE_OPENCV)
  vpMatrix inverseByCholeskyOpenCV() const;
#endif

  // inverse matrix A using the QR decomposition
  vpMatrix inverseByQR() const;
#if defined(VISP_HAVE_LAPACK)
  vpMatrix inverseByQRLapack() const;
#endif

  // inverse triangular matrix
  vpMatrix inverseTriangular(bool upper = true) const;

  vpMatrix pseudoInverse(double svThreshold = 1e-6) const;
  unsigned int pseudoInverse(vpMatrix &Ap, double svThreshold = 1e-6) const;
  unsigned int pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold = 1e-6) const;
  unsigned int pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA, vpMatrix &imAt) const;
  unsigned int pseudoInverse(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA, vpMatrix &imAt, vpMatrix &kerAt) const;
  vpMatrix pseudoInverse(int rank_in) const;
  int pseudoInverse(vpMatrix &Ap, int rank_in) const;
  int pseudoInverse(vpMatrix &Ap, vpColVector &sv, int rank_in) const;
  int pseudoInverse(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA, vpMatrix &imAt) const;
  int pseudoInverse(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA, vpMatrix &imAt, vpMatrix &kerAt) const;

#if defined(VISP_HAVE_LAPACK)
  vpMatrix pseudoInverseLapack(double svThreshold = 1e-6) const;
  unsigned int pseudoInverseLapack(vpMatrix &Ap, double svThreshold = 1e-6) const;
  unsigned int pseudoInverseLapack(vpMatrix &Ap, vpColVector &sv, double svThreshold = 1e-6) const;
  unsigned int pseudoInverseLapack(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA, vpMatrix &imAt, vpMatrix &kerAt) const;
  vpMatrix pseudoInverseLapack(int rank_in) const;
  int pseudoInverseLapack(vpMatrix &Ap, int rank_in) const;
  int pseudoInverseLapack(vpMatrix &Ap, vpColVector &sv, int rank_in) const;
  int pseudoInverseLapack(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA, vpMatrix &imAt, vpMatrix &kerAt) const;
#endif
#if defined(VISP_HAVE_EIGEN3)
  vpMatrix pseudoInverseEigen3(double svThreshold = 1e-6) const;
  unsigned int pseudoInverseEigen3(vpMatrix &Ap, double svThreshold = 1e-6) const;
  unsigned int pseudoInverseEigen3(vpMatrix &Ap, vpColVector &sv, double svThreshold = 1e-6) const;
  unsigned int pseudoInverseEigen3(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA, vpMatrix &imAt, vpMatrix &kerAt) const;
  vpMatrix pseudoInverseEigen3(int rank_in) const;
  int pseudoInverseEigen3(vpMatrix &Ap, int rank_in) const;
  int pseudoInverseEigen3(vpMatrix &Ap, vpColVector &sv, int rank_in) const;
  int pseudoInverseEigen3(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA, vpMatrix &imAt, vpMatrix &kerAt) const;
#endif
#if defined(VISP_HAVE_OPENCV)
  vpMatrix pseudoInverseOpenCV(double svThreshold = 1e-6) const;
  unsigned int pseudoInverseOpenCV(vpMatrix &Ap, double svThreshold = 1e-6) const;
  unsigned int pseudoInverseOpenCV(vpMatrix &Ap, vpColVector &sv, double svThreshold = 1e-6) const;
  unsigned int pseudoInverseOpenCV(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA, vpMatrix &imAt, vpMatrix &kerAt) const;
  vpMatrix pseudoInverseOpenCV(int rank_in) const;
  int pseudoInverseOpenCV(vpMatrix &Ap, int rank_in) const;
  int pseudoInverseOpenCV(vpMatrix &Ap, vpColVector &sv, int rank_in) const;
  int pseudoInverseOpenCV(vpMatrix &Ap, vpColVector &sv, int rank_in, vpMatrix &imA, vpMatrix &imAt, vpMatrix &kerAt) const;
#endif
  //@}

  //-------------------------------------------------
  // SVD decomposition
  //-------------------------------------------------

  /** @name SVD decomposition  */
  //@{
  double cond(double svThreshold = 1e-6) const;
  unsigned int kernel(vpMatrix &kerAt, double svThreshold = 1e-6) const;
  unsigned int nullSpace(vpMatrix &kerA, double svThreshold = 1e-6) const;
  unsigned int nullSpace(vpMatrix &kerA, int dim) const;

  // solve Ax=B using the SVD decomposition (usage A = solveBySVD(B,x) )
  void solveBySVD(const vpColVector &B, vpColVector &x) const;
  // solve Ax=B using the SVD decomposition (usage  x=A.solveBySVD(B))
  vpColVector solveBySVD(const vpColVector &B) const;

  // singular value decomposition SVD
  void svd(vpColVector &w, vpMatrix &V);
#ifdef VISP_HAVE_EIGEN3
  void svdEigen3(vpColVector &w, vpMatrix &V);
#endif
#if defined(VISP_HAVE_LAPACK)
  void svdLapack(vpColVector &w, vpMatrix &V);
#endif
#if defined(VISP_HAVE_OPENCV) // Require opencv >= 2.1.1
  void svdOpenCV(vpColVector &w, vpMatrix &V);
#endif
  //@}

  //-------------------------------------------------
  // QR decomposition
  //-------------------------------------------------

  /** @name QR decomposition  */
  //@{
  unsigned int qr(vpMatrix &Q, vpMatrix &R, bool full = false, bool squareR = false, double tol = 1e-6) const;
  unsigned int qrPivot(vpMatrix &Q, vpMatrix &R, vpMatrix &P, bool full = false, bool squareR = false,
                       double tol = 1e-6) const;
  void solveByQR(const vpColVector &b, vpColVector &x) const;
  vpColVector solveByQR(const vpColVector &b) const;
  //@}

  //-------------------------------------------------
  // Eigen values and vectors
  //-------------------------------------------------
  /** @name Eigen values  */

  //@{
  // Compute the eigen values using Lapack.
  vpColVector eigenValues() const;
  void eigenValues(vpColVector &evalue, vpMatrix &evector) const;
  //@}

  //-------------------------------------------------
  // Norms
  //-------------------------------------------------
  /** @name Norms  */
  //@{
  double frobeniusNorm() const;
  double inducedL2Norm() const;
  double infinityNorm() const;
  //@}

  //---------------------------------
  // Printing
  //---------------------------------
  /** @name Printing  */
  //@{
  std::ostream &cppPrint(std::ostream &os, const std::string &matrixName = "A", bool octet = false) const;
  std::ostream &csvPrint(std::ostream &os) const;
  std::ostream &maplePrint(std::ostream &os) const;
  std::ostream &matlabPrint(std::ostream &os) const;
  int print(std::ostream &s, unsigned int length, const std::string &intro = "") const;
  void printSize() const { std::cout << getRows() << " x " << getCols() << "  "; }
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
  static void createDiagonalMatrix(const vpColVector &A, vpMatrix &DA);
  //@}

  //---------------------------------
  // Matrix insertion with Static Public Member Functions
  //---------------------------------
  /** @name Matrix insertion with Static Public Member Functions  */
  //@{
  // Insert matrix B in matrix A at the given position (r, c).
  static vpMatrix insert(const vpMatrix &A, const vpMatrix &B, unsigned int r, unsigned int c);
  // Insert matrix B in matrix A (not modified) at the given position (r, c),
  // the result is given in matrix C.
  static void insert(const vpMatrix &A, const vpMatrix &B, vpMatrix &C, unsigned int r, unsigned int c);

  //---------------------------------
  // Stacking with Static Public Member Functions
  //---------------------------------
  /** @name Stacking with Static Public Member Functions  */
  //@{
  // Juxtapose to matrices C = [ A B ]
  static vpMatrix juxtaposeMatrices(const vpMatrix &A, const vpMatrix &B);
  // Juxtapose to matrices C = [ A B ]
  static void juxtaposeMatrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C);
  // Stack two matrices C = [ A B ]^T
  static vpMatrix stack(const vpMatrix &A, const vpMatrix &B);
  static vpMatrix stack(const vpMatrix &A, const vpRowVector &r);
  static vpMatrix stack(const vpMatrix &A, const vpColVector &c);

  // Stack two matrices C = [ A B ]^T
  static void stack(const vpMatrix &A, const vpMatrix &B, vpMatrix &C);
  static void stack(const vpMatrix &A, const vpRowVector &r, vpMatrix &C);
  static void stack(const vpMatrix &A, const vpColVector &c, vpMatrix &C);
  //@}

  //---------------------------------
  // Matrix operations Static Public Member Functions
  //---------------------------------
  /** @name Matrix operations with Static Public Member Functions  */
  //@{
  static void add2Matrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C);
  static void add2Matrices(const vpColVector &A, const vpColVector &B, vpColVector &C);
  static void add2WeightedMatrices(const vpMatrix &A, const double &wA, const vpMatrix &B, const double &wB,
                                   vpMatrix &C);
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
  static void kron(const vpMatrix &m1, const vpMatrix &m2, vpMatrix &out);

  // Compute Kronecker product matrix
  static vpMatrix kron(const vpMatrix &m1, const vpMatrix &m2);
  //@}

  //---------------------------------
  // Covariance computation Static Public Member Functions
  //---------------------------------
  /** @name Covariance computation with Static Public Member Functions  */
  //@{
  static vpMatrix computeCovarianceMatrix(const vpMatrix &A, const vpColVector &x, const vpColVector &b);
  static vpMatrix computeCovarianceMatrix(const vpMatrix &A, const vpColVector &x, const vpColVector &b,
                                          const vpMatrix &w);
  static vpMatrix computeCovarianceMatrixVVS(const vpHomogeneousMatrix &cMo, const vpColVector &deltaS,
                                             const vpMatrix &Ls, const vpMatrix &W);
  static vpMatrix computeCovarianceMatrixVVS(const vpHomogeneousMatrix &cMo, const vpColVector &deltaS,
                                             const vpMatrix &Ls);
  //@}

  //---------------------------------
  // Matrix I/O  Static Public Member Functions
  //---------------------------------
  /** @name Matrix I/O with Static Public Member Functions  */
  //@{
  /*!
    Load a matrix from a file. This function overloads vpArray2D::load().

    \param filename : Absolute file name.
    \param M : Matrix to be loaded.
    \param binary : If true the matrix data are considered as binary, otherwise as human readable (text) data. Using
    binary data allows to keep data precision.
    \param header : Header of the file is loaded in this parameter.

    \return Returns true if success, false otherwise.

    The following example shows how to use this function:
    \code
    #include <visp3/core/vpMatrix.h>

    #ifdef ENABLE_VISP_NAMESPACE
    using namespace VISP_NAMESPACE_NAME;
    #endif

    int main()
    {
      std::string filename("matrix.bin");
      bool binary_data = true;
      {
        vpMatrix M(2, 3);
        M[0][0] = -1; M[0][1] =  -2; M[0][2] = -3;
        M[1][0] =  4; M[1][1] = 5.5; M[1][2] =  6.0f;

        std::string header("My header");

        if (vpMatrix::saveMatrix(filename, M, binary_data, header.c_str())) {
          std::cout << "Matrix saved in " << filename << std::endl;
          M.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot save matrix in " << filename << std::endl;
        }
      }
      {
        vpMatrix N;
        char header[FILENAME_MAX];
        if (vpMatrix::loadMatrix(filename, N, binary_data, header)) {
          std::cout << "Matrix loaded from " << filename << std::endl;
          N.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot load matrix from " << filename << std::endl;
        }
      }
    }
    \endcode

    The output of this example is the following:
    \verbatim
    Matrix saved in matrix.bin
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    Matrix loaded from matrix.bin
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    \endverbatim

    And the content of `matrix.bin` file where data are saved as binary data is the following:
    \verbatim
    % cat matrix.bin
    My header??@@@%
    \endverbatim

    \sa saveMatrix(), saveMatrixYAML(), loadMatrixYAML()
  */
  static inline bool loadMatrix(const std::string &filename, vpArray2D<double> &M, bool binary = false,
                                char *header = nullptr)
  {
    return vpArray2D<double>::load(filename, M, binary, header);
  }

  /*!
    Load a matrix from a YAML-formatted file. This function overloads
    vpArray2D::loadYAML().

    \param filename : Absolute YAML file name.
    \param M : Matrix to be loaded from the file.
    \param header : Header of the file is loaded in this parameter.

    \return Returns true when success, false otherwise.

    The following example shows how to use this function:
    \code
    #include <visp3/core/vpMatrix.h>

    #ifdef ENABLE_VISP_NAMESPACE
    using namespace VISP_NAMESPACE_NAME;
    #endif

    int main()
    {
      std::string filename("matrix.yaml");
      {
        vpMatrix M(2, 3);
        M[0][0] = -1; M[0][1] =  -2; M[0][2] = -3;
        M[1][0] =  4; M[1][1] = 5.5; M[1][2] =  6.0f;

        std::string header("My header");

        if (vpMatrix::saveMatrixYAML(filename, M, header.c_str())) {
          std::cout << "Matrix saved in " << filename << std::endl;
          M.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot save matrix in " << filename << std::endl;
        }
      }
      {
        vpMatrix N;
        char header[FILENAME_MAX];
        if (vpMatrix::loadMatrixYAML(filename, N, header)) {
          std::cout << "Matrix loaded from " << filename << std::endl;
          N.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot load matrix from " << filename << std::endl;
        }
      }
    }
    \endcode

    The output of this example is the following:
    \verbatim
    Matrix saved in matrix.yaml
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    Matrix loaded from matrix.yaml
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    \endverbatim

    And the content of `matrix.yaml` file is the following:
    \verbatim
    % cat matrix.yaml
    My header
    rows: 2
    cols: 3
    data:
      - [-1, -2, -3]
      - [4, 5.5, 6]
    \endverbatim

    \sa saveMatrixYAML(), saveMatrix(), loadMatrix()
  */
  static inline bool loadMatrixYAML(const std::string &filename, vpArray2D<double> &M, char *header = nullptr)
  {
    return vpArray2D<double>::loadYAML(filename, M, header);
  }

  /*!
    Save a matrix to a file. This function overloads vpArray2D::save().

    \param filename : Absolute file name.
    \param M : Matrix to be saved.
    \param binary : If true the matrix is save as a binary file, otherwise as a text file.
    \param header : Optional line that will be saved at the beginning of the file as a header.

    \return Returns true if no problem appends.

    \warning If you save the matrix as a text file the precision is less
    than if you save it as a binary file.

    The following example shows how to use this function:
    \code
    #include <visp3/core/vpMatrix.h>

    #ifdef ENABLE_VISP_NAMESPACE
    using namespace VISP_NAMESPACE_NAME;
    #endif

    int main()
    {
      std::string filename("matrix.bin");
      bool binary_data = true;
      {
        vpMatrix M(2, 3);
        M[0][0] = -1; M[0][1] =  -2; M[0][2] = -3;
        M[1][0] =  4; M[1][1] = 5.5; M[1][2] =  6.0f;

        std::string header("My header");

        if (vpMatrix::saveMatrix(filename, M, binary_data, header.c_str())) {
          std::cout << "Matrix saved in " << filename << std::endl;
          M.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot save matrix in " << filename << std::endl;
        }
      }
      {
        vpMatrix N;
        char header[FILENAME_MAX];
        if (vpMatrix::loadMatrix(filename, N, binary_data, header)) {
          std::cout << "Matrix loaded from " << filename << std::endl;
          N.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot load matrix from " << filename << std::endl;
        }
      }
    }
    \endcode

    The output of this example is the following:
    \verbatim
    Matrix saved in matrix.bin
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    Matrix loaded from matrix.bin
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    \endverbatim

    And the content of `matrix.bin` file where data are saved as binary data is the following:
    \verbatim
    % cat matrix.bin
    My header??@@@%
    \endverbatim

    \sa loadMatrix(), saveMatrixYAML(), loadMatrixYAML()
  */
  static inline bool saveMatrix(const std::string &filename, const vpArray2D<double> &M, bool binary = false,
                                const char *header = "")
  {
    return vpArray2D<double>::save(filename, M, binary, header);
  }

  /*!
    Save a matrix in a YAML-formatted file. This function overloads
    vpArray2D::saveYAML().

    \param filename : Absolute file name.
    \param M : Matrix to be saved in the file.
    \param header : Optional lines that will be saved at the beginning of the
    file as a header.

    \return Returns true if success.

    The following example shows how to use this function:
    \code
    #include <visp3/core/vpMatrix.h>

    #ifdef ENABLE_VISP_NAMESPACE
    using namespace VISP_NAMESPACE_NAME;
    #endif

    int main()
    {
      std::string filename("matrix.yaml");
      {
        vpMatrix M(2, 3);
        M[0][0] = -1; M[0][1] =  -2; M[0][2] = -3;
        M[1][0] =  4; M[1][1] = 5.5; M[1][2] =  6.0f;

        std::string header("My header");

        if (vpMatrix::saveMatrixYAML(filename, M, header.c_str())) {
          std::cout << "Matrix saved in " << filename << std::endl;
          M.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot save matrix in " << filename << std::endl;
        }
      }
      {
        vpMatrix N;
        char header[FILENAME_MAX];
        if (vpMatrix::loadMatrixYAML(filename, N, header)) {
          std::cout << "Matrix loaded from " << filename << std::endl;
          N.print(std::cout, 10, header);
        } else {
          std::cout << "Cannot load matrix from " << filename << std::endl;
        }
      }
    }
    \endcode

    The output of this example is the following:
    \verbatim
    Matrix saved in matrix.yaml
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    Matrix loaded from matrix.yaml
    My header[2,3]=
      -1.0 -2.0 -3.0
       4.0  5.5  6.0
    \endverbatim

    And the content of `matrix.yaml` file is the following:
    \verbatim
    % cat matrix.yaml
    My header
    rows: 2
    cols: 3
    data:
      - [-1, -2, -3]
      - [4, 5.5, 6]
    \endverbatim

    \sa saveMatrix(), loadMatrix(), loadMatrixYAML()
  */
  static inline bool saveMatrixYAML(const std::string &filename, const vpArray2D<double> &M, const char *header = "")
  {
    return vpArray2D<double>::saveYAML(filename, M, header);
  }
  //@}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  VP_DEPRECATED double euclideanNorm() const;

  /*!
    @name Deprecated functions
  */
  //@{
  /*!
     \deprecated Only provided for compatibility with ViSP previous releases.
     This function does nothing.
   */
  VP_DEPRECATED void init() { }

  /*!
     \deprecated You should rather use stack(const vpMatrix &A)
   */
  VP_DEPRECATED void stackMatrices(const vpMatrix &A) { stack(A); }
  /*!
     \deprecated You should rather use stack(const vpMatrix &A, const vpMatrix
     &B)
   */
  VP_DEPRECATED static vpMatrix stackMatrices(const vpMatrix &A, const vpMatrix &B) { return stack(A, B); }
  /*!
     \deprecated You should rather use stack(const vpMatrix &A, const vpMatrix
     &B, vpMatrix &C)
   */
  VP_DEPRECATED static void stackMatrices(const vpMatrix &A, const vpMatrix &B, vpMatrix &C) { stack(A, B, C); }
  /*!
     \deprecated You should rather use stack(const vpMatrix &A, const vpMatrix
     &B)
   */
  VP_DEPRECATED static vpMatrix stackMatrices(const vpMatrix &A, const vpRowVector &B);
  /*!
     \deprecated You should rather use stack(const vpMatrix &A, const
     vpRowVector &B, vpMatrix &C)
   */
  VP_DEPRECATED static void stackMatrices(const vpMatrix &A, const vpRowVector &B, vpMatrix &C);
  /*!
     \deprecated You should rather use vpColVector::stack(const vpColVector
     &A, const vpColVector &B)
   */
  VP_DEPRECATED static vpMatrix stackMatrices(const vpColVector &A, const vpColVector &B);
  /*!
     \deprecated You should rather use vpColVector::stack(const vpColVector
     &A, const vpColVector &B, vpColVector &C)
   */
  VP_DEPRECATED static void stackMatrices(const vpColVector &A, const vpColVector &B, vpColVector &C);

  /*!
     \deprecated You should rather use diag(const double &)
   */
  VP_DEPRECATED void setIdentity(const double &val = 1.0);

  VP_DEPRECATED vpRowVector row(unsigned int i);
  VP_DEPRECATED vpColVector column(unsigned int j);

  // Deprecated functions using GSL
#ifndef DOXYGEN_SHOULD_SKIP_THIS
  /*!
     \deprecated You should rather use detByLULapack() or detByLU().
   */
  VP_DEPRECATED double detByLUGsl() const
  {
#if defined(VISP_HAVE_LAPACK)
    return detByLULapack();
#else
    throw(vpException(vpException::fatalError, "Undefined detByLULapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use inverseByLULapack() or inverseByLU().
   */
  VP_DEPRECATED vpMatrix inverseByLUGsl() const
  {
#if defined(VISP_HAVE_LAPACK)
    return inverseByLULapack();
#else
    throw(vpException(vpException::fatalError, "Undefined inverseByLULapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use inverseByCholeskyLapack() or inverseByCholesky().
   */
  VP_DEPRECATED vpMatrix inverseByCholeskyGsl() const
  {
#if defined(VISP_HAVE_LAPACK)
    return inverseByCholeskyLapack();
#else
    throw(vpException(vpException::fatalError, "Undefined inverseByCholeskyLapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use inverseByQRLapack() or inverseByQR().
   */
  VP_DEPRECATED vpMatrix inverseByQRGsl() const
  {
#if defined(VISP_HAVE_LAPACK)
    return inverseByQRLapack();
#else
    throw(vpException(vpException::fatalError, "Undefined inverseByQRLapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use pseudoInverseLapack() or pseudoInverse().
   */
  VP_DEPRECATED vpMatrix pseudoInverseGsl(double svThreshold = 1e-6) const
  {
#if defined(VISP_HAVE_LAPACK)
    return pseudoInverseLapack(svThreshold);
#else
    (void)svThreshold;
    throw(vpException(vpException::fatalError, "Undefined pseudoInverseLapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use pseudoInverseLapack() or pseudoInverse().
   */
  VP_DEPRECATED unsigned int pseudoInverseGsl(vpMatrix &Ap, double svThreshold = 1e-6) const
  {
#if defined(VISP_HAVE_LAPACK)
    return pseudoInverseLapack(Ap, svThreshold);
#else
    (void)Ap;
    (void)svThreshold;
    throw(vpException(vpException::fatalError, "Undefined pseudoInverseLapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use pseudoInverseLapack() or pseudoInverse().
   */
  VP_DEPRECATED unsigned int pseudoInverseGsl(vpMatrix &Ap, vpColVector &sv, double svThreshold = 1e-6) const
  {
#if defined(VISP_HAVE_LAPACK)
    return pseudoInverseLapack(Ap, sv, svThreshold);
#else
    (void)Ap;
    (void)sv;
    (void)svThreshold;
    throw(vpException(vpException::fatalError, "Undefined pseudoInverseLapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use pseudoInverseLapack() or pseudoInverse().
   */
  VP_DEPRECATED unsigned int pseudoInverseGsl(vpMatrix &Ap, vpColVector &sv, double svThreshold, vpMatrix &imA, vpMatrix &imAt,
                                vpMatrix &kerAt) const
  {
#if defined(VISP_HAVE_LAPACK)
    return pseudoInverseLapack(Ap, sv, svThreshold, imA, imAt, kerAt);
#else
    (void)Ap;
    (void)sv;
    (void)svThreshold;
    (void)imA;
    (void)imAt;
    (void)kerAt;
    throw(vpException(vpException::fatalError, "Undefined pseudoInverseLapack(). Install Lapack 3rd party"));
#endif
  }

  /*!
     \deprecated You should rather use svdLapack() or svd().
   */
  VP_DEPRECATED void svdGsl(vpColVector &w, vpMatrix &V)
  {
#if defined(VISP_HAVE_LAPACK)
    svdLapack(w, V);
#else
    (void)w;
    (void)V;
    throw(vpException(vpException::fatalError, "Undefined svdLapack(). Install Lapack 3rd party"));
#endif
  }

#endif // ifndef DOXYGEN_SHOULD_SKIP_THIS
  //@}
#endif

private:
  static unsigned int m_lapack_min_size;
  static const unsigned int m_lapack_min_size_default;

#if defined(VISP_HAVE_LAPACK)
  static void blas_dgemm(char trans_a, char trans_b, unsigned int M_, unsigned int N_, unsigned int K_, double alpha,
                         double *a_data, unsigned int lda_, double *b_data, unsigned int ldb_, double beta,
                         double *c_data, unsigned int ldc_);
  static void blas_dgemv(char trans, unsigned int M_, unsigned int N_, double alpha, double *a_data, unsigned int lda_,
                         double *x_data, int incx_, double beta, double *y_data, int incy_);
  static void blas_dsyev(char jobz, char uplo, unsigned int n_, double *a_data, unsigned int lda_, double *w_data,
                         double *work_data, int lwork_, int &info_);

  unsigned int qrPivotLapack(vpMatrix &Q, vpMatrix &R, vpMatrix &P, bool full, bool squareR,
                       double tol) const;

#ifdef VISP_HAVE_GSL
  unsigned int qrPivotLapackGSL(vpMatrix &Q, vpMatrix &R, vpMatrix &P, bool full, bool squareR,
                       double tol) const;
#endif
#endif

  static void computeCovarianceMatrixVVS(const vpHomogeneousMatrix &cMo, const vpColVector &deltaS, const vpMatrix &Ls,
                                         vpMatrix &Js, vpColVector &deltaP);
};

//////////////////////////////////////////////////////////////////////////
#if defined(VISP_USE_MSVC) && defined(visp_EXPORTS)
const __declspec(selectany) unsigned int vpMatrix::m_lapack_min_size_default = 0;
__declspec(selectany) unsigned int vpMatrix::m_lapack_min_size = vpMatrix::m_lapack_min_size_default;
#endif

#ifndef DOXYGEN_SHOULD_SKIP_THIS
VISP_EXPORT
#endif
vpMatrix operator*(const double &x, const vpMatrix &A);

END_VISP_NAMESPACE
#endif
