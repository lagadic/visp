/****************************************************************************
 *
 * This file is part of the ViSP software.
 * Copyright (C) 2005 - 2017 by Inria. All rights reserved.
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
 * Provide some simple operation on column vectors.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpColVector_H
#define vpColVector_H

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpRotationVector.h>
#include <visp3/core/vpRowVector.h>

class vpMatrix;
class vpRowVector;
class vpRotationVector;
class vpTranslationVector;
class vpPoseVector;

/*!
  \file vpColVector.h
  \brief definition of column vector class as well
  as a set of operations on these vector
*/

/*!
  \class vpColVector
  \ingroup group_core_matrices

  \brief Implementation of column vector and the associated operations.

  This class provides a data structure for a column vector that contains
  values of double. It contains also some functions to achieve a set of
  operations on these vectors.

  The vpColVector class is derived from vpArray2D<double>.
*/
class VISP_EXPORT vpColVector : public vpArray2D<double>
{
  friend class vpMatrix;

public:
  //! Basic constructor that creates an empty 0-size column vector.
  vpColVector() : vpArray2D<double>() {}
  //! Construct a column vector of size n. \warning Elements are not
  //! initialized. If you want to set an initial value use
  //! vpColVector(unsigned int, double).
  explicit vpColVector(unsigned int n) : vpArray2D<double>(n, 1) {}
  //! Construct a column vector of size n. Each element is set to \e val.
  vpColVector(unsigned int n, double val) : vpArray2D<double>(n, 1, val) {}
  //! Copy constructor that allows to construct a column vector from an other
  //! one.
  vpColVector(const vpColVector &v) : vpArray2D<double>(v) {}
  vpColVector(const vpColVector &v, unsigned int r, unsigned int nrows);
  //! Constructor that initialize a column vector from a 3-dim (Euler or
  //! \f$\theta {\bf u}\f$) or 4-dim (quaternion) rotation vector.
  vpColVector(const vpRotationVector &v);
  //! Constructor that initialize a column vector from a 6-dim pose vector.
  vpColVector(const vpPoseVector &p);
  //! Constructor that initialize a column vector from a 3-dim translation
  //! vector.
  vpColVector(const vpTranslationVector &t);
  vpColVector(const vpMatrix &M);
  vpColVector(const vpMatrix &M, unsigned int j);
  vpColVector(const std::vector<double> &v);
  vpColVector(const std::vector<float> &v);
#ifdef VISP_HAVE_CPP11_COMPATIBILITY
  vpColVector(vpColVector &&v);
#endif
  /*!
    Destructor.
  */
  virtual ~vpColVector() {}

  /*!
    Removes all elements from the vector (which are destroyed),
    leaving the container with a size of 0.
  */
  void clear()
  {
    if (data != NULL) {
      free(data);
      data = NULL;
    }

    if (rowPtrs != NULL) {
      free(rowPtrs);
      rowPtrs = NULL;
    }
    rowNum = colNum = dsize = 0;
  }

  std::ostream &cppPrint(std::ostream &os, const std::string &matrixName = "A", bool octet = false) const;
  std::ostream &csvPrint(std::ostream &os) const;

  /*!
    Convert a column vector containing angles in degrees into radians.
    \sa rad2deg()
  */
  inline void deg2rad()
  {
    double d2r = M_PI / 180.0;

    (*this) *= d2r;
  }

  double euclideanNorm() const;
  /*!
     Extract a sub-column vector from a column vector.
     \param r : Index of the row corresponding to the first element of the
     vector to extract. \param colsize : Size of the vector to extract.
     \exception vpException::fatalError If the vector to extract is not
     contained in the original one.

     \code
     vpColVector v1;
     for (unsigned int i=0; i<4; i++)
       v1.stack(i);
     // v1 is equal to [0 1 2 3]^T
     vpColVector v2 = v1.extract(1, 3);
     // v2 is equal to [1 2 3]^T
     \endcode
   */
  vpColVector extract(unsigned int r, unsigned int colsize) const
  {
    if (r >= rowNum || r + colsize > rowNum) {
      throw(vpException(vpException::fatalError,
                        "Cannot extract a (%dx1) column vector from a (%dx1) "
                        "column vector starting at index %d",
                        colsize, rowNum, r));
    }

    return vpColVector(*this, r, colsize);
  }

  vpColVector hadamard(const vpColVector &v) const;

  double infinityNorm() const;
  void init(const vpColVector &v, unsigned int r, unsigned int nrows);
  void insert(unsigned int i, const vpColVector &v);
  void insert(const vpColVector &v, unsigned int i);

  std::ostream &maplePrint(std::ostream &os) const;
  std::ostream &matlabPrint(std::ostream &os) const;

  vpColVector &normalize();
  vpColVector &normalize(vpColVector &x) const;

  //! Operator that allows to set a value of an element \f$v_i\f$: v[i] = x
  inline double &operator[](unsigned int n) { return *(data + n); }
  //! Operator that allows to get the value of an element \f$v_i\f$: x = v[i]
  inline const double &operator[](unsigned int n) const { return *(data + n); }
  //! Copy operator.   Allow operation such as A = v
  vpColVector &operator=(const vpColVector &v);
  vpColVector &operator=(const vpPoseVector &p);
  vpColVector &operator=(const vpRotationVector &rv);
  vpColVector &operator=(const vpTranslationVector &tv);
  vpColVector &operator=(const vpMatrix &M);
  vpColVector &operator=(const std::vector<double> &v);
  vpColVector &operator=(const std::vector<float> &v);
  vpColVector &operator=(double x);
#ifdef VISP_HAVE_CPP11_COMPATIBILITY
  vpColVector &operator=(vpColVector &&v);
#endif

  double operator*(const vpColVector &x) const;
  vpMatrix operator*(const vpRowVector &v) const;
  vpColVector operator*(const double x) const;
  vpColVector &operator*=(double x);

  vpColVector operator/(const double x) const;
  vpColVector &operator/=(double x);

  vpColVector operator+(const vpColVector &v) const;
  vpTranslationVector operator+(const vpTranslationVector &t) const;
  vpColVector &operator+=(vpColVector v);

  vpColVector operator-(const vpColVector &v) const;
  vpColVector &operator-=(vpColVector v);
  vpColVector operator-() const;

  vpColVector &operator<<(const vpColVector &v);
  vpColVector &operator<<(double *);

  int print(std::ostream &s, unsigned int length, char const *intro = 0) const;

  /*!
    Convert a column vector containing angles in radians into degrees.
    \sa deg2rad()
  */
  inline void rad2deg()
  {
    double r2d = 180.0 / M_PI;

    (*this) *= r2d;
  }

  void reshape(vpMatrix &M, const unsigned int &nrows, const unsigned int &ncols);
  vpMatrix reshape(const unsigned int &nrows, const unsigned int &ncols);

  /*! Modify the size of the column vector.
    \param i : Size of the vector. This value corresponds to the vector number
    of rows.
    \param flagNullify : If true, set the data to zero.
    \exception vpException::fatalError When \e ncols is not equal to 1.
   */
  void resize(const unsigned int i, const bool flagNullify = true) { vpArray2D<double>::resize(i, 1, flagNullify); }
  /*!
    Resize the column vector to a \e nrows-dimension vector.
    This function can only be used with \e ncols = 1.
    \param nrows : Vector number of rows. This value corresponds
    to the size of the vector.
    \param ncols : Vector number of columns. This value should be set to 1.
    \param flagNullify : If true, set the data to zero.
    \exception vpException::fatalError When \e ncols is not equal to 1.

    */
  void resize(const unsigned int nrows, const unsigned int ncols, const bool flagNullify)
  {
    if (ncols != 1)
      throw(vpException(vpException::fatalError,
                        "Cannot resize a column vector to a (%dx%d) "
                        "dimension vector that has more than one column",
                        nrows, ncols));
    vpArray2D<double>::resize(nrows, ncols, flagNullify);
  }

  void stack(const double &d);
  void stack(const vpColVector &v);

  double sum() const;
  double sumSquare() const;
  vpRowVector t() const;
  vpRowVector transpose() const;
  void transpose(vpRowVector &v) const;

  /*!
     Compute and return the cross product of two 3-dimension vectors: \f$a
     \times b\f$. \param a : 3-dimension column vector. \param b : 3-dimension
     column vector. \return The cross product \f$a \times b\f$.

     \exception vpException::dimensionError If the vectors dimension is not
     equal to 3.

   */
  inline static vpColVector cross(const vpColVector &a, const vpColVector &b) { return crossProd(a, b); }
  static vpColVector crossProd(const vpColVector &a, const vpColVector &b);

  static double dotProd(const vpColVector &a, const vpColVector &b);
  static vpColVector invSort(const vpColVector &v);
  static double median(const vpColVector &v);
  static double mean(const vpColVector &v);
  // Compute the skew matrix [v]x
  static vpMatrix skew(const vpColVector &v);

  static vpColVector sort(const vpColVector &v);

  static vpColVector stack(const vpColVector &A, const vpColVector &B);
  static void stack(const vpColVector &A, const vpColVector &B, vpColVector &C);

  static double stdev(const vpColVector &v, const bool useBesselCorrection = false);

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
     \deprecated Provided only for compat with previous releases.
     This function does nothing.
   */
  vp_deprecated void init() {}
  /*!
     \deprecated You should rather use extract().
   */
  vp_deprecated vpColVector rows(unsigned int first_row, unsigned int last_row) const
  {
    return vpColVector(*this, first_row - 1, last_row - first_row + 1);
  }
  /*!
     \deprecated You should rather use eye()
   */
  vp_deprecated void setIdentity(const double &val = 1.0);
  /*!
     \deprecated You should rather use stack(const vpColVector &)
   */
  vp_deprecated void stackMatrices(const vpColVector &r) { stack(r); }
  /*!
     \deprecated You should rather use stack(const vpColVector &A, const
     vpColVector &B)
   */
  vp_deprecated static vpColVector stackMatrices(const vpColVector &A, const vpColVector &B) { return stack(A, B); }
  /*!
     \deprecated You should rather use stack(const vpColVector &A, const
     vpColVector &B, vpColVector &C)
   */
  vp_deprecated static void stackMatrices(const vpColVector &A, const vpColVector &B, vpColVector &C)
  {
    stack(A, B, C);
  }

  vp_deprecated void insert(const vpColVector &v, const unsigned int r, const unsigned int c = 0);
//@}
#endif
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS
VISP_EXPORT
#endif
vpColVector operator*(const double &x, const vpColVector &v);

#endif
