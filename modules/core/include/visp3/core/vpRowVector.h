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
 * Operation on row vectors.
 */

#ifndef VP_ROW_VECTOR_H
#define VP_ROW_VECTOR_H

#include <vector>

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

BEGIN_VISP_NAMESPACE

class vpMatrix;
class vpColVector;

/*!
  \file vpRowVector.h
  \brief Definition of row vector class as well
  as a set of operations on these vectors.
*/

/*!
  \class vpRowVector

  \ingroup group_core_matrices

  \brief Implementation of row vector and the associated operations.

  This class provides a data structure for a row vector that contains values
  of double. It contains also some functions to achieve a set of operations on
  these vectors.

  The vpRowVector class is derived from vpArray2D<double>.

  The code below shows how to create a 3-element row vector of doubles, set the element values and access them:
  \code
  #include <visp3/code/vpRowVector.h

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRowVector v(3);
    v[0] = -1; v[1] = -2.1; v[2] = -3;

    std::cout << "v:" << std::endl;
    for (unsigned int i = 0; i < v.size(); ++i) {
      std::cout << v[i] << " ";
    }
    std::cout << std::endl;
  }
  \endcode
  Once build, this previous code produces the following output:
  \code
  v:
  -1 -2.1 -3
  \endcode
  If ViSP is build with c++11 enabled, you can do the same using:
  \code
  #include <visp3/code/vpRowVector.h

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRowVector v{-1, -2.1, -3};
    std::cout << "v:\n" << v << std::endl;
  }
  \endcode
  The vector could also be initialized using operator=(const std::initializer_list< double > &)
  \code
  #include <visp3/code/vpRowVector.h

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRowVector v;
    v = {-1, -2.1, -3};
  }
  \endcode
*/
class VISP_EXPORT vpRowVector : public vpArray2D<double>
{
public:
  //! Basic constructor that creates an empty 0-size row vector.
  vpRowVector() : vpArray2D<double>() { }
  //! Construct a row vector of size n. All the elements are initialized to
  //! zero.
  VP_EXPLICIT vpRowVector(unsigned int n) : vpArray2D<double>(1, n) { }
  //! Construct a row vector of size n. Each element is set to \e val.
  vpRowVector(unsigned int n, double val) : vpArray2D<double>(1, n, val) { }
  //! Copy constructor that allows to construct a row vector from an other
  //! one.
  vpRowVector(const vpRowVector &v) : vpArray2D<double>(v) { }
  vpRowVector(const vpRowVector &v, unsigned int c, unsigned int ncols);
  VP_EXPLICIT vpRowVector(const vpMatrix &M);
  vpRowVector(const vpMatrix &M, unsigned int i);
  VP_EXPLICIT vpRowVector(const std::vector<double> &v);
  VP_EXPLICIT vpRowVector(const std::vector<float> &v);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpRowVector(vpRowVector &&v);
  VP_EXPLICIT vpRowVector(const std::initializer_list<double> &list) : vpArray2D<double>(list) { }
#endif

  /*!
    Removes all elements from the vector (which are destroyed),
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

  /*!
    Extract a sub-row vector from a row vector.
    \param c : Index of the column corresponding to the first element of the
    vector to extract.
    \param rowsize : Size of the vector to extract.
    \exception vpException::fatalError If the vector to extract is not
    contained in the original one.

    \code
    vpRowVector r1;
    for (unsigned int i=0; i<4; ++i)
      r1.stack(i);
    // r1 is equal to [0 1 2 3]
    vpRowVector r2 = r1.extract(1, 3);
    // r2 is equal to [1 2 3]
    \endcode
   */
  vpRowVector extract(unsigned int c, unsigned int rowsize) const
  {
    if ((c >= colNum) || ((c + rowsize) > colNum)) {
      throw(vpException(vpException::fatalError,
                        "Cannot extract a (1x%d) row vector from a (1x%d) "
                        "row vector starting at index %d",
                        rowsize, colNum, c));
    }

    return vpRowVector(*this, c, rowsize);
  }

  double frobeniusNorm() const;

  /*!
   * Compute the Hadamard product (element wise vector multiplication).
   *
   * \param v : Second vector;
   * \return v1.hadamard(v2) The kronecker product :
   * \f$ v1 \circ v2 = (v1 \circ v2)_{i} = (v1)_{i} (v2)_{i} \f$
   */
  vpRowVector hadamard(const vpRowVector &v) const;

  void init(const vpRowVector &v, unsigned int c, unsigned int ncols);
  void insert(unsigned int i, const vpRowVector &v);

  std::ostream &maplePrint(std::ostream &os) const;
  std::ostream &matlabPrint(std::ostream &os) const;

  vpRowVector &normalize();
  vpRowVector &normalize(vpRowVector &x) const;

  //! Operator that allows to set a value of an element \f$v_i\f$: v[i] = x
  inline double &operator[](unsigned int n) { return *(data + n); }
  //! Operator that allows to get the value of an element \f$v_i\f$: x = v[i]
  inline const double &operator[](unsigned int n) const { return *(data + n); }

  //! Copy operator.   Allow operation such as A = v
  vpRowVector &operator=(const vpRowVector &v);
  vpRowVector &operator=(const vpMatrix &M);
  vpRowVector &operator=(const std::vector<double> &v);
  vpRowVector &operator=(const std::vector<float> &v);
  vpRowVector &operator=(double x);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpRowVector &operator=(vpRowVector &&v);
  vpRowVector &operator=(const std::initializer_list<double> &list);
#endif
  //! Comparison operator.
  bool operator==(const vpRowVector &v) const;
  bool operator!=(const vpRowVector &v) const;

  double operator*(const vpColVector &x) const;
  vpRowVector operator*(const vpMatrix &M) const;
  vpRowVector operator*(double x) const;
  vpRowVector &operator*=(double x);

  vpRowVector operator/(double x) const;
  vpRowVector &operator/=(double x);

  vpRowVector operator+(const vpRowVector &v) const;
  vpRowVector &operator+=(vpRowVector v);

  vpRowVector operator-(const vpRowVector &v) const;
  vpRowVector &operator-=(vpRowVector v);
  vpRowVector operator-() const;

  vpRowVector &operator<<(const vpRowVector &v);
  vpRowVector &operator<<(double val);
  vpRowVector &operator,(double val);

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
  vpMatrix reshape(unsigned int nrows, unsigned int ncols);

  /*! Modify the size of the row vector.
    \param i : Size of the vector. This value corresponds to the vector number
    of columns.
    \param flagNullify : If true, set the data to zero.
   */
  inline void resize(unsigned int i, bool flagNullify = true) { vpArray2D<double>::resize(1, i, flagNullify); }

  /*!
    Resize the row vector to a \e ncols-dimension vector.
    This function can only be used with \e nrows = 1.
    \param nrows : Vector number of rows. This value should be set to 1.
    \param ncols : Vector number of columns. This value corresponds
    to the size of the vector.
    \param flagNullify : If true, set the data to zero.
    \exception vpException::fatalError When \e nrows is not equal to 1.

    */
  void resize(unsigned int nrows, unsigned int ncols, bool flagNullify)
  {
    if (nrows != 1) {
      throw(vpException(vpException::fatalError,
                        "Cannot resize a row vector to a (%dx%d) dimension "
                        "vector that has more than one row",
                        nrows, ncols));
    }
    vpArray2D<double>::resize(nrows, ncols, flagNullify);
  }

  void stack(double d);
  void stack(const vpRowVector &v);

  double sum() const;
  double sumSquare() const;
  vpColVector t() const;
  std::vector<double> toStdVector() const;
  vpColVector transpose() const;
  void transpose(vpColVector &v) const;

  static double mean(const vpRowVector &v);
  static double median(const vpRowVector &v);
  static vpRowVector stack(const vpRowVector &A, const vpRowVector &B);
  static void stack(const vpRowVector &A, const vpRowVector &B, vpRowVector &C);
  static double stdev(const vpRowVector &v, bool useBesselCorrection = false);

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
     \deprecated Provided only for compat with previous releases.
     This function does nothing.
   */
  VP_DEPRECATED void init() { }
  /*!
     \deprecated You should rather use stack(const vpRowVector &)
   */
  VP_DEPRECATED void stackMatrices(const vpRowVector &r) { stack(r); }
  /*!
     \deprecated You should rather use stack(const vpRowVector &A, const
     vpRowVector &B)
   */
  VP_DEPRECATED static vpRowVector stackMatrices(const vpRowVector &A, const vpRowVector &B) { return stack(A, B); }
  /*!
     \deprecated You should rather use stack(const vpRowVector &A, const
     vpRowVector &B, vpRowVector &C)
   */
  VP_DEPRECATED static void stackMatrices(const vpRowVector &A, const vpRowVector &B, vpRowVector &C)
  {
    stack(A, B, C);
  }

  /*!
     \deprecated You should rather use eye()
   */
  VP_DEPRECATED void setIdentity(const double &val = 1.0);
  VP_DEPRECATED double euclideanNorm() const;
  //@}
#endif
};

VISP_EXPORT vpRowVector operator*(const double &x, const vpRowVector &v);

END_VISP_NAMESPACE
#endif
