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
 * Provide some simple operation on column vectors.
 */

/*!
 * \file vpColVector.h
 * \brief definition of column vector class as well
 * as a set of operations on these vector
 */

#ifndef VP_COLVECTOR_H
#define VP_COLVECTOR_H

#include <visp3/core/vpConfig.h>

#ifdef VISP_HAVE_NLOHMANN_JSON
#include <nlohmann/json.hpp>
#endif
BEGIN_VISP_NAMESPACE
class vpMatrix;
class vpRowVector;
class vpRotationVector;
class vpTranslationVector;
class vpPoseVector;
END_VISP_NAMESPACE

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpRotationVector.h>
#include <visp3/core/vpRowVector.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpColVector
 * \ingroup group_core_matrices
 *
 * \brief Implementation of column vector and the associated operations.
 *
 * This class provides a data structure for a column vector that contains
 * values of double. It contains also some functions to achieve a set of
 * operations on these vectors.
 *
 * The vpColVector class is derived from vpArray2D<double>.
 *
 * The code below shows how to create a 3-element column vector of doubles, set the element values and access them:
 * \code
 * #include <visp3/code/vpColVector.h
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpColVector v(3);
 *   v[0] = -1; v[1] = -2.1; v[2] = -3;
 *
 *   std::cout << "v:" << std::endl;
 *   for (unsigned int i = 0; i < v.size(); ++i) {
 *     std::cout << v[i] << std::endl;
 *   }
 * }
 * \endcode
 * Once build, this previous code produces the following output:
 * \code{.unparsed}
 * v:
 * -1
 * -2.1
 * -3
 * \endcode
 * You can also use operator<< to initialize a column vector as previously:
 * \code
 * #include <visp3/code/vpColVector.h
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpColVector v;
 *   v << -1, -2.1, -3;
 *   std::cout << "v:" << v << std::endl;
 * }
 * \endcode
 *
 * If ViSP is build with c++11 enabled, you can do the same using:
 * \code
 * #include <visp3/code/vpColVector.h
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpColVector v({-1, -2.1, -3});
 *   std::cout << "v:\n" << v << std::endl;
 * }
 * \endcode
 * The vector could also be initialized using operator=(const std::initializer_list< double > &)
 * \code
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 *   vpColVector v;
 *   v = {-1, -2.1, -3};
 * }
 * \endcode
 *
 * <b>JSON serialization</b>
 *
 * Since ViSP 3.6.0, if ViSP is build with \ref soft_tool_json 3rd-party we introduce JSON serialization capabilities for vpColVector.
 * The following sample code shows how to save a pose vector in a file named `col-vector.json`
 * and reload the values from this JSON file.
 * \code
 * #include <visp3/core/vpColVector.h>
 *
 * #ifdef ENABLE_VISP_NAMESPACE
 * using namespace VISP_NAMESPACE_NAME;
 * #endif
 *
 * int main()
 * {
 * #if defined(VISP_HAVE_NLOHMANN_JSON)
 *   std::string filename = "col-vector.json";
 *   {
 *     vpColVector v({ 1, 2, 3, 4 });
 *     std::ofstream file(filename);
 *     const nlohmann::json j = v;
 *     file << j;
 *     file.close();
 *   }
 *   {
 *     std::ifstream file(filename);
 *     const nlohmann::json j = nlohmann::json::parse(file);
 *     vpColVector v;
 *     v = j.get<vpColVector>();
 *     file.close();
 *     std::cout << "Read homogeneous matrix from " << filename << ":\n" << v.t() << std::endl;
 *   }
 * #endif
 * }
 * \endcode
 * If you build and execute the sample code, it will produce the following output:
 * \code{.unparsed}
 * Read homogeneous matrix from col-vector.json:
 * 1  2  3  4
 * \endcode
 *
 * The content of the `pose-vector.json` file is the following:
 * \code{.unparsed}
 * $ cat col-vector.json
 * {"cols":1,"data":[1.0,2.0,3.0,4.0],"rows":4,"type":"vpColVector"}
 * \endcode
*/
class VISP_EXPORT vpColVector : public vpArray2D<double>
{
  friend class vpMatrix;

public:
  /*!
   * Basic constructor that creates an empty 0-size column vector.
   */
  vpColVector() : vpArray2D<double>() { }

  /*!
   * Construct a column vector of size n.
   * \warning Elements are not  initialized. If you want to set an initial value use
   * vpColVector(unsigned int, double).
   */
  VP_EXPLICIT vpColVector(unsigned int n) : vpArray2D<double>(n, 1) { }

  /*!
   * Construct a column vector of size n. Each element is set to \e val.
   */
  vpColVector(unsigned int n, double val) : vpArray2D<double>(n, 1, val) { }

  /*!
   * Copy constructor that allows to construct a column vector from an other one.
   */
  vpColVector(const vpColVector &v) : vpArray2D<double>(v) { }

  /*!
   * Construct a column vector from a part of an input column vector \e v.
   *
   * \param v : Input column vector used for initialization.
   * \param r : row index in \e v that corresponds to the first element of the
   * column vector to construct.
   * \param nrows : Number of rows of the constructed
   * column vector.
   *
   * The sub-vector starting from v[r] element and ending on v[r+nrows-1] element
   * is used to initialize the constructed column vector.
   *
   * \sa init()
   */
  vpColVector(const vpColVector &v, unsigned int r, unsigned int nrows);

  /*!
   * Constructor that initialize a column vector from a 3-dim (Euler or
   * \f$\theta {\bf u}\f$) or 4-dim (quaternion) rotation vector.
   */
  VP_EXPLICIT vpColVector(const vpRotationVector &v);

  /*!
   * Constructor that initialize a column vector from a 6-dim pose vector.
   */
  VP_EXPLICIT vpColVector(const vpPoseVector &p);

  /*!
   * Constructor that initialize a column vector from a 3-dim translation vector.
   */
  VP_EXPLICIT vpColVector(const vpTranslationVector &t);

  /*!
   * Constructor that creates a column vector from a m-by-1 matrix `M`.
   *
   * \exception vpException::dimensionError If the matrix is not a m-by-1
   * matrix.
   */
  VP_EXPLICIT vpColVector(const vpMatrix &M);

  /*!
   * Constructor that takes column `j` of matrix `M`.
   */
  vpColVector(const vpMatrix &M, unsigned int j);

  /*!
   * Constructor that creates a column vector from a std vector of double.
   */
  VP_EXPLICIT vpColVector(const std::vector<double> &v);

  /*!
   * Constructor that creates a column vector from a std vector of float.
   */
  VP_EXPLICIT vpColVector(const std::vector<float> &v);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  /*!
   * Move constructor that take rvalue.
   */
  vpColVector(vpColVector &&v);
#endif

#if ((__cplusplus >= 201103L) || (defined(_MSVC_LANG) && (_MSVC_LANG >= 201103L))) // Check if cxx11 or higher
  vpColVector(const std::initializer_list<double> &list) : vpArray2D<double>(static_cast<unsigned int>(list.size()), 1)
  {
    std::copy(list.begin(), list.end(), data);
  }
#endif

  /*!
   * Removes all elements from the vector (which are destroyed),
   * leaving the container with a size of 0.
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

  /*!
   * Print to be used as part of a C++ code later.
   *
   * \param os : the stream to be printed in.
   * \param matrixName : name of the column vector, "A" by default.
   * \param octet : if false, print using double, if true, print byte per byte
   * each bytes of the double array.
   *
   * The following code shows how to use this function:
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpColVector v(3);
   *   for (unsigned int i=0; i<v.size(); ++i)
   *     v[i] = i;
   *   v.cppPrint(std::cout, "v");
   * }
   * \endcode
   * It produces the following output that could be copy/paste in a C++ code:
   * \code
   * vpColVector v (3);
   * v[0] = 0;
   * v[1] = 1;
   * v[2] = 2;
   * \endcode
   *
   * \sa print(), matlabPrint(), maplePrint()
   */
  std::ostream &cppPrint(std::ostream &os, const std::string &matrixName = "A", bool octet = false) const;

  /*!
   * Print/save a column vector in csv format.
   *
   * The following code
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   std::ofstream ofs("log.csv", std::ofstream::out);
   *   vpColVector v(3);
   *   for (unsigned int i=0; i<v.size(); ++i)
   *     v[i] = i;
   *
   *   v.csvPrint(ofs);
   *
   *   ofs.close();
   * }
   * \endcode
   * produces `log.csv` file that contains:
   * \code
   * 0
   * 1
   * 2
   * \endcode
   */
  std::ostream &csvPrint(std::ostream &os) const;

  /*!
   * Converts a column vector containing angles in degrees into radians and returns a reference to the vector.
   *
   * \return A reference to the vector with values expressed in [rad].
   * \sa rad2deg()
   */
  inline vpColVector &deg2rad()
  {
    double d2r = M_PI / 180.0;

    (*this) *= d2r;
    return (*this);
  }

  /*!
   *  Extract a sub-column vector from a column vector.
   *
   * \param r : Index of the row corresponding to the first element of the vector to extract.
   * \param colsize : Size of the vector to extract.
   * \exception vpException::fatalError If the vector to extract is not
   *  contained in the original one.
   *
   * \code
   * vpColVector v1;
   * for (unsigned int i=0; i<4; ++i)
   *   v1.stack(i);
   * // v1 is equal to [0 1 2 3]^T
   * vpColVector v2 = v1.extract(1, 3);
   * // v2 is equal to [1 2 3]^T
   * \endcode
   */
  vpColVector extract(unsigned int r, unsigned int colsize) const
  {
    if ((r >= rowNum) || ((r + colsize) > rowNum)) {
      throw(vpException(vpException::fatalError,
                        "Cannot extract a (%dx1) column vector from a (%dx1) "
                        "column vector starting at index %d",
                        colsize, rowNum, r));
    }

    return vpColVector(*this, r, colsize);
  }

  /*!
   * Compute and return the Frobenius norm \f$ ||v|| = \sqrt{ \sum_{v_{i}^2}} \f$ of
   * all the elements \f$v_{i}\f$ of the column vector \f$ \bf v \f$
   * that is of dimension \f$ m \f$.
   *
   * \return The Frobenius norm if the vector is initialized, 0 otherwise.
   *
   * \sa infinityNorm()
   *
   */
  double frobeniusNorm() const;

  /*!
   * Compute the Hadamard product (element wise vector multiplication).
   *
   * \param v : Second vector;
   * \return v1.hadamard(v2) The kronecker product :
   * \f$ v1 \circ v2 = (v1 \circ v2)_{i} = (v1)_{i} (v2)_{i} \f$
   */
  vpColVector hadamard(const vpColVector &v) const;

  /*!
   * Compute and return the infinity norm \f$ {||v||}_{\infty} =
   * max\left({\mid v_{i} \mid}\right) \f$ with \f$i \in
   * \{0, ..., m-1\}\f$ where \e m is the vector size and \f$v_i\f$ an element of
   * the vector.
   *
   * \return The infinity norm if the matrix is initialized, 0 otherwise.
   *
   * \sa frobeniusNorm()
   */
  double infinityNorm() const;

  /*!
   * Initialize the column vector from a part of an input column vector \e v.
   *
   * \param v : Input column vector used for initialization.
   * \param r : row index in \e v that corresponds to the first element of the
   * column vector to construct.
   * \param nrows : Number of rows of the constructed
   * column vector.
   *
   * The sub-vector starting from v[r] element and ending on v[r+nrows-1] element
   * is used to initialize the constructed column vector.
   *
   * The following code shows how to use this function:
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpColVector v(4);
   *   int val = 0;
   *   for(size_t i=0; i<v.getRows(); ++i) {
   *     v[i] = val++;
   *   }
   *   std::cout << "v: " << v.t() << std::endl;
   *
   *   vpColVector w;
   *   w.init(v, 0, 2);
   *   std::cout << "w: " << w.t() << std::endl;
   * }
   * \endcode
   * It produces the following output:
   * \code
   * v: 0 1 2 3
   * w: 1 2
   * \endcode
   */
  void init(const vpColVector &v, unsigned int r, unsigned int nrows);

  /*!
   * Insert a column vector.
   * \param i : Index of the first element to introduce. This index starts from 0.
   * \param v : Column vector to insert.
   *
   * The following example shows how to use this function:
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpColVector v(4);
   *   for (unsigned int i=0; i < v.size(); ++i)
   *     v[i] = i;
   *   std::cout << "v: " << v.t() << std::endl;
   *
   *   vpColVector w(2);
   *   for (unsigned int i=0; i < w.size(); ++i)
   *     w[i] = i+10;
   *   std::cout << "w: " << w.t() << std::endl;
   *
   *   v.insert(1, w);
   *   std::cout << "v: " << v.t() << std::endl;
   * }
   * \endcode
   * It produces the following output:
   * \code
   * v: 0 1 2 3
   * w: 10 11
   * v: 0 10 11 3
   * \endcode
   */
  void insert(unsigned int i, const vpColVector &v);

  /*!
   * Print using Maple syntax, to copy/paste in Maple later.
   *
   * The following code
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpColVector v(3);
   *   for (unsigned int i=0; i<v.size(); ++i)
   *     v[i] = i;
   *   std::cout << "v = "; v.maplePrint(std::cout);
   * }
   * \endcode
   * produces this output:
   * \code
   * v = ([
   * [0, ],
   * [1, ],
   * [2, ],
   * ])
   * \endcode
   * that could be copy/paste in Maple.
   *
   * \sa print() matlabPrint(), cppPrint()
   */
  std::ostream &maplePrint(std::ostream &os) const;

  /*!
   * Print using Matlab syntax, to copy/paste in Matlab later.
   *
   * The following code
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpColVector v(3);
   *   for (unsigned int i=0; i<v.size(); ++i)
   *     v[i] = i;
   *   std::cout << "v = "; v.matlabPrint(std::cout);
   * }
   * \endcode
   * produces this output:
   * \code
   * v = [ 0, ;
   * 1, ;
   * 2, ]
   * \endcode
   * that could be copy/paste in Matlab:
   * \code
   * >> v = [ 0, ;
   * 1, ;
   * 2, ]
   *
   * v =
   *
   *     0
   *     1
   *     2
   *
   * >>
   * \endcode
   *
   * \sa print(), cppPrint(), maplePrint()
   */
  std::ostream &matlabPrint(std::ostream &os) const;

  /*!
   * Normalize the column vector.
   *
   * Considering the n-dim column vector \f$ {\bf x} = (x_0, x_1, \ldots, n_{n-1})\f$
   * normalize each vector element \f$ i \f$: \f[
   * x_i = \frac{x_i}{\sqrt{\sum_{i=0}^{n-1}x^2_i}}
   * \f]
   *
   * \return A reference to the normalized vector.
   */
  vpColVector &normalize();

  /*!
   * Normalize a column vector.
   *
   * Considering the n-dim column vector \f$ {\bf x} = (x_0, x_1, \ldots, n_{n-1})\f$ normalize
   * each vector element \f$ i \f$: \f[
   * x_i = \frac{x_i}{\sqrt{\sum_{i=0}^{n-1} x^2_i}}
   * \f]
   *
   * \param[inout] x : As input, the vector to normalize, as output the normalized vector.
   * \return A reference to the normalized vector.
   */
  vpColVector &normalize(vpColVector &x) const;

  /*!
   * Operator that allows to set a value of an element \f$v_i\f$: v[i] = x
   */
  inline double &operator[](unsigned int n) { return *(data + n); }

  /*!
   * Operator that allows to get the value of an element \f$v_i\f$: x = v[i]
   */
  inline const double &operator[](unsigned int n) const { return *(data + n); }

  /*!
   * Copy operator. Allow operation such as `w = v`.
   * \code
   * vpColVector v(6, 1);
   * vpColVector w;
   * w = v;
   * \endcode
   */
  vpColVector &operator=(const vpColVector &v);

  /*!
   * Operator that allows to convert a pose vector into a column vector.
   */
  vpColVector &operator=(const vpPoseVector &p);

  /*!
   * Operator that allows to convert a rotation vector into a column vector.
   */
  vpColVector &operator=(const vpRotationVector &rv);

  /*!
   * Operator that allows to convert a translation vector into a column vector.
   */
  vpColVector &operator=(const vpTranslationVector &tv);

  /*!
   * Transform a m-by-1 matrix into a column vector.
   * \warning  Handled with care; M should be a 1 column matrix.
   * \exception vpException::dimensionError If the matrix has more than 1 column.
   */
  vpColVector &operator=(const vpMatrix &M);

  /*!
   * Initialize a row vector from a standard vector of double.
   */
  vpColVector &operator=(const std::vector<double> &v);

  /*!
   * Initialize a row vector from a standard vector of float.
   */
  vpColVector &operator=(const std::vector<float> &v);

  /*!
   * Set each element of the column vector to x.
   */
  vpColVector &operator=(double x);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  /*!
   * Overloaded move assignment operator taking rvalue.
   */
  vpColVector &operator=(vpColVector &&v);

  /*!
   * Set vector elements and size from a list of values.
   * \param list : List of double. Vector size matches the number of elements.
   * \return The modified vector.
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpColVector c;
   *   c = { 0, -1, -2 };
   *   std::cout << "c:\n" << c << std::endl;
   * }
   * \endcode
   * It produces the following printings:
   * \code
   * c:
   * 0
   * -1
   * -2
   * \endcode
   * \sa operator<<()
   */
  vpColVector &operator=(const std::initializer_list<double> &list);
#endif

  /*!
   * Compare two column vectors.
   *
   * \param v : Vector to compare with.
   * \return true when their respective size and their respective values are the same,
   * false when their size or values differ.
   */
  bool operator==(const vpColVector &v) const;

  /*!
   * Compare a column vector to a floating point value.
   *
   * \param v : Floating point value to compare with.
   * \return true when all the values of the vector are equal to the floating point value `v`,
   * false otherwise.
   */
  bool operator==(double v) const;

  /*!
   * Compare two column vectors.
   *
   * \param v : Vector to compare with.
   * \return true when their respective size or their values differ, false when their size and values are the same.
   */
  bool operator!=(const vpColVector &v) const;

  /*!
   * Compare a column vector to a floating point value.
   *
   * \param v : Floating point value to compare with.
   * \return true when at least one value of the vector differ from the floating point value `v`.
   * false when all the vector values are equal to `v`.
   */
  bool operator!=(double v) const;

  /*!
   * Operator that performs the dot product between two column vectors.
   *
   * \exception vpException::dimensionError If the vector dimension differ.
   *
   * \sa dotProd()
   */
  double operator*(const vpColVector &v) const;

  /*!
   * Multiply a column vector by a row vector.
   *
   * \param v : Row vector.
   *
   * \return The resulting matrix.
   */
  vpMatrix operator*(const vpRowVector &v) const;

  /*!
   * Multiply a column vector by a matrix.
   *
   * \param M : Matrix.
   *
   * \return The resulting matrix.
   */
  vpMatrix operator*(const vpMatrix &M) const;

  /*!
   * Operator that allows to multiply each element of a column vector by a
   * scalar.
   *
   * \param x : The scalar.
   *
   * \return The column vector multiplied by the scalar. The current
   * column vector (*this) is unchanged.
   *
   * \code
   * vpColVector v(3);
   * v[0] = 1;
   * v[1] = 2;
   * v[2] = 3;
   *
   * vpColVector w = v * 3;
   * // v is unchanged
   * // w is now equal to : [3, 6, 9]
   * \endcode
   */
  vpColVector operator*(double x) const;

  /*!
   * Operator that allows to multiply each element of a column vector by a
   * scalar.
   *
   * \param x : The scalar.
   *
   * \return The column vector multiplied by the scalar.
   *
   * \code
   * vpColVector v(3);
   * v[0] = 1;
   * v[1] = 2;
   * v[2] = 3;
   *
   * v *= 3;
   * // v is now equal to : [3, 6, 9]
   * \endcode
   */
  vpColVector &operator*=(double x);

  /*!
   * Operator that allows to divide each element of a column vector by a scalar.
   *
   * \param x : The scalar.
   *
   * \return The column vector divided by the scalar. The current
   * column vector (*this) is unchanged.
   *
   * \code
   * vpColVector v(3);
   * v[0] = 8;
   * v[1] = 4;
   * v[2] = 2;
   *
   * vpColVector w = v / 2;
   * // v is unchanged
   * // w is now equal to : [4, 2, 1]
   * \endcode
   */
  vpColVector operator/(double x) const;

  /*!
   * Operator that allows to divide each element of a column vector by a scalar.
   *
   * \param x : The scalar.
   *
   * \return The column vector divided by the scalar.
   *
   * \code
   * vpColVector v(3);
   * v[0] = 8;
   * v[1] = 4;
   * v[2] = 2;
   *
   * v /= 2;
   * // v is now equal to : [4, 2, 1]
   * \endcode
   */
  vpColVector &operator/=(double x);

  /*!
   * Operator that allows to add two column vectors.
   */
  vpColVector operator+(const vpColVector &v) const;

  /*!
   * Operator that allows to add a column vector to a translation vector.
   *
   * \param t : 3-dimension translation vector to add.
   *
   * \return The sum of the current column vector (*this) and the translation
   * vector to add.
   * \code
   * vpTranslationVector t1(1,2,3);
   * vpColVector v(3);
   * v[0] = 4;
   * v[1] = 5;
   * v[2] = 6;
   * vpTranslationVector t2;
   *
   * t2 = v + t1;
   * // t1 and v leave unchanged
   * // t2 is now equal to : 5, 7, 9
   * \endcode
   */
  vpTranslationVector operator+(const vpTranslationVector &t) const;

  /*!
   * Operator that allows to add two column vectors.
   */
  vpColVector &operator+=(vpColVector v);

  /*!
   * Operator subtraction of two vectors this = this - v
   */
  vpColVector operator-(const vpColVector &v) const;

  /*!
   * Operator that allows to subtract two column vectors.
   */
  vpColVector &operator-=(vpColVector v);

  /*!
   * Operator that allows to negate all the column vector elements.
   *
   * \code
   * vpColVector r(3, 1);
   * // r contains [1 1 1]^T
   * vpColVector v = -r;
   * // v contains [-1 -1 -1]^T
   * \endcode
   */
  vpColVector operator-() const;

  /*!
   * Copy operator.
   * Allows operation such as A << v
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpColVector A, B(5);
   *   for (unsigned int i=0; i<B.size(); ++i)
   *     B[i] = i;
   *   A << B;
   *   std::cout << "A: " << A.t() << std::endl;
   * }
   * \endcode
   * In column vector A we get:
   * \code
   * A: 0 1 2 3 4
   * \endcode
   */
  vpColVector &operator<<(const vpColVector &v);

  /*!
   * Assignment operator. Allow operation such as A = *v
   *
   * The following example shows how to use this operator.
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   size_t n = 5;
   *   vpColVector A(n);
   *   double *B = new double [n];
   *   for (unsigned int i = 0; i < n; ++i)
   *     B[i] = i;
   *   A << B;
   *   std::cout << "A: " << A.t() << std::endl;
   *   delete [] B;
   * }
   * \endcode
   * It produces the following output:
   * \code
   * A: 0 1 2 3 4
   * \endcode
   */
  vpColVector &operator<<(double *x);

  /*!
   * This operator could be used to set column vector elements:
   * \code
   * #include <visp3/code/vpColVector.h
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpColVector v;
   *   v << -1, -2.1, -3;
   *   std::cout << "v:" << v << std::endl;
   * }
   * \endcode
   * It produces the following printings:
   * \code
   * v: -1  -2.1  -3
   * \endcode
   * \sa operator,()
   */
  vpColVector &operator<<(double val);

  /*!
   * This operator could be used to set column vector elements:
   * \code
   * #include <visp3/code/vpColVector.h
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpColVector v;
   *   v << -1, -2.1, -3;
   *   std::cout << "v:" << v << std::endl;
   * }
   * \endcode
   * It produces the following printings:
   * \code
   * v: -1  -2.1  -3
   * \endcode
   * \sa operator<<()
   */
  vpColVector &operator,(double val);

  /*!
   * Pretty print a column vector. The data are tabulated.
   * The common widths before and after the decimal point
   * are set with respect to the parameter maxlen.
   *
   * \param s Stream used for the printing.
   *
   * \param length The suggested width of each vector element.
   * The actual width grows in order to accommodate the whole integral part,
   * and shrinks if the whole extent is not needed for all the numbers.
   * \param intro The introduction which is printed before the vector.
   * Can be set to zero (or omitted), in which case
   * the introduction is not printed.
   *
   * \return Returns the common total width for all vector elements.
   *
   * \sa std::ostream &operator<<(std::ostream &s, const vpArray2D<Type> &A)
   */
  int print(std::ostream &s, unsigned int length, char const *intro = 0) const;

  /*!
   * Converts a column vector containing angles in radians into degrees and returns a reference
   * to the vector.
   * \return A reference to the vector with values expressed in [deg].
   * \sa deg2rad()
   */
  inline vpColVector &rad2deg()
  {
    double r2d = 180.0 / M_PI;

    (*this) *= r2d;
    return (*this);
  }

  /*!
   * Reshape the column vector in a matrix.
   *
   * \param M : the reshaped matrix.
   * \param nrows : number of rows of the matrix.
   * \param ncols : number of columns of the matrix.
   *
   * \exception vpException::dimensionError If the matrix and the column vector
   * have not the same size.
   *
   * The following example shows how to use this method.
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   int var=0;
   *   vpMatrix mat(3, 4);
   *   for (int i = 0; i < 3; ++i)
   *     for (int j = 0; j < 4; ++j)
   *       mat[i][j] = ++var;
   *   std::cout << "mat: \n" << mat << std::endl;
   *
   *   vpColVector col = mat.stackColumns();
   *   std::cout << "column vector: \n" << col << std::endl;
   *
   *   vpMatrix remat = col.reshape(3, 4);
   *   std::cout << "remat: \n" << remat << std::endl;
   * }
   * \endcode
   *
   * If you run the previous example, you get:
   * \code
   * mat:
   * 1  2  3  4
   * 5  6  7  8
   * 9  10  11  12
   * column vector:
   * 1
   * 5
   * 9
   * 2
   * 6
   * 10
   * 3
   * 7
   * 11
   * 4
   * 8
   * 12
   * remat:
   * 1  2  3  4
   * 5  6  7  8
   * 9  10  11  12
   * \endcode
   */
  void reshape(vpMatrix &M, const unsigned int &nrows, const unsigned int &ncols);

  /*!
   * Reshape the column vector in a matrix.
   *
   * \param nrows : number of rows of the matrix
   * \param ncols : number of columns of the matrix
   * \return The reshaped matrix.
   *
   * \sa reshape(vpMatrix &, const unsigned int &, const unsigned int &)
   */
  vpMatrix reshape(unsigned int nrows, unsigned int ncols);

  /*!
   * Modify the size of the column vector.

   * \param i : Size of the vector. This value corresponds to the vector number
   * of rows.
   * \param flagNullify : If true, set the data to zero.
   * \exception vpException::fatalError When \e ncols is not equal to 1.
   */

  void resize(unsigned int i, bool flagNullify = true)
  {
    vpArray2D<double>::resize(i, 1, flagNullify);
  }

  /*!
   * Resize the column vector to a \e nrows-dimension vector.
   * This function can only be used with \e ncols = 1.
   *
   * \param nrows : Vector number of rows. This value corresponds
   * to the size of the vector.
   * \param ncols : Vector number of columns. This value should be set to 1.
   * \param flagNullify : If true, set the data to zero.
   *
   * \exception vpException::fatalError When \e ncols is not equal to 1.
   */
  void resize(unsigned int nrows, unsigned int ncols, bool flagNullify)
  {
    if (ncols != 1) {
      throw(vpException(vpException::fatalError,
                        "Cannot resize a column vector to a (%dx%d) "
                        "dimension vector that has more than one column",
                        nrows, ncols));
    }
    vpArray2D<double>::resize(nrows, ncols, flagNullify);
  }

  /*!
   * Stack column vector with a new element at the end of the vector.
   *
   * \param d : Element to stack to the existing vector.
   *
   * \code
   * vpColVector v(3, 1);
   * // v is equal to [1 1 1]^T
   * v.stack(-2);
   * // v is equal to [1 1 1 -2]^T
   * \endcode
   *
   * \sa stack(const vpColVector &, const vpColVector &)
   * \sa stack(const vpColVector &, const vpColVector &, vpColVector &)
   */
  void stack(double d);

  /*!
   * Stack column vectors.
   *
   * \param v : Vector to stack to the existing one.
   *
   * \code
   * vpColVector v1(3, 1);
   * // v1 is equal to [1 1 1]^T
   * vpColVector v2(2, 3);
   * // v2 is equal to [3 3]^T
   * v1.stack(v2);
   * // v1 is equal to [1 1 1 3 3]^T
   * \endcode
   *
   * \sa stack(const vpColVector &, const double &)
   * \sa stack(const vpColVector &, const vpColVector &)
   * \sa stack(const vpColVector &, const vpColVector &, vpColVector &)
   */
  void stack(const vpColVector &v);

  /*!
   * Return the sum of all the elements \f$v_{i}\f$ of the column vector \f$ \bf v \f$
   * that is of dimension \f$ m \f$.
   *
   * \return The value \f[ \sum_{i=0}^{m-1} v_i \f].
   */
  double sum() const;

  /*!
   * Return the sum of squares of all the elements \f$v_{i}\f$ of the column vector
   * \f$ \bf v \f$ that is of dimension \f$ m \f$.
   *
   *\return The value \f[\sum_{i=0}^{m-1} v_i^{2}\f].
   */
  double sumSquare() const;

  /*!
   * Transpose the column vector. The resulting vector becomes a row vector.
   */
  vpRowVector t() const;

  /*!
   * Converts the vpColVector to a std::vector.
   * \return The corresponding std::vector<double>.
   */
  std::vector<double> toStdVector() const;

  /*!
   * Transpose the column vector. The resulting vector becomes a row vector.
   * \sa t()
   */
  vpRowVector transpose() const;

  /*!
   * Transpose the column vector. The resulting vector \e v becomes a row vector.
   * \sa t()
   */
  void transpose(vpRowVector &v) const;

  /*!
   * Compute and return the cross product of two 3-dimension vectors: \f$a
   * \times b\f$.
   *
   * \param a : 3-dimension column vector.
   * \param b : 3-dimension column vector.
   * \return The cross product \f$a \times b\f$.
   *
   * \exception vpException::dimensionError If the vectors dimension is not
   * equal to 3.
   *
   * \sa crossProd(), dotProd(), operator*(const vpColVector &)
   */
  inline static vpColVector cross(const vpColVector &a, const vpColVector &b) { return crossProd(a, b); }

  /*!
   * Compute and return the cross product of two vectors \f$a \times b\f$.
   *
   * \param[in] a : 3-dimension column vector.
   * \param[in] b : 3-dimension column vector.
   * \return The cross product \f$a \times b\f$.
   *
   * \exception vpException::dimensionError If the vectors dimension is not equal to 3.
   *
   * \sa dotProd()
   */
  static vpColVector crossProd(const vpColVector &a, const vpColVector &b);

  /*!
   * Compute end return the dot product of two column vectors:
   * \f[ a \cdot b = \sum_{i=0}^n a_i * b_i\f] where \e n is the dimension of
   * both vectors.
   *
   * \exception vpException::dimensionError If the vector dimension differ.
   *
   * \sa cross(), crossProd()
   */
  static double dotProd(const vpColVector &a, const vpColVector &b);

  /*!
   * Return a column vector with elements of \e v that are reverse sorted with
   * values going from greatest to lowest.
   *
   * Example:
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpColVector v(10);
   *   v[0] = 5; v[1] = 7; v[2] = 4; v[3] = 2; v[4] = 8;
   *   v[5] = 6; v[6] = 1; v[7] = 9; v[8] = 0; v[9] = 3;
   *
   *   std::cout << "v: " << v.t() << std::endl;
   *
   *   vpColVector s = vpColVector::invSort(v);
   *   std::cout << "s: " << s.t() << std::endl;
   * }
   * \endcode
   * Output:
   * \code
   * v: 5  7  4  2  8  6  1  9  0  3
   * s: 9  8  7  6  5  4  3  2  1  0
   * \endcode
   *
   * \sa sort()
   */
  static vpColVector invSort(const vpColVector &v);

  /*!
   * Compute the median value of all the elements of the vector.
   */
  static double median(const vpColVector &v);

  /*!
   * Compute the mean value of all the elements of the vector.
   */
  static double mean(const vpColVector &v);

  /*!
   * Compute the skew symmetric matrix \f$[{\bf v}]_\times\f$ of vector v.
   *
   * \f[ \mbox{if} \quad {\bf v} = \left( \begin{array}{c} x \\ y \\ z
   * \end{array}\right), \quad \mbox{then} \qquad
   * [{\bf v}]_\times = \left( \begin{array}{ccc}
   * 0 & -z & y \\
   * z & 0 & -x \\
   * -y & x & 0
   * \end{array}\right)
   * \f]
   *
   * \param v : Input vector used to compute the skew symmetric matrix.
   */
  static vpMatrix skew(const vpColVector &v);

  /*!
   * Return a column vector with elements of \e v that are sorted with values
   * going from lowest to greatest.
   *
   * Example:
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpColVector v(10);
   *   v[0] = 5; v[1] = 7; v[2] = 4; v[3] = 2; v[4] = 8;
   *   v[5] = 6; v[6] = 1; v[7] = 9; v[8] = 0; v[9] = 3;
   *
   *   std::cout << "v: " << v.t() << std::endl;
   *
   *   vpColVector s = vpColVector::sort(v);
   *   std::cout << "s: " << s.t() << std::endl;
   * }
   * \endcode
   * Output:
   * \code
   * v: 5  7  4  2  8  6  1  9  0  3
   * s: 0  1  2  3  4  5  6  7  8  9
   * \endcode
   * \sa invSort()
   */
  static vpColVector sort(const vpColVector &v);

  /*!
   * Stack column vectors.
   *
   * \param A : Initial vector.
   * \param B : Vector to stack at the end of A.
   * \return Stacked vector \f$[A B]^T\f$.
   *
   * \code
   * vpColVector A(3);
   * vpColVector B(5);
   * vpColVector C;
   * C = vpColVector::stack(A, B); // C = [A B]T
   * // C is now an 8 dimension column vector
   * \endcode
   *
   * \sa stack(const vpColVector &)
   * \sa stack(const vpColVector &, const vpColVector &, vpColVector &)
   */
  static vpColVector stack(const vpColVector &A, const vpColVector &B);

  /*!
   * Stack column vectors.
   *
   * \param A : Initial vector.
   * \param B : Vector to stack at the end of A.
   * \param C : Resulting stacked vector \f$C = [A B]^T\f$.
   *
   * \code{.cpp}
   * vpColVector A(3);
   * vpColVector B(5);
   * vpColVector C;
   * vpColVector::stack(A, B, C); // C = [A B]T
   * // C is now an 8 dimension column vector
   * \endcode
   *
   * \sa stack(const vpColVector &)
   * \sa stack(const vpColVector &, const vpColVector &)
   */
  static void stack(const vpColVector &A, const vpColVector &B, vpColVector &C);

  /*!
   * Compute the standard deviation value of all the elements of the vector.
   */
  static double stdev(const vpColVector &v, bool useBesselCorrection = false);

#ifdef VISP_HAVE_NLOHMANN_JSON
  /*!
   * Convert a vpColVector object to a JSON representation.
   *
   * @param j : Resulting json object.
   * @param v : The object to convert.
   */
  friend void to_json(nlohmann::json &j, const vpColVector &v);

  /*!
   * Retrieve a vpColVector object from a JSON representation.
   *
   * @param j : JSON representation to convert.
   * @param v : Converted object.
   */
  friend void from_json(const nlohmann::json &j, vpColVector &v);
#endif

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
   * \deprecated Provided only for compat with previous releases.
   * This function does nothing.
   */
  VP_DEPRECATED void init() { }

  /*!
   * \deprecated Provided only for compat with previous releases. Use rather
   * insert(unsigned int i, const vpColVector &v)
   *
   * Insert a column vector.
   * \param i : Index of the first element to introduce. This index starts from 0.
   * \param v : Column vector to insert.
   *
   * The following example shows how to use this function:
   * \code
   * #include <visp3/core/vpColVector.h>
   *
   * #ifdef ENABLE_VISP_NAMESPACE
   * using namespace VISP_NAMESPACE_NAME;
   * #endif
   *
   * int main()
   * {
   *   vpColVector v(4);
   *   for (unsigned int i=0; i < v.size(); ++i)
   *     v[i] = i;
   *   std::cout << "v: " << v.t() << std::endl;
   *
   *   vpColVector w(2);
   *   for (unsigned int i=0; i < w.size(); ++i)
   *     w[i] = i+10;
   *   std::cout << "w: " << w.t() << std::endl;
   *
   *   v.insert(w, 1);
   *   std::cout << "v: " << v.t() << std::endl;
   * }
   * \endcode
   * It produces the following output:
   * \code
   * v: 0 1 2 3
   * w: 10 11
   * v: 0 10 11 3
   * \endcode
   */
  VP_DEPRECATED void insert(const vpColVector &v, unsigned int i);

  /*!
   * \deprecated You should rather use extract().
   */
  VP_DEPRECATED vpColVector rows(unsigned int first_row, unsigned int last_row) const
  {
    return vpColVector(*this, first_row - 1, last_row - first_row + 1);
  }

  /*!
   * \deprecated You should rather use stack(const vpColVector &)
   */
  VP_DEPRECATED void stackMatrices(const vpColVector &r) { stack(r); }

  /*!
   * \deprecated You should rather use stack(const vpColVector &A, const vpColVector &B)
   */
  VP_DEPRECATED static vpColVector stackMatrices(const vpColVector &A, const vpColVector &B) { return stack(A, B); }

  /*!
   * \deprecated You should rather use stack(const vpColVector &A, const vpColVector &B, vpColVector &C)
   */
  VP_DEPRECATED static void stackMatrices(const vpColVector &A, const vpColVector &B, vpColVector &C)
  {
    stack(A, B, C);
  }

  /*!
   * \deprecated You should rather use insert(unsigned int, const vpColVector &).
   *
   * Insert column vector \e v at the given position \e r in the current column
   * vector.
   *
   * \warning Throw vpMatrixException::incorrectMatrixSizeError if the
   * dimensions of the matrices do not allow the operation.
   *
   * \param v : The column vector to insert.
   * \param r : The index of the row to begin to insert data.
   * \param c : Not used.
   */
  VP_DEPRECATED void insert(const vpColVector &v, unsigned int r, unsigned int c = 0);

  /*!
   * \deprecated This function is deprecated. You should rather use frobeniusNorm().
   *
   * Compute and return the Euclidean norm also called Frobenius norm \f$ ||v|| = \sqrt{ \sum{v_{i}^2}} \f$.
   *
   * \return The Euclidean norm if the vector is initialized, 0 otherwise.
   *
   * \sa frobeniusNorm(), infinityNorm()
   */
  VP_DEPRECATED double euclideanNorm() const;
  //@}
#endif
};

/*!
 * \relates vpColVector
 * Allows to multiply a scalar by a column vector.
 */
#ifndef DOXYGEN_SHOULD_SKIP_THIS
VISP_EXPORT
#endif
vpColVector operator*(const double &x, const vpColVector &v);


#ifdef VISP_HAVE_NLOHMANN_JSON
inline void to_json(nlohmann::json &j, const vpColVector &v)
{
  const vpArray2D<double> *asArray = (vpArray2D<double>*) & v;
  to_json(j, *asArray);
  j["type"] = "vpColVector";
}

inline void from_json(const nlohmann::json &j, vpColVector &v)
{
  vpArray2D<double> *asArray = (vpArray2D<double>*) & v;
  from_json(j, *asArray);
  if (v.getCols() != 1) {
    throw vpException(vpException::badValue, "From JSON, tried to read a 2D array into a vpColVector");
  }
}
#endif
END_VISP_NAMESPACE
#endif
