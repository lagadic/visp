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
 * Translation vector.
 */

/*!
  \file vpTranslationVector.cpp
  \brief Class that consider the case of a translation vector.
*/

#include <stdio.h>
#include <string.h>

#include <visp3/core/vpTranslationVector.h>

BEGIN_VISP_NAMESPACE
/*!
  Construct a translation vector \f$ \bf t \f$ from 3 doubles.

  \param tx,ty,tz : Translation respectively along x, y and z axis. Values are
  in meters.

*/
vpTranslationVector::vpTranslationVector(double tx, double ty, double tz) : vpArray2D<double>(3, 1), m_index(0)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  (*this)[index_0] = tx;
  (*this)[index_1] = ty;
  (*this)[index_2] = tz;
}

/*!
  Construct a translation vector \f$ \bf t \f$ from the translation contained
  in an homogeneous matrix.

  \param M : Homogeneous matrix where translations are in meters.

*/
vpTranslationVector::vpTranslationVector(const vpHomogeneousMatrix &M) : vpArray2D<double>(3, 1), m_index(0)
{
  M.extract(*this);
}

/*!
  Construct a translation vector \f$ \bf t \f$ from the translation contained
  in a pose vector.

  \param p : Pose vector where translations are in meters.

*/
vpTranslationVector::vpTranslationVector(const vpPoseVector &p) : vpArray2D<double>(3, 1), m_index(0)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  (*this)[index_0] = p[index_0];
  (*this)[index_1] = p[index_1];
  (*this)[index_2] = p[index_2];
}

/*!
  Copy constructor.

  \param tv : Translation vector to copy.

  \code
  vpTranslationVector t1(1,2,3); // Create and initialize a translation vector
  vpTranslationVector t2(t1);    // t2 is now a copy of t1
  \endcode
*/
vpTranslationVector::vpTranslationVector(const vpTranslationVector &tv) : vpArray2D<double>(tv), m_index(0) { }

/*!
  Construct a translation vector \f$ \bf t \f$ from a 3-dimension column
  vector.

  \param v : 3-dimension column vector.

  \code
  vpColVector v(3);
  v[0] = 1; v[1] = 2; v[2] = 3; // Create and initialize a column vector

  vpTranslationVector t(v);     // t contains [1, 2, 3,]
  \endcode

*/
vpTranslationVector::vpTranslationVector(const vpColVector &v) : vpArray2D<double>(v), m_index(0)
{
  if (v.size() != 3) {
    throw(vpException(vpException::dimensionError,
                      "Cannot construct a translation vector from a "
                      "%d-dimension column vector",
                      v.size()));
  }
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  \deprecated You should use build(const vpHomogeneousMatrix &) instead.
  Build a 3 dimension translation vector \f$ \bf t\f$ from
  an homogeneous matrix \f$ \bf M \f$.

  \param M : Homogeneous matrix \f$ \bf M \f$ from which translation \f$
  \bf t \f$ and \f$\theta \bf u \f$ vectors are extracted to initialize
  the pose vector.

  \return The build translation vector.

*/
vpTranslationVector vpTranslationVector::buildFrom(const vpHomogeneousMatrix &M)
{
  build(M);
  return *this;
}

/*!
  \deprecated You should use build(const vpPoseVector &) instead.
  Build a 3 dimension translation vector \f$ \bf t\f$ from
  the translation contained in a pose vector.

  \param p : Pose vector where translations are in meters.

  \return The build translation vector.

*/
vpTranslationVector vpTranslationVector::buildFrom(const vpPoseVector &p)
{
  build(p);
  return *this;
}

/*!
  \deprecated You should use build(const vpColVector &) instead.
  Build a 3 dimension translation vector \f$ \bf t\f$ from
  a 3-dimension column vector.

  \param v : 3-dimension column vector.

  \return The build translation vector.
*/
vpTranslationVector vpTranslationVector::buildFrom(const vpColVector &v)
{
  build(v);
  return *this;
}

/*!
  \deprecated You should use build(const double &, const double &, const double &) instead.
  Build a 3 dimension translation vector \f$ \bf t\f$ from 3 doubles.

  \param tx,ty,tz : Translation respectively along x, y and z axis in meter.

  \return The build translation vector.
  \sa build()
*/
vpTranslationVector vpTranslationVector::buildFrom(double tx, double ty, double tz)
{
  build(tx, ty, tz);
  return *this;
}
#endif

/*!
  Build a 3 dimension translation vector \f$ \bf t\f$ from
  an homogeneous matrix \f$ \bf M \f$.

  \param M : Homogeneous matrix \f$ \bf M \f$ from which translation \f$
  \bf t \f$ and \f$\theta \bf u \f$ vectors are extracted to initialize
  the pose vector.

  \return The build translation vector.

*/
vpTranslationVector &vpTranslationVector::build(const vpHomogeneousMatrix &M)
{
  M.extract(*this);
  return *this;
}

/*!
  Build a 3 dimension translation vector \f$ \bf t\f$ from
  the translation contained in a pose vector.

  \param p : Pose vector where translations are in meters.

  \return The build translation vector.

*/
vpTranslationVector &vpTranslationVector::build(const vpPoseVector &p)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  (*this)[index_0] = p[index_0];
  (*this)[index_1] = p[index_1];
  (*this)[index_2] = p[index_2];
  return *this;
}

/*!
  Build a 3 dimension translation vector \f$ \bf t\f$ from
  a 3-dimension column vector.

  \param v : 3-dimension column vector.

  \return The build translation vector.

*/
vpTranslationVector &vpTranslationVector::build(const vpColVector &v)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int v_size = 3;
  if (v.size() != v_size) {
    throw(vpException(vpException::dimensionError,
                      "Cannot build a translation vector from a %d-dimension column vector", v.size()));
  }

  (*this)[index_0] = v[index_0];
  (*this)[index_1] = v[index_1];
  (*this)[index_2] = v[index_2];
  return *this;
}

/*!
  Build a 3 dimension translation vector \f$ \bf t\f$ from 3 doubles.

  \param tx,ty,tz : Translation respectively along x, y and z axis in meter.

  \return The build translation vector.
  \sa set()
*/
vpTranslationVector &vpTranslationVector::build(const double &tx, const double &ty, const double &tz)
{
  set(tx, ty, tz);
  return *this;
}

/*!
  Initialize a translation vector from 3 doubles.

  \param tx,ty,tz : Translation respectively along x, y and z axis in meter.

*/
void vpTranslationVector::set(double tx, double ty, double tz)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  (*this)[index_0] = tx;
  (*this)[index_1] = ty;
  (*this)[index_2] = tz;
}

/*!
  Operator that allows to add two translation vectors.

  \param tv : Translation  vector to add.

  \return The sum of the current translation vector (*this) and the one to
  add.
\code
  vpTranslationVector t1(1,2,3);
  vpTranslationVector t2(4,5,6);
  vpTranslationVector t3;

  t3 = t2 + t1;
  // t1 and t2 leave unchanged
  // t3 is now equal to : 5, 7, 9
  \endcode

*/
vpTranslationVector vpTranslationVector::operator+(const vpTranslationVector &tv) const
{
  vpTranslationVector s;

  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    s[i] = (*this)[i] + tv[i];
  }

  return s;
}
/*!
  Operator that allows to add a translation vector to a column vector.

  \param v : 3-dimension column vector to add.

  \return The sum of the current translation vector (*this) and the column
  vector to add.
  \code
  vpTranslationVector t1(1,2,3);
  vpColVector v(3);
  v[0] = 4;
  v[1] = 5;
  v[2] = 6;
  vpTranslationVector t2;

  t2 = t1 + v;
  // t1 and v leave unchanged
  // t2 is now equal to : 5, 7, 9
  \endcode

*/
vpTranslationVector vpTranslationVector::operator+(const vpColVector &v) const
{
  if (v.size() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot add translation vector to a %d-dimension column vector",
                      v.size()));
  }
  vpTranslationVector s;

  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    s[i] = (*this)[i] + v[i];
  }

  return s;
}

/*!
  Operator that allows to subtract two translation vectors.

  \param tv : Translation  vector to subtract.

  \return The subtraction of the current translation vector (*this) and the
  one to subtract.
  \code
  vpTranslationVector t1(1,2,3);
  vpTranslationVector t2(4,5,6);
  vpTranslationVector t3;

  t3 = t2 - t1;
  // t1 and t2 leave unchanged
  // t3 is now equal to : 3, 3, 3
  \endcode

*/
vpTranslationVector vpTranslationVector::operator-(const vpTranslationVector &tv) const
{
  vpTranslationVector sub;

  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    sub[i] = (*this)[i] - tv[i];
  }

  return sub;
}

/*!
  Operator that allows to negate a translation vector.

  \return The negate translation. The current translation vector
  (*this) is unchanged.

  \code
  vpTranslationVector t1(1,2,3);
  vpTranslationVector t2;
  t2 = -t1;
  // t1 is unchanged
  // t2 is now equal to : [-1, -2, -3]
  \endcode
*/
vpTranslationVector vpTranslationVector::operator-() const // negate
{
  vpTranslationVector tv;
  for (unsigned int i = 0; i < dsize; ++i) {
    *(tv.data + i) = -*(data + i);
  }

  return tv;
}

/*!
  Operator that allows to multiply each element of a translation vector by a
  scalar.

  \param x : The scalar.

  \return The translation vector multiplied by the scalar. The current
  translation vector (*this) is unchanged.

  \code
  vpTranslationVector t1(1,2,3);
  t2 = t1 * 3;
  // t1 is unchanged
  // t2 is now equal to : [3, 6, 9]
  \endcode
*/
vpTranslationVector vpTranslationVector::operator*(double x) const
{
  vpTranslationVector tv;
  for (unsigned int i = 0; i < dsize; ++i) {
    *(tv.data + i) = (*(data + i)) * x;
  }

  return tv;
}

/*!

  Multiply a 3-by-1 dimension translation vector by a 1-by-n row vector.

  \param v : Row vector.

  \return The resulting matrix that is 3-by-n dimension.

*/
vpMatrix vpTranslationVector::operator*(const vpRowVector &v) const
{
  vpMatrix M(rowNum, v.getCols());
  unsigned int v_col = v.getCols();
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < v_col; ++j) {
      M[i][j] = (*this)[i] * v[j];
    }
  }
  return M;
}

/*!
  Operator that allows to multiply each element of a translation vector by a
  scalar.

  \param x : The scalar.

  \return The translation vector multiplied by the scalar.
*/
vpTranslationVector &vpTranslationVector::operator*=(double x)
{
  for (unsigned int i = 0; i < rowNum; ++i) {
    (*this)[i] *= x;
  }
  return *this;
}
/*!
  Operator that allows to divide each element of a translation vector by a
  scalar.

  \param x : The scalar.

  \return The column vector divided by the scalar.
*/
vpTranslationVector &vpTranslationVector::operator/=(double x)
{
  for (unsigned int i = 0; i < rowNum; ++i) {
    (*this)[i] /= x;
  }
  return *this;
}

/*!
  Operator that allows to divide each element of a translation vector by a
  scalar.

  \param x : The scalar.

  \return The translation vector divided by the scalar. The current
  translation vector (*this) is unchanged.

  \code
  vpTranslationVector t1(8,4,2);
  t2 = t1 / 2;
  // t1 is unchanged
  // t2 is now equal to : [4, 2, 1]
  \endcode
*/
vpTranslationVector vpTranslationVector::operator/(double x) const
{
  vpTranslationVector tv;
  for (unsigned int i = 0; i < dsize; ++i) {
    *(tv.data + i) = (*(data + i)) / x;
  }

  return tv;
}

/*!
  Copy operator.
  \param tv : Translation vector to copy
  \return A copy of \e tv.

  \code
  vpColVector t1(3);
  t1[0] = 1;
  t1[1] = 2;
  t1[2] = 3;
  vpTranslationVector t2;
  t2 = t1;
  // t1 is unchanged
  // t2 is now equal to t1 : 1, 2, 3
  \endcode
*/
vpTranslationVector &vpTranslationVector::operator=(const vpColVector &tv)
{
  if (tv.size() != 3) {
    throw(vpException(vpException::dimensionError,
                      "Cannot initialize a translation vector from a "
                      "%d-dimension col vector",
                      tv.size()));
  }
  unsigned int k = tv.size();
  if (rowNum != k) {
    try {
      resize(k, 1);
    }
    catch (...) {
      throw;
    }
  }

  memcpy(data, tv.data, rowNum * sizeof(double));

  return *this;
}
/*!
  Copy operator.
  \param tv : Translation vector to copy
  \return A copy of \e tv.

  \code
  vpTranslationVector t1(1,2,3);
  vpTranslationVector t2;
  t2 = t1;
  // t1 is unchanged
  // t2 is now equal to t1 : 1, 2, 3
  \endcode
*/
vpTranslationVector &vpTranslationVector::operator=(const vpTranslationVector &tv)
{
  unsigned int k = tv.rowNum;
  if (rowNum != k) {
    try {
      resize(k, 1);
    }
    catch (...) {
      throw;
    }
  }

  memcpy(data, tv.data, rowNum * sizeof(double));

  return *this;
}

/*!
  Initialize each element of a translation vector to the same value x.

  \param x : Value to set for each element of the translation vector.

  \code
  vpTranslationVector t;
  t = 3;
  // Here t is set to 3,3,3
  \endcode
*/
vpTranslationVector &vpTranslationVector::operator=(double x)
{
  double *d = data;

  for (int i = 0; i < 3; ++i) {
    *(d++) = x;
  }

  return *this;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  Set vector from a list of 3 double values in meters.
  \code
  #include <visp3/core/vpTranslationVector.cpp>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpTranslationVector t = {0, 0.1, 0.5};
    std::cout << "t: " << t.t() << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  t: 0  0.1  0.5
  \endcode
  \sa operator<<()
*/
vpTranslationVector &vpTranslationVector::operator=(const std::initializer_list<double> &list)
{
  if (list.size() > size()) {
    throw(vpException(
      vpException::dimensionError,
      "Cannot set translation vector out of bounds. It has only %d values while you try to initialize with %d values",
      size(), list.size()));
  }
  std::copy(list.begin(), list.end(), data);
  return *this;
}
#endif

/*!
  Set vector first element value.
  \param val : Value of the vector first element [meter].
  \return An updated vector.

  The following example shows how to initialize a translation vector from a list of 3 values [meter].
  \code
  #include <visp3/core/vpTranslationVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpTranslationVector t;
    t << 0, 0.1, 0.5;
    std::cout << "t: " << t.t() << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  t: 0  0.1  0.5
  \endcode

  \sa operator,()
 */
vpTranslationVector &vpTranslationVector::operator<<(double val)
{
  m_index = 0;
  data[m_index] = val;
  return *this;
}

/*!
  Set vector second and third element values.
  \param val : Value of the vector element [meter].
  \return An updated vector.

  The following example shows how to initialize a translations vector from a list of 3 values [meter].
  \code
  #include <visp3/core/vpTranslationVector.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpTranslationVector t;
    t << 0, 0.1, 0.5;
    std::cout << "t: " << t.t() << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  t: 0  0.1  0.5
  \endcode

  \sa operator<<()
 */
vpTranslationVector &vpTranslationVector::operator,(double val)
{
  ++m_index;
  if (m_index >= size()) {
    throw(vpException(
      vpException::dimensionError,
      "Cannot set translation vector out of bounds. It has only %d values while you try to initialize with %d values",
      size(), m_index + 1));
  }
  data[m_index] = val;
  return *this;
}

/*!
  Compute the skew symmetric matrix \f$M\f$ of translation vector \e tv.

  \f[ \mbox{if} \quad  {\bf t} =  \left( \begin{array}{c} t_x \\ t_y \\ t_z
  \end{array}\right), \quad \mbox{then} \qquad
  M = \left( \begin{array}{ccc}
  0 & -t_z & t_y \\
  t_z & 0 & -t_x \\
  -t_y & t_x & 0
  \end{array}\right)
  \f]

  \param tv : Translation vector in input used to compute the skew symmetric
  matrix M.

  \param M : Skew symmetric matrix of translation vector \f$t\f$.
*/
void vpTranslationVector::skew(const vpTranslationVector &tv, vpMatrix &M)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  M.resize(3, 3);
  M[index_0][index_0] = 0;
  M[index_0][index_1] = -tv[index_2];
  M[index_0][index_2] = tv[index_1];
  M[index_1][index_0] = tv[index_2];
  M[index_1][index_1] = 0;
  M[index_1][index_2] = -tv[index_0];
  M[index_2][index_0] = -tv[index_1];
  M[index_2][index_1] = tv[index_0];
  M[index_2][index_2] = 0;
}

/*!

  Compute the skew symmetric matrix \f$M\f$ of translation vector \e tv.

  \f[ \mbox{if} \quad  {\bf t} =  \left( \begin{array}{c} t_x \\ t_y \\ t_z
  \end{array}\right), \quad \mbox{then} \qquad
  M = \left( \begin{array}{ccc}
  0 & -t_z & t_y \\
  t_z & 0 & -t_x \\
  -t_y & t_x & 0
  \end{array}\right)
  \f]

  \param tv : Translation vector in input.

  \return Skew symmetric matrix \f$M\f$ of translation vector \f$t\f$.

*/
vpMatrix vpTranslationVector::skew(const vpTranslationVector &tv)
{
  vpMatrix M(3, 3);
  skew(tv, M);
  return M;
}

/*!

  Compute the skew symmetric matrix \f$M\f$ of the translation vector (matrice
  de pre-produit vectoriel), where

  \f[ M = \left( \begin{array}{ccc}
  0 & -t_z & t_y \\
  t_z & 0 & -t_x \\
  -t_y & t_x & 0
  \end{array}\right)
  \f]

  and where \f$(t_x,t_y,t_z)\f$ are the coordinates of the translation
  vector.

  \return Skew symmetric matrix \f$M\f$ of the translation vector.

*/
vpMatrix vpTranslationVector::skew() const
{
  vpMatrix M(3, 3);
  skew(*this, M);
  return M;
}

/*!

  Return the cross product of two translation vectors \f$a \times b\f$.

  \param a,b : Translation vectors in input.

  \return The cross product of two translation vectors \f$a \times
  b\f$.
*/
vpTranslationVector vpTranslationVector::cross(const vpTranslationVector &a, const vpTranslationVector &b)
{
  vpMatrix skew_a = vpTranslationVector::skew(a);
  return static_cast<vpTranslationVector>(skew_a * b);
}

/*!
  Transpose the translation vector. The resulting vector becomes a row vector.
*/
vpRowVector vpTranslationVector::t() const
{
  vpRowVector v(rowNum);
  memcpy(v.data, data, rowNum * sizeof(double));
  return v;
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

/*!
  \deprecated This function is deprecated. You should rather use frobeniusNorm().

  Compute and return the Euclidean norm also called Frobenius nom of the translation vector
  \f$ ||t|| = \sqrt{ \sum{t_{i}^2}} \f$.

  \return The Euclidean norm if the vector is initialized, 0 otherwise.

  \sa frobeniusNorm()
*/
double vpTranslationVector::euclideanNorm() const { return frobeniusNorm(); }

#endif

/*!
  Compute and return the Frobenius norm \f$ ||t|| = \sqrt{ \sum{t_{i}^2}} \f$.

  \return The Frobenius norm if the vector is initialized, 0 otherwise.
*/
double vpTranslationVector::frobeniusNorm() const
{
  double norm = sumSquare();

  return sqrt(norm);
}

/*!
  Return the sum square of all the elements \f$t_{i}\f$ of the translation
  vector t(m).

  \return The value \f[\sum{i=0}^{m} t_i^{2}\f].
*/
double vpTranslationVector::sumSquare() const
{
  double sum_square = 0.0;

  for (unsigned int i = 0; i < rowNum; ++i) {
    double x = rowPtrs[i][0];
    sum_square += x * x;
  }

  return sum_square;
}

/*!
  Compute the Euclidean mean of the translation vector extracted from a vector of homogeneous matrices.

  \param[in] vec_M : Set of homogeneous matrices.
  \return The Euclidean mean of the translation vectors.

  \sa vpRotationMatrix::mean()
 */
vpTranslationVector vpTranslationVector::mean(const std::vector<vpHomogeneousMatrix> &vec_M)
{
  vpColVector meanT(3);
  size_t vec_m_size = vec_M.size();
  for (size_t i = 0; i < vec_m_size; ++i) {
    meanT += static_cast<vpColVector>(vec_M[i].getTranslationVector());
  }
  meanT /= static_cast<double>(vec_M.size());

  vpTranslationVector t(meanT);
  return t;
}

/*!
  Compute the Euclidean mean of a vector of translation vector.

  \param[in] vec_t : Set of translation vectors.
  \return The Euclidean mean of the translation vectors.

  \sa vpRotationMatrix::mean()
 */
vpTranslationVector vpTranslationVector::mean(const std::vector<vpTranslationVector> &vec_t)
{
  vpColVector meanT(3);
  size_t l_vec_t_size = vec_t.size();
  for (size_t i = 0; i < l_vec_t_size; ++i) {
    meanT += static_cast<vpColVector>(vec_t[i]);
  }
  meanT /= static_cast<double>(vec_t.size());

  vpTranslationVector t(meanT);
  return t;
}
END_VISP_NAMESPACE
