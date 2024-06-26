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
 * Rotation matrix.
 */

/*!
  \file vpRotationMatrix.cpp
  \brief Class that consider
  the particular case of rotation matrix
*/

#include <visp3/core/vpMath.h>
#include <visp3/core/vpMatrix.h>

// Rotation classes
#include <visp3/core/vpRotationMatrix.h>

// Exception
#include <visp3/core/vpException.h>

// Debug trace
#include <math.h>

BEGIN_VISP_NAMESPACE
/*!
  Initialize the rotation matrix as identity.

  \sa setIdentity()
*/
void vpRotationMatrix::eye()
{
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      if (i == j) {
        (*this)[i][j] = 1.0;
      }
      else {
        (*this)[i][j] = 0.0;
      }
    }
  }
}

/*!
  Set the current rotation matrix from a rotation matrix \e R.

  \param R : Rotation matrix.
  \code
  vpRotationMatrix R1(vpMath::rad(10, vpMath::rad(20), vpMath::rad(30));
  vpRotationMatrix R2 = R1;
  \endcode
*/
vpRotationMatrix &vpRotationMatrix::operator=(const vpRotationMatrix &R)
{
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      rowPtrs[i][j] = R.rowPtrs[i][j];
    }
  }

  return *this;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  Set a rotation matrix from a list of 9 double values.
  \param list : List of double.
  The following code shows how to use this constructor to initialize a rotation matrix:
  \code
  #include <visp3/core/vpRotationMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRotationMatrix R
    R = { 0, 0, -1, 0, -1, 0, -1, 0, 0 };
    std::cout << "R:\n" << R << std::endl;
  }
  \endcode
  It produces the following output:
  \code
  R:
  0  0  -1
  0  -1  0
  -1  0  0
  \endcode
  \sa operator<<()
 */
vpRotationMatrix &vpRotationMatrix::operator=(const std::initializer_list<double> &list)
{
  if (dsize != static_cast<unsigned int>(list.size())) {
    throw(vpException(vpException::dimensionError,
                      "Cannot set a 3-by-3 rotation matrix from a %d-elements list of doubles."));
  }

  std::copy(list.begin(), list.end(), data);

  if (!isARotationMatrix()) {
    if (isARotationMatrix(1e-3)) {
      orthogonalize();
    }
    else {
      throw(vpException(
        vpException::fatalError,
        "Rotation matrix initialization fails since its elements do not represent a valid rotation matrix"));
    }
  }

  return *this;
}
#endif

/*!
  Converts a 3-by-3 matrix into a rotation matrix.

  \param M : Input matrix.

  \code
  vpMatrix M(3, 3);
  M.eye()
  vpRotationMatrix R = M;
  \endcode

  \exception vpException::fatalError If the input matrix is not a rotation
  matrix.

  \sa isARotationMatrix()
*/
vpRotationMatrix &vpRotationMatrix::operator=(const vpMatrix &M)
{
  if ((M.getCols() != 3) && (M.getRows() != 3)) {
    throw(vpException(vpException::dimensionError, "Cannot set a (3x3) rotation matrix from a (%dx%d) matrix",
                      M.getRows(), M.getCols()));
  }

  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      (*this)[i][j] = M[i][j];
    }
  }

  if (isARotationMatrix() == false) {
    throw(vpException(vpException::fatalError, "Cannot set a rotation matrix "
                      "from a matrix that is not a "
                      "rotation matrix"));
  }

  return *this;
}

/*!
  Set rotation matrix first element.

  \param val : Value of the matrix first element.
  \return An updated matrix.

  The following example shows how to initialize a rotation matrix using this operator.
  \code
  #include <visp3/core/vpRotationMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRotationMatrix R;
    R << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    std::cout << "R:\n" << R << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  R:
  0  0  -1
  0  -1  0
  -1  0  0
  \endcode

  \sa operator,()
 */
vpRotationMatrix &vpRotationMatrix::operator<<(double val)
{
  m_index = 0;
  data[m_index] = val;
  return *this;
}

/*!
  Set the second and next element of the rotation matrix.

  \param val : Value of the matrix second or next element.
  \return An updated matrix.

  The following example shows how to initialize a rotation matrix using this operator.
  \code
  #include <visp3/core/vpRotationMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRotationMatrix R;
    R << 0, 0, -1, 0, -1, 0, -1, 0, 0;
    std::cout << "R:\n" << R << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  R:
  0  0  -1
  0  -1  0
  -1  0  0
  \endcode

  \sa operator<<()
 */
vpRotationMatrix &vpRotationMatrix::operator,(double val)
{
  ++m_index;
  if (m_index >= size()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot set rotation matrix out of bounds. It has only %d elements while you try to initialize "
                      "with %d elements",
                      size(), m_index + 1));
  }
  data[m_index] = val;
  return *this;
}

/*!
   Compute the product between two rotation matrices.
 */
vpRotationMatrix vpRotationMatrix::operator*(const vpRotationMatrix &R) const
{
  vpRotationMatrix p;

  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      double s = 0;
      for (unsigned int k = 0; k < 3; ++k) {
        s += rowPtrs[i][k] * R.rowPtrs[k][j];
      }
      p[i][j] = s;
    }
  }
  return p;
}

/*!
  Operator that allows to multiply a rotation matrix by a 3-by-3 matrix.
  Allows for example to multiply a rotation matrix by a skew matrix.

  \param[in] M : 3-by-3 matrix.

  \return The product between the rotation matrix and the 3-by-3 matrix `M`.
  \exception vpException::dimensionError : If \e M is not  a 3-by-3 dimension matrix.

  The following snippet shows how to use this method:
  \code
  vpRotationMatrix R;
  vpTranslationVector t;
  vpMatrix M = t.skew();
  vpMatrix RM = R * M;
  \endcode
*/
vpMatrix vpRotationMatrix::operator*(const vpMatrix &M) const
{
  if ((M.getRows() != 3) || (M.getCols() != 3)) {
    throw(vpException(vpException::dimensionError, "Cannot set a (3x3) rotation matrix from a (%dx%d) matrix",
                      M.getRows(), M.getCols()));
  }
  vpMatrix p(3, 3);

  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      double s = 0;
      for (unsigned int k = 0; k < 3; ++k) {
        s += (*this)[i][k] * M[k][j];
      }
      p[i][j] = s;
    }
  }
  return p;
}

/*!
  Operator that allows to multiply a rotation matrix by a homogeneous matrix.

  \param[in] M : Homogeneous matrix.

  \return The product between the rotation matrix and the homogeneous matrix `M`.

  The following snippet shows how to use this method:
  \code
  vpRotationMatrix c1_R_c2;
  vpHomogeneousMatrix c2_M_c3;
  vpHomogeneousMatrix c1_M_c3 = c1_R_c2 * c2_M_c3;
  \endcode
*/
vpHomogeneousMatrix vpRotationMatrix::operator*(const vpHomogeneousMatrix &M) const
{
  return (vpHomogeneousMatrix(*this * M.getTranslationVector(), *this * M.getRotationMatrix()));
}

/*!

  Operator that allows to multiply a rotation matrix by a 3 dimension
  column vector.

  \param v : Three dimension column vector.

  \return The product of the rotation matrix by the column vector

  \exception vpException::dimensionError If the column
  vector \e v is not a 3 dimension vector.

  The code below shows how to use this operator.
  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpRotationMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpColVector p1(3), p2(3);
    vpRotationMatrix R;

    p2 = R * p1;

    return 0;
  }
  \endcode

*/
vpColVector vpRotationMatrix::operator*(const vpColVector &v) const
{
  const unsigned int rows_size = 3;
  if (v.getRows() != rows_size) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply a (3x3) rotation matrix by a %d "
                      "dimension column vector",
                      v.getRows()));
  }
  vpColVector v_out(3);

  for (unsigned int j = 0; j < colNum; ++j) {
    double vj = v[j]; // optimization em 5/12/2006
    for (unsigned int i = 0; i < rowNum; ++i) {
      v_out[i] += rowPtrs[i][j] * vj;
    }
  }

  return v_out;
}

/*!
  Multiply a rotation matrix by a translation vector and return the resulting
  translation vector.
 */
vpTranslationVector vpRotationMatrix::operator*(const vpTranslationVector &tv) const
{
  vpTranslationVector p;
  const unsigned int val_3 = 3;

  for (unsigned int j = 0; j < val_3; ++j) {
    p[j] = 0;
  }

  for (unsigned int j = 0; j < val_3; ++j) {
    for (unsigned int i = 0; i < val_3; ++i) {
      p[i] += rowPtrs[i][j] * tv[j];
    }
  }

  return p;
}

/*!
   Operator that allows to multiply all the elements of a rotation matrix
   by a scalar.
 */
vpRotationMatrix vpRotationMatrix::operator*(double x) const
{
  vpRotationMatrix R;

  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      R[i][j] = rowPtrs[i][j] * x;
    }
  }

  return R;
}

/*!
   Operator that allows to multiply all the elements of a rotation matrix
   by a scalar.
 */
vpRotationMatrix &vpRotationMatrix::operator*=(double x)
{
  for (unsigned int i = 0; i < rowNum; ++i) {
    for (unsigned int j = 0; j < colNum; ++j) {
      rowPtrs[i][j] *= x;
    }
  }

  return *this;
}

/*********************************************************************/

/*!
  Test if the rotation matrix is really a rotation matrix.

  \return true if the matrix is a rotation matrix, false otherwise.

*/
bool vpRotationMatrix::isARotationMatrix(double threshold) const
{
  bool isRotation = true;

  if ((getCols() != 3) || (getRows() != 3)) {
    return false;
  }

  // --comment: test R^TR = Id
  vpRotationMatrix RtR = (*this).t() * (*this);
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      if (i == j) {
        if (fabs(RtR[i][j] - 1) > threshold) {
          isRotation = false;
        }
      }
      else {
        if (fabs(RtR[i][j]) > threshold) {
          isRotation = false;
        }
      }
    }
  }
  // test if it is a basis
  // test || Ci || = 1
  const unsigned int index_2 = 2;
  for (unsigned int i = 0; i < val_3; ++i) {
    if ((sqrt(vpMath::sqr(RtR[0][i]) + vpMath::sqr(RtR[1][i]) + vpMath::sqr(RtR[index_2][i])) - 1) > threshold) {
      isRotation = false;
    }
  }

  // test || Ri || = 1
  for (unsigned int i = 0; i < val_3; ++i) {
    if ((sqrt(vpMath::sqr(RtR[i][0]) + vpMath::sqr(RtR[i][1]) + vpMath::sqr(RtR[i][index_2])) - 1) > threshold) {
      isRotation = false;
    }
  }

  //  test if the basis is orthogonal
  return isRotation;
}

/*!
  Default constructor that initialise a 3-by-3 rotation matrix to identity.
*/
vpRotationMatrix::vpRotationMatrix() : vpArray2D<double>(3, 3), m_index(0) { eye(); }

/*!
  Copy constructor that construct a 3-by-3 rotation matrix from another
  rotation matrix.
*/
vpRotationMatrix::vpRotationMatrix(const vpRotationMatrix &M) : vpArray2D<double>(3, 3), m_index(0) { (*this) = M; }

/*!
  Construct a 3-by-3 rotation matrix from an homogeneous matrix.
*/
vpRotationMatrix::vpRotationMatrix(const vpHomogeneousMatrix &M) : vpArray2D<double>(3, 3), m_index(0) { build(M); }

/*!
  Construct a 3-by-3 rotation matrix from \f$ \theta {\bf u}\f$ angle
  representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpThetaUVector &tu) : vpArray2D<double>(3, 3), m_index(0) { build(tu); }

/*!
  Construct a 3-by-3 rotation matrix from a pose vector.
 */
vpRotationMatrix::vpRotationMatrix(const vpPoseVector &p) : vpArray2D<double>(3, 3), m_index(0) { build(p); }

/*!
  Construct a 3-by-3 rotation matrix from \f$ R(z,y,z) \f$ Euler angle
  representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpRzyzVector &euler) : vpArray2D<double>(3, 3), m_index(0)
{
  build(euler);
}

/*!
  Construct a 3-by-3 rotation matrix from \f$ R(x,y,z) \f$ Euler angle
  representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpRxyzVector &Rxyz) : vpArray2D<double>(3, 3), m_index(0) { build(Rxyz); }

/*!
  Construct a 3-by-3 rotation matrix from \f$ R(z,y,x) \f$ Euler angle
  representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpRzyxVector &Rzyx) : vpArray2D<double>(3, 3), m_index(0) { build(Rzyx); }

/*!
  Construct a 3-by-3 rotation matrix from a matrix that contains values corresponding to a rotation matrix.
*/
vpRotationMatrix::vpRotationMatrix(const vpMatrix &R) : vpArray2D<double>(3, 3), m_index(0) { *this = R; }

/*!
  Construct a 3-by-3 rotation matrix from \f$ \theta {\bf u}=(\theta u_x,
  \theta u_y, \theta u_z)^T\f$ angle representation.
 */
vpRotationMatrix::vpRotationMatrix(double tux, double tuy, double tuz) : vpArray2D<double>(3, 3), m_index(0)
{
  build(tux, tuy, tuz);
}

/*!
  Construct a 3-by-3 rotation matrix from quaternion angle representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpQuaternionVector &q) : vpArray2D<double>(3, 3), m_index(0) { build(q); }

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  Construct a rotation matrix from a list of 9 double values.
  \param list : List of double.
  The following code shows how to use this constructor to initialize a rotation matrix:
  \code
  #include <visp3/core/vpRotationMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRotationMatrix R{ 0, 0, -1, 0, -1, 0, -1, 0, 0 };
    std::cout << "R:\n" << R << std::endl;
  }
  \endcode
  It produces the following output:
  \code
  R:
  0  0  -1
  0  -1  0
  -1  0  0
  \endcode
 */
vpRotationMatrix::vpRotationMatrix(const std::initializer_list<double> &list)
  : vpArray2D<double>(3, 3, list), m_index(0)
{
  if (!isARotationMatrix()) {
    if (isARotationMatrix(1e-3)) {
      orthogonalize();
    }
    else {
      throw(vpException(
        vpException::fatalError,
        "Rotation matrix initialization fails since its elements do not represent a valid rotation matrix"));
    }
  }
}
#endif

/*!
  Return the rotation matrix transpose which is also the inverse of the
  rotation matrix.

  \sa inverse()
*/
vpRotationMatrix vpRotationMatrix::t() const
{
  vpRotationMatrix Rt;

  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      Rt[j][i] = (*this)[i][j];
    }
  }

  return Rt;
}

/*!
  Return the rotation matrix inverse which is also the transpose of the
  rotation matrix.

  \sa t()
*/
vpRotationMatrix vpRotationMatrix::inverse() const
{
  vpRotationMatrix Ri = (*this).t();

  return Ri;
}

/*!
  Inverse the rotation matrix.

  \param R (output): Inverted rotation matrix.

  \code
  #include <visp3/core/vpRotationMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRotationMatrix R, Rinv;
    // ... Update rotation matrix R
    // Compute the inverse in Rinv
    R.inverse(Rinv);
  }
  \endcode
*/
void vpRotationMatrix::inverse(vpRotationMatrix &R) const { R = inverse(); }

/*!
  Print to std::cout the rotation matrix as a \f$ \theta {\bf u} \f$ angle
  representation vector.
 */
void vpRotationMatrix::printVector()
{
  vpThetaUVector tu(*this);

  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    std::cout << tu[i] << "  ";
  }

  std::cout << std::endl;
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  \deprecated You should use build(const vpThetaUVector &) instead.
  Transform a \f$ \theta {\bf u}\f$ angle representation into a rotation
  matrix.

  The rotation is computed using :
  \f[
  R = \cos{ \theta} \; {I}_{3} + (1 - \cos{ \theta}) \; u u^{T} + \sin{
  \theta} \; [u]_\times \f]
*/
vpRotationMatrix vpRotationMatrix::buildFrom(const vpThetaUVector &v)
{
  build(v);
  return *this;
}

/*!
  \deprecated You should use build(const vpHomogeneousMatrix &) instead.
  Build a rotation matrix from an homogeneous matrix.
*/
vpRotationMatrix vpRotationMatrix::buildFrom(const vpHomogeneousMatrix &M)
{
  build(M);
  return *this;
}

/*!
  \deprecated You should use build(const vpPoseVector &) instead.
  Build a rotation matrix from a pose vector.

  \sa build(const vpThetaUVector &)
*/
vpRotationMatrix vpRotationMatrix::buildFrom(const vpPoseVector &p)
{
  return build(p);
}

/*!
  \deprecated You should use build(const vpRzyzVector &) instead.
  Transform a vector representing the Euler angle
  into a rotation matrix.
  Rzyz(\f$ \phi, \theta , \psi \f$) =  Rot(\f$ z,\phi \f$) Rot(\f$ y,\theta
  \f$) Rot(\f$ z,\psi \f$)

*/
vpRotationMatrix vpRotationMatrix::buildFrom(const vpRzyzVector &v)
{
  build(v);
  return *this;
}

/*!
  \deprecated You should use build(const vpRxyzVector &) instead.
  Transform a vector representing the Rxyz angle into a rotation
  matrix.
  Rxyz(\f$ \phi,\theta, \psi \f$) = Rot(\f$ x, \psi \f$) Rot(\f$ y, \theta \f$
  ) Rot(\f$ z,\phi \f$)

*/
vpRotationMatrix vpRotationMatrix::buildFrom(const vpRxyzVector &v)
{
  build(v);
  return *this;
}

/*!
  \deprecated You should use build(const vpRzyxVector &) instead.
  Transform a vector representing the Rzyx angle
  into a rotation matrix.
  Rxyz(\f$ \phi, \theta , \psi \f$) =
  Rot(\f$ z, \psi \f$) Rot(\f$ y, \theta \f$)Rot(\f$ x, \phi \f$)
*/
vpRotationMatrix vpRotationMatrix::buildFrom(const vpRzyxVector &v)
{
  build(v);
  return *this;
}

/*!
  \deprecated You should use build(const double &, const double &, const double &) instead.
  Construct a 3-by-3 rotation matrix from \f$ \theta {\bf u}=(\theta u_x,
  \theta u_y, \theta u_z)^T\f$ angle representation.
 */
vpRotationMatrix vpRotationMatrix::buildFrom(double tux, double tuy, double tuz)
{
  build(tux, tuy, tuz);
  return *this;
}

/*!
  \deprecated You should use build(const vpQuaternionVector &) instead.
  Construct a 3-by-3 rotation matrix from a quaternion representation.
 */
vpRotationMatrix vpRotationMatrix::buildFrom(const vpQuaternionVector &q)
{
  build(q);
  return *this;
}
#endif

/*!
  Transform a \f$ \theta {\bf u}\f$ angle representation into a rotation
  matrix.

  The rotation is computed using :
  \f[
  R = \cos{ \theta} \; {I}_{3} + (1 - \cos{ \theta}) \; u u^{T} + \sin{
  \theta} \; [u]_\times \f]
*/
vpRotationMatrix &vpRotationMatrix::build(const vpThetaUVector &v)
{
  double theta, si, co, sinc, mcosc;
  vpRotationMatrix R;

  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  theta = sqrt((v[index_0] * v[index_0]) + (v[index_1] * v[index_1]) + (v[index_2] * v[index_2]));
  si = sin(theta);
  co = cos(theta);
  sinc = vpMath::sinc(si, theta);
  mcosc = vpMath::mcosc(co, theta);

  R[index_0][index_0] = co + (mcosc * v[index_0] * v[index_0]);
  R[index_0][index_1] = (-sinc * v[index_2]) + (mcosc * v[index_0] * v[index_1]);
  R[index_0][index_2] = (sinc * v[index_1])  + (mcosc * v[index_0] * v[index_2]);
  R[index_1][index_0] = (sinc * v[index_2])  + (mcosc * v[index_1] * v[index_0]);
  R[index_1][index_1] = co + (mcosc * v[index_1] * v[index_1]);
  R[index_1][index_2] = (-sinc * v[index_0]) + (mcosc * v[index_1] * v[index_2]);
  R[index_2][index_0] = (-sinc * v[index_1]) + (mcosc * v[index_2] * v[index_0]);
  R[index_2][index_1] = (sinc * v[index_0])  + (mcosc * v[index_2] * v[index_1]);
  R[index_2][index_2] = co + (mcosc * v[index_2] * v[index_2]);

  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      (*this)[i][j] = R[i][j];
    }
  }

  return *this;
}

/*!
  Build a rotation matrix from an homogeneous matrix.
*/
vpRotationMatrix &vpRotationMatrix::build(const vpHomogeneousMatrix &M)
{
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      (*this)[i][j] = M[i][j];
    }
  }

  return *this;
}

/*!
  Build a rotation matrix from a pose vector.

  \sa build(const vpThetaUVector &)
*/
vpRotationMatrix &vpRotationMatrix::build(const vpPoseVector &p)
{
  vpThetaUVector tu(p);
  return build(tu);
}

/*!
  Transform a vector representing the Euler angle
  into a rotation matrix.
  Rzyz(\f$ \phi, \theta , \psi \f$) =  Rot(\f$ z,\phi \f$) Rot(\f$ y,\theta
  \f$) Rot(\f$ z,\psi \f$)

*/
vpRotationMatrix &vpRotationMatrix::build(const vpRzyzVector &v)
{
  double c0, c1, c2, s0, s1, s2;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;

  c0 = cos(v[index_0]);
  c1 = cos(v[index_1]);
  c2 = cos(v[index_2]);
  s0 = sin(v[index_0]);
  s1 = sin(v[index_1]);
  s2 = sin(v[index_2]);

  (*this)[index_0][index_0] = (c0 * c1 * c2) - (s0 * s2);
  (*this)[index_0][index_1] = (-c0 * c1 * s2) - (s0 * c2);
  (*this)[index_0][index_2] = c0 * s1;
  (*this)[index_1][index_0] = (s0 * c1 * c2) + (c0 * s2);
  (*this)[index_1][index_1] = (-s0 * c1 * s2) + (c0 * c2);
  (*this)[index_1][index_2] = s0 * s1;
  (*this)[index_2][index_0] = -s1 * c2;
  (*this)[index_2][index_1] = s1 * s2;
  (*this)[index_2][index_2] = c1;

  return *this;
}

/*!

  Transform a vector representing the Rxyz angle into a rotation
  matrix.
  Rxyz(\f$ \phi,\theta, \psi \f$) = Rot(\f$ x, \psi \f$) Rot(\f$ y, \theta \f$
  ) Rot(\f$ z,\phi \f$)

*/
vpRotationMatrix &vpRotationMatrix::build(const vpRxyzVector &v)
{
  double c0, c1, c2, s0, s1, s2;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;

  c0 = cos(v[index_0]);
  c1 = cos(v[index_1]);
  c2 = cos(v[index_2]);
  s0 = sin(v[index_0]);
  s1 = sin(v[index_1]);
  s2 = sin(v[index_2]);

  (*this)[index_0][index_0] = c1 * c2;
  (*this)[index_0][index_1] = -c1 * s2;
  (*this)[index_0][index_2] = s1;
  (*this)[index_1][index_0] = (c0 * s2) + (s0 * s1 * c2);
  (*this)[index_1][index_1] = (c0 * c2) - (s0 * s1 * s2);
  (*this)[index_1][index_2] = -s0 * c1;
  (*this)[index_2][index_0] = (-c0 * s1 * c2) + (s0 * s2);
  (*this)[index_2][index_1] = (c0 * s1 * s2) + (c2 * s0);
  (*this)[index_2][index_2] = c0 * c1;

  return *this;
}

/*!
  Transform a vector representing the Rzyx angle
  into a rotation matrix.
  Rxyz(\f$ \phi, \theta , \psi \f$) =
  Rot(\f$ z, \psi \f$) Rot(\f$ y, \theta \f$)Rot(\f$ x, \phi \f$)
*/
vpRotationMatrix &vpRotationMatrix::build(const vpRzyxVector &v)
{
  double c0, c1, c2, s0, s1, s2;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;

  c0 = cos(v[index_0]);
  c1 = cos(v[index_1]);
  c2 = cos(v[index_2]);
  s0 = sin(v[index_0]);
  s1 = sin(v[index_1]);
  s2 = sin(v[index_2]);

  (*this)[index_0][index_0] = c0 * c1;
  (*this)[index_0][index_1] = (c0 * s1 * s2) - (s0 * c2);
  (*this)[index_0][index_2] = (c0 * s1 * c2) + (s0 * s2);

  (*this)[index_1][index_0] = s0 * c1;
  (*this)[index_1][index_1] = (s0 * s1 * s2) + (c0 * c2);
  (*this)[index_1][index_2] = (s0 * s1 * c2) - (c0 * s2);

  (*this)[index_2][index_0] = -s1;
  (*this)[index_2][index_1] = c1 * s2;
  (*this)[index_2][index_2] = c1 * c2;

  return *this;
}

/*!
  Construct a 3-by-3 rotation matrix from \f$ \theta {\bf u}=(\theta u_x,
  \theta u_y, \theta u_z)^T\f$ angle representation.
 */
vpRotationMatrix &vpRotationMatrix::build(const double &tux, const double &tuy, const double &tuz)
{
  vpThetaUVector tu(tux, tuy, tuz);
  build(tu);
  return *this;
}

/*!
  Construct a 3-by-3 rotation matrix from a quaternion representation.
 */
vpRotationMatrix &vpRotationMatrix::build(const vpQuaternionVector &q)
{
  double a = q.w();
  double b = q.x();
  double c = q.y();
  double d = q.z();
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  (*this)[index_0][index_0] = (((a * a) + (b * b)) - (c * c)) - (d * d);
  (*this)[index_0][index_1] = (2 * b * c) - (2 * a * d);
  (*this)[index_0][index_2] = (2 * a * c) + (2 * b * d);

  (*this)[index_1][index_0] = (2 * a * d) + (2 * b * c);
  (*this)[index_1][index_1] = (((a * a) - (b * b)) + (c * c)) - (d * d);
  (*this)[index_1][index_2] = (2 * c * d) - (2 * a * b);

  (*this)[index_2][index_0] = (2 * b * d) - (2 * a * c);
  (*this)[index_2][index_1] = (2 * a * b) + (2 * c * d);
  (*this)[index_2][index_2] = ((a * a) - (b * b) - (c * c)) + (d * d);
  return *this;
}

/*!
  Return the \f$\theta {\bf u}\f$ vector that corresponds to the rotation
  matrix.
 */
vpThetaUVector vpRotationMatrix::getThetaUVector()
{
  vpThetaUVector tu;
  tu.build(*this);
  return tu;
}

/*!
  Extract a column vector from a rotation matrix.
  \warning All the indexes start from 0 in this function.
  \param j : Index of the column to extract. If j=0, the first column is extracted.
  \return The extracted column vector.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpRotationMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRotationMatrix R;

    vpColVector r = R.getCol(2);
    std::cout << "Last column: \n" << r << std::endl;
  }
  \endcode
  It produces the following output:
  \code
  Last column:
  0
  0
  1
  \endcode
 */
vpColVector vpRotationMatrix::getCol(unsigned int j) const
{
  if (j >= getCols()) {
    throw(vpException(vpException::dimensionError, "Unable to extract a column vector from the homogeneous matrix"));
  }
  unsigned int nb_rows = getRows();
  vpColVector c(nb_rows);
  for (unsigned int i = 0; i < nb_rows; ++i) {
    c[i] = (*this)[i][j];
  }
  return c;
}

/*!
  Compute the Euclidean mean of the rotation matrices extracted from a vector of homogeneous matrices following
  Moakher's method (SIAM 2002).

  \param[in] vec_M : Set of homogeneous matrices.
  \return The Euclidean mean of the rotation matrices.

  \sa vpTranslationVector::mean()
 */
vpRotationMatrix vpRotationMatrix::mean(const std::vector<vpHomogeneousMatrix> &vec_M)
{
  vpMatrix meanR(3, 3);
  vpRotationMatrix R;
  size_t vec_m_size = vec_M.size();
  for (size_t i = 0; i < vec_m_size; ++i) {
    R = vec_M[i].getRotationMatrix();
    meanR += (vpMatrix)R;
  }
  meanR /= static_cast<double>(vec_M.size());

  // Euclidean mean of the rotation matrix following Moakher's method (SIAM 2002)
  vpMatrix M, U, V;
  vpColVector sv;
  meanR.pseudoInverse(M, sv, 1e-6, U, V);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  double det = sv[index_0] * sv[index_1] * sv[index_2];
  if (det > 0) {
    meanR = U * V.t();
  }
  else {
    vpMatrix D(3, 3);
    D = 0.0;
    D[index_0][index_0] = 1.0;
    D[index_1][index_1] = 1.0;
    D[index_2][index_2] = -1;
    meanR = U * D * V.t();
  }

  R = meanR;
  return R;
}

/*!
  Compute the Euclidean mean of the rotation matrices following Moakher's method (SIAM 2002).

  \param[in] vec_R : Set of rotation matrices.
  \return The Euclidean mean of the rotation matrices.

  \sa vpTranslationVector::mean()
 */
vpRotationMatrix vpRotationMatrix::mean(const std::vector<vpRotationMatrix> &vec_R)
{
  vpMatrix meanR(3, 3);
  vpRotationMatrix R;
  size_t vec_r_size = vec_R.size();
  for (size_t i = 0; i < vec_r_size; ++i) {
    meanR += (vpMatrix)vec_R[i];
  }
  meanR /= static_cast<double>(vec_R.size());

  // Euclidean mean of the rotation matrix following Moakher's method (SIAM 2002)
  vpMatrix M, U, V;
  vpColVector sv;
  meanR.pseudoInverse(M, sv, 1e-6, U, V);
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  double det = sv[index_0] * sv[index_1] * sv[index_2];
  if (det > 0) {
    meanR = U * V.t();
  }
  else {
    vpMatrix D(3, 3);
    D = 0.0;
    D[index_0][index_0] = 1.0;
    D[index_1][index_1] = 1.0;
    D[index_2][index_2] = -1;
    meanR = U * D * V.t();
  }

  R = meanR;
  return R;
}

/*!
  Perform rotation matrix orthogonalization.
 */
void vpRotationMatrix::orthogonalize()
{
  vpMatrix U(*this);
  vpColVector w;
  vpMatrix V;
  U.svd(w, V);
  vpMatrix Vt = V.t();
  vpMatrix R = U * Vt;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  const unsigned int index_6 = 6;
  const unsigned int index_7 = 7;
  const unsigned int index_8 = 8;

  double det = R.det();
  if (det < 0) {
    Vt[index_2][index_0] *= -1;
    Vt[index_2][index_1] *= -1;
    Vt[index_2][index_2] *= -1;

    R = U * Vt;
  }

  data[index_0] = R[index_0][index_0];
  data[index_1] = R[index_0][index_1];
  data[index_2] = R[index_0][index_2];
  data[index_3] = R[index_1][index_0];
  data[index_4] = R[index_1][index_1];
  data[index_5] = R[index_1][index_2];
  data[index_6] = R[index_2][index_0];
  data[index_7] = R[index_2][index_1];
  data[index_8] = R[index_2][index_2];
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

/*!
  \deprecated You should rather use eye().

  Initializes the rotation matrix as identity.

  \sa eye()
*/
void vpRotationMatrix::setIdentity() { eye(); }

#endif //#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

/*!
  Allow to multiply a scalar by a rotation matrix.
*/
vpRotationMatrix operator*(const double &x, const vpRotationMatrix &R)
{
  vpRotationMatrix C;

  unsigned int Rrow = R.getRows();
  unsigned int Rcol = R.getCols();

  for (unsigned int i = 0; i < Rrow; ++i) {
    for (unsigned int j = 0; j < Rcol; ++j) {
      C[i][j] = R[i][j] * x;
    }
  }

  return C;
}
END_VISP_NAMESPACE
