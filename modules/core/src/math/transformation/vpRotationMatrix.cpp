/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * Rotation matrix.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

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
#include <visp3/core/vpDebug.h>
const double vpRotationMatrix::threshold = 1e-6;

/*!
  Initialize the rotation matrix as identity.

  \sa setIdentity()
*/
void vpRotationMatrix::eye()
{
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      if (i == j)
        (*this)[i][j] = 1.0;
      else
        (*this)[i][j] = 0.0;
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
  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      rowPtrs[i][j] = R.rowPtrs[i][j];
    }
  }

  return *this;
}

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

  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
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
   Compute the product between two rotation matrices.
 */
vpRotationMatrix vpRotationMatrix::operator*(const vpRotationMatrix &R) const
{
  vpRotationMatrix p;

  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      double s = 0;
      for (unsigned int k = 0; k < 3; k++)
        s += rowPtrs[i][k] * R.rowPtrs[k][j];
      p[i][j] = s;
    }
  }
  return p;
}
/*!
  Operator that allows to multiply a rotation matrix by a 3-by-3 matrix.
  Allows for example to multiply a rotation matrix by a skew matrix.
  \code
  vpRotationMatrix R;
  vpTranslationVector t;
  vpMatrix M = t.skew();
  vpMatrix RM = R * M;
  \endcode

  \exception vpException::dimensionError : If \e M is not
  a 3-by-3 dimension matrix.

*/
vpMatrix vpRotationMatrix::operator*(const vpMatrix &M) const
{
  if (M.getRows() != 3 || M.getCols() != 3) {
    throw(vpException(vpException::dimensionError, "Cannot set a (3x3) rotation matrix from a (%dx%d) matrix",
                      M.getRows(), M.getCols()));
  }
  vpMatrix p(3, 3);

  for (unsigned int i = 0; i < 3; i++) {
    for (unsigned int j = 0; j < 3; j++) {
      double s = 0;
      for (unsigned int k = 0; k < 3; k++)
        s += (*this)[i][k] * M[k][j];
      p[i][j] = s;
    }
  }
  return p;
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
  if (v.getRows() != 3) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply a (3x3) rotation matrix by a %d "
                      "dimension column vector",
                      v.getRows()));
  }
  vpColVector v_out(3);

  for (unsigned int j = 0; j < colNum; j++) {
    double vj = v[j]; // optimization em 5/12/2006
    for (unsigned int i = 0; i < rowNum; i++) {
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

  for (unsigned int j = 0; j < 3; j++)
    p[j] = 0;

  for (unsigned int j = 0; j < 3; j++) {
    for (unsigned int i = 0; i < 3; i++) {
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

  for (unsigned int i = 0; i < rowNum; i++)
    for (unsigned int j = 0; j < colNum; j++)
      R[i][j] = rowPtrs[i][j] * x;

  return R;
}

/*!
   Operator that allows to multiply all the elements of a rotation matrix
   by a scalar.
 */
vpRotationMatrix &vpRotationMatrix::operator*=(double x)
{
  for (unsigned int i = 0; i < rowNum; i++)
    for (unsigned int j = 0; j < colNum; j++)
      rowPtrs[i][j] *= x;

  return *this;
}

/*********************************************************************/

/*!
  Test if the rotation matrix is really a rotation matrix.
*/
bool vpRotationMatrix::isARotationMatrix() const
{
  unsigned int i, j;
  bool isRotation = true;

  // test R^TR = Id ;
  vpRotationMatrix RtR = (*this).t() * (*this);
  for (i = 0; i < 3; i++) {
    for (j = 0; j < 3; j++) {
      if (i == j) {
        if (fabs(RtR[i][j] - 1) > threshold)
          isRotation = false;
      } else {
        if (fabs(RtR[i][j]) > threshold)
          isRotation = false;
      }
    }
  }
  // test if it is a basis
  // test || Ci || = 1
  for (i = 0; i < 3; i++) {
    if ((sqrt(vpMath::sqr(RtR[0][i]) + vpMath::sqr(RtR[1][i]) + vpMath::sqr(RtR[2][i])) - 1) > threshold)
      isRotation = false;
  }

  // test || Ri || = 1
  for (i = 0; i < 3; i++) {
    if ((sqrt(vpMath::sqr(RtR[i][0]) + vpMath::sqr(RtR[i][1]) + vpMath::sqr(RtR[i][2])) - 1) > threshold)
      isRotation = false;
  }

  //  test if the basis is orthogonal
  return isRotation;
}

/*!
  Default constructor that initialise a 3-by-3 rotation matrix to identity.
*/
vpRotationMatrix::vpRotationMatrix() : vpArray2D<double>(3, 3) { eye(); }

/*!
  Copy contructor that construct a 3-by-3 rotation matrix from another
  rotation matrix.
*/
vpRotationMatrix::vpRotationMatrix(const vpRotationMatrix &M) : vpArray2D<double>(3, 3) { (*this) = M; }
/*!
  Construct a 3-by-3 rotation matrix from an homogeneous matrix.
*/
vpRotationMatrix::vpRotationMatrix(const vpHomogeneousMatrix &M) : vpArray2D<double>(3, 3) { buildFrom(M); }

/*!
  Construct a 3-by-3 rotation matrix from \f$ \theta {\bf u}\f$ angle
  representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpThetaUVector &tu) : vpArray2D<double>(3, 3) { buildFrom(tu); }

/*!
  Construct a 3-by-3 rotation matrix from a pose vector.
 */
vpRotationMatrix::vpRotationMatrix(const vpPoseVector &p) : vpArray2D<double>(3, 3) { buildFrom(p); }

/*!
  Construct a 3-by-3 rotation matrix from \f$ R(z,y,z) \f$ Euler angle
  representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpRzyzVector &euler) : vpArray2D<double>(3, 3) { buildFrom(euler); }

/*!
  Construct a 3-by-3 rotation matrix from \f$ R(x,y,z) \f$ Euler angle
  representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpRxyzVector &Rxyz) : vpArray2D<double>(3, 3) { buildFrom(Rxyz); }

/*!
  Construct a 3-by-3 rotation matrix from \f$ R(z,y,x) \f$ Euler angle
  representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpRzyxVector &Rzyx) : vpArray2D<double>(3, 3) { buildFrom(Rzyx); }

/*!
  Construct a 3-by-3 rotation matrix from a matrix that contains values corresponding to a rotation matrix.
*/
vpRotationMatrix::vpRotationMatrix(const vpMatrix &R) : vpArray2D<double>(3, 3) { *this = R; }

/*!
  Construct a 3-by-3 rotation matrix from \f$ \theta {\bf u}=(\theta u_x,
  \theta u_y, \theta u_z)^T\f$ angle representation.
 */
vpRotationMatrix::vpRotationMatrix(const double tux, const double tuy, const double tuz) : vpArray2D<double>(3, 3)
{
  buildFrom(tux, tuy, tuz);
}

/*!
  Construct a 3-by-3 rotation matrix from quaternion angle representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpQuaternionVector &q) : vpArray2D<double>(3, 3) { buildFrom(q); }

/*!
  Return the rotation matrix transpose which is also the inverse of the
  rotation matrix.

  \sa inverse()
*/
vpRotationMatrix vpRotationMatrix::t() const
{
  vpRotationMatrix Rt;

  unsigned int i, j;
  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      Rt[j][i] = (*this)[i][j];

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

  for (unsigned int i = 0; i < 3; i++)
    std::cout << tu[i] << "  ";

  std::cout << std::endl;
}

/*!
  Transform a \f$ \theta {\bf u}\f$ angle representation into a rotation
  matrix.

  The rotation is computed using :
  \f[
  R = \cos{ \theta} \; {I}_{3} + (1 - \cos{ \theta}) \; u u^{T} + \sin{
  \theta} \; [u]_\times \f]
*/
vpRotationMatrix vpRotationMatrix::buildFrom(const vpThetaUVector &v)
{
  unsigned int i, j;
  double theta, si, co, sinc, mcosc;
  vpRotationMatrix R;

  theta = sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
  si = sin(theta);
  co = cos(theta);
  sinc = vpMath::sinc(si, theta);
  mcosc = vpMath::mcosc(co, theta);

  R[0][0] = co + mcosc * v[0] * v[0];
  R[0][1] = -sinc * v[2] + mcosc * v[0] * v[1];
  R[0][2] = sinc * v[1] + mcosc * v[0] * v[2];
  R[1][0] = sinc * v[2] + mcosc * v[1] * v[0];
  R[1][1] = co + mcosc * v[1] * v[1];
  R[1][2] = -sinc * v[0] + mcosc * v[1] * v[2];
  R[2][0] = -sinc * v[1] + mcosc * v[2] * v[0];
  R[2][1] = sinc * v[0] + mcosc * v[2] * v[1];
  R[2][2] = co + mcosc * v[2] * v[2];

  for (i = 0; i < 3; i++)
    for (j = 0; j < 3; j++)
      (*this)[i][j] = R[i][j];

  return *this;
}

/*!
  Build a rotation matrix from an homogeneous matrix.
*/
vpRotationMatrix vpRotationMatrix::buildFrom(const vpHomogeneousMatrix &M)
{
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      (*this)[i][j] = M[i][j];

  return *this;
}

/*!
  Build a rotation matrix from a pose vector.

  \sa buildFrom(const vpThetaUVector &)
*/
vpRotationMatrix vpRotationMatrix::buildFrom(const vpPoseVector &p)
{
  vpThetaUVector tu(p);
  return buildFrom(tu);
}

/*!
  Transform a vector representing the Euler angle
  into a rotation matrix.
  Rzyz(\f$ \phi, \theta , \psi \f$) =  Rot(\f$ z,\phi \f$) Rot(\f$ y,\theta
  \f$) Rot(\f$ z,\psi \f$)

*/
vpRotationMatrix vpRotationMatrix::buildFrom(const vpRzyzVector &v)
{
  double c0, c1, c2, s0, s1, s2;

  c0 = cos(v[0]);
  c1 = cos(v[1]);
  c2 = cos(v[2]);
  s0 = sin(v[0]);
  s1 = sin(v[1]);
  s2 = sin(v[2]);

  (*this)[0][0] = c0 * c1 * c2 - s0 * s2;
  (*this)[0][1] = -c0 * c1 * s2 - s0 * c2;
  (*this)[0][2] = c0 * s1;
  (*this)[1][0] = s0 * c1 * c2 + c0 * s2;
  (*this)[1][1] = -s0 * c1 * s2 + c0 * c2;
  (*this)[1][2] = s0 * s1;
  (*this)[2][0] = -s1 * c2;
  (*this)[2][1] = s1 * s2;
  (*this)[2][2] = c1;

  return (*this);
}

/*!

  Transform a vector representing the Rxyz angle into a rotation
  matrix.
  Rxyz(\f$ \phi,\theta, \psi \f$) = Rot(\f$ x, \psi \f$) Rot(\f$ y, \theta \f$
  ) Rot(\f$ z,\phi \f$)

*/
vpRotationMatrix vpRotationMatrix::buildFrom(const vpRxyzVector &v)
{
  double c0, c1, c2, s0, s1, s2;

  c0 = cos(v[0]);
  c1 = cos(v[1]);
  c2 = cos(v[2]);
  s0 = sin(v[0]);
  s1 = sin(v[1]);
  s2 = sin(v[2]);

  (*this)[0][0] = c1 * c2;
  (*this)[0][1] = -c1 * s2;
  (*this)[0][2] = s1;
  (*this)[1][0] = c0 * s2 + s0 * s1 * c2;
  (*this)[1][1] = c0 * c2 - s0 * s1 * s2;
  (*this)[1][2] = -s0 * c1;
  (*this)[2][0] = -c0 * s1 * c2 + s0 * s2;
  (*this)[2][1] = c0 * s1 * s2 + c2 * s0;
  (*this)[2][2] = c0 * c1;

  return (*this);
}

/*!
  Transform a vector representing the Rzyx angle
  into a rotation matrix.
  Rxyz(\f$ \phi, \theta , \psi \f$) =
  Rot(\f$ z, \psi \f$) Rot(\f$ y, \theta \f$)Rot(\f$ x, \phi \f$)
*/
vpRotationMatrix vpRotationMatrix::buildFrom(const vpRzyxVector &v)
{
  double c0, c1, c2, s0, s1, s2;

  c0 = cos(v[0]);
  c1 = cos(v[1]);
  c2 = cos(v[2]);
  s0 = sin(v[0]);
  s1 = sin(v[1]);
  s2 = sin(v[2]);

  (*this)[0][0] = c0 * c1;
  (*this)[0][1] = c0 * s1 * s2 - s0 * c2;
  (*this)[0][2] = c0 * s1 * c2 + s0 * s2;

  (*this)[1][0] = s0 * c1;
  (*this)[1][1] = s0 * s1 * s2 + c0 * c2;
  (*this)[1][2] = s0 * s1 * c2 - c0 * s2;

  (*this)[2][0] = -s1;
  (*this)[2][1] = c1 * s2;
  (*this)[2][2] = c1 * c2;

  return (*this);
}

/*!
  Construct a 3-by-3 rotation matrix from \f$ \theta {\bf u}=(\theta u_x,
  \theta u_y, \theta u_z)^T\f$ angle representation.
 */
vpRotationMatrix vpRotationMatrix::buildFrom(const double tux, const double tuy, const double tuz)
{
  vpThetaUVector tu(tux, tuy, tuz);
  buildFrom(tu);
  return *this;
}

/*!
  Construct a 3-by-3 rotation matrix from a quaternion representation.
 */
vpRotationMatrix vpRotationMatrix::buildFrom(const vpQuaternionVector &q)
{
  double a = q.w();
  double b = q.x();
  double c = q.y();
  double d = q.z();
  (*this)[0][0] = a * a + b * b - c * c - d * d;
  (*this)[0][1] = 2 * b * c - 2 * a * d;
  (*this)[0][2] = 2 * a * c + 2 * b * d;

  (*this)[1][0] = 2 * a * d + 2 * b * c;
  (*this)[1][1] = a * a - b * b + c * c - d * d;
  (*this)[1][2] = 2 * c * d - 2 * a * b;

  (*this)[2][0] = 2 * b * d - 2 * a * c;
  (*this)[2][1] = 2 * a * b + 2 * c * d;
  (*this)[2][2] = a * a - b * b - c * c + d * d;
  return *this;
}

/*!
  Allow to multiply a scalar by a rotation matrix.
*/
vpRotationMatrix operator*(const double &x, const vpRotationMatrix &R)
{
  vpRotationMatrix C;

  unsigned int Rrow = R.getRows();
  unsigned int Rcol = R.getCols();

  for (unsigned int i = 0; i < Rrow; i++)
    for (unsigned int j = 0; j < Rcol; j++)
      C[i][j] = R[i][j] * x;

  return C;
}

/*!
  Return the \f$\theta {\bf u}\f$ vector that corresponds to the rotation
  matrix.
 */
vpThetaUVector vpRotationMatrix::getThetaUVector()
{
  vpThetaUVector tu;
  tu.buildFrom(*this);
  return tu;
}

/*!
  Extract a column vector from a rotation matrix.
  \warning All the indexes start from 0 in this function.
  \param j : Index of the column to extract. If j=0, the first column is
extracted. \return The extracted column vector.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpRotationMatrix.h>

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
vpColVector vpRotationMatrix::getCol(const unsigned int j) const
{
  if (j >= getCols())
    throw(vpException(vpException::dimensionError, "Unable to extract a column vector from the homogeneous matrix"));
  unsigned int nb_rows = getRows();
  vpColVector c(nb_rows);
  for (unsigned int i = 0; i < nb_rows; i++)
    c[i] = (*this)[i][j];
  return c;
}

/*!
  Compute the Euclidean mean of the rotation matrices extracted from a vector of homogeneous matrices following Moakher's method (SIAM 2002).

  \param[in] vec_M : Set of homogeneous matrices.
  \return The Euclidian mean of the rotation matrices.

  \sa vpTranslationVector::mean()
 */
vpRotationMatrix vpRotationMatrix::mean(const std::vector<vpHomogeneousMatrix> &vec_M)
{
  vpMatrix meanR(3, 3);
  vpRotationMatrix R;
  for (size_t i = 0; i < vec_M.size(); i++) {
    R = vec_M[i].getRotationMatrix();
    meanR += (vpMatrix) R;
  }
  meanR /= static_cast<double>(vec_M.size());

  // Euclidean mean of the rotation matrix following Moakher's method (SIAM 2002)
  vpMatrix M, U, V;
  vpColVector sv;
  meanR.pseudoInverse(M, sv, 1e-6, U, V);
  double det = sv[0]*sv[1]*sv[2];
  if (det > 0) {
    meanR = U * V.t();
  }
  else {
    vpMatrix D(3,3);
    D = 0.0;
    D[0][0] = D[1][1] = 1.0; D[2][2] = -1;
    meanR = U * D * V.t();
  }

  R = meanR;
  return R;
}

/*!
  Compute the Euclidean mean of the rotation matrices following Moakher's method (SIAM 2002).

  \param[in] vec_R : Set of rotation matrices.
  \return The Euclidian mean of the rotation matrices.

  \sa vpTranslationVector::mean()
 */
vpRotationMatrix vpRotationMatrix::mean(const std::vector<vpRotationMatrix> &vec_R)
{
  vpMatrix meanR(3, 3);
  vpRotationMatrix R;
  for (size_t i = 0; i < vec_R.size(); i++) {
    meanR += (vpMatrix) vec_R[i];
  }
  meanR /= static_cast<double>(vec_R.size());

  // Euclidean mean of the rotation matrix following Moakher's method (SIAM 2002)
  vpMatrix M, U, V;
  vpColVector sv;
  meanR.pseudoInverse(M, sv, 1e-6, U, V);
  double det = sv[0]*sv[1]*sv[2];
  if (det > 0) {
    meanR = U * V.t();
  }
  else {
    vpMatrix D(3,3);
    D = 0.0;
    D[0][0] = D[1][1] = 1.0; D[2][2] = -1;
    meanR = U * D * V.t();
  }

  R = meanR;
  return R;
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

/*!
  \deprecated You should rather use eye().

  Initializes the rotation matrix as identity.

  \sa eye()
*/
void vpRotationMatrix::setIdentity() { eye(); }

#endif //#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
