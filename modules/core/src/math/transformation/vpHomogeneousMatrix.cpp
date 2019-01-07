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
 * Homogeneous matrix.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpHomogeneousMatrix.cpp
  \brief Defines vpHomogeneousMatrix class. Class that consider
  the particular case of an homogeneous matrix.
*/

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpQuaternionVector.h>

/*!
  Construct an homogeneous matrix from a translation vector and quaternion
  rotation vector.
 */
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpTranslationVector &t, const vpQuaternionVector &q)
  : vpArray2D<double>(4, 4)
{
  buildFrom(t, q);
  (*this)[3][3] = 1.;
}

/*!
  Default constructor that initialize an homogeneous matrix as identity.
*/
vpHomogeneousMatrix::vpHomogeneousMatrix() : vpArray2D<double>(4, 4) { eye(); }

/*!
  Copy constructor that initialize an homogeneous matrix from another
  homogeneous matrix.
*/
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpHomogeneousMatrix &M) : vpArray2D<double>(4, 4) { *this = M; }

/*!
  Construct an homogeneous matrix from a translation vector and \f$\theta {\bf
  u}\f$ rotation vector.
 */
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpTranslationVector &t, const vpThetaUVector &tu)
  : vpArray2D<double>(4, 4)
{
  buildFrom(t, tu);
  (*this)[3][3] = 1.;
}

/*!
  Construct an homogeneous matrix from a translation vector and a rotation
  matrix.
 */
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpTranslationVector &t, const vpRotationMatrix &R)
  : vpArray2D<double>(4, 4)
{
  insert(R);
  insert(t);
  (*this)[3][3] = 1.;
}

/*!
  Construct an homogeneous matrix from a pose vector.
 */
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpPoseVector &p) : vpArray2D<double>(4, 4)
{
  buildFrom(p[0], p[1], p[2], p[3], p[4], p[5]);
  (*this)[3][3] = 1.;
}

/*!
  Construct an homogeneous matrix from a vector of float.
  \param v : Vector of 12 or 16 values corresponding to the values of the
homogeneous matrix.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpHomogeneousMatrix.h>

int main()
{
  std::vector<float> v(12, 0);
  v[1]  = -1.; // ry=-90
  v[4]  =  1.; // rx=90
  v[10] = -1.; // rz=-90
  v[3]  = 0.3; // tx
  v[7]  = 0.4; // ty
  v[11] = 0.5; // tz

  std::cout << "v: ";
  for(unsigned int i=0; i<v.size(); i++)
    std::cout << v[i] << " ";
  std::cout << std::endl;

  vpHomogeneousMatrix M(v);
  std::cout << "M:\n" << M << std::endl;
}
  \endcode

  It produces the following printings:
  \code
v: 0 -1 0 0.3 1 0 0 0.4 0 0 -1 0.5
M:
0  -1  0  0.3000000119
1  0  0  0.400000006
0  0  -1  0.5
0  0  0  1
  \endcode
  */
vpHomogeneousMatrix::vpHomogeneousMatrix(const std::vector<float> &v) : vpArray2D<double>(4, 4)
{
  buildFrom(v);
  (*this)[3][3] = 1.;
}

/*!
  Construct an homogeneous matrix from a vector of double.
  \param v : Vector of 12 or 16 values corresponding to the values of the
homogeneous matrix.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpHomogeneousMatrix.h>

int main()
{
  std::vector<double> v(12, 0);
  v[1]  = -1.; // ry=-90
  v[4]  =  1.; // rx=90
  v[10] = -1.; // rz=-90
  v[3]  = 0.3; // tx
  v[7]  = 0.4; // ty
  v[11] = 0.5; // tz

  std::cout << "v: ";
  for(unsigned int i=0; i<v.size(); i++)
    std::cout << v[i] << " ";
  std::cout << std::endl;

  vpHomogeneousMatrix M(v);
  std::cout << "M:\n" << M << std::endl;
}
  \endcode

  It produces the following printings:
  \code
v: 0 -1 0 0.3 1 0 0 0.4 0 0 -1 0.5
M:
0  -1  0  0.3
1  0  0  0.4
0  0  -1  0.5
0  0  0  1
  \endcode
  */
vpHomogeneousMatrix::vpHomogeneousMatrix(const std::vector<double> &v) : vpArray2D<double>(4, 4)
{
  buildFrom(v);
  (*this)[3][3] = 1.;
}

/*!
  Construct an homogeneous matrix from a translation vector \f${\bf t}=(t_x,
  t_y, t_z)^T\f$ and a \f$\theta {\bf u}=(\theta u_x, \theta u_y, \theta
  u_z)^T\f$ rotation vector.
 */
vpHomogeneousMatrix::vpHomogeneousMatrix(const double tx, const double ty, const double tz, const double tux,
                                         const double tuy, const double tuz)
  : vpArray2D<double>(4, 4)
{
  buildFrom(tx, ty, tz, tux, tuy, tuz);
  (*this)[3][3] = 1.;
}

/*!
  Build an homogeneous matrix from a translation vector
  and a \f$\theta {\bf u}\f$ rotation vector.
 */
void vpHomogeneousMatrix::buildFrom(const vpTranslationVector &t, const vpThetaUVector &tu)
{
  insert(tu);
  insert(t);
}

/*!
  Build an homogeneous matrix from a translation vector
  and a rotation matrix.
 */
void vpHomogeneousMatrix::buildFrom(const vpTranslationVector &t, const vpRotationMatrix &R)
{
  insert(R);
  insert(t);
}

/*!
  Build an homogeneous matrix from a pose vector.
 */
void vpHomogeneousMatrix::buildFrom(const vpPoseVector &p)
{
  vpTranslationVector tv(p[0], p[1], p[2]);
  vpThetaUVector tu(p[3], p[4], p[5]);

  insert(tu);
  insert(tv);
}

/*!
  Build an homogeneous matrix from a translation vector
  and a quaternion rotation vector.
 */
void vpHomogeneousMatrix::buildFrom(const vpTranslationVector &t, const vpQuaternionVector &q)
{
  insert(t);
  insert(q);
}

/*!
  Build an homogeneous matrix from a translation vector \f${\bf t}=(t_x, t_y,
  t_z)^T\f$ and a \f$\theta {\bf u}=(\theta u_x, \theta u_y, \theta u_z)^T\f$
  rotation vector.
 */
void vpHomogeneousMatrix::buildFrom(const double tx, const double ty, const double tz, const double tux,
                                    const double tuy, const double tuz)
{
  vpRotationMatrix R(tux, tuy, tuz);
  vpTranslationVector t(tx, ty, tz);

  insert(R);
  insert(t);
}

/*!
  Build an homogeneous matrix from a vector of float.
  \param v : Vector of 12 or 16 values corresponding to the values of the
homogeneous matrix.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpHomogeneousMatrix.h>

int main()
{
  std::vector<float> v(12, 0);
  v[1]  = -1.; // ry=-90
  v[4]  =  1.; // rx=90
  v[10] = -1.; // rz=-90
  v[3]  = 0.3; // tx
  v[7]  = 0.4; // ty
  v[11] = 0.5; // tz

  std::cout << "v: ";
  for(unsigned int i=0; i<v.size(); i++)
    std::cout << v[i] << " ";
  std::cout << std::endl;

  vpHomogeneousMatrix M;
  M.buildFrom(v);
  std::cout << "M:\n" << M << std::endl;
}
  \endcode

  It produces the following printings:
  \code
v: 0 -1 0 0.3 1 0 0 0.4 0 0 -1 0.5
M:
0  -1  0  0.3000000119
1  0  0  0.400000006
0  0  -1  0.5
0  0  0  1
  \endcode
  */
void vpHomogeneousMatrix::buildFrom(const std::vector<float> &v)
{
  if (v.size() != 12 && v.size() != 16) {
    throw(vpException(vpException::dimensionError, "Cannot convert std::vector<float> to vpHomogeneousMatrix"));
  }

  for (unsigned int i = 0; i < 12; i++)
    this->data[i] = (double)v[i];
}

/*!
  Build an homogeneous matrix from a vector of double.
  \param v : Vector of 12 or 16 values corresponding to the values of the
homogeneous matrix.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpHomogeneousMatrix.h>

int main()
{
  std::vector<double> v(12, 0);
  v[1]  = -1.; // ry=-90
  v[4]  =  1.; // rx=90
  v[10] = -1.; // rz=-90
  v[3]  = 0.3; // tx
  v[7]  = 0.4; // ty
  v[11] = 0.5; // tz

  std::cout << "v: ";
  for(unsigned int i=0; i<v.size(); i++)
    std::cout << v[i] << " ";
  std::cout << std::endl;

  vpHomogeneousMatrix M;
  M.buildFrom(v);
  std::cout << "M:\n" << M << std::endl;
}
  \endcode

  It produces the following printings:
  \code
v: 0 -1 0 0.3 1 0 0 0.4 0 0 -1 0.5
M:
0  -1  0  0.3
1  0  0  0.4
0  0  -1  0.5
0  0  0  1
  \endcode
  */
void vpHomogeneousMatrix::buildFrom(const std::vector<double> &v)
{
  if (v.size() != 12 && v.size() != 16) {
    throw(vpException(vpException::dimensionError, "Cannot convert std::vector<double> to vpHomogeneousMatrix"));
  }

  for (unsigned int i = 0; i < 12; i++)
    this->data[i] = v[i];
}

/*!
  Copy operator that allows to set an homogeneous matrix from an other one.

  \param M : Matrix to copy.
*/
vpHomogeneousMatrix &vpHomogeneousMatrix::operator=(const vpHomogeneousMatrix &M)
{
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      rowPtrs[i][j] = M.rowPtrs[i][j];
    }
  }
  return *this;
}

/*!
  Operator that allow to multiply an homogeneous matrix by an other one.

  \code
#include <visp3/core/vpHomogeneousMatrix.h>

int main()
{
  vpHomogeneousMatrix aMb, bMc;
  // Initialize aMb and bMc...

  // Compute aMc * bMc
  vpHomogeneousMatrix aMc = aMb * bMc;
}
  \endcode

*/
vpHomogeneousMatrix vpHomogeneousMatrix::operator*(const vpHomogeneousMatrix &M) const
{
  vpHomogeneousMatrix p;

  vpRotationMatrix R1, R2, R;
  vpTranslationVector T1, T2, T;

  extract(T1);
  M.extract(T2);

  extract(R1);
  M.extract(R2);

  R = R1 * R2;

  T = R1 * T2 + T1;

  p.insert(T);
  p.insert(R);

  return p;
}

/*!
  Operator that allow to multiply an homogeneous matrix by an other one.

  \code
#include <visp3/core/vpHomogeneousMatrix.h>

int main()
{
  vpHomogeneousMatrix M1, M2;
  // Initialize M1 and M2...

  // Compute M1 = M1 * M2
  M1 *= M2;
}
  \endcode

*/
vpHomogeneousMatrix &vpHomogeneousMatrix::operator*=(const vpHomogeneousMatrix &M)
{
  (*this) = (*this) * M;
  return (*this);
}

/*!
  Operator that allow to multiply an homogeneous matrix by a 4-dimension
  column vector.

  \exception vpException::dimensionError : If the vector \e v is not a
  4-dimension vector.
*/
vpColVector vpHomogeneousMatrix::operator*(const vpColVector &v) const
{
  if (v.getRows() != 4) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply a (4x4) homogeneous matrix by a "
                      "(%dx1) column vector",
                      v.getRows()));
  }
  vpColVector p(rowNum);

  p = 0.0;

  for (unsigned int j = 0; j < 4; j++) {
    for (unsigned int i = 0; i < 4; i++) {
      p[i] += rowPtrs[i][j] * v[j];
    }
  }

  return p;
}

/*!
  From the coordinates of the point in camera frame b and the transformation
  between camera frame a and camera frame b computes the coordinates of the
  point in camera frame a.

  \param bP : 3D coordinates of the point in camera frame bP.

  \return A point with 3D coordinates in the camera frame a. The coordinates
  in the world or object frame are set to the same coordinates than the one in
  the camera frame.
*/
vpPoint vpHomogeneousMatrix::operator*(const vpPoint &bP) const
{
  vpPoint aP;

  vpColVector v(4), v1(4);

  v[0] = bP.get_X();
  v[1] = bP.get_Y();
  v[2] = bP.get_Z();
  v[3] = bP.get_W();

  v1[0] = (*this)[0][0] * v[0] + (*this)[0][1] * v[1] + (*this)[0][2] * v[2] + (*this)[0][3] * v[3];
  v1[1] = (*this)[1][0] * v[0] + (*this)[1][1] * v[1] + (*this)[1][2] * v[2] + (*this)[1][3] * v[3];
  v1[2] = (*this)[2][0] * v[0] + (*this)[2][1] * v[1] + (*this)[2][2] * v[2] + (*this)[2][3] * v[3];
  v1[3] = (*this)[3][0] * v[0] + (*this)[3][1] * v[1] + (*this)[3][2] * v[2] + (*this)[3][3] * v[3];

  v1 /= v1[3];

  //  v1 = M*v ;
  aP.set_X(v1[0]);
  aP.set_Y(v1[1]);
  aP.set_Z(v1[2]);
  aP.set_W(v1[3]);

  aP.set_oX(v1[0]);
  aP.set_oY(v1[1]);
  aP.set_oZ(v1[2]);
  aP.set_oW(v1[3]);

  return aP;
}

/*!
  Since a translation vector could be seen as the origin point of a frame,
  this function computes the new coordinates of a translation vector after
  applying an homogeneous transformation.

  \param t : Translation vector seen as the 3D coordinates of a point.

  \return A translation vector that contains the new 3D coordinates after
  applying the homogeneous transformation.
*/
vpTranslationVector vpHomogeneousMatrix::operator*(const vpTranslationVector &t) const
{
  vpTranslationVector t_out;
  t_out[0] = (*this)[0][0] * t[0] + (*this)[0][1] * t[1] + (*this)[0][2] * t[2] + (*this)[0][3];
  t_out[1] = (*this)[1][0] * t[0] + (*this)[1][1] * t[1] + (*this)[1][2] * t[2] + (*this)[1][3];
  t_out[2] = (*this)[2][0] * t[0] + (*this)[2][1] * t[1] + (*this)[2][2] * t[2] + (*this)[2][3];

  return t_out;
}

/*********************************************************************/

/*!
  Test if the 3x3 rotational part of the homogeneous matrix is really
  a rotation matrix.
*/
bool vpHomogeneousMatrix::isAnHomogeneousMatrix() const
{
  vpRotationMatrix R;
  extract(R);

  return R.isARotationMatrix();
}

/*!
  Extract the rotational matrix from the homogeneous matrix.
  \param R : rotational component as a rotation matrix.
*/
void vpHomogeneousMatrix::extract(vpRotationMatrix &R) const
{
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      R[i][j] = (*this)[i][j];
}

/*!
  Extract the translation vector from the homogeneous matrix.
*/
void vpHomogeneousMatrix::extract(vpTranslationVector &t) const
{
  t[0] = (*this)[0][3];
  t[1] = (*this)[1][3];
  t[2] = (*this)[2][3];
}
/*!
  Extract the rotation as a \f$\theta \bf u\f$ vector.
*/
void vpHomogeneousMatrix::extract(vpThetaUVector &tu) const
{
  vpRotationMatrix R;
  (*this).extract(R);
  tu.buildFrom(R);
}

/*!
  Extract the rotation as a quaternion.
*/
void vpHomogeneousMatrix::extract(vpQuaternionVector &q) const
{
  vpRotationMatrix R;
  (*this).extract(R);
  q.buildFrom(R);
}

/*!
  Insert the rotational component of the homogeneous matrix.
*/
void vpHomogeneousMatrix::insert(const vpRotationMatrix &R)
{
  for (unsigned int i = 0; i < 3; i++)
    for (unsigned int j = 0; j < 3; j++)
      (*this)[i][j] = R[i][j];
}

/*!

  Insert the rotational component of the homogeneous matrix from a
  \f$theta {\bf u}\f$ rotation vector.

*/
void vpHomogeneousMatrix::insert(const vpThetaUVector &tu)
{
  vpRotationMatrix R(tu);
  insert(R);
}

/*!
  Insert the translational component in a homogeneous matrix.
*/
void vpHomogeneousMatrix::insert(const vpTranslationVector &t)
{
  (*this)[0][3] = t[0];
  (*this)[1][3] = t[1];
  (*this)[2][3] = t[2];
}

/*!

  Insert the rotational component of the homogeneous matrix from a
  quaternion rotation vector.

*/
void vpHomogeneousMatrix::insert(const vpQuaternionVector &q) { insert(vpRotationMatrix(q)); }

/*!
  Invert the homogeneous matrix

  \return \f$\left[\begin{array}{cc}
  {\bf R} & {\bf t} \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right]^{-1} = \left[\begin{array}{cc}
  {\bf R}^T & -{\bf R}^T {\bf t} \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right]\f$

*/
vpHomogeneousMatrix vpHomogeneousMatrix::inverse() const
{
  vpHomogeneousMatrix Mi;

  vpRotationMatrix R;
  extract(R);
  vpTranslationVector T;
  extract(T);

  vpTranslationVector RtT;
  RtT = -(R.t() * T);

  Mi.insert(R.t());
  Mi.insert(RtT);

  return Mi;
}

/*!
  Set transformation to identity.
*/
void vpHomogeneousMatrix::eye()
{
  (*this)[0][0] = 1;
  (*this)[1][1] = 1;
  (*this)[2][2] = 1;
  (*this)[3][3] = 1;

  (*this)[0][1] = (*this)[0][2] = (*this)[0][3] = 0;
  (*this)[1][0] = (*this)[1][2] = (*this)[1][3] = 0;
  (*this)[2][0] = (*this)[2][1] = (*this)[2][3] = 0;
  (*this)[3][0] = (*this)[3][1] = (*this)[3][2] = 0;
}

/*!
  Invert the homogeneous matrix.

  \param M : The inverted homogeneous matrix: \f$\left[\begin{array}{cc}
  {\bf R} & {\bf t} \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right]^{-1} = \left[\begin{array}{cc}
  {\bf R}^T & -{\bf R}^T {\bf t} \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right]\f$

*/
void vpHomogeneousMatrix::inverse(vpHomogeneousMatrix &M) const { M = inverse(); }

/*!
  Write an homogeneous matrix in an output file stream.

  \param f : Output file stream. The homogeneous matrix is saved as a
  4 by 4 matrix.

  The code below shows how to save an homogeneous matrix in a file.

  \code
  // Contruct an homogeneous matrix
  vpTranslationVector t(1,2,3);
  vpRxyzVector r(M_PI, 0, -M_PI/4.);
  vpRotationMatrix R(r);
  vpHomogeneousMatrix M(t, R);

  // Save the content of the matrix in "homogeneous.dat"
  std::ofstream f("homogeneous.dat");
  M.save(f);
  \endcode

  \sa load()
*/
void vpHomogeneousMatrix::save(std::ofstream &f) const
{
  if (!f.fail()) {
    f << *this;
  } else {
    throw(vpException(vpException::ioError, "Cannot save homogeneous matrix: ostream not open"));
  }
}

/*!

  Read an homogeneous matrix from an input file stream. The
  homogeneous matrix is considered as a 4 by 4 matrix.

  \param f : Input file stream.

  The code below shows how to get an homogeneous matrix from a file.

  \code
  vpHomogeneousMatrix M;

  std::ifstream f("homogeneous.dat");
  M.load(f);
  \endcode

  \sa save()
*/
void vpHomogeneousMatrix::load(std::ifstream &f)
{
  if (!f.fail()) {
    for (unsigned int i = 0; i < 4; i++) {
      for (unsigned int j = 0; j < 4; j++) {
        f >> (*this)[i][j];
      }
    }
  } else {
    throw(vpException(vpException::ioError, "Cannot laad homogeneous matrix: ifstream not open"));
  }
}

//! Print the matrix as a pose vector \f$({\bf t}^T \theta {\bf u}^T)\f$
void vpHomogeneousMatrix::print() const
{
  vpPoseVector r(*this);
  std::cout << r.t();
}

/*!
  Converts an homogeneous matrix to a vector of 12 floats.
  \param M : Converted matrix.
 */
void vpHomogeneousMatrix::convert(std::vector<float> &M)
{
  M.resize(12);
  for (unsigned int i = 0; i < 12; i++)
    M[i] = (float)(this->data[i]);
}

/*!
  Converts an homogeneous matrix to a vector of 12 doubles.
  \param M : Converted matrix.
 */
void vpHomogeneousMatrix::convert(std::vector<double> &M)
{
  M.resize(12);
  for (unsigned int i = 0; i < 12; i++)
    M[i] = this->data[i];
}

/*!
  Return the translation vector from the homogeneous transformation matrix.
 */
vpTranslationVector vpHomogeneousMatrix::getTranslationVector() const
{
  vpTranslationVector tr;
  this->extract(tr);
  return tr;
}

/*!
  Return the rotation matrix from the homogeneous transformation matrix.
 */
vpRotationMatrix vpHomogeneousMatrix::getRotationMatrix() const
{
  vpRotationMatrix R;
  this->extract(R);
  return R;
}

/*!
  Return the \f$\theta {\bf u}\f$ vector that corresponds to the rotation part
  of the homogeneous transformation.
 */
vpThetaUVector vpHomogeneousMatrix::getThetaUVector() const
{
  vpThetaUVector tu;
  this->extract(tu);
  return tu;
}

/*!
  Extract a column vector from an homogeneous matrix.
  \warning All the indexes start from 0 in this function.
  \param j : Index of the column to extract. If j=0, the first column is
extracted. \return The extracted column vector.

  The following example shows how to use this function:
  \code
#include <visp3/core/vpColVector.h>
#include <visp3/core/vpHomogeneousMatrix.h>

int main()
{
  vpHomogeneousMatrix M;

  vpColVector t = M.getCol(3);
  std::cout << "Last column: \n" << t << std::endl;
}
  \endcode
It produces the following output:
  \code
Last column:
0
0
1
0
  \endcode
 */
vpColVector vpHomogeneousMatrix::getCol(const unsigned int j) const
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
  Compute the Euclidean mean of the homogeneous matrices.
  The Euclidean mean of the rotation matrices is computed following Moakher's method (SIAM 2002).

  \param[in] vec_M : Set of homogeneous matrices.
  \return The Euclidian mean of the homogeneous matrices.

  \sa vpTranslationVector::mean(), vpRotationMatrix::mean()
 */
vpHomogeneousMatrix vpHomogeneousMatrix::mean(const std::vector<vpHomogeneousMatrix> &vec_M)
{
  vpMatrix meanR(3, 3);
  vpColVector meanT(3);
  vpRotationMatrix R;
  for (size_t i = 0; i < vec_M.size(); i++) {
    R = vec_M[i].getRotationMatrix();
    meanR += (vpMatrix) R;
    meanT += (vpColVector) vec_M[i].getTranslationVector();
  }
  meanR /= static_cast<double>(vec_M.size());
  meanT /= static_cast<double>(vec_M.size());

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

  vpTranslationVector t(meanT);
  vpHomogeneousMatrix mean(t, R);
  return mean;
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)

/*!
  \deprecated You should rather use eye().

   Set homogeneous matrix to identity.
   \sa eye()
 */
void vpHomogeneousMatrix::setIdentity() { eye(); }

#endif //#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
