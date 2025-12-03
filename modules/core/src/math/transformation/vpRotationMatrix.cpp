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

#include <visp3/core/vpSIMDUtils.h>

// Rotation classes
#include <visp3/core/vpRotationMatrix.h>

// Exception
#include <visp3/core/vpException.h>

// Debug trace
#include <math.h>

BEGIN_VISP_NAMESPACE


void vpRotationMatrix::rotateVectors(const vpMatrix &input, vpMatrix &output, bool isTransposed) const
{
  output.resize(input.getRows(), input.getCols(), false, false);
  if (isTransposed) {
    if (input.getCols() != 3) {
      throw vpException(vpException::dimensionError, "Input matrix should have 3 columns");
    }
    double *inputData = input.data;
    double *outputData = output.data;
#if defined(VISP_HAVE_AVX)
    __m256d rows[] = {
      _mm256_setr_pd(rowPtrs[0][0], rowPtrs[0][1], rowPtrs[0][2], 0.0),
      _mm256_setr_pd(rowPtrs[1][0], rowPtrs[1][1], rowPtrs[1][2], 0.0),
      _mm256_setr_pd(rowPtrs[2][0], rowPtrs[2][1], rowPtrs[2][2], 0.0),
    };

    double result[4];

    for (unsigned int i = 0; i < input.getRows(); ++i) {
      const __m256d xyzw = _mm256_setr_pd(inputData[0], inputData[1], inputData[2], 0.0);

      const __m256d rs[] = {
        _mm256_mul_pd(rows[0], xyzw),
        _mm256_mul_pd(rows[1], xyzw),
        _mm256_mul_pd(rows[2], xyzw),
      };

      __m256d rs_half_sum1 = _mm256_hadd_pd(rs[0], rs[1]);


      _mm256_storeu_pd(result, rs_half_sum1);
      __m128d hi = _mm256_extractf128_pd(rs[2], 1);   // [A3, A2]
      __m128d lo = _mm256_castpd256_pd128(rs[2]);     // [A1, A0]

      __m128d sum2 = _mm_add_pd(lo, hi);          // [A3+A1, A2+A0]
      __m128d sum1 = _mm_hadd_pd(sum2, sum2);     // [total, total]
      double total = _mm_cvtsd_f64(sum1);


      outputData[0] = result[0] + result[2];
      outputData[1] = result[1] + result[3];
      outputData[2] = total;

      inputData += 3;
      outputData += 3;
    }

#elif VISP_HAVE_SSE2


    __m128d row01_lo = _mm_loadu_pd(rowPtrs[0]);
    __m128d row01_hi = _mm_loadu_pd(rowPtrs[1]);
    __m128d row2_xy = _mm_loadu_pd(rowPtrs[2]);

    // Preload third column as scalars
    double c0 = rowPtrs[0][2];
    double c1 = rowPtrs[1][2];
    double c2 = rowPtrs[2][2];


    for (unsigned i = 0; i < input.getRows(); ++i) {

      const __m128d xy = _mm_loadu_pd(inputData);

      const double z = inputData[2];
      // const __m128d zz = _mm_set_pd(z, z);

      __m128d mul0 = _mm_mul_pd(row01_lo, xy);
      __m128d mul1 = _mm_mul_pd(row01_hi, xy);
      __m128d r01 = _mm_hadd_pd(mul0, mul1);
      __m128d mul2 = _mm_mul_pd(row2_xy, xy);

      // Adding scalars at the end without using simd is faster
      // __m128d mulzz = _mm_mul_pd(row01_z, zz);
      // r01 = _mm_add_pd(r01, mulzz);

      // store result
      outputData[0] = _mm_cvtsd_f64(r01) + c0 * z;
      outputData[1] = _mm_cvtsd_f64(_mm_unpackhi_pd(r01, r01)) + c1 * z;
      outputData[2] = _mm_cvtsd_f64(_mm_hadd_pd(mul2, mul2)) + c2 * z;

      inputData += 3;
      outputData += 3;
    }

#else
    double *r0 = rowPtrs[0];
    double *r1 = rowPtrs[1];
    double *r2 = rowPtrs[2];

    for (unsigned int i = 0; i < input.getRows(); ++i) {
      outputData[0] = r0[0] * inputData[0] + r0[1] * inputData[1] + r0[2] * inputData[2];
      outputData[1] = r1[0] * inputData[0] + r1[1] * inputData[1] + r1[2] * inputData[2];
      outputData[2] = r2[0] * inputData[0] + r2[1] * inputData[1] + r2[2] * inputData[2];
      inputData += 3;
      outputData += 3;
    }
#endif
  }



  else { // 3xN matrix
    if (input.getRows() != 3) {
      throw vpException(vpException::dimensionError, "Expected input to have 3 rows");
    }
    double *inputX = input[0];
    double *inputY = input[1];
    double *inputZ = input[2];
    double *outputX = output[0];
    double *outputY = output[1];
    double *outputZ = output[2];


#if defined(VISP_HAVE_AVX)

    __m256d elems[9];
    for (unsigned int i = 0; i < 9; ++i) {
      elems[i] = _mm256_set1_pd(data[i]);
    }
    for (int i = 0; i <= static_cast<int>(input.getCols()) - 4; i += 4) {
      const __m256d x4 = _mm256_loadu_pd(inputX);
      const __m256d y4 = _mm256_loadu_pd(inputY);
      const __m256d z4 = _mm256_loadu_pd(inputZ);

#if defined(VISP_HAVE_FMA)
      __m256d dp1 = _mm256_mul_pd(x4, elems[0]);
      dp1 = _mm256_fmadd_pd(y4, elems[1], dp1);
      dp1 = _mm256_fmadd_pd(z4, elems[2], dp1);
      __m256d dp2 = _mm256_mul_pd(x4, elems[3]);
      dp2 = _mm256_fmadd_pd(y4, elems[4], dp2);
      dp2 = _mm256_fmadd_pd(z4, elems[5], dp2);
      __m256d dp3 = _mm256_mul_pd(x4, elems[6]);
      dp3 = _mm256_fmadd_pd(y4, elems[7], dp3);
      dp3 = _mm256_fmadd_pd(z4, elems[8], dp3);

#else
      const __m256d muls = {
        _mm256_mul_pd(x4, elems[0]),
        _mm256_mul_pd(y4, elems[1]),
        _mm256_mul_pd(z4, elems[2]),

        _mm256_mul_pd(x4, elems[3]),
        _mm256_mul_pd(y4, elems[4]),
        _mm256_mul_pd(z4, elems[5]),

        _mm256_mul_pd(x4, elems[6]),
        _mm256_mul_pd(y4, elems[7]),
        _mm256_mul_pd(z4, elems[8]),
      }

      __m256d dp1 = _mm256_add_pd(_mm256_add_pd(muls[0], muls[1]), muls[2]);
      __m256d dp2 = _mm256_add_pd(_mm256_add_pd(muls[3], muls[4]), muls[5]);
      __m256d dp3 = _mm256_add_pd(_mm256_add_pd(muls[6], muls[7]), muls[8]);


#endif

      _mm256_storeu_pd(outputX, dp1);
      _mm256_storeu_pd(outputY, dp2);
      _mm256_storeu_pd(outputZ, dp3);


      inputX += 4; inputY += 4; inputZ += 4;
      outputX += 4; outputY += 4; outputZ += 4;

    }
    double *r0 = rowPtrs[0];
    double *r1 = rowPtrs[1];
    double *r2 = rowPtrs[2];

    for (unsigned int i = (input.getCols() / 4) * 4; i < input.getCols(); ++i) {
      // std::cout << "i = " << i << std::endl;
      double X = *inputX, Y = *inputY, Z = *inputZ;
      *outputX = r0[0] * X + r0[1] * Y + r0[2] * Z;
      *outputY = r1[0] * X + r1[1] * Y + r1[2] * Z;
      *outputZ = r2[0] * X + r2[1] * Y + r2[2] * Z;
      ++inputX; ++inputY; ++inputZ;
      ++outputX; ++outputY; ++outputZ;
    }

#else
    double *r0 = rowPtrs[0];
    double *r1 = rowPtrs[1];
    double *r2 = rowPtrs[2];

    for (unsigned int i = 0; i < input.getRows(); ++i) {
      double X = *inputX, Y = *inputY, Z = *inputZ;
      *outputX = r0[0] * X + r0[1] * Y + r0[2] * Z;
      *outputY = r1[0] * X + r1[1] * Y + r1[2] * Z;
      *outputZ = r2[0] * X + r2[1] * Y + r2[2] * Z;
      ++inputX; ++inputY; ++inputZ;
      ++outputX; ++outputY; ++outputZ;
    }
#endif
  }



}


const unsigned int vpRotationMatrix::constr_val_3 = 3;
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
  const unsigned int val_3 = 3;
  if ((M.getCols() != val_3) && (M.getRows() != val_3)) {
    throw(vpException(vpException::dimensionError, "Cannot set a (3x3) rotation matrix from a (%dx%d) matrix",
                      M.getRows(), M.getCols()));
  }

  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      (*this)[i][j] = M[i][j];
    }
  }
  if (!isARotationMatrix()) {
    if (isARotationMatrix(1e-3)) {
      orthogonalize();
    }
    else {
      throw(vpException(vpException::fatalError, "Cannot set a rotation matrix "
                        "from a matrix that is not a "
                        "rotation matrix"));
    }
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
      for (unsigned int k = 0; k < val_3; ++k) {
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
  const unsigned int val_3 = 3;
  if ((M.getRows() != val_3) || (M.getCols() != val_3)) {
    throw(vpException(vpException::dimensionError, "Cannot set a (3x3) rotation matrix from a (%dx%d) matrix",
                      M.getRows(), M.getCols()));
  }
  vpMatrix p(3, 3);

  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      double s = 0;
      for (unsigned int k = 0; k < val_3; ++k) {
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
  const unsigned int val_3 = 3;

  if ((getCols() != val_3) || (getRows() != val_3)) {
    return false;
  }

  // --comment: test R^TR = Id
  vpRotationMatrix RtR = (*this).t() * (*this);
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
vpRotationMatrix::vpRotationMatrix() : vpArray2D<double>(constr_val_3, constr_val_3), m_index(0) { eye(); }

/*!
  Copy constructor that construct a 3-by-3 rotation matrix from another
  rotation matrix.
*/
vpRotationMatrix::vpRotationMatrix(const vpRotationMatrix &M) : vpArray2D<double>(constr_val_3, constr_val_3), m_index(0) { (*this) = M; }

/*!
  Construct a 3-by-3 rotation matrix from an homogeneous matrix.
*/
vpRotationMatrix::vpRotationMatrix(const vpHomogeneousMatrix &M) : vpArray2D<double>(constr_val_3, constr_val_3), m_index(0) { buildFrom(M); }

/*!
  Construct a 3-by-3 rotation matrix from \f$ \theta {\bf u}\f$ angle
  representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpThetaUVector &tu) : vpArray2D<double>(constr_val_3, constr_val_3), m_index(0) { buildFrom(tu); }

/*!
  Construct a 3-by-3 rotation matrix from a pose vector.
 */
vpRotationMatrix::vpRotationMatrix(const vpPoseVector &p) : vpArray2D<double>(constr_val_3, constr_val_3), m_index(0) { buildFrom(p); }

/*!
  Construct a 3-by-3 rotation matrix from \f$ R(z,y,z) \f$ Euler angle
  representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpRzyzVector &euler) : vpArray2D<double>(constr_val_3, constr_val_3), m_index(0)
{
  buildFrom(euler);
}

/*!
  Construct a 3-by-3 rotation matrix from \f$ R(x,y,z) \f$ Euler angle
  representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpRxyzVector &Rxyz) : vpArray2D<double>(constr_val_3, constr_val_3), m_index(0) { buildFrom(Rxyz); }

/*!
  Construct a 3-by-3 rotation matrix from \f$ R(z,y,x) \f$ Euler angle
  representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpRzyxVector &Rzyx) : vpArray2D<double>(constr_val_3, constr_val_3), m_index(0) { buildFrom(Rzyx); }

/*!
  Construct a 3-by-3 rotation matrix from a matrix that contains values corresponding to a rotation matrix.
*/
vpRotationMatrix::vpRotationMatrix(const vpMatrix &R) : vpArray2D<double>(constr_val_3, constr_val_3), m_index(0) { *this = R; }

/*!
  Construct a 3-by-3 rotation matrix from \f$ \theta {\bf u}=(\theta u_x,
  \theta u_y, \theta u_z)^T\f$ angle representation.
 */
vpRotationMatrix::vpRotationMatrix(double tux, double tuy, double tuz) : vpArray2D<double>(constr_val_3, constr_val_3), m_index(0)
{
  buildFrom(tux, tuy, tuz);
}

/*!
  Construct a 3-by-3 rotation matrix from quaternion angle representation.
 */
vpRotationMatrix::vpRotationMatrix(const vpQuaternionVector &q) : vpArray2D<double>(constr_val_3, constr_val_3), m_index(0) { buildFrom(q); }

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

/*!
  Transform a \f$ \theta {\bf u}\f$ angle representation into a rotation
  matrix.

  The rotation is computed using :
  \f[
  R = \cos{ \theta} \; {I}_{3} + (1 - \cos{ \theta}) \; u u^{T} + \sin{
  \theta} \; [u]_\times \f]
*/
vpRotationMatrix &vpRotationMatrix::buildFrom(const vpThetaUVector &v)
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
vpRotationMatrix &vpRotationMatrix::buildFrom(const vpHomogeneousMatrix &M)
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

  \sa buildFrom(const vpThetaUVector &)
*/
vpRotationMatrix &vpRotationMatrix::buildFrom(const vpPoseVector &p)
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
vpRotationMatrix &vpRotationMatrix::buildFrom(const vpRzyzVector &v)
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
vpRotationMatrix &vpRotationMatrix::buildFrom(const vpRxyzVector &v)
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
vpRotationMatrix &vpRotationMatrix::buildFrom(const vpRzyxVector &v)
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
vpRotationMatrix &vpRotationMatrix::buildFrom(const double &tux, const double &tuy, const double &tuz)
{
  vpThetaUVector tu(tux, tuy, tuz);
  buildFrom(tu);
  return *this;
}

/*!
  Construct a 3-by-3 rotation matrix from a quaternion representation.
 */
vpRotationMatrix &vpRotationMatrix::buildFrom(const vpQuaternionVector &q)
{
  double a = q.w();
  double b = q.x();
  double c = q.y();
  double d = q.z();
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  (*this)[index_0][index_0] = (((a * a) + (b * b)) - (c * c)) - (d * d);
  (*this)[index_0][index_1] = (2.0 * b * c) - (2.0 * a * d);
  (*this)[index_0][index_2] = (2.0 * a * c) + (2.0 * b * d);

  (*this)[index_1][index_0] = (2.0 * a * d) + (2.0 * b * c);
  (*this)[index_1][index_1] = (((a * a) - (b * b)) + (c * c)) - (d * d);
  (*this)[index_1][index_2] = (2.0 * c * d) - (2.0 * a * b);

  (*this)[index_2][index_0] = (2.0 * b * d) - (2.0 * a * c);
  (*this)[index_2][index_1] = (2.0 * a * b) + (2.0 * c * d);
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
  tu.buildFrom(*this);
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
    meanR += static_cast<vpMatrix>(R);
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
    meanR += static_cast<vpMatrix>(vec_R[i]);
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
