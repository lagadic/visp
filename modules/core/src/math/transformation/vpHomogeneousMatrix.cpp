/*
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2025 by Inria. All rights reserved.
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
 * Homogeneous matrix.
 */

/*!
  \file vpHomogeneousMatrix.cpp
  \brief Defines vpHomogeneousMatrix class. Class that consider
  the particular case of an homogeneous matrix.
*/

#include <visp3/core/vpException.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpSIMDUtils.h>


BEGIN_VISP_NAMESPACE
const unsigned int vpHomogeneousMatrix::constr_value_4 = 4;

void vpHomogeneousMatrix::project(const vpMatrix &input, vpMatrix &output, bool transposed) const
{
  output.resize(input.getRows(), input.getCols(), false, false);
  if (transposed) {

    if (input.getCols() != 3) {
      throw vpException(vpException::dimensionError, "Input matrix should have 3 columns");
    }
    double *inputData = input.data;
    double *outputData = output.data;
#if defined(VISP_HAVE_AVX)
    __m256d rows[] = {
      _mm256_loadu_pd(rowPtrs[0]),
      _mm256_loadu_pd(rowPtrs[1]),
      _mm256_loadu_pd(rowPtrs[2]),
      _mm256_loadu_pd(rowPtrs[3]),

    };


    double result[4];

    for (unsigned int i = 0; i < input.getRows(); ++i) {
      const __m256d xyzw = _mm256_setr_pd(inputData[0], inputData[1], inputData[2], 1.0);

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

#elif (VISP_HAVE_SSE3)
    __m128d rows[] = {
      _mm_loadu_pd(rowPtrs[0]), _mm_loadu_pd(rowPtrs[0] + 2),
      _mm_loadu_pd(rowPtrs[1]), _mm_loadu_pd(rowPtrs[1] + 2),
      _mm_loadu_pd(rowPtrs[2]), _mm_loadu_pd(rowPtrs[2] + 2)
    };
    for (unsigned int i = 0; i < input.getRows(); ++i) {

      __m128d r1 = _mm_set_pd(inputData[1], inputData[0]);
      __m128d r2 = _mm_set_pd(1.0, inputData[2]);

      for (unsigned int j = 0; j < 3; ++j) {
        __m128d m1 = _mm_mul_pd(r1, rows[j * 2]);
#if !defined(VISP_HAVE_FMA)
        __m128d m2 = _mm_mul_pd(r2, rows[j * 2 + 1]);
        __m128d add = _mm_add_pd(m1, m2);
#else
        __m128d add = _mm_fmadd_pd(r2, rows[j * 2 + 1], m1);
#endif
        __m128d sum = _mm_hadd_pd(add, add);
        outputData[j] = _mm_cvtsd_f64(sum);
      }
      inputData += 3;
      outputData += 3;
    }
#else
    double *r0 = rowPtrs[0];
    double *r1 = rowPtrs[1];
    double *r2 = rowPtrs[2];

    for (unsigned int i = 0; i < input.getRows(); ++i) {
      output[i][0] = r0[0] * inputData[0] + r0[1] * inputData[1] + r0[2] * inputData[2] + r0[3];
      output[i][1] = r1[0] * inputData[0] + r1[1] * inputData[1] + r1[2] * inputData[2] + r1[3];
      output[i][2] = r2[0] * inputData[0] + r2[1] * inputData[1] + r2[2] * inputData[2] + r2[3];
      inputData += 3;
    }
#endif
  }
  else {
    if (input.getRows() != 3) {
      throw vpException(vpException::dimensionError, "Expected input to have 3 rows");
    }
    double *inputX = input[0];
    double *inputY = input[1];
    double *inputZ = input[2];
    double *outputX = output[0];
    double *outputY = output[1];
    double *outputZ = output[2];

#if defined(VISP_HAVE_AVX2) || defined(VISP_HAVE_AVX) || defined(VISP_HAVE_SSE2)

    using namespace vpSIMD;

    Register elems[12];
    for (unsigned int i = 0; i < 12; ++i) {
      elems[i] = set1(data[i]);
    }

    for (int i = 0; i <= static_cast<int>(input.getCols()) - numLanes; i += numLanes) {
      const Register x4 = loadu(inputX);
      const Register y4 = loadu(inputY);
      const Register z4 = loadu(inputZ);

      Register dp1 = mul(x4, elems[0]);
      dp1 = vpSIMD::fma(y4, elems[1], dp1);
      dp1 = vpSIMD::fma(z4, elems[2], dp1);
      dp1 = add(elems[3], dp1);

      Register dp2 = mul(x4, elems[4]);
      dp2 = vpSIMD::fma(y4, elems[5], dp2);
      dp2 = vpSIMD::fma(z4, elems[6], dp2);
      dp2 = add(elems[7], dp2);

      Register dp3 = mul(x4, elems[8]);
      dp3 = vpSIMD::fma(y4, elems[9], dp3);
      dp3 = vpSIMD::fma(z4, elems[10], dp3);
      dp3 = add(elems[11], dp3);


      storeu(outputX, dp1);
      storeu(outputY, dp2);
      storeu(outputZ, dp3);

      inputX += numLanes; inputY += numLanes; inputZ += numLanes;
      outputX += numLanes; outputY += numLanes; outputZ += numLanes;

    }
    double *r0 = rowPtrs[0];
    double *r1 = rowPtrs[1];
    double *r2 = rowPtrs[2];

    for (unsigned int i = (input.getCols() / numLanes) * numLanes; i < input.getCols(); ++i) {
      // std::cout << "i = " << i << std::endl;
      double X = *inputX, Y = *inputY, Z = *inputZ;
      *outputX = r0[0] * X + r0[1] * Y + r0[2] * Z + r0[3];
      *outputY = r1[0] * X + r1[1] * Y + r1[2] * Z + r1[3];
      *outputZ = r2[0] * X + r2[1] * Y + r2[2] * Z + r2[3];
      ++inputX; ++inputY; ++inputZ;
      ++outputX; ++outputY; ++outputZ;
    }

#else
    double *r0 = rowPtrs[0];
    double *r1 = rowPtrs[1];
    double *r2 = rowPtrs[2];

    for (unsigned int i = 0; i < input.getRows(); ++i) {
      double X = *inputX, Y = *inputY, Z = *inputZ;
      *outputX = r0[0] * X + r0[1] * Y + r0[2] * Z + r0[3];
      *outputY = r1[0] * X + r1[1] * Y + r1[2] * Z + r1[3];
      *outputZ = r2[0] * X + r2[1] * Y + r2[2] * Z + r2[3];
      ++inputX; ++inputY; ++inputZ;
      ++outputX; ++outputY; ++outputZ;
    }
#endif

  }
}


/*!
  Construct an homogeneous matrix from a translation vector and quaternion
  rotation vector.
 */
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpTranslationVector &t, const vpQuaternionVector &q)
  : vpArray2D<double>(constr_value_4, constr_value_4)
{
  const unsigned int index_3 = 3;
  buildFrom(t, q);
  (*this)[index_3][index_3] = 1.;
}

/*!
  Default constructor that initialize an homogeneous matrix as identity.
*/
vpHomogeneousMatrix::vpHomogeneousMatrix() : vpArray2D<double>(constr_value_4, constr_value_4), m_index(0) { eye(); }

/*!
  Copy constructor that initialize an homogeneous matrix from another
  homogeneous matrix.
*/
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpHomogeneousMatrix &M) : vpArray2D<double>(constr_value_4, constr_value_4), m_index(0)
{
  *this = M;
}

/*!
  Construct an homogeneous matrix from a translation vector and \f$\theta {\bf
  u}\f$ rotation vector.
 */
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpTranslationVector &t, const vpThetaUVector &tu)
  : vpArray2D<double>(constr_value_4, constr_value_4), m_index(0)
{
  const unsigned int index_3 = 3;
  buildFrom(t, tu);
  (*this)[index_3][index_3] = 1.;
}

/*!
  Construct an homogeneous matrix from a translation vector and a rotation
  matrix.
 */
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpTranslationVector &t, const vpRotationMatrix &R)
  : vpArray2D<double>(constr_value_4, constr_value_4), m_index(0)
{
  const unsigned int index_3 = 3;
  insert(R);
  insert(t);
  (*this)[index_3][index_3] = 1.;
}

/*!
  Construct an homogeneous matrix from a pose vector.
 */
vpHomogeneousMatrix::vpHomogeneousMatrix(const vpPoseVector &p) : vpArray2D<double>(constr_value_4, constr_value_4), m_index(0)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  buildFrom(p[index_0], p[index_1], p[index_2], p[index_3], p[index_4], p[index_5]);
  (*this)[index_3][index_3] = 1.;
}

/*!
  Construct an homogeneous matrix from a vector of float.
  \param v : Vector of 12 or 16 values corresponding to the values of the
  homogeneous matrix.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
    for(unsigned int i=0; i<v.size(); ++i)
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
vpHomogeneousMatrix::vpHomogeneousMatrix(const std::vector<float> &v) : vpArray2D<double>(constr_value_4, constr_value_4), m_index(0)
{
  const unsigned int index_3 = 3;
  buildFrom(v);
  (*this)[index_3][index_3] = 1.;
}

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
/*!
  Construct an homogeneous matrix from a list of 12 or 16 double values.
  \param list : List of double.
  The following code shows how to use this constructor to initialize an homogeneous matrix:
  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpHomogeneousMatrix M {
      0, 0, 1, 0.1,
      0, 1, 0, 0.2,
      1, 0, 0, 0.3 };
    std::cout << "M:\n" << M << std::endl;
    vpHomogeneousMatrix N {
      0, 0, 1, 0.1,
      0, 1, 0, 0.2,
      1, 0, 0, 0.3,
      0, 0, 0, 1 };
    std::cout << "N:\n" << N << std::endl;
  }
  \endcode
  It produces the following output:
  \code
  M:
  0  0  1  0.1
  0  1  0  0.2
  1  0  0  0.3
  0  0  0  1
  N:
  0  0  1  0.1
  0  1  0  0.2
  1  0  0  0.3
  0  0  0  1
  \endcode
 */
vpHomogeneousMatrix::vpHomogeneousMatrix(const std::initializer_list<double> &list)
  : vpArray2D<double>(4, 4), m_index(0)
{
  if (list.size() == 12) {
    std::copy(list.begin(), list.end(), data);
    data[12] = 0.;
    data[13] = 0.;
    data[14] = 0.;
    data[15] = 1.;
  }
  else if (list.size() == 16) {
    std::copy(list.begin(), list.end(), data);
    for (size_t i = 12; i < 15; ++i) {
      if (std::fabs(data[i]) > std::numeric_limits<double>::epsilon()) {
        throw(vpException(vpException::fatalError,
                          "Cannot initialize homogeneous matrix. "
                          "List element %d (%f) should be 0.",
                          i, data[i]));
      }
    }
    if (std::fabs(data[15] - 1.) > std::numeric_limits<double>::epsilon()) {
      throw(vpException(vpException::fatalError,
                        "Cannot initialize homogeneous matrix. "
                        "List element 15 (%f) should be 1.",
                        data[15]));
    }
  }
  else {
    throw(vpException(vpException::fatalError,
                      "Cannot initialize homogeneous matrix from a list (%d elements) that has not 12 or 16 elements",
                      list.size()));
  }

  if (!isAnHomogeneousMatrix()) {
    if (isAnHomogeneousMatrix(1e-3)) {
      // re-orthogonalize rotation matrix since the input is close to a valid rotation matrix
      orthogonalizeRotation();
    }
    else {
      throw(vpException(
        vpException::fatalError,
        "Homogeneous matrix initialization fails since its elements are not valid (rotation part or last row)"));
    }
  }
}
#endif

/*!
  Construct an homogeneous matrix from a vector of double.
  \param v : Vector of 12 or 16 values corresponding to the values of the
  homogeneous matrix.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
    for(unsigned int i=0; i<v.size(); ++i)
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
vpHomogeneousMatrix::vpHomogeneousMatrix(const std::vector<double> &v) : vpArray2D<double>(constr_value_4, constr_value_4), m_index(0)
{
  const unsigned int index_3 = 3;
  buildFrom(v);
  (*this)[index_3][index_3] = 1.;
}

/*!
  Construct an homogeneous matrix from a translation vector \f${\bf t}=(t_x,
  t_y, t_z)^T\f$ and a \f$\theta {\bf u}=(\theta u_x, \theta u_y, \theta
  u_z)^T\f$ rotation vector.
 */
vpHomogeneousMatrix::vpHomogeneousMatrix(double tx, double ty, double tz, double tux, double tuy, double tuz)
  : vpArray2D<double>(constr_value_4, constr_value_4), m_index(0)
{
  const unsigned int index_3 = 3;
  buildFrom(tx, ty, tz, tux, tuy, tuz);
  (*this)[index_3][index_3] = 1.;
}

/*!
  Build an homogeneous matrix from a translation vector
  and a \f$\theta {\bf u}\f$ rotation vector.
 */
vpHomogeneousMatrix &vpHomogeneousMatrix::buildFrom(const vpTranslationVector &t, const vpThetaUVector &tu)
{
  insert(tu);
  insert(t);
  return *this;
}

/*!
  Build an homogeneous matrix from a translation vector
  and a rotation matrix.
 */
vpHomogeneousMatrix &vpHomogeneousMatrix::buildFrom(const vpTranslationVector &t, const vpRotationMatrix &R)
{
  insert(R);
  insert(t);
  return *this;
}

/*!
  Build an homogeneous matrix from a pose vector.
 */
vpHomogeneousMatrix &vpHomogeneousMatrix::buildFrom(const vpPoseVector &p)
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  vpTranslationVector tv(p[index_0], p[index_1], p[index_2]);
  vpThetaUVector tu(p[index_3], p[index_4], p[index_5]);

  insert(tu);
  insert(tv);
  return *this;
}

/*!
  Build an homogeneous matrix from a translation vector
  and a quaternion rotation vector.
 */
vpHomogeneousMatrix &vpHomogeneousMatrix::buildFrom(const vpTranslationVector &t, const vpQuaternionVector &q)
{
  insert(t);
  insert(q);
  return *this;
}

/*!
  Build an homogeneous matrix from a translation vector \f${\bf t}=(t_x, t_y,
  t_z)^T\f$ and a \f$\theta {\bf u}=(\theta u_x, \theta u_y, \theta u_z)^T\f$
  rotation vector.
 */
vpHomogeneousMatrix &vpHomogeneousMatrix::buildFrom(const double &tx, const double &ty, const double &tz, const double &tux, const double &tuy, const double &tuz)
{
  vpRotationMatrix R(tux, tuy, tuz);
  vpTranslationVector t(tx, ty, tz);

  insert(R);
  insert(t);
  return *this;
}

/*!
  Build an homogeneous matrix from a vector of float.
  \param v : Vector of 12 or 16 values corresponding to the values of the
  homogeneous matrix.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
    for(unsigned int i=0; i<v.size(); ++i)
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
vpHomogeneousMatrix &vpHomogeneousMatrix::buildFrom(const std::vector<float> &v)
{
  const std::size_t val_12 = 12;
  const std::size_t val_16 = 16;
  const unsigned int val_12ui = 12, val_16ui = 16;
  if ((v.size() != val_12) && (v.size() != val_16)) {
    throw(vpException(vpException::dimensionError, "Cannot convert std::vector<float> to vpHomogeneousMatrix"));
  }

  for (unsigned int i = 0; i < val_12ui; ++i) {
    this->data[i] = static_cast<double>(v[i]);
  }

  if (v.size() == val_12) {
    data[12] = 0.;
    data[13] = 0.;
    data[14] = 0.;
    data[15] = 1.;
  }
  else {
    for (unsigned int i = val_12ui; i < val_16ui; ++i) {
      this->data[i] = static_cast<double>(v[i]);
    }
  }

  if (!isAnHomogeneousMatrix()) {
    if (isAnHomogeneousMatrix(1e-3)) {
      // re-orthogonalize rotation matrix since the input is close to a valid rotation matrix
      orthogonalizeRotation();
    }
    else {
      throw(vpException(
        vpException::fatalError,
        "Homogeneous matrix initialization failed since its elements are not valid (rotation part or last row)"));
    }
  }
  return *this;
}

/*!
  Build an homogeneous matrix from a vector of double.
  \param v : Vector of 12 or 16 values corresponding to the values of the
  homogeneous matrix.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
    for(unsigned int i=0; i<v.size(); ++i)
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
vpHomogeneousMatrix &vpHomogeneousMatrix::buildFrom(const std::vector<double> &v)
{
  const std::size_t val_12 = 12;
  const std::size_t val_16 = 16;
  const unsigned int val_12ui = 12, val_16ui = 16;


  if ((v.size() != val_12) && (v.size() != val_16)) {
    throw(vpException(vpException::dimensionError, "Cannot convert std::vector<double> to vpHomogeneousMatrix"));
  }

  for (unsigned int i = 0; i < val_12ui; ++i) {
    this->data[i] = static_cast<double>(v[i]);
  }

  if (v.size() == val_12) {
    data[12] = 0.;
    data[13] = 0.;
    data[14] = 0.;
    data[15] = 1.;
  }
  else {
    for (unsigned int i = val_12ui; i < val_16ui; ++i) {
      this->data[i] = static_cast<double>(v[i]);
    }
  }

  if (!isAnHomogeneousMatrix()) {
    if (isAnHomogeneousMatrix(1e-3)) {
      // re-orthogonalize rotation matrix since the input is close to a valid rotation matrix
      orthogonalizeRotation();
    }
    else {
      throw(vpException(
        vpException::fatalError,
        "Homogeneous matrix initialization fails since its elements are not valid (rotation part or last row)"));
    }
  }

  return *this;
}

/*!
  Copy operator that allows to set an homogeneous matrix from an other one.

  \param M : Matrix to copy.
*/
vpHomogeneousMatrix &vpHomogeneousMatrix::operator=(const vpHomogeneousMatrix &M)
{
  const int val_4 = 4;
  for (int i = 0; i < val_4; ++i) {
    for (int j = 0; j < val_4; ++j) {
      rowPtrs[i][j] = M.rowPtrs[i][j];
    }
  }
  return *this;
}

/*!
  Operator that allow to multiply an homogeneous matrix by an other one.

  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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

  T = (R1 * T2) + T1;

  p.insert(T);
  p.insert(R);

  return p;
}

/*!
  Operator that allow to multiply an homogeneous matrix by an other one.

  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
  const unsigned int val_4 = 4;
  if (v.getRows() != val_4) {
    throw(vpException(vpException::dimensionError,
                      "Cannot multiply a (4x4) homogeneous matrix by a "
                      "(%dx1) column vector",
                      v.getRows()));
  }
  vpColVector p(rowNum, 0.0);

// #if USE_SSE
//   __m128d rows[] = {
//     _mm_loadu_pd(rowPtrs[0]), _mm_loadu_pd(rowPtrs[0] + 2),
//     _mm_loadu_pd(rowPtrs[1]), _mm_loadu_pd(rowPtrs[1] + 2),
//     _mm_loadu_pd(rowPtrs[2]), _mm_loadu_pd(rowPtrs[2] + 2),
//     _mm_loadu_pd(rowPtrs[3]), _mm_loadu_pd(rowPtrs[3] + 2)

//   };

//   __m128d r1 = _mm_loadu_pd(v.data);
//   __m128d r2 = _mm_loadu_pd(v.data + 2);

//   for (unsigned int j = 0; j < 4; ++j) {
//     __m128d m1 = _mm_mul_pd(r1, rows[j * 2]);
// #if !defined(__FMA__)
//     __m128d m2 = _mm_mul_pd(r2, rows[j * 2 + 1]);
//     __m128d add = _mm_add_pd(m1, m2);
// #else
//     __m128d add = _mm_fmadd_pd(r2, rows[j * 2 + 1], m1);
// #endif
//     __m128d sum = _mm_hadd_pd(add, add);
//     p[j] = _mm_cvtsd_f64(sum);
//   }


// #else

  for (unsigned int j = 0; j < val_4; ++j) {
    for (unsigned int i = 0; i < val_4; ++i) {
      p[i] += rowPtrs[i][j] * v[j];
    }
  }
// #endif
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

  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;

  v[index_0] = bP.get_X();
  v[index_1] = bP.get_Y();
  v[index_2] = bP.get_Z();
  v[index_3] = bP.get_W();

  v1[index_0] = ((*this)[index_0][index_0] * v[index_0]) + ((*this)[index_0][index_1] * v[index_1]) + ((*this)[index_0][index_2] * v[index_2]) + ((*this)[index_0][index_3] * v[index_3]);
  v1[index_1] = ((*this)[index_1][index_0] * v[index_0]) + ((*this)[index_1][index_1] * v[index_1]) + ((*this)[index_1][index_2] * v[index_2]) + ((*this)[index_1][index_3] * v[index_3]);
  v1[index_2] = ((*this)[index_2][index_0] * v[index_0]) + ((*this)[index_2][index_1] * v[index_1]) + ((*this)[index_2][index_2] * v[index_2]) + ((*this)[index_2][index_3] * v[index_3]);
  v1[index_3] = ((*this)[index_3][index_0] * v[index_0]) + ((*this)[index_3][index_1] * v[index_1]) + ((*this)[index_3][index_2] * v[index_2]) + ((*this)[index_3][index_3] * v[index_3]);

  v1 /= v1[index_3];

  //  --comment: v1 equals M multiplied by v
  aP.set_X(v1[index_0]);
  aP.set_Y(v1[index_1]);
  aP.set_Z(v1[index_2]);
  aP.set_W(v1[index_3]);

  aP.set_oX(v1[index_0]);
  aP.set_oY(v1[index_1]);
  aP.set_oZ(v1[index_2]);
  aP.set_oW(v1[index_3]);

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
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  t_out[index_0] = (((*this)[index_0][0] * t[0]) + ((*this)[index_0][1] * t[1]) + ((*this)[index_0][index_2] * t[index_2])) + (*this)[index_0][index_3];
  t_out[index_1] = (((*this)[index_1][0] * t[0]) + ((*this)[index_1][1] * t[1]) + ((*this)[index_1][index_2] * t[index_2])) + (*this)[index_1][index_3];
  t_out[index_2] = (((*this)[index_2][0] * t[0]) + ((*this)[index_2][1] * t[1]) + ((*this)[index_2][index_2] * t[index_2])) + (*this)[index_2][index_3];

  return t_out;
}

/*!
  Operator that allows to multiply a rotation matrix by a rotation matrix.

  \param[in] R : Rotation matrix.

  \return The product between the homogeneous matrix and the rotation matrix `R`.

  The following snippet shows how to use this method:
  \code
  vpHomogeneousMatrix c1_M_c2;
  vpRotationMatrix c2_R_c3;
  vpHomogeneousMatrix c1_M_c3 = c1_M_c2 * c2_R_c3;
  \endcode
*/
vpHomogeneousMatrix vpHomogeneousMatrix::operator*(const vpRotationMatrix &R) const
{
  return (vpHomogeneousMatrix((*this).getTranslationVector(), (*this).getRotationMatrix() * R));
}

/*!
  Set homogeneous matrix first element.

  \param val : Value of the matrix first element.
  \return An updated matrix.

  The following example shows how to initialize a rotation matrix using this operator.
  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpHomogeneousMatrix M;
    M << 0, 0, 1, 0.1,
        0, 1, 0, 0.2,
        1, 0, 0, 0.3;
    std::cout << "M:\n" << M << std::endl;

    vpHomogeneousMatrix N;
    N << 0, 0, 1, 0.1,
        0, 1, 0, 0.2,
        1, 0, 0, 0.3,
        0, 0, 0, 1;
    std::cout << "N:\n" << N << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  M:
  0  0  1  0.1
  0  1  0  0.2
  1  0  0  0.3
  0  0  0  1
  N:
  0  0  1  0.1
  0  1  0  0.2
  1  0  0  0.3
  0  0  0  1
  \endcode

  \sa operator,()
 */
vpHomogeneousMatrix &vpHomogeneousMatrix::operator<<(double val)
{
  m_index = 0;
  data[m_index] = val;
  return *this;
}

/*!
  Set the second and next element of the homogenous matrix.

  \param val : Value of the matrix second or next element.
  \return An updated matrix.

  The following example shows how to initialize an homogeneous matrix using this operator.
  \code
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpHomogeneousMatrix M;
    M << 0, 0, 1, 0.1,
        0, 1, 0, 0.2,
        1, 0, 0, 0.3;
    std::cout << "M:\n" << M << std::endl;

    vpHomogeneousMatrix N;
    N << 0, 0, 1, 0.1,
        0, 1, 0, 0.2,
        1, 0, 0, 0.3,
        0, 0, 0, 1;
    std::cout << "N:\n" << N << std::endl;
  }
  \endcode
  It produces the following printings:
  \code
  M:
  0  0  1  0.1
  0  1  0  0.2
  1  0  0  0.3
  0  0  0  1
  N:
  0  0  1  0.1
  0  1  0  0.2
  1  0  0  0.3
  0  0  0  1
  \endcode

  \sa operator<<()
 */
vpHomogeneousMatrix &vpHomogeneousMatrix::operator,(double val)
{
  ++m_index;
  if (m_index >= size()) {
    throw(vpException(vpException::dimensionError,
                      "Cannot set homogenous matrix out of bounds. It has only %d elements while you try to initialize "
                      "with %d elements",
                      size(), m_index + 1));
  }
  data[m_index] = val;
  return *this;
}

/*********************************************************************/

/*!
  Test if the 3x3 rotational part of the homogeneous matrix is a valid
  rotation matrix and the last row is equal to [0, 0, 0, 1].

  \return true if the matrix is a homogeneous matrix, false otherwise.
*/
bool vpHomogeneousMatrix::isAnHomogeneousMatrix(double threshold) const
{
  vpRotationMatrix R;
  extract(R);

  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const double epsilon = std::numeric_limits<double>::epsilon();
  bool isLastRowOK = vpMath::nul((*this)[index_3][index_0], epsilon) && vpMath::nul((*this)[index_3][index_1], epsilon) &&
    vpMath::nul((*this)[index_3][index_2], epsilon);
  return R.isARotationMatrix(threshold) && isLastRowOK && vpMath::equal((*this)[index_3][index_3], 1.0, epsilon);
}

/*!
 * Check if the homogeneous transformation matrix doesn't have a value NaN.
 * \return true when no NaN found, false otherwise.
 */
bool vpHomogeneousMatrix::isValid() const
{
  unsigned int l_size = size();
  for (unsigned int i = 0; i < l_size; ++i) {
    if (vpMath::isNaN(data[i])) {
      return false;
    }
  }
  return true;
}

/*!
  Extract the rotational matrix from the homogeneous matrix.
  \param R : rotational component as a rotation matrix.
*/
void vpHomogeneousMatrix::extract(vpRotationMatrix &R) const
{
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      R[i][j] = (*this)[i][j];
    }
  }
}

/*!
  Extract the translation vector from the homogeneous matrix.
*/
void vpHomogeneousMatrix::extract(vpTranslationVector &t) const
{
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  t[index_0] = (*this)[index_0][index_3];
  t[index_1] = (*this)[index_1][index_3];
  t[index_2] = (*this)[index_2][index_3];
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
  const unsigned int val_3 = 3;
  for (unsigned int i = 0; i < val_3; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      (*this)[i][j] = R[i][j];
    }
  }
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
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  (*this)[index_0][index_3] = t[index_0];
  (*this)[index_1][index_3] = t[index_1];
  (*this)[index_2][index_3] = t[index_2];
}

/*!

  Insert the rotational component of the homogeneous matrix from a
  quaternion rotation vector.

*/
void vpHomogeneousMatrix::insert(const vpQuaternionVector &q) { insert(vpRotationMatrix(q)); }

/*!
  Invert the homogeneous matrix.

  \return The inverse of the homogenous matrix.
  \f[\left[\begin{array}{cc}
  {\bf R} & {\bf t} \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right]^{-1} = \left[\begin{array}{cc}
  {\bf R}^T & -{\bf R}^T {\bf t} \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right]\f]
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
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  (*this)[index_0][index_0] = 1;
  (*this)[index_1][index_1] = 1;
  (*this)[index_2][index_2] = 1;
  (*this)[index_3][index_3] = 1;

  (*this)[index_0][index_1] = 0;
  (*this)[index_0][index_2] = 0;
  (*this)[index_0][index_3] = 0;
  (*this)[index_1][index_0] = 0;
  (*this)[index_1][index_2] = 0;
  (*this)[index_1][index_3] = 0;
  (*this)[index_2][index_0] = 0;
  (*this)[index_2][index_1] = 0;
  (*this)[index_2][index_3] = 0;
  (*this)[index_3][index_0] = 0;
  (*this)[index_3][index_1] = 0;
  (*this)[index_3][index_2] = 0;
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

void vpHomogeneousMatrix::save(std::ofstream &f) const
{
  if (!f.fail()) {
    f << *this;
  }
  else {
    throw(vpException(vpException::ioError, "Cannot save homogeneous matrix: ostream not open"));
  }
}

void vpHomogeneousMatrix::save(const std::string &filename) const
{
  std::ofstream f;
  f.open(filename.c_str());
  save(f);
  f.close();
}

void vpHomogeneousMatrix::load(std::ifstream &f)
{
  if (!f.fail()) {
    const unsigned int val_4 = 4;
    for (unsigned int i = 0; i < val_4; ++i) {
      for (unsigned int j = 0; j < val_4; ++j) {
        f >> (*this)[i][j];
      }
    }
  }
  else {
    throw(vpException(vpException::ioError, "Cannot load homogeneous matrix: ifstream not open"));
  }
}

void vpHomogeneousMatrix::load(const std::string &filename)
{
  std::ifstream f;
  f.open(filename.c_str());
  load(f);
  f.close();
}

/*!
  Perform orthogonalization of the rotation part of the homogeneous transformation.
 */
void vpHomogeneousMatrix::orthogonalizeRotation()
{
  vpRotationMatrix R(*this);
  R.orthogonalize();
  insert(R);
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
  const std::size_t val_12 = 12;
  const unsigned int val_12ui = 12;
  M.resize(val_12);
  for (unsigned int i = 0; i < val_12ui; ++i) {
    M[i] = static_cast<float>(this->data[i]);
  }
}

/*!
  Converts an homogeneous matrix to a vector of 12 doubles.
  \param M : Converted matrix.
 */
void vpHomogeneousMatrix::convert(std::vector<double> &M)
{
  const std::size_t val_12 = 12;
  const unsigned int val_12ui = 12;
  M.resize(val_12);
  for (unsigned int i = 0; i < val_12ui; ++i) {
    M[i] = this->data[i];
  }
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
  \param j : Index of the column to extract. If j=0, the first column is extracted.
  \return The extracted column vector.

  The following example shows how to use this function:
  \code
  #include <visp3/core/vpColVector.h>
  #include <visp3/core/vpHomogeneousMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

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
vpColVector vpHomogeneousMatrix::getCol(unsigned int j) const
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
 * Compute the transformation between two point clouds.
 * \param[in] p : First point cloud.
 * \param[in] q : Second point cloud.
 * \return The homogeneous transformation \f${^p}{\bf M}_q\f$.
 */
vpHomogeneousMatrix vpHomogeneousMatrix::compute3d3dTransformation(const std::vector<vpPoint> &p, const std::vector<vpPoint> &q)
{
  const double N = static_cast<double>(p.size());

  vpColVector p_bar(3, 0.0);
  vpColVector q_bar(3, 0.0);
  size_t p_size = p.size();
  const unsigned int val_3 = 3;
  for (size_t i = 0; i < p_size; ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      p_bar[j] += p.at(i).oP[j];
      q_bar[j] += q.at(i).oP[j];
    }
  }

  for (unsigned int j = 0; j < val_3; ++j) {
    p_bar[j] /= N;
    q_bar[j] /= N;
  }

  vpMatrix pc(static_cast<unsigned int>(p.size()), 3);
  vpMatrix qc(static_cast<unsigned int>(q.size()), 3);

  for (unsigned int i = 0; i < static_cast<unsigned int>(p_size); ++i) {
    for (unsigned int j = 0; j < val_3; ++j) {
      pc[i][j] = p.at(i).oP[j] - p_bar[j];
      qc[i][j] = q.at(i).oP[j] - q_bar[j];
    }
  }

  const vpMatrix pct_qc = pc.t() * qc;
  vpMatrix U = pct_qc, V;
  vpColVector W;
  U.svd(W, V);

  vpMatrix Vt = V.t();
  vpMatrix R = U * Vt;
  if (R.det() < 0) {
    const unsigned int index_0 = 0;
    const unsigned int index_1 = 1;
    const unsigned int index_2 = 2;
    Vt[index_2][index_0] *= -1.;
    Vt[index_2][index_1] *= -1.;
    Vt[index_2][index_2] *= -1.;

    R = U * Vt;
  }

  const vpColVector t = p_bar - (R * q_bar);

  return vpHomogeneousMatrix(vpTranslationVector(t), vpRotationMatrix(R));
}

/*!
  Compute the Euclidean mean of the homogeneous matrices.
  The Euclidean mean of the rotation matrices is computed following Moakher's method (SIAM 2002).

  \param[in] vec_M : Set of homogeneous matrices.
  \return The Euclidean mean of the homogeneous matrices.

  \sa vpTranslationVector::mean(), vpRotationMatrix::mean()
 */
vpHomogeneousMatrix vpHomogeneousMatrix::mean(const std::vector<vpHomogeneousMatrix> &vec_M)
{
  vpMatrix meanR(3, 3);
  vpColVector meanT(3);
  vpRotationMatrix R;
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  size_t vec_m_size = vec_M.size();
  for (size_t i = 0; i < vec_m_size; ++i) {
    R = vec_M[i].getRotationMatrix();
    meanR += static_cast<vpMatrix>(R);
    meanT += static_cast<vpColVector>(vec_M[i].getTranslationVector());
  }
  meanR /= static_cast<double>(vec_M.size());
  meanT /= static_cast<double>(vec_M.size());

  // Euclidean mean of the rotation matrix following Moakher's method (SIAM 2002)
  vpMatrix M, U, V;
  vpColVector sv;
  meanR.pseudoInverse(M, sv, 1e-6, U, V);
  double det = sv[index_0] * sv[index_1] * sv[index_2];
  if (det > 0) {
    meanR = U * V.t();
  }
  else {
    vpMatrix D(3, 3);
    D = 0.0;
    D[index_0][index_0] = 1.0;
    D[index_1][index_1] = 1.0;
    D[index_2][index_2] = -1.;
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

#ifdef VISP_HAVE_NLOHMANN_JSON
VP_ATTRIBUTE_NO_DESTROY const std::string vpHomogeneousMatrix::jsonTypeName = "vpHomogeneousMatrix";
#include <visp3/core/vpJsonParsing.h>
void vpHomogeneousMatrix::convert_to_json(nlohmann::json &j) const
{
  const vpArray2D<double> *asArray = (vpArray2D<double>*) this;
  to_json(j, *asArray);
  j["type"] = vpHomogeneousMatrix::jsonTypeName;
}

void vpHomogeneousMatrix::parse_json(const nlohmann::json &j)
{
#ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
#endif
  vpArray2D<double> *asArray = (vpArray2D<double>*) this;
  if (j.is_object() && j.contains("type")) { // Specific conversions
    const bool converted = convertFromTypeAndBuildFrom<vpHomogeneousMatrix, vpPoseVector>(j, *this);
    if (!converted) {
      from_json(j, *asArray);
    }
  }
  else { // Generic 2D array conversion
    from_json(j, *asArray);
  }

  if ((getCols() != 4) && (getRows() != 4)) {
    throw vpException(vpException::badValue, "From JSON, tried to read something that is not a 4x4 matrix");
  }
  if (!isAnHomogeneousMatrix()) {
    throw vpException(vpException::badValue, "From JSON read a non homogeneous matrix into a vpHomogeneousMatrix");
  }
}
#endif
END_VISP_NAMESPACE
