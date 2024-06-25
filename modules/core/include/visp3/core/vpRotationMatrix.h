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
  \file vpRotationMatrix.h
  \brief Class that consider the particular case of rotation matrix
*/

#ifndef VP_ROTATION_MATRIX_H
#define VP_ROTATION_MATRIX_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpRzyxVector.h>
#include <visp3/core/vpRzyzVector.h>
#include <visp3/core/vpThetaUVector.h>
#include <visp3/core/vpTranslationVector.h>

BEGIN_VISP_NAMESPACE
/*!
  \class vpRotationMatrix

  \ingroup group_core_transformations

  \brief Implementation of a rotation matrix and operations on such kind of
  matrices.

  The vpRotationMatrix considers the particular case of
  a rotation matrix.

  The vpRotationMatrix class is derived from vpArray2D<double>.

  The code below shows how to create a rotation matrix, set the element values and access them:
  \code
  #include <visp3/core/vpRotationMatrix.h>

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRotationMatrix R;
    R[0][0] =  0; R[0][1] =  0; R[0][2] = -1;
    R[1][0] =  0; R[1][1] = -1; R[1][2] =  0;
    R[2][0] = -1; R[2][1] =  0; R[2][2] =  0;

    std::cout << "R:" << std::endl;
    for (unsigned int i = 0; i < R.getRows(); ++i) {
      for (unsigned int j = 0; j < R.getCols(); ++j) {
        std::cout << R[i][j] << " ";
      }
      std::cout << std::endl;
    }
  }
  \endcode
  Once build, this previous code produces the following output:
  \code
  R:
  0 0 -1
  0 -1 0
  -1 0 0
  \endcode
  You can also use operator<< to initialize a rotation matrix as previously:
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

  If ViSP is build with c++11 enabled, you can do the same using:
  \code
  #include <visp3/code/vpRotationMatrix.h

  #ifdef ENABLE_VISP_NAMESPACE
  using namespace VISP_NAMESPACE_NAME;
  #endif

  int main()
  {
    vpRotationMatrix R{ 0, 0, -1, 0, -1, 0, -1, 0, 0 };
    std::cout << "R:\n" << R << std::endl;
  }
  \endcode
*/
class VISP_EXPORT vpRotationMatrix : public vpArray2D<double>
{
public:
  vpRotationMatrix();
  vpRotationMatrix(const vpRotationMatrix &R);
  VP_EXPLICIT vpRotationMatrix(const vpHomogeneousMatrix &M);
  VP_EXPLICIT vpRotationMatrix(const vpThetaUVector &r);
  VP_EXPLICIT vpRotationMatrix(const vpPoseVector &p);
  VP_EXPLICIT vpRotationMatrix(const vpRzyzVector &r);
  VP_EXPLICIT vpRotationMatrix(const vpRxyzVector &r);
  VP_EXPLICIT vpRotationMatrix(const vpRzyxVector &r);
  VP_EXPLICIT vpRotationMatrix(const vpQuaternionVector &q);
  VP_EXPLICIT vpRotationMatrix(const vpMatrix &R);
  vpRotationMatrix(double tux, double tuy, double tuz);

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  VP_EXPLICIT vpRotationMatrix(const std::initializer_list<double> &list);
#endif

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  VP_DEPRECATED vpRotationMatrix buildFrom(const vpHomogeneousMatrix &M);
  VP_DEPRECATED vpRotationMatrix buildFrom(const vpThetaUVector &v);
  VP_DEPRECATED vpRotationMatrix buildFrom(const vpPoseVector &p);
  VP_DEPRECATED vpRotationMatrix buildFrom(const vpRzyzVector &v);
  VP_DEPRECATED vpRotationMatrix buildFrom(const vpRxyzVector &v);
  VP_DEPRECATED vpRotationMatrix buildFrom(const vpRzyxVector &v);
  VP_DEPRECATED vpRotationMatrix buildFrom(const vpQuaternionVector &q);
  VP_DEPRECATED vpRotationMatrix buildFrom(double tux, double tuy, double tuz);
#endif
  vpRotationMatrix &build(const vpHomogeneousMatrix &M);
  vpRotationMatrix &build(const vpThetaUVector &v);
  vpRotationMatrix &build(const vpPoseVector &p);
  vpRotationMatrix &build(const vpRzyzVector &v);
  vpRotationMatrix &build(const vpRxyzVector &v);
  vpRotationMatrix &build(const vpRzyxVector &v);
  vpRotationMatrix &build(const vpQuaternionVector &q);
  vpRotationMatrix &build(const double &tux, const double &tuy, const double &tuz);

  void eye();

  vpColVector getCol(unsigned int j) const;
  vpThetaUVector getThetaUVector();

  vpRotationMatrix inverse() const;
  void inverse(vpRotationMatrix &R) const;

  bool isARotationMatrix(double threshold = 1e-6) const;

  // copy operator from vpRotationMatrix
  vpRotationMatrix &operator=(const vpRotationMatrix &R);
  // copy operator from vpMatrix (handle with care)
  vpRotationMatrix &operator=(const vpMatrix &M);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpRotationMatrix &operator=(const std::initializer_list<double> &list);
#endif
  // operation c = A * b (A is unchanged)
  vpTranslationVector operator*(const vpTranslationVector &tv) const;
  // operation C = A * B (A is unchanged)
  vpRotationMatrix operator*(const vpRotationMatrix &R) const;
  // operation C = A * B (A is unchanged)
  vpHomogeneousMatrix operator*(const vpHomogeneousMatrix &M) const;
  vpMatrix operator*(const vpMatrix &M) const;
  // operation v2 = A * v1 (A is unchanged)
  vpColVector operator*(const vpColVector &v) const;
  vpRotationMatrix operator*(double x) const;
  vpRotationMatrix &operator*=(double x);

  vpRotationMatrix &operator<<(double val);
  vpRotationMatrix &operator,(double val);

  void orthogonalize();

  void printVector();

  /*!
    This function is not applicable to a rotation matrix that is always a
    3-by-3 matrix.
    \exception vpException::fatalError When this function is called.
    */
  void resize(unsigned int nrows, unsigned int ncols, bool flagNullify = true)
  {
    (void)nrows;
    (void)ncols;
    (void)flagNullify;
    throw(vpException(vpException::fatalError, "Cannot resize a rotation matrix"));
  }

  // transpose
  vpRotationMatrix t() const;

  static vpRotationMatrix mean(const std::vector<vpHomogeneousMatrix> &vec_M);
  static vpRotationMatrix mean(const std::vector<vpRotationMatrix> &vec_R);

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
     \deprecated You should rather use eye().
   */
  VP_DEPRECATED void setIdentity();
  //@}
#endif

protected:
  unsigned int m_index;
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS
VISP_EXPORT
#endif
VISP_NAMESPACE_ADDRESSING vpRotationMatrix operator*(const double &x, const VISP_NAMESPACE_ADDRESSING vpRotationMatrix &R);
END_VISP_NAMESPACE
#endif
