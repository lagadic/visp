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
 * Rotation matrix.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpROTATIONMATRIX_H
#define vpROTATIONMATRIX_H

/*!
  \file vpRotationMatrix.h
  \brief Class that consider the particular case of rotation matrix
*/

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpPoseVector.h>
#include <visp3/core/vpQuaternionVector.h>
#include <visp3/core/vpRxyzVector.h>
#include <visp3/core/vpRzyxVector.h>
#include <visp3/core/vpRzyzVector.h>
#include <visp3/core/vpThetaUVector.h>
#include <visp3/core/vpTranslationVector.h>

/*!
  \class vpRotationMatrix

  \ingroup group_core_transformations

  \brief Implementation of a rotation matrix and operations on such kind of
  matrices.

  The vpRotationMatrix considers the particular case of
  a rotation matrix.

  The vpRotationMatrix class is derived from vpArray2D<double>.

*/
class VISP_EXPORT vpRotationMatrix : public vpArray2D<double>
{
public:
  vpRotationMatrix();
  vpRotationMatrix(const vpRotationMatrix &R);
  explicit vpRotationMatrix(const vpHomogeneousMatrix &M);
  explicit vpRotationMatrix(const vpThetaUVector &r);
  explicit vpRotationMatrix(const vpPoseVector &p);
  explicit vpRotationMatrix(const vpRzyzVector &r);
  explicit vpRotationMatrix(const vpRxyzVector &r);
  explicit vpRotationMatrix(const vpRzyxVector &r);
  explicit vpRotationMatrix(const vpQuaternionVector &q);
  vpRotationMatrix(const double tux, const double tuy, const double tuz);
  /*!
    Destructor.
  */
  virtual ~vpRotationMatrix(){};

  vpRotationMatrix buildFrom(const vpHomogeneousMatrix &M);
  vpRotationMatrix buildFrom(const vpThetaUVector &v);
  vpRotationMatrix buildFrom(const vpPoseVector &p);
  vpRotationMatrix buildFrom(const vpRzyzVector &v);
  vpRotationMatrix buildFrom(const vpRxyzVector &v);
  vpRotationMatrix buildFrom(const vpRzyxVector &v);
  vpRotationMatrix buildFrom(const vpQuaternionVector &q);
  vpRotationMatrix buildFrom(const double tux, const double tuy, const double tuz);

  void eye();

  vpColVector getCol(const unsigned int j) const;
  vpThetaUVector getThetaUVector();

  vpRotationMatrix inverse() const;
  void inverse(vpRotationMatrix &R) const;

  bool isARotationMatrix() const;

  // copy operator from vpRotationMatrix
  vpRotationMatrix &operator=(const vpRotationMatrix &R);
  // copy operator from vpMatrix (handle with care)
  vpRotationMatrix &operator=(const vpMatrix &M);
  // operation c = A * b (A is unchanged)
  vpTranslationVector operator*(const vpTranslationVector &tv) const;
  // operation C = A * B (A is unchanged)
  vpRotationMatrix operator*(const vpRotationMatrix &R) const;
  // operation C = A * B (A is unchanged)
  vpMatrix operator*(const vpMatrix &M) const;
  // operation v2 = A * v1 (A is unchanged)
  vpColVector operator*(const vpColVector &v) const;
  vpRotationMatrix operator*(const double x) const;
  vpRotationMatrix &operator*=(const double x);

  void printVector();

  /*!
    This function is not applicable to a rotation matrix that is always a
    3-by-3 matrix.
    \exception vpException::fatalError When this function is called.
    */
  void resize(const unsigned int nrows, const unsigned int ncols, const bool flagNullify = true)
  {
    (void)nrows;
    (void)ncols;
    (void)flagNullify;
    throw(vpException(vpException::fatalError, "Cannot resize a rotation matrix"));
  };

  // transpose
  vpRotationMatrix t() const;

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  /*!
     \deprecated Provided only for compat with previous releases.
     This function does nothing.
   */
  vp_deprecated void init(){};
  /*!
     \deprecated You should rather use eye().
   */
  vp_deprecated void setIdentity();
//@}
#endif

private:
  static const double threshold;
};

#ifndef DOXYGEN_SHOULD_SKIP_THIS
VISP_EXPORT
#endif
vpRotationMatrix operator*(const double &x, const vpRotationMatrix &R);

#endif
