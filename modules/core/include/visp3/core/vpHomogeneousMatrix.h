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
 * Homogeneous matrix.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

/*!
  \file vpHomogeneousMatrix.h
  \brief Definition and computation on the homogeneous matrices
*/

#ifndef VPHOMOGENEOUSMATRIX_HH
#define VPHOMOGENEOUSMATRIX_HH

class vpTranslationVector;
class vpPoseVector;
class vpMatrix;
class vpRotationMatrix;
class vpPoseVector;
class vpThetaUVector;
class vpQuaternionVector;
class vpPoint;

#include <fstream>
#include <vector>

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpThetaUVector.h>
//#include <visp3/core/vpTranslationVector.h>
#include <visp3/core/vpPoseVector.h>

/*!
  \class vpHomogeneousMatrix

  \ingroup group_core_transformations

  \brief Implementation of an homogeneous matrix and operations on such kind
  of matrices.

  The class provides a data structure for the homogeneous matrices
  as well as a set of operations on these matrices.

  The vpHomogeneousMatrix class is derived from vpArray2D<double>.

  An homogeneous matrix is 4x4 matrix defines as
  \f[
  ^a{\bf M}_b = \left(\begin{array}{cc}
  ^a{\bf R}_b & ^a{\bf t}_b \\
  {\bf 0}_{1\times 3} & 1
  \end{array}
  \right)
  \f]
  that defines the position of frame <em>b</em> in frame <em>a</em>

  \f$ ^a{\bf R}_b \f$ is a rotation matrix and
  \f$ ^a{\bf t}_b \f$ is a translation vector.

*/
class VISP_EXPORT vpHomogeneousMatrix : public vpArray2D<double>
{
public:
  vpHomogeneousMatrix();
  vpHomogeneousMatrix(const vpHomogeneousMatrix &M);
  vpHomogeneousMatrix(const vpTranslationVector &t, const vpRotationMatrix &R);
  vpHomogeneousMatrix(const vpTranslationVector &t, const vpThetaUVector &tu);
  vpHomogeneousMatrix(const vpTranslationVector &t, const vpQuaternionVector &q);
  explicit vpHomogeneousMatrix(const vpPoseVector &p);
  explicit vpHomogeneousMatrix(const std::vector<float> &v);
  explicit vpHomogeneousMatrix(const std::vector<double> &v);
  vpHomogeneousMatrix(const double tx, const double ty, const double tz, const double tux, const double tuy,
                      const double tuz);
  /*!
    Destructor.
  */
  virtual ~vpHomogeneousMatrix(){};

  void buildFrom(const vpTranslationVector &t, const vpRotationMatrix &R);
  void buildFrom(const vpTranslationVector &t, const vpThetaUVector &tu);
  void buildFrom(const vpTranslationVector &t, const vpQuaternionVector &q);
  void buildFrom(const vpPoseVector &p);
  void buildFrom(const std::vector<float> &v);
  void buildFrom(const std::vector<double> &v);
  void buildFrom(const double tx, const double ty, const double tz, const double tux, const double tuy,
                 const double tuz);

  void convert(std::vector<float> &M);
  void convert(std::vector<double> &M);

  // Set to identity
  void eye();

  vpColVector getCol(const unsigned int j) const;
  vpRotationMatrix getRotationMatrix() const;
  vpThetaUVector getThetaUVector() const;
  vpTranslationVector getTranslationVector() const;

  // Invert the homogeneous matrix.
  vpHomogeneousMatrix inverse() const;
  // Invert the homogeneous matrix.
  void inverse(vpHomogeneousMatrix &Mi) const;

  // Test if the rotational part of the matrix is a rotation matrix.
  bool isAnHomogeneousMatrix() const;

  void insert(const vpRotationMatrix &R);
  void insert(const vpThetaUVector &tu);
  void insert(const vpTranslationVector &t);
  void insert(const vpQuaternionVector &t);

  void extract(vpRotationMatrix &R) const;
  void extract(vpThetaUVector &tu) const;
  void extract(vpTranslationVector &t) const;
  void extract(vpQuaternionVector &q) const;

  // Load an homogeneous matrix from a file
  void load(std::ifstream &f);
  // Save an homogeneous matrix in a file
  void save(std::ofstream &f) const;

  vpHomogeneousMatrix &operator=(const vpHomogeneousMatrix &M);
  vpHomogeneousMatrix operator*(const vpHomogeneousMatrix &M) const;
  vpHomogeneousMatrix &operator*=(const vpHomogeneousMatrix &M);

  vpColVector operator*(const vpColVector &v) const;
  vpTranslationVector operator*(const vpTranslationVector &t) const;

  // Multiply by a point
  vpPoint operator*(const vpPoint &bP) const;

  void print() const;

  /*!
    This function is not applicable to an homogeneous matrix that is always a
    4-by-4 matrix.
    \exception vpException::fatalError When this function is called.
    */
  void resize(const unsigned int nrows, const unsigned int ncols, const bool flagNullify = true)
  {
    (void)nrows;
    (void)ncols;
    (void)flagNullify;
    throw(vpException(vpException::fatalError, "Cannot resize an homogeneous matrix"));
  };

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
};

#endif

/*
 * Local variables:
 * c-basic-offset: 2
 * End:
 */
