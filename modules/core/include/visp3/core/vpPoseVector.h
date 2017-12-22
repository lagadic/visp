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
 * Pose object. A pose is a size 6 vector [t, tu]^T where tu is
 * a rotation vector (theta u representation) and t is a translation vector.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpPOSEVECTOR_H
#define vpPOSEVECTOR_H

/*!
  \file vpPoseVector.h

  \brief Pose representation. A pose is a 6 dimension vector [t,tu]^T
    where tu is a rotation vector (theta u representation) and t is a
    translation vector.
*/

class vpRotationMatrix;
class vpHomogeneousMatrix;
class vpTranslationVector;
class vpThetaUVector;
class vpRowVector;

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpRotationMatrix.h>

/*!
  \class vpPoseVector

  \ingroup group_core_transformations

  \brief Implementation of a pose vector and operations on poses.

  The vpPose class implements a complete representation of every rigid motion
  in the euclidian space.

  The vpPose class is derived from vpArray2D<double>.

  The pose is composed of a translation and a rotation
  minimaly represented by a 6 dimension pose vector as: \f[ ^{a}{\bf
  r}_b = [^{a}{\bf t}_{b},\theta {\bf u}]^\top \in R^6\f]

  where \f$ ^{a}{\bf r}_b \f$ is the pose from frame \f$ a \f$ to
  frame \f$ b \f$, with \f$ ^{a}{\bf t}_{b} \f$ being the translation
  vector between these frames along the x,y,z
  axis and \f$\theta \bf u \f$, the axis-angle representation of the
  rotation \f$^{a}\bf{R}_{b}\f$ between these frames.

  Translations are expressed in meters, while the angles in the \f$\theta {\bf
  u}\f$ axis-angle representation are expressed in radians.

  To know more about the \f$\theta \bf u\f$ rotation representation,
  see vpThetaUVector documentation.

*/
class VISP_EXPORT vpPoseVector : public vpArray2D<double>
{
public:
  // constructor
  vpPoseVector();
  // constructor from 3 angles (in radian)
  vpPoseVector(const double tx, const double ty, const double tz, const double tux, const double tuy, const double tuz);
  // constructor convert an homogeneous matrix in a pose
  explicit vpPoseVector(const vpHomogeneousMatrix &M);
  // constructor  convert a translation and a "thetau" vector into a pose
  vpPoseVector(const vpTranslationVector &tv, const vpThetaUVector &tu);
  // constructor  convert a translation and a rotation matrix into a pose
  vpPoseVector(const vpTranslationVector &tv, const vpRotationMatrix &R);
  /*!
    Destructor.
  */
  virtual ~vpPoseVector(){};

  vpPoseVector buildFrom(const double tx, const double ty, const double tz, const double tux, const double tuy,
                         const double tuz);
  // convert an homogeneous matrix in a pose
  vpPoseVector buildFrom(const vpHomogeneousMatrix &M);
  //  convert a translation and a "thetau" vector into a pose
  vpPoseVector buildFrom(const vpTranslationVector &tv, const vpThetaUVector &tu);
  //  convert a translation and a rotation matrix into a pose
  vpPoseVector buildFrom(const vpTranslationVector &tv, const vpRotationMatrix &R);

  void extract(vpRotationMatrix &R) const;
  void extract(vpThetaUVector &tu) const;
  void extract(vpTranslationVector &tv) const;
  void extract(vpQuaternionVector &q) const;

  vpRotationMatrix getRotationMatrix() const;
  vpThetaUVector getThetaUVector() const;
  vpTranslationVector getTranslationVector() const;

  // Load an homogeneous matrix from a file
  void load(std::ifstream &f);

  /*!
    Set the value of an element of the pose vector: r[i] = x.

    \param i : Pose vector element index

    \code
    // Create a pose vector with translation and rotation set to zero
    vpPoseVector r;

    // Initialize the pose vector
    r[0] = 1;
    r[1] = 2;
    r[2] = 3;
    r[3] = M_PI;
    r[4] = -M_PI;
    r[5] = 0;
    \endcode

    This code produces the same effect:
    \code
    vpPoseVector r(1, 2, 3, M_PI, -M_PI, 0);
    \endcode

  */
  inline double &operator[](unsigned int i) { return *(data + i); }
  /*!
    Get the value of an element of the pose vector: x = r[i].

    \param i : Pose vector element index

    \code
    vpPoseVector r(1, 2, 3, M_PI, -M_PI, 0);

    double tx,ty,tz; // Translation
    double tux, tuy,tuz; // Theta u rotation
    tx  = r[0];
    ty  = r[1];
    tz  = r[2];
    tux = r[3];
    tuy = r[4];
    tuz = r[5];
    \endcode
  */
  inline const double &operator[](unsigned int i) const { return *(data + i); }

  // Print  a vector [T thetaU] thetaU in degree
  void print() const;
  int print(std::ostream &s, unsigned int length, char const *intro = 0) const;

  /*!
    This function is not applicable to a pose vector that is always a
    6-by-1 column vector.
    \exception vpException::fatalError When this function is called.
    */
  void resize(const unsigned int nrows, const unsigned int ncols, const bool flagNullify = true)
  {
    (void)nrows;
    (void)ncols;
    (void)flagNullify;
    throw(vpException(vpException::fatalError, "Cannot resize a pose vector"));
  };

  // Save an homogeneous matrix in a file
  void save(std::ofstream &f) const;
  void set(const double tx, const double ty, const double tz, const double tux, const double tuy, const double tuz);
  vpRowVector t() const;

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
//@}
#endif
};

#endif
