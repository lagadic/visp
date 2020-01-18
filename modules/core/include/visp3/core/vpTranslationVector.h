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
 * Translation vector.
 *
 * Authors:
 * Eric Marchand
 * Fabien Spindler
 *
 *****************************************************************************/

#ifndef vpTRANSLATIONVECTOR_H
#define vpTRANSLATIONVECTOR_H

/*!
  \file vpTranslationVector.h
  \brief Class that consider the case of a translation vector.
*/

#include <visp3/core/vpArray2D.h>
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoseVector.h>

/*!
  \class vpTranslationVector

  \ingroup group_core_transformations

  \brief Class that consider the case of a translation vector.

  Let us denote \f$^{a}{\bf t}_{b} = [t_x,t_y,t_z]^\top\f$ the translation
  from frame \f$ a \f$ to frame \f$ b \f$.  The representation of a
  translation is a column vector of dimension 3.

  Translations along x,y,z axis are expressed in meters.

  From the implementation point of view, it is nothing more than an
  array of three doubles with values in [meters].

  You can set values [meters] accessing each element:
  \code
  vpTranslationVector t;
  t[0] = 0;
  t[1] = 0.1;
  t[2] = 0.5;
  \endcode
  You can also initialize the vector using operator<<(double):
  \code
  t << 0, 0.1, 05;
  \endcode
  Or you can also initialize the vector from a list of doubles if ViSP is build with c++11 enabled:
  \code
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  t = {0, 0.1, 0.5};
#endif
  \endcode

  To get the values [meters] use:
  \code
  double tx = t[0];
  double ty = t[1];
  double tz = t[2];
  \endcode

  The code below shows how to use a translation vector to build an
  homogeneous matrix.

  \code
#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRotationMatrix.h>
#include <visp3/core/vpTranslationVector.h>

int main()
{
  vpTranslationVector t; // Translation vector

  // Initialization of the translation vector
  t[0] =  0.2; // tx = 0.2 meters
  t[1] = -0.1; // ty = -0.1 meters
  t[2] =  1.0; // tz = 1 meters

  // Construction of a rotation matrix
  vpRotationMatrix R; // Set to identity by default

  // Construction of an homogeneous matrix
  vpHomogeneousMatrix M(t, R);
}
  \endcode
*/
class VISP_EXPORT vpTranslationVector : public vpArray2D<double>
{
public:
  /*!
      Default constructor.
      The translation vector is initialized to zero.
    */
  vpTranslationVector() : vpArray2D<double>(3, 1), m_index(0) {};
  vpTranslationVector(const double tx, const double ty, const double tz);
  vpTranslationVector(const vpTranslationVector &tv);
  explicit vpTranslationVector(const vpHomogeneousMatrix &M);
  explicit vpTranslationVector(const vpPoseVector &p);
  explicit vpTranslationVector(const vpColVector &v);

  vpTranslationVector buildFrom(const double tx, const double ty, const double tz);
  vpTranslationVector buildFrom(const vpHomogeneousMatrix &M);
  vpTranslationVector buildFrom(const vpPoseVector &p);
  vpTranslationVector buildFrom(const vpColVector &v);

  vp_deprecated double euclideanNorm() const;
  double frobeniusNorm() const;

  // operators

  // translation vectors additions  c = a + b (a, b  unchanged)
  vpTranslationVector operator+(const vpTranslationVector &tv) const;
  vpTranslationVector operator+(const vpColVector &v) const;
  // translation vectors substraction  c = a - b (a, b  unchanged)
  vpTranslationVector operator-(const vpTranslationVector &tv) const;
  // negate t = -a  (t is unchanged)
  vpTranslationVector operator-() const;
  vpMatrix operator*(const vpRowVector &v) const;
  // b = x * a (x=scalar)
  vpTranslationVector operator*(const double x) const;
  vpTranslationVector &operator*=(double x);
  vpTranslationVector operator/(const double x) const;
  vpTranslationVector &operator/=(double x);
  // Copy operator.   Allow operation such as A = v
  vpTranslationVector &operator=(const vpColVector &tv);
  vpTranslationVector &operator=(const vpTranslationVector &tv);
  vpTranslationVector &operator=(double x);
#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
  vpTranslationVector &operator=(const std::initializer_list<double> &list);
#endif

  //! Operator that allows to set a value of an element \f$t_i\f$: t[i] = x
  inline double &operator[](unsigned int n) { return *(data + n); }
  //! Operator that allows to get the value of an element \f$t_i\f$: x = t[i]
  inline const double &operator[](unsigned int n) const { return *(data + n); }

  vpTranslationVector &operator<<(double val);
  vpTranslationVector &operator,(double val);


  /*!
    This function is not applicable to a translation vector that is always a
    3-by-1 column vector.
    \exception vpException::fatalError When this function is called.
    */
  void resize(const unsigned int nrows, const unsigned int ncols, const bool flagNullify = true)
  {
    (void)nrows;
    (void)ncols;
    (void)flagNullify;
    throw(vpException(vpException::fatalError, "Cannot resize a translation vector"));
  };

  void set(const double tx, const double ty, const double tz);

  // Skew Symmetric matrix
  vpMatrix skew() const;

  double sumSquare() const;

  vpRowVector t() const;

  static vpTranslationVector cross(const vpTranslationVector &a, const vpTranslationVector &b);
  static vpTranslationVector mean(const std::vector<vpHomogeneousMatrix> &vec_M);
  static vpTranslationVector mean(const std::vector<vpTranslationVector> &vec_t);
  static vpMatrix skew(const vpTranslationVector &tv);
  static void skew(const vpTranslationVector &tv, vpMatrix &M);

protected:
  unsigned int m_index; // index used for operator<< and operator, to fill a vector
};

#endif
