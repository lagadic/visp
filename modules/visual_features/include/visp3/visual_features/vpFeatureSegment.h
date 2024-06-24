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
 * Segment visual feature.
 */

/*!
 * \file vpFeatureSegment.h
 * \brief class that defines the Segment visual feature
 */

#ifndef VP_FEATURE_SEGMENT_H
#define VP_FEATURE_SEGMENT_H

#include <visp3/core/vpConfig.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoint.h>
#include <visp3/core/vpRGBa.h>
#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/visual_features/vpFeatureException.h>

BEGIN_VISP_NAMESPACE
/*!
 * \class vpFeatureSegment
 * \ingroup group_visual_features
 *
 * \brief Class that defines a 2D segment visual features.
 * This class allow to consider two sets of visual features:
 * - the non normalized features \f${\bf s} = (x_c, y_c, l, \alpha)\f$ where
 *   \f$(x_c,y_c)\f$ are the coordinates of the segment center, \f$ l \f$ the
 *   segment length and \f$ \alpha \f$ the orientation of the segment with
 *   respect to the \f$ x \f$ axis.
 * - or the normalized features \f${\bf s} = (x_n, y_n, l_n, \alpha)\f$ with
 *   \f$x_n = x_c/l\f$, \f$y_n = y_c/l\f$ and \f$l_n = 1/l\f$.
 *
 * The selection of the feature set is done either during construction using
 * vpFeatureSegment(bool), or by setNormalized(bool).
*/
class VISP_EXPORT vpFeatureSegment : public vpBasicFeature
{
public:
  // empty constructor
  VP_EXPLICIT vpFeatureSegment(bool normalized = false);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  // change values of the segment
  void buildFrom(double x1, double y1, double Z1, double x2, double y2, double Z2);
#endif
// change values of the segment
  vpFeatureSegment &build(const double &x1, const double &y1, const double &Z1, const double &x2, const double &y2, const double &Z2);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const VP_OVERRIDE;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const VP_OVERRIDE;
  //! Feature duplication.
  vpFeatureSegment *duplicate() const VP_OVERRIDE;
  // compute the error between two visual features from a subset
  // a the possible features
  vpColVector error(const vpBasicFeature &s_star, unsigned int select = FEATURE_ALL) VP_OVERRIDE;

  /*!
   * Get the x coordinate of the segment center in the image plane.
   *
   * \return If normalized features are used, return \f$ x_n = x_c / l \f$.
   *  Otherwise return \f$ x_c \f$.
   */
  inline double getXc() const { return s[0]; }

  /*!
   * Get the y coordinate of the segment center in the image plane.
   *
   * \return If normalized features are used, return \f$ y_n = y_c / l \f$.
   *  Otherwise return \f$ y_c \f$.
   */
  inline double getYc() const { return s[1]; }

  /*!
   * Get the length of the segment.
   *
   * \return If normalized features are used, return \f$ l_n = 1 / l \f$.
   *  Otherwise return \f$ l \f$.
   */
  inline double getL() const { return s[2]; }

  /*!
   * Get the value of \f$ \alpha \f$ which represents the orientation of
   * the segment.
   *
   * \return The value of \f$ \alpha \f$.
   */
  inline double getAlpha() const { return s[3]; }

  /*!
   * Get the value of \f$ Z_1 \f$ which represents the Z coordinate in the
   * camera frame of the 3D point that corresponds to the segment first point.
   *
   * \return The value of the depth \f$ Z_1 \f$.
   */
  inline double getZ1() const { return Z1_; }

  /*!
   * Get the value of \f$ Z_2 \f$ which represents the Z coordinate in the
   * camera frame of the 3D point that corresponds to the segment second
   * point.
   *
   * \return The value of the depth \f$ Z_2 \f$.
   */
  inline double getZ2() const { return Z2_; }

  // Basic construction.
  void init() VP_OVERRIDE;

  // compute the interaction matrix from a subset a the possible features
  vpMatrix interaction(unsigned int select = FEATURE_ALL) VP_OVERRIDE;

  void print(unsigned int select = FEATURE_ALL) const VP_OVERRIDE;

  /*!
   * Indicates if the normalized features are considered.
   */
  bool isNormalized() { return normalized_; };

  static unsigned int selectXc();
  static unsigned int selectYc();
  static unsigned int selectL();
  static unsigned int selectAlpha();

  /*!
   * Set the king of feature to consider.
   * \param normalized : If true, use normalized features \f${\bf s} = (x_n,
   * y_n, l_n, \alpha)\f$. If false, use non normalized features \f${\bf s} =
   * (x_c, y_c, l_c, \alpha)\f$.
   */
  void setNormalized(bool normalized) { normalized_ = normalized; };

  /*!
   * Set the value of the x coordinate of the segment center
   * in the image plane.  It is one parameter of the visual feature \f$ s \f$.
   *
   * \param val : Value to set, that is either equal to \f$ x_n = x_c/l \f$
   * when normalized features are considered, or equal to \f$ x_c \f$
   * otherwise.
   */
  inline void setXc(double val)
  {
    s[0] = xc_ = val;
    flags[0] = true;
  }

  /*!
   * Set the value of the y coordinate of the segment center
   * in the image plane.  It is one parameter of the visual feature \f$ s \f$.
   *
   * \param val : Value to set, that is either equal to \f$ y_n = y_c/l \f$
   * when normalized features are considered, or equal to \f$ y_c \f$
   * otherwise.
   */
  inline void setYc(double val)
  {
    s[1] = yc_ = val;
    flags[1] = true;
  }

  /*!
   * Set the value of the segment length in the image plane. It is one
   * parameter of the visual feature \f$ s \f$.
   *
   * \param val : Value to set, that is either equal to \f$l_n= 1/l \f$ when
   * normalized features are considered, or equal to \f$ l \f$ otherwise.
   */
  inline void setL(double val)
  {
    s[2] = l_ = val;
    flags[2] = true;
  }

  /*!
   * Set the value of \f$ \alpha \f$ which represents the orientation of the
   * segment in the image plane. It is one parameter of the visual feature \f$
   * s \f$.
   *
   * \param val : \f$ \alpha \f$ value to set.
   */
  inline void setAlpha(double val)
  {
    s[3] = alpha_ = val;
    cos_a_ = cos(val);
    sin_a_ = sin(val);
    flags[3] = true;
  }

  /*!
   * Set the value of \f$ Z_1 \f$ which represents the Z coordinate in the
   * camera frame of the 3D point that corresponds to the segment first point.
   *
   * This value is requested to compute the interaction matrix.
   *
   * \param val : \f$ Z_1 \f$ value to set.
   *
   * \exception vpFeatureException::badInitializationError : If Z1 is behind
   * the camera or equal to zero.
   */
  inline void setZ1(double val)
  {
    Z1_ = val;

    if (Z1_ < 0) {
      vpERROR_TRACE("Point is behind the camera ");
      std::cout << "Z1 = " << Z1_ << std::endl;

      throw(vpFeatureException(vpFeatureException::badInitializationError, "Point Z1 is behind the camera "));
    }

    if (fabs(Z1_) < 1e-6) {
      vpERROR_TRACE("Point Z1 coordinates is null ");
      std::cout << "Z1 = " << Z1_ << std::endl;

      throw(vpFeatureException(vpFeatureException::badInitializationError, "Point Z1 coordinates is null"));
    }

    flags[4] = true;
  }

  /*!
   * Set the value of \f$ Z_2 \f$ which represents the Z coordinate in the
   * camera frame of the 3D point that corresponds to the segment second point.
   *
   * This value is requested to compute the interaction matrix.
   *
   * \param val : \f$ Z_2 \f$ value to set.
   *
   * \exception vpFeatureException::badInitializationError : If Z2 is behind
   * the camera or equal to zero.
   */
  inline void setZ2(double val)
  {
    Z2_ = val;

    if (Z2_ < 0) {
      vpERROR_TRACE("Point Z2 is behind the camera ");
      std::cout << "Z2 = " << Z2_ << std::endl;

      throw(vpFeatureException(vpFeatureException::badInitializationError, "Point Z2 is behind the camera "));
    }

    if (fabs(Z2_) < 1e-6) {
      vpERROR_TRACE("Point Z2 coordinates is null ");
      std::cout << "Z2 = " << Z2_ << std::endl;

      throw(vpFeatureException(vpFeatureException::badInitializationError, "Point Z2 coordinates is null"));
    }

    flags[5] = true;
  }

private:
  double xc_;
  double yc_;
  double l_;
  double alpha_;
  double Z1_;
  double Z2_;
  double cos_a_;
  double sin_a_;
  bool normalized_;
};
END_VISP_NAMESPACE
#endif
