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
 * 2D ellipse visual feature.
 *
 * Authors:
 * Eric Marchand
 *
 *****************************************************************************/

#ifndef vpFeatureEllipse_H
#define vpFeatureEllipse_H

/*!
  \file vpFeatureEllipse.h
  \brief Class that defines 2D ellipse visual feature
*/

#include <visp3/core/vpMatrix.h>
#include <visp3/visual_features/vpBasicFeature.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRGBa.h>

/*!
  \class vpFeatureEllipse
  \ingroup group_visual_features
  \brief Class that defines 2D ellipse visual feature.
*/
class VISP_EXPORT vpFeatureEllipse : public vpBasicFeature
{
  /*
    attributes and members directly related to the vpBasicFeature needs
    other functionalities are useful but not mandatory
  */
private:
  //! FeatureEllipse depth (required to compute the interaction matrix)
  //! default Z = 1m
  double A, B, C;

public:
  //! Default constructor.
  vpFeatureEllipse();
  //! Destructor.
  virtual ~vpFeatureEllipse() {}

  /*!
    \section Set coordinates
  */
  //! basic constructor
  vpFeatureEllipse(double x, double y, double n20, double n11, double n02);

  // void buildFrom(const vpEllipse &p) ;
  void buildFrom(double x, double y, double n20, double n11, double n02);
  void buildFrom(double x, double y, double n20, double n11, double n02,
                 double A, double B, double C);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  //! Feature duplication
  vpFeatureEllipse *duplicate() const;

  //! compute the error between two visual features from a subset
  //! a the possible features
  vpColVector error(const vpBasicFeature &s_star, unsigned int select = FEATURE_ALL);
  //! compute the error between a visual features and zero
  vpColVector error(unsigned int select = FEATURE_ALL);

  /*!
   * Returns the visual feature corresponding to the ellipse centroid coordinate along camera x-axis.
   */
  double get_x() const { return s[0]; }
  /*!
   * Returns the visual feature corresponding to the ellipse centroid coordinate along camera y-axis.
   */
  double get_y() const { return s[1]; }
  /*!
   * Returns the visual feature corresponding to the second order centered moments
   * of the ellipse normalized by its area \f$n_20 = mu_20/a\f$.
   */
  double get_n20() const { return s[2]; }
  /*!
   * Returns the visual feature corresponding to the second order centered moments
   * of the ellipse normalized by its area \f$n_11 = mu_11/a\f$.
   */
  double get_n11() const { return s[3]; }
  /*!
   * Returns the visual feature corresponding to the second order centered moments
   * of the ellipse normalized by its area \f$n_02 = mu_02/a\f$.
   */
  double get_n02() const { return s[4]; }

  //! Default initialization.
  void init();
  //! compute the interaction matrix from a subset a the possible features
  vpMatrix interaction(unsigned int select = FEATURE_ALL);

  //! print the name of the feature
  void print(unsigned int select = FEATURE_ALL) const;

  void set_x(double x);
  void set_y(double y);
  void set_xy(double x, double y);
  void setABC(double A, double B, double C);
  void setMoments(double n20, double n11, double n02);

  /*!
    vpBasicFeature method instantiation
  */

  // feature selection
  static unsigned int selectX();
  static unsigned int selectY();
  static unsigned int select_n20();
  static unsigned int select_n11();
  static unsigned int select_n02();

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
  /*!
    @name Deprecated functions
  */
  //@{
  vp_deprecated void setMu(double mu20, double mu11, double mu02);
  vp_deprecated static unsigned int selectMu20();
  vp_deprecated static unsigned int selectMu11();
  vp_deprecated static unsigned int selectMu02();
  /*!
   * \deprecated You should rather use get_n20().
   * This function is incorrectly named and is confusing since it
   * returns the visual feature corresponding to the second order centered moments
   * of the ellipse normalized by its area \f$n_20 = mu_20/a\f$.
   */
  vp_deprecated double getMu20() const { return s[2]; }
  /*!
   * \deprecated You should rather use get_n11().
   * This function is incorrectly named and is confusing since it
   * returns the visual feature corresponding to the second order centered moments
   * of the ellipse normalized by its area \f$n_11 = mu_11/a\f$.
   */
  vp_deprecated double getMu11() const { return s[3]; }
  /*!
   * \deprecated You should rather use get_n02().
   * This function is incorrectly named and is confusing since it
   * returns the visual feature corresponding to the second order centered moments
   * of the ellipse normalized by its area \f$n_02 = mu_02/a\f$.
   */
  vp_deprecated double getMu02() const { return s[4]; }

  //@}
#endif

};

#endif
