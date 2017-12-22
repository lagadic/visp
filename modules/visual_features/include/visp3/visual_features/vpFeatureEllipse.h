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
  vpFeatureEllipse(const double x, const double y, const double mu20, const double mu11, const double mu02);

  // void buildFrom(const vpEllipse &p) ;
  void buildFrom(const double x, const double y, const double mu20, const double mu11, const double mu02);
  void buildFrom(const double x, const double y, const double mu20, const double mu11, const double mu02,
                 const double A, const double B, const double C);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  //! Feature duplication
  vpFeatureEllipse *duplicate() const;

  //! compute the error between two visual features from a subset
  //! a the possible features
  vpColVector error(const vpBasicFeature &s_star, const unsigned int select = FEATURE_ALL);
  //! compute the error between a visual features and zero
  vpColVector error(const unsigned int select = FEATURE_ALL);

  double get_x() const { return s[0]; }
  double get_y() const { return s[1]; }
  double getMu20() const { return s[2]; }
  double getMu11() const { return s[3]; }
  double getMu02() const { return s[4]; }

  //! Default initialization.
  void init();
  //! compute the interaction matrix from a subset a the possible features
  vpMatrix interaction(const unsigned int select = FEATURE_ALL);

  //! print the name of the feature
  void print(const unsigned int select = FEATURE_ALL) const;

  void set_x(const double x);
  void set_y(const double y);
  void set_xy(const double x, const double y);
  void setABC(const double A, const double B, const double C);
  void setMu(const double mu20, const double mu11, const double mu02);

public:
  /*!
    vpBasicFeature method instantiation
  */

  // feature selection
  static unsigned int selectX();
  static unsigned int selectY();
  static unsigned int selectMu20();
  static unsigned int selectMu11();
  static unsigned int selectMu02();
};

#endif
