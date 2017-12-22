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
 * 2D vanishing point visual feature (Z coordinate in 3D space is infinity)
 *
 * Authors:
 * Odile Bourquardez
 *
 *****************************************************************************/

#ifndef vpFeatureVanishingPoint_H
#define vpFeatureVanishingPoint_H

/*!
  \file vpFeatureVanishingPoint.h \brief Class that defines 2D vanishing
  point visual feature (Z coordinate in 3D space is infinity)
*/

#include <visp3/core/vpMatrix.h>
#include <visp3/core/vpPoint.h>
#include <visp3/visual_features/vpBasicFeature.h>

#include <visp3/core/vpHomogeneousMatrix.h>
#include <visp3/core/vpRGBa.h>

/*!
  \class vpFeatureVanishingPoint
  \ingroup group_visual_features

  \brief Class that defines 2D vanishing point visual feature (Z
  coordinate in 3D space is infinity).
*/
class VISP_EXPORT vpFeatureVanishingPoint : public vpBasicFeature
{
public:
  typedef enum {
    X = 1, // x coordinates
    Y = 2  // y coordinates
  } vpFeatureVanishingPointType;

  /*
    attributes and members directly related to the vpBasicFeature needs
    other functionalities ar useful but not mandatory
  */
  // no Z required

public:
  //! Default constructor.
  vpFeatureVanishingPoint();
  //! Destructor.
  virtual ~vpFeatureVanishingPoint() {}

  // void buildFrom(const vpPoint &p) ;
  void buildFrom(const double _x, const double _y);

  void display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;
  void display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color = vpColor::green,
               unsigned int thickness = 1) const;

  //! feature duplication
  vpFeatureVanishingPoint *duplicate() const;

  //! compute the error between two visual features from a subset
  //! a the possible features
  vpColVector error(const vpBasicFeature &s_star, const unsigned int select = FEATURE_ALL);
  //! compute the error between a visual features and zero
  vpColVector error(const unsigned int select = FEATURE_ALL);

  //! get the point x-coordinates
  double get_x() const;
  //! get the point y-coordinates
  double get_y() const;

  //! Default initialization.
  void init();
  //! compute the interaction matrix from a subset a the possible features
  vpMatrix interaction(const unsigned int select = FEATURE_ALL);

  //! print the name of the feature
  void print(const unsigned int select = FEATURE_ALL) const;

  //! Set the point x-coordinates
  void set_x(const double _x);
  //! Set the point y-coordinates
  void set_y(const double _y);
  //! Set the point xy coordinates
  void set_xy(const double _x, const double _y);

public:
  /*
    vpBasicFeature method instantiation
  */
  // feature selection
  static unsigned int selectX();
  static unsigned int selectY();
};

#endif
