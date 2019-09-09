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
 * 2D vanishing point visual feature (Z coordinate in 3D space is infinity)
 *
 * Authors:
 * Odile Bourquardez
 *
 *****************************************************************************/

/*!  \file vpFeatureVanishingPoint.cpp
  \brief Class that defines 2D vanishing
  point visual feature (Z coordinate in 3D space is infinity)
*/
#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/visual_features/vpFeatureVanishingPoint.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureException.h>

// Debug trace
#include <visp3/core/vpDebug.h>

// math
#include <visp3/core/vpMath.h>

#include <visp3/core/vpFeatureDisplay.h>

void vpFeatureVanishingPoint::init()
{
  // feature dimension
  dim_s = 2;
  nbParameters = 2;

  // memory allocation
  s.resize(dim_s);
  if (flags == NULL)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = false;

  // Z not required  (infinity)
  // set_Z(1) ;
}
vpFeatureVanishingPoint::vpFeatureVanishingPoint() { init(); }

//! set the point x-coordinates
void vpFeatureVanishingPoint::set_x(const double _x)
{
  s[0] = _x;
  flags[0] = true;
}
//! get the point x-coordinates
double vpFeatureVanishingPoint::get_x() const { return s[0]; }

//! set the point y-coordinates
void vpFeatureVanishingPoint::set_y(const double _y)
{
  s[1] = _y;
  flags[1] = true;
}
//! get the point y-coordinates
double vpFeatureVanishingPoint::get_y() const { return s[1]; }

//! set the point xy coordinates
void vpFeatureVanishingPoint::set_xy(const double _x, const double _y)
{
  set_x(_x);
  set_y(_y);
}

//! compute the interaction matrix from a subset of the possible features
vpMatrix vpFeatureVanishingPoint::interaction(const unsigned int select)
{
  vpMatrix L;

  L.resize(0, 6);

  if (deallocate == vpBasicFeature::user) {
    for (unsigned int i = 0; i < nbParameters; i++) {
      if (flags[i] == false) {
        switch (i) {
        case 0:
          vpTRACE("Warning !!!  The interaction matrix is computed but x was "
                  "not set yet");
          break;
        case 1:
          vpTRACE("Warning !!!  The interaction matrix is computed but y was "
                  "not set yet");
          break;
        default:
          vpTRACE("Problem during the reading of the variable flags");
        }
      }
    }
    resetFlags();
  }

  double x = get_x();
  double y = get_y();

  if (vpFeatureVanishingPoint::selectX() & select) {
    vpMatrix Lx(1, 6);
    Lx = 0;

    Lx[0][0] = 0.;
    Lx[0][1] = 0.;
    Lx[0][2] = 0.;
    Lx[0][3] = x * y;
    Lx[0][4] = -(1 + x * x);
    Lx[0][5] = y;

    L = vpMatrix::stack(L, Lx);
  }

  if (vpFeatureVanishingPoint::selectY() & select) {
    vpMatrix Ly(1, 6);
    Ly = 0;

    Ly[0][0] = 0;
    Ly[0][1] = 0.;
    Ly[0][2] = 0.;
    Ly[0][3] = 1 + y * y;
    Ly[0][4] = -x * y;
    Ly[0][5] = -x;

    L = vpMatrix::stack(L, Ly);
  }
  return L;
}

/*! compute the error between two visual features from a subset of the
  possible features
 */
vpColVector vpFeatureVanishingPoint::error(const vpBasicFeature &s_star, const unsigned int select)
{
  vpColVector e(0);

  try {
    if (vpFeatureVanishingPoint::selectX() & select) {
      vpColVector ex(1);
      ex[0] = s[0] - s_star[0];

      e = vpColVector::stack(e, ex);
    }

    if (vpFeatureVanishingPoint::selectY() & select) {
      vpColVector ey(1);
      ey[0] = s[1] - s_star[1];
      e = vpColVector::stack(e, ey);
    }
  } catch (...) {
    throw;
  }
  return e;
}

void vpFeatureVanishingPoint::print(const unsigned int select) const
{

  std::cout << "Point: " << std::endl;
  if (vpFeatureVanishingPoint::selectX() & select)
    std::cout << " x=" << get_x();
  if (vpFeatureVanishingPoint::selectY() & select)
    std::cout << " y=" << get_y();
  std::cout << std::endl;
}

void vpFeatureVanishingPoint::buildFrom(const double _x, const double _y)
{
  s[0] = _x;
  s[1] = _y;
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = true;
}

/*!

  Display vanishing point feature.

  \param cam : Camera parameters.
  \param I : Image.
  \param color : Color to use for the display.
  \param thickness : Thickness of the feature representation.

*/
void vpFeatureVanishingPoint::display(const vpCameraParameters &cam, const vpImage<unsigned char> &I,
                                      const vpColor &color, unsigned int thickness) const
{
  try {
    double x, y;
    x = get_x();
    y = get_y();

    vpFeatureDisplay::displayPoint(x, y, cam, I, color, thickness);

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}
/*!

  Display vanishing point feature.

  \param cam : Camera parameters.
  \param I : color Image.
  \param color : Color to use for the display.
  \param thickness : Thickness of the feature representation.

*/
void vpFeatureVanishingPoint::display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color,
                                      unsigned int thickness) const
{
  try {
    double x, y;
    x = get_x();
    y = get_y();

    vpFeatureDisplay::displayPoint(x, y, cam, I, color, thickness);

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*! for memory issue (used by the vpServo class only)
 */
vpFeatureVanishingPoint *vpFeatureVanishingPoint::duplicate() const
{
  vpFeatureVanishingPoint *feature = new vpFeatureVanishingPoint;
  return feature;
}

unsigned int vpFeatureVanishingPoint::selectX() { return FEATURE_LINE[0]; }
unsigned int vpFeatureVanishingPoint::selectY() { return FEATURE_LINE[1]; }
