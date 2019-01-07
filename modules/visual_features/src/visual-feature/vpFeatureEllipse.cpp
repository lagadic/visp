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

/*!
  \file vpFeatureEllipse.cpp
  \brief Class that defines 2D ellipse visual feature
*/

#include <visp3/visual_features/vpBasicFeature.h>
#include <visp3/visual_features/vpFeatureEllipse.h>

// Exception
#include <visp3/core/vpException.h>
#include <visp3/visual_features/vpFeatureException.h>

// Debug trace
#include <visp3/core/vpDebug.h>

// math
#include <visp3/core/vpMath.h>

#include <visp3/core/vpFeatureDisplay.h>

/*



attributes and members directly related to the vpBasicFeature needs
other functionalities ar useful but not mandatory





*/

void vpFeatureEllipse::init()
{
  // feature dimension
  dim_s = 5;
  nbParameters = 8;

  // memory allocation
  s.resize(dim_s);
  if (flags == NULL)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = false;

  // default depth values
  A = B = 0;
  C = 1;
}

vpFeatureEllipse::vpFeatureEllipse() : A(0), B(0), C(0) { init(); }

//! compute the interaction matrix from a subset a the possible features
vpMatrix vpFeatureEllipse::interaction(const unsigned int select)
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
        case 2:
          vpTRACE("Warning !!!  The interaction matrix is computed but mu20 "
                  "was not set yet");
          break;
        case 3:
          vpTRACE("Warning !!!  The interaction matrix is computed but mu11 "
                  "was not set yet");
          break;
        case 4:
          vpTRACE("Warning !!!  The interaction matrix is computed but mu02 "
                  "was not set yet");
          break;
        case 5:
          vpTRACE("Warning !!!  The interaction matrix is computed but A was "
                  "not set yet");
          break;
        case 6:
          vpTRACE("Warning !!!  The interaction matrix is computed but B was "
                  "not set yet");
          break;
        case 7:
          vpTRACE("Warning !!!  The interaction matrix is computed but C was "
                  "not set yet");
          break;
        default:
          vpTRACE("Problem during the reading of the variable flags");
        }
      }
    }
    resetFlags();
  }

  double xc = s[0];
  double yc = s[1];
  double mu20 = s[2];
  double mu11 = s[3];
  double mu02 = s[4];

  // eq 39
  double Z = 1 / (A * xc + B * yc + C);

  if (vpFeatureEllipse::selectX() & select) {
    vpMatrix H(1, 6);
    H = 0;

    H[0][0] = -1 / Z;
    H[0][1] = 0;
    H[0][2] = xc / Z + A * mu20 + B * mu11;
    H[0][3] = xc * yc + mu11;
    H[0][4] = -1 - vpMath::sqr(xc) - mu20;
    H[0][5] = yc;

    L = vpMatrix::stack(L, H);
  }

  if (vpFeatureEllipse::selectY() & select) {
    vpMatrix H(1, 6);
    H = 0;

    H[0][0] = 0;
    H[0][1] = -1 / Z;
    H[0][2] = yc / Z + A * mu11 + B * mu02;
    H[0][3] = 1 + vpMath::sqr(yc) + mu02;
    H[0][4] = -xc * yc - mu11;
    H[0][5] = -xc;

    L = vpMatrix::stack(L, H);
  }

  if (vpFeatureEllipse::selectMu20() & select) {
    vpMatrix H(1, 6);
    H = 0;

    H[0][0] = -2 * (A * mu20 + B * mu11);
    H[0][1] = 0;
    H[0][2] = 2 * ((1 / Z + A * xc) * mu20 + B * xc * mu11);
    H[0][3] = 2 * (yc * mu20 + xc * mu11);
    H[0][4] = -4 * mu20 * xc;
    H[0][5] = 2 * mu11;

    L = vpMatrix::stack(L, H);
  }

  if (vpFeatureEllipse::selectMu11() & select) {
    vpMatrix H(1, 6);
    H = 0;

    H[0][0] = -A * mu11 - B * mu02;
    H[0][1] = -A * mu20 - B * mu11;
    H[0][2] = A * yc * mu20 + (3 / Z - C) * mu11 + B * xc * mu02;
    H[0][3] = 3 * yc * mu11 + xc * mu02;
    H[0][4] = -yc * mu20 - 3 * xc * mu11;
    H[0][5] = mu02 - mu20;

    L = vpMatrix::stack(L, H);
  }

  if (vpFeatureEllipse::selectMu02() & select) {
    vpMatrix H(1, 6);
    H = 0;

    H[0][0] = 0;
    H[0][1] = -2 * (A * mu11 + B * mu02);
    H[0][2] = 2 * ((1 / Z + B * yc) * mu02 + A * yc * mu11);
    H[0][3] = 4 * yc * mu02;
    H[0][4] = -2 * (yc * mu11 + xc * mu02);
    H[0][5] = -2 * mu11;
    L = vpMatrix::stack(L, H);
  }

  return L;
}

//! compute the error between two visual features from a subset
//! a the possible features
vpColVector vpFeatureEllipse::error(const vpBasicFeature &s_star, const unsigned int select)
{
  vpColVector e(0);

  try {
    if (vpFeatureEllipse::selectX() & select) {
      vpColVector ex(1);
      ex[0] = s[0] - s_star[0];

      e = vpColVector::stack(e, ex);
    }

    if (vpFeatureEllipse::selectY() & select) {
      vpColVector ey(1);
      ey[0] = s[1] - s_star[1];
      e = vpColVector::stack(e, ey);
    }

    if (vpFeatureEllipse::selectMu20() & select) {
      vpColVector ex(1);
      ex[0] = s[2] - s_star[2];

      e = vpColVector::stack(e, ex);
    }

    if (vpFeatureEllipse::selectMu11() & select) {
      vpColVector ey(1);
      ey[0] = s[3] - s_star[3];
      e = vpColVector::stack(e, ey);
    }

    if (vpFeatureEllipse::selectMu02() & select) {
      vpColVector ey(1);
      ey[0] = s[4] - s_star[4];
      e = vpColVector::stack(e, ey);
    }

  } catch (...) {
    throw;
  }

  return e;
}

void vpFeatureEllipse::print(const unsigned int select) const
{

  std::cout << "Ellipse:  " << std::endl;
  if (vpFeatureEllipse::selectX() & select)
    std::cout << " x=" << s[0] << std::endl;
  ;
  if (vpFeatureEllipse::selectY() & select)
    std::cout << " y=" << s[1] << std::endl;
  if (vpFeatureEllipse::selectMu20() & select)
    std::cout << " mu20=" << s[2] << std::endl;
  if (vpFeatureEllipse::selectMu11() & select)
    std::cout << " mu11=" << s[3] << std::endl;
  if (vpFeatureEllipse::selectMu02() & select)
    std::cout << " mu02=" << s[4] << std::endl;
  std::cout << "A = " << A << " B = " << B << " C = " << C << std::endl;
}

void vpFeatureEllipse::buildFrom(const double x, const double y, const double mu20, const double mu11,
                                 const double mu02)
{

  s[0] = x;
  s[1] = y;
  s[2] = mu20;
  s[3] = mu11;
  s[4] = mu02;

  for (int i = 0; i < 5; i++)
    flags[i] = true;
}

void vpFeatureEllipse::buildFrom(const double x, const double y, const double mu20, const double mu11,
                                 const double mu02, const double a, const double b, const double c)
{

  s[0] = x;
  s[1] = y;
  s[2] = mu20;
  s[3] = mu11;
  s[4] = mu02;

  this->A = a;
  this->B = b;
  this->C = c;

  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = true;
}

void vpFeatureEllipse::set_x(const double x)
{
  s[0] = x;
  flags[0] = true;
}

void vpFeatureEllipse::set_y(const double y)
{
  s[1] = y;
  flags[1] = true;
}

void vpFeatureEllipse::set_xy(const double x, const double y)
{
  s[0] = x;
  s[1] = y;
  for (int i = 0; i < 2; i++)
    flags[i] = true;
}

void vpFeatureEllipse::setABC(const double a, const double b, const double c)
{
  this->A = a;
  this->B = b;
  this->C = c;
  for (unsigned int i = 5; i < nbParameters; i++)
    flags[i] = true;
}

void vpFeatureEllipse::setMu(const double mu20, const double mu11, const double mu02)
{

  s[2] = mu20;
  s[3] = mu11;
  s[4] = mu02;
  for (int i = 2; i < 5; i++)
    flags[i] = true;
}

/*!

  Display ellipse feature.

  \param cam : Camera parameters.
  \param I : Image on which features have to be displayed.
  \param color : Color used to display the feature.
  \param thickness : Thickness of the feature representation.
*/
void vpFeatureEllipse::display(const vpCameraParameters &cam, const vpImage<unsigned char> &I, const vpColor &color,
                               unsigned int thickness) const
{
  try {
    double x = s[0];
    double y = s[1];

    double mu20 = s[2];
    double mu11 = s[3];
    double mu02 = s[4];

    vpFeatureDisplay::displayEllipse(x, y, mu20, mu11, mu02, cam, I, color, thickness);

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Display ellipse feature.

  \param cam : Camera parameters.
  \param I : Color image on which features have to be displayed.
  \param color : Color used to display the feature.
  \param thickness : Thickness of the feature representation.
*/
void vpFeatureEllipse::display(const vpCameraParameters &cam, const vpImage<vpRGBa> &I, const vpColor &color,
                               unsigned int thickness) const
{
  try {
    double x = s[0];
    double y = s[1];

    double mu20 = s[2];
    double mu11 = s[3];
    double mu02 = s[4];

    vpFeatureDisplay::displayEllipse(x, y, mu20, mu11, mu02, cam, I, color, thickness);

  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

//! for memory issue (used by the vpServo class only)
vpFeatureEllipse *vpFeatureEllipse::duplicate() const
{
  vpFeatureEllipse *feature = new vpFeatureEllipse;
  return feature;
}

unsigned int vpFeatureEllipse::selectX() { return FEATURE_LINE[0]; }
unsigned int vpFeatureEllipse::selectY() { return FEATURE_LINE[1]; }
unsigned int vpFeatureEllipse::selectMu20() { return FEATURE_LINE[2]; }
unsigned int vpFeatureEllipse::selectMu11() { return FEATURE_LINE[3]; }
unsigned int vpFeatureEllipse::selectMu02() { return FEATURE_LINE[4]; }
