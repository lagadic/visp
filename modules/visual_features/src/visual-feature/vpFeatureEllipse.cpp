/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2023 by Inria. All rights reserved.
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
 * 2D ellipse visual feature.
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

BEGIN_VISP_NAMESPACE

void vpFeatureEllipse::init()
{
  // feature dimension
  dim_s = 5;
  nbParameters = 8;

  // memory allocation
  s.resize(dim_s);
  if (flags == nullptr)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = false;

  // default depth values
  A = B = 0;
  C = 1;
}

vpFeatureEllipse::vpFeatureEllipse() : A(0), B(0), C(0) { init(); }
vpFeatureEllipse::vpFeatureEllipse(double x, double y, double n20, double n11, double n02) { this->build(x, y, n20, n11, n02); }


//! compute the interaction matrix from a subset a the possible features
vpMatrix vpFeatureEllipse::interaction(unsigned int select)
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
          vpTRACE("Warning !!!  The interaction matrix is computed but n20 "
                  "was not set yet");
          break;
        case 3:
          vpTRACE("Warning !!!  The interaction matrix is computed but n11 "
                  "was not set yet");
          break;
        case 4:
          vpTRACE("Warning !!!  The interaction matrix is computed but n02 "
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
  double n20 = s[2];
  double n11 = s[3];
  double n02 = s[4];

  double Zinv = A * xc + B * yc + C;

  if (vpFeatureEllipse::selectX() & select) {
    vpMatrix H(1, 6);
    H = 0;
    // Eq (14) of Chaumette 2004 TRO paper on moments
    H[0][0] = -Zinv;
    H[0][1] = 0;
    H[0][2] = xc * Zinv + 4.0 * (A * n20 + B * n11);
    H[0][3] = xc * yc + 4.0 * n11;
    H[0][4] = -1 - vpMath::sqr(xc) - 4.0 * n20;
    H[0][5] = yc;

    L = vpMatrix::stack(L, H);
  }

  if (vpFeatureEllipse::selectY() & select) {
    vpMatrix H(1, 6);
    H = 0;
    // Eq (14) of Chaumette 2004 TRO paper on moments
    H[0][0] = 0;
    H[0][1] = -Zinv;
    H[0][2] = yc * Zinv + 4.0 * (A * n11 + B * n02);
    H[0][3] = 1 + vpMath::sqr(yc) + 4.0 * n02;
    H[0][4] = -xc * yc - 4.0 * n11;
    H[0][5] = -xc;

    L = vpMatrix::stack(L, H);
  }

  if (vpFeatureEllipse::select_n20() & select) {
    vpMatrix H(1, 6);
    H = 0;
    // Eq (26) of Chaumette 2004 TRO paper on moments
    H[0][0] = -2.0 * (A * n20 + B * n11);
    H[0][1] = 0;
    H[0][2] = 2 * ((Zinv + A * xc) * n20 + B * xc * n11);
    H[0][3] = 2 * (yc * n20 + xc * n11);
    H[0][4] = -4 * n20 * xc;
    H[0][5] = 2 * n11;

    L = vpMatrix::stack(L, H);
  }

  if (vpFeatureEllipse::select_n11() & select) {
    vpMatrix H(1, 6);
    H = 0;
    // Eq (26) of Chaumette 2004 TRO paper on moments
    H[0][0] = -A * n11 - B * n02;
    H[0][1] = -A * n20 - B * n11;
    H[0][2] = A * yc * n20 + (3 * Zinv - C) * n11 + B * xc * n02;
    H[0][3] = 3 * yc * n11 + xc * n02;
    H[0][4] = -yc * n20 - 3 * xc * n11;
    H[0][5] = n02 - n20;

    L = vpMatrix::stack(L, H);
  }

  if (vpFeatureEllipse::select_n02() & select) {
    vpMatrix H(1, 6);
    H = 0;
    // Eq (26) of Chaumette 2004 TRO paper on moments
    H[0][0] = 0;
    H[0][1] = -2 * (A * n11 + B * n02);
    H[0][2] = 2 * ((Zinv + B * yc) * n02 + A * yc * n11);
    H[0][3] = 4 * yc * n02;
    H[0][4] = -2 * (yc * n11 + xc * n02);
    H[0][5] = -2 * n11;
    L = vpMatrix::stack(L, H);
  }

  return L;
}

//! compute the error between two visual features from a subset
//! a the possible features
vpColVector vpFeatureEllipse::error(const vpBasicFeature &s_star, unsigned int select)
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

    if (vpFeatureEllipse::select_n20() & select) {
      vpColVector ex(1);
      ex[0] = s[2] - s_star[2];

      e = vpColVector::stack(e, ex);
    }

    if (vpFeatureEllipse::select_n11() & select) {
      vpColVector ey(1);
      ey[0] = s[3] - s_star[3];
      e = vpColVector::stack(e, ey);
    }

    if (vpFeatureEllipse::select_n02() & select) {
      vpColVector ey(1);
      ey[0] = s[4] - s_star[4];
      e = vpColVector::stack(e, ey);
    }

  }
  catch (...) {
    throw;
  }

  return e;
}

void vpFeatureEllipse::print(unsigned int select) const
{

  std::cout << "Ellipse:  " << std::endl;
  if (vpFeatureEllipse::selectX() & select)
    std::cout << " x=" << s[0] << std::endl;
  ;
  if (vpFeatureEllipse::selectY() & select)
    std::cout << " y=" << s[1] << std::endl;
  if (vpFeatureEllipse::select_n20() & select)
    std::cout << " n20=" << s[2] << std::endl;
  if (vpFeatureEllipse::select_n11() & select)
    std::cout << " n11=" << s[3] << std::endl;
  if (vpFeatureEllipse::select_n02() & select)
    std::cout << " n02=" << s[4] << std::endl;
  std::cout << "A = " << A << " B = " << B << " C = " << C << std::endl;
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  \deprecated You should use build(const double &, const double &, const double &, const double &, const double &) instead.
 */
void vpFeatureEllipse::buildFrom(double x, double y, double n20, double n11, double n02)
{
  build(x, y, n20, n11, n02);
}

/*!
  \deprecated You should use build(const double &, const double &, const double &, const double &, const double &) instead.
 */
void vpFeatureEllipse::buildFrom(double x, double y, double n20, double n11, double n02, double a, double b, double c)
{
  build(x, y, n20, n11, n02, a, b, c);
}
#endif

vpFeatureEllipse &vpFeatureEllipse::build(const double &x, const double &y, const double &n20, const double &n11, const double &n02)
{
  s[0] = x;
  s[1] = y;
  s[2] = n20;
  s[3] = n11;
  s[4] = n02;

  for (int i = 0; i < 5; ++i) {
    flags[i] = true;
  }
  return *this;
}

vpFeatureEllipse &vpFeatureEllipse::build(const double &x, const double &y, const double &n20, const double &n11, const double &n02, const double &a, const double &b, const double &c)
{
  s[0] = x;
  s[1] = y;
  s[2] = n20;
  s[3] = n11;
  s[4] = n02;

  this->A = a;
  this->B = b;
  this->C = c;

  for (unsigned int i = 0; i < nbParameters; ++i) {
    flags[i] = true;
  }
  return *this;
}

void vpFeatureEllipse::set_x(double x)
{
  s[0] = x;
  flags[0] = true;
}

void vpFeatureEllipse::set_y(double y)
{
  s[1] = y;
  flags[1] = true;
}

void vpFeatureEllipse::set_xy(double x, double y)
{
  s[0] = x;
  s[1] = y;
  for (int i = 0; i < 2; i++)
    flags[i] = true;
}

void vpFeatureEllipse::setABC(double a, double b, double c)
{
  this->A = a;
  this->B = b;
  this->C = c;
  for (unsigned int i = 5; i < nbParameters; i++)
    flags[i] = true;
}

/*!
 * Update visual features corresponding to the second order centered moments of the ellipse normalized
 * by its area (i.e., such that \f$n_{ij} = \mu_{ij}/a\f$ where \f$\mu_{ij}\f$ are the centered moments
 * and a the area).
 * \param n20, n11, n02: Second order centered moments.
 */
void vpFeatureEllipse::setMoments(double n20, double n11, double n02)
{
  s[2] = n20;
  s[3] = n11;
  s[4] = n02;
  for (int i = 2; i < 5; i++)
    flags[i] = true;
}

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
 * \deprecated You should rather use setMoments().
 * This function and its parameters are incorrectly named and are confusing since this function
 * is waiting for second order centered moments of the ellipse normalized
 * by its area that corresponds to \f$n_{ij} = \mu_{ij}/a\f$.
 */
void vpFeatureEllipse::setMu(double mu20, double mu11, double mu02) { setMoments(mu20, mu11, mu02); }
#endif

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
  double x = s[0];
  double y = s[1];

  double n20 = s[2];
  double n11 = s[3];
  double n02 = s[4];

  vpFeatureDisplay::displayEllipse(x, y, n20, n11, n02, cam, I, color, thickness);
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
  double x = s[0];
  double y = s[1];

  double n20 = s[2];
  double n11 = s[3];
  double n02 = s[4];

  vpFeatureDisplay::displayEllipse(x, y, n20, n11, n02, cam, I, color, thickness);
}

//! For memory issue (used by the vpServo class only).
vpFeatureEllipse *vpFeatureEllipse::duplicate() const
{
  vpFeatureEllipse *feature = new vpFeatureEllipse;
  return feature;
}

/*!
 * Select as visual feature ellipse centroid coordinate along camera x-axis.
 */
unsigned int vpFeatureEllipse::selectX() { return FEATURE_LINE[0]; }
/*!
 * Select as visual feature ellipse centroid coordinate along camera y-axis.
 */
unsigned int vpFeatureEllipse::selectY() { return FEATURE_LINE[1]; }

/*!
 * Select as visual feature second order centered moments of the ellipse normalized
 * by its area that corresponds to \f$n_20 = mu_20/a\f$.
 */
unsigned int vpFeatureEllipse::select_n20() { return FEATURE_LINE[2]; }
/*!
 * Select as visual feature second order centered moments of the ellipse normalized
 * by its area that corresponds to \f$n_11 = mu_11/a\f$.
 */
unsigned int vpFeatureEllipse::select_n11() { return FEATURE_LINE[3]; }
/*!
 * Select as visual feature second order centered moments of the ellipse normalized
 * by its area that corresponds to \f$n_02 = mu_02/a\f$.
 */
unsigned int vpFeatureEllipse::select_n02() { return FEATURE_LINE[4]; }

#if defined(VISP_BUILD_DEPRECATED_FUNCTIONS)
/*!
 * \deprecated You should rather use select_n20().
 * This function is incorrectly named and is confusing since it
 * intends to select as visual feature second order centered moments of the ellipse normalized
 * by its area that corresponds to \f$n_20 = mu_20/a\f$.
 */
VP_DEPRECATED unsigned int vpFeatureEllipse::selectMu20() { return FEATURE_LINE[2]; }
/*!
 * \deprecated You should rather use select_n20().
 * This function is incorrectly named and is confusing since it
 * intends to select as visual feature second order centered moments of the ellipse normalized
 * by its area that corresponds to \f$n_11 = mu_11/a\f$.
 */
VP_DEPRECATED unsigned int vpFeatureEllipse::selectMu11() { return FEATURE_LINE[3]; }
/*!
 * \deprecated You should rather use select_n20().
 * This function is incorrectly named and is confusing since it
 * intends to select as visual feature second order centered moments of the ellipse normalized
 * by its area that corresponds to \f$n_02 = mu_02/a\f$.
 */
VP_DEPRECATED unsigned int vpFeatureEllipse::selectMu02() { return FEATURE_LINE[4]; }
#endif
END_VISP_NAMESPACE
