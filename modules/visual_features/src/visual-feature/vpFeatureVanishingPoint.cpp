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

BEGIN_VISP_NAMESPACE
/*!
 * Vanishing point visual feature initialization.
 */
void vpFeatureVanishingPoint::init()
{
  // Feature dimension
  dim_s = 5;
  nbParameters = 5;
  m_select = 0;

  // memory allocation
  s.resize(dim_s);
  if (flags == nullptr)
    flags = new bool[nbParameters];
  for (unsigned int i = 0; i < nbParameters; i++)
    flags[i] = false;
}

//! Default constructor that calls init().
vpFeatureVanishingPoint::vpFeatureVanishingPoint() { init(); }

//! Set vanishing point feature \f$ x \f$ value.
void vpFeatureVanishingPoint::set_x(double x)
{
  s[0] = x;
  flags[0] = true;
  m_select |= selectX();
}

//! Get vanishing point feature \f$ x \f$ value.
double vpFeatureVanishingPoint::get_x() const { return s[0]; }

//! Set vanishing point feature \f$ y \f$ value.
void vpFeatureVanishingPoint::set_y(double y)
{
  s[1] = y;
  flags[1] = true;
  m_select |= selectY();
}

//! Get vanishing point feature \f$ y \f$ value.
double vpFeatureVanishingPoint::get_y() const { return s[1]; }

//! Set vanishing point visual feature \f$ {\bf s} = (x, y) \f$ from cartesian coordinates. Same as build().
void vpFeatureVanishingPoint::set_xy(double x, double y)
{
  set_x(x);
  set_y(y);
  m_select = selectX() | selectY();
}

//! Set vanishing point feature \f$ 1/\rho \f$ value.
void vpFeatureVanishingPoint::setOneOverRho(double one_over_rho)
{
  s[2] = one_over_rho;
  flags[2] = true;
  m_select |= selectOneOverRho();
}

//! Set vanishing point feature \f$ \arctan(1/\rho) \f$ value.
void vpFeatureVanishingPoint::setAtanOneOverRho(double atan_one_over_rho)
{
  s[3] = atan_one_over_rho;
  flags[3] = true;
  m_select |= selectAtanOneOverRho();
}

//! Get vanishing point feature \f$ 1/\rho \f$ value.
double vpFeatureVanishingPoint::getOneOverRho() const { return s[2]; }

//! Get vanishing point feature \f$ \arctan(1/\rho) \f$ value.
double vpFeatureVanishingPoint::getAtanOneOverRho() const { return s[3]; }

//! Set vanishing point feature \f$ \alpha \f$ value.
void vpFeatureVanishingPoint::setAlpha(double alpha)
{
  s[4] = alpha;
  flags[4] = true;
  m_select |= selectAlpha();
}

//! Get vanishing point feature \f$ \alpha \f$ value.
double vpFeatureVanishingPoint::getAlpha() const { return s[4]; }

/*!
 * Compute the interaction matrix from a subset of the possible features.
 *
 * \param select : Feature selector. Value is either selectX() to select x visual feature, selectY()
 * for y visual feature, selectOneOverRho() for \f$ 1/\rho \f$, or selectAlpha() for
 * \f$ \alpha \f$ visual feature. You can also use a combination like selectX() | selectY() to
 * select x and y visual features.
 */
vpMatrix vpFeatureVanishingPoint::interaction(unsigned int select)
{
  vpMatrix L;

  L.resize(0, 6);

  if (deallocate == vpBasicFeature::user) {
    for (unsigned int i = 0; i < nbParameters; i++) {
      if (flags[i] == false) {
        switch (i) {
        case 0:
          vpTRACE("Warning !!!  The interaction matrix is computed but x was not set yet");
          break;
        case 1:
          vpTRACE("Warning !!!  The interaction matrix is computed but y was not set yet");
          break;
        case 2:
          vpTRACE("Warning !!! The interaction matrix is computed but 1/rho was not set yet");
          break;
        case 3:
          vpTRACE("Warning !!! The interaction matrix is computed but atan(1/rho) was not set yet");
          break;
        case 4:
          vpTRACE("Warning !!! The interaction matrix is computed but alpha was not set yet");
          break;
        default:
          vpTRACE("Problem during the reading of the variable flags");
        }
      }
    }
    resetFlags();
  }

  if (vpFeatureVanishingPoint::selectX() & select) {
    double x = get_x();
    double y = get_y();
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
    double x = get_x();
    double y = get_y();
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

  if (vpFeatureVanishingPoint::selectOneOverRho() & select) {
    double one_over_rho = getOneOverRho();
    double alpha = getAlpha();
    vpMatrix Lone_over_rho(1, 6);
    double rho2 = 1. + one_over_rho * one_over_rho;

    Lone_over_rho[0][0] = 0.;
    Lone_over_rho[0][1] = 0.;
    Lone_over_rho[0][2] = 0.;
    Lone_over_rho[0][3] = -rho2 * sin(alpha);
    Lone_over_rho[0][4] = rho2 * cos(alpha);
    Lone_over_rho[0][5] = 0.;

    L = vpMatrix::stack(L, Lone_over_rho);
  }

  if (vpFeatureVanishingPoint::selectAtanOneOverRho() & select) {
    double alpha = getAlpha();
    vpMatrix Latan_one_over_rho(1, 6);

    Latan_one_over_rho[0][0] = 0.;
    Latan_one_over_rho[0][1] = 0.;
    Latan_one_over_rho[0][2] = 0.;
    Latan_one_over_rho[0][3] = -sin(alpha);
    Latan_one_over_rho[0][4] = cos(alpha);
    Latan_one_over_rho[0][5] = 0.;

    L = vpMatrix::stack(L, Latan_one_over_rho);
  }

  if (vpFeatureVanishingPoint::selectAlpha() & select) {
    double one_over_rho = getOneOverRho();
    double alpha = getAlpha();
    vpMatrix Lalpha(1, 6);

    Lalpha[0][0] = 0;
    Lalpha[0][1] = 0.;
    Lalpha[0][2] = 0.;
    Lalpha[0][3] = cos(alpha) * one_over_rho;
    Lalpha[0][4] = sin(alpha) * one_over_rho;
    Lalpha[0][5] = -1.;

    L = vpMatrix::stack(L, Lalpha);
  }

  return L;
}

/*!
 * Compute the error between two visual features from a subset of the
 * possible features.
 *
 * \param s_star : Desired visual feature \f$ {\bf s}^* \f$.
 *
 * \param select : Feature selector. Value is either selectX() to select x visual feature, selectY()
 * for y visual feature, selectOneOverRho() for \f$ 1/\rho \f$, or selectAlpha() for
 * \f$ \alpha \f$ visual feature. You can also use a combination like selectX() | selectY() to
 * select x and y visual features.
 */
vpColVector vpFeatureVanishingPoint::error(const vpBasicFeature &s_star, unsigned int select)
{
  vpColVector e(0);

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

  if (vpFeatureVanishingPoint::selectOneOverRho() & select) {
    vpColVector e_one_over_rho(1);
    e_one_over_rho[0] = s[2] - s_star[2];

    e = vpColVector::stack(e, e_one_over_rho);
  }

  if (vpFeatureVanishingPoint::selectAtanOneOverRho() & select) {
    vpColVector e_atan_one_over_rho(1);
    e_atan_one_over_rho[0] = s[3] - s_star[3];

    e = vpColVector::stack(e, e_atan_one_over_rho);
  }

  if (vpFeatureVanishingPoint::selectAlpha() & select) {
    vpColVector e_alpha(1);
    double err = s[4] - s_star[4];

    if (err < -M_PI)
      err += 2 * M_PI;
    if (err > M_PI)
      err -= 2 * M_PI;

    e_alpha[0] = err;
    e = vpColVector::stack(e, e_alpha);
  }

  return e;
}

/*!
 * Print vanishing point features values to stdout.
 *
 * \param select : Use either selectX() to display x value, selectY() to display y value,
 * select selectOneOverRho() to display \f$ 1/\rho \f$ value, or selectAlpha() to display
 * \f$ \alpha \f$ value. You can also use a combination like selectX() | selectY() to
 * display x and y values.
 */
void vpFeatureVanishingPoint::print(unsigned int select) const
{
  std::cout << "Vanishing point:";
  if (vpFeatureVanishingPoint::selectX() & select)
    std::cout << " x=" << get_x();
  if (vpFeatureVanishingPoint::selectY() & select)
    std::cout << " y=" << get_y();
  if (vpFeatureVanishingPoint::selectOneOverRho() & select) {
    std::cout << " 1/rho=" << getOneOverRho();
  }
  if (vpFeatureVanishingPoint::selectAtanOneOverRho() & select) {
    std::cout << " atan(1/rho)=" << getAtanOneOverRho();
  }
  if (vpFeatureVanishingPoint::selectAlpha() & select) {
    std::cout << " alpha=" << getAlpha();
  }
  std::cout << std::endl;
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/**
 * \deprecated You should use build(const double&, const double &) instead.
 * \brief Set vanishing point visual feature \f$ {\bf s} = (x, y) \f$ from cartesian coordinates. Same as set_xy().
 */
void vpFeatureVanishingPoint::buildFrom(double x, double y) { set_xy(x, y); }
#endif

//! Set vanishing point visual feature \f$ {\bf s} = (x, y) \f$ from cartesian coordinates. Same as set_xy().
vpFeatureVanishingPoint &vpFeatureVanishingPoint::build(const double &x, const double &y)
{
  set_xy(x, y);
  return *this;
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
  if ((vpFeatureVanishingPoint::selectX() & m_select) || (vpFeatureVanishingPoint::selectY() & m_select)) {
    double x, y;
    x = get_x();
    y = get_y();

    vpFeatureDisplay::displayPoint(x, y, cam, I, color, thickness);
  }
  else if (vpFeatureVanishingPoint::selectOneOverRho() & m_select) {
    double one_over_rho = getOneOverRho();
    double alpha = getAlpha();
    double x = cos(alpha) / one_over_rho;
    double y = sin(alpha) / one_over_rho;
    vpFeatureDisplay::displayPoint(x, y, cam, I, color, thickness);
  }
  else if (vpFeatureVanishingPoint::selectAtanOneOverRho() & m_select) {
    double atan_one_over_rho = getAtanOneOverRho();
    double alpha = getAlpha();
    double x = cos(alpha) / tan(atan_one_over_rho);
    double y = sin(alpha) / tan(atan_one_over_rho);
    vpFeatureDisplay::displayPoint(x, y, cam, I, color, thickness);
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
  if ((vpFeatureVanishingPoint::selectX() & m_select) || (vpFeatureVanishingPoint::selectY() & m_select)) {
    double x, y;
    x = get_x();
    y = get_y();

    vpFeatureDisplay::displayPoint(x, y, cam, I, color, thickness);
  }
  else if (vpFeatureVanishingPoint::selectOneOverRho() & m_select) {
    double one_over_rho = getOneOverRho();
    double alpha = getAlpha();
    double x = cos(alpha) / one_over_rho;
    double y = sin(alpha) / one_over_rho;
    vpFeatureDisplay::displayPoint(x, y, cam, I, color, thickness);
  }
  else if (vpFeatureVanishingPoint::selectAtanOneOverRho() & m_select) {
    double atan_one_over_rho = getAtanOneOverRho();
    double alpha = getAlpha();
    double x = cos(alpha) / tan(atan_one_over_rho);
    double y = sin(alpha) / tan(atan_one_over_rho);
    vpFeatureDisplay::displayPoint(x, y, cam, I, color, thickness);
  }
}

/*!
 * Duplicate visual feature (used by the vpServo class only).
 */
vpFeatureVanishingPoint *vpFeatureVanishingPoint::duplicate() const
{
  vpFeatureVanishingPoint *feature = new vpFeatureVanishingPoint;
  return feature;
}

//! Select visual feature \f$ s = x \f$.
unsigned int vpFeatureVanishingPoint::selectX() { return FEATURE_LINE[0]; }

//! Select visual feature \f$ s = y \f$.
unsigned int vpFeatureVanishingPoint::selectY() { return FEATURE_LINE[1]; }

/*!
 * Select visual feature \f$ s = 1/\rho \f$.
 */
unsigned int vpFeatureVanishingPoint::selectOneOverRho() { return FEATURE_LINE[2]; }

/*!
 * Select visual feature \f$ s = \arctan(1/\rho) \f$.
 */
unsigned int vpFeatureVanishingPoint::selectAtanOneOverRho() { return FEATURE_LINE[3]; }

/*!
 * Select \f$ s = \theta \f$ visual feature.
 */
unsigned int vpFeatureVanishingPoint::selectAlpha() { return FEATURE_LINE[4]; }
END_VISP_NAMESPACE
