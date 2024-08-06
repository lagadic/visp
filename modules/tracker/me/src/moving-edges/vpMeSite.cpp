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
 * Moving edges.
 */

/*!
  \file vpMeSite.cpp
  \brief Moving edges
*/

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <stdlib.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/me/vpMe.h>
#include <visp3/me/vpMeSite.h>

BEGIN_VISP_NAMESPACE

#ifndef DOXYGEN_SHOULD_SKIP_THIS
static bool horsImage(int i, int j, int half, int rows, int cols)
{
  int half_1 = half + 1;
  int half_3 = half + 3;
  return ((0 < (half_1 - i)) || (((i - rows) + half_3) > 0) || (0 < (half_1 - j)) || (((j - cols) + half_3) > 0));
}
#endif

void vpMeSite::init()
{
  // Site components
  m_alpha = 0.0;
  m_convlt = 0.0;
  m_weight = -1;
  m_contrastThreshold = 10000.0;

  m_selectDisplay = NONE;

  // Pixel components
  m_i = 0;
  m_j = 0;
  m_ifloat = m_i;
  m_jfloat = m_j;

  m_mask_sign = 1;

  m_normGradient = 0;

  m_state = NO_SUPPRESSION;
}

vpMeSite::vpMeSite()
  : m_i(0), m_j(0), m_ifloat(0), m_jfloat(0), m_mask_sign(1), m_alpha(0.), m_convlt(0.), m_normGradient(0),
  m_weight(1), m_contrastThreshold(10000.0), m_selectDisplay(NONE), m_state(NO_SUPPRESSION)
{ }

vpMeSite::vpMeSite(const double &ip, const double &jp)
  : m_i(0), m_j(0), m_ifloat(0), m_jfloat(0), m_mask_sign(1), m_alpha(0.), m_convlt(0.), m_normGradient(0),
  m_weight(1), m_contrastThreshold(10000.0), m_selectDisplay(NONE), m_state(NO_SUPPRESSION)
{
  m_i = vpMath::round(ip);
  m_j = vpMath::round(jp);
  m_ifloat = ip;
  m_jfloat = jp;
}

vpMeSite::vpMeSite(const vpMeSite &mesite)
  : m_i(0), m_j(0), m_ifloat(0), m_jfloat(0), m_mask_sign(1), m_alpha(0.), m_convlt(0.), m_normGradient(0),
  m_weight(1), m_contrastThreshold(10000.0), m_selectDisplay(NONE), m_state(NO_SUPPRESSION)
{
  *this = mesite;
}

// More an Update than init
// For points in meter form (to avoid approximations)
void vpMeSite::init(const double &ip, const double &jp, const double &alphap)
{
  // Note: keep old value of m_convlt, contrast and threshold
  m_selectDisplay = NONE;

  m_ifloat = ip;
  m_i = vpMath::round(ip);
  m_jfloat = jp;
  m_j = vpMath::round(jp);
  m_alpha = alphap;
  m_mask_sign = 1;
}

// initialise with convolution()
void vpMeSite::init(const double &ip, const double &jp, const double &alphap, const double &convltp)
{
  m_selectDisplay = NONE;
  m_ifloat = ip;
  m_i = static_cast<int>(ip);
  m_jfloat = jp;
  m_j = static_cast<int>(jp);
  m_alpha = alphap;
  m_convlt = convltp;
  m_mask_sign = 1;
}

// initialise with convolution and sign
void vpMeSite::init(const double &ip, const double &jp, const double &alphap, const double &convltp, const int &sign)
{
  m_selectDisplay = NONE;
  m_ifloat = ip;
  m_i = static_cast<int>(ip);
  m_jfloat = jp;
  m_j = static_cast<int>(jp);
  m_alpha = alphap;
  m_convlt = convltp;
  m_mask_sign = sign;
}

// initialise with convolution and sign
void vpMeSite::init(const double &ip, const double &jp, const double &alphap, const double &convltp, const int &sign, const double &contrastThreshold)
{
  m_selectDisplay = NONE;
  m_ifloat = ip;
  m_i = static_cast<int>(ip);
  m_jfloat = jp;
  m_j = static_cast<int>(jp);
  m_alpha = alphap;
  m_convlt = convltp;
  m_mask_sign = sign;
  m_contrastThreshold = contrastThreshold;
}

vpMeSite &vpMeSite::operator=(const vpMeSite &m)
{
  m_i = m.m_i;
  m_j = m.m_j;
  m_ifloat = m.m_ifloat;
  m_jfloat = m.m_jfloat;
  m_mask_sign = m.m_mask_sign;
  m_alpha = m.m_alpha;
  m_convlt = m.m_convlt;
  m_normGradient = m.m_normGradient;
  m_weight = m.m_weight;
  m_contrastThreshold = m.m_contrastThreshold;
  m_selectDisplay = m.m_selectDisplay;
  m_state = m.m_state;

  return *this;
}

vpMeSite *vpMeSite::getQueryList(const vpImage<unsigned char> &I, const int &range) const
{
  unsigned int range_ = static_cast<unsigned int>(range);
  // Size of query list includes the point on the line
  vpMeSite *list_query_pixels = new vpMeSite[(2 * range_) + 1];

  // range : +/- the range within which the pixel's
  // correspondent will be sought

  double salpha = sin(m_alpha);
  double calpha = cos(m_alpha);
  int n = 0;
  vpImagePoint ip;

  for (int k = -range; k <= range; ++k) {
    double ii = m_ifloat + (k * salpha);
    double jj = m_jfloat + (k * calpha);

    // Display
    if ((m_selectDisplay == RANGE_RESULT) || (m_selectDisplay == RANGE)) {
      ip.set_i(ii);
      ip.set_j(jj);
      vpDisplay::displayCross(I, ip, 1, vpColor::yellow);
    }

    // Copy parent's convolution
    vpMeSite pel;
    pel.init(ii, jj, m_alpha, m_convlt, m_mask_sign, m_contrastThreshold);
    pel.setDisplay(m_selectDisplay); // Display

    // Add site to the query list
    list_query_pixels[n] = pel;
    ++n;
  }

  return list_query_pixels;
}

// Specific function for ME
double vpMeSite::convolution(const vpImage<unsigned char> &I, const vpMe *me)
{
  int half;
  int height_ = static_cast<int>(I.getHeight());
  int width_ = static_cast<int>(I.getWidth());

  double conv = 0.0;
  unsigned int msize = me->getMaskSize();
  half = static_cast<int>((msize - 1) >> 1);

  if (horsImage(m_i, m_j, half + me->getStrip(), height_, width_)) {
    conv = 0.0;
    m_i = 0;
    m_j = 0;
  }
  else {
    // Calculate tangent angle from normal
    double theta = m_alpha + (M_PI / 2);
    // Move tangent angle to within 0->M_PI for a positive
    // mask index
    while (theta < 0) {
      theta += M_PI;
    }
    while (theta > M_PI) {
      theta -= M_PI;
    }

    // Convert radians to degrees
    int thetadeg = vpMath::round((theta * 180) / M_PI);

    const int flatAngle = 180;
    if (abs(thetadeg) == flatAngle) {
      thetadeg = 0;
    }

    unsigned int index_mask = static_cast<unsigned int>(thetadeg / static_cast<double>(me->getAngleStep()));

    unsigned int i_ = static_cast<unsigned int>(m_i);
    unsigned int j_ = static_cast<unsigned int>(m_j);
    unsigned int half_ = static_cast<unsigned int>(half);

    unsigned int ihalf = i_ - half_;
    unsigned int jhalf = j_ - half_;

    for (unsigned int a = 0; a < msize; ++a) {
      unsigned int ihalfa = ihalf + a;
      for (unsigned int b = 0; b < msize; ++b) {
        conv += m_mask_sign * me->getMask()[index_mask][a][b] * I(ihalfa, jhalf + b);
      }
    }
  }

  return conv;
}

void vpMeSite::track(const vpImage<unsigned char> &I, const vpMe *me, const bool &test_contrast)
{
  int max_rank = -1;
  double max_convolution = 0;
  double max = 0;
  double contrast = 0;

  // range = +/- range of pixels within which the correspondent
  // of the current pixel will be sought
  unsigned int range = me->getRange();
  const unsigned int normalSides = 2;
  const unsigned int numQueries = range * normalSides + 1;

  vpMeSite *list_query_pixels = getQueryList(I, static_cast<int>(range));

  double contrast_max = 1 + me->getMu2();
  double contrast_min = 1 - me->getMu1();

  double threshold = computeFinalThreshold(*me);

  if (test_contrast) {
    double diff = 1e6;
    for (unsigned int n = 0; n < numQueries; ++n) {
      // convolution results

      double convolution_ = list_query_pixels[n].convolution(I, me);
      // luminance ratio of reference pixel to potential correspondent pixel
      // the luminance must be similar, hence the ratio value should
      // lay between, for instance, 0.5 and 1.5 (parameter tolerance)
      const double likelihood = fabs(convolution_ + m_convlt);

      if (likelihood > threshold) {
        contrast = convolution_ / m_convlt;
        if ((contrast > contrast_min) && (contrast < contrast_max) && (fabs(1 - contrast) < diff)) {
          diff = fabs(1 - contrast);
          max_convolution = convolution_;
          max = likelihood;
          max_rank = static_cast<int>(n);
        }
      }
    }
  }
  else { // test on contrast only
    for (unsigned int n = 0; n < numQueries; ++n) {

      // convolution results
      double convolution_ = list_query_pixels[n].convolution(I, me);
      const double likelihood = fabs(2 * convolution_);
      if ((likelihood > max) && (likelihood > threshold)) {
        max_convolution = convolution_;
        max = likelihood;
        max_rank = static_cast<int>(n);
      }
    }
  }

  vpImagePoint ip;

  if (max_rank >= 0) {
    if ((m_selectDisplay == RANGE_RESULT) || (m_selectDisplay == RESULT)) {
      ip.set_i(list_query_pixels[max_rank].m_i);
      ip.set_j(list_query_pixels[max_rank].m_j);
      vpDisplay::displayPoint(I, ip, vpColor::red);
    }

    *this = list_query_pixels[max_rank]; // The vpMeSite2 is replaced by the
    // vpMeSite2 of max likelihood
    m_normGradient = vpMath::sqr(max_convolution);

    m_convlt = max_convolution;
  }
  else // none of the query sites is better than the threshold
  {
    if ((m_selectDisplay == RANGE_RESULT) || (m_selectDisplay == RESULT)) {
      ip.set_i(list_query_pixels[0].m_i);
      ip.set_j(list_query_pixels[0].m_j);
      vpDisplay::displayPoint(I, ip, vpColor::green);
    }
    m_normGradient = 0;
    if (std::fabs(contrast) > std::numeric_limits<double>::epsilon()) {
      m_state = CONTRAST; // contrast suppression
    }
    else {
      m_state = THRESHOLD; // threshold suppression
    }

  }
  delete[] list_query_pixels;
}

void vpMeSite::trackMultipleHypotheses(const vpImage<unsigned char> &I, const vpMe &me, const bool &test_contrast,
std::vector<vpMeSite> &outputHypotheses, const unsigned numCandidates)
{

  // range = +/- range of pixels within which the correspondent
  // of the current pixel will be sought
  unsigned int range = me.getRange();
  const unsigned int numQueries = range * 2 + 1;

  if (numCandidates > numQueries) {
    throw vpException(vpException::badValue, "Error in vpMeSite::track: the number of retained hypotheses cannot superior to the number of queried sites.");
  }

  vpMeSite *list_query_pixels = getQueryList(I, static_cast<int>(range));


  // Insert into a map, where the key is the sorting criterion (negative likelihood or contrast diff)
  // and the key is the ME site + its computed likelihood and contrast.
  // After computation: iterating on the map is guaranteed to be done with the keys being sorted according to the criterion.
  // Multimap allows to have multiple values (sites) with the same key (likelihood/contrast diff)
  // Only the candidates that are above the threshold are kept
  std::multimap<double, std::tuple<vpMeSite *, double, double>> candidates;

  const double contrast_max = 1 + me.getMu2();
  const double contrast_min = 1 - me.getMu1();

  const double threshold = computeFinalThreshold(me);

  // First step: compute likelihoods and contrasts for all queries
  if (test_contrast) {
    for (unsigned int n = 0; n < numQueries; ++n) {
      vpMeSite &query = list_query_pixels[n];
      //   convolution results
      const double convolution_ = query.convolution(I, &me);
      // luminance ratio of reference pixel to potential correspondent pixel
      // the luminance must be similar, hence the ratio value should
      // lay between, for instance, 0.5 and 1.5 (parameter tolerance)
      const double likelihood = fabs(convolution_ + m_convlt);

      query.m_convlt = convolution_;
      const double contrast = convolution_ / m_convlt;
      candidates.insert({ fabs(1.0 - contrast), {&query, likelihood, contrast} });
    }
  }
  else { // test on likelihood only
    for (unsigned int n = 0; n < numQueries; ++n) {
      // convolution results
      vpMeSite &query = list_query_pixels[n];
      const double convolution_ = query.convolution(I, &me);
      const double likelihood = fabs(2 * convolution_);
      query.m_convlt = convolution_;
      candidates.insert({ -likelihood, {&query, likelihood, 0.0} });
    }
  }
  // Take first numCandidates hypotheses: map is sorted according to the likelihood/contrast difference so we can just
  // iterate from the start
  outputHypotheses.resize(numCandidates);
  std::multimap<double, std::tuple<vpMeSite *, double, double>>::iterator it = candidates.begin();
  if (test_contrast) {
    for (unsigned int i = 0; i < numCandidates; ++i, ++it) {
      outputHypotheses[i] = *(std::get<0>(it->second));
      outputHypotheses[i].m_normGradient = vpMath::sqr(outputHypotheses[i].m_convlt);
      const double likelihood = std::get<1>(it->second);
      const double contrast = std::get<2>(it->second);

      if (likelihood > threshold) {
        if (contrast <= contrast_min || contrast >= contrast_max) {
          outputHypotheses[i].m_state = vpMeSiteState::CONTRAST;
        }
        else {
          outputHypotheses[i].m_state = vpMeSiteState::NO_SUPPRESSION;
        }
      }
      else {
        outputHypotheses[i].m_state = vpMeSiteState::THRESHOLD;
      }
    }
  }
  else {
    for (unsigned int i = 0; i < numCandidates; ++i, ++it) {
      outputHypotheses[i] = *(std::get<0>(it->second));
      const double likelihood = std::get<1>(it->second);
      if (likelihood > threshold) {
        outputHypotheses[i].m_state = vpMeSiteState::NO_SUPPRESSION;
      }
      else {
        outputHypotheses[i].m_state = vpMeSiteState::THRESHOLD;
      }
    }
  }

  const vpMeSite &bestMatch = outputHypotheses[0];


  if (bestMatch.m_state != NO_SUPPRESSION) {
    if ((m_selectDisplay == RANGE_RESULT) || (m_selectDisplay == RESULT)) {

      vpDisplay::displayPoint(I, bestMatch.m_i, bestMatch.m_j, vpColor::red);
    }
    *this = outputHypotheses[0];
  }
  else {
    if ((m_selectDisplay == RANGE_RESULT) || (m_selectDisplay == RESULT)) {
      vpDisplay::displayPoint(I, bestMatch.m_i, bestMatch.m_j, vpColor::green);
    }
    m_normGradient = 0;
  }

  delete[] list_query_pixels;
}

int vpMeSite::operator!=(const vpMeSite &m) { return ((m.m_i != m_i) || (m.m_j != m_j)); }


void vpMeSite::display(const vpImage<unsigned char> &I) const { vpMeSite::display(I, m_ifloat, m_jfloat, m_state); }

void vpMeSite::display(const vpImage<vpRGBa> &I) const { vpMeSite::display(I, m_ifloat, m_jfloat, m_state); }

// Static functions

void vpMeSite::display(const vpImage<unsigned char> &I, const double &i, const double &j, const vpMeSiteState &state)
{
  const unsigned int crossSize = 3;
  switch (state) {
  case NO_SUPPRESSION:
    vpDisplay::displayCross(I, vpImagePoint(i, j), crossSize, vpColor::green, 1);
    break;

  case CONTRAST:
    vpDisplay::displayCross(I, vpImagePoint(i, j), crossSize, vpColor::blue, 1);
    break;

  case THRESHOLD:
    vpDisplay::displayCross(I, vpImagePoint(i, j), crossSize, vpColor::purple, 1);
    break;

  case M_ESTIMATOR:
    vpDisplay::displayCross(I, vpImagePoint(i, j), crossSize, vpColor::red, 1);
    break;

  case OUTSIDE_ROI_MASK:
    vpDisplay::displayCross(I, vpImagePoint(i, j), crossSize, vpColor::cyan, 1);
    break;

  default:
    vpDisplay::displayCross(I, vpImagePoint(i, j), crossSize, vpColor::yellow, 1);
  }
}

void vpMeSite::display(const vpImage<vpRGBa> &I, const double &i, const double &j, const vpMeSiteState &state)
{
  const unsigned int cross_size = 3;
  switch (state) {
  case NO_SUPPRESSION:
    vpDisplay::displayCross(I, vpImagePoint(i, j), cross_size, vpColor::green, 1);
    break;

  case CONTRAST:
    vpDisplay::displayCross(I, vpImagePoint(i, j), cross_size, vpColor::blue, 1);
    break;

  case THRESHOLD:
    vpDisplay::displayCross(I, vpImagePoint(i, j), cross_size, vpColor::purple, 1);
    break;

  case M_ESTIMATOR:
    vpDisplay::displayCross(I, vpImagePoint(i, j), cross_size, vpColor::red, 1);
    break;

  case OUTSIDE_ROI_MASK:
    vpDisplay::displayCross(I, vpImagePoint(i, j), cross_size, vpColor::cyan, 1);
    break;

  default:
    vpDisplay::displayCross(I, vpImagePoint(i, j), cross_size, vpColor::yellow, 1);
  }
}

VISP_EXPORT std::ostream &operator<<(std::ostream &os, vpMeSite &vpMeS)
{
  return (os << "Alpha: " << vpMeS.m_alpha << "  Convolution: " << vpMeS.m_convlt << "  Weight: " << vpMeS.m_weight << "  Threshold: " << vpMeS.m_contrastThreshold);
}

END_VISP_NAMESPACE
