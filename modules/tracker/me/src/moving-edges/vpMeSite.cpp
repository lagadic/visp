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
 * Moving edges.
 *
*****************************************************************************/

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

#ifndef DOXYGEN_SHOULD_SKIP_THIS
static bool horsImage(int i, int j, int half, int rows, int cols)
{
  // return((i < half + 1) || ( i > (rows - half - 3) )||(j < half + 1) || (j
  // > (cols - half - 3) )) ;
  int half_1 = half + 1;
  int half_3 = half + 3;
  // return((i < half + 1) || ( i > (rows - half - 3) )||(j < half + 1) || (j
  // > (cols - half - 3) )) ;
  return ((0 < (half_1 - i)) || ((i - rows + half_3) > 0) || (0 < (half_1 - j)) || ((j - cols + half_3) > 0));
}
#endif

void vpMeSite::init()
{
  // Site components
  alpha = 0.0;
  convlt = 0.0;
  weight = -1;

  selectDisplay = NONE;

  // Pixel components
  i = 0;
  j = 0;
  v = 0;
  ifloat = i;
  jfloat = j;

  mask_sign = 1;

  normGradient = 0;

  state = NO_SUPPRESSION;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  suppress = 0;
#endif
}

vpMeSite::vpMeSite()
  : i(0), j(0), ifloat(0), jfloat(0), v(0), mask_sign(1), alpha(0.), convlt(0.), normGradient(0),
  weight(1), selectDisplay(NONE), state(NO_SUPPRESSION)
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  ,
  suppress(0)
#endif
{ }

vpMeSite::vpMeSite(double ip, double jp)
  : i(0), j(0), ifloat(0), jfloat(0), v(0), mask_sign(1), alpha(0.), convlt(0.), normGradient(0),
  weight(1), selectDisplay(NONE), state(NO_SUPPRESSION)
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  ,
  suppress(0)
#endif
{
  i = vpMath::round(ip);
  j = vpMath::round(jp);
  ifloat = ip;
  jfloat = jp;
}

/*!
  Copy constructor.
*/
vpMeSite::vpMeSite(const vpMeSite &mesite)
  : i(0), j(0), ifloat(0), jfloat(0), v(0), mask_sign(1), alpha(0.), convlt(0.), normGradient(0),
  weight(1), selectDisplay(NONE), state(NO_SUPPRESSION)
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  ,
  suppress(0)
#endif
{
  *this = mesite;
}

// More an Update than init
// For points in meter form (to avoid approximations)
void vpMeSite::init(double ip, double jp, double alphap)
{
  // Note: keep old value of convlt, suppress and contrast
  selectDisplay = NONE;

  ifloat = ip;
  i = vpMath::round(ip);
  jfloat = jp;
  j = vpMath::round(jp);
  alpha = alphap;
  mask_sign = 1;

  v = 0;
}

// initialise with convolution
void vpMeSite::init(double ip, double jp, double alphap, double convltp)
{
  selectDisplay = NONE;
  ifloat = ip;
  i = (int)ip;
  jfloat = jp;
  j = (int)jp;
  alpha = alphap;
  convlt = convltp;
  mask_sign = 1;

  v = 0;
}

// initialise with convolution and sign
void vpMeSite::init(double ip, double jp, double alphap, double convltp, int sign)
{
  selectDisplay = NONE;
  ifloat = ip;
  i = (int)ip;
  jfloat = jp;
  j = (int)jp;
  alpha = alphap;
  convlt = convltp;
  mask_sign = sign;

  v = 0;
}

vpMeSite &vpMeSite::operator=(const vpMeSite &m)
{
  i = m.i;
  j = m.j;
  ifloat = m.ifloat;
  jfloat = m.jfloat;
  v = m.v;
  mask_sign = m.mask_sign;
  alpha = m.alpha;
  convlt = m.convlt;
  normGradient = m.normGradient;
  weight = m.weight;
  selectDisplay = m.selectDisplay;
  state = m.state;

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  suppress = m.suppress;
#endif

  return *this;
}

/*!
 * Construct and return the list of vpMeSite along the normal to the contour,
 * in the given range.
 * \pre : ifloat, jfloat, and the direction of the normal (alpha) have to be set.
 * \param I : Image in which the display is performed.
 * \param range :  +/- the range within which the pixel's correspondent will be sought.
 * \return Pointer to the list of query sites
 */
vpMeSite *vpMeSite::getQueryList(const vpImage<unsigned char> &I, const int range)
{
  unsigned int range_ = static_cast<unsigned int>(range);
  // Size of query list includes the point on the line
  vpMeSite *list_query_pixels = new vpMeSite[2 * range_ + 1];

  // range : +/- the range within which the pixel's
  // correspondent will be sought

  double salpha = sin(alpha);
  double calpha = cos(alpha);
  int n = 0;
  vpImagePoint ip;

  for (int k = -range; k <= range; k++) {
    double ii = (ifloat + k * salpha);
    double jj = (jfloat + k * calpha);

    // Display
    if ((selectDisplay == RANGE_RESULT) || (selectDisplay == RANGE)) {
      ip.set_i(ii);
      ip.set_j(jj);
      vpDisplay::displayCross(I, ip, 1, vpColor::yellow);
    }

    // Copy parent's convolution
    vpMeSite pel;
    pel.init(ii, jj, alpha, convlt, mask_sign);
    pel.setDisplay(selectDisplay); // Display

    // Add site to the query list
    list_query_pixels[n] = pel;
    n++;
  }

  return (list_query_pixels);
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
 * Get the sign (according to the difference of values of the intensities of
 * the extremities).
 * \pre : ifloat, jfloat, and the direction of the normal (alpha) have to be set.
 * \param I : Image in which the sign is computed.
 * \param range :  +/- the range within which the pixel's correspondent is sought.
 * \post : mask_sign is computed
 */
void vpMeSite::getSign(const vpImage<unsigned char> &I, const int range)
{
  int k;

  double salpha = sin(alpha);
  double calpha = cos(alpha);

  // First extremity
  k = -range;
  unsigned int i1 = static_cast<unsigned int>(vpMath::round(ifloat + k * salpha));
  unsigned int j1 = static_cast<unsigned int>(vpMath::round(jfloat + k * calpha));

  // Second extremity
  k = range;
  unsigned int i2 = static_cast<unsigned int>(vpMath::round(ifloat + k * salpha));
  unsigned int j2 = static_cast<unsigned int>(vpMath::round(jfloat + k * calpha));

  // TODO: Here check if i1,j1,i2,j2 > 0 else ??
  if (I[i1][j1] > I[i2][j2])
    mask_sign = 1;
  else
    mask_sign = -1;
}
#endif

// Specific function for ME
double vpMeSite::convolution(const vpImage<unsigned char> &I, const vpMe *me)
{
  int half;
  int height_ = static_cast<int>(I.getHeight());
  int width_ = static_cast<int>(I.getWidth());

  double conv = 0.0;
  unsigned int msize = me->getMaskSize();
  half = (static_cast<int>(msize) - 1) >> 1;

  if (horsImage(i, j, half + me->getStrip(), height_, width_)) {
    conv = 0.0;
    i = 0;
    j = 0;
  }
  else {
    // Calculate tangent angle from normal
    double theta = alpha + M_PI / 2;
    // Move tangent angle to within 0->M_PI for a positive
    // mask index
    while (theta < 0)
      theta += M_PI;
    while (theta > M_PI)
      theta -= M_PI;

    // Convert radians to degrees
    int thetadeg = vpMath::round(theta * 180 / M_PI);

    if (abs(thetadeg) == 180) {
      thetadeg = 0;
    }

    unsigned int index_mask = (unsigned int)(thetadeg / (double)me->getAngleStep());

    unsigned int i_ = static_cast<unsigned int>(i);
    unsigned int j_ = static_cast<unsigned int>(j);
    unsigned int half_ = static_cast<unsigned int>(half);

    unsigned int ihalf = i_ - half_;
    unsigned int jhalf = j_ - half_;

    for (unsigned int a = 0; a < msize; a++) {
      unsigned int ihalfa = ihalf + a;
      for (unsigned int b = 0; b < msize; b++) {
        conv += mask_sign * me->getMask()[index_mask][a][b] * I(ihalfa, jhalf + b);
      }
    }
  }

  return (conv);
}

/*!

  Specific function for moving-edges.

  \warning To display the moving edges graphics a call to vpDisplay::flush() is needed after this function.

*/
void vpMeSite::track(const vpImage<unsigned char> &I, const vpMe *me, bool test_likelihood)
{
  int max_rank = -1;
  double max_convolution = 0;
  double max = 0;
  double contrast = 0;

  // range = +/- range of pixels within which the correspondent
  // of the current pixel will be sought
  unsigned int range = me->getRange();

  vpMeSite *list_query_pixels = getQueryList(I, (int)range);

  double contrast_max = 1 + me->getMu2();
  double contrast_min = 1 - me->getMu1();

  // array in which likelihood ratios will be stored
  double *likelihood = new double[2 * range + 1];

  double threshold;
  if (me->getLikelihoodThresholdType() == vpMe::NORMALIZED_THRESHOLD) {
    threshold = 2.0 * me->getThreshold();
  }
  else {
    double n_d = me->getMaskSize();
    threshold = me->getThreshold() / (100.0 * n_d * trunc(n_d / 2.0));
  }

  if (test_likelihood) {
    double diff = 1e6;
    for (unsigned int n = 0; n < 2 * range + 1; n++) {
      //   convolution results
      double convolution_ = list_query_pixels[n].convolution(I, me);

      // luminance ratio of reference pixel to potential correspondent pixel
      // the luminance must be similar, hence the ratio value should
      // lay between, for instance, 0.5 and 1.5 (parameter tolerance)
      likelihood[n] = fabs(convolution_ + convlt);
      if (likelihood[n] > threshold) {
        contrast = convolution_ / convlt;
        if ((contrast > contrast_min) && (contrast < contrast_max) && fabs(1 - contrast) < diff) {
          diff = fabs(1 - contrast);
          max_convolution = convolution_;
          max = likelihood[n];
          max_rank = (int)n;
        }
      }
    }
  }
  else { // test on contrast only
    for (unsigned int n = 0; n < 2 * range + 1; n++) {
      // convolution results
      double convolution_ = list_query_pixels[n].convolution(I, me);
      likelihood[n] = fabs(2 * convolution_);
      if (likelihood[n] > max && likelihood[n] > threshold) {
        max_convolution = convolution_;
        max = likelihood[n];
        max_rank = (int)n;
      }
    }
  }

  vpImagePoint ip;

  if (max_rank >= 0) {
    if ((selectDisplay == RANGE_RESULT) || (selectDisplay == RESULT)) {
      ip.set_i(list_query_pixels[max_rank].i);
      ip.set_j(list_query_pixels[max_rank].j);
      vpDisplay::displayPoint(I, ip, vpColor::red);
    }

    *this = list_query_pixels[max_rank]; // The vpMeSite2 is replaced by the
    // vpMeSite2 of max likelihood
    normGradient = vpMath::sqr(max_convolution);

    convlt = max_convolution;

    delete [] list_query_pixels;
    delete [] likelihood;
  }
  else // none of the query sites is better than the threshold
  {
    if ((selectDisplay == RANGE_RESULT) || (selectDisplay == RESULT)) {
      ip.set_i(list_query_pixels[0].i);
      ip.set_j(list_query_pixels[0].j);
      vpDisplay::displayPoint(I, ip, vpColor::green);
    }
    normGradient = 0;
    if (std::fabs(contrast) > std::numeric_limits<double>::epsilon())
      state = CONTRAST; // contrast suppression
    else
      state = THRESHOLD; // threshold suppression

    delete [] list_query_pixels;
    delete [] likelihood; // modif portage
  }
}

int vpMeSite::operator!=(const vpMeSite &m) { return ((m.i != i) || (m.j != j)); }

VISP_EXPORT std::ostream &operator<<(std::ostream &os, vpMeSite &vpMeS)
{
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  return (os << "Alpha: " << vpMeS.alpha << "  Convolution: " << vpMeS.convlt << "  Flag: " << vpMeS.suppress
    << "  Weight: " << vpMeS.weight);
#else
  return (os << "Alpha: " << vpMeS.alpha << "  Convolution: " << vpMeS.convlt << "  Weight: " << vpMeS.weight);
#endif
}

void vpMeSite::display(const vpImage<unsigned char> &I) { vpMeSite::display(I, ifloat, jfloat, state); }

void vpMeSite::display(const vpImage<vpRGBa> &I) { vpMeSite::display(I, ifloat, jfloat, state); }

// Static functions

/*!
  Display the moving edge site with a color corresponding to their state.

  - If green : The vpMeSite is a good point.
  - If blue : The point is removed because of the vpMeSite tracking phase (contrast problem).
  - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
  - If red : The point is removed because of the robust method in the virtual visual servoing (M-Estimator problem).
  - If cyan : The point is removed because it's too close to another.
  - Yellow otherwise.

  \param I : The image.
  \param i : Pixel i of the site.
  \param j : Pixel j of the site.
  \param state : State of the site.
*/
void vpMeSite::display(const vpImage<unsigned char> &I, const double &i, const double &j, const vpMeSiteState &state)
{
  switch (state) {
  case NO_SUPPRESSION:
    vpDisplay::displayCross(I, vpImagePoint(i, j), 3, vpColor::green, 1);
    break;

  case CONTRAST:
    vpDisplay::displayCross(I, vpImagePoint(i, j), 3, vpColor::blue, 1);
    break;

  case THRESHOLD:
    vpDisplay::displayCross(I, vpImagePoint(i, j), 3, vpColor::purple, 1);
    break;

  case M_ESTIMATOR:
    vpDisplay::displayCross(I, vpImagePoint(i, j), 3, vpColor::red, 1);
    break;

  case TOO_NEAR:
    vpDisplay::displayCross(I, vpImagePoint(i, j), 3, vpColor::cyan, 1);
    break;

  default:
    vpDisplay::displayCross(I, vpImagePoint(i, j), 3, vpColor::yellow, 1);
  }
}

/*!
  Display the moving edge site with a color corresponding to their state.

  - If green : The vpMeSite is a good point.
  - If blue : The point is removed because of the vpMeSite tracking phase (contrast problem).
  - If purple : The point is removed because of the vpMeSite tracking phase (threshold problem).
  - If red : The point is removed because of the robust method in the virtual visual servoing (M-Estimator problem).
  - If cyan : The point is removed because it's too close to another.
  - Yellow otherwise

  \param I : The image.
  \param i : Pixel i of the site.
  \param j : Pixel j of the site.
  \param state : State of the site.
*/
void vpMeSite::display(const vpImage<vpRGBa> &I, const double &i, const double &j, const vpMeSiteState &state)
{
  switch (state) {
  case NO_SUPPRESSION:
    vpDisplay::displayCross(I, vpImagePoint(i, j), 3, vpColor::green, 1);
    break;

  case CONTRAST:
    vpDisplay::displayCross(I, vpImagePoint(i, j), 3, vpColor::blue, 1);
    break;

  case THRESHOLD:
    vpDisplay::displayCross(I, vpImagePoint(i, j), 3, vpColor::purple, 1);
    break;

  case M_ESTIMATOR:
    vpDisplay::displayCross(I, vpImagePoint(i, j), 3, vpColor::red, 1);
    break;

  case TOO_NEAR:
    vpDisplay::displayCross(I, vpImagePoint(i, j), 3, vpColor::cyan, 1);
    break;

  default:
    vpDisplay::displayCross(I, vpImagePoint(i, j), 3, vpColor::yellow, 1);
  }
}
