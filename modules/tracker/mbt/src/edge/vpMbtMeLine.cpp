/*
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
 * Make the complete tracking of an object by using its CAD model
 */

#include <visp3/core/vpConfig.h>
#ifndef DOXYGEN_SHOULD_SKIP_THIS

/*!
 \file vpMbtMeLine.cpp
 \brief Make the complete tracking of an object by using its CAD model.
*/
#include <algorithm> // (std::min)
#include <cmath>     // std::fabs
#include <limits>    // numeric_limits

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpRobust.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/mbt/vpMbtMeLine.h>

BEGIN_VISP_NAMESPACE
//! Normalize an angle between -Pi and Pi
static void normalizeAngle(double &delta)
{
  while (delta > M_PI) {
    delta -= M_PI;
  }
  while (delta < -M_PI) {
    delta += M_PI;
  }
}

/*!
  Basic constructor that calls the constructor of the class vpMeTracker.
*/
vpMbtMeLine::vpMbtMeLine()
  : m_rho(0.), m_theta(0.), m_theta_1(M_PI / 2), m_delta(0.), m_delta_1(0), m_sign(1), m_a(0.), m_b(0.), m_c(0.),
  imin(0), imax(0), jmin(0), jmax(0), expecteddensity(0.)
{ }

/*!
  Basic destructor.
*/
vpMbtMeLine::~vpMbtMeLine()
{
  m_meList.clear();
}

/*!
  Initialization of the tracking. The line is defined thanks to the
  coordinates of two points corresponding to the extremities and its (\f$\rho
  \: \theta\f$) parameters.

  Remember the equation of a line : \f$ i \; cos(\theta) + j \; sin(\theta) -
  \rho = 0 \f$

  \param I : Image in which the line appears.
  \param ip1 : Coordinates of the first point.
  \param ip2 : Coordinates of the second point.
  \param rho : The \f$\rho\f$ parameter
  \param theta : The \f$\theta\f$ parameter
  \param doNoTrack : If true, ME are not tracked
*/
void vpMbtMeLine::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                               double rho, double theta, bool doNoTrack)
{
  vpCDEBUG(1) << " begin vpMeLine::initTracking()" << std::endl;

  try {
    //  1. On fait ce qui concerne les droites (peut etre vide)
    // Points extremites
    m_PExt[0].m_ifloat = (float)ip1.get_i();
    m_PExt[0].m_jfloat = (float)ip1.get_j();
    m_PExt[1].m_ifloat = (float)ip2.get_i();
    m_PExt[1].m_jfloat = (float)ip2.get_j();

    m_rho = rho;
    m_theta = theta;
    m_theta_1 = theta;

    m_a = cos(m_theta);
    m_b = sin(m_theta);
    m_c = -m_rho;

    m_delta = -m_theta + M_PI / 2.0;
    normalizeAngle(m_delta);
    m_delta_1 = m_delta;

    sample(I, doNoTrack);
    expecteddensity = (double)m_meList.size();

    if (!doNoTrack)
      vpMeTracker::track(I);
  }
  catch (...) {
    throw; // throw the original exception
  }
  vpCDEBUG(1) << " end vpMeLine::initTracking()" << std::endl;
}

/*!
  Construct a list of vpMeSite moving edges at a particular sampling
  step between the two extremities of the line.

  \param I : Image in which the line appears.
  \param doNoTrack : If true, ME are not tracked.
*/
void vpMbtMeLine::sample(const vpImage<unsigned char> &I, bool doNoTrack)
{
  int nbrows = static_cast<int>(I.getHeight());
  int nbcols = static_cast<int>(I.getWidth());
  double n_sample;

  if (std::fabs(m_me->getSampleStep()) <= std::numeric_limits<double>::epsilon()) {
    throw(vpTrackingException(vpTrackingException::fatalError, "Function vpMbtMeLine::sample() called with "
                              "moving-edges sample step = 0"));
  }

  // i, j portions of the line_p
  double diffsi = m_PExt[0].m_ifloat - m_PExt[1].m_ifloat;
  double diffsj = m_PExt[0].m_jfloat - m_PExt[1].m_jfloat;

  double length_p = sqrt((vpMath::sqr(diffsi) + vpMath::sqr(diffsj)));

  // number of samples along line_p
  n_sample = length_p / (double)m_me->getSampleStep();

  double stepi = diffsi / (double)n_sample;
  double stepj = diffsj / (double)n_sample;

  // Choose starting point
  double is = m_PExt[1].m_ifloat;
  double js = m_PExt[1].m_jfloat;

  // Delete old list
  m_meList.clear();

  // sample positions at i*m_me->getSampleStep() interval along the
  // line_p, starting at PSiteExt[0]

  for (int i = 0; i <= vpMath::round(n_sample); i++) {
    vpImagePoint iP;
    iP.set_i(is);
    iP.set_j(js);
    unsigned int is_uint = static_cast<unsigned int>(is);
    unsigned int js_uint = static_cast<unsigned int>(js);
    // If point is in the image, add to the sample list
    if ((!outOfImage(iP, (int)(m_me->getRange() + m_me->getMaskSize() + 1), nbrows, nbcols)) && inRoiMask(m_mask, is_uint, js_uint)
        && inMeMaskCandidates(m_maskCandidates, is_uint, js_uint)) {
      vpMeSite pix;
      pix.init(iP.get_i(), iP.get_j(), m_delta, 0, m_sign);
      pix.setDisplay(m_selectDisplay);
      pix.setState(vpMeSite::NO_SUPPRESSION);
      const double marginRatio = m_me->getThresholdMarginRatio();
      double convolution = pix.convolution(I, m_me);
      double contrastThreshold = fabs(convolution) * marginRatio;
      pix.setContrastThreshold(contrastThreshold, *m_me);

      if (!doNoTrack) {
        pix.track(I, m_me, false);
      }

      if (vpDEBUG_ENABLE(3)) {
        vpDisplay::displayCross(I, iP, 2, vpColor::blue);
      }

      m_meList.push_back(pix);
    }
    is += stepi;
    js += stepj;
  }

  vpCDEBUG(1) << "end vpMeLine::sample() : ";
  vpCDEBUG(1) << m_meList.size() << " point inserted in the list " << std::endl;
}

/*!
  Suppress the moving which belong no more to the line.

  \param I : The image.
*/
void vpMbtMeLine::suppressPoints(const vpImage<unsigned char> &I)
{
  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != m_meList.end();) {
    vpMeSite s = *it; // current reference pixel

    if (fabs(sin(m_theta)) > 0.9) // Vertical line management
    {
      if ((s.m_i < imin) || (s.m_i > imax)) {
        s.setState(vpMeSite::CONTRAST);
      }
    }

    else if (fabs(cos(m_theta)) > 0.9) // Horizontal line management
    {
      if ((s.m_j < jmin) || (s.m_j > jmax)) {
        s.setState(vpMeSite::CONTRAST);
      }
    }

    else {
      if ((s.m_i < imin) || (s.m_i > imax) || (s.m_j < jmin) || (s.m_j > jmax)) {
        s.setState(vpMeSite::CONTRAST);
      }
    }

    if (outOfImage(s.m_i, s.m_j, (int)(m_me->getRange() + m_me->getMaskSize() + 1), (int)I.getHeight(), (int)I.getWidth())) {
      s.setState(vpMeSite::TOO_NEAR);
    }

    if (s.getState() != vpMeSite::NO_SUPPRESSION)
      it = m_meList.erase(it);
    else
      ++it;
  }
}

/*!
 Seek along the line defined by its equation, the two extremities of the line.
 This function is useful in case of translation of the line.

 \param I : Image in which the line appears.
*/
void vpMbtMeLine::seekExtremities(const vpImage<unsigned char> &I)
{
  vpCDEBUG(1) << "begin vpMeLine::sample() : " << std::endl;

  int rows = (int)I.getHeight();
  int cols = (int)I.getWidth();
  double n_sample;

  // if (m_me->getSampleStep()==0)
  if (std::fabs(m_me->getSampleStep()) <= std::numeric_limits<double>::epsilon()) {
    throw(vpTrackingException(vpTrackingException::fatalError, "Function called with sample step = 0"));
  }

  // i, j portions of the line_p
  double diffsi = m_PExt[0].m_ifloat - m_PExt[1].m_ifloat;
  double diffsj = m_PExt[0].m_jfloat - m_PExt[1].m_jfloat;

  double s = vpMath::sqr(diffsi) + vpMath::sqr(diffsj);

  double di = diffsi / sqrt(s); // pas de risque de /0 car d(P1,P2) >0
  double dj = diffsj / sqrt(s);

  double length_p = sqrt(s); /*(vpMath::sqr(diffsi)+vpMath::sqr(diffsj))*/

  // number of samples along line_p
  n_sample = length_p / (double)m_me->getSampleStep();
  double sample_step = (double)m_me->getSampleStep();

  vpMeSite P;
  P.init((int)m_PExt[0].m_ifloat, (int)m_PExt[0].m_jfloat, m_delta_1, 0, m_sign);
  P.setDisplay(m_selectDisplay);

  unsigned int memory_range = m_me->getRange();
  m_me->setRange(1);

  for (int i = 0; i < 3; i++) {
    P.m_ifloat = P.m_ifloat + di * sample_step;
    P.m_i = (int)P.m_ifloat;
    P.m_jfloat = P.m_jfloat + dj * sample_step;
    P.m_j = (int)P.m_jfloat;

    if ((P.m_i < imin) || (P.m_i > imax) || (P.m_j < jmin) || (P.m_j > jmax)) {
      if (vpDEBUG_ENABLE(3))
        vpDisplay::displayCross(I, P.m_i, P.m_j, 5, vpColor::cyan);
    }
    else if (!outOfImage(P.m_i, P.m_j, (int)(m_me->getRange() + m_me->getMaskSize() + 1), (int)rows, (int)cols)) {
      P.track(I, m_me, false);

      if (P.getState() == vpMeSite::NO_SUPPRESSION) {
        m_meList.push_back(P);
        if (vpDEBUG_ENABLE(3))
          vpDisplay::displayCross(I, P.m_i, P.m_j, 5, vpColor::green);
      }
      else if (vpDEBUG_ENABLE(3))
        vpDisplay::displayCross(I, P.m_i, P.m_j, 10, vpColor::blue);
    }
  }

  P.init((int)m_PExt[1].m_ifloat, (int)m_PExt[1].m_jfloat, m_delta_1, 0, m_sign);
  P.setDisplay(m_selectDisplay);
  for (int i = 0; i < 3; i++) {
    P.m_ifloat = P.m_ifloat - di * sample_step;
    P.m_i = (int)P.m_ifloat;
    P.m_jfloat = P.m_jfloat - dj * sample_step;
    P.m_j = (int)P.m_jfloat;

    if ((P.m_i < imin) || (P.m_i > imax) || (P.m_j < jmin) || (P.m_j > jmax)) {
      if (vpDEBUG_ENABLE(3))
        vpDisplay::displayCross(I, P.m_i, P.m_j, 5, vpColor::cyan);
    }

    else if (!outOfImage(P.m_i, P.m_j, (int)(m_me->getRange() + m_me->getMaskSize() + 1), (int)rows, (int)cols)) {
      P.track(I, m_me, false);

      if (P.getState() == vpMeSite::NO_SUPPRESSION) {
        m_meList.push_back(P);
        if (vpDEBUG_ENABLE(3))
          vpDisplay::displayCross(I, P.m_i, P.m_j, 5, vpColor::green);
      }
      else if (vpDEBUG_ENABLE(3))
        vpDisplay::displayCross(I, P.m_i, P.m_j, 10, vpColor::blue);
    }
  }

  m_me->setRange(memory_range);

  vpCDEBUG(1) << "end vpMeLine::sample() : ";
  vpCDEBUG(1) << n_sample << " point inserted in the list " << std::endl;
}

/*!
  Compute the projection error of the line. Compare the gradient direction
  around samples of the line to its direction. Error is expressed in radians
  between 0 and M_PI/2.0;

  \param _I : Image in which the line appears.
  \param _sumErrorRad : sum of the error per feature.
  \param _nbFeatures : Number of features used to compute _sumErrorRad.
  \param SobelX : Sobel kernel in X-direction.
  \param SobelY : Sobel kernel in Y-direction.
  \param display : If true, display gradient and model orientation.
  \param length : Length of arrows used to show gradient and model orientation.
  \param thickness : Thickness of arrows used to show gradient and model orientation.
*/
void vpMbtMeLine::computeProjectionError(const vpImage<unsigned char> &_I, double &_sumErrorRad,
                                         unsigned int &_nbFeatures, const vpMatrix &SobelX, const vpMatrix &SobelY,
                                         bool display, unsigned int length, unsigned int thickness)
{
  _sumErrorRad = 0;
  _nbFeatures = 0;
  double deltaNormalized = m_theta;
  unsigned int iter = 0;

  while (deltaNormalized < 0) {
    deltaNormalized += M_PI;
  }
  while (deltaNormalized > M_PI) {
    deltaNormalized -= M_PI;
  }

  vpColVector vecLine(2);
  vecLine[0] = cos(deltaNormalized);
  vecLine[1] = sin(deltaNormalized);
  vecLine.normalize();

  double offset = std::floor(SobelX.getRows() / 2.0f);

  for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != m_meList.end(); ++it) {
    if (iter != 0 && iter + 1 != m_meList.size()) {
      double gradientX = 0;
      double gradientY = 0;

      double iSite = it->m_ifloat;
      double jSite = it->m_jfloat;

      for (unsigned int i = 0; i < SobelX.getRows(); i++) {
        double iImg = iSite + (i - offset);
        for (unsigned int j = 0; j < SobelX.getCols(); j++) {
          double jImg = jSite + (j - offset);

          if (iImg < 0)
            iImg = 0.0;
          if (jImg < 0)
            jImg = 0.0;

          if (iImg > _I.getHeight() - 1)
            iImg = _I.getHeight() - 1;
          if (jImg > _I.getWidth() - 1)
            jImg = _I.getWidth() - 1;

          gradientX += SobelX[i][j] * _I((unsigned int)iImg, (unsigned int)jImg);
        }
      }

      for (unsigned int i = 0; i < SobelY.getRows(); i++) {
        double iImg = iSite + (i - offset);
        for (unsigned int j = 0; j < SobelY.getCols(); j++) {
          double jImg = jSite + (j - offset);

          if (iImg < 0)
            iImg = 0.0;
          if (jImg < 0)
            jImg = 0.0;

          if (iImg > _I.getHeight() - 1)
            iImg = _I.getHeight() - 1;
          if (jImg > _I.getWidth() - 1)
            jImg = _I.getWidth() - 1;

          gradientY += SobelY[i][j] * _I((unsigned int)iImg, (unsigned int)jImg);
        }
      }

      double angle = atan2(gradientX, gradientY);
      while (angle < 0)
        angle += M_PI;
      while (angle > M_PI)
        angle -= M_PI;

      vpColVector vecGrad(2);
      vecGrad[0] = cos(angle);
      vecGrad[1] = sin(angle);
      vecGrad.normalize();

      double angle1 = acos(vecLine * vecGrad);
      double angle2 = acos(vecLine * (-vecGrad));

      if (display) {
        vpDisplay::displayArrow(_I, it->get_i(), it->get_j(), (int)(it->get_i() + length * cos(deltaNormalized)),
                                (int)(it->get_j() + length * sin(deltaNormalized)), vpColor::blue,
                                length >= 20 ? length / 5 : 4, length >= 20 ? length / 10 : 2, thickness);
        if (angle1 < angle2) {
          vpDisplay::displayArrow(_I, it->get_i(), it->get_j(), (int)(it->get_i() + length * cos(angle)),
                                  (int)(it->get_j() + length * sin(angle)), vpColor::red, length >= 20 ? length / 5 : 4,
                                  length >= 20 ? length / 10 : 2, thickness);
        }
        else {
          vpDisplay::displayArrow(_I, it->get_i(), it->get_j(), (int)(it->get_i() + length * cos(angle + M_PI)),
                                  (int)(it->get_j() + length * sin(angle + M_PI)), vpColor::red,
                                  length >= 20 ? length / 5 : 4, length >= 20 ? length / 10 : 2, thickness);
        }
      }

      _sumErrorRad += std::min<double>(angle1, angle2);

      _nbFeatures++;
    }
    iter++;
  }
}

/*!
  Resample the line if the number of sample is less than 50% of the
  expected value.

  \note The expected value is computed thanks to the length of the
  line and the parameter which indicates the number of pixel between
  two points (vpMe::sample_step).

  \param I : Image in which the line appears.
*/
void vpMbtMeLine::reSample(const vpImage<unsigned char> &I)
{
  unsigned int n = numberOfSignal();

  if ((double)n < 0.5 * expecteddensity && n > 0) {
    double delta_new = m_delta;
    m_delta = m_delta_1;
    sample(I);
    expecteddensity = (double)m_meList.size();
    m_delta = delta_new;
    //  2. On appelle ce qui n'est pas specifique
    vpMeTracker::initTracking(I);
  }
}

/*!
  Resample the line if the number of sample is less than 50% of the
  expected value.

  \note The expected value is computed thanks to the length of the
  line and the parameter which indicates the number of pixel between
  two points (vpMe::sample_step).

  \param I : Image in which the line appears.
  \param ip1 : The first extremity of the line.
  \param ip2 : The second extremity of the line.
*/
void vpMbtMeLine::reSample(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2)
{
  size_t n = m_meList.size();

  if ((double)n < 0.5 * expecteddensity /*&& n > 0*/) // n is always > 0
  {
    double delta_new = m_delta;
    m_delta = m_delta_1;
    m_PExt[0].m_ifloat = (float)ip1.get_i();
    m_PExt[0].m_jfloat = (float)ip1.get_j();
    m_PExt[1].m_ifloat = (float)ip2.get_i();
    m_PExt[1].m_jfloat = (float)ip2.get_j();
    sample(I);
    expecteddensity = (double)m_meList.size();
    m_delta = delta_new;
    vpMeTracker::track(I);
  }
}

/*!
  Set the alpha value of the different vpMeSite to the value of delta.
*/
void vpMbtMeLine::updateDelta()
{
  vpMeSite p_me;

  double diff = 0;

  // if(fabs(theta) == M_PI )
  if (std::fabs(std::fabs(m_theta) - M_PI) <=
      vpMath::maximum(std::fabs(m_theta), (double)M_PI) * std::numeric_limits<double>::epsilon()) {
    m_theta = 0;
  }

  diff = fabs(m_theta - m_theta_1);
  if (diff > M_PI / 2.0) {
    m_sign *= -1;
  }

  m_theta_1 = m_theta;

  m_delta = -m_theta + M_PI / 2.0;
  normalizeAngle(m_delta);

  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != m_meList.end(); ++it) {
    p_me = *it;
    p_me.m_alpha = m_delta;
    p_me.m_mask_sign = m_sign;
    *it = p_me;
  }
  m_delta_1 = m_delta;
}

/*!
  Track the line in the image I.

  \param I : Image in which the line appears.
*/
void vpMbtMeLine::track(const vpImage<unsigned char> &I)
{
  try {
    vpMeTracker::track(I);
    if (m_mask != nullptr) {
      // Expected density could be modified if some vpMeSite are no more tracked because they are outside the mask.
      expecteddensity = (double)m_meList.size();
    }
  }
  catch (...) {
    throw; // throw the original exception
  }
}

/*!
  Update the moving edges parameters after the virtual visual servoing.

  \param  I : The image.
  \param  rho : The \f$\rho\f$ parameter used in the line's polar equation.
  \param  theta : The \f$\theta\f$ parameter used in the line's polar
  equation.
*/
void vpMbtMeLine::updateParameters(const vpImage<unsigned char> &I, double rho, double theta)
{
  m_rho = rho;
  m_theta = theta;
  m_a = cos(m_theta);
  m_b = sin(m_theta);
  m_c = -m_rho;
  // recherche de point aux extremite de la droites
  // dans le cas d'un glissement
  suppressPoints(I);
  seekExtremities(I);
  suppressPoints(I);
  setExtremities();
  // reechantillonage si necessaire
  reSample(I);

  // remet a jour l'angle delta pour chaque  point de la liste
  updateDelta();
}

/*!
  Update the moving edges parameters after the virtual visual servoing.

  \param I : The image.
  \param ip1 : The first extremity of the line.
  \param ip2 : The second extremity of the line.
  \param rho : The \f$\rho\f$ parameter used in the line's polar equation.
  \param theta : The \f$\theta\f$ parameter used in the line's polar
  equation.
*/
void vpMbtMeLine::updateParameters(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2,
                                   double rho, double theta)
{
  m_rho = rho;
  m_theta = theta;
  m_a = cos(m_theta);
  m_b = sin(m_theta);
  m_c = -m_rho;
  // recherche de point aux extremite de la droites
  // dans le cas d'un glissement
  suppressPoints(I);
  seekExtremities(I);
  suppressPoints(I);
  setExtremities();
  // reechantillonage si necessaire
  reSample(I, ip1, ip2);

  // remet a jour l'angle delta pour chaque  point de la liste
  updateDelta();
}

/*!
  Seek in the list of available points the two extremities of the line.
*/
void vpMbtMeLine::setExtremities()
{
  double i_min = +1e6;
  double j_min = +1e6;
  double i_max = -1;
  double j_max = -1;

  // Loop through list of sites to track
  for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != m_meList.end(); ++it) {
    vpMeSite s = *it; // current reference pixel
    if (s.m_ifloat < i_min) {
      i_min = s.m_ifloat;
      j_min = s.m_jfloat;
    }

    if (s.m_ifloat > i_max) {
      i_max = s.m_ifloat;
      j_max = s.m_jfloat;
    }
  }

  if (!m_meList.empty()) {
    m_PExt[0].m_ifloat = i_min;
    m_PExt[0].m_jfloat = j_min;
    m_PExt[1].m_ifloat = i_max;
    m_PExt[1].m_jfloat = j_max;
  }

  if (fabs(i_min - i_max) < 25) {
    for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != m_meList.end(); ++it) {
      vpMeSite s = *it; // current reference pixel
      if (s.m_jfloat < j_min) {
        i_min = s.m_ifloat;
        j_min = s.m_jfloat;
      }

      if (s.m_jfloat > j_max) {
        i_max = s.m_ifloat;
        j_max = s.m_jfloat;
      }
    }

    if (!m_meList.empty()) {
      m_PExt[0].m_ifloat = i_min;
      m_PExt[0].m_jfloat = j_min;
      m_PExt[1].m_ifloat = i_max;
      m_PExt[1].m_jfloat = j_max;
    }
    bubbleSortJ();
  }

  else
    bubbleSortI();
}

static bool sortByI(const vpMeSite &s1, const vpMeSite &s2) { return (s1.m_ifloat > s2.m_ifloat); }

void vpMbtMeLine::bubbleSortI()
{
#if 0
  unsigned int nbElmt = m_meList.size();
  for (unsigned int pass = 1; pass < nbElmt; pass++) {
    m_meList.front();
    for (unsigned int i = 0; i < nbElmt-pass; i++) {
      vpMeSite s1 = m_meList.value();
      vpMeSite s2 = m_meList.nextValue();
      if (s1.m_ifloat > s2.m_ifloat)
        m_meList.swapRight();
      else
        m_meList.next();
    }
  }
#endif
  m_meList.sort(sortByI);
}

static bool sortByJ(const vpMeSite &s1, const vpMeSite &s2) { return (s1.m_jfloat > s2.m_jfloat); }

void vpMbtMeLine::bubbleSortJ()
{
#if 0
  unsigned int nbElmt = m_meList.size();
  for (unsigned int pass = 1; pass < nbElmt; pass++) {
    m_meList.front();
    for (unsigned int i = 0; i < nbElmt-pass; i++) {
      vpMeSite s1 = m_meList.value();
      vpMeSite s2 = m_meList.nextValue();
      if (s1.m_jfloat > s2.m_jfloat)
        m_meList.swapRight();
      else
        m_meList.next();
    }
  }
#endif
  m_meList.sort(sortByJ);
}
END_VISP_NAMESPACE
#endif
