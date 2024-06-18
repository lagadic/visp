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
 * Moving edges.
 */

/*!
  \file vpMeLine.cpp
  \brief Moving edges
*/

#include <algorithm> // (std::min)
#include <cmath>     // std::fabs
#include <limits>    // numeric_limits
#include <stdlib.h>
#include <visp3/core/vpDebug.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRobust.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/me/vpMe.h>
#include <visp3/me/vpMeLine.h>
#include <visp3/me/vpMeSite.h>
#include <visp3/me/vpMeTracker.h>

#define INCR_MIN 1

BEGIN_VISP_NAMESPACE
void computeDelta(double &delta, int i1, int j1, int i2, int j2);

static void normalizeAngle(double &delta)
{
  while (delta > M_PI) {
    delta -= M_PI;
  }
  while (delta < -M_PI) {
    delta += M_PI;
  }
}

void computeDelta(double &delta, int i1, int j1, int i2, int j2)
{

  double B = double(i1 - i2);
  double A = double(j1 - j2);

  delta = atan2(B, A);
  delta -= M_PI / 2.0;
  normalizeAngle(delta);
}

static void project(double a, double b, double c, double i, double j, double &ip, double &jp)
{
  if (fabs(a) > fabs(b)) {
    jp = (vpMath::sqr(a) * j - a * b * i - c * b) / (vpMath::sqr(a) + vpMath::sqr(b));
    ip = (-c - b * jp) / a;
  }
  else {
    ip = (vpMath::sqr(b) * i - a * b * j - c * a) / (vpMath::sqr(a) + vpMath::sqr(b));
    jp = (-c - a * ip) / b;
  }
}

vpMeLine::vpMeLine()
  : m_rho(0.), m_theta(0.), m_delta(0.), m_delta_1(0.), m_angle(0.), m_angle_1(90), m_sign(1),
  m_useIntensityForRho(true), m_a(0.), m_b(0.), m_c(0.)
{ }

vpMeLine::vpMeLine(const vpMeLine &meline)
  : vpMeTracker(meline), m_rho(0.), m_theta(0.), m_delta(0.), m_delta_1(0.), m_angle(0.), m_angle_1(90), m_sign(1),
  m_useIntensityForRho(true), m_a(0.), m_b(0.), m_c(0.)

{
  m_rho = meline.m_rho;
  m_theta = meline.m_theta;
  m_delta = meline.m_delta;
  m_delta_1 = meline.m_delta_1;
  m_angle = meline.m_angle;
  m_angle_1 = meline.m_angle_1;
  m_sign = meline.m_sign;

  m_a = meline.m_a;
  m_b = meline.m_b;
  m_c = meline.m_c;
  m_useIntensityForRho = meline.m_useIntensityForRho;
  m_PExt[0] = meline.m_PExt[0];
  m_PExt[1] = meline.m_PExt[1];
}

vpMeLine::~vpMeLine()
{
  m_meList.clear();
}

void vpMeLine::sample(const vpImage<unsigned char> &I, bool doNotTrack)
{
  (void)doNotTrack;
  if (!m_me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  int nbrows = static_cast<int>(I.getHeight());
  int nbcols = static_cast<int>(I.getWidth());
  double n_sample;

  if (std::fabs(m_me->getSampleStep()) <= std::numeric_limits<double>::epsilon()) {
    vpERROR_TRACE("function called with sample step = 0");
    throw(vpTrackingException(vpTrackingException::fatalError, "sample step = 0"));
  }

  // i, j portions of the line_p
  double diffsi = m_PExt[0].m_ifloat - m_PExt[1].m_ifloat;
  double diffsj = m_PExt[0].m_jfloat - m_PExt[1].m_jfloat;

  double length_p = sqrt((vpMath::sqr(diffsi) + vpMath::sqr(diffsj)));
  if (std::fabs(length_p) <= std::numeric_limits<double>::epsilon())
    throw(vpTrackingException(vpTrackingException::fatalError, "points too close of each other to define a line"));
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
    if ((!outOfImage(iP, 0, nbrows, nbcols)) && inRoiMask(m_mask, is_uint, js_uint)
        && inMeMaskCandidates(m_maskCandidates, is_uint, js_uint)) {
      vpMeSite pix;
      pix.init(iP.get_i(), iP.get_j(), m_delta, 0, m_sign);
      pix.setDisplay(m_selectDisplay);
      pix.setState(vpMeSite::NO_SUPPRESSION);
      const double marginRatio = m_me->getThresholdMarginRatio();
      double convolution = pix.convolution(I, m_me);
      double contrastThreshold = fabs(convolution) * marginRatio;
      pix.setContrastThreshold(contrastThreshold, *m_me);

      if (vpDEBUG_ENABLE(3)) {
        vpDisplay::displayCross(I, iP, 2, vpColor::blue);
      }

      m_meList.push_back(pix);
    }
    is += stepi;
    js += stepj;
  }

  vpCDEBUG(1) << "end vpMeLine::sample() : ";
  vpCDEBUG(1) << n_sample << " point inserted in the list " << std::endl;
}

void vpMeLine::display(const vpImage<unsigned char> &I, const vpColor &color, unsigned int thickness)
{
  vpMeLine::displayLine(I, m_PExt[0], m_PExt[1], m_meList, m_a, m_b, m_c, color, thickness);
}

void vpMeLine::initTracking(const vpImage<unsigned char> &I)
{
  vpImagePoint ip1, ip2;

  vpDisplay::flush(I);

  std::cout << "Click on the line first point..." << std::endl;
  while (vpDisplay::getClick(I, ip1) != true) { }

  vpDisplay::displayCross(I, ip1, 7, vpColor::red);
  vpDisplay::flush(I);
  std::cout << "Click on the line second point..." << std::endl;
  while (vpDisplay::getClick(I, ip2) != true) { }

  vpDisplay::displayCross(I, ip2, 7, vpColor::red);
  vpDisplay::flush(I);

  try {
    initTracking(I, ip1, ip2);
  }
  catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

void vpMeLine::leastSquare()
{
  vpMatrix A(numberOfSignal(), 2);
  vpColVector x(2), x_1(2);
  x_1 = 0;

  vpRobust r;
  r.setMinMedianAbsoluteDeviation(2);
  vpMatrix D(numberOfSignal(), numberOfSignal());
  D.eye();
  vpMatrix DA(numberOfSignal(), 2);
  vpColVector w(numberOfSignal());
  vpColVector B(numberOfSignal());
  w = 1;
  vpMeSite p_me;
  unsigned int iter = 0;
  unsigned int nos_1 = 0;
  double distance = 100;

  if (m_meList.size() <= 2 || numberOfSignal() <= 2) {
    // vpERROR_TRACE("Not enough point") ;
    vpCDEBUG(1) << "Not enough point";
    throw(vpTrackingException(vpTrackingException::notEnoughPointError, "not enough point"));
  }

  if ((fabs(m_b) >= 0.9)) // Construction du systeme Ax=B
    // a i + j + c = 0
    // A = (i 1)   B = (-j)
  {
    nos_1 = numberOfSignal();
    unsigned int k = 0;
    for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != m_meList.end(); ++it) {
      p_me = *it;
      if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
        A[k][0] = p_me.m_ifloat;
        A[k][1] = 1;
        B[k] = -p_me.m_jfloat;
        k++;
      }
    }

    while (iter < 4 && distance > 0.05) {
      for (unsigned int i = 0; i < k; i++) {
        for (unsigned int j = 0; j < 2; j++) {
          DA[i][j] = w[i] * A[i][j];
        }
      }

      x = DA.pseudoInverse(1e-26) * D * B;

      vpColVector residu(nos_1);
      residu = B - A * x;
      r.MEstimator(vpRobust::TUKEY, residu, w);

      k = 0;
      for (unsigned int i = 0; i < nos_1; i++) {
        D[k][k] = w[k];
        k++;
      }
      iter++;
      distance = fabs(x[0] - x_1[0]) + fabs(x[1] - x_1[1]);
      x_1 = x;
    }

    k = 0;
    for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != m_meList.end(); ++it) {
      p_me = *it;
      if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
        if (w[k] < 0.2) {
          p_me.setState(vpMeSite::M_ESTIMATOR);

          *it = p_me;
        }
        k++;
      }
    }

    // mise a jour de l'equation de la droite
    m_a = x[0];
    m_b = 1;
    m_c = x[1];

    double s = sqrt(vpMath::sqr(m_a) + vpMath::sqr(m_b));
    m_a /= s;
    m_b /= s;
    m_c /= s;
  }

  else // Construction du systeme Ax=B
       // i + bj + c = 0
       // A = (j 1)   B = (-i)
  {
    nos_1 = numberOfSignal();
    unsigned int k = 0;
    for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != m_meList.end(); ++it) {
      p_me = *it;
      if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
        A[k][0] = p_me.m_jfloat;
        A[k][1] = 1;
        B[k] = -p_me.m_ifloat;
        k++;
      }
    }

    while (iter < 4 && distance > 0.05) {
      DA = D * A;
      x = DA.pseudoInverse(1e-26) * D * B;

      vpColVector residu(nos_1);
      residu = B - A * x;
      r.MEstimator(vpRobust::TUKEY, residu, w);

      k = 0;
      for (unsigned int i = 0; i < nos_1; i++) {
        D[k][k] = w[k];
        k++;
      }
      iter++;
      distance = fabs(x[0] - x_1[0]) + fabs(x[1] - x_1[1]);
      x_1 = x;
    }

    k = 0;
    for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != m_meList.end(); ++it) {
      p_me = *it;
      if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
        if (w[k] < 0.2) {
          p_me.setState(vpMeSite::M_ESTIMATOR);

          *it = p_me;
        }
        k++;
      }
    }
    m_a = 1;
    m_b = x[0];
    m_c = x[1];

    double s = sqrt(vpMath::sqr(m_a) + vpMath::sqr(m_b));
    m_a /= s;
    m_b /= s;
    m_c /= s;
  }

  // mise a jour du delta
  m_delta = atan2(m_a, m_b);

  normalizeAngle(m_delta);
}

void vpMeLine::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2)
{
  vpCDEBUG(1) << " begin vpMeLine::initTracking()" << std::endl;

  int i1s, j1s, i2s, j2s;

  i1s = vpMath::round(ip1.get_i());
  i2s = vpMath::round(ip2.get_i());
  j1s = vpMath::round(ip1.get_j());
  j2s = vpMath::round(ip2.get_j());

  try {

    //  1. On fait ce qui concerne les droites (peut etre vide)
    {
      // Points extremites
      m_PExt[0].m_ifloat = (float)ip1.get_i();
      m_PExt[0].m_jfloat = (float)ip1.get_j();
      m_PExt[1].m_ifloat = (float)ip2.get_i();
      m_PExt[1].m_jfloat = (float)ip2.get_j();

      double angle_ = atan2((double)(i1s - i2s), (double)(j1s - j2s));
      m_a = cos(angle_);
      m_b = sin(angle_);

      // Real values of a, b can have an other sign. So to get the good values
      // of a and b in order to initialise then c, we call track(I) just below

      computeDelta(m_delta, i1s, j1s, i2s, j2s);
      m_delta_1 = m_delta;

      //      vpTRACE("a: %f b: %f c: %f -b/a: %f delta: %f", a, b, c, -(b/a),
      //      delta);

      sample(I);
    }
    //  2. On appelle ce qui n'est pas specifique
    {
      vpMeTracker::initTracking(I);
    }
    // Call track(I) to give the good sign to a and b and to initialise c
    // which can be used for the display
    track(I);
  }
  catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
  vpCDEBUG(1) << " end vpMeLine::initTracking()" << std::endl;
}

void vpMeLine::suppressPoints()
{
  // Loop through list of sites to track
  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != m_meList.end();) {
    vpMeSite s = *it; // current reference pixel

    if (s.getState() != vpMeSite::NO_SUPPRESSION)
      it = m_meList.erase(it);
    else
      ++it;
  }
}

void vpMeLine::setExtremities()
{
  double imin = +1e6;
  double jmin = +1e6;
  double imax = -1;
  double jmax = -1;

  // Loop through list of sites to track
  for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != m_meList.end(); ++it) {
    vpMeSite s = *it; // current reference pixel
    if (s.m_ifloat < imin) {
      imin = s.m_ifloat;
      jmin = s.m_jfloat;
    }

    if (s.m_ifloat > imax) {
      imax = s.m_ifloat;
      jmax = s.m_jfloat;
    }
  }

  m_PExt[0].m_ifloat = imin;
  m_PExt[0].m_jfloat = jmin;
  m_PExt[1].m_ifloat = imax;
  m_PExt[1].m_jfloat = jmax;

  if (fabs(imin - imax) < 25) {
    for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != m_meList.end(); ++it) {
      vpMeSite s = *it; // current reference pixel
      if (s.m_jfloat < jmin) {
        imin = s.m_ifloat;
        jmin = s.m_jfloat;
      }

      if (s.m_jfloat > jmax) {
        imax = s.m_ifloat;
        jmax = s.m_jfloat;
      }
    }
    m_PExt[0].m_ifloat = imin;
    m_PExt[0].m_jfloat = jmin;
    m_PExt[1].m_ifloat = imax;
    m_PExt[1].m_jfloat = jmax;
  }
}

void vpMeLine::seekExtremities(const vpImage<unsigned char> &I)
{
  vpCDEBUG(1) << "begin vpMeLine::sample() : " << std::endl;

  if (!m_me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  int nbrows = static_cast<int>(I.getHeight());
  int nbcols = static_cast<int>(I.getWidth());
  double n_sample;

  // if (m_me->getSampleStep()==0)
  if (std::fabs(m_me->getSampleStep()) <= std::numeric_limits<double>::epsilon()) {

    vpERROR_TRACE("function called with sample step = 0");
    throw(vpTrackingException(vpTrackingException::fatalError, "sample step = 0"));
  }

  // i, j portions of the line_p
  double diffsi = m_PExt[0].m_ifloat - m_PExt[1].m_ifloat;
  double diffsj = m_PExt[0].m_jfloat - m_PExt[1].m_jfloat;

  double s = vpMath::sqr(diffsi) + vpMath::sqr(diffsj);

  double di = diffsi / sqrt(s); // pas de risque de /0 car d(P1,P2) >0
  double dj = diffsj / sqrt(s);

  double length_p = sqrt((vpMath::sqr(diffsi) + vpMath::sqr(diffsj)));

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
    P.m_i = static_cast<int>(P.m_ifloat);
    P.m_jfloat = P.m_jfloat + dj * sample_step;
    P.m_j = static_cast<int>(P.m_jfloat);

    vpImagePoint iP;
    iP.set_i(P.m_ifloat);
    iP.set_j(P.m_jfloat);

    unsigned int is_uint = static_cast<unsigned int>(P.m_ifloat);
    unsigned int js_uint = static_cast<unsigned int>(P.m_jfloat);
    if ((!outOfImage(iP, 5, nbrows, nbcols)) && inRoiMask(m_mask, is_uint, js_uint)
        && inMeMaskCandidates(m_maskCandidates, is_uint, js_uint)) {
      P.track(I, m_me, false);

      if (P.getState() == vpMeSite::NO_SUPPRESSION) {
        m_meList.push_back(P);
        if (vpDEBUG_ENABLE(3)) {
          vpDisplay::displayCross(I, iP, 5, vpColor::green);
        }
      }
      else {
        if (vpDEBUG_ENABLE(3)) {
          vpDisplay::displayCross(I, iP, 10, vpColor::blue);
        }
      }
    }
  }

  P.init((int)m_PExt[1].m_ifloat, (int)m_PExt[1].m_jfloat, m_delta_1, 0, m_sign);
  P.setDisplay(m_selectDisplay);
  for (int i = 0; i < 3; i++) {
    P.m_ifloat = P.m_ifloat - di * sample_step;
    P.m_i = static_cast<int>(P.m_ifloat);
    P.m_jfloat = P.m_jfloat - dj * sample_step;
    P.m_j = static_cast<int>(P.m_jfloat);

    vpImagePoint iP;
    iP.set_i(P.m_ifloat);
    iP.set_j(P.m_jfloat);

    unsigned int is_uint = static_cast<unsigned int>(P.m_ifloat);
    unsigned int js_uint = static_cast<unsigned int>(P.m_jfloat);
    if ((!outOfImage(iP, 5, nbrows, nbcols)) && inRoiMask(m_mask, is_uint, js_uint)
            && inMeMaskCandidates(m_maskCandidates, is_uint, js_uint)) {
      P.track(I, m_me, false);

      if (P.getState() == vpMeSite::NO_SUPPRESSION) {
        m_meList.push_back(P);
        if (vpDEBUG_ENABLE(3)) {
          vpDisplay::displayCross(I, iP, 5, vpColor::green);
        }
      }
      else {
        if (vpDEBUG_ENABLE(3)) {
          vpDisplay::displayCross(I, iP, 10, vpColor::blue);
        }
      }
    }
  }

  m_me->setRange(memory_range);

  vpCDEBUG(1) << "end vpMeLine::sample() : ";
  vpCDEBUG(1) << n_sample << " point inserted in the list " << std::endl;
}

void vpMeLine::reSample(const vpImage<unsigned char> &I)
{
  double i1, j1, i2, j2;

  if (!m_me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  project(m_a, m_b, m_c, m_PExt[0].m_ifloat, m_PExt[0].m_jfloat, i1, j1);
  project(m_a, m_b, m_c, m_PExt[1].m_ifloat, m_PExt[1].m_jfloat, i2, j2);

  // Points extremites
  m_PExt[0].m_ifloat = i1;
  m_PExt[0].m_jfloat = j1;
  m_PExt[1].m_ifloat = i2;
  m_PExt[1].m_jfloat = j2;

  double d = sqrt(vpMath::sqr(i1 - i2) + vpMath::sqr(j1 - j2));

  unsigned int n = numberOfSignal();
  double expecteddensity = d / (double)m_me->getSampleStep();

  if ((double)n < 0.9 * expecteddensity) {
    double delta_new = m_delta;
    m_delta = m_delta_1;
    sample(I);
    m_delta = delta_new;
    //  2. On appelle ce qui n'est pas specifique
    vpMeTracker::initTracking(I);
  }
}

void vpMeLine::updateDelta()
{
  vpMeSite p_me;

  double angle_ = m_delta + M_PI / 2;
  double diff = 0;

  while (angle_ < 0)
    angle_ += M_PI;
  while (angle_ > M_PI)
    angle_ -= M_PI;

  angle_ = vpMath::round(angle_ * 180 / M_PI);

  // if(fabs(angle_) == 180 )
  if (std::fabs(std::fabs(angle_) - 180) <= std::numeric_limits<double>::epsilon()) {
    angle_ = 0;
  }

  // std::cout << "angle theta : " << theta << std::endl ;
  diff = fabs(angle_ - m_angle_1);
  if (diff > 90)
    m_sign *= -1;

  m_angle_1 = angle_;

  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != m_meList.end(); ++it) {
    p_me = *it;
    p_me.setAlpha(m_delta);
    p_me.m_mask_sign = m_sign;
    *it = p_me;
  }
  m_delta_1 = m_delta;
}

void vpMeLine::track(const vpImage<unsigned char> &I)
{
  vpCDEBUG(1) << "begin vpMeLine::track()" << std::endl;

  vpMeTracker::track(I);

  // 3. On revient aux droites
  {
    // supression des points rejetes par les ME
    suppressPoints();
    setExtremities();

    // Estimation des parametres de la droite aux moindres carre
    try {
      leastSquare();
    }
    catch (...) {
      vpERROR_TRACE("Error caught");
      throw;
    }

    // recherche de point aux extremite de la droites
    // dans le cas d'un glissement
    seekExtremities(I);

    setExtremities();
    try {
      leastSquare();
    }
    catch (...) {
      vpERROR_TRACE("Error caught");
      throw;
    }

    // suppression des points rejetes par la regression robuste
    suppressPoints();
    setExtremities();

    // reechantillonage si necessaire
    reSample(I);

    // remet a jour l'angle delta pour chaque  point de la liste

    updateDelta();

    // Remise a jour de delta dans la liste de site me
    if (vpDEBUG_ENABLE(2)) {
      display(I, vpColor::red);
      vpMeTracker::display(I);
      vpDisplay::flush(I);
    }
  }

  computeRhoTheta(I);

  vpCDEBUG(1) << "end vpMeLine::track()" << std::endl;
}

void vpMeLine::update_indices(double theta, int i, int j, int incr, int &i1, int &i2, int &j1, int &j2)
{
  i1 = (int)(i + cos(theta) * incr);
  j1 = (int)(j + sin(theta) * incr);

  i2 = (int)(i - cos(theta) * incr);
  j2 = (int)(j - sin(theta) * incr);
}

void vpMeLine::computeRhoTheta(const vpImage<unsigned char> &I)
{
  m_rho = fabs(m_c);
  m_theta = atan2(m_b, m_a);

  while (m_theta >= M_PI)
    m_theta -= M_PI;
  while (m_theta < 0)
    m_theta += M_PI;

  if (m_useIntensityForRho) {
    // convention pour choisir le signe de rho
    int i, j;
    i = vpMath::round((m_PExt[0].m_ifloat + m_PExt[1].m_ifloat) / 2);
    j = vpMath::round((m_PExt[0].m_jfloat + m_PExt[1].m_jfloat) / 2);

    int end = false;
    int incr = 10;

    int i1 = 0, i2 = 0, j1 = 0, j2 = 0;
    unsigned char v1 = 0, v2 = 0;

    int width_ = (int)I.getWidth();
    int height_ = (int)I.getHeight();
    update_indices(m_theta, i, j, incr, i1, i2, j1, j2);

    if (i1 < 0 || i1 >= height_ || i2 < 0 || i2 >= height_ || j1 < 0 || j1 >= width_ || j2 < 0 || j2 >= width_) {
      double rho_lim1 = fabs((double)i / cos(m_theta));
      double rho_lim2 = fabs((double)j / sin(m_theta));

      double co_rho_lim1 = fabs(((double)(height_ - i)) / cos(m_theta));
      double co_rho_lim2 = fabs(((double)(width_ - j)) / sin(m_theta));

      double rho_lim = std::min<double>(rho_lim1, rho_lim2);
      double co_rho_lim = std::min<double>(co_rho_lim1, co_rho_lim2);
      incr = (int)std::floor(std::min<double>(rho_lim, co_rho_lim));
      if (incr < INCR_MIN) {
        vpERROR_TRACE("increment is too small");
        throw(vpTrackingException(vpTrackingException::fatalError, "increment is too small"));
      }
      update_indices(m_theta, i, j, incr, i1, i2, j1, j2);
    }

    while (!end) {
      end = true;
      unsigned int i1_ = static_cast<unsigned int>(i1);
      unsigned int j1_ = static_cast<unsigned int>(j1);
      unsigned int i2_ = static_cast<unsigned int>(i2);
      unsigned int j2_ = static_cast<unsigned int>(j2);
      v1 = I[i1_][j1_];
      v2 = I[i2_][j2_];
      if (abs(v1 - v2) < 1) {
        incr--;
        end = false;
        if (incr == 1) {
          throw(vpException(vpException::fatalError, "In vpMeLine cannot determine rho sign, since "
                            "there is no gray level difference between both "
                            "sides of the line"));
        }
      }
      update_indices(m_theta, i, j, incr, i1, i2, j1, j2);
    }

    if (m_theta >= 0 && m_theta <= M_PI / 2) {
      if (v2 < v1) {
        m_theta += M_PI;
        m_rho *= -1;
      }
    }

    else {
      double jinter;
      jinter = -m_c / m_b;
      if (v2 < v1) {
        m_theta += M_PI;
        if (jinter > 0) {
          m_rho *= -1;
        }
      }

      else {
        if (jinter < 0) {
          m_rho *= -1;
        }
      }
    }

    if (vpDEBUG_ENABLE(2)) {
      vpImagePoint ip, ip1, ip2;

      ip.set_i(i);
      ip.set_j(j);
      ip1.set_i(i1);
      ip1.set_j(j1);
      ip2.set_i(i2);
      ip2.set_j(j2);

      vpDisplay::displayArrow(I, ip, ip1, vpColor::green);
      vpDisplay::displayArrow(I, ip, ip2, vpColor::red);
    }
  }
}

double vpMeLine::getRho() const { return m_rho; }

double vpMeLine::getTheta() const { return m_theta; }

void vpMeLine::getExtremities(vpImagePoint &ip1, vpImagePoint &ip2)
{
  /*Return the coordinates of the extremities of the line*/
  ip1.set_i(m_PExt[0].m_ifloat);
  ip1.set_j(m_PExt[0].m_jfloat);
  ip2.set_i(m_PExt[1].m_ifloat);
  ip2.set_j(m_PExt[1].m_jfloat);
}

bool vpMeLine::intersection(const vpMeLine &line1, const vpMeLine &line2, vpImagePoint &ip)
{
  double a1 = line1.m_a;
  double b1 = line1.m_b;
  double c1 = line1.m_c;
  double a2 = line2.m_a;
  double b2 = line2.m_b;
  double c2 = line2.m_c;

  try {
    double i = 0, j = 0;
    double denom = 0;

    if (a1 > 0.1) {
      denom = (-(a2 / a1) * b1 + b2);

      // if (denom == 0)
      if (std::fabs(denom) <= std::numeric_limits<double>::epsilon()) {
        std::cout << "!!!!!!!!!!!!! Problem : Lines are parallel !!!!!!!!!!!!!" << std::endl;
        return (false);
      }

      // if (denom != 0 )
      if (std::fabs(denom) > std::numeric_limits<double>::epsilon()) {
        j = ((a2 / a1) * c1 - c2) / denom;
        i = (-b1 * j - c1) / a1;
      }
    }

    else {
      denom = (-(b2 / b1) * a1 + a2);

      // if (denom == 0)
      if (std::fabs(denom) <= std::numeric_limits<double>::epsilon()) {
        std::cout << "!!!!!!!!!!!!! Problem : Lines are parallel !!!!!!!!!!!!!" << std::endl;
        return (false);
      }

      // if (denom != 0 )
      if (std::fabs(denom) > std::numeric_limits<double>::epsilon()) {
        i = ((b2 / b1) * c1 - c2) / denom;
        j = (-a1 * i - c1) / b1;
      }
    }
    ip.set_i(i);
    ip.set_j(j);

    return (true);
  }
  catch (...) {
    return (false);
  }
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  \deprecated This static function is deprecated. You should rather use vpMeLine::displayLine().

  Display of a moving line thanks to its equation parameters and its extremities.

  \param I : The image used as background.

  \param PExt1 : First extremity

  \param PExt2 : Second extremity

  \param A : Parameter a of the line equation a*i + b*j + c = 0

  \param B : Parameter b of the line equation a*i + b*j + c = 0

  \param C : Parameter c of the line equation a*i + b*j + c = 0

  \param color : Color used to display the line.

  \param thickness : Thickness of the line.
*/
void vpMeLine::display(const vpImage<unsigned char> &I, const vpMeSite &PExt1, const vpMeSite &PExt2, const double &A,
  const double &B, const double &C, const vpColor &color, unsigned int thickness)
{
  vpMeLine::displayLine(I, PExt1, PExt2, A, B, C, color, thickness);
}

/*!
  \deprecated This static function is deprecated. You should rather use vpMeLine::displayLine().

  Display of a moving line thanks to its equation parameters and its extremities.

  \param I : The image used as background.

  \param PExt1 : First extremity

  \param PExt2 : Second extremity

  \param A : Parameter a of the line equation a*i + b*j + c = 0

  \param B : Parameter b of the line equation a*i + b*j + c = 0

  \param C : Parameter c of the line equation a*i + b*j + c = 0

  \param color : Color used to display the line.

  \param thickness : Thickness of the line.
*/
void vpMeLine::display(const vpImage<vpRGBa> &I, const vpMeSite &PExt1, const vpMeSite &PExt2, const double &A,
  const double &B, const double &C, const vpColor &color, unsigned int thickness)
{
  vpMeLine::displayLine(I, PExt1, PExt2, A, B, C, color, thickness);
}

/*!
  \deprecated This static function is deprecated. You should rather use vpMeLine::displayLine().

  Display of a moving line thanks to its equation parameters and its extremities with all the site list.

  \param I : The image used as background.

  \param PExt1 : First extremity

  \param PExt2 : Second extremity

  \param site_list : vpMeSite list

  \param A : Parameter a of the line equation a*i + b*j + c = 0

  \param B : Parameter b of the line equation a*i + b*j + c = 0

  \param C : Parameter c of the line equation a*i + b*j + c = 0

  \param color : Color used to display the line.

  \param thickness : Thickness of the line.
*/
void vpMeLine::display(const vpImage<unsigned char> &I, const vpMeSite &PExt1, const vpMeSite &PExt2,
  const std::list<vpMeSite> &site_list, const double &A, const double &B, const double &C,
  const vpColor &color, unsigned int thickness)
{
  vpMeLine::displayLine(I, PExt1, PExt2, site_list, A, B, C, color, thickness);
}

/*!
  \deprecated This static function is deprecated. You should rather use vpMeLine::displayLine().

  Display of a moving line thanks to its equation parameters and its extremities with all the site list.

  \param I : The image used as background.

  \param PExt1 : First extremity

  \param PExt2 : Second extremity

  \param site_list : vpMeSite list

  \param A : Parameter a of the line equation a*i + b*j + c = 0

  \param B : Parameter b of the line equation a*i + b*j + c = 0

  \param C : Parameter c of the line equation a*i + b*j + c = 0

  \param color : Color used to display the line.

  \param thickness : Thickness of the line.
*/
void vpMeLine::display(const vpImage<vpRGBa> &I, const vpMeSite &PExt1, const vpMeSite &PExt2,
  const std::list<vpMeSite> &site_list, const double &A, const double &B, const double &C,
  const vpColor &color, unsigned int thickness)
{

  vpMeLine::displayLine(I, PExt1, PExt2, site_list, A, B, C, color, thickness);
}
#endif // Deprecated

void vpMeLine::displayLine(const vpImage<unsigned char> &I, const vpMeSite &PExt1, const vpMeSite &PExt2, const double &A,
  const double &B, const double &C, const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip1, ip2;

  if (fabs(A) < fabs(B)) {
    double i1, j1, i2, j2;
    i1 = 0;
    j1 = (-A * i1 - C) / B;
    i2 = I.getHeight() - 1.0;
    j2 = (-A * i2 - C) / B;

    ip1.set_i(i1);
    ip1.set_j(j1);
    ip2.set_i(i2);
    ip2.set_j(j2);
    vpDisplay::displayLine(I, ip1, ip2, color);
  }
  else {
    double i1, j1, i2, j2;
    j1 = 0;
    i1 = -(B * j1 + C) / A;
    j2 = I.getWidth() - 1.0;
    i2 = -(B * j2 + C) / A;

    ip1.set_i(i1);
    ip1.set_j(j1);
    ip2.set_i(i2);
    ip2.set_j(j2);
    vpDisplay::displayLine(I, ip1, ip2, color);
  }

  ip1.set_i(PExt1.m_ifloat);
  ip1.set_j(PExt1.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);

  ip1.set_i(PExt2.m_ifloat);
  ip1.set_j(PExt2.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);
}

void vpMeLine::displayLine(const vpImage<vpRGBa> &I, const vpMeSite &PExt1, const vpMeSite &PExt2, const double &A,
                           const double &B, const double &C, const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip1, ip2;

  if (fabs(A) < fabs(B)) {
    double i1, j1, i2, j2;
    i1 = 0;
    j1 = (-A * i1 - C) / B;
    i2 = I.getHeight() - 1.0;
    j2 = (-A * i2 - C) / B;

    ip1.set_i(i1);
    ip1.set_j(j1);
    ip2.set_i(i2);
    ip2.set_j(j2);
    vpDisplay::displayLine(I, ip1, ip2, color);
  }
  else {
    double i1, j1, i2, j2;
    j1 = 0;
    i1 = -(B * j1 + C) / A;
    j2 = I.getWidth() - 1.0;
    i2 = -(B * j2 + C) / A;

    ip1.set_i(i1);
    ip1.set_j(j1);
    ip2.set_i(i2);
    ip2.set_j(j2);
    vpDisplay::displayLine(I, ip1, ip2, color);
  }

  ip1.set_i(PExt1.m_ifloat);
  ip1.set_j(PExt1.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);

  ip1.set_i(PExt2.m_ifloat);
  ip1.set_j(PExt2.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);
}

void vpMeLine::displayLine(const vpImage<unsigned char> &I, const vpMeSite &PExt1, const vpMeSite &PExt2,
                           const std::list<vpMeSite> &site_list, const double &A, const double &B, const double &C,
                           const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip;

  for (std::list<vpMeSite>::const_iterator it = site_list.begin(); it != site_list.end(); ++it) {
    vpMeSite pix = *it;
    ip.set_i(pix.m_ifloat);
    ip.set_j(pix.m_jfloat);

    if (pix.getState() == vpMeSite::M_ESTIMATOR)
      vpDisplay::displayCross(I, ip, 5, vpColor::green, thickness);
    else
      vpDisplay::displayCross(I, ip, 5, color, thickness);
  }

  vpImagePoint ip1, ip2;

  if (fabs(A) < fabs(B)) {
    double i1, j1, i2, j2;
    i1 = 0;
    j1 = (-A * i1 - C) / B;
    i2 = I.getHeight() - 1.0;
    j2 = (-A * i2 - C) / B;

    ip1.set_i(i1);
    ip1.set_j(j1);
    ip2.set_i(i2);
    ip2.set_j(j2);
    vpDisplay::displayLine(I, ip1, ip2, color);
  }
  else {
    double i1, j1, i2, j2;
    j1 = 0;
    i1 = -(B * j1 + C) / A;
    j2 = I.getWidth() - 1.0;
    i2 = -(B * j2 + C) / A;

    ip1.set_i(i1);
    ip1.set_j(j1);
    ip2.set_i(i2);
    ip2.set_j(j2);
    vpDisplay::displayLine(I, ip1, ip2, color);
  }

  ip1.set_i(PExt1.m_ifloat);
  ip1.set_j(PExt1.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);

  ip1.set_i(PExt2.m_ifloat);
  ip1.set_j(PExt2.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);
}

void vpMeLine::displayLine(const vpImage<vpRGBa> &I, const vpMeSite &PExt1, const vpMeSite &PExt2,
                           const std::list<vpMeSite> &site_list, const double &A, const double &B, const double &C,
                           const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip;

  for (std::list<vpMeSite>::const_iterator it = site_list.begin(); it != site_list.end(); ++it) {
    vpMeSite pix = *it;
    ip.set_i(pix.m_ifloat);
    ip.set_j(pix.m_jfloat);

    if (pix.getState() == vpMeSite::M_ESTIMATOR)
      vpDisplay::displayCross(I, ip, 5, vpColor::green, thickness);
    else
      vpDisplay::displayCross(I, ip, 5, color, thickness);
  }

  vpImagePoint ip1, ip2;

  if (fabs(A) < fabs(B)) {
    double i1, j1, i2, j2;
    i1 = 0;
    j1 = (-A * i1 - C) / B;
    i2 = I.getHeight() - 1.0;
    j2 = (-A * i2 - C) / B;

    ip1.set_i(i1);
    ip1.set_j(j1);
    ip2.set_i(i2);
    ip2.set_j(j2);
    vpDisplay::displayLine(I, ip1, ip2, color);
  }
  else {
    double i1, j1, i2, j2;
    j1 = 0;
    i1 = -(B * j1 + C) / A;
    j2 = I.getWidth() - 1.0;
    i2 = -(B * j2 + C) / A;

    ip1.set_i(i1);
    ip1.set_j(j1);
    ip2.set_i(i2);
    ip2.set_j(j2);
    vpDisplay::displayLine(I, ip1, ip2, color);
  }

  ip1.set_i(PExt1.m_ifloat);
  ip1.set_j(PExt1.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);

  ip1.set_i(PExt2.m_ifloat);
  ip1.set_j(PExt2.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);
}
END_VISP_NAMESPACE
