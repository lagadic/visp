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
 */

#include <cmath>  // std::fabs
#include <limits> // numeric_limits
#include <vector>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRobust.h>
#include <visp3/me/vpMe.h>
#include <visp3/me/vpMeEllipse.h>

// #define VP_ME_ELLIPSE_REGULAR_SAMPLING
#ifndef VP_ME_ELLIPSE_REGULAR_SAMPLING
#define VP_ME_ELLIPSE_TWO_CONCENTRIC_CIRCLES
#endif

vpMeEllipse::vpMeEllipse()
  : m_K(), m_iPc(), m_a(0.), m_b(0.), m_e(0.), m_iP1(), m_iP2(), m_alpha1(0), m_alpha2(2 * M_PI), m_ce(0.), m_se(0.), m_angleList(), m_m00(0.),
  m_thresholdWeight(0.2), m_alphamin(0.), m_alphamax(0.), m_uc(0.), m_vc(0.), m_n20(0.), m_n11(0.), m_n02(0.),
  m_expectedDensity(0), m_numberOfGoodPoints(0), m_trackCircle(false), m_trackArc(false), m_arcEpsilon(1e-6)
{
  // Resize internal parameters vector
  // K0 u^2 + K1 v^2 + 2 K2 u v + 2 K3 u + 2 K4 v + K5 =  0
  m_K.resize(6);
  m_iP1.set_ij(0, 0);
  m_iP2.set_ij(0, 0);
}

vpMeEllipse::vpMeEllipse(const vpMeEllipse &me_ellipse)
  : vpMeTracker(me_ellipse), m_K(me_ellipse.m_K), m_iPc(me_ellipse.m_iPc), m_a(me_ellipse.m_a), m_b(me_ellipse.m_b), m_e(me_ellipse.m_e),
  m_iP1(me_ellipse.m_iP1), m_iP2(me_ellipse.m_iP2), m_alpha1(me_ellipse.m_alpha1), m_alpha2(me_ellipse.m_alpha2), m_ce(me_ellipse.m_ce),
  m_se(me_ellipse.m_se), m_angleList(me_ellipse.m_angleList), m_m00(me_ellipse.m_m00),
  m_thresholdWeight(me_ellipse.m_thresholdWeight),
  m_alphamin(me_ellipse.m_alphamin), m_alphamax(me_ellipse.m_alphamax), m_uc(me_ellipse.m_uc), m_vc(me_ellipse.m_vc),
  m_n20(me_ellipse.m_n20), m_n11(me_ellipse.m_n11), m_n02(me_ellipse.m_n02),
  m_expectedDensity(me_ellipse.m_expectedDensity), m_numberOfGoodPoints(me_ellipse.m_numberOfGoodPoints),
  m_trackCircle(me_ellipse.m_trackCircle), m_trackArc(me_ellipse.m_trackArc)
{ }

vpMeEllipse::~vpMeEllipse()
{
  m_meList.clear();
  m_angleList.clear();
}

double vpMeEllipse::computeTheta(const vpImagePoint &iP) const
{
  double u = iP.get_u();
  double v = iP.get_v();

  return (computeTheta(u, v));
}

double vpMeEllipse::computeTheta(double u, double v) const
{
  double A = (m_K[0] * u) + (m_K[2] * v) + m_K[3];
  double B = (m_K[1] * v) + (m_K[2] * u) + m_K[4];

  double theta = atan2(B, A); // Angle between the tangent and the u axis.
  if (theta < 0) {            // theta in [0;Pi]  // FC : pourquoi ? pour me sans doute
    theta += M_PI;
  }
  return theta;
}

void vpMeEllipse::updateTheta()
{
  vpMeSite p_me;
  vpImagePoint iP;
  std::list<vpMeSite>::iterator end = m_meList.end();
  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != end; ++it) {
    p_me = *it;
    // (i,j) frame used for vpMESite
    iP.set_ij(p_me.m_ifloat, p_me.m_jfloat);
    p_me.m_alpha = computeTheta(iP);
    *it = p_me;
  }
}

void vpMeEllipse::computePointOnEllipse(const double angle, vpImagePoint &iP)
{
  // Two versions are available. If you change from one version to the other
  // one, do not forget to adapt, for a correct display of an arc
  // of ellipse, vpMeEllipse::display() below and
  // vp_display_display_ellipse() in modules/core/src/display/vpDisplay_impl.h
  // so that the two extremities of the arc are correctly shown.

#ifdef VP_ME_ELLIPSE_REGULAR_SAMPLING
  // Version that gives a regular angular sampling on the ellipse, so less
  // points at its extremities
  double co = cos(angle);
  double si = sin(angle);
  double coef = m_a * m_b / sqrt((m_b * m_b * co * co) + (m_a * m_a * si * si));
  double u = co * coef;
  double v = si * coef;
  iP.set_u((uc + (m_ce * u)) - (m_se * v));
  iP.set_v(vc + (m_se * u) + (m_ce * v));
#elif defined(VP_ME_ELLIPSE_TWO_CONCENTRIC_CIRCLES)
  // Version from "the two concentric circles" method that gives more points
  // at the ellipse extremities for a regular angle sampling. It is better to
  // display an ellipse, not necessarily to track it

  // (u,v) are the coordinates on the canonical centered ellipse;
  double u = m_a * cos(angle);
  double v = m_b * sin(angle);
  // a rotation of e and a translation by (uc,vc) are done
  // to get the coordinates of the point on the shifted ellipse
  iP.set_uv((m_uc + (m_ce * u)) - (m_se * v), m_vc + (m_se * u) + (m_ce * v));
#endif
}

double vpMeEllipse::computeAngleOnEllipse(const vpImagePoint &pt) const
{
  // Two versions are available. If you change from one version to the other
  // one, do not forget to change also the reciprocal function
  // computePointOnEllipse() just above. Adapt also the display; see comment
  // at the beginning of computePointOnEllipse()

#ifdef VP_ME_ELLIPSE_REGULAR_SAMPLING
  // Regular angle sampling method
  double du = pt.get_u() - uc;
  double dv = pt.get_v() - vc;
  double ang = atan2(dv, du) - m_e;
  if (ang > M_PI) {
    ang -= 2.0 * M_PI;
  }
  else if (ang < -M_PI) {
    ang += 2.0 * M_PI;
  }
#ifdef VP_ME_ELLIPSE_TWO_CONCENTRIC_CIRCLES
  // for the "two concentric circles method" starting from the previous one
  // (just to remember the link between both methods:
  // tan(theta_2cc) = a/b tan(theta_rs))

  double co = cos(ang);
  double si = sin(ang);
  double coeff = 1.0 / sqrt((m_b * m_b * co * co) + (m_a * m_a * si * si));
  si *= m_a * coeff;
  co *= m_b * coeff;
  ang = atan2(si, co);
#endif
#elif defined(VP_ME_ELLIPSE_TWO_CONCENTRIC_CIRCLES)
  // For the "two concentric circles" method starting from scratch
  double du = pt.get_u() - m_uc;
  double dv = pt.get_v() - m_vc;
  double co = ((du * m_ce) + (dv * m_se)) / m_a;
  double si = ((-du * m_se) + (dv * m_ce)) / m_b;
  double angle = atan2(si, co);
#endif

  return angle;
}

void vpMeEllipse::computeAbeFromNij()
{
  double num = m_n20 - m_n02;
  double d = (num * num) + (4.0 * m_n11 * m_n11); // always >= 0
  if (d <= std::numeric_limits<double>::epsilon()) {
    m_e = 0.0; // case n20 = n02 and n11 = 0 : circle, e undefined
    m_ce = 1.0;
    m_se = 0.0;
    m_a = (m_b = 2.0 * sqrt(m_n20));         // = sqrt(2.0*(n20+n02))
  }
  else {                             // real ellipse
    m_e = atan2(2.0 * m_n11, num) / 2.0; // e in [-Pi/2 ; Pi/2]
    m_ce = cos(m_e);
    m_se = sin(m_e);

    d = sqrt(d); // d in sqrt always >= 0
    num = m_n20 + m_n02;
    m_a = sqrt(2.0 * (num + d)); // term in sqrt always > 0
    m_b = sqrt(2.0 * (num - d)); // term in sqrt always > 0
  }
}

void vpMeEllipse::computeKiFromNij()
{
  m_K[0] = m_n02;
  m_K[1] = m_n20;
  m_K[2] = -m_n11;
  m_K[3] = (m_n11 * m_vc) - (m_n02 * m_uc);
  m_K[4] = (m_n11 * m_uc) - (m_n20 * m_vc);
  m_K[5] = (m_n02 * m_uc * m_uc) + (m_n20 * m_vc * m_vc) - (2.0 * m_n11 * m_uc * m_vc) + (4.0 * ((m_n11 * m_n11) - (m_n20 * m_n02)));
}

void vpMeEllipse::computeNijFromAbe()
{
  m_n20 = 0.25 * ((m_a * m_a * m_ce * m_ce) + (m_b * m_b * m_se * m_se));
  m_n11 = 0.25 * m_se * m_ce * ((m_a * m_a) - (m_b * m_b));
  m_n02 = 0.25 * ((m_a * m_a * m_se * m_se) + (m_b * m_b * m_ce * m_ce));
}

void vpMeEllipse::getParameters()
{
  // Equations below from Chaumette PhD and TRO 2004 paper
  double num = (m_K[0] * m_K[1]) - (m_K[2] * m_K[2]); // > 0 for an ellipse
  if (num <= 0) {
    throw(vpException(vpException::fatalError, "The points do not belong to an ellipse! num: %f", num));
  }

  m_uc = ((m_K[2] * m_K[4]) - (m_K[1] * m_K[3])) / num;
  m_vc = ((m_K[2] * m_K[3]) - (m_K[0] * m_K[4])) / num;
  m_iPc.set_uv(m_uc, m_vc);

  double d = (((m_K[0] * m_uc * m_uc) + (m_K[1] * m_vc * m_vc) + (2.0 * m_K[2] * m_uc * m_vc)) - m_K[5]) / (4.0 * num);
  m_n20 = m_K[1] * d; // always > 0
  m_n11 = -m_K[2] * d;
  m_n02 = m_K[0] * d; // always > 0

  computeAbeFromNij();

  // normalization so that K0 = n02, K1 = n20, etc (Eq (25) of TRO paper)
  d = m_n02 / m_K[0]; // fabs(K[0]) > 0
  unsigned int Ksize = m_K.size();
  for (unsigned int i = 0; i < Ksize; ++i) {
    m_K[i] *= d;
  }
  if (vpDEBUG_ENABLE(3)) {
    printParameters();
  }
}

void vpMeEllipse::printParameters() const
{
  std::cout << "K :" << m_K.t() << std::endl;
  std::cout << "xc = " << m_uc << ", yc = " << m_vc << std::endl;
  std::cout << "n20 = " << m_n20 << ", n11 = " << m_n11 << ", n02 = " << m_n02 << std::endl;
  std::cout << "A = " << m_a << ", B = " << m_b << ", E (dg) " << vpMath::deg(m_e) << std::endl;
}

void vpMeEllipse::sample(const vpImage<unsigned char> &I, bool doNotTrack)
{
  // Warning: similar code in vpMbtMeEllipse::sample()
  if (!m_me) {
    throw(vpException(vpException::fatalError, "Moving edges on ellipse not initialized"));
  }
  // Delete old lists
  m_meList.clear();
  m_angleList.clear();

  int nbrows = static_cast<int>(I.getHeight());
  int nbcols = static_cast<int>(I.getWidth());
  // New version using distance for sampling
  if (std::fabs(m_me->getSampleStep()) <= std::numeric_limits<double>::epsilon()) {
    std::cout << "Warning: In vpMeEllipse::sample() ";
    std::cout << "function called with sample step = 0. We set it rather to 10 pixels";
    // std::cout << "function called with sample step = 0, set to 10 dg";
    m_me->setSampleStep(10.0);
  }
  // Perimeter of the ellipse using Ramanujan formula
  double perim = M_PI * ((3.0 * (m_a + m_b)) - sqrt(((3.0 * m_a) + m_b) * (m_a + (3.0 * m_b))));
  // Number of points for a complete ellipse
  unsigned int nb_pt = static_cast<unsigned int>(floor(perim / m_me->getSampleStep()));
  double incr = (2.0 * M_PI) / nb_pt;
  // Compute of the expected density
  if (!m_trackArc) { // number of points for a complete ellipse
    m_expectedDensity = nb_pt;
  }
  else { // number of points for an arc of ellipse
    m_expectedDensity *= static_cast<unsigned int>(floor((perim / m_me->getSampleStep()) * ((m_alpha2 - m_alpha1) / (2.0 * M_PI))));
  }

  // Starting angle for sampling: new version to not start at 0
  double ang = m_alpha1 + (incr / 2.0);

  // sample positions
  for (unsigned int i = 0; i < m_expectedDensity; ++i) {
    vpImagePoint iP;
    computePointOnEllipse(ang, iP);
    // If point is in the image, add to the sample list
    // Check done in (i,j) frame)
    if ((!outOfImage(iP, 0, nbrows, nbcols)) && inRoiMask(m_mask, iP.get_i(), iP.get_j())
        && inMeMaskCandidates(m_maskCandidates, iP.get_i(), iP.get_j())) {
      const unsigned int crossSize = 5;
      vpDisplay::displayCross(I, iP, crossSize, vpColor::red);

      double theta = computeTheta(iP);
      vpMeSite pix;
      // (i,j) frame used for vpMeSite
      pix.init(iP.get_i(), iP.get_j(), theta);
      pix.setDisplay(m_selectDisplay);
      pix.setState(vpMeSite::NO_SUPPRESSION);
      const double marginRatio = m_me->getThresholdMarginRatio();
      double convolution = pix.convolution(I, m_me);
      double contrastThreshold = fabs(convolution) * marginRatio;
      pix.setContrastThreshold(contrastThreshold, *m_me);
      m_meList.push_back(pix);
      m_angleList.push_back(ang);
    }
    ang += incr;
  }

  if (!doNotTrack) {
    vpMeTracker::initTracking(I);
  }
}

unsigned int vpMeEllipse::plugHoles(const vpImage<unsigned char> &I)
{
  if (!m_me) {
    throw(vpException(vpException::fatalError, "Moving edges on ellipse tracking not initialized"));
  }
  unsigned int nb_pts_added = 0;
  int nbrows = static_cast<int>(I.getHeight());
  int nbcols = static_cast<int>(I.getWidth());

  unsigned int memory_range = m_me->getRange();
  m_me->setRange(2);

  // Perimeter of the ellipse using Ramanujan formula
  double perim = M_PI * ((3.0 * (m_a + m_b)) - sqrt(((3.0 * m_a) + m_b) * (m_a + (3.0 * m_b))));
  // Number of points for a complete ellipse
  unsigned int nb_pt = static_cast<unsigned int>(floor(perim / m_me->getSampleStep()));
  double incr = (2.0 * M_PI) / nb_pt;

  // Detect holes and try to complete them
  // In this option, the sample step is used to complete the holes as much as possible
  std::list<double>::iterator angleList = m_angleList.begin();
  std::list<vpMeSite>::iterator meList = m_meList.begin();
  const double marginRatio = m_me->getThresholdMarginRatio();
  double ang = *angleList;
  ++angleList;
  ++meList;

  while (meList != m_meList.end()) {
    double nextang = *angleList;
    if ((nextang - ang) > (2.0 * incr)) { // A hole exists
      ang += incr;                      // next point to be checked
      // adding only 1 point if hole of 1 point
      while (ang < (nextang - incr)) {
        vpImagePoint iP;
        computePointOnEllipse(ang, iP);
        if ((!outOfImage(iP, 0, nbrows, nbcols)) && inRoiMask(m_mask, iP.get_i(), iP.get_j())) {
          double theta = computeTheta(iP);
          vpMeSite pix;
          pix.init(iP.get_i(), iP.get_j(), theta);
          pix.setDisplay(m_selectDisplay);
          pix.setState(vpMeSite::NO_SUPPRESSION);
          double convolution = pix.convolution(I, m_me);
          double contrastThreshold = fabs(convolution) * marginRatio;
          pix.setContrastThreshold(contrastThreshold, *m_me);
          pix.track(I, m_me, false);
          if (pix.getState() == vpMeSite::NO_SUPPRESSION) { // good point
            ++nb_pts_added;
            iP.set_ij(pix.get_ifloat(), pix.get_jfloat());
            double new_ang = computeAngleOnEllipse(iP);
            if ((new_ang - ang) > M_PI) {
              new_ang -= 2.0 * M_PI;
            }
            else if ((ang - new_ang) > M_PI) {
              new_ang += 2.0 * M_PI;
            }
            m_meList.insert(meList, pix);
            m_angleList.insert(angleList, new_ang);
            if (vpDEBUG_ENABLE(3)) {
              const unsigned int crossSize = 5;
              vpDisplay::displayCross(I, iP, crossSize, vpColor::blue);
            }
          }
        }
        ang += incr;
      }
    }
    ang = nextang;
    ++angleList;
    ++meList;
  }

  if (vpDEBUG_ENABLE(3)) {
    if (nb_pts_added > 0) {
      std::cout << "Number of added points from holes with angles: " << nb_pts_added << std::endl;
    }
  }

  // Add points in case two neighboring points are too far away
  angleList = m_angleList.begin();
  ang = *angleList;
  meList = m_meList.begin();
  vpMeSite pix1 = *meList;
  ++angleList;
  ++meList;
  while (meList != m_meList.end()) {
    double nextang = *angleList;
    vpMeSite pix2 = *meList;
    double dist = sqrt(((pix1.get_ifloat() - pix2.get_ifloat()) * (pix1.get_ifloat() - pix2.get_ifloat()))
                       + ((pix1.get_jfloat() - pix2.get_jfloat()) * (pix1.get_jfloat() - pix2.get_jfloat())));
    // Only one point is added if two neighboring points are too far away
    if (dist > (2.0 * m_me->getSampleStep())) {
      ang = (nextang + ang) / 2.0; // point added at mid angle
      vpImagePoint iP;
      computePointOnEllipse(ang, iP);
      if ((!outOfImage(iP, 0, nbrows, nbcols)) && inRoiMask(m_mask, iP.get_i(), iP.get_j())) {
        double theta = computeTheta(iP);
        vpMeSite pix;
        pix.init(iP.get_i(), iP.get_j(), theta);
        pix.setDisplay(m_selectDisplay);
        pix.setState(vpMeSite::NO_SUPPRESSION);
        double convolution = pix.convolution(I, m_me);
        double contrastThreshold = fabs(convolution) * marginRatio;
        pix.setContrastThreshold(contrastThreshold, *m_me);
        pix.track(I, m_me, false);
        if (pix.getState() == vpMeSite::NO_SUPPRESSION) { // good point
          ++nb_pts_added;
          iP.set_ij(pix.get_ifloat(), pix.get_jfloat());
          double new_ang = computeAngleOnEllipse(iP);
          if ((new_ang - ang) > M_PI) {
            new_ang -= 2.0 * M_PI;
          }
          else if ((ang - new_ang) > M_PI) {
            new_ang += 2.0 * M_PI;
          }
          m_meList.insert(meList, pix);
          m_angleList.insert(angleList, new_ang);
          if (vpDEBUG_ENABLE(3)) {
            const unsigned int crossSize = 5;
            vpDisplay::displayCross(I, iP, crossSize, vpColor::blue);
          }
        }
      }
    }
    ang = nextang;
    pix1 = pix2;
    ++angleList;
    ++meList;
  }

  if (vpDEBUG_ENABLE(3)) {
    if (nb_pts_added > 0) {
      std::cout << "Number of added points from holes : " << nb_pts_added << std::endl;
      angleList = m_angleList.begin();
      while (angleList != m_angleList.end()) {
        ang = *angleList;
        std::cout << "ang = " << vpMath::deg(ang) << std::endl;
        ++angleList;
      }
    }
  }

  // Try to fill the first extremity: from alpha_min - incr to alpha1 + incr/2
  meList = m_meList.begin();
  pix1 = *meList;
  unsigned int nbpts = 0;
  // Add - incr/2.0 to avoid being too close to 0
  if ((m_alphamin - m_alpha1 - (incr / 2.0)) > 0.0) {
    nbpts = static_cast<unsigned int>(floor((m_alphamin - m_alpha1 - (incr / 2.0)) / incr));
  }
  ang = m_alphamin - incr;
  for (unsigned int i = 0; i < nbpts; ++i) {
    vpImagePoint iP;
    computePointOnEllipse(ang, iP);
    if ((!outOfImage(iP, 0, nbrows, nbcols)) && inRoiMask(m_mask, iP.get_i(), iP.get_j())) {
      double theta = computeTheta(iP);
      vpMeSite pix;
      pix.init(iP.get_i(), iP.get_j(), theta);
      pix.setDisplay(m_selectDisplay);
      pix.setState(vpMeSite::NO_SUPPRESSION);
      //pix.setContrastThreshold(pix1.getContrastThreshold(), *m_me);
      double convolution = pix.convolution(I, m_me);
      double contrastThreshold = fabs(convolution) * marginRatio;
      pix.setContrastThreshold(contrastThreshold, *m_me);
      pix.track(I, m_me, false);
      if (pix.getState() == vpMeSite::NO_SUPPRESSION) {
        ++nb_pts_added;
        iP.set_ij(pix.get_ifloat(), pix.get_jfloat());
        double new_ang = computeAngleOnEllipse(iP);
        if ((new_ang - ang) > M_PI) {
          new_ang -= 2.0 * M_PI;
        }
        else if ((ang - new_ang) > M_PI) {
          new_ang += 2.0 * M_PI;
        }
        m_meList.push_front(pix);
        m_angleList.push_front(new_ang);
        if (vpDEBUG_ENABLE(3)) {
          const unsigned int crossSize = 5;
          vpDisplay::displayCross(I, iP, crossSize, vpColor::blue);
          std::cout << "Add extremity 1, ang = " << vpMath::deg(new_ang) << std::endl;
        }
      }
    }
    ang -= incr;
  }

  if (vpDEBUG_ENABLE(3)) {
    if (nb_pts_added > 0) {
      std::cout << "Number of added points from holes and first extremity : " << nb_pts_added << std::endl;
    }
  }

  // Try to fill the second extremity: from alphamax + incr to alpha2 - incr/2
  pix1 = m_meList.back();
  nbpts = 0;
  if ((m_alpha2 - (incr / 2.0) - m_alphamax) > 0.0) {
    nbpts = static_cast<unsigned int>(floor((m_alpha2 - (incr / 2.0) - m_alphamax) / incr));
  }

  ang = m_alphamax + incr;
  for (unsigned int i = 0; i < nbpts; ++i) {
    vpImagePoint iP;
    computePointOnEllipse(ang, iP);
    if ((!outOfImage(iP, 0, nbrows, nbcols)) && inRoiMask(m_mask, iP.get_i(), iP.get_j())) {
      double theta = computeTheta(iP);
      vpMeSite pix;
      pix.init(iP.get_i(), iP.get_j(), theta);
      pix.setDisplay(m_selectDisplay);
      pix.setState(vpMeSite::NO_SUPPRESSION);
      double convolution = pix.convolution(I, m_me);
      double contrastThreshold = fabs(convolution) * marginRatio;
      pix.setContrastThreshold(contrastThreshold, *m_me);
      pix.track(I, m_me, false);
      if (pix.getState() == vpMeSite::NO_SUPPRESSION) {
        ++nb_pts_added;
        iP.set_ij(pix.get_ifloat(), pix.get_jfloat());
        double new_ang = computeAngleOnEllipse(iP);
        if ((new_ang - ang) > M_PI) {
          new_ang -= 2.0 * M_PI;
        }
        else if ((ang - new_ang) > M_PI) {
          new_ang += 2.0 * M_PI;
        }
        m_meList.push_back(pix);
        m_angleList.push_back(new_ang);
        if (vpDEBUG_ENABLE(3)) {
          const unsigned int crossSize = 5;
          vpDisplay::displayCross(I, iP, crossSize, vpColor::blue);
          std::cout << "Add extremity 2, ang = " << vpMath::deg(new_ang) << std::endl;
        }
      }
    }
    ang += incr;
  }
  m_me->setRange(memory_range);

  if (m_meList.size() != m_angleList.size()) {
    // Should never occur
    throw(vpException(vpException::fatalError, "Lists are not coherent in vpMeEllipse::plugHoles(): nb MEs %ld, nb ang %ld",
                      m_meList.size(), m_angleList.size()));
  }

  if (vpDEBUG_ENABLE(3)) {
    if (nb_pts_added > 0) {
      std::cout << "In plugHoles(): nb of added points : " << nb_pts_added << std::endl;
    }
  }
  return nb_pts_added;
}

void vpMeEllipse::leastSquare(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &iP)
{
  double um = I.getWidth() / 2.;
  double vm = I.getHeight() / 2.;
  unsigned int n = static_cast<unsigned int>(iP.size());

  if (m_trackCircle) { // we track a circle
    const unsigned int circleDims = 3;
    if (n < circleDims) {
      throw(vpException(vpException::dimensionError, "Not enough points to compute the circle"));
    }
    // System A x = b to be solved by least squares
    // with A = (u v 1), b = (u^2 + v^2) and x = (2xc, 2yc, r^2-xc^2-yc^2)

    vpMatrix A(n, 3);
    vpColVector b(n);

    for (unsigned int k = 0; k < n; ++k) {
      // normalization so that (u,v) in [-1;1]
      double u = (iP[k].get_u() - um) / um;
      double v = (iP[k].get_v() - vm) / um; // um here to not deform the circle
      A[k][0] = u;
      A[k][1] = v;
      A[k][2] = 1.0;
      b[k] = (u * u) + (v * v);
    }
    vpColVector x(3);
    x = A.solveBySVD(b);
    // A circle is a particular ellipse. Going from x for circle to K for ellipse
    // using inverse normalization to go back to pixel values
    double ratio = vm / um;
    m_K[0] = (m_K[1] = 1.0 / (um * um));
    m_K[2] = 0.0;
    m_K[3] = -(1.0 + (x[0] / 2.0)) / um;
    m_K[4] = -(ratio + (x[1] / 2.0)) / um;
    m_K[5] = -x[2] + 1.0 + (ratio * ratio) + x[0] + (ratio * x[1]);
  }
  else { // we track an ellipse
    if (n < 5) {
      throw(vpException(vpException::dimensionError, "Not enough points to compute the ellipse"));
    }
    // Homogeneous system A x = 0  ; x is the nullspace of A
    // K0 u^2 + K1 v^2 + 2 K2 u v + 2 K3 u + 2 K4 v + K5 = 0
    // A = (u^2 v^2 2uv 2u 2v 1), x = (K0 K1 K2 K3 K4 K5)^T

    // It would be a bad idea to solve the same system using A x = b where
    // A = (u^2 v^2 2uv 2u 2v), b = (-1), x = (K0 K1 K2 K3 K4)^T since it
    // cannot consider the case where the origin belongs to the ellipse.
    // Another possibility would be to consider K0+K1=1 which is always valid,
    // leading to the system A x = b where
    // A = (u^2-v^2 2uv 2u 2v 1), b = (-v^2), x = (K0 K2 K3 K4 K5)^T

    vpMatrix A(n, 6);

    for (unsigned int k = 0; k < n; ++k) {
      // Normalization so that (u,v) in [-1;1]
      double u = (iP[k].get_u() - um) / um;
      double v = (iP[k].get_v() - vm) / vm;
      A[k][0] = u * u;
      A[k][1] = v * v;
      A[k][2] = 2.0 * u * v;
      A[k][3] = 2.0 * u;
      A[k][4] = 2.0 * v;
      A[k][5] = 1.0;
    }
    vpMatrix KerA;
    unsigned int dim = A.nullSpace(KerA, 1);
    if (dim > 1) { // case with less than 5 independent points
      throw(vpException(vpMatrixException::rankDeficient, "Linear system for computing the ellipse equation ill conditioned"));
    }
    unsigned int nbRows = m_K.getRows();
    for (unsigned int i = 0; i < nbRows; ++i) {
      m_K[i] = KerA[i][0];
    }

    // inverse normalization
    m_K[0] *= vm / um;
    m_K[1] *= um / vm;
    m_K[3] = (m_K[3] * vm) - (m_K[0] * um) - (m_K[2] * vm);
    m_K[4] = (m_K[4] * um) - (m_K[1] * vm) - (m_K[2] * um);
    m_K[5] = (m_K[5] * um * vm) - (m_K[0] * um * um) - (m_K[1] * vm * vm) - (2.0 * m_K[2] * um * vm) - (2.0 * m_K[3] * um) - (2.0 * m_K[4] * vm);
  }
  getParameters();
}

unsigned int vpMeEllipse::leastSquareRobust(const vpImage<unsigned char> &I)
{
  double um = I.getWidth() / 2.;
  double vm = I.getHeight() / 2.;

  const unsigned int nos = numberOfSignal();
  unsigned int k = 0; // count the number of tracked MEs

  vpColVector w(nos);
  w = 1.0;
  // Note that the (nos-k) last rows of w are not used. Hopefully, this is not an issue.

  if (m_trackCircle) { // we track a circle
    // System A x = b to be solved by least squares
    // with A = (u v 1), b = (u^2 + v^2) and x = (2xc, 2yc, r^2-xc^2-yc^2)

    // Note that the (nos-k) last rows of A, b, xp and yp are not used.
    // Hopefully, this is not an issue.
    vpMatrix A(nos, 3);
    vpColVector b(nos);

    // Useful to compute the weights in the robust estimation
    vpColVector xp(nos), yp(nos);
    std::list<vpMeSite>::const_iterator end = m_meList.end();

    for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != end; ++it) {
      vpMeSite p_me = *it;
      if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
        // from (i,j) to (u,v) frame + normalization so that (u,v) in [-1;1]
        double u = (p_me.get_jfloat() - um) / um;
        double v = (p_me.get_ifloat() - vm) / um; // um to not deform the circle
        A[k][0] = u;
        A[k][1] = v;
        A[k][2] = 1.0;
        b[k] = (u * u) + (v * v);
        // Useful to compute the weights in the robust estimation
        xp[k] = p_me.get_jfloat();
        yp[k] = p_me.get_ifloat();

        ++k;
      }
    }

    const unsigned int minRequiredNbMe = 3;
    if (k < minRequiredNbMe) {
      throw(vpException(vpException::dimensionError, "Not enough moving edges %d / %d to track the circle ",
                        k, m_meList.size()));
    }

    vpRobust r;
    r.setMinMedianAbsoluteDeviation(1.0); // Image noise in pixels for the algebraic distance

    unsigned int iter = 0;
    double var = 1.0;
    vpColVector x(3);
    vpMatrix DA(k, 3);
    vpColVector Db(k);
    vpColVector xg_prev(2);
    xg_prev = -10.0;

    // stop after 4 it or if cog variation between 2 it is more than 1 pixel
    const unsigned int maxNbIter = 4;
    const unsigned int widthDA = DA.getCols();
    while ((iter < maxNbIter) && (var > 0.1)) {
      for (unsigned int i = 0; i < k; ++i) {
        for (unsigned int j = 0; j < widthDA; ++j) {
          DA[i][j] = w[i] * A[i][j];
        }
        Db[i] = w[i] * b[i];
      }
      x = DA.solveBySVD(Db);

      // A circle is a particular ellipse. Going from x for circle to K for ellipse
      // using inverse normalization to go back to pixel values
      double ratio = vm / um;
      m_K[0] = (m_K[1] = 1.0 / (um * um));
      m_K[2] = 0.0;
      m_K[3] = -(1.0 + (x[0] / 2.0)) / um;
      m_K[4] = -(ratio + (x[1] / 2.0)) / um;
      m_K[5] = -x[2] + 1.0 + (ratio * ratio) + x[0] + (ratio * x[1]);

      getParameters();
      vpColVector xg(2);
      xg[0] = m_uc;
      xg[1] = m_vc;
      var = (xg - xg_prev).frobeniusNorm();
      xg_prev = xg;

      vpColVector residu(k); // near to geometric distance in pixel
      for (unsigned int i = 0; i < k; ++i) {
        double x = xp[i];
        double y = yp[i];
        double sign = (m_K[0] * x * x) + (m_K[1] * y * y) + (2. * m_K[2] * x * y) + (2. * m_K[3] * x) + (2. * m_K[4] * y) + m_K[5];
        vpImagePoint ip1, ip2;
        ip1.set_uv(x, y);
        double ang = computeAngleOnEllipse(ip1);
        computePointOnEllipse(ang, ip2);
        // residu = 0 if point is exactly on the ellipse, not otherwise
        if (sign > 0) {
          residu[i] = vpImagePoint::distance(ip1, ip2);
        }
        else {
          residu[i] = -vpImagePoint::distance(ip1, ip2);
        }
      }
      r.MEstimator(vpRobust::TUKEY, residu, w);

      ++iter;
    }
  }
  else { // we track an ellipse

    // Homogeneous system A x = 0  ; x is the nullspace of A
    // K0 u^2 + K1 v^2 + 2 K2 u v + 2 K3 u + 2 K4 v + K5 = 0
    // A = (u^2 v^2 2uv 2u 2v 1), x = (K0 K1 K2 K3 K4 K5)^T

    // It would be a bad idea to solve the same system using A x = b where
    // A = (u^2 v^2 2uv 2u 2v), b = (-1), x = (K0 K1 K2 K3 K4)^T since it
    // cannot consider the case where the origin belongs to the ellipse.
    // Another possibility would be to consider K0+K1=1 which is always valid,
    // leading to the system A x = b where
    // A = (u^2-v^2 2uv 2u 2v 1), b = (-v^2), x = (K0 K2 K3 K4 K5)^T

    vpMatrix A(nos, 6);
    // Useful to compute the weights in the robust estimation
    vpColVector xp(nos), yp(nos);
    std::list<vpMeSite>::const_iterator end = m_meList.end();

    for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != end; ++it) {
      vpMeSite p_me = *it;
      if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
        // from (i,j) to (u,v) frame + normalization so that (u,v) in [-1;1]
        double u = (p_me.get_jfloat() - um) / um;
        double v = (p_me.get_ifloat() - vm) / vm;
        A[k][0] = u * u;
        A[k][1] = v * v;
        A[k][2] = 2.0 * u * v;
        A[k][3] = 2.0 * u;
        A[k][4] = 2.0 * v;
        A[k][5] = 1.0;
        // Useful to compute the weights in the robust estimation
        xp[k] = p_me.get_jfloat();
        yp[k] = p_me.get_ifloat();

        ++k;
      }
    }

    const unsigned int minRequiredMe = 5;
    if (k < minRequiredMe) {
      throw(vpException(vpException::dimensionError, "Not enough moving edges to track the ellipse"));
    }

    vpRobust r;

    r.setMinMedianAbsoluteDeviation(1.0); // image noise in pixels for the geometrical distance
    unsigned int iter = 0;
    double var = 1.0;
    vpMatrix DA(k, 6);
    vpMatrix KerDA;
    vpColVector xg_prev(2);
    xg_prev = -10.0;

    // Stop after 4 iterations or if cog variation between 2 iterations is more than 0.1 pixel
    const unsigned int maxIter = 4;
    const unsigned int widthDA = DA.getCols();
    while ((iter < maxIter) && (var > 0.1)) {
      for (unsigned int i = 0; i < k; ++i) {
        for (unsigned int j = 0; j < widthDA; ++j) {
          DA[i][j] = w[i] * A[i][j];
        }
      }
      unsigned int dim = DA.nullSpace(KerDA, 1);
      if (dim > 1) { // case with less than 5 independent points
        throw(vpException(vpMatrixException::rankDeficient, "Linear system for computing the ellipse equation ill conditioned"));
      }


      for (unsigned int i = 0; i < 6; ++i) {
        m_K[i] = KerDA[i][0]; // norm(K) = 1
      }

      // inverse normalization
      m_K[0] *= vm / um;
      m_K[1] *= um / vm;
      m_K[3] = (m_K[3] * vm) - (m_K[0] * um) - (m_K[2] * vm);
      m_K[4] = (m_K[4] * um) - (m_K[1] * vm) - (m_K[2] * um);
      m_K[5] = (m_K[5] * um * vm) - (m_K[0] * um * um) - (m_K[1] * vm * vm) - (2.0 * m_K[2] * um * vm) - (2.0 * m_K[3] * um) - (2.0 * m_K[4] * vm);

      getParameters(); // since a, b, and e are used just after
      vpColVector xg(2);
      xg[0] = m_uc;
      xg[1] = m_vc;
      var = (xg - xg_prev).frobeniusNorm();
      xg_prev = xg;

      vpColVector residu(k);
      for (unsigned int i = 0; i < k; ++i) {
        double x = xp[i];
        double y = yp[i];
        double sign = (m_K[0] * x * x) + (m_K[1] * y * y) + (2. * m_K[2] * x * y) + (2. * m_K[3] * x) + (2. * m_K[4] * y) + m_K[5];
        vpImagePoint ip1, ip2;
        ip1.set_uv(x, y);
        double ang = computeAngleOnEllipse(ip1);
        computePointOnEllipse(ang, ip2);
        // residu = 0 if point is exactly on the ellipse, not otherwise
        if (sign > 0) {
          residu[i] = vpImagePoint::distance(ip1, ip2);
        }
        else {
          residu[i] = -vpImagePoint::distance(ip1, ip2);
        }
      }
      r.MEstimator(vpRobust::TUKEY, residu, w);

      ++iter;
    }
  } // end of case ellipse

  // Remove bad points and outliers from the lists
  // Modify the angle to order the list
  double previous_ang = -4.0 * M_PI;
  k = 0;
  std::list<double>::iterator angleList = m_angleList.begin();
  std::list<vpMeSite>::iterator end = m_meList.end();
  std::list<vpMeSite>::iterator meList = m_meList.begin();
  while (meList != end) {
    vpMeSite p_me = *meList;
    if (p_me.getState() != vpMeSite::NO_SUPPRESSION) {
      // points not selected as me
      double ang = *angleList;
      meList = m_meList.erase(meList);
      angleList = m_angleList.erase(angleList);
      if (vpDEBUG_ENABLE(3)) {
        vpImagePoint iP;
        iP.set_ij(p_me.m_ifloat, p_me.m_jfloat);
        printf("point %d not me i : %.0f , j : %0.f, ang = %lf\n", k, p_me.get_ifloat(), p_me.get_jfloat(), vpMath::deg(ang));
        const unsigned int crossSize = 10;
        vpDisplay::displayCross(I, iP, crossSize, vpColor::blue, 1);
      }
    }
    else {
      if (w[k] < m_thresholdWeight) { // outlier
        double ang = *angleList;
        meList = m_meList.erase(meList);
        angleList = m_angleList.erase(angleList);
        if (vpDEBUG_ENABLE(3)) {
          vpImagePoint iP;
          iP.set_ij(p_me.m_ifloat, p_me.m_jfloat);
          printf("point %d outlier i : %.0f , j : %0.f, ang = %lf, poids : %lf\n",
                 k, p_me.get_ifloat(), p_me.get_jfloat(), vpMath::deg(ang), w[k]);
          const unsigned int crossSize = 10;
          vpDisplay::displayCross(I, iP, crossSize, vpColor::cyan, 1);
        }
      }
      else { //  good point
        double ang = *angleList;
        vpImagePoint iP;
        iP.set_ij(p_me.m_ifloat, p_me.m_jfloat);
        double new_ang = computeAngleOnEllipse(iP);
        if ((new_ang - ang) > M_PI) {
          new_ang -= 2.0 * M_PI;
        }
        else if ((ang - new_ang) > M_PI) {
          new_ang += 2.0 * M_PI;
        }
        previous_ang = new_ang;
        *angleList = new_ang;
        ++meList;
        ++angleList;
        if (vpDEBUG_ENABLE(3)) {
          printf("point %d inlier i : %.0f , j : %0.f, poids : %lf\n", k, p_me.get_ifloat(), p_me.get_jfloat(), w[k]);
          const unsigned int crossSize = 10;
          vpDisplay::displayCross(I, iP, crossSize, vpColor::cyan, 1);
        }
      }
      ++k; // k contains good points and outliers (used for w[k])
    }
  }

  if (m_meList.size() != m_angleList.size()) {
    // Should never occur
    throw(vpException(vpException::fatalError, "Lists are not coherent in vpMeEllipse::leastSquareRobust(): nb MEs %ld, nb ang %ld",
                      m_meList.size(), m_angleList.size()));
  }

  //  Manage the list so that all new angles belong to [0;2Pi]
  bool nbdeb = false;
  std::list<double> finAngle;
  finAngle.clear();
  std::list<vpMeSite> finMe;
  finMe.clear();
  std::list<double>::iterator debutAngleList;
  std::list<vpMeSite>::iterator debutMeList;
  angleList = m_angleList.begin();
  meList = m_meList.begin();
  end = m_meList.end();
  while (meList != end) {
    vpMeSite p_me = *meList;
    double ang = *angleList;

    // Move these ones to another list to be added at the end
    if (ang < m_alpha1) {
      ang += 2.0 * M_PI;
      angleList = m_angleList.erase(angleList);
      finAngle.push_back(ang);
      meList = m_meList.erase(meList);
      finMe.push_back(p_me);
    }
    // Moved at the beginning of  the list
    else if (ang > m_alpha2) {
      ang -= 2.0 * M_PI;
      angleList = m_angleList.erase(angleList);
      meList = m_meList.erase(meList);
      if (!nbdeb) {
        m_angleList.push_front(ang);
        debutAngleList = m_angleList.begin();
        ++debutAngleList;

        m_meList.push_front(p_me);
        debutMeList = m_meList.begin();
        ++debutMeList;

        nbdeb = true;
      }
      else {
        debutAngleList = m_angleList.insert(debutAngleList, ang);
        ++debutAngleList;
        debutMeList = m_meList.insert(debutMeList, p_me);
        ++debutMeList;
      }
    }
    else {
      ++angleList;
      ++meList;
    }
  }
  // Fuse the lists
  angleList = m_angleList.end();
  m_angleList.splice(angleList, finAngle);
  meList = m_meList.end();
  m_meList.splice(meList, finMe);

  unsigned int numberOfGoodPoints = 0;
  previous_ang = -4.0 * M_PI;

  // Perimeter of the ellipse using Ramanujan formula
  double perim = M_PI * ((3.0 * (m_a + m_b)) - sqrt(((3.0 * m_a) + m_b) * (m_a + (3.0 * m_b))));
  unsigned int nb_pt = static_cast<unsigned int>(floor(perim / m_me->getSampleStep()));
  double incr = (2.0 * M_PI) / nb_pt;
  // Update of the expected density
  if (!m_trackArc) { // number of points for a complete ellipse
    m_expectedDensity = nb_pt;
  }
  else { // number of points for an arc of ellipse
    m_expectedDensity *= static_cast<unsigned int>(floor((perim / m_me->getSampleStep()) * ((m_alpha2 - m_alpha1) / (2.0 * M_PI))));
  }

  // Keep only the points  in the interval [alpha1 ; alpha2]
  // and those  that are not too close
  angleList = m_angleList.begin();
  end = m_meList.end();
  meList = m_meList.begin();
  while (meList != end) {
    vpMeSite p_me = *meList;
    double new_ang = *angleList;
    if ((new_ang >= m_alpha1) && (new_ang <= m_alpha2)) {
      if ((new_ang - previous_ang) >= (0.6 * incr)) {
        previous_ang = new_ang;
        ++numberOfGoodPoints;
        ++meList;
        ++angleList;
        if (vpDEBUG_ENABLE(3)) {
          vpImagePoint iP;
          iP.set_ij(p_me.m_ifloat, p_me.m_jfloat);
          const unsigned int crossSize = 10;
          vpDisplay::displayCross(I, iP, crossSize, vpColor::red, 1);
          printf("In LQR: angle : %lf, i = %.0lf, j = %.0lf\n", vpMath::deg(new_ang), iP.get_i(), iP.get_j());
        }
      }
      else {
        meList = m_meList.erase(meList);
        angleList = m_angleList.erase(angleList);
        if (vpDEBUG_ENABLE(3)) {
          vpImagePoint iP;
          iP.set_ij(p_me.m_ifloat, p_me.m_jfloat);
          const unsigned int crossSize = 10;
          vpDisplay::displayCross(I, iP, crossSize, vpColor::orange, 1);
          printf("too near : angle  %lf, i %.0f , j : %0.f\n", vpMath::deg(new_ang), p_me.get_ifloat(), p_me.get_jfloat());
        }
      }
    }
    else { // point not in the interval [alpha1 ; alpha2]
      meList = m_meList.erase(meList);
      angleList = m_angleList.erase(angleList);
      if (vpDEBUG_ENABLE(3)) {
        vpImagePoint iP;
        iP.set_ij(p_me.m_ifloat, p_me.m_jfloat);
        const unsigned int crossSize = 10;
        vpDisplay::displayCross(I, iP, crossSize, vpColor::green, 1);
        printf("not in interval: angle : %lf, i %.0f , j : %0.f\n", vpMath::deg(new_ang), p_me.get_ifloat(), p_me.get_jfloat());
      }
    }
  }

  if ((m_meList.size() != numberOfGoodPoints) || (m_angleList.size() != numberOfGoodPoints)) {
    // Should never occur
    throw(vpException(vpException::fatalError, "Lists are not coherent at the end of vpMeEllipse::leastSquareRobust(): nb goog MEs %d and %ld, nb ang %ld",
                      numberOfGoodPoints, m_meList.size(), m_angleList.size()));
  }

  // set extremities of the angle list
  m_alphamin = m_angleList.front();
  m_alphamax = m_angleList.back();

  if (vpDEBUG_ENABLE(3)) {
    printf("alphamin : %lf, alphamax : %lf\n", vpMath::deg(m_alphamin), vpMath::deg(m_alphamax));
    printf("Fin leastSquareRobust : nb pts ok  = %d \n", numberOfGoodPoints);
  }

  return numberOfGoodPoints;
}

void vpMeEllipse::display(const vpImage<unsigned char> &I, const vpColor &col, unsigned int thickness)
{
  vpMeEllipse::displayEllipse(I, m_iPc, m_a, m_b, m_e, m_alpha1, m_alpha2, col, thickness);
}

void vpMeEllipse::initTracking(const vpImage<unsigned char> &I, bool trackCircle, bool trackArc)
{
  unsigned int n = 5; // by default, 5 points for an ellipse
  const unsigned int nForCircle = 3;
  m_trackCircle = trackCircle;
  if (trackCircle) {
    n = nForCircle;
  }
  std::vector<vpImagePoint> iP(n);
  m_trackArc = trackArc;

  vpDisplay::flush(I);

  if (m_trackCircle) {
    if (m_trackArc) {
      std::cout << "First and third points specify the extremities of the arc of circle (clockwise)" << std::endl;
    }
    for (unsigned int k = 0; k < n; ++k) {
      std::cout << "Click point " << (k + 1) << "/" << n << " on the circle " << std::endl;
      vpDisplay::getClick(I, iP[k], true);
      const unsigned int crossSize = 10;
      vpDisplay::displayCross(I, iP[k], crossSize, vpColor::red);
      vpDisplay::flush(I);
      std::cout << iP[k] << std::endl;
    }
  }
  else {
    if (m_trackArc) {
      std::cout << "First and fifth points specify the extremities of the arc of ellipse (clockwise)" << std::endl;
    }
    for (unsigned int k = 0; k < n; ++k) {
      std::cout << "Click point " << (k + 1) << "/" << n << " on the ellipse " << std::endl;
      vpDisplay::getClick(I, iP[k], true);
      const unsigned int crossSize = 10;
      vpDisplay::displayCross(I, iP[k], crossSize, vpColor::red);
      vpDisplay::flush(I);
      std::cout << iP[k] << std::endl;
    }
  }
  initTracking(I, iP, trackCircle, trackArc);
}

void vpMeEllipse::initTracking(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &iP,
                               bool trackCircle, bool trackArc)
{
  m_trackArc = trackArc;
  m_trackCircle = trackCircle;
  // useful for sample(I):
  leastSquare(I, iP);
  if (trackArc) {
    // useful for track(I):
    m_iP1 = iP.front();
    m_iP2 = iP.back();
    // useful for sample(I):
    m_alpha1 = computeAngleOnEllipse(m_iP1);
    m_alpha2 = computeAngleOnEllipse(m_iP2);
    if ((m_alpha2 <= m_alpha1) || (std::fabs(m_alpha2 - m_alpha1) < m_arcEpsilon)) {
      m_alpha2 += 2.0 * M_PI;
    }
  }
  else {
    m_alpha1 = 0.0;
    m_alpha2 = 2.0 * M_PI;
    // useful for track(I):
    vpImagePoint ip;
    computePointOnEllipse(m_alpha1, ip);
    m_iP1 = (m_iP2 = ip);
  }

  sample(I);
  track(I);
  vpMeTracker::display(I);
  vpDisplay::flush(I);
}

void vpMeEllipse::initTracking(const vpImage<unsigned char> &I, const vpColVector &param, vpImagePoint *pt1,
                               const vpImagePoint *pt2, bool trackCircle)
{
  m_trackCircle = trackCircle;
  if ((pt1 != nullptr) && (pt2 != nullptr)) {
    m_trackArc = true;
  }
  // useful for sample(I) : uc, vc, a, b, e, Ki, alpha1, alpha2
  m_uc = param[0];
  m_vc = param[1];
  m_n20 = param[2];
  m_n11 = param[3];
  m_n02 = param[4];
  computeAbeFromNij();
  computeKiFromNij();

  if (m_trackArc) {
    m_alpha1 = computeAngleOnEllipse(*pt1);
    m_alpha2 = computeAngleOnEllipse(*pt2);
    if ((m_alpha2 <= m_alpha1) || (std::fabs(m_alpha2 - m_alpha1) < m_arcEpsilon)) {
      m_alpha2 += 2.0 * M_PI;
    }
    // useful for track(I)
    m_iP1 = *pt1;
    m_iP2 = *pt2;
  }
  else {
    m_alpha1 = 0.0;
    m_alpha2 = 2.0 * M_PI;
    // useful for track(I)
    vpImagePoint ip;
    computePointOnEllipse(m_alpha1, ip);
    m_iP1 = (m_iP2 = ip);
  }
  // useful for display(I) so useless if no display before track(I)
  m_iPc.set_uv(m_uc, m_vc);

  sample(I);
  track(I);
  vpMeTracker::display(I);
  vpDisplay::flush(I);
}

/*!
  Track the ellipse in the image I.

  \param I : Image in which the ellipse appears.
*/
void vpMeEllipse::track(const vpImage<unsigned char> &I)
{
  vpMeTracker::track(I);

  // recompute alpha1 and alpha2 in case they have been changed by setEndPoints()
  if (m_trackArc) {
    m_alpha1 = computeAngleOnEllipse(m_iP1);
    m_alpha2 = computeAngleOnEllipse(m_iP2);
    if ((m_alpha2 <= m_alpha1) || (std::fabs(m_alpha2 - m_alpha1) < m_arcEpsilon)) {
      m_alpha2 += 2.0 * M_PI;
    }
  }
  // Compute the ellipse parameters from the tracked points, manage the lists,
  // and update the expected density (
  m_numberOfGoodPoints = leastSquareRobust(I);
  if (vpDEBUG_ENABLE(3)) {
    printf("1st step: nb of Good points %u, density %d, alphamin %lf, alphamax %lf\n",
           m_numberOfGoodPoints, m_expectedDensity,
           vpMath::deg(m_alphamin), vpMath::deg(m_alphamax));
  }

  if (plugHoles(I) > 0) {
    m_numberOfGoodPoints = leastSquareRobust(I); // if new points have been added, recompute the ellipse parameters and manage again the lists
    if (vpDEBUG_ENABLE(3)) {
      printf("2nd step: nb of Good points %u, density %d, alphamin %lf, alphamax %lf\n", m_numberOfGoodPoints, m_expectedDensity,
             vpMath::deg(m_alphamin), vpMath::deg(m_alphamax));
    }
  }

  const unsigned int minNbGoodPoints = 5;
  if (m_numberOfGoodPoints <= minNbGoodPoints) {
    if (vpDEBUG_ENABLE(3)) {
      printf("Before RESAMPLE !!! nb points %d \n", m_numberOfGoodPoints);
      printf("A click to continue \n");
      vpDisplay::flush(I);
      vpDisplay::getClick(I);
      vpDisplay::display(I);
    }
    sample(I);
    vpMeTracker::track(I);
    leastSquareRobust(I);
    if (vpDEBUG_ENABLE(3)) {
      printf("nb of Good points %u, density %d %lf, alphamin %lf, alphamax\n",
             m_numberOfGoodPoints, m_expectedDensity,
             vpMath::deg(m_alphamin), vpMath::deg(m_alphamax));
    }

    // Stop in case of failure after resample
    if (m_numberOfGoodPoints <= minNbGoodPoints) {
      throw(vpException(vpTrackingException::notEnoughPointError, "Impossible to track the ellipse, not enough features"));
    }
  }

  if (vpDEBUG_ENABLE(3)) {
    printParameters();
  }

  // remet a jour l'angle delta pour chaque vpMeSite de la liste
  updateTheta();
  // not in getParameters since computed only once for each image
  m_m00 = M_PI * m_a * m_b;

  // Useful only for tracking an arc of ellipse, but done to give them a value
  computePointOnEllipse(m_alpha1, m_iP1);
  computePointOnEllipse(m_alpha2, m_iP2);

  if (vpDEBUG_ENABLE(3)) {
    display(I, vpColor::red);
    vpMeTracker::display(I);
    vpDisplay::flush(I);
  }
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS

/*!
  \deprecated This static function is deprecated. Use rather vpMeEllipse::displayEllipse().

  Display the ellipse or the arc of ellipse thanks to the ellipse parameters.

  \param I : The image used as background.

  \param center : Center of the ellipse.

  \param A : Semi major axis of the ellipse.

  \param B : Semi minor axis of the ellipse.

  \param E : Angle made by the major axis and the u axis of the image frame
  \f$ (u,v) \f$ (in rad).

  \param smallalpha : Smallest \f$ alpha \f$ angle in rad (0 for a complete ellipse).

  \param highalpha : Highest \f$ alpha \f$ angle in rad (2 \f$ \Pi \f$ for a complete ellipse).

  \param color : Color used to display the ellipse.

  \param thickness : Thickness of the drawings.
*/
void vpMeEllipse::display(const vpImage<unsigned char> &I, const vpImagePoint &center, const double &A,
                          const double &B, const double &E, const double &smallalpha, const double &highalpha,
                          const vpColor &color, unsigned int thickness)
{
  vpMeEllipse::displayEllipse(I, center, A, B, E, smallalpha, highalpha, color, thickness);
}

/*!

  \deprecated This static function is deprecated. Use rather vpMeEllipse::displayEllipse().

  Display the ellipse or the arc of ellipse thanks to the ellipse parameters.

  \param I : The image used as background.

  \param center : Center of the ellipse

  \param A : Semi major axis of the ellipse.

  \param B : Semi minor axis of the ellipse.

  \param E : Angle made by the major axis and the u axis of the image frame
  \f$ (u,v) \f$ (in rad)

  \param smallalpha : Smallest \f$ alpha \f$ angle in rad  (0 for a complete ellipse)

  \param highalpha : Highest \f$ alpha \f$ angle in rad  (\f$ 2 \Pi \f$ for a complete ellipse)

  \param color : Color used to display th lines.

  \param thickness : Thickness of the drawings.

  \sa vpDisplay::displayEllipse()
*/
void vpMeEllipse::display(const vpImage<vpRGBa> &I, const vpImagePoint &center, const double &A, const double &B,
                          const double &E, const double &smallalpha, const double &highalpha,
                          const vpColor &color, unsigned int thickness)
{
  vpMeEllipse::displayEllipse(I, center, A, B, E, smallalpha, highalpha, color, thickness);
}
#endif // Deprecated


void vpMeEllipse::displayEllipse(const vpImage<unsigned char> &I, const vpImagePoint &center, const double &A,
                                 const double &B, const double &E, const double &smallalpha, const double &highalpha,
                                 const vpColor &color, unsigned int thickness)
{
  vpDisplay::displayEllipse(I, center, A, B, E, smallalpha, highalpha, false, color, thickness, true, true);
}

void vpMeEllipse::displayEllipse(const vpImage<vpRGBa> &I, const vpImagePoint &center, const double &A, const double &B,
                                 const double &E, const double &smallalpha, const double &highalpha,
                                 const vpColor &color, unsigned int thickness)
{
  vpDisplay::displayEllipse(I, center, A, B, E, smallalpha, highalpha, false, color, thickness, true, true);
}
