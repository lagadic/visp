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


vpMeEllipse::vpMeEllipse()
  : K(), iPc(), a(0.), b(0.), e(0.), iP1(), iP2(), alpha1(0), alpha2(2 * M_PI), ce(0.), se(0.), angle(), m00(0.),
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  mu11(0.), mu20(0.), mu02(0.), m10(0.), m01(0.), m11(0.), m02(0.), m20(0.),
#endif
  thresholdWeight(0.2), m_alphamin(0.), m_alphamax(0.), m_uc(0.), m_vc(0.), m_n20(0.), m_n11(0.), m_n02(0.), m_expectedDensity(0),
  m_numberOfGoodPoints(0), m_trackCircle(false), m_trackArc(false), m_arcEpsilon(1e-6)
{
  // Resize internal parameters vector
  // K0 u^2 + K1 v^2 + 2 K2 u v + 2 K3 u + 2 K4 v + K5 =  0
  K.resize(6);
  iP1.set_ij(0, 0);
  iP2.set_ij(0, 0);
}

vpMeEllipse::vpMeEllipse(const vpMeEllipse &me_ellipse)
  : vpMeTracker(me_ellipse), K(me_ellipse.K), iPc(me_ellipse.iPc), a(me_ellipse.a), b(me_ellipse.b), e(me_ellipse.e),
  iP1(me_ellipse.iP1), iP2(me_ellipse.iP2), alpha1(me_ellipse.alpha1), alpha2(me_ellipse.alpha2), ce(me_ellipse.ce),
  se(me_ellipse.se), angle(me_ellipse.angle), m00(me_ellipse.m00),
#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  mu11(me_ellipse.mu11), mu20(me_ellipse.mu20), mu02(me_ellipse.mu02), m10(me_ellipse.m10), m01(me_ellipse.m01),
  m11(me_ellipse.m11), m02(me_ellipse.m02), m20(me_ellipse.m20),
#endif
  thresholdWeight(me_ellipse.thresholdWeight),

  m_alphamin(me_ellipse.m_alphamin), m_alphamax(me_ellipse.m_alphamax), m_uc(me_ellipse.m_uc), m_vc(me_ellipse.m_vc),
  m_n20(me_ellipse.m_n20), m_n11(me_ellipse.m_n11), m_n02(me_ellipse.m_n02),
  m_expectedDensity(me_ellipse.m_expectedDensity), m_numberOfGoodPoints(me_ellipse.m_numberOfGoodPoints),
  m_trackCircle(me_ellipse.m_trackCircle), m_trackArc(me_ellipse.m_trackArc)
{ }

vpMeEllipse::~vpMeEllipse()
{
  list.clear();
  angle.clear();
}

double vpMeEllipse::computeTheta(const vpImagePoint &iP) const
{
  double u = iP.get_u();
  double v = iP.get_v();

  return (computeTheta(u, v));
}

double vpMeEllipse::computeTheta(double u, double v) const
{
  double A = K[0] * u + K[2] * v + K[3];
  double B = K[1] * v + K[2] * u + K[4];

  double theta = atan2(B, A); // Angle between the tangent and the u axis.
  if (theta < 0) {            // theta in [0;Pi]  // FC : pourquoi ? pour me sans doute
    theta += M_PI;
  }
  return (theta);
}

void vpMeEllipse::updateTheta()
{
  vpMeSite p_me;
  vpImagePoint iP;
  for (std::list<vpMeSite>::iterator it = list.begin(); it != list.end(); ++it) {
    p_me = *it;
    // (i,j) frame used for vpMESite
    iP.set_ij(p_me.ifloat, p_me.jfloat);
    p_me.alpha = computeTheta(iP);
    *it = p_me;
  }
}

void vpMeEllipse::computePointOnEllipse(const double angle, vpImagePoint &iP)
{
  // Two versions are available. If you change from one version to the other
  // one, do not forget to change also the reciprocal function
  // computeAngleOnEllipse() just below and, for a correct display of an arc
  // of ellipse, adapt vpMeEllipse::display() below and
  // vp_display_display_ellipse() in modules/core/src/display/vpDisplay_impl.h
  // so that the two extremities of the arc are correctly shown.

  // Version that gives a regular angular sampling on the ellipse, so less
  // points at its extremities
  /*
  double co = cos(angle);
  double si = sin(angle);
  double coef = a * b / sqrt(b * b * co * co + a * a * si * si);
  double u = co * coef;
  double v = si * coef;
  iP.set_u(uc + ce * u - se * v);
  iP.set_v(vc + se * u + ce * v);
  */

  // Version from "the two concentric circles" method that gives more points
  // at the ellipse extremities for a regular angle sampling. It is better to
  // display an ellipse, not necessarily to track it

  // (u,v) are the coordinates on the canonical centered ellipse;
  double u = a * cos(angle);
  double v = b * sin(angle);
  // a rotation of e and a translation by (uc,vc) are done
  // to get the coordinates of the point on the shifted ellipse
  iP.set_uv(m_uc + ce * u - se * v, m_vc + se * u + ce * v);
}

double vpMeEllipse::computeAngleOnEllipse(const vpImagePoint &pt) const
{
  // Two versions are available. If you change from one version to the other
  // one, do not forget to change also the reciprocal function
  // computePointOnEllipse() just above. Adapt also the display; see comment
  // at the beginning of computePointOnEllipse()

  // Regular angle sampling method
  /*
  double du = pt.get_u() - uc;
  double dv = pt.get_v() - vc;
  double ang = atan2(dv,du) - e;
  if (ang > M_PI) {
    ang -= 2.0 * M_PI;
  }
  else if (ang < -M_PI) {
    ang += 2.0 * M_PI;
  }
  */
  // for the "two concentric circles method" starting from the previous one
  // (just to remember the link between both methods:
  // tan(theta_2cc) = a/b tan(theta_rs))
  /*
  double co = cos(ang);
  double si = sin(ang);
  double coeff = 1.0/sqrt(b*b*co*co+a*a*si*si);
  si *= a*coeff;
  co *= b*coeff;
  ang = atan2(si,co);
  */
  // For the "two concentric circles" method starting from scratch
  double du = pt.get_u() - m_uc;
  double dv = pt.get_v() - m_vc;
  double co = (du * ce + dv * se) / a;
  double si = (-du * se + dv * ce) / b;
  double angle = atan2(si, co);

  return (angle);
}

void vpMeEllipse::computeAbeFromNij()
{
  double num = m_n20 - m_n02;
  double d = num * num + 4.0 * m_n11 * m_n11; // always >= 0
  if (d <= std::numeric_limits<double>::epsilon()) {
    e = 0.0; // case n20 = n02 and n11 = 0 : circle, e undefined
    ce = 1.0;
    se = 0.0;
    a = b = 2.0 * sqrt(m_n20);         // = sqrt(2.0*(n20+n02))
  }
  else {                             // real ellipse
    e = atan2(2.0 * m_n11, num) / 2.0; // e in [-Pi/2 ; Pi/2]
    ce = cos(e);
    se = sin(e);

    d = sqrt(d); // d in sqrt always >= 0
    num = m_n20 + m_n02;
    a = sqrt(2.0 * (num + d)); // term in sqrt always > 0
    b = sqrt(2.0 * (num - d)); // term in sqrt always > 0
  }
}

void vpMeEllipse::computeKiFromNij()
{
  K[0] = m_n02;
  K[1] = m_n20;
  K[2] = -m_n11;
  K[3] = m_n11 * m_vc - m_n02 * m_uc;
  K[4] = m_n11 * m_uc - m_n20 * m_vc;
  K[5] = m_n02 * m_uc * m_uc + m_n20 * m_vc * m_vc - 2.0 * m_n11 * m_uc * m_vc + 4.0 * (m_n11 * m_n11 - m_n20 * m_n02);
}

void vpMeEllipse::computeNijFromAbe()
{
  m_n20 = 0.25 * (a * a * ce * ce + b * b * se * se);
  m_n11 = 0.25 * se * ce * (a * a - b * b);
  m_n02 = 0.25 * (a * a * se * se + b * b * ce * ce);
}

void vpMeEllipse::getParameters()
{
  // Equations below from Chaumette PhD and TRO 2004 paper
  double num = K[0] * K[1] - K[2] * K[2]; // > 0 for an ellipse
  if (num <= 0) {
    throw(vpException(vpException::fatalError, "The points do not belong to an ellipse! num: %f", num));
  }

  m_uc = (K[2] * K[4] - K[1] * K[3]) / num;
  m_vc = (K[2] * K[3] - K[0] * K[4]) / num;
  iPc.set_uv(m_uc, m_vc);

  double d = (K[0] * m_uc * m_uc + K[1] * m_vc * m_vc + 2.0 * K[2] * m_uc * m_vc - K[5]) / (4.0 * num);
  m_n20 = K[1] * d; // always > 0
  m_n11 = -K[2] * d;
  m_n02 = K[0] * d; // always > 0

  computeAbeFromNij();

  // normalization so that K0 = n02, K1 = n20, etc (Eq (25) of TRO paper)
  d = m_n02 / K[0]; // fabs(K[0]) > 0
  for (unsigned int i = 0; i < 6; i++) {
    K[i] *= d;
  }
  if (vpDEBUG_ENABLE(3)) {
    printParameters();
  }
}

void vpMeEllipse::printParameters() const
{
  std::cout << "K :" << K.t() << std::endl;
  std::cout << "xc = " << m_uc << ", yc = " << m_vc << std::endl;
  std::cout << "n20 = " << m_n20 << ", n11 = " << m_n11 << ", n02 = " << m_n02 << std::endl;
  std::cout << "A = " << a << ", B = " << b << ", E (dg) " << vpMath::deg(e) << std::endl;
}

void vpMeEllipse::sample(const vpImage<unsigned char> &I, bool doNotTrack)
{
  // Warning: similar code in vpMbtMeEllipse::sample()
  if (!me) {
    throw(vpException(vpException::fatalError, "Moving edges on ellipse not initialized"));
  }
  // Delete old lists
  list.clear();
  angle.clear();

  int nbrows = static_cast<int>(I.getHeight());
  int nbcols = static_cast<int>(I.getWidth());
  // New version using distance for sampling
  if (std::fabs(me->getSampleStep()) <= std::numeric_limits<double>::epsilon()) {
    std::cout << "Warning: In vpMeEllipse::sample() ";
    std::cout << "function called with sample step = 0. We set it rather to 10 pixels";
    // std::cout << "function called with sample step = 0, set to 10 dg";
    me->setSampleStep(10.0);
  }
  // Perimeter of the ellipse using Ramanujan formula
  double perim = M_PI * (3.0 * (a + b) - sqrt((3.0 * a + b) * (a + 3.0 * b)));
  // Number of points for a complete ellipse
  unsigned int nb_pt = static_cast<unsigned int>(floor(perim / me->getSampleStep()));
  double incr = 2.0 * M_PI / nb_pt;
  // Compute of the expected density
  if (!m_trackArc) { // number of points for a complete ellipse
    m_expectedDensity = nb_pt;
  }
  else { // number of points for an arc of ellipse
    m_expectedDensity *= static_cast<unsigned int>(floor(perim / me->getSampleStep() * (alpha2 - alpha1) / (2.0 * M_PI)));
  }

  // Starting angle for sampling: new version to not start at 0
  double ang = alpha1 + incr / 2.0;

  // sample positions
  for (unsigned int i = 0; i < m_expectedDensity; i++) {
    vpImagePoint iP;
    computePointOnEllipse(ang, iP);
    // If point is in the image, add to the sample list
    // Check done in (i,j) frame)
    if (!outOfImage(vpMath::round(iP.get_i()), vpMath::round(iP.get_j()), 0, nbrows, nbcols)) {
      vpDisplay::displayCross(I, iP, 5, vpColor::red);

      double theta = computeTheta(iP);
      vpMeSite pix;
      // (i,j) frame used for vpMeSite
      pix.init(iP.get_i(), iP.get_j(), theta);
      pix.setDisplay(selectDisplay);
      pix.setState(vpMeSite::NO_SUPPRESSION);
      list.push_back(pix);
      angle.push_back(ang);
    }
    ang += incr;
  }

  if (!doNotTrack) {
    vpMeTracker::initTracking(I);
  }
}

unsigned int vpMeEllipse::plugHoles(const vpImage<unsigned char> &I)
{
  if (!me) {
    throw(vpException(vpException::fatalError, "Moving edges on ellipse tracking not initialized"));
  }
  unsigned int nb_pts_added = 0;
  int nbrows = static_cast<int>(I.getHeight());
  int nbcols = static_cast<int>(I.getWidth());

  unsigned int memory_range = me->getRange();
  me->setRange(2);

  // Perimeter of the ellipse using Ramanujan formula
  double perim = M_PI * (3.0 * (a + b) - sqrt((3.0 * a + b) * (a + 3.0 * b)));
  // Number of points for a complete ellipse
  unsigned int nb_pt = static_cast<unsigned int>(floor(perim / me->getSampleStep()));
  double incr = 2.0 * M_PI / nb_pt;

  // Detect holes and try to complete them
  // In this option, the sample step is used to complete the holes as much as possible
  std::list<double>::iterator angleList = angle.begin();
  std::list<vpMeSite>::iterator meList = list.begin();
  double ang = *angleList;
  ++angleList;
  ++meList;
  while (meList != list.end()) {
    double nextang = *angleList;
    if ((nextang - ang) > 2.0 * incr) { // A hole exists
      ang += incr;                      // next point to be checked
      // adding only 1 point if hole of 1 point
      while (ang < (nextang - incr)) {
        vpImagePoint iP;
        computePointOnEllipse(ang, iP);
        if (!outOfImage(vpMath::round(iP.get_i()), vpMath::round(iP.get_j()), 0, nbrows, nbcols)) {
          double theta = computeTheta(iP);
          vpMeSite pix;
          pix.init(iP.get_i(), iP.get_j(), theta);
          pix.setDisplay(selectDisplay);
          pix.setState(vpMeSite::NO_SUPPRESSION);
          pix.track(I, me, false);
          if (pix.getState() == vpMeSite::NO_SUPPRESSION) { // good point
            nb_pts_added++;
            iP.set_ij(pix.ifloat, pix.jfloat);
            double new_ang = computeAngleOnEllipse(iP);
            if ((new_ang - ang) > M_PI) {
              new_ang -= 2.0 * M_PI;
            }
            else if ((ang - new_ang) > M_PI) {
              new_ang += 2.0 * M_PI;
            }
            list.insert(meList, pix);
            angle.insert(angleList, new_ang);
            if (vpDEBUG_ENABLE(3)) {
              vpDisplay::displayCross(I, iP, 5, vpColor::blue);
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
  angleList = angle.begin();
  ang = *angleList;
  meList = list.begin();
  vpMeSite pix1 = *meList;
  ++angleList;
  ++meList;
  while (meList != list.end()) {
    double nextang = *angleList;
    vpMeSite pix2 = *meList;
    double dist = sqrt((pix1.ifloat - pix2.ifloat) * (pix1.ifloat - pix2.ifloat)
                       + (pix1.jfloat - pix2.jfloat) * (pix1.jfloat - pix2.jfloat));
    // Only one point is added if two neighboring points are too far away
    if (dist > 2.0 * me->getSampleStep()) {
      ang = (nextang + ang) / 2.0; // point added at mid angle
      vpImagePoint iP;
      computePointOnEllipse(ang, iP);
      if (!outOfImage(vpMath::round(iP.get_i()), vpMath::round(iP.get_j()), 0, nbrows, nbcols)) {
        double theta = computeTheta(iP);
        vpMeSite pix;
        pix.init(iP.get_i(), iP.get_j(), theta);
        pix.setDisplay(selectDisplay);
        pix.setState(vpMeSite::NO_SUPPRESSION);
        pix.track(I, me, false);
        if (pix.getState() == vpMeSite::NO_SUPPRESSION) { // good point
          nb_pts_added++;
          iP.set_ij(pix.ifloat, pix.jfloat);
          double new_ang = computeAngleOnEllipse(iP);
          if ((new_ang - ang) > M_PI) {
            new_ang -= 2.0 * M_PI;
          }
          else if ((ang - new_ang) > M_PI) {
            new_ang += 2.0 * M_PI;
          }
          list.insert(meList, pix);
          angle.insert(angleList, new_ang);
          if (vpDEBUG_ENABLE(3)) {
            vpDisplay::displayCross(I, iP, 5, vpColor::blue);
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
      angleList = angle.begin();
      while (angleList != angle.end()) {
        ang = *angleList;
        std::cout << "ang = " << vpMath::deg(ang) << std::endl;
        ++angleList;
      }
    }
  }

  // Try to fill the first extremity: from alpha_min - incr to alpha1 + incr/2
  unsigned int nbpts = 0;
  // Add - incr/2.0 to avoid being too close to 0
  if ((m_alphamin - alpha1 - incr / 2.0) > 0.0) {
    nbpts = static_cast<unsigned int>(floor((m_alphamin - alpha1 - incr / 2.0) / incr));
  }
  ang = m_alphamin - incr;
  for (unsigned int i = 0; i < nbpts; i++) {
    vpImagePoint iP;
    computePointOnEllipse(ang, iP);
    if (!outOfImage(vpMath::round(iP.get_i()), vpMath::round(iP.get_j()), 0, nbrows, nbcols)) {
      double theta = computeTheta(iP);
      vpMeSite pix;
      pix.init(iP.get_i(), iP.get_j(), theta);
      pix.setDisplay(selectDisplay);
      pix.setState(vpMeSite::NO_SUPPRESSION);
      pix.track(I, me, false);
      if (pix.getState() == vpMeSite::NO_SUPPRESSION) {
        nb_pts_added++;
        iP.set_ij(pix.ifloat, pix.jfloat);
        double new_ang = computeAngleOnEllipse(iP);
        if ((new_ang - ang) > M_PI) {
          new_ang -= 2.0 * M_PI;
        }
        else if ((ang - new_ang) > M_PI) {
          new_ang += 2.0 * M_PI;
        }
        list.push_front(pix);
        angle.push_front(new_ang);
        if (vpDEBUG_ENABLE(3)) {
          vpDisplay::displayCross(I, iP, 5, vpColor::blue);
          std::cout << "Add extremity 1, ang = " << vpMath::deg(new_ang) << std::endl;
        }
      }
    }
    ang -= incr;
  }

  if (vpDEBUG_ENABLE(3)) {
    if (nb_pts_added > 0) std::cout << "Number of added points from holes and first extremity : " << nb_pts_added << std::endl;
  }

  // Try to fill the second extremity: from alphamax + incr to alpha2 - incr/2
  nbpts = 0;
  if ((alpha2 - incr / 2.0 - m_alphamax) > 0.0) {
    nbpts = static_cast<unsigned int>(floor((alpha2 - incr / 2.0 - m_alphamax) / incr));
  }
  ang = m_alphamax + incr;
  for (unsigned int i = 0; i < nbpts; i++) {
    vpImagePoint iP;
    computePointOnEllipse(ang, iP);
    if (!outOfImage(vpMath::round(iP.get_i()), vpMath::round(iP.get_j()), 0, nbrows, nbcols)) {
      double theta = computeTheta(iP);
      vpMeSite pix;
      pix.init(iP.get_i(), iP.get_j(), theta);
      pix.setDisplay(selectDisplay);
      pix.setState(vpMeSite::NO_SUPPRESSION);
      pix.track(I, me, false);
      if (pix.getState() == vpMeSite::NO_SUPPRESSION) {
        nb_pts_added++;
        iP.set_ij(pix.ifloat, pix.jfloat);
        double new_ang = computeAngleOnEllipse(iP);
        if ((new_ang - ang) > M_PI) {
          new_ang -= 2.0 * M_PI;
        }
        else if ((ang - new_ang) > M_PI) {
          new_ang += 2.0 * M_PI;
        }
        list.push_back(pix);
        angle.push_back(new_ang);
        if (vpDEBUG_ENABLE(3)) {
          vpDisplay::displayCross(I, iP, 5, vpColor::blue);
          std::cout << "Add extremity 2, ang = " << vpMath::deg(new_ang) << std::endl;
        }
      }
    }
    ang += incr;
  }
  me->setRange(memory_range);

  if (vpDEBUG_ENABLE(3)) {
    if (nb_pts_added > 0) std::cout << "In plugHoles(): nb of added points : " << nb_pts_added << std::endl;
  }
  return nb_pts_added;
}

void vpMeEllipse::leastSquare(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &iP)
{
  double um = I.getWidth() / 2.;
  double vm = I.getHeight() / 2.;
  unsigned int n = static_cast<unsigned int>(iP.size());

  if (m_trackCircle) { // we track a circle
    if (n < 3) {
      throw(vpException(vpException::dimensionError, "Not enough points to compute the circle"));
    }
    // System A x = b to be solved by least squares
    // with A = (u v 1), b = (u^2 + v^2) and x = (2xc, 2yc, r^2-xc^2-yc^2)

    vpMatrix A(n, 3);
    vpColVector b(n);

    for (unsigned int k = 0; k < n; k++) {
      // normalization so that (u,v) in [-1;1]
      double u = (iP[k].get_u() - um) / um;
      double v = (iP[k].get_v() - vm) / um; // um here to not deform the circle
      A[k][0] = u;
      A[k][1] = v;
      A[k][2] = 1.0;
      b[k] = u * u + v * v;
    }
    vpColVector x(3);
    x = A.solveBySVD(b);
    // A circle is a particular ellipse. Going from x for circle to K for ellipse
    // using inverse normalization to go back to pixel values
    double ratio = vm / um;
    K[0] = K[1] = 1.0 / (um * um);
    K[2] = 0.0;
    K[3] = -(1.0 + x[0] / 2.0) / um;
    K[4] = -(ratio + x[1] / 2.0) / um;
    K[5] = -x[2] + 1.0 + ratio * ratio + x[0] + ratio * x[1];
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

    for (unsigned int k = 0; k < n; k++) {
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
    for (unsigned int i = 0; i < 6; i++)
      K[i] = KerA[i][0];

    // inverse normalization
    K[0] *= vm / um;
    K[1] *= um / vm;
    K[3] = K[3] * vm - K[0] * um - K[2] * vm;
    K[4] = K[4] * um - K[1] * vm - K[2] * um;
    K[5] = K[5] * um * vm - K[0] * um * um - K[1] * vm * vm - 2.0 * K[2] * um * vm - 2.0 * K[3] * um - 2.0 * K[4] * vm;
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

    for (std::list<vpMeSite>::const_iterator it = list.begin(); it != list.end(); ++it) {
      vpMeSite p_me = *it;
      if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
        // from (i,j) to (u,v) frame + normalization so that (u,v) in [-1;1]
        double u = (p_me.jfloat - um) / um;
        double v = (p_me.ifloat - vm) / um; // um to not deform the circle
        A[k][0] = u;
        A[k][1] = v;
        A[k][2] = 1.0;
        b[k] = u * u + v * v;
        // Useful to compute the weights in the robust estimation
        xp[k] = p_me.jfloat;
        yp[k] = p_me.ifloat;

        k++;
      }
    }
    if (k < 3) {
      throw(vpException(vpException::dimensionError, "Not enough moving edges %d / %d to track the circle ", k, list.size()));
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
    while ((iter < 4) && (var > 0.1)) {
      for (unsigned int i = 0; i < k; i++) {
        for (unsigned int j = 0; j < 3; j++) {
          DA[i][j] = w[i] * A[i][j];
        }
        Db[i] = w[i] * b[i];
      }
      x = DA.solveBySVD(Db);

      // A circle is a particular ellipse. Going from x for circle to K for ellipse
      // using inverse normalization to go back to pixel values
      double ratio = vm / um;
      K[0] = K[1] = 1.0 / (um * um);
      K[2] = 0.0;
      K[3] = -(1.0 + x[0] / 2.0) / um;
      K[4] = -(ratio + x[1] / 2.0) / um;
      K[5] = -x[2] + 1.0 + ratio * ratio + x[0] + ratio * x[1];

      getParameters();
      vpColVector xg(2);
      xg[0] = m_uc;
      xg[1] = m_vc;
      var = (xg - xg_prev).frobeniusNorm();
      xg_prev = xg;

      vpColVector residu(k); // near to geometric distance in pixel
      for (unsigned int i = 0; i < k; i++) {
        double x = xp[i];
        double y = yp[i];
        double sign = K[0] * x * x + K[1] * y * y + 2. * K[2] * x * y + 2. * K[3] * x + 2. * K[4] * y + K[5];
        vpImagePoint ip1, ip2;
        ip1.set_uv(x, y);
        double ang = computeAngleOnEllipse(ip1);
        computePointOnEllipse(ang, ip2);
        // residu = 0 if point is exactly on the ellipse, not otherwise
        if (sign > 0) residu[i] = vpImagePoint::distance(ip1, ip2);
        else residu[i] = -vpImagePoint::distance(ip1, ip2);
      }
      r.MEstimator(vpRobust::TUKEY, residu, w);

      iter++;
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

    for (std::list<vpMeSite>::const_iterator it = list.begin(); it != list.end(); ++it) {
      vpMeSite p_me = *it;
      if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
        // from (i,j) to (u,v) frame + normalization so that (u,v) in [-1;1]
        double u = (p_me.jfloat - um) / um;
        double v = (p_me.ifloat - vm) / vm;
        A[k][0] = u * u;
        A[k][1] = v * v;
        A[k][2] = 2.0 * u * v;
        A[k][3] = 2.0 * u;
        A[k][4] = 2.0 * v;
        A[k][5] = 1.0;
        // Useful to compute the weights in the robust estimation
        xp[k] = p_me.jfloat;
        yp[k] = p_me.ifloat;

        k++;
      }
    }
    if (k < 5) {
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
    while ((iter < 4) && (var > 0.1)) {
      for (unsigned int i = 0; i < k; i++) {
        for (unsigned int j = 0; j < 6; j++) {
          DA[i][j] = w[i] * A[i][j];
        }
      }
      unsigned int dim = DA.nullSpace(KerDA, 1);
      if (dim > 1) { // case with less than 5 independent points
        throw(vpException(vpMatrixException::rankDeficient, "Linear system for computing the ellipse equation ill conditioned"));
      }

      for (unsigned int i = 0; i < 6; i++)
        K[i] = KerDA[i][0]; // norm(K) = 1

      // inverse normalization
      K[0] *= vm / um;
      K[1] *= um / vm;
      K[3] = K[3] * vm - K[0] * um - K[2] * vm;
      K[4] = K[4] * um - K[1] * vm - K[2] * um;
      K[5] = K[5] * um * vm - K[0] * um * um - K[1] * vm * vm - 2.0 * K[2] * um * vm - 2.0 * K[3] * um - 2.0 * K[4] * vm;

      getParameters(); // since a, b, and e are used just after
      vpColVector xg(2);
      xg[0] = m_uc;
      xg[1] = m_vc;
      var = (xg - xg_prev).frobeniusNorm();
      xg_prev = xg;

      vpColVector residu(k);
      for (unsigned int i = 0; i < k; i++) {
        double x = xp[i];
        double y = yp[i];
        double sign = K[0] * x * x + K[1] * y * y + 2. * K[2] * x * y + 2. * K[3] * x + 2. * K[4] * y + K[5];
        vpImagePoint ip1, ip2;
        ip1.set_uv(x, y);
        double ang = computeAngleOnEllipse(ip1);
        computePointOnEllipse(ang, ip2);
        // residu = 0 if point is exactly on the ellipse, not otherwise
        if (sign > 0) residu[i] = vpImagePoint::distance(ip1, ip2);
        else residu[i] = -vpImagePoint::distance(ip1, ip2);
      }
      r.MEstimator(vpRobust::TUKEY, residu, w);

      iter++;
    }
  } // end of case ellipse

  // Remove bad points and outliers from the lists
  // Modify the angle to order the list
  double previous_ang = -4.0 * M_PI;
  k = 0;
  std::list<double>::iterator angleList = angle.begin();
  for (std::list<vpMeSite>::iterator meList = list.begin(); meList != list.end();) {
    vpMeSite p_me = *meList;
    if (p_me.getState() != vpMeSite::NO_SUPPRESSION) {
      // points not selected as me
      double ang = *angleList;
      meList = list.erase(meList);
      angleList = angle.erase(angleList);
      if (vpDEBUG_ENABLE(3)) {
        vpImagePoint iP;
        iP.set_ij(p_me.ifloat, p_me.jfloat);
        printf("point %d not me i : %.0f , j : %0.f, ang = %lf\n", k, p_me.ifloat, p_me.jfloat, vpMath::deg(ang));
        vpDisplay::displayCross(I, iP, 10, vpColor::blue, 1);
      }
    }
    else {
      if (w[k] < thresholdWeight) { // outlier
        double ang = *angleList;
        meList = list.erase(meList);
        angleList = angle.erase(angleList);
        if (vpDEBUG_ENABLE(3)) {
          vpImagePoint iP;
          iP.set_ij(p_me.ifloat, p_me.jfloat);
          printf("point %d outlier i : %.0f , j : %0.f, ang = %lf, poids : %lf\n",
                 k, p_me.ifloat, p_me.jfloat, vpMath::deg(ang), w[k]);
          vpDisplay::displayCross(I, iP, 10, vpColor::cyan, 1);
        }
      }
      else { //  good point
        double ang = *angleList;
        vpImagePoint iP;
        iP.set_ij(p_me.ifloat, p_me.jfloat);
        double new_ang = computeAngleOnEllipse(iP);
        if ((new_ang - ang) > M_PI) {
          new_ang -= 2.0 * M_PI;
        }
        else if ((ang - new_ang) > M_PI) {
          new_ang += 2.0 * M_PI;
        }
        *angleList = previous_ang = new_ang;
        ++meList;
        ++angleList;
        if (vpDEBUG_ENABLE(3)) {
          printf("point %d inlier i : %.0f , j : %0.f, poids : %lf\n", k, p_me.ifloat, p_me.jfloat, w[k]);
          vpDisplay::displayCross(I, iP, 10, vpColor::cyan, 1);
        }
      }
      k++;
    }
  }

  //  Manage the list so that all new angles belong to [0;2Pi]
  bool nbdeb = false;
  std::list<double> finAngle;
  finAngle.clear();
  std::list<vpMeSite> finMe;
  finMe.clear();
  std::list<double>::iterator debutAngleList;
  std::list<vpMeSite>::iterator debutMeList;
  angleList = angle.begin();
  std::list<vpMeSite>::iterator meList;
  for (meList = list.begin(); meList != list.end();) {
    vpMeSite p_me = *meList;
    double ang = *angleList;

    // Move these ones to another list to be added at the end
    if (ang < alpha1) {
      ang += 2.0 * M_PI;
      angleList = angle.erase(angleList);
      finAngle.push_back(ang);
      meList = list.erase(meList);
      finMe.push_back(p_me);
    }
    // Moved at the beginning of  the list
    else if (ang > alpha2) {
      ang -= 2.0 * M_PI;
      angleList = angle.erase(angleList);
      meList = list.erase(meList);
      if (!nbdeb) {
        angle.push_front(ang);
        debutAngleList = angle.begin();
        ++debutAngleList;

        list.push_front(p_me);
        debutMeList = list.begin();
        ++debutMeList;

        nbdeb = true;
      }
      else {
        debutAngleList = angle.insert(debutAngleList, ang);
        ++debutAngleList;
        debutMeList = list.insert(debutMeList, p_me);
        ++debutMeList;
      }
    }
    else {
      ++angleList;
      ++meList;
    }
  }
  // Fuse the lists
  angleList = angle.end();
  angle.splice(angleList, finAngle);
  meList = list.end();
  list.splice(meList, finMe);

  unsigned int numberOfGoodPoints = 0;
  previous_ang = -4.0 * M_PI;

  // Perimeter of the ellipse using Ramanujan formula
  double perim = M_PI * (3.0 * (a + b) - sqrt((3.0 * a + b) * (a + 3.0 * b)));
  unsigned int nb_pt = static_cast<unsigned int>(floor(perim / me->getSampleStep()));
  double incr = 2.0 * M_PI / nb_pt;
  // Update of the expected density
  if (!m_trackArc) { // number of points for a complete ellipse
    m_expectedDensity = nb_pt;
  }
  else { // number of points for an arc of ellipse
    m_expectedDensity *= static_cast<unsigned int>(floor(perim / me->getSampleStep() * (alpha2 - alpha1) / (2.0 * M_PI)));
  }

  // Keep only the points  in the interval [alpha1 ; alpha2]
  // and those  that are not too close
  angleList = angle.begin();
  for (std::list<vpMeSite>::iterator meList = list.begin(); meList != list.end();) {
    vpMeSite p_me = *meList;
    double new_ang = *angleList;
    if ((new_ang >= alpha1) && (new_ang <= alpha2)) {
      if ((new_ang - previous_ang) >= (0.6 * incr)) {
        previous_ang = new_ang;
        numberOfGoodPoints++;
        ++meList;
        ++angleList;
        if (vpDEBUG_ENABLE(3)) {
          vpImagePoint iP;
          iP.set_ij(p_me.ifloat, p_me.jfloat);
          vpDisplay::displayCross(I, iP, 10, vpColor::red, 1);
          printf("In LQR: angle : %lf, i = %.0lf, j = %.0lf\n", vpMath::deg(new_ang), iP.get_i(), iP.get_j());
        }
      }
      else {
        meList = list.erase(meList);
        angleList = angle.erase(angleList);
        if (vpDEBUG_ENABLE(3)) {
          vpImagePoint iP;
          iP.set_ij(p_me.ifloat, p_me.jfloat);
          vpDisplay::displayCross(I, iP, 10, vpColor::orange, 1);
          printf("too near : angle  %lf, i %.0f , j : %0.f\n", vpMath::deg(new_ang), p_me.ifloat, p_me.jfloat);
        }
      }
    }
    else { // point not in the interval [alpha1 ; alpha2]
      meList = list.erase(meList);
      angleList = angle.erase(angleList);
      if (vpDEBUG_ENABLE(3)) {
        vpImagePoint iP;
        iP.set_ij(p_me.ifloat, p_me.jfloat);
        vpDisplay::displayCross(I, iP, 10, vpColor::green, 1);
        printf("not in interval: angle : %lf, i %.0f , j : %0.f\n", vpMath::deg(new_ang), p_me.ifloat, p_me.jfloat);
      }
    }
  }
  // set extremities of the angle list
  m_alphamin = angle.front();
  m_alphamax = angle.back();

  if (vpDEBUG_ENABLE(3)) {
    printf("alphamin : %lf, alphamax : %lf\n", vpMath::deg(m_alphamin), vpMath::deg(m_alphamax));
    printf("Fin leastSquareRobust : nb pts ok  = %d \n", numberOfGoodPoints);
  }

  return(numberOfGoodPoints);
}

void vpMeEllipse::display(const vpImage<unsigned char> &I, const vpColor &col, unsigned int thickness)
{
  vpMeEllipse::displayEllipse(I, iPc, a, b, e, alpha1, alpha2, col, thickness);
}

void vpMeEllipse::initTracking(const vpImage<unsigned char> &I, bool trackCircle, bool trackArc)
{
  unsigned int n = 5; // by default, 5 points for an ellipse
  m_trackCircle = trackCircle;
  if (trackCircle)
    n = 3;
  std::vector<vpImagePoint> iP(n);
  m_trackArc = trackArc;

  vpDisplay::flush(I);

  if (m_trackCircle) {
    if (m_trackArc) {
      std::cout << "First and third points specify the extremities of the arc of circle (clockwise)" << std::endl;
    }
    for (unsigned int k = 0; k < n; k++) {
      std::cout << "Click point " << k + 1 << "/" << n << " on the circle " << std::endl;
      vpDisplay::getClick(I, iP[k], true);
      vpDisplay::displayCross(I, iP[k], 10, vpColor::red);
      vpDisplay::flush(I);
      std::cout << iP[k] << std::endl;
    }
  }
  else {
    if (m_trackArc) {
      std::cout << "First and fifth points specify the extremities of the arc of ellipse (clockwise)" << std::endl;
    }
    for (unsigned int k = 0; k < n; k++) {
      std::cout << "Click point " << k + 1 << "/" << n << " on the ellipse " << std::endl;
      vpDisplay::getClick(I, iP[k], true);
      vpDisplay::displayCross(I, iP[k], 10, vpColor::red);
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
    iP1 = iP.front();
    iP2 = iP.back();
    // useful for sample(I):
    alpha1 = computeAngleOnEllipse(iP1);
    alpha2 = computeAngleOnEllipse(iP2);
    if ((alpha2 <= alpha1) || (std::fabs(alpha2 - alpha1) < m_arcEpsilon)) {
      alpha2 += 2.0 * M_PI;
    }
  }
  else {
    alpha1 = 0.0;
    alpha2 = 2 * M_PI;
    // useful for track(I):
    vpImagePoint ip;
    computePointOnEllipse(alpha1, ip);
    iP1 = iP2 = ip;
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
  if (pt1 != NULL && pt2 != NULL) {
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
    alpha1 = computeAngleOnEllipse(*pt1);
    alpha2 = computeAngleOnEllipse(*pt2);
    if ((alpha2 <= alpha1) || (std::fabs(alpha2 - alpha1) < m_arcEpsilon)) {
      alpha2 += 2.0 * M_PI;
    }
    // useful for track(I)
    iP1 = *pt1;
    iP2 = *pt2;
  }
  else {
    alpha1 = 0.0;
    alpha2 = 2.0 * M_PI;
    // useful for track(I)
    vpImagePoint ip;
    computePointOnEllipse(alpha1, ip);
    iP1 = iP2 = ip;
  }
  // useful for display(I) so useless if no display before track(I)
  iPc.set_uv(m_uc, m_vc);

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
    alpha1 = computeAngleOnEllipse(iP1);
    alpha2 = computeAngleOnEllipse(iP2);
    if ((alpha2 <= alpha1) || (std::fabs(alpha2 - alpha1) < m_arcEpsilon)) {
      alpha2 += 2.0 * M_PI;
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

  if (m_numberOfGoodPoints <= 5) {
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
    if (m_numberOfGoodPoints <= 5) {
      throw(vpException(vpTrackingException::notEnoughPointError, "Impossible to track the ellipse, not enough features"));
    }
  }

  if (vpDEBUG_ENABLE(3)) {
    printParameters();
  }

  // remet a jour l'angle delta pour chaque vpMeSite de la liste
  updateTheta();
  // not in getParameters since computed only once for each image
  m00 = M_PI * a * b;

  // Useful only for tracking an arc of ellipse, but done to give them a value
  computePointOnEllipse(alpha1, iP1);
  computePointOnEllipse(alpha2, iP2);

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
  computeMoments();
#endif

  if (vpDEBUG_ENABLE(3)) {
    display(I, vpColor::red);
    vpMeTracker::display(I);
    vpDisplay::flush(I);
  }
}

#ifdef VISP_BUILD_DEPRECATED_FUNCTIONS
/*!
  \deprecated Computes the first and second order moments \f$ m_{ij} \f$
*/
void vpMeEllipse::computeMoments()
{
  // m00 = M_PI * a * b;  // moved in track(I)

  m10 = m_uc * m00;
  m01 = m_vc * m00;

  mu20 = m_n20 * m00;
  mu11 = m_n11 * m00;
  mu02 = m_n02 * m00;

  m20 = mu20 + m10 * m_uc;
  m11 = mu11 + m10 * m_vc;
  m02 = mu02 + m01 * m_vc;
}

/*!
  \deprecated Initialization of the tracking. The arc of the ellipse is defined thanks to
  its center, semi major axis, semi minor axis, orientation and the angle
  of its two extremities

  \param I : Image in which the ellipse appears.
  \param center_p : Ellipse center.
  \param a_p : Semi major axis.
  \param b_p : Semi minor axis.
  \param e_p : Orientation in rad.
  \param alpha1_p : Angle in rad defining the first extremity of the arc.
  \param alpha2_p : Angle in rad defining the second extremity of the arc.
*/
void vpMeEllipse::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &center_p, double a_p,
                                     double b_p, double e_p, double alpha1_p, double alpha2_p)
{
  m_trackArc = true;
  // useful for sample(I): uc, vc, a, b, e, Ki, alpha1, alpha2
  m_uc = center_p.get_u();
  m_vc = center_p.get_v();
  a = a_p;
  b = b_p;
  e = e_p;
  ce = cos(e);
  se = sin(e);
  computeNijFromAbe();
  computeKiFromNij();

  alpha1 = alpha1_p;
  alpha2 = alpha2_p;
  if (alpha2 < alpha1) {
    alpha2 += 2 * M_PI;
  }
  // useful for track(I)
  vpImagePoint ip;
  computePointOnEllipse(alpha1, ip);
  iP1 = ip;
  computePointOnEllipse(alpha2, ip);
  iP2 = ip;
  // currently not used after, but done to be complete
  // would be needed for displaying the ellipse here
  iPc = center_p;

  sample(I);
  track(I);
  vpMeTracker::display(I);
  vpDisplay::flush(I);
}

/*!
  \deprecated Initialization of the tracking. The ellipse is defined thanks to the
  coordinates of n points.

  \warning It is mandatory to use at least five points to estimate the
  ellipse parameters.
  \warning The n points should be selected as far as possible from each other.

  \param I : Image in which the ellipse appears.
  \param n : The number of points in the list.
  \param iP : A pointer to a list of points belonging to the ellipse edge.
*/
void vpMeEllipse::initTracking(const vpImage<unsigned char> &I, unsigned int n, vpImagePoint *iP)
{
  std::vector<vpImagePoint> v_iP(n);

  for (unsigned int i = 0; i < n; i++) {
    v_iP[i] = iP[i];
  }
  initTracking(I, v_iP);
}

/*!
 * \deprecated Use an other initTracking() function.
 */
void vpMeEllipse::initTracking(const vpImage<unsigned char> &I, unsigned int n, unsigned *i, unsigned *j)
{
  std::vector<vpImagePoint> v_iP(n);

  for (unsigned int k = 0; k < n; k++) {
    v_iP[k].set_ij(i[k], j[k]);
  }
  initTracking(I, v_iP);
}

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
