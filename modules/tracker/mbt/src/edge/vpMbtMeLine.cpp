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

/*!
  Basic constructor that calls the constructor of the class vpMeTracker.
*/
vpMbtMeLine::vpMbtMeLine()
  : vpMeLine(), imin(0), imax(0), jmin(0), jmax(0), expecteddensity(0.)
{ }

/*!
 * Copy constructor.
 */
vpMbtMeLine::vpMbtMeLine(const vpMbtMeLine &meline)
  : vpMeLine(meline)
{
  imin = meline.imin;
  jmin = meline.jmin;
  imax = meline.imax;
  jmax = meline.jmax;
  expecteddensity = meline.expecteddensity;
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
  // 1.We do what concerns straight lines
  // Extremities
  double id1, jd1, id2, jd2;
  id1 = ip1.get_i();
  jd1 = ip1.get_j();
  id2 = ip2.get_i();
  jd2 = ip2.get_j();

  m_PExt[0].m_ifloat = id1;
  m_PExt[0].m_jfloat = jd1;
  m_PExt[1].m_ifloat = id2;
  m_PExt[1].m_jfloat = jd2;

  m_rho = rho;
  m_theta = theta;

  m_a = cos(m_theta);
  m_b = sin(m_theta);
  m_c = -m_rho;

  m_delta = atan2((jd2 - jd1), (id1 - id2));

  sample(I, doNoTrack);

  // 2. We call what is not specific
  vpMeTracker::initTracking(I);

  expecteddensity = (double)m_meList.size();

  if (!doNoTrack) {
    vpMeTracker::track(I);
  }
}

/*!
  Seek along the line defined by its equation, the two extremities of the line.
  This function is useful in case of translation of the line.

  \param I : Image in which the line appears.
*/
unsigned int vpMbtMeLine::seekExtremities(const vpImage<unsigned char> &I)
{
  int nbrows = static_cast<int>(I.getHeight());
  int nbcols = static_cast<int>(I.getWidth());

  // Point extremities strictly on the straight line
  vpImagePoint ip1, ip2;
  getExtremities(ip1, ip2);
  double id1 = ip1.get_i();
  double jd1 = ip1.get_j();
  double id2 = ip2.get_i();
  double jd2 = ip2.get_j();

  // i, j portions of the line_p
  double diffsi = id2 - id1;
  double diffsj = jd2 - jd1;
  double s = sqrt(vpMath::sqr(diffsi) + vpMath::sqr(diffsj));

  double sample_step = (double)m_me->getSampleStep();

  double di = diffsi * sample_step / s; // pas de risque de /0 car d(P1,P2) >0
  double dj = diffsj * sample_step / s;

  vpMeSite P;

  P.init((int)id1, (int)jd1, m_delta, 0, m_sign);
  P.setDisplay(m_selectDisplay);
  const double marginRatio = m_me->getThresholdMarginRatio();

  unsigned int memory_range = m_me->getRange();
  m_me->setRange(1);

  unsigned int nb_added_points = 0;

  // Try to add at max 3 points along first extremity
  // (could be a little bit more or less)
  for (int i = 0; i < 3; ++i) {
    id1 -= di;
    jd1 -= dj;
    vpImagePoint iP;
    iP.set_ij(id1, jd1);

    if ((id1 < imin) || (id1 > imax) || (jd1 < jmin) || (jd1 > jmax)) {
      if (vpDEBUG_ENABLE(3)) {
        vpDisplay::displayCross(I, static_cast<int>(id1), static_cast<int>(jd1), 15, vpColor::cyan);
        vpDisplay::flush(I);
      }
    }
    // First test to ensure that iP coordinates are > 0 before casting to unsigned int
    else if (!outOfImage(iP, 5, nbrows, nbcols)) {
      unsigned int is_uint = static_cast<unsigned int>(id1);
      unsigned int js_uint = static_cast<unsigned int>(jd1);
      if (inRoiMask(m_mask, is_uint, js_uint) && inMeMaskCandidates(m_maskCandidates, is_uint, js_uint)) {
        // ajout
        P.m_ifloat = id1;
        P.m_i = static_cast<int>(id1);
        P.m_jfloat = jd1;
        P.m_j = static_cast<int>(jd1);
        double convolution = P.convolution(I, m_me);
        double contrastThreshold = fabs(convolution) * marginRatio;
        P.setContrastThreshold(contrastThreshold, *m_me);
        // fin ajout
        P.track(I, m_me, false);
        if (P.getState() == vpMeSite::NO_SUPPRESSION) {
          m_meList.push_front(P);
          nb_added_points++;
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
  }

  // Try to add at max 3 points along second extremity
  // (could be a little bit more or less)

  for (int i = 0; i < 3; ++i) {
    id2 += di;
    jd2 += dj;

    vpImagePoint iP;
    iP.set_i(id2);
    iP.set_j(jd2);

    if ((id2 < imin) || (id2 > imax) || (jd2 < jmin) || (jd2 > jmax)) {
      if (vpDEBUG_ENABLE(3)) {
        vpDisplay::displayCross(I, static_cast<int>(id2), static_cast<int>(jd2), 5, vpColor::cyan);
        vpDisplay::flush(I);
      }
    }
    // First test to ensure that iP coordinates are > 0 before casting to unsigned int
    else if (!outOfImage(iP, 5, nbrows, nbcols)) {
      unsigned int is_uint = static_cast<unsigned int>(id2);
      unsigned int js_uint = static_cast<unsigned int>(jd2);
      if (inRoiMask(m_mask, is_uint, js_uint) && inMeMaskCandidates(m_maskCandidates, is_uint, js_uint)) {
        // ajout
        P.m_ifloat = id2;
        P.m_i = static_cast<int>(id2);
        P.m_jfloat = jd2;
        P.m_j = static_cast<int>(jd2);
        double convolution = P.convolution(I, m_me);
        double contrastThreshold = fabs(convolution) * marginRatio;
        P.setContrastThreshold(contrastThreshold, *m_me);
        // fin ajout
        P.track(I, m_me, false);
        if (P.getState() == vpMeSite::NO_SUPPRESSION) {
          m_meList.push_back(P);
          nb_added_points++;
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
  }

  m_me->setRange(memory_range);
  return(nb_added_points);
}

/*!
  Suppress the moving which belong no more to the line.

  \param I : The image.
*/
void vpMbtMeLine::suppressPoints(const vpImage<unsigned char> &I)
{
  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != m_meList.end();) {
    vpMeSite s = *it; // current reference pixel

    // Vertical line management
    if (fabs(sin(m_theta)) > 0.9) {
      if ((s.m_i < imin) || (s.m_i > imax)) {
        s.setState(vpMeSite::CONTRAST);
      }
    }
    // Horizontal line management
    else if (fabs(cos(m_theta)) > 0.9) {
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

    if (s.getState() != vpMeSite::NO_SUPPRESSION) {
      it = m_meList.erase(it);
    }
    else {
      ++it;
    }
  }
}

/*!
  Compute the projection error of the line. Compare the gradient direction
  around samples of the line to its direction. Error is expressed in radians
  between 0 and M_PI/2.0;

  \param I : Image in which the line appears.
  \param sumErrorRad : sum of the error per feature.
  \param nbFeatures : Number of features used to compute `sumErrorRad`.
  \param SobelX : Sobel kernel in X-direction.
  \param SobelY : Sobel kernel in Y-direction.
  \param display : If true, display gradient and model orientation.
  \param length : Length of arrows used to show gradient and model orientation.
  \param thickness : Thickness of arrows used to show gradient and model orientation.
*/
void vpMbtMeLine::computeProjectionError(const vpImage<unsigned char> &I, double &sumErrorRad,
                                         unsigned int &nbFeatures, const vpMatrix &SobelX, const vpMatrix &SobelY,
                                         bool display, unsigned int length, unsigned int thickness)
{
  sumErrorRad = 0;
  nbFeatures = 0;
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

      for (unsigned int i = 0; i < SobelX.getRows(); ++i) {
        double iImg = iSite + (i - offset);
        for (unsigned int j = 0; j < SobelX.getCols(); ++j) {
          double jImg = jSite + (j - offset);

          if (iImg < 0) {
            iImg = 0.0;
          }
          if (jImg < 0) {
            jImg = 0.0;
          }

          if (iImg > I.getHeight() - 1) {
            iImg = I.getHeight() - 1;
          }
          if (jImg > I.getWidth() - 1) {
            jImg = I.getWidth() - 1;
          }

          gradientX += SobelX[i][j] * I((unsigned int)iImg, (unsigned int)jImg);
        }
      }

      for (unsigned int i = 0; i < SobelY.getRows(); ++i) {
        double iImg = iSite + (i - offset);
        for (unsigned int j = 0; j < SobelY.getCols(); ++j) {
          double jImg = jSite + (j - offset);

          if (iImg < 0)
            iImg = 0.0;
          if (jImg < 0)
            jImg = 0.0;

          if (iImg > I.getHeight() - 1)
            iImg = I.getHeight() - 1;
          if (jImg > I.getWidth() - 1)
            jImg = I.getWidth() - 1;

          gradientY += SobelY[i][j] * I((unsigned int)iImg, (unsigned int)jImg);
        }
      }

      double angle = atan2(gradientX, gradientY);
      while (angle < 0) {
        angle += M_PI;
      }
      while (angle > M_PI) {
        angle -= M_PI;
      }

      vpColVector vecGrad(2);
      vecGrad[0] = cos(angle);
      vecGrad[1] = sin(angle);
      vecGrad.normalize();

      double angle1 = acos(vecLine * vecGrad);
      double angle2 = acos(vecLine * (-vecGrad));

      if (display) {
        vpDisplay::displayArrow(I, it->get_i(), it->get_j(), (int)(it->get_i() + length * cos(deltaNormalized)),
                                (int)(it->get_j() + length * sin(deltaNormalized)), vpColor::blue,
                                length >= 20 ? length / 5 : 4, length >= 20 ? length / 10 : 2, thickness);
        if (angle1 < angle2) {
          vpDisplay::displayArrow(I, it->get_i(), it->get_j(), (int)(it->get_i() + length * cos(angle)),
                                  (int)(it->get_j() + length * sin(angle)), vpColor::red, length >= 20 ? length / 5 : 4,
                                  length >= 20 ? length / 10 : 2, thickness);
        }
        else {
          vpDisplay::displayArrow(I, it->get_i(), it->get_j(), (int)(it->get_i() + length * cos(angle + M_PI)),
                                  (int)(it->get_j() + length * sin(angle + M_PI)), vpColor::red,
                                  length >= 20 ? length / 5 : 4, length >= 20 ? length / 10 : 2, thickness);
        }
      }

      sumErrorRad += std::min<double>(angle1, angle2);

      nbFeatures++;
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
  \param ip1 : The first extremity of the line.
  \param ip2 : The second extremity of the line.
*/
void vpMbtMeLine::reSample(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2)
{
  m_PExt[0].m_ifloat = (float)ip1.get_i();
  m_PExt[0].m_jfloat = (float)ip1.get_j();
  m_PExt[1].m_ifloat = (float)ip2.get_i();
  m_PExt[1].m_jfloat = (float)ip2.get_j();

  vpMeLine::reSample(I);
}

/*!
  Track the line in the image I.

  \param I : Image in which the line appears.
*/
void vpMbtMeLine::track(const vpImage<unsigned char> &I)
{
  if (m_mask != nullptr) {
  // Expected density could be modified if some vpMeSite are no more tracked because they are outside the mask.
    expecteddensity = (double)m_meList.size();
  }

  vpMeLine::track(I);
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
  // Search for points at the ends of the straight line in the case of sliding
  suppressPoints(I);
  seekExtremities(I);
  suppressPoints(I);
  // Resampling if necessary
  vpMeLine::reSample(I);
  // Updates the delta angle for each point in the list
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
  // Search for points at the ends of the straight line in the case of sliding
  suppressPoints(I);
  seekExtremities(I);
  suppressPoints(I);
  // Resampling if necessary
  reSample(I, ip1, ip2);

  // Updates the delta angle for each point in the list
  updateDelta();
}

END_VISP_NAMESPACE
#endif
