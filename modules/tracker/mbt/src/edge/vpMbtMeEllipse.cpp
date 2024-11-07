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

#ifndef DOXYGEN_SHOULD_SKIP_THIS

#include <visp3/mbt/vpMbtMeEllipse.h>

#include <visp3/core/vpDebug.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpRobust.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/me/vpMe.h>

#include <algorithm> // (std::min)
#include <cmath>     // std::fabs
#include <limits>    // numeric_limits

BEGIN_VISP_NAMESPACE
/*!
  Basic constructor that calls the constructor of the class vpMeTracker.
*/
vpMbtMeEllipse::vpMbtMeEllipse() : vpMeEllipse() { }

/*!
  Copy constructor.
*/
vpMbtMeEllipse::vpMbtMeEllipse(const vpMbtMeEllipse &me_ellipse) : vpMeEllipse(me_ellipse) { }

/*!
  Compute the projection error of the ellipse.
  Compare the gradient direction around samples of the ellipse to the normal
  of the tangent of the considered sample. Error is expressed in radians
  between 0 and M_PI/2.0;

  \param I : Image in which the line appears.
  \param sumErrorRad : sum of the error per feature.
  \param nbFeatures : Number of features used to compute _sumErrorRad.
  \param SobelX : Sobel kernel in X-direction.
  \param SobelY : Sobel kernel in Y-direction.
  \param display : If true, display gradient and model orientation.
  \param length : Length of arrows used to show gradient and model orientation.
  \param thickness : Thickness of arrows used to show gradient and model orientation.
*/
void vpMbtMeEllipse::computeProjectionError(const vpImage<unsigned char> &I, double &sumErrorRad,
                                            unsigned int &nbFeatures, const vpMatrix &SobelX, const vpMatrix &SobelY,
                                            bool display, unsigned int length, unsigned int thickness)
{
  sumErrorRad = 0;
  nbFeatures = 0;

  double offset = static_cast<double>(std::floor(SobelX.getRows() / 2.0f));
  int height = static_cast<int>(I.getHeight());
  int width = static_cast<int>(I.getWidth());

  double max_iImg = height - 1.;
  double max_jImg = width - 1.;

  vpColVector vecSite(2);
  vpColVector vecGrad(2);

  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != m_meList.end(); ++it) {
    double iSite = it->m_ifloat;
    double jSite = it->m_jfloat;

    if (!outOfImage(vpMath::round(iSite), vpMath::round(jSite), 0, height, width)) { // Check if necessary
      // The tangent angle to the ellipse at a site
      double theta = computeTheta(vpImagePoint(iSite, jSite));

      vecSite[0] = cos(theta);
      vecSite[1] = sin(theta);
      vecSite.normalize();

      double gradientX = 0;
      double gradientY = 0;

      for (unsigned int i = 0; i < SobelX.getRows(); i++) {
        double iImg = iSite + (i - offset);
        for (unsigned int j = 0; j < SobelX.getCols(); j++) {
          double jImg = jSite + (j - offset);

          if (iImg < 0)
            iImg = 0.0;
          if (jImg < 0)
            jImg = 0.0;

          if (iImg > max_iImg)
            iImg = max_iImg;
          if (jImg > max_jImg)
            jImg = max_jImg;

          gradientX += SobelX[i][j] * I((unsigned int)iImg, (unsigned int)jImg);
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

          if (iImg > max_iImg)
            iImg = max_iImg;
          if (jImg > max_jImg)
            jImg = max_jImg;

          gradientY += SobelY[i][j] * I((unsigned int)iImg, (unsigned int)jImg);
        }
      }

      double angle = atan2(gradientY, gradientX);
      while (angle < 0)
        angle += M_PI;
      while (angle > M_PI)
        angle -= M_PI;

      vecGrad[0] = cos(angle);
      vecGrad[1] = sin(angle);
      vecGrad.normalize();

      double angle1 = acos(vecSite * vecGrad);
      double angle2 = acos(vecSite * (-vecGrad));

      if (display) {
        vpDisplay::displayArrow(I, it->get_i(), it->get_j(), static_cast<int>(it->get_i() + length * cos(theta)),
                                static_cast<int>(it->get_j() + length * sin(theta)), vpColor::blue,
                                length >= 20 ? length / 5 : 4, length >= 20 ? length / 10 : 2, thickness);
        if (angle1 < angle2) {
          vpDisplay::displayArrow(I, it->get_i(), it->get_j(), static_cast<int>(it->get_i() + length * cos(angle)),
                                  static_cast<int>(it->get_j() + length * sin(angle)), vpColor::red,
                                  length >= 20 ? length / 5 : 4, length >= 20 ? length / 10 : 2, thickness);
        }
        else {
          vpDisplay::displayArrow(I, it->get_i(), it->get_j(),
                                  static_cast<int>(it->get_i() + length * cos(angle + M_PI)),
                                  static_cast<int>(it->get_j() + length * sin(angle + M_PI)), vpColor::red,
                                  length >= 20 ? length / 5 : 4, length >= 20 ? length / 10 : 2, thickness);
        }
      }

      sumErrorRad += std::min<double>(angle1, angle2);

      nbFeatures++;
    }
  }
}

void vpMbtMeEllipse::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &center_p, double n20_p,
                                  double n11_p, double n02_p, bool doNotTrack, vpImagePoint *pt1,
                                  const vpImagePoint *pt2)
{
  if (pt1 != nullptr && pt2 != nullptr) {
    m_trackArc = true;
  }

  // useful for sample(I) : uc, vc, a, b, e, Ki, alpha1, alpha2
  m_uc = center_p.get_u();
  m_vc = center_p.get_v();
  m_n20 = n20_p;
  m_n11 = n11_p;
  m_n02 = n02_p;

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
    m_iP1 = ip;
    m_iP2 = ip;
  }
  // useful for display(I) so useless if no display before track(I)
  m_iPc.set_uv(m_uc, m_vc);

  sample(I, doNotTrack);

  try {
    if (!doNotTrack)
      track(I);
  }
  catch (const vpException &exception) {
    throw(exception);
  }
}

/*!
  Track the ellipse in the image I.

  \param I : Image in which the ellipse appears.
*/
void vpMbtMeEllipse::track(const vpImage<unsigned char> &I)
{
  try {
    vpMeTracker::track(I);
    if (m_mask != nullptr) {
      // Expected density could be modified if some vpMeSite are no more tracked because they are outside the mask.
      m_expectedDensity = static_cast<unsigned int>(m_meList.size());
    }
  }
  catch (const vpException &exception) {
    throw(exception);
  }
}

/*!
 * Update ellipse parameters.
 */
void vpMbtMeEllipse::updateParameters(const vpImage<unsigned char> &I, const vpImagePoint &center_p, double n20_p,
                                      double n11_p, double n02_p)
{
  m_uc = center_p.get_u();
  m_vc = center_p.get_v();
  m_n20 = n20_p;
  m_n11 = n11_p;
  m_n02 = n02_p;

  computeAbeFromNij();
  computeKiFromNij();

  suppressPoints();
  reSample(I);

  // remet a jour l'angle delta pour chaque  point de la liste
  updateTheta();
}

/*!
  Resample the ellipse if the number of sample is less than 90% of the
  expected value.

  \note The expected value is computed thanks to the difference between the
  smallest and the biggest \f$ \alpha \f$ angles and the parameter which
  indicates the number of degrees between two points (vpMe::sample_step).

  \param I : Image in which the ellipse appears.

  \exception vpTrackingException::initializationError : Moving edges not
  initialized.
*/
void vpMbtMeEllipse::reSample(const vpImage<unsigned char> &I)
{
  if (!m_me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  unsigned int n = numberOfSignal();
  if ((double)n < 0.9 * m_expectedDensity) {
    sample(I);
    vpMeTracker::track(I);
  }
}

/*!
  Construct a list of vpMeSite moving edges at a particular sampling
  step between the two extremities. The two extremities are defined by
  the points with the smallest and the biggest \f$ alpha \f$ angle.

  \param I : Image in which the ellipse appears.
  \param doNotTrack : If true, moving-edges are not tracked.

  \exception vpTrackingException::initializationError : Moving edges not
  initialized.

*/
void vpMbtMeEllipse::sample(const vpImage<unsigned char> &I, bool doNotTrack)
{
  // Warning: similar code in vpMeEllipse::sample() except for display that is removed here
  if (!m_me) {
    throw(vpException(vpException::fatalError, "Moving edges on ellipse not initialized"));
  }
  // Delete old lists
  m_meList.clear();
  m_angleList.clear();

  int nbrows = static_cast<int>(I.getHeight());
  int nbcols = static_cast<int>(I.getWidth());

  if (std::fabs(m_me->getSampleStep()) <= std::numeric_limits<double>::epsilon()) {
    std::cout << "In vpMeEllipse::sample: ";
    std::cout << "function called with sample step = 0, set to 10 dg";
    m_me->setSampleStep(10.0);
  }
  double incr = vpMath::rad(m_me->getSampleStep()); // angle increment
  // alpha2 - alpha1 = 2 * M_PI for a complete ellipse
  m_expectedDensity = static_cast<unsigned int>(floor((m_alpha2 - m_alpha1) / incr));

  // starting angle for sampling
  double ang = m_alpha1 + ((m_alpha2 - m_alpha1) - static_cast<double>(m_expectedDensity) * incr) / 2.0;
  // sample positions
  for (unsigned int i = 0; i < m_expectedDensity; i++) {
    vpImagePoint iP;
    computePointOnEllipse(ang, iP);
    // If point is in the image, add to the sample list
    // Check done in (i,j) frame)
    if (!outOfImage(vpMath::round(iP.get_i()), vpMath::round(iP.get_j()), 0, nbrows, nbcols)) {
      double theta = computeTheta(iP);
      vpMeSite pix;
      // (i,j) frame used for vpMeSite
      pix.init(iP.get_i(), iP.get_j(), theta);
      pix.setDisplay(m_selectDisplay);
      pix.setState(vpMeSite::NO_SUPPRESSION);
      m_meList.push_back(pix);
      m_angleList.push_back(ang);
    }
    ang += incr;
  }
  if (!doNotTrack) {
    vpMeTracker::initTracking(I);
  }
}

/*!
  Suppress the vpMeSite which are no more detected as point which belongs to
  the ellipse edge.
*/
void vpMbtMeEllipse::suppressPoints()
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
END_VISP_NAMESPACE
#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
