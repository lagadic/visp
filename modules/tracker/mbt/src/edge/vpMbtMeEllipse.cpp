/****************************************************************************
 *
 * ViSP, open source Visual Servoing Platform software.
 * Copyright (C) 2005 - 2019 by Inria. All rights reserved.
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
 * See http://visp.inria.fr for more information.
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
 * Authors:
 * Eric Marchand
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

/*!
  Basic constructor that calls the constructor of the class vpMeTracker.
*/
vpMbtMeEllipse::vpMbtMeEllipse()
  : iPc(), a(0.), b(0.), e(0.), ce(0.), se(0.), mu11(0.), mu20(0.), mu02(0.), thresholdWeight(0.), expecteddensity(0.)
{
}

/*!
  Copy constructor.
*/
vpMbtMeEllipse::vpMbtMeEllipse(const vpMbtMeEllipse &meellipse)
  : vpMeTracker(meellipse), iPc(meellipse.iPc), a(0.), b(0.), e(0.), ce(0.), se(0.), mu11(0.), mu20(0.), mu02(0.),
    thresholdWeight(0.), expecteddensity(0.)
{
  a = meellipse.a;
  b = meellipse.b;
  e = meellipse.e;

  ce = meellipse.ce;
  se = meellipse.se;

  mu11 = meellipse.mu11;
  mu20 = meellipse.mu20;
  mu02 = meellipse.mu02;

  expecteddensity = meellipse.expecteddensity;
}

/*!
  Basic destructor.
*/
vpMbtMeEllipse::~vpMbtMeEllipse() { list.clear(); }

/*!
  Compute the projection error of the ellipse.
  Compare the gradient direction around samples of the ellipse to the normal
  of the tangent of the considered sample. Error is expressed in radians
  between 0 and M_PI/2.0;

  \param _I : Image in which the line appears.
  \param _sumErrorRad : sum of the error per feature.
  \param _nbFeatures : Number of features used to compute _sumErrorRad.
  \param SobelX : Sobel kernel in X-direction.
  \param SobelX : Sobel kernel in Y-direction.
  \param display : If true, display gradient and model orientation.
  \param length : Length of arrows used to show gradient and model orientation.
  \param thickness : Thickness of arrows used to show gradient and model orientation.
*/
void vpMbtMeEllipse::computeProjectionError(const vpImage<unsigned char> &_I, double &_sumErrorRad,
                                            unsigned int &_nbFeatures,
                                            const vpMatrix &SobelX, const vpMatrix &SobelY,
                                            const bool display, const unsigned int length,
                                            const unsigned int thickness)
{
  _sumErrorRad = 0;
  _nbFeatures = 0;

  double offset = std::floor(SobelX.getRows() / 2.0f);
  //  std::cout << "offset=" << offset << std::endl;
  int height = (int)_I.getHeight();
  int width = (int)_I.getWidth();

  for (std::list<vpMeSite>::iterator it = list.begin(); it != list.end(); ++it) {
    double iSite = it->ifloat;
    double jSite = it->jfloat;

    if (!outOfImage(vpMath::round(iSite), vpMath::round(jSite), 0, height,
                    width)) { // Check if necessary
      // The tangent angle to the ellipse at a site
      double theta = atan((-mu02 * jSite + mu02 * iPc.get_j() + mu11 * iSite - mu11 * iPc.get_i()) /
                          (mu20 * iSite - mu11 * jSite + mu11 * iPc.get_j() - mu20 * iPc.get_i())) -
                     M_PI / 2;

      double deltaNormalized = theta;
      while (deltaNormalized < 0)
        deltaNormalized += M_PI;
      while (deltaNormalized > M_PI)
        deltaNormalized -= M_PI;

      vpColVector vecSite(2);
      vecSite[0] = cos(deltaNormalized);
      vecSite[1] = sin(deltaNormalized);
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

      double angle = atan2(gradientY, gradientX);
      while (angle < 0)
        angle += M_PI;
      while (angle > M_PI)
        angle -= M_PI;

      //      if(std::fabs(deltaNormalized-angle) > M_PI / 2)
      //      {
      //        sumErrorRad += sqrt(vpMath::sqr(deltaNormalized-angle)) - M_PI
      //        / 2;
      //      } else {
      //        sumErrorRad += sqrt(vpMath::sqr(deltaNormalized-angle));
      //      }

      //      double angle1 = sqrt(vpMath::sqr(deltaNormalized-angle));
      //      double angle2 = sqrt(vpMath::sqr(deltaNormalized-
      //      (angle-M_PI)));

      vpColVector vecGrad(2);
      vecGrad[0] = cos(angle);
      vecGrad[1] = sin(angle);
      vecGrad.normalize();

      double angle1 = acos(vecSite * vecGrad);
      double angle2 = acos(vecSite * (-vecGrad));

      if (display) {
        vpDisplay::displayArrow(_I, it->get_i(), it->get_j(), (int)(it->get_i() + length*cos(deltaNormalized)),
                                (int)(it->get_j() + length*sin(deltaNormalized)), vpColor::blue,
                                length >= 20 ? length/5 : 4, length >= 20 ? length/10 : 2, thickness);
        if (angle1 < angle2) {
          vpDisplay::displayArrow(_I, it->get_i(), it->get_j(), (int)(it->get_i() + length*cos(angle)),
                                  (int)(it->get_j() + length*sin(angle)), vpColor::red,
                                  length >= 20 ? length/5 : 4, length >= 20 ? length/10 : 2, thickness);
        } else {
          vpDisplay::displayArrow(_I, it->get_i(), it->get_j(), (int)(it->get_i() + length*cos(angle+M_PI)),
                                  (int)(it->get_j() + length*sin(angle+M_PI)), vpColor::red,
                                  length >= 20 ? length/5 : 4, length >= 20 ? length/10 : 2, thickness);
        }
      }

      _sumErrorRad += (std::min)(angle1, angle2);

      _nbFeatures++;
    }
  }
}

/*!
  Construct a list of vpMeSite moving edges at a particular sampling
  step between the two extremities. The two extremities are defined by
  the points with the smallest and the biggest \f$ alpha \f$ angle.

  \param I : Image in which the ellipse appears.
  \param doNotTrack : If true, ME are not tracked.

  \exception vpTrackingException::initializationError : Moving edges not
  initialized.
*/
void vpMbtMeEllipse::sample(const vpImage<unsigned char> &I, const bool doNotTrack)
{
  if (!me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  int height = (int)I.getHeight();
  int width = (int)I.getWidth();

  // if (me->getSampleStep()==0)
  if (std::fabs(me->getSampleStep()) <= std::numeric_limits<double>::epsilon()) {
    std::cout << "In vpMbtMeEllipse::sample: ";
    std::cout << "function called with sample step = 0";
    // return fatalError;
  }

  // Approximation of the circumference of an ellipse:
  // [Ramanujan, S., "Modular Equations and Approximations to ,"
  // Quart. J. Pure. Appl. Math., vol. 45 (1913-1914), pp. 350-372]
  double t = (a - b) / (a + b);
  double circumference = M_PI * (a + b) * (1 + 3 * vpMath::sqr(t) / (10 + sqrt(4 - 3 * vpMath::sqr(t))));
  int nb_points_to_track = (int)(circumference / me->getSampleStep());
  double incr = 2 * M_PI / nb_points_to_track;

  expecteddensity = 0; // nb_points_to_track;

  // Delete old list
  list.clear();

  // sample positions
  double k = 0;
  for (int pt = 0; pt < nb_points_to_track; pt++) {
    double j = a * cos(k); // equation of an ellipse
    double i = b * sin(k); // equation of an ellipse

    double iP_j = iPc.get_j() + ce * j - se * i;
    double iP_i = iPc.get_i() + se * j + ce * i;

    // vpColor col = vpColor::red;
    // vpDisplay::displayCross(I, vpImagePoint(iP_i, iP_j),  5, col) ; //debug
    // only

    // If point is in the image, add to the sample list
    if (!outOfImage(vpMath::round(iP_i), vpMath::round(iP_j), 0, height, width)) {
      // The tangent angle to the ellipse at a site
      double theta = atan((-mu02 * iP_j + mu02 * iPc.get_j() + mu11 * iP_i - mu11 * iPc.get_i()) /
                          (mu20 * iP_i - mu11 * iP_j + mu11 * iPc.get_j() - mu20 * iPc.get_i())) -
                     M_PI / 2;

      vpMeSite pix;
      pix.init((int)iP_i, (int)iP_j, theta);
      pix.setDisplay(selectDisplay);
      pix.setState(vpMeSite::NO_SUPPRESSION);

      list.push_back(pix);
      expecteddensity++;
    }
    k += incr;
  }

  if (!doNotTrack)
    vpMeTracker::initTracking(I);
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
  if (!me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  unsigned int n = numberOfSignal();
  if ((double)n < 0.9 * expecteddensity) {
    sample(I);
    vpMeTracker::track(I);
  }
}

/*!
  Compute the oriantation angle for each vpMeSite.

  \note The \f$ \theta \f$ angle is useful during the tracking part.
*/
void vpMbtMeEllipse::updateTheta()
{
  vpMeSite p_me;
  for (std::list<vpMeSite>::iterator it = list.begin(); it != list.end(); ++it) {
    p_me = *it;
    vpImagePoint iP;
    iP.set_i(p_me.ifloat);
    iP.set_j(p_me.jfloat);

    // The tangent angle to the ellipse at a site
    double theta = atan((-mu02 * p_me.jfloat + mu02 * iPc.get_j() + mu11 * p_me.ifloat - mu11 * iPc.get_i()) /
                        (mu20 * p_me.ifloat - mu11 * p_me.jfloat + mu11 * iPc.get_j() - mu20 * iPc.get_i())) -
                   M_PI / 2;

    p_me.alpha = theta;
    *it = p_me;
  }
}

/*!
  Suppress the vpMeSite which are no more detected as point which belongs to
  the ellipse edge.
*/
void vpMbtMeEllipse::suppressPoints()
{
  // Loop through list of sites to track
  for (std::list<vpMeSite>::iterator itList = list.begin(); itList != list.end();) {
    vpMeSite s = *itList; // current reference pixel
    if (s.getState() != vpMeSite::NO_SUPPRESSION)
      itList = list.erase(itList);
    else
      ++itList;
  }
}

/*!
  Display the ellipse.

  \warning To effectively display the ellipse a call to
  vpDisplay::flush() is needed.

  \param I : Image in which the ellipse appears.
  \param col : Color of the displayed ellipse.
 */
void vpMbtMeEllipse::display(const vpImage<unsigned char> &I, vpColor col)
{
  vpDisplay::displayEllipse(I, iPc, mu20, mu11, mu02, true, col);
}

void vpMbtMeEllipse::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ic, double mu20_p, double mu11_p,
                                  double mu02_p,
                                  const bool doNotTrack)
{
  iPc = ic;
  mu20 = mu20_p;
  mu11 = mu11_p;
  mu02 = mu02_p;

  if (std::fabs(mu11_p) > std::numeric_limits<double>::epsilon()) {

    double val_p = sqrt(vpMath::sqr(mu20_p - mu02_p) + 4 * vpMath::sqr(mu11_p));
    a = sqrt((mu20_p + mu02_p + val_p) / 2);
    b = sqrt((mu20_p + mu02_p - val_p) / 2);

    e = (mu02_p - mu20_p + val_p) / (2 * mu11_p);
  } else {
    a = sqrt(mu20_p);
    b = sqrt(mu02_p);
    e = 0.;
  }

  e = atan(e);

  ce = cos(e);
  se = sin(e);

  sample(I, doNotTrack);

  if (!doNotTrack)
    vpMeTracker::initTracking(I);

  try {
    if (!doNotTrack)
      track(I);
  } catch (const vpException &exception) {
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
    if (m_mask != NULL) {
      // Expected density could be modified if some vpMeSite are no more tracked because they are outside the mask.
      expecteddensity = (double)list.size();
    }
  } catch (const vpException &exception) {
    throw(exception);
  }
}

void vpMbtMeEllipse::updateParameters(const vpImage<unsigned char> &I, const vpImagePoint &ic, double mu20_p,
                                      double mu11_p, double mu02_p)
{
  iPc = ic;
  mu20 = mu20_p;
  mu11 = mu11_p;
  mu02 = mu02_p;

  if (std::fabs(mu11_p) > std::numeric_limits<double>::epsilon()) {

    double val_p = sqrt(vpMath::sqr(mu20_p - mu02_p) + 4 * vpMath::sqr(mu11_p));
    a = sqrt((mu20_p + mu02_p + val_p) / 2);
    b = sqrt((mu20_p + mu02_p - val_p) / 2);

    e = (mu02_p - mu20_p + val_p) / (2 * mu11_p);
  } else {
    a = sqrt(mu20_p);
    b = sqrt(mu02_p);
    e = 0.;
  }

  e = atan(e);

  ce = cos(e);
  se = sin(e);

  suppressPoints();
  reSample(I);

  // remet a jour l'angle delta pour chaque  point de la liste
  updateTheta();
}

#endif // #ifndef DOXYGEN_SHOULD_SKIP_THIS
