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

/*!
  \file vpMeLine.cpp
  \brief Moving edges
*/

#include <algorithm> // (std::min)
#include <cmath>     // std::fabs
#include <limits>    // numeric_limits
#include <stdlib.h>
#include <visp3/core/vpImagePoint.h>
#include <visp3/core/vpMath.h>
#include <visp3/core/vpRobust.h>
#include <visp3/core/vpTrackingException.h>
#include <visp3/me/vpMe.h>
#include <visp3/me/vpMeLine.h>
#include <visp3/me/vpMeSite.h>
#include <visp3/me/vpMeTracker.h>

#define INCR_MIN 1

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
  } else {
    ip = (vpMath::sqr(b) * i - a * b * j - c * a) / (vpMath::sqr(a) + vpMath::sqr(b));
    jp = (-c - a * ip) / b;
  }
}

/*!

  Basic constructor that calls the constructor of the class vpMeTracker.

*/
vpMeLine::vpMeLine()
  : rho(0.), theta(0.), delta(0.), delta_1(0.), angle(0.), angle_1(90), sign(1), _useIntensityForRho(true), a(0.),
    b(0.), c(0.)
{
}
/*!

  Copy constructor.

*/
vpMeLine::vpMeLine(const vpMeLine &meline)
  : vpMeTracker(meline), rho(0.), theta(0.), delta(0.), delta_1(0.), angle(0.), angle_1(90), sign(1),
    _useIntensityForRho(true), a(0.), b(0.), c(0.)

{
  rho = meline.rho;
  theta = meline.theta;
  delta = meline.delta;
  delta_1 = meline.delta_1;
  angle = meline.angle;
  angle_1 = meline.angle_1;
  sign = meline.sign;

  a = meline.a;
  b = meline.b;
  c = meline.c;
  _useIntensityForRho = meline._useIntensityForRho;
  PExt[0] = meline.PExt[0];
  PExt[1] = meline.PExt[1];
}

/*!

  Basic destructor.

*/
vpMeLine::~vpMeLine() { list.clear(); }

/*!

  Construct a list of vpMeSite moving edges at a particular sampling
  step between the two extremities of the line.

  \param I : Image in which the line appears.
  \param doNotTrack : Inherited parameter, not used.

  \exception vpTrackingException::initializationError : Moving edges not
  initialized.

*/
void vpMeLine::sample(const vpImage<unsigned char> &I, const bool doNotTrack)
{
  (void)doNotTrack;
  if (!me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  int rows = (int)I.getHeight();
  int cols = (int)I.getWidth();
  double n_sample;

  if (std::fabs(me->getSampleStep()) <= std::numeric_limits<double>::epsilon()) {
    vpERROR_TRACE("function called with sample step = 0");
    throw(vpTrackingException(vpTrackingException::fatalError, "sample step = 0"));
  }

  // i, j portions of the line_p
  double diffsi = PExt[0].ifloat - PExt[1].ifloat;
  double diffsj = PExt[0].jfloat - PExt[1].jfloat;

  double length_p = sqrt((vpMath::sqr(diffsi) + vpMath::sqr(diffsj)));
  if (std::fabs(length_p) <= std::numeric_limits<double>::epsilon())
    throw(vpTrackingException(vpTrackingException::fatalError, "points too close of each other to define a line"));
  // number of samples along line_p
  n_sample = length_p / (double)me->getSampleStep();

  double stepi = diffsi / (double)n_sample;
  double stepj = diffsj / (double)n_sample;

  // Choose starting point
  double is = PExt[1].ifloat;
  double js = PExt[1].jfloat;

  // Delete old list
  list.clear();

  // sample positions at i*me->getSampleStep() interval along the
  // line_p, starting at PSiteExt[0]

  vpImagePoint ip;
  for (int i = 0; i <= vpMath::round(n_sample); i++) {
    // If point is in the image, add to the sample list
    if (!outOfImage(vpMath::round(is), vpMath::round(js), 0, rows, cols)) {
      vpMeSite pix; //= list.value();
      pix.init((int)is, (int)js, delta, 0, sign);
      pix.setDisplay(selectDisplay);

      if (vpDEBUG_ENABLE(3)) {
        ip.set_i(is);
        ip.set_j(js);
        vpDisplay::displayCross(I, ip, 2, vpColor::blue);
      }

      list.push_back(pix);
    }
    is += stepi;
    js += stepj;
  }

  vpCDEBUG(1) << "end vpMeLine::sample() : ";
  vpCDEBUG(1) << n_sample << " point inserted in the list " << std::endl;
}

/*!
  Display line.

  \warning To effectively display the line a call to
  vpDisplay::flush() is needed.

  \param I : Image in which the line appears.

  \param col : Color of the displayed line. Note that a moving edge
  that is considered as an outlier is displayed in green.

 */
void vpMeLine::display(const vpImage<unsigned char> &I, vpColor col)
{
  vpMeLine::display(I, PExt[0], PExt[1], list, a, b, c, col);
}

/*!

  Initilization of the tracking. Ask the user to click on two points
  from the line to track.

  \param I : Image in which the line appears.
*/
void vpMeLine::initTracking(const vpImage<unsigned char> &I)
{
  vpImagePoint ip1, ip2;

  std::cout << "Click on the line first point..." << std::endl;
  while (vpDisplay::getClick(I, ip1) != true)
    ;
  vpDisplay::displayCross(I, ip1, 7, vpColor::red);
  vpDisplay::flush(I);
  std::cout << "Click on the line second point..." << std::endl;
  while (vpDisplay::getClick(I, ip2) != true)
    ;
  vpDisplay::displayCross(I, ip2, 7, vpColor::red);
  vpDisplay::flush(I);

  try {
    initTracking(I, ip1, ip2);
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
}

/*!

  Least squares method used to make the tracking more robust. It
  ensures that the points taken into account to compute the right
  equation belong to the line.
*/
void vpMeLine::leastSquare()
{
  vpMatrix A(numberOfSignal(), 2);
  vpColVector x(2), x_1(2);
  x_1 = 0;

  unsigned int i;

  vpRobust r(numberOfSignal());
  r.setThreshold(2);
  r.setIteration(0);
  vpMatrix D(numberOfSignal(), numberOfSignal());
  D.eye();
  vpMatrix DA, DAmemory;
  vpColVector DAx;
  vpColVector w(numberOfSignal());
  vpColVector B(numberOfSignal());
  w = 1;
  vpMeSite p_me;
  unsigned int iter = 0;
  unsigned int nos_1 = 0;
  double distance = 100;

  if (list.size() <= 2 || numberOfSignal() <= 2) {
    // vpERROR_TRACE("Not enough point") ;
    vpCDEBUG(1) << "Not enough point";
    throw(vpTrackingException(vpTrackingException::notEnoughPointError, "not enough point"));
  }

  if ((fabs(b) >= 0.9)) // Construction du systeme Ax=B
                        // a i + j + c = 0
                        // A = (i 1)   B = (-j)
  {
    nos_1 = numberOfSignal();
    unsigned int k = 0;
    for (std::list<vpMeSite>::const_iterator it = list.begin(); it != list.end(); ++it) {
      p_me = *it;
      if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
        A[k][0] = p_me.ifloat;
        A[k][1] = 1;
        B[k] = -p_me.jfloat;
        k++;
      }
    }

    while (iter < 4 && distance > 0.05) {
      DA = D * A;
      x = DA.pseudoInverse(1e-26) * D * B;

      vpColVector residu(nos_1);
      residu = B - A * x;
      r.setIteration(iter);
      r.MEstimator(vpRobust::TUKEY, residu, w);

      k = 0;
      for (i = 0; i < nos_1; i++) {
        D[k][k] = w[k];
        k++;
      }
      iter++;
      distance = fabs(x[0] - x_1[0]) + fabs(x[1] - x_1[1]);
      x_1 = x;
    }

    k = 0;
    for (std::list<vpMeSite>::iterator it = list.begin(); it != list.end(); ++it) {
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
    a = x[0];
    b = 1;
    c = x[1];

    double s = sqrt(vpMath::sqr(a) + vpMath::sqr(b));
    a /= s;
    b /= s;
    c /= s;
  }

  else // Construction du systeme Ax=B
       // i + bj + c = 0
       // A = (j 1)   B = (-i)
  {
    nos_1 = numberOfSignal();
    unsigned int k = 0;
    for (std::list<vpMeSite>::const_iterator it = list.begin(); it != list.end(); ++it) {
      p_me = *it;
      if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
        A[k][0] = p_me.jfloat;
        A[k][1] = 1;
        B[k] = -p_me.ifloat;
        k++;
      }
    }

    while (iter < 4 && distance > 0.05) {
      DA = D * A;
      x = DA.pseudoInverse(1e-26) * D * B;

      vpColVector residu(nos_1);
      residu = B - A * x;
      r.setIteration(iter);
      r.MEstimator(vpRobust::TUKEY, residu, w);

      k = 0;
      for (i = 0; i < nos_1; i++) {
        D[k][k] = w[k];
        k++;
      }
      iter++;
      distance = fabs(x[0] - x_1[0]) + fabs(x[1] - x_1[1]);
      x_1 = x;
    }

    k = 0;
    for (std::list<vpMeSite>::iterator it = list.begin(); it != list.end(); ++it) {
      p_me = *it;
      if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
        if (w[k] < 0.2) {
          p_me.setState(vpMeSite::M_ESTIMATOR);

          *it = p_me;
        }
        k++;
      }
    }
    a = 1;
    b = x[0];
    c = x[1];

    double s = sqrt(vpMath::sqr(a) + vpMath::sqr(b));
    a /= s;
    b /= s;
    c /= s;
  }

  // mise a jour du delta
  delta = atan2(a, b);

  normalizeAngle(delta);
}

/*!

  Initialization of the tracking. The line is defined thanks to the
  coordinates of two points.

  \param I : Image in which the line appears.
  \param ip1 : Coordinates of the first point.
  \param ip2 : Coordinates of the second point.
*/
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
      PExt[0].ifloat = (float)ip1.get_i();
      PExt[0].jfloat = (float)ip1.get_j();
      PExt[1].ifloat = (float)ip2.get_i();
      PExt[1].jfloat = (float)ip2.get_j();

      double angle_ = atan2((double)(i1s - i2s), (double)(j1s - j2s));
      a = cos(angle_);
      b = sin(angle_);

      // Real values of a, b can have an other sign. So to get the good values
      // of a and b in order to initialise then c, we call track(I) just below

      computeDelta(delta, i1s, j1s, i2s, j2s);
      delta_1 = delta;

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
  } catch (...) {
    vpERROR_TRACE("Error caught");
    throw;
  }
  vpCDEBUG(1) << " end vpMeLine::initTracking()" << std::endl;
}

/*!
  Suppression of the points which belong no more to the line.
*/
void vpMeLine::suppressPoints()
{
  // Loop through list of sites to track
  for (std::list<vpMeSite>::iterator it = list.begin(); it != list.end();) {
    vpMeSite s = *it; // current reference pixel

    if (s.getState() != vpMeSite::NO_SUPPRESSION)
      it = list.erase(it);
    else
      ++it;
  }
}

/*!
  Seek in the list of available points the two extremities of the line.
*/
void vpMeLine::setExtremities()
{
  double imin = +1e6;
  double jmin = +1e6;
  double imax = -1;
  double jmax = -1;

  // Loop through list of sites to track
  for (std::list<vpMeSite>::const_iterator it = list.begin(); it != list.end(); ++it) {
    vpMeSite s = *it; // current reference pixel
    if (s.ifloat < imin) {
      imin = s.ifloat;
      jmin = s.jfloat;
    }

    if (s.ifloat > imax) {
      imax = s.ifloat;
      jmax = s.jfloat;
    }
  }

  PExt[0].ifloat = imin;
  PExt[0].jfloat = jmin;
  PExt[1].ifloat = imax;
  PExt[1].jfloat = jmax;

  if (fabs(imin - imax) < 25) {
    for (std::list<vpMeSite>::const_iterator it = list.begin(); it != list.end(); ++it) {
      vpMeSite s = *it; // current reference pixel
      if (s.jfloat < jmin) {
        imin = s.ifloat;
        jmin = s.jfloat;
      }

      if (s.jfloat > jmax) {
        imax = s.ifloat;
        jmax = s.jfloat;
      }
    }
    PExt[0].ifloat = imin;
    PExt[0].jfloat = jmin;
    PExt[1].ifloat = imax;
    PExt[1].jfloat = jmax;
  }
}

/*!

  Seek along the line defined by its equation, the two extremities of
  the line. This function is useful in case of translation of the
  line.

  \param I : Image in which the line appears.

  \exception vpTrackingException::initializationError : Moving edges not
  initialized.
*/
void vpMeLine::seekExtremities(const vpImage<unsigned char> &I)
{
  vpCDEBUG(1) << "begin vpMeLine::sample() : " << std::endl;

  if (!me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  int rows = (int)I.getHeight();
  int cols = (int)I.getWidth();
  double n_sample;

  // if (me->getSampleStep()==0)
  if (std::fabs(me->getSampleStep()) <= std::numeric_limits<double>::epsilon()) {

    vpERROR_TRACE("function called with sample step = 0");
    throw(vpTrackingException(vpTrackingException::fatalError, "sample step = 0"));
  }

  // i, j portions of the line_p
  double diffsi = PExt[0].ifloat - PExt[1].ifloat;
  double diffsj = PExt[0].jfloat - PExt[1].jfloat;

  double s = vpMath::sqr(diffsi) + vpMath::sqr(diffsj);

  double di = diffsi / sqrt(s); // pas de risque de /0 car d(P1,P2) >0
  double dj = diffsj / sqrt(s);

  double length_p = sqrt((vpMath::sqr(diffsi) + vpMath::sqr(diffsj)));

  // number of samples along line_p
  n_sample = length_p / (double)me->getSampleStep();
  double sample_step = (double)me->getSampleStep();

  vpMeSite P;
  P.init((int)PExt[0].ifloat, (int)PExt[0].jfloat, delta_1, 0, sign);
  P.setDisplay(selectDisplay);

  unsigned int memory_range = me->getRange();
  me->setRange(1);

  vpImagePoint ip;

  for (int i = 0; i < 3; i++) {
    P.ifloat = P.ifloat + di * sample_step;
    P.i = (int)P.ifloat;
    P.jfloat = P.jfloat + dj * sample_step;
    P.j = (int)P.jfloat;

    if (!outOfImage(P.i, P.j, 5, rows, cols)) {
      P.track(I, me, false);

      if (P.getState() == vpMeSite::NO_SUPPRESSION) {
        list.push_back(P);
        if (vpDEBUG_ENABLE(3)) {
          ip.set_i(P.i);
          ip.set_j(P.j);

          vpDisplay::displayCross(I, ip, 5, vpColor::green);
        }
      } else {
        if (vpDEBUG_ENABLE(3)) {
          ip.set_i(P.i);
          ip.set_j(P.j);
          vpDisplay::displayCross(I, ip, 10, vpColor::blue);
        }
      }
    }
  }

  P.init((int)PExt[1].ifloat, (int)PExt[1].jfloat, delta_1, 0, sign);
  P.setDisplay(selectDisplay);
  for (int i = 0; i < 3; i++) {
    P.ifloat = P.ifloat - di * sample_step;
    P.i = (int)P.ifloat;
    P.jfloat = P.jfloat - dj * sample_step;
    P.j = (int)P.jfloat;

    if (!outOfImage(P.i, P.j, 5, rows, cols)) {
      P.track(I, me, false);

      if (P.getState() == vpMeSite::NO_SUPPRESSION) {
        list.push_back(P);
        if (vpDEBUG_ENABLE(3)) {
          ip.set_i(P.i);
          ip.set_j(P.j);
          vpDisplay::displayCross(I, ip, 5, vpColor::green);
        }
      } else {
        if (vpDEBUG_ENABLE(3)) {
          ip.set_i(P.i);
          ip.set_j(P.j);
          vpDisplay::displayCross(I, ip, 10, vpColor::blue);
        }
      }
    }
  }

  me->setRange(memory_range);

  vpCDEBUG(1) << "end vpMeLine::sample() : ";
  vpCDEBUG(1) << n_sample << " point inserted in the list " << std::endl;
}

/*!

  Resample the line if the number of sample is less than 80% of the
  expected value.

  \note The expected value is computed thanks to the length of the
  line and the parameter which indicates the number of pixel between
  two points (vpMe::sample_step).

  \param I : Image in which the line appears.
*/
void vpMeLine::reSample(const vpImage<unsigned char> &I)
{
  double i1, j1, i2, j2;

  if (!me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  project(a, b, c, PExt[0].ifloat, PExt[0].jfloat, i1, j1);
  project(a, b, c, PExt[1].ifloat, PExt[1].jfloat, i2, j2);

  // Points extremites
  PExt[0].ifloat = i1;
  PExt[0].jfloat = j1;
  PExt[1].ifloat = i2;
  PExt[1].jfloat = j2;

  double d = sqrt(vpMath::sqr(i1 - i2) + vpMath::sqr(j1 - j2));

  unsigned int n = numberOfSignal();
  double expecteddensity = d / (double)me->getSampleStep();

  if ((double)n < 0.9 * expecteddensity) {
    double delta_new = delta;
    delta = delta_1;
    sample(I);
    delta = delta_new;
    //  2. On appelle ce qui n'est pas specifique
    {
      vpMeTracker::initTracking(I);
    }
  }
}

/*!

  Set the alpha value of the different vpMeSite to the value of delta.
*/
void vpMeLine::updateDelta()
{
  vpMeSite p_me;

  double angle_ = delta + M_PI / 2;
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
  diff = fabs(angle_ - angle_1);
  if (diff > 90)
    sign *= -1;

  angle_1 = angle_;

  for (std::list<vpMeSite>::iterator it = list.begin(); it != list.end(); ++it) {
    p_me = *it;
    p_me.alpha = delta;
    p_me.mask_sign = sign;
    *it = p_me;
  }
  delta_1 = delta;
}

/*!

  Track the line in the image I.

  \param I : Image in which the line appears.
*/
void vpMeLine::track(const vpImage<unsigned char> &I)
{
  vpCDEBUG(1) << "begin vpMeLine::track()" << std::endl;

  //  1. On fait ce qui concerne les droites (peut etre vide)
  {} //  2. On appelle ce qui n'est pas specifique
  {
    vpMeTracker::track(I);
  }

  // 3. On revient aux droites
  {
    // supression des points rejetes par les ME
    suppressPoints();
    setExtremities();

    // Estimation des parametres de la droite aux moindres carre
    try {
      leastSquare();
    } catch (...) {
      vpERROR_TRACE("Error caught");
      throw;
    }

    // recherche de point aux extremite de la droites
    // dans le cas d'un glissement
    seekExtremities(I);

    setExtremities();
    try {
      leastSquare();
    } catch (...) {
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

/*!

  Compute the two parameters \f$(\rho, \theta)\f$ of the line.

  \param I : Image in which the line appears.
*/
void vpMeLine::computeRhoTheta(const vpImage<unsigned char> &I)
{
  // rho = -c ;
  // theta = atan2(a,b) ;
  rho = fabs(c);
  theta = atan2(b, a);

  while (theta >= M_PI)
    theta -= M_PI;
  while (theta < 0)
    theta += M_PI;

  if (_useIntensityForRho) {

    /*  while(theta < -M_PI)	theta += 2*M_PI ;
    while(theta >= M_PI)	theta -= 2*M_PI ;

    // If theta is between -90 and -180 get the equivalent
    // between 0 and 90
    if(theta <-M_PI/2)
      {
        theta += M_PI ;
        rho *= -1 ;
      }
    // If theta is between 90 and 180 get the equivalent
    // between 0 and -90
    if(theta >M_PI/2)
      {
        theta -= M_PI ;
        rho *= -1 ;
      }
    */
    // convention pour choisir le signe de rho
    int i, j;
    i = vpMath::round((PExt[0].ifloat + PExt[1].ifloat) / 2);
    j = vpMath::round((PExt[0].jfloat + PExt[1].jfloat) / 2);

    int end = false;
    int incr = 10;

    int i1 = 0, i2 = 0, j1 = 0, j2 = 0;
    unsigned char v1 = 0, v2 = 0;

    int width_ = (int)I.getWidth();
    int height_ = (int)I.getHeight();
    update_indices(theta, i, j, incr, i1, i2, j1, j2);

    if (i1 < 0 || i1 >= height_ || i2 < 0 || i2 >= height_ || j1 < 0 || j1 >= width_ || j2 < 0 || j2 >= width_) {
      double rho_lim1 = fabs((double)i / cos(theta));
      double rho_lim2 = fabs((double)j / sin(theta));

      double co_rho_lim1 = fabs(((double)(height_ - i)) / cos(theta));
      double co_rho_lim2 = fabs(((double)(width_ - j)) / sin(theta));

      double rho_lim = (std::min)(rho_lim1, rho_lim2);
      double co_rho_lim = (std::min)(co_rho_lim1, co_rho_lim2);
      incr = (int)std::floor((std::min)(rho_lim, co_rho_lim));
      if (incr < INCR_MIN) {
        vpERROR_TRACE("increment is too small");
        throw(vpTrackingException(vpTrackingException::fatalError, "increment is too small"));
      }
      update_indices(theta, i, j, incr, i1, i2, j1, j2);
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
      update_indices(theta, i, j, incr, i1, i2, j1, j2);
    }

    if (theta >= 0 && theta <= M_PI / 2) {
      if (v2 < v1) {
        theta += M_PI;
        rho *= -1;
      }
    }

    else {
      double jinter;
      jinter = -c / b;
      if (v2 < v1) {
        theta += M_PI;
        if (jinter > 0) {
          rho *= -1;
        }
      }

      else {
        if (jinter < 0) {
          rho *= -1;
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

/*!

   Get the value of \f$\rho\f$, the distance between the origin and the
   point on the line with belong to the normal to the line crossing
   the origin.

   Depending on the convention described at the beginning of this
   class, \f$\rho\f$ is signed.

*/
double vpMeLine::getRho() const { return rho; }

/*!
   Get the value of the angle \f$\theta\f$.
*/
double vpMeLine::getTheta() const { return theta; }

/*!

  Get the extremities of the line.

  \param ip1 : Coordinates of the first extremity.
  \param ip2 : Coordinates of the second extremity.
*/
void vpMeLine::getExtremities(vpImagePoint &ip1, vpImagePoint &ip2)
{
  /*Return the coordinates of the extremities of the line*/
  ip1.set_i(PExt[0].ifloat);
  ip1.set_j(PExt[0].jfloat);
  ip2.set_i(PExt[1].ifloat);
  ip2.set_j(PExt[1].jfloat);
}

/*!

  Computes the intersection point of two lines. The result is given in
  the (i,j) frame.

  \param line1 : The first line.
  \param line2 : The second line.
  \param ip : The coordinates of the intersection point.

  \return Returns a boolean value which depends on the computation
  success. True means that the computation ends successfully.
*/
bool vpMeLine::intersection(const vpMeLine &line1, const vpMeLine &line2, vpImagePoint &ip)
{
  double a1 = line1.a;
  double b1 = line1.b;
  double c1 = line1.c;
  double a2 = line2.a;
  double b2 = line2.b;
  double c2 = line2.c;

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
  } catch (...) {
    return (false);
  }
}

/*!
  Display of a moving line thanks to its equation parameters and its
  extremities.

  \param I : The image used as background.

  \param PExt1 : First extrimity

  \param PExt2 : Second extrimity

  \param A : Parameter a of the line equation a*i + b*j + c = 0

  \param B : Parameter b of the line equation a*i + b*j + c = 0

  \param C : Parameter c of the line equation a*i + b*j + c = 0

  \param color : Color used to display the line.

  \param thickness : Thickness of the line.
*/
void vpMeLine::display(const vpImage<unsigned char> &I, const vpMeSite &PExt1, const vpMeSite &PExt2, const double &A,
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
    // vpDisplay::flush(I);

  } else {
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
    // vpDisplay::flush(I);
  }

  ip1.set_i(PExt1.ifloat);
  ip1.set_j(PExt1.jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);

  ip1.set_i(PExt2.ifloat);
  ip1.set_j(PExt2.jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);
}

/*!
  Display of a moving line thanks to its equation parameters and its
  extremities.

  \param I : The image used as background.

  \param PExt1 : First extrimity

  \param PExt2 : Second extrimity

  \param A : Parameter a of the line equation a*i + b*j + c = 0

  \param B : Parameter b of the line equation a*i + b*j + c = 0

  \param C : Parameter c of the line equation a*i + b*j + c = 0

  \param color : Color used to display the line.

  \param thickness : Thickness of the line.
*/
void vpMeLine::display(const vpImage<vpRGBa> &I, const vpMeSite &PExt1, const vpMeSite &PExt2, const double &A,
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
    // vpDisplay::flush(I);

  } else {
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
    // vpDisplay::flush(I);
  }

  ip1.set_i(PExt1.ifloat);
  ip1.set_j(PExt1.jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);

  ip1.set_i(PExt2.ifloat);
  ip1.set_j(PExt2.jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);
}

/*!
  Display of a moving line thanks to its equation parameters and its
  extremities with all the site list.

  \param I : The image used as background.

  \param PExt1 : First extrimity

  \param PExt2 : Second extrimity

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
  vpImagePoint ip;

  for (std::list<vpMeSite>::const_iterator it = site_list.begin(); it != site_list.end(); ++it) {
    vpMeSite pix = *it;
    ip.set_i(pix.ifloat);
    ip.set_j(pix.jfloat);

    if (pix.getState() == vpMeSite::M_ESTIMATOR)
      vpDisplay::displayCross(I, ip, 5, vpColor::green, thickness);
    else
      vpDisplay::displayCross(I, ip, 5, color, thickness);

    // vpDisplay::flush(I);
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
    // vpDisplay::flush(I);

  } else {
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
    // vpDisplay::flush(I);
  }

  ip1.set_i(PExt1.ifloat);
  ip1.set_j(PExt1.jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);

  ip1.set_i(PExt2.ifloat);
  ip1.set_j(PExt2.jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);
}

/*!
  Display of a moving line thanks to its equation parameters and its
  extremities with all the site list.

  \param I : The image used as background.

  \param PExt1 : First extrimity

  \param PExt2 : Second extrimity

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
  vpImagePoint ip;

  for (std::list<vpMeSite>::const_iterator it = site_list.begin(); it != site_list.end(); ++it) {
    vpMeSite pix = *it;
    ip.set_i(pix.ifloat);
    ip.set_j(pix.jfloat);

    if (pix.getState() == vpMeSite::M_ESTIMATOR)
      vpDisplay::displayCross(I, ip, 5, vpColor::green, thickness);
    else
      vpDisplay::displayCross(I, ip, 5, color, thickness);

    // vpDisplay::flush(I);
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
    // vpDisplay::flush(I);

  } else {
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
    // vpDisplay::flush(I);
  }

  ip1.set_i(PExt1.ifloat);
  ip1.set_j(PExt1.jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);

  ip1.set_i(PExt2.ifloat);
  ip1.set_j(PExt2.jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);
}
