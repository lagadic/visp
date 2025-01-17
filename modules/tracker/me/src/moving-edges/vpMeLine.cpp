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
#include <visp3/core/vpMatrixException.h>
#include <visp3/me/vpMe.h>
#include <visp3/me/vpMeLine.h>
#include <visp3/me/vpMeSite.h>
#include <visp3/me/vpMeTracker.h>

#define INCR_MIN 1

BEGIN_VISP_NAMESPACE

void vpMeLine::normalizeAngle(double &delta)
{
  // angle in [- M_PI; M_PI]
  while (delta > M_PI) {
    delta -= M_PI;
  }
  while (delta < -M_PI) {
    delta += M_PI;
  }
}

void vpMeLine::computeDelta(double &delta, double i1, double j1, double i2, double j2)
{

  double B = (double)(i1 - i2);
  double A = (double)(j1 - j2);

  delta = atan2(B, A);
  delta -= M_PI / 2.0;
  normalizeAngle(delta);
}

/*!
 * Project a ME site on a line.
 * @param[in] a,b,c : Parameters of the line
 * @param[in] P : ME site.
 * @param[out] iP : Coordinates of the ME site projected on the line.
 */
void vpMeLine::project(double a, double b, double c, const vpMeSite &P, vpImagePoint &iP)
{
  const double i = P.m_ifloat;
  const double j = P.m_jfloat;
  double ip, jp;

  //  FC 1 seul cas a faire. Rajouter normalisation si a*a+b*b <> 1 car future
  // fonction externe
  if (fabs(a) > fabs(b)) {
    jp = (vpMath::sqr(a) * j - a * b * i - c * b) / (vpMath::sqr(a) + vpMath::sqr(b));
    ip = (-c - b * jp) / a;
  }
  else {
    ip = (vpMath::sqr(b) * i - a * b * j - c * a) / (vpMath::sqr(a) + vpMath::sqr(b));
    jp = (-c - a * ip) / b;
  }
  iP.set_i(ip);
  iP.set_j(jp);
}
/* old version
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
*/

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

  double sampleStep = fabs(m_me->getSampleStep());
  if (sampleStep <= std::numeric_limits<double>::epsilon()) {
    vpERROR_TRACE("function called with sample step = 0");
    throw(vpTrackingException(vpTrackingException::fatalError, "sample step = 0"));
  }

  // i, j portions of the line_p
  double diffsi = m_PExt[1].m_ifloat - m_PExt[0].m_ifloat;
  double diffsj = m_PExt[1].m_jfloat - m_PExt[0].m_jfloat;

  double length_p = sqrt((vpMath::sqr(diffsi) + vpMath::sqr(diffsj)));
  if (length_p < (2.0 * sampleStep)) {
    throw(vpTrackingException(vpTrackingException::fatalError, "points too close of each other to define a line"));
  }
  // number of samples along line_p
  double n_sample = length_p / sampleStep;

  double stepi = diffsi / n_sample;
  double stepj = diffsj / n_sample;

  // Choose starting point
  double is = m_PExt[0].m_ifloat;
  double js = m_PExt[0].m_jfloat;

  // Delete old list
  m_meList.clear();

  // sample positions at i*m_me->getSampleStep() interval along the
  // line_p, starting at PSiteExt[0]

  int nbrows = static_cast<int>(I.getHeight());
  int nbcols = static_cast<int>(I.getWidth());
  const double marginRatio = m_me->getThresholdMarginRatio();

  for (int i = 0; i <= vpMath::round(n_sample); i++) {
    vpImagePoint iP;
    iP.set_ij(is, js);
    if (!outOfImage(iP, 5, nbrows, nbcols)) {
      unsigned int is_uint = static_cast<unsigned int>(is);
      unsigned int js_uint = static_cast<unsigned int>(js);
      if (inRoiMask(m_mask, is_uint, js_uint) && inMeMaskCandidates(m_maskCandidates, is_uint, js_uint)) {
        vpMeSite pix;
        pix.init(is, js, m_delta, 0, m_sign);
        pix.setDisplay(m_selectDisplay);
        pix.setState(vpMeSite::NO_SUPPRESSION);
        double convolution = pix.convolution(I, m_me);
        double contrastThreshold = fabs(convolution) * marginRatio;
        pix.setContrastThreshold(contrastThreshold, *m_me);

        if (vpDEBUG_ENABLE(3)) {
          vpDisplay::displayCross(I, iP, 2, vpColor::blue);
        }
        m_meList.push_back(pix);
      }
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

void vpMeLine::display(const vpImage<vpRGBa> &I, const vpColor &color, unsigned int thickness)
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

void vpMeLine::leastSquare(const vpImage<unsigned char> &I)
{
  // FC old version
  /*
  {
  vpMatrix A(numberOfSignal(), 2);
  vpColVector x(2), x_1(2);
  x_1 = 0;

  vpRobust r;
  r.setMinMedianAbsoluteDeviation(2);
  // FC vpMatrix D(numberOfSignal(), numberOfSignal());
  // FC D.eye();
  // FC vpMatrix DA(numberOfSignal(), 2);
  // FC vpColVector w(numberOfSignal());
  vpColVector B(numberOfSignal());
  // FC w = 1;
  vpMeSite p_me;
  unsigned int iter = 0;
  unsigned int nos_1 = 0;
  double distance = 100;

  if (m_meList.size() <= 2 || numberOfSignal() <= 2) {
    // FC print
    printf("m_meList.size %ld, numberOfSignal %d\n",m_meList.size(), numberOfSignal());
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
        A[k][1] = 1.0;
        B[k] = -p_me.m_jfloat;
  // FC print
        printf("Av0: nb %d, x = %lf, y= %lf\n",k, p_me.m_ifloat, p_me.m_jfloat);
  k++;
      }
    }
    // modif FC
    vpMatrix DA(k, 2);
    vpColVector Db(k);
    vpColVector w(k);
    w = 1.0;
    // fin modif FC

    while (iter < 4 && distance > 0.05) {
      for (unsigned int i = 0; i < k; i++) {
        for (unsigned int j = 0; j < 2; j++) {
          DA[i][j] = w[i] * A[i][j];
        }
  // modif FC
  Db[i] = w[i] * B[i];
  // fin modif FC
      }
      // modif FC
      x = DA.solveBySVD(Db);
      // x = DA.pseudoInverse(1e-26) * D * B;
      vpColVector residu(k);
      // vpColVector residu(nos_1);
      // fin modif FC
      // FC print
      printf("v0, iter %d, a = %lf, b = %lf, c = %lf\n", iter, x[0]/sqrt(1.0+x[0]*x[0]),  1.0/sqrt(1.0+x[0]*x[0]), x[1]/sqrt(1.0+x[0]*x[0]));
      // modif FC
      for (unsigned int i = 0; i < k; i++) {
  residu[i] =  B[i] - A[i][0] * x[0] - A[i][1] * x[1];
  // fin modif FC
      }
      // residu = B - A * x;
      // fin modif FC
      r.MEstimator(vpRobust::TUKEY, residu, w);
      // modif FC
      // k = 0;  // FC gros bug ici. il ne faut pas aller a nos_1
      // for (unsigned int i = 0; i < nos_1; i++) {
      //   D[k][k] = w[k];
      //  k++;
      // }

      // fin modif FC
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
    m_b = 1.0;
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
        A[k][1] = 1.0;
        B[k] = -p_me.m_ifloat;
  // FC print
        printf("Av0: nb %d, x = %lf, y= %lf\n",k, p_me.m_ifloat, p_me.m_jfloat);
        k++;
      }
    }
        // modif FC
    vpMatrix DA(k, 2);
    vpColVector Db(k);
    vpColVector w(k);
    w = 1.0;
    // fin modif FC

    while (iter < 4 && distance > 0.05) {
      for (unsigned int i = 0; i < k; i++) {
        for (unsigned int j = 0; j < 2; j++) {
          DA[i][j] = w[i] * A[i][j];
        }
  // modif FC
  Db[i] = w[i] * B[i];
  // fin modif FC
      }
      // modif FC
      x = DA.solveBySVD(Db);
      // x = DA.pseudoInverse(1e-26) * D * B;
      vpColVector residu(k);
      // vpColVector residu(nos_1);
      // fin modif FC
      // FC print
      printf("v0, iter %d, a = %lf, b = %lf, c = %lf\n", iter, 1.0/sqrt(1.0+x[0]*x[0]),  x[0]/sqrt(1.0+x[0]*x[0]), x[1]/sqrt(1.0+x[0]*x[0]));
      // modif FC
      for (unsigned int i = 0; i < k; i++) {
  residu[i] =  B[i] - A[i][0] * x[0] - A[i][1] * x[1];
  // fin modif FC
      }
      // residu = B - A * x;
      // fin modif FC
      r.MEstimator(vpRobust::TUKEY, residu, w);
      // modif FC
      // k = 0;  // FC gros bug ici. il ne faut pas aller a nos_1
      // for (unsigned int i = 0; i < nos_1; i++) {
      //    D[k][k] = w[k];
      //    k++;
      //  }
      // fin modif FC
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
  // FC print
  printf("a = %lf, b = %lf, c = %lf\n",m_a, m_b, m_c);
  // FC fin old version
}
*/
  // {
  // FC new version
  const unsigned int nos = numberOfSignal(); // number of MEs correctly tracked

  // FC print
  // printf("debut leastSquare : nos = %d\n",nos);

  if (m_meList.size() <= 2 || nos <= 2) {
    // FC print
    // printf("m_meList.size %ld, numberOfSignal %d\n",m_meList.size(), numberOfSignal());
    // vpERROR_TRACE("Not enough point") ;
    vpCDEBUG(1) << "Not enough point";
    throw(vpTrackingException(vpTrackingException::notEnoughPointError, "not enough point"));
  }

  const double nbr = I.getHeight() / 2.;
  const double nbc = I.getWidth() / 2.;
  unsigned int k = 0;  // number of good points (should be numberOfSignal
                     // at the end of the first loop and then all after)

  // System A x = 0 to be solved by least squares
  // with A = (u v 1) and x = (au, bu, cu)

  // Note that the (nos-k) last rows of A, xp and yp are not used.
  // Hopefully, this is not an issue.
  vpMatrix A(nos, 3);

    // Useful to compute the weights in the robust estimation
  vpColVector xp(nos), yp(nos);
  std::list<vpMeSite>::const_iterator end = m_meList.end();

  for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != end; ++it) {
    vpMeSite p_me = *it;
    // FC uncomment following line if new version only
    if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
    // FC following line for comparison with old method. Comment otherwise
    // if ((p_me.getState() == vpMeSite::NO_SUPPRESSION) ||
    //    (p_me.getState() == vpMeSite::M_ESTIMATOR)) {
      // from (i,j) to (u,v) frame so that (u,v) in [-1;1]
      double u = (p_me.m_ifloat - nbr) / nbr;
      double v = (p_me.m_jfloat - nbc) / nbc;
      A[k][0] = u;
      A[k][1] = v;
      A[k][2] = 1.0;
      // Useful to compute the weights in the robust estimation
      xp[k] = p_me.m_ifloat;
      yp[k] = p_me.m_jfloat;
      // printf("Av1: nb %d, x = %lf, y= %lf\n",k, xp[k],yp[k]);
      ++k;
    }
    // FC print
    // else {
    //   printf("mauvais point : x = %lf, y = %lf, status = %d\n", p_me.m_ifloat, p_me.m_jfloat, p_me.getState());
    // }
  }
  // FC print
  // printf("debut leastSquare k = %d\n",k);

  const unsigned int minRequiredNbMe = 2;
  if (k < minRequiredNbMe) {
    throw(vpException(vpException::dimensionError, "Not enough moving edges %d / %d to track the line ",
                      k, m_meList.size()));
  }

  vpRobust r;
  r.setMinMedianAbsoluteDeviation(1.0); // Image noise in pixels for the  distance from points to line

  unsigned int iter = 0;
  double var = 10.0;
  vpColVector x(3);
  vpMatrix DA(k, 3);
  vpMatrix KerDA;
  vpColVector w(k);
  w = 1.0;
  vpColVector x_prev(3);
  x_prev = 0.0;

  // stop after 4 it or if  variation between 2 it is less than 1 pixel
  const unsigned int maxNbIter = 4;
  const unsigned int widthDA = DA.getCols();
  while ((iter < maxNbIter) && (var > 0.1)) {
    for (unsigned int i = 0; i < k; ++i) {
      for (unsigned int j = 0; j < widthDA; ++j) {
        DA[i][j] = w[i] * A[i][j];
      }
    }
    unsigned int dim = DA.nullSpace(KerDA, 1);
    if (dim > 1) { // case with less than 2 independent points
      throw(vpMatrixException(vpMatrixException::rankDeficient, "Linear system for computing the line equation ill conditioned"));
    }

    for (unsigned int i = 0; i < 3; ++i) {
      x[i] = KerDA[i][0]; // norm(x) = 1
    }

    // using inverse normalization to go back to pixel values
    double a, b, c;
    a = x[0]/nbr;
    b = x[1]/nbc;
    c = x[2] - x[0] - x[1];
    // normalization such that a^2+b^2 = 1
    // important to compute the distance between points and line
    const double norm = 1.0/sqrt(vpMath::sqr(a)+vpMath::sqr(b));
    x[0] = a * norm;
    x[1] = b * norm;
    x[2] = c * norm;
    // printf("v1, iter %d, a = %lf, b = %lf, c = %lf\n",iter, x[0], x[1], x[2]);

    var = (x - x_prev).frobeniusNorm();
    x_prev = x;

    // FC print
    // printf("var = %lf\n",var);
    vpColVector residu(k); // distance from points to line
    for (unsigned int i = 0; i < k; ++i) {
      residu[i] = x[0] * xp[i] + x[1] * yp[i] + x[2];
    }
    r.MEstimator(vpRobust::TUKEY, residu, w);

    // FC print
    // for (unsigned int i = 0; i < k; ++i) {
    //  printf("residu %d = %lf, w = %lf \n",i, residu[i], w[i]);
    // }

    ++iter;
  }
  m_a = x[0];
  m_b = x[1];
  m_c = x[2];
  // printf("a = %lf, b = %lf, c = %lf\n",m_a, m_b, m_c);

  // remove all bad points in the list
  unsigned int i = 0;
  end = m_meList.end();
  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != end;) {
    vpMeSite p_me = *it;
    if (p_me.getState() != vpMeSite::NO_SUPPRESSION) {
      // printf(" pas outlier sup : x = %lf, y = %lf, status = %d\n", p_me.m_ifloat, p_me.m_jfloat, p_me.getState());
      it = m_meList.erase(it);
    }
    else {
      // remove outliers
      if (w[i] < 0.2) {  // m_thresholdWeight pour vpMeEllipse
        // printf("outlier sup : x = %lf, y = %lf, w = %lf, status = %d\n", p_me.m_ifloat, p_me.m_jfloat, w[i], p_me.getState());
        it = m_meList.erase(it);
      }
      else { // good point
        ++it;
      }
      ++i;
    }
  }
  // print FC
  //  printf("fin leastSquare: k %d\n",k);
  // printf("fin leastSquare: nb of good points %d\n",i);
  // printf("fin leastSquare: nb of non-outliers removed %d\n",k-i);
  // printf("fin leastSquare: nb of points removed %d\n",nos-i);
  // }
  // FC fin new version

  // mise a jour du delta
  m_delta = atan2(m_a, m_b);

  normalizeAngle(m_delta);
}

void vpMeLine::initTracking(const vpImage<unsigned char> &I, const vpImagePoint &ip1, const vpImagePoint &ip2)
{
  vpCDEBUG(1) << " begin vpMeLine::initTracking()" << std::endl;

  // 1. On fait ce qui concerne les droites
  // Points extremites
  double id1, jd1, id2, jd2;
  id1 = ip1.get_i();
  jd1 = ip1.get_j();
  id2 = ip2.get_i();
  jd2 = ip2.get_j();

  m_PExt[0].m_ifloat = id1;
  m_PExt[0].m_jfloat = jd1;
  m_PExt[1].m_ifloat = id2;
  m_PExt[1].m_jfloat = jd2;
  // FC For first point to be nearest of the origin to build an ordered
  // list of meSite but this is useless
  /*
  double d1 = id1 * id1 + jd1 * jd1;
  double d2 = id2 * id2 + jd2 * jd2;
  if (d1 < d2) {
    m_PExt[0].m_ifloat = id1;
    m_PExt[0].m_jfloat = jd1;
    m_PExt[1].m_ifloat = id2;
    m_PExt[1].m_jfloat = jd2;
  }
  else {
    m_PExt[0].m_ifloat = id2;
    m_PExt[0].m_jfloat = jd2;
    m_PExt[1].m_ifloat = id1;
    m_PExt[1].m_jfloat = jd1;
    }*/
  // FC fin test

  // FC : inutile (et c pas initialise)
  // double angle_ = atan2((id1 - id2), (jd1 - jd2));
  // m_a = cos(angle_);
  // m_b = sin(angle_);

  computeDelta(m_delta, id1, jd1, id2, jd2);
  m_delta_1 = m_delta;

  //      vpTRACE("a: %f b: %f -b/a: %f delta: %f", a, b , -(b/a), delta);

  sample(I);

  // 2. On appelle ce qui n'est pas specifique
  vpMeTracker::initTracking(I);
  // FC le track(I) initial est fait dans le programme principal
  // track(I);

  vpCDEBUG(1) << " end vpMeLine::initTracking()" << std::endl;
}
/* FC function now useless since done at the end of leastSquare()
void vpMeLine::suppressPoints()
{
  // Loop through list of sites to track
  std::list<vpMeSite>::const_iterator end = m_meList.end();
  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != end;) {
    vpMeSite s = *it; // current reference pixel

    if (s.getState() != vpMeSite::NO_SUPPRESSION)
      it = m_meList.erase(it);
    else
      ++it;
  }
}
*/
void vpMeLine::setExtremities()
{
  /* FC old version */
  /*
  double imin = +1e6;
  double jmin = +1e6;
  double imax = -1;
  double jmax = -1;

  // Loop through list of sites to track
  std::list<vpMeSite>::const_iterator end = m_meList.end();
  for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != end; ++it) {
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
    std::list<vpMeSite>::const_iterator end = m_meList.end();
    for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != end; ++it) {
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
  // FC fin old version
  // FC pour verif
  double x0, y0, x1, y1;
  x0 = m_PExt[0].m_ifloat;
  y0 = m_PExt[0].m_jfloat;
  x1 = m_PExt[1].m_ifloat;
  y1 = m_PExt[1].m_jfloat;
  printf("v0 P[0].i %lf , P[0].j %lf , P[1].i %lf , P[1].j %lf\n"
     ,m_PExt[0].m_ifloat, m_PExt[0].m_jfloat, m_PExt[1].m_ifloat , m_PExt[1].m_jfloat);
  */
  // FC new version
  m_PExt[0] = m_meList.front();
  m_PExt[1] = m_meList.back();
  // FC printf
  // printf("v1 P[0].i %lf , P[0].j %lf , P[1].i %lf , P[1].j %lf\n",
  // m_PExt[0].m_ifloat, m_PExt[0].m_jfloat,
  // m_PExt[1].m_ifloat,m_PExt[1].m_jfloat);
  // FC fin new version
  /*
 // FC verif
double d = sqrt(vpMath::sqr(x0-m_PExt[0].m_ifloat)+vpMath::sqr(y0-m_PExt[0].m_jfloat)+vpMath::sqr(x1-m_PExt[1].m_ifloat)+vpMath::sqr(y1-m_PExt[1].m_jfloat) );
  if (d > 0.00001) {
    printf("verif setExtremities\n");
    unsigned int i=0;
    std::list<vpMeSite>::const_iterator end = m_meList.end();
    for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != end; ++it) {
      vpMeSite s = *it; // current reference pixel
      printf("i %d, status %d, x = %lf, y = %lf\n", i, s.getState(), s.m_ifloat, s.m_jfloat);
      i++;
    }
 }
  // FC fin verif
  */
}

unsigned int vpMeLine::plugHoles(const vpImage<unsigned char> &I)
{
  vpCDEBUG(1) << "begin vpMeLine::plugHoles() : " << std::endl;

  if (!m_me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  // if (m_me->getSampleStep()==0)
  if (std::fabs(m_me->getSampleStep()) <= std::numeric_limits<double>::epsilon()) {

    vpERROR_TRACE("function called with sample step = 0");
    throw(vpTrackingException(vpTrackingException::fatalError, "sample step = 0"));
  }

  int nbrows = static_cast<int>(I.getHeight());
  int nbcols = static_cast<int>(I.getWidth());

  unsigned int memory_range = m_me->getRange();
  m_me->setRange(1);

  const double marginRatio = m_me->getThresholdMarginRatio();

  unsigned int nb_added_points = 0;

  std::list<vpMeSite>::iterator meList = m_meList.begin();
  std::list<vpMeSite>::const_iterator end = m_meList.end();

  vpImagePoint ip1, ip2;
  getExtremities(ip1, ip2);
  // FC pour ajouter le plus de points possibles
    // i, j portions of the line_p
  const double sampleStep = fabs(m_me->getSampleStep());
  double diffsi = m_PExt[1].m_ifloat - m_PExt[0].m_ifloat;
  double diffsj = m_PExt[1].m_jfloat - m_PExt[0].m_jfloat;
  double length = sqrt((vpMath::sqr(diffsi) + vpMath::sqr(diffsj)));
  double stepi = diffsi * sampleStep / length;
  double stepj = diffsj * sampleStep / length;
  // FC fin pour ajouter le plus de points possibles

  vpMeSite pix1 = *meList;
  project(m_a, m_b, m_c, pix1, ip1);
  ++meList;
  while (meList != end) {
    vpMeSite pix2 = *meList;
    project(m_a, m_b, m_c, pix2, ip2);

    double dist = sqrt(vpMath::sqr(ip1.get_i() - ip2.get_i())
                     + vpMath::sqr(ip1.get_j() - ip2.get_j()));
    const unsigned int n_sample = static_cast<unsigned int>(floor(dist / sampleStep));
    if (n_sample > 1) {
      // FC print
      // printf("n_sample %d\n",n_sample);
      double is = ip1.get_i();
      double js = ip1.get_j();
      for (unsigned int i = 0; i<n_sample; ++i) {
        vpImagePoint iP;
        iP.set_ij(is, js);
        if (!outOfImage(iP, 5, nbrows, nbcols)) {
          unsigned int is_uint = static_cast<unsigned int>(is);
          unsigned int js_uint = static_cast<unsigned int>(js);
          if (inRoiMask(m_mask, is_uint, js_uint) && inMeMaskCandidates(m_maskCandidates, is_uint, js_uint)) {
            vpMeSite pix;
            pix.init(is, js, m_delta, 0, m_sign);
            pix.setDisplay(m_selectDisplay);
            pix.setState(vpMeSite::NO_SUPPRESSION);
            double convolution = pix.convolution(I, m_me);
            double contrastThreshold = fabs(convolution) * marginRatio;
            pix.setContrastThreshold(contrastThreshold, *m_me);
            pix.track(I, m_me, false);
            if (pix.getState() == vpMeSite::NO_SUPPRESSION) { // good point
              nb_added_points++;
              m_meList.insert(meList, pix);
            }
            if (vpDEBUG_ENABLE(3)) {
              vpDisplay::displayCross(I, iP, 2, vpColor::blue);
            }
          }
        }
        is += stepi;
        js += stepj;
      }
    }
    /*
    // Only one point is added if two neighboring points are too far away
    if (dist > (2.0 * m_me->getSampleStep())) {
      // point added at the middle of the 2 points
      vpImagePoint iP;
      double xm = (pix1.get_ifloat()+pix2.get_ifloat())/2.0;
      double ym = (pix1.get_jfloat()+pix2.get_jfloat())/2.0;
      iP.set_ij(xm, ym);
      if (!outOfImage(iP, 0, nbrows, nbcols)) {
        unsigned int is_uint = static_cast<unsigned int>(iP.get_i());
        unsigned int js_uint = static_cast<unsigned int>(iP.get_j());
        if (inRoiMask(m_mask, is_uint, js_uint)) {
          vpMeSite pix;
          pix.init(iP.get_i(), iP.get_j(), m_delta_1, 0, m_sign);
          pix.setDisplay(m_selectDisplay);
          pix.setState(vpMeSite::NO_SUPPRESSION);
          double convolution = pix.convolution(I, m_me);
          double contrastThreshold = fabs(convolution) * marginRatio;
          pix.setContrastThreshold(contrastThreshold, *m_me);
          pix.track(I, m_me, false);
          if (pix.getState() == vpMeSite::NO_SUPPRESSION) { // good point
            nb_added_points++;
            m_meList.insert(meList, pix);
          }
        }
      }
    }
    */
    pix1 = pix2;
    ip1 = ip2;
    ++meList;
  }
  m_me->setRange(memory_range);

  // FC print
  // printf("nb added points plugHoles %d\n",nb_added_points);

  vpCDEBUG(1) << nb_added_points << " point inserted in the list with plugHoles" << std::endl;
  return(nb_added_points);
}

unsigned int vpMeLine::seekExtremities(const vpImage<unsigned char> &I)
{
  vpCDEBUG(1) << "begin vpMeLine::seekExtremities() : " << std::endl;

  // FC : Tests inutiles car deja faits dans plugHoles qui est fait
  // systematiquement avant cette fonction
  /*
  if (!m_me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  // if (m_me->getSampleStep()==0)
  if (std::fabs(m_me->getSampleStep()) <= std::numeric_limits<double>::epsilon()) {

    vpERROR_TRACE("function called with sample step = 0");
    throw(vpTrackingException(vpTrackingException::fatalError, "sample step = 0"));
  }
  */
  // FC fin tests inutiles

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

  P.init((int)id1, (int)jd1, m_delta_1, 0, m_sign);
  P.setDisplay(m_selectDisplay);
  const double marginRatio = m_me->getThresholdMarginRatio();

  unsigned int memory_range = m_me->getRange();
  m_me->setRange(1);

  unsigned int nb_added_points = 0;

  // Try to add at max 3 points along first extremity
  // (could be a little bit more or less)
  for (int i = 0; i < 3; i++) {
    id1 -= di;
    jd1 -= dj;
    vpImagePoint iP;
    iP.set_ij(id1, jd1);

    // First test to ensure that iP coordinates are > 0 before casting to unsigned int
    if (!outOfImage(iP, 5, nbrows, nbcols)) {
      unsigned int is_uint = static_cast<unsigned int>(id1);
      unsigned int js_uint = static_cast<unsigned int>(jd1);
      // Note here that it is more efficent to cast the coordinates to unsigned int instead of using
      // directly the image point iP that contains floting point coordinates.
      // It allows to makes less tests.
      if (inRoiMask(m_mask, is_uint, js_uint) && inMeMaskCandidates(m_maskCandidates, is_uint, js_uint)) {
        // ajout
        P.m_ifloat = id1;
        P.m_i = static_cast<int>(id1);  // FC : Necessaire ?
        // FC : (et ici int alors que unsigned int pour inRoiMask et inMeMaskCandidates juste au dessus)
        P.m_jfloat = jd1;
        P.m_j = static_cast<int>(jd1);  // FC : idem
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

  // P.init((int)id2, (int)jd2, m_delta_1, 0, m_sign);
  // P.setDisplay(m_selectDisplay);

  for (int i = 0; i < 3; i++) {
    id2 += di;
    jd2 += dj;

    vpImagePoint iP;
    iP.set_i(id2);
    iP.set_j(jd2);

    // First test to ensure that iP coordinates are > 0 before casting to unsigned int
    if (!outOfImage(iP, 5, nbrows, nbcols)) {
      unsigned int is_uint = static_cast<unsigned int>(id2);
      unsigned int js_uint = static_cast<unsigned int>(jd2);
      if (inRoiMask(m_mask, is_uint, js_uint) && inMeMaskCandidates(m_maskCandidates, is_uint, js_uint)) {
        // ajout
        P.m_ifloat = id2;
        P.m_i = static_cast<int>(id2);  // FC : Necessaire ?
        // FC : (et ici int alors que unsigned int pour inRoiMask et inMeMaskCandidates juste au dessus)
        P.m_jfloat = jd2;
        P.m_j = static_cast<int>(jd2);  // FC : idem
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

  vpCDEBUG(1) << nb_added_points << " point inserted in the list with seekExtremities" << std::endl;
  // FC print
  // printf("nb added points seekExtremities %d\n",nb_added_points);
  return(nb_added_points);
}

void vpMeLine::reSample(const vpImage<unsigned char> &I)
{
  if (!m_me) {
    vpDERROR_TRACE(2, "Tracking error: Moving edges not initialized");
    throw(vpTrackingException(vpTrackingException::initializationError, "Moving edges not initialized"));
  }

  // FC nouvelle version
  // FC printf
  // printf("resample : nb de bons points dans la liste %ld\n",m_meList.size());

  const unsigned int minNbGoodPoints = 4;
  if (m_meList.size() <= minNbGoodPoints) {

    double delta_new = m_delta;
    m_delta = m_delta_1;

    // printf FC
    // printf("do resample\n");

    vpImagePoint ip;
    const vpMeSite PExt0 = m_PExt[0];
    project(m_a, m_b, m_c, PExt0, ip);
    m_PExt[0].m_ifloat = ip.get_i();
    m_PExt[0].m_jfloat = ip.get_j();

    const vpMeSite PExt1 = m_PExt[1];
    project(m_a, m_b, m_c, PExt1, ip);
    m_PExt[1].m_ifloat = ip.get_i();
    m_PExt[1].m_jfloat = ip.get_j();

    sample(I);
    m_delta = delta_new;

    //  2. On appelle ce qui n'est pas specifique
    vpMeTracker::initTracking(I);
    // on fait un tracking apres resampling
    vpMeTracker::track(I);

    // Estimation des parametres de la droite aux moindres carre
    leastSquare(I);

    setExtremities(); // useful if a resampling a getExtremities are done

    // remet a jour l'angle delta pour chaque  point de la liste
    updateDelta();

    // Remise a jour de delta dans la liste de site me
    if (vpDEBUG_ENABLE(2)) {
      display(I, vpColor::red);
      vpMeTracker::display(I);
      vpDisplay::flush(I);
    }

    computeRhoTheta(I);
  }
  // FC old version
  /*
  double i1, j1, i2, j2;

  project(m_a, m_b, m_c, m_PExt[0].m_ifloat, m_PExt[0].m_jfloat, i1, j1);
  project(m_a, m_b, m_c, m_PExt[1].m_ifloat, m_PExt[1].m_jfloat, i2, j2);

  double d = sqrt(vpMath::sqr(i1 - i2) + vpMath::sqr(j1 - j2));

  unsigned int n = numberOfSignal();
  double expecteddensity = d / (double)m_me->getSampleStep();

  if ((double)n < 0.9 * expecteddensity) {
    //printf FC
    printf("do resample\n");
      // New point extremities strictly on the straight line
    m_PExt[0].m_ifloat = i1;
    m_PExt[0].m_jfloat = j1;
    m_PExt[1].m_ifloat = i2;
    m_PExt[1].m_jfloat = j2;

    // printf FC
    printf("re P[0].i %lf , P[0].j %lf , P[1].i %lf , P[1].j %lf\n"
     ,m_PExt[0].m_ifloat, m_PExt[0].m_jfloat, m_PExt[1].m_ifloat , m_PExt[1].m_jfloat);

    double delta_new = m_delta;
    m_delta = m_delta_1;
    sample(I);
    m_delta = delta_new;
    //  2. On appelle ce qui n'est pas specifique
    vpMeTracker::initTracking(I);
  }
  */
}

void vpMeLine::updateDelta()
{
  vpMeSite p_me;

  double angle_ = m_delta + M_PI / 2;
  double diff = 0;

  // angle in [0;180]
  while (angle_ < 0)
    angle_ += M_PI;
  while (angle_ > M_PI)
    angle_ -= M_PI;

  // angle in degree
  angle_ = vpMath::round(angle_ * 180.0 / M_PI);
  // modif FC
  int angle_int = (int)angle_;
  if (angle_int == 180) angle_ = 179.0;
  /* FC pas logique de passer de 180 a 0. 179 plus proche !
  // if(fabs(angle_) == 180 )
  if (std::fabs(std::fabs(angle_) - 180) <= std::numeric_limits<double>::epsilon()) {
    angle_ = 0;
  }
  */
  // fin modif FC

  // std::cout << "angle theta : " << angle_ << std::endl ;
  diff = fabs(angle_ - m_angle_1);
  if (diff > 90)  // FC : pour assurer continuite ?????
    m_sign *= -1;

  m_angle_1 = angle_;

  std::list<vpMeSite>::const_iterator end = m_meList.end();
  for (std::list<vpMeSite>::iterator it = m_meList.begin(); it != end; ++it) {
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

  // supression des points rejetes par les ME
  // FC inutile car leastSquare ne prend que les bons points
  // suppressPoints();

  // FC inutile car inutilise dans leastSquare
  // setExtremities();

  // Estimation des parametres de la droite aux moindres carre
  leastSquare(I);

  // keep only good points in the list
  // FC useless since leastSquare remove the bad points
  // suppressPoints();
  // Try adding adding new points in holes
  unsigned int nb = plugHoles(I);
  // Try adding adding new points at both extremities
  nb += seekExtremities(I);
  if (nb > 0) {
    // computation of new line parameters if points are added
    // FC inutile
    // setExtremities();

    leastSquare(I);

    // suppression des points rejetes par la regression robuste
    // FC useless since leastSquare remove the bad points
    // suppressPoints();
  }
  setExtremities(); // useful if a resampling a getExtremities are done

  // reechantillonage si necessaire
  reSample(I);

  // remet a jour l'angle delta pour chaque point de la liste
  updateDelta();

  // Remise a jour de delta dans la liste de site me
  if (vpDEBUG_ENABLE(2)) {
    display(I, vpColor::red);
    vpMeTracker::display(I);
    vpDisplay::flush(I);
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

  // angle in [0;180]
  while (m_theta >= M_PI)
    m_theta -= M_PI;
  while (m_theta < 0)
    m_theta += M_PI;

  if (m_useIntensityForRho) {
    // convention pour choisir le signe de rho
    int i, j;
    i = vpMath::round((m_PExt[0].m_ifloat + m_PExt[1].m_ifloat) / 2.0);
    j = vpMath::round((m_PExt[0].m_jfloat + m_PExt[1].m_jfloat) / 2.0);

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
                            "there is no grey level difference between both "
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

void vpMeLine::getExtremities(vpImagePoint &ip1, vpImagePoint &ip2) const
{
  /* Return the coordinates of the two extremities of the line*/

  const vpMeSite P0 = m_meList.front();
  project(m_a, m_b, m_c, P0, ip1);
  // project(m_a, m_b, m_c, m_PExt[0].m_ifloat, m_PExt[0].m_jfloat, id1, jd1);
  // m_PExt[0].m_ifloat = id1;
  // m_PExt[0].m_jfloat = jd1;

  const vpMeSite P1 = m_meList.back();
  project(m_a, m_b, m_c, P1, ip2);
  // project(m_a, m_b, m_c, m_PExt[1].m_ifloat, m_PExt[1].m_jfloat, id2, jd2);
  // m_PExt[1].m_ifloat = id2;
  // m_PExt[1].m_jfloat = jd2;
}

bool vpMeLine::intersection(const vpMeLine &line1, const vpMeLine &line2, vpImagePoint &iP)
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
        return false;
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
        return false;
      }

      // if (denom != 0 )
      if (std::fabs(denom) > std::numeric_limits<double>::epsilon()) {
        i = ((b2 / b1) * c1 - c2) / denom;
        j = (-a1 * i - c1) / b1;
      }
    }
    iP.set_i(i);
    iP.set_j(j);

    return true;
  }
  catch (...) {
    return false;
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

  // FC 1 seul cas a faire... Idem pour les autres displayLine

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

  project(A, B, C, PExt1, ip1);
  // ip1.set_i(PExt1.m_ifloat);
  // ip1.set_j(PExt1.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);

  project(A, B, C, PExt2, ip1);
  // ip1.set_i(PExt2.m_ifloat);
  // ip1.set_j(PExt2.m_jfloat);
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
  project(A, B, C, PExt1, ip1);
  // ip1.set_i(PExt1.m_ifloat);
  // ip1.set_j(PExt1.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);

  project(A, B, C, PExt2, ip1);
  // ip1.set_i(PExt2.m_ifloat);
  // ip1.set_j(PExt2.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);
}

void vpMeLine::displayLine(const vpImage<unsigned char> &I, const vpMeSite &PExt1, const vpMeSite &PExt2,
                           const std::list<vpMeSite> &site_list, const double &A, const double &B, const double &C,
                           const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip;
  std::list<vpMeSite>::const_iterator end = site_list.end();

  for (std::list<vpMeSite>::const_iterator it = site_list.begin(); it != end; ++it) {
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
  project(A, B, C, PExt1, ip1);
  // ip1.set_i(PExt1.m_ifloat);
  // ip1.set_j(PExt1.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);

  project(A, B, C, PExt2, ip1);
  // ip1.set_i(PExt2.m_ifloat);
  // ip1.set_j(PExt2.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);
}

void vpMeLine::displayLine(const vpImage<vpRGBa> &I, const vpMeSite &PExt1, const vpMeSite &PExt2,
                           const std::list<vpMeSite> &site_list, const double &A, const double &B, const double &C,
                           const vpColor &color, unsigned int thickness)
{
  vpImagePoint ip;
  std::list<vpMeSite>::const_iterator end = site_list.end();

  for (std::list<vpMeSite>::const_iterator it = site_list.begin(); it != end; ++it) {
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
  project(A, B, C, PExt1, ip1);
  // ip1.set_i(PExt1.m_ifloat);
  // ip1.set_j(PExt1.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);

  project(A, B, C, PExt2, ip1);
  // ip1.set_i(PExt2.m_ifloat);
  // ip1.set_j(PExt2.m_jfloat);
  vpDisplay::displayCross(I, ip1, 10, vpColor::green, thickness);
}
END_VISP_NAMESPACE
