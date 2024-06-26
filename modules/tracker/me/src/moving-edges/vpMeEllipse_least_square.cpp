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
 */

#include <visp3/core/vpConfig.h>
#include <visp3/me/vpMeEllipse.h>
#include <visp3/core/vpMatrixException.h>
#include <visp3/core/vpRobust.h>
#include <visp3/core/vpTrackingException.h>

BEGIN_VISP_NAMESPACE

void vpMeEllipse::leastSquare(const vpImage<unsigned char> &I, const std::vector<vpImagePoint> &iP)
{
  double um = I.getWidth() / 2.;
  double vm = I.getHeight() / 2.;
  unsigned int n = static_cast<unsigned int>(iP.size());
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;

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
      A[k][index_0] = u;
      A[k][index_1] = v;
      A[k][index_2] = 1.0;
      b[k] = (u * u) + (v * v);
    }
    vpColVector x(3);
    x = A.solveBySVD(b);
    // A circle is a particular ellipse. Going from x for circle to K for ellipse
    // using inverse normalization to go back to pixel values
    double ratio = vm / um;
    m_K[index_0] = (m_K[index_1] = (1.0 / (um * um)));
    m_K[index_2] = 0.0;
    m_K[index_3] = -(1.0 + (x[index_0] / 2.0)) / um;
    m_K[index_4] = -(ratio + (x[index_1] / 2.0)) / um;
    m_K[index_5] = -x[index_2] + 1.0 + (ratio * ratio) + x[index_0] + (ratio * x[index_1]);
  }
  else { // we track an ellipse
    const unsigned int npoints_min = 5;
    if (n < npoints_min) {
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
      A[k][index_0] = u * u;
      A[k][index_1] = v * v;
      A[k][index_2] = 2.0 * u * v;
      A[k][index_3] = 2.0 * u;
      A[k][index_4] = 2.0 * v;
      A[k][index_5] = 1.0;
    }
    vpMatrix KerA;
    unsigned int dim = A.nullSpace(KerA, 1);
    if (dim > 1) { // case with less than 5 independent points
      throw(vpMatrixException(vpMatrixException::rankDeficient, "Linear system for computing the ellipse equation ill conditioned"));
    }
    unsigned int nbRows = m_K.getRows();
    for (unsigned int i = 0; i < nbRows; ++i) {
      m_K[i] = KerA[i][0];
    }

    // inverse normalization
    m_K[index_0] *= vm / um;
    m_K[index_1] *= um / vm;
    m_K[index_3] = (m_K[index_3] * vm) - (m_K[index_0] * um) - (m_K[index_2] * vm);
    m_K[index_4] = (m_K[index_4] * um) - (m_K[index_1] * vm) - (m_K[index_2] * um);
    m_K[index_5] = (m_K[index_5] * um * vm) - (m_K[index_0] * um * um) - (m_K[index_1] * vm * vm) -
      (2.0 * m_K[index_2] * um * vm) - (2.0 * m_K[index_3] * um) - (2.0 * m_K[index_4] * vm);
  }
  getParameters();
}

void vpMeEllipse::leastSquareRobustCircle(const double &um, const double &vm, unsigned int &k, vpColVector &w)
{
  const unsigned int nos = numberOfSignal();
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;

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
      A[k][index_0] = u;
      A[k][index_1] = v;
      A[k][index_2] = 1.0;
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
    m_K[index_0] = (m_K[index_1] = (1.0 / (um * um)));
    m_K[index_2] = 0.0;
    m_K[index_3] = -(1.0 + (x[index_0] / 2.0)) / um;
    m_K[index_4] = -(ratio + (x[index_1] / 2.0)) / um;
    m_K[index_5] = -x[index_2] + 1.0 + (ratio * ratio) + x[index_0] + (ratio * x[index_1]);

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
      double sign = (m_K[index_0] * x * x) + (m_K[index_1] * y * y) + (2. * m_K[index_2] * x * y)
        + (2. * m_K[index_3] * x) + (2. * m_K[index_4] * y) + m_K[index_5];
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

void vpMeEllipse::leastSquareRobustEllipse(const double &um, const double &vm, unsigned int &k, vpColVector &w)
{
  const unsigned int nos = numberOfSignal();
  const unsigned int index_0 = 0;
  const unsigned int index_1 = 1;
  const unsigned int index_2 = 2;
  const unsigned int index_3 = 3;
  const unsigned int index_4 = 4;
  const unsigned int index_5 = 5;
  // Homogeneous system A x = 0  ; x is the nullspace of A
  // K0 u^2 + K1 v^2 + 2 K2 u v + 2 K3 u + 2 K4 v + K5 = 0
  // A = (u^2 v^2 2uv 2u 2v 1), x = (K0 K1 K2 K3 K4 K5)^T

  // It would be a bad idea to solve the same system using A x = b where
  // A = (u^2 v^2 2uv 2u 2v), b = (-1), x = (K0 K1 K2 K3 K4)^T since it
  // cannot consider the case where the origin belongs to the ellipse.
  // Another possibility would be to consider K0+K1=1 which is always valid,
  // leading to the system A x = b where
  // A = (u^2-v^2 2uv 2u 2v 1), b = (-v^2), x = (K0 K2 K3 K4 K5)^T
  const unsigned int nbColsA = 6;
  vpMatrix A(nos, nbColsA);
  // Useful to compute the weights in the robust estimation
  vpColVector xp(nos), yp(nos);
  std::list<vpMeSite>::const_iterator end = m_meList.end();

  for (std::list<vpMeSite>::const_iterator it = m_meList.begin(); it != end; ++it) {
    vpMeSite p_me = *it;
    if (p_me.getState() == vpMeSite::NO_SUPPRESSION) {
      // from (i,j) to (u,v) frame + normalization so that (u,v) in [-1;1]
      double u = (p_me.get_jfloat() - um) / um;
      double v = (p_me.get_ifloat() - vm) / vm;
      A[k][index_0] = u * u;
      A[k][index_1] = v * v;
      A[k][index_2] = 2.0 * u * v;
      A[k][index_3] = 2.0 * u;
      A[k][index_4] = 2.0 * v;
      A[k][index_5] = 1.0;
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
      throw(vpMatrixException(vpMatrixException::rankDeficient, "Linear system for computing the ellipse equation ill conditioned"));
    }

    const unsigned int nparam = 6;
    for (unsigned int i = 0; i < nparam; ++i) {
      m_K[i] = KerDA[i][0]; // norm(K) = 1
    }

    // inverse normalization
    m_K[index_0] *= vm / um;
    m_K[index_1] *= um / vm;
    m_K[index_3] = (m_K[index_3] * vm) - (m_K[0] * um) - (m_K[index_2] * vm);
    m_K[index_4] = (m_K[index_4] * um) - (m_K[1] * vm) - (m_K[index_2] * um);
    m_K[index_5] = (m_K[index_5] * um * vm) - (m_K[index_0] * um * um) - (m_K[index_1] * vm * vm)
      - (2.0 * m_K[index_2] * um * vm) - (2.0 * m_K[index_3] * um) - (2.0 * m_K[index_4] * vm);

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
    leastSquareRobustCircle(um, vm, k, w);
  }
  else { // we track an ellipse
    leastSquareRobustEllipse(um, vm, k, w);
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
      meList = m_meList.erase(meList);
      angleList = m_angleList.erase(angleList);
    }
    else {
      if (w[k] < m_thresholdWeight) { // outlier
        meList = m_meList.erase(meList);
        angleList = m_angleList.erase(angleList);
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
      }
      ++k; // k contains good points and outliers (used for w[k])
    }
  }

  if (m_meList.size() != m_angleList.size()) {
    // Should never occur
    throw(vpTrackingException(vpTrackingException::fatalError, "Lists are not coherent in vpMeEllipse::leastSquareRobust(): nb MEs %ld, nb ang %ld",
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
      }
      else {
        meList = m_meList.erase(meList);
        angleList = m_angleList.erase(angleList);
      }
    }
    else { // point not in the interval [alpha1 ; alpha2]
      meList = m_meList.erase(meList);
      angleList = m_angleList.erase(angleList);
    }
  }

  if ((m_meList.size() != numberOfGoodPoints) || (m_angleList.size() != numberOfGoodPoints)) {
    // Should never occur
    throw(vpTrackingException(vpTrackingException::fatalError, "Lists are not coherent at the end of vpMeEllipse::leastSquareRobust(): nb goog MEs %d and %ld, nb ang %ld",
                              numberOfGoodPoints, m_meList.size(), m_angleList.size()));
  }

  // set extremities of the angle list
  m_alphamin = m_angleList.front();
  m_alphamax = m_angleList.back();

  return numberOfGoodPoints;
}

END_VISP_NAMESPACE
