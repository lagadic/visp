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
 * Display a point cloud using PCL library.
 */

/*!
  \file vpUnscentedKalman.cpp
  \brief Unscented Kalman filtering implementation.
*/

#include <visp3/core/vpUnscentedKalman.h>

vpUnscentedKalman::vpUnscentedKalman(const vpMatrix &Q, const vpMatrix &R, vpUKSigmaDrawerAbstract *drawer, const process_function &f, const measurement_function &h)
  : m_Q(Q)
  , m_R(R)
  , m_f(f)
  , m_h(h)
  , m_sigmaDrawer(drawer)
{

}

void vpUnscentedKalman::init(const vpColVector &mu0, const vpMatrix &P0)
{
  m_Xest = mu0;
  m_Pest = P0;
}

void vpUnscentedKalman::filter(const vpColVector &z, const double &dt)
{
  predict(dt);
  update(z);
}

void vpUnscentedKalman::predict(const double &dt)
{
  // Drawing the sigma points
  m_chi = m_sigmaDrawer->drawSigmaPoints(m_Xest, m_Pest);

  // Computation of the attached weights
  vpUKSigmaDrawerAbstract::vpSigmaPointsWeights weights = m_sigmaDrawer->computeWeights();
  m_wm = weights.m_wm;
  m_wc = weights.m_wc;

  // Computation of the prior based on the sigma points
  m_Y = m_f(m_chi, dt);

  // Computation of the mean and covariance of the prior
  vpUnscentedTransformResult transformResults = unscentedTransform(m_Y, m_wm, m_wc, m_Q);
  m_mu = transformResults.m_mu;
  m_P = transformResults.m_P;
}

void vpUnscentedKalman::update(const vpColVector &z)
{
  m_Z = m_h(m_Y);

  // Computation of the mean and covariance of the prior expressed in the measurement space
  vpUnscentedTransformResult transformResults = unscentedTransform(m_Z, m_wm, m_wc, m_R);
  m_muz = transformResults.m_mu;
  m_Pz = transformResults.m_P;

  // Computation of the Kalman gain
  vpMatrix Pxz = m_wc[0] * (m_Y[0] - m_mu) * (m_Z[0] - m_muz).transpose();
  unsigned int nbPts = m_wc.size();
  for (unsigned int i = 1; i < nbPts; ++i) {
    Pxz += m_wc[i] * (m_Y[i] - m_mu) * (m_Z[i] - m_muz).transpose();
  }
  m_K = Pxz * m_Pz.inverseByCholesky();

  // Updating the estimate
  m_Xest = m_mu + m_K * (z - m_muz);
  m_Pest = m_P - m_K * m_Pz * m_K.transpose();
}

vpUnscentedKalman::vpUnscentedTransformResult vpUnscentedKalman::unscentedTransform(const std::vector<vpColVector> &sigmaPoints, const vpColVector &wm, const vpColVector &wc, const vpMatrix &cov)
{
  vpUnscentedKalman::vpUnscentedTransformResult result;

  // Computation of the mean
  unsigned int nbSigmaPoints = wm.size();
  result.m_mu = wm[0] * sigmaPoints[0];
  for (unsigned int i = 1; i < nbSigmaPoints; ++i) {
    result.m_mu += wm[i] * sigmaPoints[i];
  }

  // Computation of the covariance
  result.m_P = cov;
  for (unsigned int i = 0; i < nbSigmaPoints; ++i) {
    vpColVector e = sigmaPoints[i] - result.m_mu;
    result.m_P += wc[i] * e*e.transpose();
  }
  return result;
}
