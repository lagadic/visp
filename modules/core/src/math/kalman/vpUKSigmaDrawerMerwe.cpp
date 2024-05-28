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
 * Kalman filtering.
 *
*****************************************************************************/

/*!
  \file vpUKSigmaDrawerMerwe.cpp
  \brief Sigma points drawer following the E. A. Wan and R. van der Merwe's method.
*/

#include <visp3/core/vpUKSigmaDrawerMerwe.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
BEGIN_VISP_NAMESPACE
vpUKSigmaDrawerMerwe::vpUKSigmaDrawerMerwe(const int &n, const double &alpha, const double &beta, const double &kappa,
                       const vpAddSubFunction &resFunc, const vpAddSubFunction &addFunc)
  : vpUKSigmaDrawerAbstract(n)
  , m_alpha(alpha)
  , m_beta(beta)
  , m_kappa(kappa)
  , m_resFunc(resFunc)
  , m_addFunc(addFunc)
{
  computeLambda();
}

std::vector<vpColVector> vpUKSigmaDrawerMerwe::drawSigmaPoints(const vpColVector &mean, const vpMatrix &covariance)
{
  const unsigned int nbSigmaPoints = 2 * m_n + 1;
  std::vector<vpColVector> sigmaPoints(nbSigmaPoints);
  sigmaPoints[0] = mean;
  vpMatrix scaledCov = (static_cast<double>(m_n) + m_lambda) * covariance;
  vpMatrix cholesky = scaledCov.cholesky();
  for (unsigned int i = 0; i < m_n; ++i) {
    sigmaPoints[i + 1] = m_addFunc(mean, cholesky.getRow(i).transpose());
    sigmaPoints[i + m_n + 1] = m_resFunc(mean, cholesky.getRow(i).transpose());
  }
  return sigmaPoints;
}

vpUKSigmaDrawerMerwe::vpSigmaPointsWeights vpUKSigmaDrawerMerwe::computeWeights()
{
  const unsigned int nbSigmaPoints = 2 * m_n + 1;
  vpSigmaPointsWeights weights;
  weights.m_wm.resize(nbSigmaPoints);
  weights.m_wc.resize(nbSigmaPoints);

  weights.m_wm[0] = m_lambda / (static_cast<double>(m_n) + m_lambda);
  weights.m_wc[0] = (m_lambda / (static_cast<double>(m_n) + m_lambda)) + 1.0 - m_alpha * m_alpha + m_beta;

  double cstWeight = 1. / (2. * (static_cast<double>(m_n) + m_lambda));
  for (unsigned int i = 1; i < nbSigmaPoints; ++i) {
    weights.m_wm[i] = cstWeight;
    weights.m_wc[i] = cstWeight;
  }
  return weights;
}
END_VISP_NAMESPACE
#else
void vpUKSigmaDrawerMerwe_dummy()
{

}
#endif
