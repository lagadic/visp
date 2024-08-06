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
  \file vpUKSigmaDrawerSimplex.cpp
  \brief Sigma points drawer following the simplex method from Julier.
*/

#include <visp3/core/vpUKSigmaDrawerSimplex.h>

#if (VISP_CXX_STANDARD >= VISP_CXX_STANDARD_11)
BEGIN_VISP_NAMESPACE
vpUKSigmaDrawerSimplex::vpUKSigmaDrawerSimplex(int n, double w0, const vpAddSubFunction &addFunc)
  : vpUKSigmaDrawerAbstract(n),
  m_w0(w0), m_addFunc(addFunc),
  m_chis(n + 1, vpColVector(n)),
  m_ws(n + 1)
{
  // m_ws[0] = m_w0;
  m_ws[0] = (1.0 - m_w0) / std::pow(2.0, m_n);
  m_ws[1] = m_ws[0];
  for (unsigned int i = 2; i < m_n + 1; ++i) {
    m_ws[i] = std::pow(2.0, i - 1) * m_ws[0];
  }

}

std::vector<vpColVector> vpUKSigmaDrawerSimplex::drawSigmaPoints(const vpColVector &mean, const vpMatrix &covariance)
{
  const unsigned int nbSigmaPoints = m_n + 1;
  std::vector<vpColVector> sigmaPoints(nbSigmaPoints);
  vpMatrix chi(m_n, nbSigmaPoints, 0.0);
  chi[0][0] = 0.0;
  chi[0][1] = (-1.0) / sqrt(2.0 * m_ws[0]);
  chi[0][2] = (1.0) / sqrt(2.0 * m_ws[0]);
  for (unsigned int j = 1; j < m_n; ++j) {
    chi[j][0] = 0.0;
    for (unsigned int i = 1; i < j + 1; ++i) {
      chi[j][i] = (-1.0) / sqrt(2.0 * m_ws[j - 1]);
    }
    chi[j][j + 1] = 1.0 / sqrt(2.0 * m_ws[j - 1]);
  }

  const vpMatrix cholesky = covariance.cholesky();
  for (unsigned int i = 0; i < nbSigmaPoints; ++i) {
    std::cout << "Cholesky = " << cholesky <<std::endl;
    std::cout << "chi = " << chi <<std::endl;

    const vpColVector offset = cholesky * chi.getCol(i);

    std::cout << "offset(" << i << ") = " << offset.t() << std::endl;
    sigmaPoints[i] = m_addFunc(mean, offset);
  }

  return sigmaPoints;
}

vpUKSigmaDrawerSimplex::vpSigmaPointsWeights vpUKSigmaDrawerSimplex::computeWeights()
{
  vpSigmaPointsWeights weights;
  weights.m_wm = m_ws;
  weights.m_wc = m_ws;
  return weights;
}
END_VISP_NAMESPACE
#else
void vpUKSigmaDrawerSimplex_dummy()
{

}
#endif
