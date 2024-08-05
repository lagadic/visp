/****************************************************************************
 *
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
*****************************************************************************/

#include "vpTutoMeanSquareFitting.h"

namespace tutorial
{
#ifdef ENABLE_VISP_NAMESPACE
using VISP_NAMESPACE_NAME;
#endif

vpTutoMeanSquareFitting::vpTutoMeanSquareFitting()
  : m_model()
  , m_isFitted(false)
{ }

void vpTutoMeanSquareFitting::fit(const std::vector<vpImagePoint> &pts)
{
  unsigned int nbPts = pts.size();
  vpMatrix A(nbPts, 3, 1.); // The matrix that contains the u^2, u and 1s
  vpMatrix X(3, 1); // The matrix we want to estimate, that contains the a, b and c coefficients.
  vpMatrix b(nbPts, 1); // The matrix that contains the v values

  // Fill the matrices that form the system we want to solve
  for (unsigned int i = 0; i < nbPts; ++i) {
    float u = pts[i].get_u();
    float v = pts[i].get_v();
    A[i][0] = u *u;
    A[i][1] = u;
    A[i][2] = 1.f;
    b[i][0] = v;
  }

  // Compute the parabola coefficients using the least-mean-square method.
  X = A.pseudoInverse() * b;
  m_model = vpTutoParabolaModel(X[0][0], X[1][0], X[2][0]);
  m_isFitted = true;
}

float vpTutoMeanSquareFitting::evaluate(const std::vector<vpImagePoint> &pts)
{
  if (!m_isFitted) {
    throw(vpException(vpException::notInitialized, "fit() has not been called."));
  }
  unsigned int nbPts = pts.size();

  // Compute the mean absolute error
  float meanError = 0.f;
  for (unsigned int i = 0; i < nbPts; ++i) {
    float squareError = evaluate(pts[i]);
    meanError += squareError;
  }
  meanError /= static_cast<float>(nbPts);
  return meanError;
}

float vpTutoMeanSquareFitting::evaluateRobust(const std::vector<vpImagePoint> &pts)
{
  if (!m_isFitted) {
    throw(vpException(vpException::notInitialized, "fit() has not been called."));
  }
  unsigned int nbPts = pts.size();
  vpColVector residuals(nbPts);
  vpColVector weights(nbPts, 1.);
  // Compute the residuals
  for (unsigned int i = 0; i < nbPts; ++i) {
    float squareError = evaluate(pts[i]);
    residuals[i] = squareError;
  }
  vpRobust robust;
  robust.MEstimator(vpRobust::TUKEY, residuals, weights);
  float sumWeights = weights.sum();
  float numerator = (weights.hadamard(residuals)).sum();
  float meanError = numerator / sumWeights;
  return meanError;
}

float vpTutoMeanSquareFitting::evaluate(const vpImagePoint &pt)
{
  if (!m_isFitted) {
    throw(vpException(vpException::notInitialized, "fit() has not been called."));
  }
  float u = pt.get_u();
  float v = pt.get_v();
  float v_model = model(u);
  float error = v - v_model;
  float squareError = error * error;
  return squareError;
}

float vpTutoMeanSquareFitting::model(const float &u)
{
  if (!m_isFitted) {
    throw(vpException(vpException::notInitialized, "fit() has not been called."));
  }
  float v = m_model.eval(u);
  return v;
}
}